mod analysis;
mod opt;

use crate::compile::CompileOptions;
use crate::graph::Graph;
use crate::ports::Observed;
use rustc_hash::{FxHashMap, FxHashSet};
use std::any::{Any, TypeId};
use std::collections::HashMap;
use std::ops::ControlFlow;
use tracing::trace;

use analysis::*;
use opt::*;

pub fn build_pass_pipeline<'p>(
    registry: &'p PassRegistry,
    options: &CompileOptions,
) -> PassPipeline<'p> {
    let mut builder = PassPipelineBuilder::new(registry);

    builder.add_pass::<clamp_weights::ClampWeights>();

    if options.optimize {
        builder.add_pass::<dedup_links::DedupLinks>();
        builder.add_pass::<constant_fold::ConstantFold>();
        builder.add_pass::<unreachable_output::UnreachableOutput>();
        if options.reorder {
            builder.add_pass::<coalesce::Coalesce>();
        }
        if options.observed == Observed::Ports {
            builder.add_pass::<prune_orphans::PruneOrphans>();
        }
    }

    builder.build()
}

pub struct PassRegistry {
    passes: FxHashMap<TypeId, Box<dyn Pass>>,
    driver_name_map: FxHashMap<&'static str, TypeId>,
}

impl Default for PassRegistry {
    fn default() -> Self {
        let mut registry = Self::new();

        registry.register_pass(ss_set_analysis::SSSetAnalysis);

        registry.register_pass(clamp_weights::ClampWeights);
        registry.register_pass(dedup_links::DedupLinks);
        registry.register_pass(constant_fold::ConstantFold);
        registry.register_pass(unreachable_output::UnreachableOutput);
        registry.register_pass(coalesce::Coalesce);
        registry.register_pass(prune_orphans::PruneOrphans);

        registry
    }
}

impl PassRegistry {
    pub fn new() -> Self {
        Self {
            passes: FxHashMap::default(),
            driver_name_map: FxHashMap::default(),
        }
    }

    pub fn register_pass<P: Pass>(&mut self, pass: P) {
        self.driver_name_map
            .insert(pass.driver_key(), pass.type_id());
        if let Some(pass) = self.passes.insert(pass.type_id(), Box::new(pass)) {
            panic!("registered duplicate pass: {}", pass.debug_name())
        }
    }

    pub fn get_pass<P: Pass>(&self) -> &dyn Pass {
        self.get_pass_from_id(TypeId::of::<P>())
    }

    pub fn get_pass_from_id(&self, id: TypeId) -> &dyn Pass {
        &*self.passes[&id]
    }

    pub fn get_pass_from_driver_key(&self, key: &str) -> Option<&dyn Pass> {
        self.driver_name_map
            .get(key)
            .map(|type_id| self.get_pass_from_id(*type_id))
    }
}

pub trait AnalysisInfo: Any {}

#[derive(Default)]
pub struct AnalysisInfos {
    analysis_infos: HashMap<TypeId, Box<dyn AnalysisInfo>>,
}

impl AnalysisInfos {
    pub fn insert_analysis<A: AnalysisInfo>(&mut self, analysis: A) {
        self.analysis_infos
            .insert(analysis.type_id(), Box::new(analysis));
    }

    pub fn get_analysis<A: AnalysisInfo>(&self) -> Option<&A> {
        let type_id = TypeId::of::<A>();
        self.analysis_infos
            .get(&type_id)
            .and_then(|ai| (ai.as_ref() as &dyn Any).downcast_ref())
    }
}

pub struct PassPipelineBuilder<'p> {
    registry: &'p PassRegistry,
    passes: Vec<&'p dyn Pass>,
    available_analysis: FxHashSet<TypeId>,
}

impl<'p> PassPipelineBuilder<'p> {
    pub fn new(registry: &'p PassRegistry) -> Self {
        Self {
            registry,
            passes: Vec::new(),
            available_analysis: FxHashSet::default(),
        }
    }

    /// Returns true if the pass was found
    pub fn add_pass_by_driver_key(&mut self, driver_key: &str) -> bool {
        let Some(pass) = self.registry.get_pass_from_driver_key(driver_key) else {
            return false;
        };
        self.add_pass_by_instance(pass);
        true
    }

    pub fn add_pass<P: Pass>(&mut self) {
        let pass = self.registry.get_pass::<P>();
        self.add_pass_by_instance(pass);
    }

    pub fn add_pass_by_instance(&mut self, pass: &'p dyn Pass) {
        let mut au = AnalysisUsage::default();

        pass.analysis_usage(&mut au);
        for type_id in &au.required {
            if !self.available_analysis.contains(type_id) {
                let analysis_pass = self.registry.get_pass_from_id(*type_id);
                self.add_pass_by_instance(analysis_pass);
            }
        }

        self.passes.push(pass);

        if !au.preserves_all {
            self.available_analysis
                .retain(|type_id| au.preserved.contains(type_id));
        }

        self.available_analysis.insert(pass.pass_type_id());
    }

    pub fn build(self) -> PassPipeline<'p> {
        PassPipeline {
            passes: self.passes,
        }
    }
}

pub struct PassPipeline<'p> {
    passes: Vec<&'p dyn Pass>,
}

impl PassPipeline<'_> {
    pub fn len(&self) -> usize {
        self.passes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.passes.is_empty()
    }

    pub fn passes(&self) -> &[&dyn Pass] {
        &self.passes
    }

    /// Runs every pass in order, then calls `after_each`, which can stop the pipeline.
    pub fn run_passes(
        &self,
        graph: &mut Graph,
        after_each: &mut dyn FnMut(&dyn Pass, &Graph) -> ControlFlow<()>,
    ) -> ControlFlow<()> {
        let mut analysis_infos = AnalysisInfos::default();
        for &pass in &self.passes {
            trace!("Running pass: {}", pass.debug_name());
            pass.run_pass(graph, &mut analysis_infos);
            trace!(
                "{} nodes and {} links remain",
                graph.node_count(),
                graph.edge_count()
            );
            after_each(pass, graph)?;
        }
        ControlFlow::Continue(())
    }
}

#[derive(Default)]
pub struct AnalysisUsage {
    preserves_all: bool,
    required: Vec<TypeId>,
    preserved: Vec<TypeId>,
}

impl AnalysisUsage {
    pub fn set_preserves_all(&mut self) {
        self.preserves_all = true;
    }

    pub fn set_required<P: Pass>(&mut self) {
        self.required.push(TypeId::of::<P>());
    }

    pub fn set_preserved<P: Pass>(&mut self) {
        self.preserved.push(TypeId::of::<P>());
    }
}

pub trait Pass: 'static {
    fn run_pass(&self, graph: &mut Graph, analysis_infos: &mut AnalysisInfos);

    /// This name should only be use for debugging purposes,
    /// it is not a valid identifier of the pass.
    fn debug_name(&self) -> &'static str {
        std::any::type_name::<Self>()
    }

    fn analysis_usage(&self, _au: &mut AnalysisUsage) {}

    fn status_message(&self) -> &'static str;

    /// A kebab-case identifier for this pass. Used by rilc.
    fn driver_key(&self) -> &'static str;

    /// Gets the typeId of the underlying pass,
    /// this is different from `.type_id`,
    /// as that results in getting the TypeId of the dynamic trait object
    fn pass_type_id(&self) -> TypeId {
        TypeId::of::<Self>()
    }
}
