use crate::diff::{not_compared, when, Pair, Verdict};
use crate::input::Input as Loaded;
use crate::numbers::counted;
use crate::scenario::{Action, Location, Scenario, Step};
use crate::setup::Setup;
use crate::subject::Subject;
use crate::{Error, Outcome};
use mchprs_blocks::blocks::ComparatorMode;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::engine::Input;
use mchprs_redpiler::graph::{
    Direction, DriveKind, Graph, Link, LinkKind, Node, NodeIdx, NodeKind, NodeState,
};
use mchprs_redpiler::ports::Role;
use mchprs_ril::{Module, Printed};
use std::fs;
use std::num::NonZeroU64;
use std::path::{Path, PathBuf};

const INPUT_CHANCE_PERCENT: u64 = 30;
const SHOWN_CASES: usize = 10;

pub struct Cases {
    pub seed: u64,
    pub count: NonZeroU64,
    pub ticks: u64,
}

impl Cases {
    fn seeds(&self) -> impl Iterator<Item = u64> + use<> {
        let first = self.seed;
        (0..self.count.get()).map(move |index| first.wrapping_add(index))
    }

    fn seed_range(&self) -> String {
        match self.count.get() {
            1 => format!("seed {}", self.seed),
            count => format!(
                "seeds {} to {}",
                self.seed,
                self.seed.wrapping_add(count - 1)
            ),
        }
    }
}

pub fn command(
    input: Option<&Path>,
    setups: [&Setup; 2],
    cases: &Cases,
    out: Option<&Path>,
) -> Outcome {
    if let Some(out) = out {
        fs::create_dir_all(out)
            .map_err(|error| format!("couldn't create {}: {error}", out.display()))?;
    }
    let tally = match input {
        Some(path) => scenarios(path, setups, cases, out)?,
        None => circuits(setups, cases, out)?,
    };
    print!("{}", tally.report(setups, cases, out));
    Ok(tally.differing.is_empty())
}

fn scenarios(
    path: &Path,
    setups: [&Setup; 2],
    cases: &Cases,
    out: Option<&Path>,
) -> Result<Tally, Error> {
    let input = Loaded::read(path, None)?;
    let subjects = [
        Subject::compile(&input, setups[0])?,
        Subject::compile(&input, setups[1])?,
    ];
    let pair = Pair::new(&subjects, setups);
    let inputs = pair.inputs();
    let absolute = std::path::absolute(path)
        .map_err(|error| format!("couldn't resolve {}: {error}", path.display()))?;
    let mut tally = Tally {
        cases: format!(
            "{} on the {} of {} ({})",
            counted(cases.count.get(), "random scenario"),
            counted(inputs.len() as u64, "input"),
            path.display(),
            cases.seed_range()
        ),
        files: "<seed>.toml",
        circuit: subjects[0].is_circuit(),
        unshared: pair.unshared(),
        ..Tally::default()
    };
    for seed in cases.seeds() {
        let scenario = random_scenario(&inputs, &mut Xorshift::new(seed), cases.ticks, seed, path);
        let verdict = pair.play(&scenario).map_err(|error| in_case(seed, error))?;
        if verdict.first_difference_tick.is_some() {
            save(
                out,
                &[(format!("{seed}.toml"), scenario.to_toml(&absolute)?)],
            )?;
        }
        tally.add(seed, verdict);
    }
    Ok(tally)
}

fn circuits(setups: [&Setup; 2], cases: &Cases, out: Option<&Path>) -> Result<Tally, Error> {
    for setup in setups {
        setup.check_for_circuit()?;
    }
    let mut tally = Tally {
        cases: format!(
            "{} ({})",
            counted(cases.count.get(), "random circuit"),
            cases.seed_range()
        ),
        files: "<seed>.toml and <seed>.ril",
        circuit: true,
        ..Tally::default()
    };
    for seed in cases.seeds() {
        let mut rng = Xorshift::new(seed);
        let ril = Printed::new(&random_graph(&mut rng), &format!("seed_{seed}")).to_string();
        let ril_path = PathBuf::from(format!("{seed}.ril"));
        let (verdict, scenario, unshared) =
            play_circuit(&ril, &ril_path, setups, &mut rng, cases.ticks, seed)
                .map_err(|error| in_case(seed, error))?;
        if verdict.first_difference_tick.is_some() {
            let toml = scenario.to_toml(&ril_path)?;
            save(
                out,
                &[(format!("{seed}.toml"), toml), (format!("{seed}.ril"), ril)],
            )?;
        }
        for (total, unshared) in tally.unshared.iter_mut().zip(unshared) {
            *total += unshared;
        }
        tally.add(seed, verdict);
    }
    Ok(tally)
}

fn play_circuit(
    ril: &str,
    ril_path: &Path,
    setups: [&Setup; 2],
    rng: &mut Xorshift,
    ticks: u64,
    seed: u64,
) -> Result<(Verdict, Scenario, [u64; 2]), Error> {
    let module =
        Module::parse(ril).map_err(|error| format!("the random circuit isn't valid: {error}"))?;
    let input = Loaded::Circuit(module.circuits()[0].clone());
    let subjects = [
        Subject::compile(&input, setups[0])?,
        Subject::compile(&input, setups[1])?,
    ];
    let pair = Pair::new(&subjects, setups);
    let scenario = random_scenario(&pair.inputs(), rng, ticks, seed, ril_path);
    let verdict = pair.play(&scenario)?;
    Ok((verdict, scenario, pair.unshared()))
}

fn in_case(seed: u64, error: Error) -> Error {
    format!("seed {seed}: {error}").into()
}

fn save(out: Option<&Path>, files: &[(String, String)]) -> Result<(), Error> {
    let Some(out) = out else {
        return Ok(());
    };
    for (name, text) in files {
        let path = out.join(name);
        fs::write(&path, text)
            .map_err(|error| format!("couldn't write {}: {error}", path.display()))?;
    }
    Ok(())
}

#[derive(Default)]
struct Tally {
    cases: String,
    files: &'static str,
    circuit: bool,
    unshared: [u64; 2],
    differing: Vec<(u64, u64)>,
    first_report: Option<String>,
}

impl Tally {
    fn add(&mut self, seed: u64, verdict: Verdict) {
        if let Some(tick) = verdict.first_difference_tick {
            self.differing.push((seed, tick));
            self.first_report.get_or_insert(verdict.report);
        }
    }

    fn report(&self, setups: [&Setup; 2], cases: &Cases, out: Option<&Path>) -> String {
        let [first, second] = setups;
        let mut report = format!(
            "fuzz: {}, {} each, using an input before {INPUT_CHANCE_PERCENT}% of ticks\n",
            self.cases,
            counted(cases.ticks, "tick")
        );
        if self.differing.is_empty() {
            report.push_str(&format!("{first} and {second} agree on every case\n"));
        } else {
            let named: Vec<_> = self
                .differing
                .iter()
                .take(SHOWN_CASES)
                .map(|&(seed, tick)| format!("seed {seed} {}", when(tick)))
                .collect();
            report.push_str(&format!(
                "{first} and {second} differ on {}: {}",
                counted(self.differing.len() as u64, "case"),
                named.join(", ")
            ));
            if self.differing.len() > SHOWN_CASES {
                let more = (self.differing.len() - SHOWN_CASES) as u64;
                report.push_str(&format!(" and {} more", counted(more, "case")));
            }
            report.push('\n');
        }
        report.push_str(&not_compared(setups, self.unshared, self.circuit));
        if let Some(out) = out
            && !self.differing.is_empty()
        {
            report.push_str(&format!(
                "wrote each differing case to {} as {}\n",
                out.display(),
                self.files
            ));
        }
        if let (Some(&(seed, _)), Some(first_report)) = (self.differing.first(), &self.first_report)
        {
            report.push_str(&format!("\nseed {seed}: {first_report}"));
        }
        report
    }
}

fn random_scenario(
    inputs: &[(&Location, DriveKind)],
    rng: &mut Xorshift,
    ticks: u64,
    seed: u64,
    input: &Path,
) -> Scenario {
    let mut steps: Vec<Step> = Vec::new();
    for _ in 0..ticks {
        let action = (!inputs.is_empty() && rng.chance(INPUT_CHANCE_PERCENT)).then(|| {
            let (location, kind) = inputs[rng.below(inputs.len() as u64) as usize];
            let input = match kind {
                DriveKind::OnOff => Input::Toggle,
                DriveKind::Press => Input::Press,
                DriveKind::Strength => Input::Set(random_strength(rng)),
            };
            Action {
                location: location.clone(),
                input,
            }
        });
        match steps.last_mut() {
            Some(step) if action.is_none() => step.ticks += 1,
            _ => steps.push(Step { action, ticks: 1 }),
        }
    }
    Scenario {
        path: PathBuf::from(format!("{seed}.toml")),
        input: input.to_owned(),
        circuit: None,
        settle: None,
        steps,
    }
}

fn random_graph(rng: &mut Xorshift) -> Graph {
    let size = 4 + rng.below(61);
    let mut graph = Graph::default();
    let mut sources = Vec::new();
    let mut targets = Vec::new();
    for _ in 0..size {
        let (kind, port) = random_kind(rng);
        let power = match kind {
            NodeKind::Comparator { .. } | NodeKind::Constant | NodeKind::Wire => {
                random_strength(rng)
            }
            _ => rng.chance(50).into(),
        };
        let state = NodeState {
            power,
            repeater_locked: matches!(kind, NodeKind::Repeater { .. }) && rng.chance(20),
        };
        let node = graph.add_node(Node {
            kind,
            blocks: Default::default(),
            names: Vec::new(),
            state,
            pending_tick: None,
            port,
            observed: true,
        });
        if graph[node].kind.is_link_source() {
            sources.push(node);
        }
        if graph[node].kind.is_link_target() && !graph[node].is_input() {
            targets.push(node);
        }
    }
    if sources.is_empty() || targets.is_empty() {
        return graph;
    }
    for &target in &targets {
        let reads_side = matches!(
            graph[target].kind,
            NodeKind::Repeater { .. } | NodeKind::Comparator { .. }
        );
        for _ in 0..1 + rng.below(3) {
            let source = sources[rng.below(sources.len() as u64) as usize];
            let kind = if reads_side && rng.chance(30) {
                LinkKind::Side
            } else {
                LinkKind::Default
            };
            let weight = if rng.chance(50) {
                0
            } else {
                rng.below(6) as u8
            };
            graph.add_edge(source, target, Link::new(kind, weight));
        }
        if rng.chance(15) {
            let source = sources[rng.below(sources.len() as u64) as usize];
            graph.add_edge(source, target, Link::default(rng.below(6) as u8));
        }
    }
    for _ in 0..size / 3 {
        let original = targets[rng.below(targets.len() as u64) as usize];
        let twin = graph.add_node(graph[original].clone());
        let inputs: Vec<(NodeIdx, Link)> = graph
            .edges(original, Direction::Incoming)
            .map(|edge| (edge.source(), *edge.weight()))
            .collect();
        for (source, link) in inputs {
            graph.add_edge(source, twin, link);
        }
        if graph[twin].kind.is_link_source() {
            for _ in 0..1 + rng.below(3) {
                let target = targets[rng.below(targets.len() as u64) as usize];
                graph.add_edge(twin, target, Link::default(rng.below(4) as u8));
            }
        }
    }
    graph
}

fn random_kind(rng: &mut Xorshift) -> (NodeKind, Option<Role>) {
    match rng.below(100) {
        0..7 => (NodeKind::Lever, Some(Role::Input)),
        7..10 => (NodeKind::Button { press_ticks: 10 }, Some(Role::Input)),
        10..37 => (
            NodeKind::Repeater {
                delay: 1 + rng.below(4) as u8,
                prioritized: rng.chance(20),
            },
            None,
        ),
        37..55 => (NodeKind::Torch, None),
        55..78 => (
            NodeKind::Comparator {
                mode: if rng.chance(50) {
                    ComparatorMode::Compare
                } else {
                    ComparatorMode::Subtract
                },
                far_input: rng.chance(15).then(|| random_strength(rng)),
                prioritized: rng.chance(20),
            },
            None,
        ),
        78..83 => (NodeKind::Constant, None),
        83..91 => (NodeKind::Lamp, Some(Role::Output)),
        91..95 => (NodeKind::Trapdoor, Some(Role::Output)),
        _ => (NodeKind::Wire, Some(Role::Output)),
    }
}

fn random_strength(rng: &mut Xorshift) -> SignalStrength {
    SignalStrength::try_from(rng.below(16) as u8).expect("below 16 is a signal strength")
}

struct Xorshift(u64);

impl Xorshift {
    fn new(seed: u64) -> Xorshift {
        Xorshift(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }

    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    fn below(&mut self, bound: u64) -> u64 {
        self.next() % bound
    }

    fn chance(&mut self, percent: u64) -> bool {
        self.below(100) < percent
    }
}
