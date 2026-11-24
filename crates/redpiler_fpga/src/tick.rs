//! Runs a whole redstone tick in one clock.
//!
//! The `nodes` module runs one tick priority per clock. Yosys turns each of its flip-flops into
//! an input `<register>_Q` and an output `<register>_D`, which leaves `step`: what one clock does
//! to the registers. It then makes one copy of `step` per priority and one for the clocks between
//! two ticks, in which the host drives the nodes, each with what its stage fixes folded in. The
//! `tick` module written here holds the registers and chains the copies of the priorities.

use crate::process;
use crate::progress::Progress;
use crate::rtl::Sources;
use anyhow::{ensure, Context};
use mchprs_redstone::tick_queue::TickPriority;
use mchprs_simulation::backend::CompileError;
use serde::Deserialize;
use serde_json::Value;
use std::collections::{BTreeMap, HashMap, HashSet};
use std::fs;
use std::path::Path;
use std::process::Command;
use std::time::Duration;

const SCRIPT: &str = "steps.ys";
const NETLIST: &str = "step.json";
const STEPS: &str = "steps.v";
const TICK: &str = "tick.sv";
const REGISTER_PREFIX: &str = "register_";
const TIMEOUT: Duration = Duration::from_secs(30 * 60);

#[derive(Debug, Deserialize)]
struct Netlist {
    modules: HashMap<String, Step>,
}

#[derive(Debug, Deserialize)]
struct Step {
    ports: BTreeMap<String, Port>,
    netnames: HashMap<String, Net>,
}

#[derive(Debug, Deserialize)]
struct Port {
    direction: Direction,
    bits: Vec<Value>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
enum Direction {
    Input,
    Output,
}

#[derive(Debug, Deserialize)]
struct Net {
    bits: Vec<Value>,
    attributes: Attributes,
}

#[derive(Debug, Deserialize)]
struct Attributes {
    init: Option<String>,
    /// A register with it gets no flip-flop here: every tick and every clock between two ticks
    /// leaves it at its initial value.
    within_tick: Option<String>,
}

struct Register<'a> {
    name: &'a str,
    width: usize,
    initial: String,
    is_within_tick: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stage {
    BetweenTicks,
    Priority(TickPriority),
}

pub fn compose(sources: Sources, progress: &Progress) -> Result<Sources, CompileError> {
    let directory = &sources.directory;
    let read = [&sources.packages[..], &sources.nodes].concat();
    let copies: String = Stage::all().map(Stage::copy_of_step).collect();
    let script = format!(
        "read_verilog -sv {}\n\
         hierarchy -top nodes\n\
         proc\n\
         flatten\n\
         opt_clean\n\
         rename -enumerate -pattern {REGISTER_PREFIX}% t:$dff\n\
         expose -evert -sep _ t:$dff\n\
         select -assert-none t:$*ff* t:$*latch* t:$mem*\n\
         rename nodes step\n\
         write_json {NETLIST}\n\
         {copies}\
         delete step\n\
         write_verilog -noattr {STEPS}\n",
        read.join(" ")
    );
    fs::write(directory.join(SCRIPT), script)
        .context("could not write the script that composes the tick")
        .map_err(CompileError::Failed)?;
    let mut yosys = Command::new("yosys");
    yosys.current_dir(directory).args(["-q", "-s", SCRIPT]);
    process::run(&mut yosys, &directory.join("steps.log"), TIMEOUT, progress)?;
    write_tick(directory)
        .context("could not compose the tick from what Yosys wrote")
        .map_err(CompileError::Failed)?;
    Ok(Sources {
        nodes: vec![STEPS, TICK],
        ..sources
    })
}

fn write_tick(directory: &Path) -> anyhow::Result<()> {
    let netlist = fs::read_to_string(directory.join(NETLIST))?;
    let netlist: Netlist = serde_json::from_str(&netlist)?;
    let step = netlist.modules.get("step");
    let step = step.context("the netlist has no module step")?;
    Ok(fs::write(directory.join(TICK), tick_module(step)?)?)
}

fn tick_module(step: &Step) -> anyhow::Result<String> {
    let registers = registers(step)?;
    let register_ports: HashSet<String> = registers
        .iter()
        .flat_map(|Register { name, .. }| ["Q", "D", "CLK"].map(|pin| format!("{name}_{pin}")))
        .collect();
    let is_host =
        |port: &str| !["clock", "firing"].contains(&port) && !register_ports.contains(port);
    let host: Vec<(&str, &Port)> = step
        .ports
        .iter()
        .filter(|(port, _)| is_host(port))
        .map(|(name, port)| (name.as_str(), port))
        .collect();

    let host_ports = host.iter().map(|(name, Port { direction, bits })| {
        let direction = match direction {
            Direction::Input => "input",
            Direction::Output => "output",
        };
        format!(",\n    {direction} logic [{}:0] {name}", bits.len() - 1)
    });
    let host_ports: String = host_ports.collect();
    let mut tick =
        format!("module tick (\n    input logic clock,\n    input logic run{host_ports}\n);\n");
    for Register {
        name,
        width,
        initial,
        is_within_tick,
    } in &registers
    {
        let range = format!("[{}:0]", width - 1);
        let after = |stage: Stage| format!("{name}_{}", stage.name());
        let ticked = after(Stage::LAST_PRIORITY);
        let between_ticks = after(Stage::BetweenTicks);
        let next = format!("run ? {ticked} : {between_ticks}");
        let results: Vec<_> = Stage::all().map(after).collect();
        let results = results.join(", ");
        tick += &format!("    logic {range} {results};\n");
        tick += &match is_within_tick {
            true => format!(
                "    wire {range} {name} = {width}'b{initial};\n\
                 `ifndef SYNTHESIS\n    \
                 always_ff @(posedge clock) assert (({next}) == {name});\n\
                 `endif\n"
            ),
            false => format!(
                "    logic {range} {name} = {width}'b{initial};\n    \
                 always_ff @(posedge clock) {name} <= {next};\n"
            ),
        };
    }
    for stage in Stage::all() {
        tick += &stage.instance(&host, &registers);
    }
    Ok(tick + "endmodule\n")
}

impl Stage {
    const LAST_PRIORITY: Stage = Stage::Priority(TickPriority::ALL[TickPriority::ALL.len() - 1]);

    fn all() -> impl Iterator<Item = Stage> {
        let priorities = TickPriority::ALL.into_iter().map(Stage::Priority);
        [Stage::BetweenTicks].into_iter().chain(priorities)
    }

    fn name(self) -> String {
        match self {
            Stage::BetweenTicks => "between_ticks".to_owned(),
            Stage::Priority(priority) => priority.to_string(),
        }
    }

    /// The stage that hands this one the registers; without one it takes them as they are held.
    fn stage_before(self) -> Option<Stage> {
        let Stage::Priority(priority) = self else {
            return None;
        };
        let before = (priority as usize).checked_sub(1)?;
        Some(Stage::Priority(TickPriority::ALL[before]))
    }

    /// Yosys commands for a copy of `step` that keeps only the ports of this stage: no priority
    /// fires between two ticks, and the host drives nothing while one runs.
    fn copy_of_step(self) -> String {
        let stage = self.name();
        let ports = match self {
            Stage::BetweenTicks => format!("delete -port clock firing n:{REGISTER_PREFIX}*_CLK"),
            Stage::Priority(priority) => format!(
                "delete -port x:* n:{REGISTER_PREFIX}*_Q %d n:{REGISTER_PREFIX}*_D %d\n\
                 connect -set firing {}'d{}",
                TickPriority::ALL.len(),
                1 << priority as usize
            ),
        };
        format!(
            "copy step step_{stage}\n\
             cd step_{stage}\n\
             {ports}\n\
             setundef -undriven -zero\n\
             opt -fast\n\
             opt_clean -purge\n\
             cd\n"
        )
    }

    fn instance(self, host: &[(&str, &Port)], registers: &[Register]) -> String {
        let stage = self.name();
        let host = match self {
            Stage::BetweenTicks => host,
            Stage::Priority(_) => &[],
        };
        let host = host.iter().map(|(name, _)| format!(".{name}({name})"));
        let registers = registers.iter().map(|Register { name, .. }| {
            let before = match self.stage_before() {
                Some(stage_before) => format!("{name}_{}", stage_before.name()),
                None => (*name).to_owned(),
            };
            format!(".{name}_Q({before}), .{name}_D({name}_{stage})")
        });
        let ports: Vec<_> = host.chain(registers).collect();
        let ports = ports.join(",\n        ");
        format!("    step_{stage} {stage} (\n        {ports}\n    );\n")
    }
}

fn registers(step: &Step) -> anyhow::Result<Vec<Register<'_>>> {
    let mut initial = HashMap::new();
    let mut within_tick = HashSet::new();
    for Net { bits, attributes } in step.netnames.values() {
        if let Some(digits) = &attributes.init {
            let digits = bits.iter().zip(digits.chars().rev());
            initial.extend(digits.filter(|(_, digit)| ['0', '1'].contains(digit)));
        }
        let is_within_tick = |value: &String| value.contains('1');
        if attributes.within_tick.as_ref().is_some_and(is_within_tick) {
            within_tick.extend(bits);
        }
    }
    let inputs = step.ports.iter();
    let inputs = inputs.filter_map(|(port, Port { direction, bits })| {
        let name = port.strip_suffix("_Q")?;
        let is_register = name.starts_with(REGISTER_PREFIX) && *direction == Direction::Input;
        is_register.then_some((name, bits))
    });
    let registers = inputs.map(|(name, bits)| {
        let next = step.ports.get(&format!("{name}_D"));
        ensure!(
            next.is_some_and(|next| next.bits.len() == bits.len()),
            "{name} has no next value as wide as itself"
        );
        let digits: Option<String> = bits.iter().rev().map(|bit| initial.get(bit)).collect();
        Ok(Register {
            name,
            width: bits.len(),
            initial: digits.with_context(|| format!("{name} has no initial value"))?,
            is_within_tick: bits.iter().all(|bit| within_tick.contains(bit)),
        })
    });
    registers.collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_a_register_that_lasts_between_ticks_is_a_flip_flop() {
        let step = r#"{
            "ports": {
                "clock": {"direction": "input", "bits": [2]},
                "register_0_Q": {"direction": "input", "bits": [3, 4]},
                "register_0_D": {"direction": "output", "bits": [5, 6]},
                "register_1_Q": {"direction": "input", "bits": [7]},
                "register_1_D": {"direction": "output", "bits": [8]}
            },
            "netnames": {
                "node_0.countdown": {"bits": [3, 4], "attributes": {"init": "10"}},
                "node_0.timer.due": {"bits": [7], "attributes": {"init": "0", "within_tick": "1"}}
            }
        }"#;
        let step = serde_json::from_str(step).expect("the netlist parses");
        let tick = tick_module(&step).expect("every register has an initial value");
        assert!(tick.contains("logic [1:0] register_0 = 2'b10;"));
        assert!(tick.contains("always_ff @(posedge clock) register_0 <="));
        assert!(tick.contains("wire [0:0] register_1 = 1'b0;"));
        assert!(!tick.contains("always_ff @(posedge clock) register_1 <="));
    }
}
