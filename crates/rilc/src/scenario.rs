use crate::input::Input as Loaded;
use crate::numbers::counted;
use crate::Error;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockPos;
use mchprs_redpiler::engine::Input;
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::fmt;
use std::fs;
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum Location {
    Position(BlockPos),
    Name(String),
}

impl Location {
    fn sort_key(&self) -> (i32, i32, i32, &str) {
        match self {
            Location::Position(pos) => (pos.x, pos.y, pos.z, ""),
            Location::Name(name) => (0, 0, 0, name),
        }
    }
}

impl Ord for Location {
    fn cmp(&self, other: &Location) -> Ordering {
        self.sort_key().cmp(&other.sort_key())
    }
}

impl PartialOrd for Location {
    fn partial_cmp(&self, other: &Location) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl fmt::Display for Location {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Location::Position(pos) => write!(f, "{pos}"),
            Location::Name(name) => write!(f, "%{name}"),
        }
    }
}

#[derive(Serialize, Deserialize)]
#[serde(untagged)]
enum Target {
    Position([i32; 3]),
    Name(String),
}

impl From<Target> for Location {
    fn from(target: Target) -> Location {
        match target {
            Target::Position([x, y, z]) => Location::Position(BlockPos::new(x, y, z)),
            Target::Name(name) => Location::Name(name),
        }
    }
}

impl From<&Location> for Target {
    fn from(location: &Location) -> Target {
        match location {
            Location::Position(pos) => Target::Position([pos.x, pos.y, pos.z]),
            Location::Name(name) => Target::Name(name.clone()),
        }
    }
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ScenarioFile {
    input: PathBuf,
    #[serde(skip_serializing_if = "Option::is_none")]
    circuit: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    settle: Option<u64>,
    #[serde(default)]
    step: Vec<StepFile>,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct StepFile {
    #[serde(skip_serializing_if = "Option::is_none")]
    toggle: Option<Target>,
    #[serde(skip_serializing_if = "Option::is_none")]
    press: Option<Target>,
    #[serde(skip_serializing_if = "Option::is_none")]
    set: Option<Target>,
    #[serde(skip_serializing_if = "Option::is_none")]
    power: Option<u32>,
    run: u64,
}

pub struct Scenario {
    pub path: PathBuf,
    pub input: PathBuf,
    pub circuit: Option<String>,
    pub settle: Option<u64>,
    pub steps: Vec<Step>,
}

pub struct Step {
    pub action: Option<Action>,
    pub ticks: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Action {
    pub location: Location,
    pub input: Input,
}

impl fmt::Display for Action {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.input {
            Input::Toggle => write!(f, "toggle {}", self.location),
            Input::Press => write!(f, "press {}", self.location),
            Input::Set(power) => write!(f, "set {} to {power}", self.location),
        }
    }
}

impl Scenario {
    pub fn read(path: &Path) -> Result<Scenario, Error> {
        let text = fs::read_to_string(path)
            .map_err(|error| format!("couldn't read {}: {error}", path.display()))?;
        let file: ScenarioFile =
            toml::from_str(&text).map_err(|error| format!("{}: {error}", path.display()))?;
        let mut scenario = Scenario {
            path: path.to_owned(),
            input: path.parent().unwrap_or(Path::new("")).join(file.input),
            circuit: file.circuit,
            settle: file.settle,
            steps: Vec::with_capacity(file.step.len()),
        };
        for (index, step) in file.step.into_iter().enumerate() {
            let step = step
                .into_step()
                .map_err(|problem| scenario.step_error(index, problem))?;
            scenario.steps.push(step);
        }
        Ok(scenario)
    }

    pub fn to_toml(&self, input: &Path) -> Result<String, Error> {
        let file = ScenarioFile {
            input: input.to_owned(),
            circuit: self.circuit.clone(),
            settle: self.settle,
            step: self.steps.iter().map(StepFile::from).collect(),
        };
        Ok(toml::to_string(&file)?)
    }

    pub fn step_error(&self, index: usize, problem: impl fmt::Display) -> Error {
        format!("{}: step {}: {problem}", self.path.display(), index + 1).into()
    }

    pub fn ticks(&self) -> u64 {
        self.steps.iter().map(|step| step.ticks).sum()
    }

    pub fn read_input(&self) -> Result<Loaded, Error> {
        let mut input = Loaded::read(&self.input, self.circuit.as_deref())?;
        let world = matches!(input, Loaded::World(_));
        for (index, step) in self.steps.iter().enumerate() {
            let Some(action) = &step.action else {
                continue;
            };
            match (&action.location, world) {
                (Location::Name(_), true) => {
                    let problem = "a world's inputs are positions like [x, y, z]";
                    return Err(self.step_error(index, problem));
                }
                (Location::Position(_), false) => {
                    let problem = "a RIL circuit's inputs are names like \"lever\"";
                    return Err(self.step_error(index, problem));
                }
                _ => {}
            }
        }
        if let Some(limit) = self.settle {
            let Loaded::World(world) = &mut input else {
                let problem = format!("{}: settle only applies to worlds", self.path.display());
                return Err(problem.into());
            };
            let region = world.blocks().region();
            if mchprs_redstone::settle(world, region, limit).is_none() {
                let ticks = counted(limit, "tick");
                return Err(format!("the world hasn't settled within {ticks}").into());
            }
        }
        Ok(input)
    }
}

impl From<&Step> for StepFile {
    fn from(step: &Step) -> StepFile {
        let mut file = StepFile {
            toggle: None,
            press: None,
            set: None,
            power: None,
            run: step.ticks,
        };
        if let Some(action) = &step.action {
            let target = Some(Target::from(&action.location));
            match action.input {
                Input::Toggle => file.toggle = target,
                Input::Press => file.press = target,
                Input::Set(power) => {
                    file.set = target;
                    file.power = Some(power.get().into());
                }
            }
        }
        file
    }
}

impl StepFile {
    fn into_step(self) -> Result<Step, String> {
        let set = match (self.set, self.power) {
            (Some(target), Some(power)) => {
                let power = SignalStrength::try_from(power).map_err(|error| error.to_string())?;
                Some((target, Input::Set(power)))
            }
            (Some(_), None) => return Err("set needs a power".to_owned()),
            (None, Some(_)) => return Err("power only goes with set".to_owned()),
            (None, None) => None,
        };
        let mut actions = [
            self.toggle.map(|target| (target, Input::Toggle)),
            self.press.map(|target| (target, Input::Press)),
            set,
        ]
        .into_iter()
        .flatten();
        let action = actions.next();
        if actions.next().is_some() {
            return Err("a step holds at most one of toggle, press and set".to_owned());
        }
        Ok(Step {
            action: action.map(|(target, input)| Action {
                location: target.into(),
                input,
            }),
            ticks: self.run,
        })
    }
}
