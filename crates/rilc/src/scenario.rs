use crate::input::{settle, Loaded};
use crate::Error;
use anyhow::{anyhow, Context};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockPos;
use mchprs_redstone::input::Input;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::fs;
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
#[serde(untagged)]
pub enum Target {
    Position([i32; 3]),
    Name(String),
}

impl From<BlockPos> for Target {
    fn from(pos: BlockPos) -> Target {
        Target::Position([pos.x, pos.y, pos.z])
    }
}

impl fmt::Display for Target {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Target::Position([x, y, z]) => write!(f, "{}", BlockPos::new(*x, *y, *z)),
            Target::Name(name) => write!(f, "%{name}"),
        }
    }
}

pub struct Scenario {
    pub path: PathBuf,
    pub input: PathBuf,
    pub circuit: Option<String>,
    pub settle: Option<u64>,
    pub steps: Vec<Step>,
}

impl Scenario {
    pub fn read(path: &Path) -> Result<Scenario, Error> {
        let text = fs::read_to_string(path)
            .with_context(|| format!("couldn't read {}", path.display()))?;
        let file: ScenarioFile =
            toml::from_str(&text).with_context(|| format!("{}", path.display()))?;
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

    pub fn read_input(&self) -> Result<Loaded, Error> {
        let mut input = Loaded::read(&self.input, self.circuit.as_deref())?;
        if let Loaded::World(_) = input {
            for (index, step) in self.steps.iter().enumerate() {
                if let Some(Action {
                    at: Target::Name(_),
                    ..
                }) = step.action
                {
                    let problem = "a world's inputs are positions like [x, y, z]";
                    return Err(self.step_error(index, problem));
                }
            }
        }
        if let Some(limit) = self.settle {
            let Loaded::World(world) = &mut input else {
                let problem = format!("{}: settle only applies to worlds", self.path.display());
                return Err(anyhow!(problem));
            };
            settle(world, limit)?;
        }
        Ok(input)
    }

    pub fn step_error(&self, index: usize, problem: impl fmt::Display) -> Error {
        anyhow!("{}: step {}: {problem}", self.path.display(), index + 1)
    }

    pub fn ticks(&self) -> u64 {
        self.steps.iter().map(|step| step.ticks).sum()
    }
}

pub struct Step {
    pub action: Option<Action>,
    pub ticks: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Action {
    pub at: Target,
    pub input: Input,
}

impl fmt::Display for Action {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.input {
            Input::Toggle => write!(f, "toggle {}", self.at),
            Input::Press => write!(f, "press {}", self.at),
            Input::Set(power) => write!(f, "set {} to {power}", self.at),
            Input::Touch(true) => write!(f, "touch {}", self.at),
            Input::Touch(false) => write!(f, "leave {}", self.at),
        }
    }
}

#[derive(Deserialize, Serialize)]
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

#[derive(Default, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct StepFile {
    #[serde(skip_serializing_if = "Option::is_none")]
    toggle: Option<Target>,
    #[serde(skip_serializing_if = "Option::is_none")]
    press: Option<Target>,
    #[serde(skip_serializing_if = "Option::is_none")]
    set: Option<Target>,
    #[serde(skip_serializing_if = "Option::is_none")]
    plate: Option<Target>,
    #[serde(skip_serializing_if = "Option::is_none")]
    power: Option<u32>,
    run: u64,
}

impl StepFile {
    fn into_step(self) -> Result<Step, String> {
        let power = self
            .power
            .map(|power| SignalStrength::try_from(power).map_err(|error| error.to_string()))
            .transpose()?;
        let set = match (self.set, power) {
            (Some(target), Some(power)) => Some((target, Input::Set(power))),
            (Some(_), None) => return Err("set needs a power".to_owned()),
            (None, _) => None,
        };
        let plate = match (self.plate, power) {
            (Some(target), Some(power @ (SignalStrength::Zero | SignalStrength::Fifteen))) => {
                Some((target, Input::Touch(!power.is_zero())))
            }
            (Some(_), _) => return Err("plate needs a power of 0 or 15".to_owned()),
            (None, _) => None,
        };
        if power.is_some() && set.is_none() && plate.is_none() {
            return Err("power only goes with set or plate".to_owned());
        }
        let mut actions = [
            self.toggle.map(|target| (target, Input::Toggle)),
            self.press.map(|target| (target, Input::Press)),
            set,
            plate,
        ]
        .into_iter()
        .flatten();
        let action = actions.next();
        if actions.next().is_some() {
            return Err("a step holds at most one of toggle, press, set and plate".to_owned());
        }
        Ok(Step {
            action: action.map(|(at, input)| Action { at, input }),
            ticks: self.run,
        })
    }
}

impl From<Step> for StepFile {
    fn from(step: Step) -> StepFile {
        let mut file = StepFile {
            run: step.ticks,
            ..StepFile::default()
        };
        let Some(Action { at, input }) = step.action else {
            return file;
        };
        let (target, power) = match input {
            Input::Toggle => (&mut file.toggle, None),
            Input::Press => (&mut file.press, None),
            Input::Set(power) => (&mut file.set, Some(power)),
            Input::Touch(touched) => (&mut file.plate, Some(SignalStrength::from(touched))),
        };
        *target = Some(at);
        file.power = power.map(|power| power.get().into());
        file
    }
}

pub fn to_toml(input: &Path, steps: Vec<Step>) -> Result<String, Error> {
    let file = ScenarioFile {
        input: input.to_owned(),
        circuit: None,
        settle: None,
        step: steps.into_iter().map(StepFile::from).collect(),
    };
    Ok(toml::to_string(&file)?)
}
