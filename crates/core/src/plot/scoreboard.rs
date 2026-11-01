use super::advance::Progress;
use crate::player::{PacketSender, Player};
use mchprs_network::packets::clientbound::{
    CDisplayObjective, CResetScore, CUpdateObjectives, CUpdateScore, ClientBoundPacket,
    ObjectiveNumberFormat,
};
use mchprs_network::text::{TextColor, TextComponent};
use mchprs_redpiler::compile::Stage;
use mchprs_redpiler_backends::BackendSetup;
use mchprs_redstone::pace::Goal;
use mchprs_save_data::plot_data::{Tps, WorldSendRate};
use std::time::Duration;

const OBJECTIVE: &str = "plot_status";
const SIDEBAR: u8 = 1;
/// The client shows at most this many lines.
const MAX_LINES: usize = 15;
const PROGRESS_BAR_WIDTH: usize = 20;

const TITLE: &str = "#FFB000";
const LABEL: &str = "#E5E7EB";
const VALUE: &str = "#FFFFFF";
const ACCENT: &str = "#5EEAD4";
const GOOD: &str = "#4ADE80";
const BUSY: &str = "#FACC15";
const BAD: &str = "#F87171";
const MUTED: &str = "#4B5563";

pub enum RedpilerStatus {
    Stopped,
    Compiling {
        setup: BackendSetup,
        stage: Option<Stage>,
    },
    Running {
        setup: BackendSetup,
    },
}

pub struct PlotStatus {
    pub redpiler: RedpilerStatus,
    pub advance: Option<Progress>,
    pub tps: Option<f64>,
    pub tps_target: Tps,
    pub tick_time: Option<Duration>,
    pub world_send_rate: WorldSendRate,
    pub running_behind: bool,
}

#[derive(PartialEq)]
struct Line {
    label: TextComponent,
    value: Option<TextComponent>,
}

impl Line {
    fn new(label: &str, value: TextComponent) -> Line {
        Line {
            label: colored(label, LABEL),
            value: Some(value),
        }
    }

    fn text(text: &str, color: &str) -> Line {
        Line {
            label: colored(text, color),
            value: None,
        }
    }

    fn blank() -> Line {
        Line {
            label: TextComponent::default(),
            value: None,
        }
    }
}

pub struct Scoreboard {
    title: TextComponent,
    lines: Vec<Line>,
}

impl Scoreboard {
    pub fn new(plot_x: i32, plot_z: i32) -> Scoreboard {
        let mut title = colored("Plot ", TITLE);
        title.bold = true;
        title
            .extra
            .push(colored(&format!("{plot_x}, {plot_z}"), VALUE));
        Scoreboard {
            title,
            lines: Vec::new(),
        }
    }

    pub fn add_player(&self, player: &Player) {
        player.send_packet(
            &CUpdateObjectives {
                objective_name: OBJECTIVE.into(),
                mode: 0,
                objective_value: self.title.clone(),
                ty: 0,
                number_format: Some(ObjectiveNumberFormat::Blank),
            }
            .encode(),
        );
        player.send_packet(
            &CDisplayObjective {
                position: SIDEBAR,
                score_name: OBJECTIVE.into(),
            }
            .encode(),
        );
        for (index, line) in self.lines.iter().enumerate() {
            player.send_packet(&update_packet(index, line).encode());
        }
    }

    pub fn remove_player(&self, player: &Player) {
        player.send_packet(
            &CUpdateObjectives {
                objective_name: OBJECTIVE.into(),
                mode: 1,
                objective_value: TextComponent::default(),
                ty: 0,
                number_format: None,
            }
            .encode(),
        );
    }

    pub fn show(&mut self, players: &[Player], status: &PlotStatus) {
        let lines = layout(status);
        for (index, line) in lines.iter().enumerate() {
            if self.lines.get(index) != Some(line) {
                let packet = update_packet(index, line).encode();
                players
                    .iter()
                    .for_each(|player| player.send_packet(&packet));
            }
        }
        for index in lines.len()..self.lines.len() {
            let packet = CResetScore {
                entity_name: entity_name(index),
                objective_name: Some(OBJECTIVE.into()),
            }
            .encode();
            players
                .iter()
                .for_each(|player| player.send_packet(&packet));
        }
        self.lines = lines;
    }
}

fn layout(status: &PlotStatus) -> Vec<Line> {
    let mut lines = Vec::new();
    match &status.redpiler {
        RedpilerStatus::Stopped => {
            lines.push(Line::new("Redpiler", colored("● Stopped", LABEL)));
        }
        RedpilerStatus::Compiling { setup, stage } => {
            lines.push(Line::new("Redpiler", colored("● Compiling", BUSY)));
            push_setup(&mut lines, setup);
            push_progress(&mut lines, *stage);
        }
        RedpilerStatus::Running { setup } => {
            lines.push(Line::new("Redpiler", colored("● Running", GOOD)));
            push_setup(&mut lines, setup);
        }
    }

    lines.push(Line::blank());
    let rate_color = if status.running_behind { BAD } else { GOOD };
    let measured = status.tps.map_or("–".to_owned(), format_rate);
    let mut rate = colored(&measured, rate_color);
    let target = match status.tps_target {
        Tps::Limited(rate) if rate <= 0.0 => "paused".to_owned(),
        Tps::Limited(rate) => format_rate(f64::from(rate)),
        Tps::Unlimited => "∞".to_owned(),
    };
    rate.extra.push(colored(&format!(" / {target}"), LABEL));
    lines.push(Line::new("RTPS", rate));
    if let Some(progress) = status.advance {
        push_advance(&mut lines, progress);
    }
    let tick_time = status.tick_time.map_or("–".to_owned(), format_duration);
    lines.push(Line::new("Tick time", colored(&tick_time, VALUE)));
    let send_rate = match status.world_send_rate.0 {
        rate if rate > 0.0 => format!("{} Hz", format_rate(f64::from(rate))),
        _ => "off".to_owned(),
    };
    lines.push(Line::new("World sends", colored(&send_rate, VALUE)));
    if status.running_behind {
        lines.push(Line::text("⚠ Running behind", BAD));
    }
    lines.truncate(MAX_LINES);
    lines
}

fn push_setup(lines: &mut Vec<Line>, setup: &BackendSetup) {
    lines.push(Line::new(
        "Backend",
        colored(setup.backend().name(), ACCENT),
    ));
    let flags = setup.flags().to_string();
    if !flags.is_empty() {
        lines.push(Line::new("Flags", colored(&flags, ACCENT)));
    }
}

fn push_progress(lines: &mut Vec<Line>, stage: Option<Stage>) {
    match stage {
        None => lines.push(Line::text("Extracting components", VALUE)),
        Some(Stage::Extracted { .. }) => lines.push(Line::text("Optimizing the circuit", VALUE)),
        Some(Stage::Optimizing {
            pass: step,
            completed,
            total,
        })
        | Some(Stage::Building {
            step,
            completed,
            total,
        }) => push_step(lines, step, completed, total),
    }
}

fn push_step(lines: &mut Vec<Line>, step: &str, completed: usize, total: usize) {
    if total <= 1 {
        lines.push(Line::text(step, VALUE));
        return;
    }
    lines.push(Line {
        label: colored(step, VALUE),
        value: Some(colored(&format!("{completed}/{total}"), LABEL)),
    });
    push_bar(lines, completed as u64, total as u64);
}

fn push_advance(lines: &mut Vec<Line>, progress: Progress) {
    let mut ran = colored(&format_count(progress.ran), BUSY);
    match progress.goal {
        Goal::Ticks(ticks) => {
            ran.extra
                .push(colored(&format!(" / {}", format_count(ticks)), LABEL));
            lines.push(Line::new("Advancing", ran));
            push_bar(lines, progress.ran, ticks);
        }
        Goal::Settled => lines.push(Line::new("Settling", ran)),
    }
}

fn push_bar(lines: &mut Vec<Line>, completed: u64, total: u64) {
    let fraction = match total {
        0 => 0.0,
        total => completed as f64 / total as f64,
    };
    let filled = ((fraction * PROGRESS_BAR_WIDTH as f64) as usize).min(PROGRESS_BAR_WIDTH);
    let mut bar = colored(&"|".repeat(filled), GOOD);
    bar.extra
        .push(colored(&"|".repeat(PROGRESS_BAR_WIDTH - filled), MUTED));
    lines.push(Line {
        label: bar,
        value: None,
    });
}

fn update_packet(index: usize, line: &Line) -> CUpdateScore {
    CUpdateScore {
        entity_name: entity_name(index),
        objective_name: OBJECTIVE.into(),
        value: (MAX_LINES - index) as i32,
        display_name: Some(line.label.clone()),
        number_format: Some(match &line.value {
            Some(value) => ObjectiveNumberFormat::Fixed {
                content: value.clone(),
            },
            None => ObjectiveNumberFormat::Blank,
        }),
    }
}

fn entity_name(index: usize) -> String {
    format!("line_{index}")
}

fn colored(text: &str, color: &str) -> TextComponent {
    TextComponent {
        text: text.to_owned(),
        color: Some(TextColor::Hex(color.to_owned())),
        ..Default::default()
    }
}

fn format_rate(rate: f64) -> String {
    match rate {
        rate if rate >= 1e6 => format!("{:.2}M", rate / 1e6),
        rate if rate >= 1e4 => format!("{:.1}k", rate / 1e3),
        rate if rate >= 100.0 => format!("{rate:.0}"),
        rate if rate.fract() == 0.0 => format!("{rate:.0}"),
        rate => format!("{rate:.1}"),
    }
}

fn format_count(count: u64) -> String {
    match count {
        1_000_000.. => format!("{:.2}M", count as f64 / 1e6),
        10_000.. => format!("{:.1}k", count as f64 / 1e3),
        _ => count.to_string(),
    }
}

fn format_duration(duration: Duration) -> String {
    let nanos = duration.as_nanos();
    match nanos {
        0..1_000 => format!("{nanos} ns"),
        1_000..1_000_000 => format!("{:.1} µs", nanos as f64 / 1e3),
        _ => format!("{:.2} ms", nanos as f64 / 1e6),
    }
}
