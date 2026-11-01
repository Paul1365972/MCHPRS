use crate::player::{PacketSender, Player};
use mchprs_network::packets::clientbound::{
    CDisplayObjective, CResetScore, CUpdateObjectives, CUpdateScore, ClientBoundPacket,
    ObjectiveNumberFormat,
};
use mchprs_redpiler::{CompilerOptions, PassStatus, Reporting};
use mchprs_text::{ColorCode, TextComponent, TextComponentBuilder};
use std::borrow::Cow;

#[derive(PartialEq, Eq, Default, Clone, Copy)]
pub enum RedpilerState {
    #[default]
    Stopped,
    Compiling(Option<PassStatus>),
    Running,
}

impl RedpilerState {
    fn to_str(self) -> TextComponent {
        let (text, color) = match self {
            RedpilerState::Stopped => ("Stopped".to_owned(), ColorCode::LightPurple.into()),
            RedpilerState::Compiling(None) => ("Compiling".to_owned(), ColorCode::Yellow.into()),
            RedpilerState::Compiling(Some(pass)) => (
                format!("Compiling {}/{}: {}", pass.index + 1, pass.count, pass.name),
                ColorCode::Yellow.into(),
            ),
            RedpilerState::Running => ("Running".to_owned(), ColorCode::Green.into()),
        };
        TextComponentBuilder::new(text).color(color).bold().finish()
    }
}

#[derive(Clone)]
pub struct ScoreboardLine {
    entity_name: String,
    text: TextComponent,
}

impl ScoreboardLine {
    pub fn new(entity_name: String, text: TextComponent) -> Self {
        Self { entity_name, text }
    }
}

pub struct Scoreboard {
    current_state: Vec<ScoreboardLine>,
}

impl Default for Scoreboard {
    fn default() -> Scoreboard {
        let mut sb = Scoreboard {
            current_state: vec![],
        };
        sb.set_redpiler_state(&[], RedpilerState::Stopped);
        sb
    }
}

impl Scoreboard {
    fn make_update_packet(&self, line: usize) -> CUpdateScore {
        CUpdateScore {
            entity_name: self.current_state[line].entity_name.clone(),
            objective_name: "redpiler_status".to_string(),
            value: (self.current_state.len() - line) as i32,
            display_name: Some(self.current_state[line].text.clone()),
            number_format: Some(ObjectiveNumberFormat::Blank),
        }
    }

    fn make_removal_packet(&self, line: usize) -> CResetScore {
        CResetScore {
            entity_name: self.current_state[line].entity_name.clone(),
            objective_name: Some("redpiler_status".to_string()),
        }
    }

    fn set_lines(&mut self, players: &[Player], lines: Vec<ScoreboardLine>) {
        for line in 0..self.current_state.len() {
            let kept = lines
                .iter()
                .any(|new| new.entity_name == self.current_state[line].entity_name);
            if !kept {
                let removal_packet = self.make_removal_packet(line).encode();
                players.iter().for_each(|p| p.send_packet(&removal_packet));
            }
        }

        self.current_state = lines;

        for line in 0..self.current_state.len() {
            let update_packet = self.make_update_packet(line).encode();
            players.iter().for_each(|p| p.send_packet(&update_packet));
        }
    }

    fn set_line(&mut self, players: &[Player], line: usize, text: ScoreboardLine) {
        if line == self.current_state.len() {
            self.current_state.push(text);
        } else {
            if self.current_state[line].entity_name != text.entity_name {
                let removal_packet = self.make_removal_packet(line).encode();
                players.iter().for_each(|p| p.send_packet(&removal_packet));
            }

            self.current_state[line] = text;
        }

        let update_packet = self.make_update_packet(line).encode();
        players.iter().for_each(|p| p.send_packet(&update_packet));
    }

    pub fn add_player(&self, player: &Player) {
        player.send_packet(
            &CUpdateObjectives {
                objective_name: "redpiler_status".into(),
                mode: 0,
                objective_value: TextComponentBuilder::new("Redpiler Status".into())
                    .color_code(ColorCode::Red)
                    .finish(),
                ty: 0,
                number_format: Some(ObjectiveNumberFormat::Blank),
            }
            .encode(),
        );
        player.send_packet(
            &CDisplayObjective {
                position: 1,
                score_name: "redpiler_status".into(),
            }
            .encode(),
        );
        for i in 0..self.current_state.len() {
            player.send_packet(&self.make_update_packet(i).encode());
        }
    }

    pub fn remove_player(&mut self, player: &Player) {
        for i in 0..self.current_state.len() {
            player.send_packet(&self.make_removal_packet(i).encode());
        }
    }

    pub fn set_redpiler_state(&mut self, players: &[Player], state: RedpilerState) {
        self.set_line(
            players,
            0,
            ScoreboardLine::new("redpiler_state".into(), state.to_str()),
        );
    }

    pub fn set_redpiler_options(&mut self, players: &[Player], options: &CompilerOptions) {
        let mut new_lines = vec![self.current_state[0].clone()];

        let mut flags: Vec<(&str, Cow<'static, str>)> = Vec::new();
        if let Some(backend) = &options.backend {
            flags.push(("backend", format!("- {backend} backend").into()));
        }
        if options.optimize {
            flags.push(("o", "- optimize".into()));
        }
        if options.export {
            flags.push(("e", "- export".into()));
        }
        match options.reporting {
            Reporting::All => {}
            Reporting::InputsAndOutputsWhileRunning => flags.push(("i", "- io only".into())),
            Reporting::InputsAndOutputs => flags.push(("i", "- io snapshots".into())),
        }
        if options.update {
            flags.push(("u", "- update".into()));
        }
        if options.wire_dot_out {
            flags.push(("d", "- wire dot out".into()));
        }
        if options.illegal_states_out {
            flags.push(("l", "- illegal states out".into()));
        }
        if options.wire_cross_out {
            flags.push(("c", "- wire cross out".into()));
        }

        if !flags.is_empty() {
            new_lines.push(ScoreboardLine::new(
                "flags".into(),
                TextComponentBuilder::new("Flags:".into())
                    .color_code(ColorCode::Gray)
                    .finish(),
            ));
            new_lines.extend(flags.into_iter().map(|(name, text)| {
                ScoreboardLine::new(
                    format!("flag_{name}"),
                    TextComponentBuilder::new(text.into_owned())
                        .color_code(ColorCode::Aqua)
                        .finish(),
                )
            }));
        }
        self.set_lines(players, new_lines);
    }
}
