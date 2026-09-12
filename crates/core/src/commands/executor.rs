use super::argument::Arguments;
use super::context::ExecutionContext;
use super::error::{CommandError, CommandResult, InternalError};
use super::registry::CommandRegistry;
use crate::player::PacketSender;
use crate::plot::Plot;
use std::sync::Arc;

pub(super) type Executor =
    Arc<dyn Fn(&mut ExecutionContext<'_>) -> CommandResult<()> + Send + Sync>;

pub(super) struct Invocation {
    executor: Executor,
    arguments: Arguments,
}

impl CommandRegistry {
    pub fn execute(
        &self,
        plot: &mut Plot,
        player_idx: usize,
        input: &str,
    ) -> Result<(), InternalError> {
        if input.is_empty() {
            return Ok(());
        }
        let player = plot
            .players
            .get(player_idx)
            .ok_or(InternalError::InvalidPlayerIndex { index: player_idx })?;
        let invocation = match self.dispatch(input) {
            Ok(invocation) => invocation,
            Err(error) => {
                player.send_error_message(&error);
                return Ok(());
            }
        };
        let mut context = ExecutionContext::new(plot, player_idx, &invocation.arguments);
        let result = (invocation.executor)(&mut context);
        let mutated_world = context.mutated_world();
        if let Err(CommandError::Runtime(error)) = &result {
            context.player()?.send_error_message(error);
        }
        if mutated_world {
            plot.publish_world();
        }
        match result {
            Ok(()) | Err(CommandError::Runtime(_)) => Ok(()),
            Err(CommandError::Internal(error)) => Err(error),
        }
    }

    fn dispatch(&self, input: &str) -> Result<Invocation, String> {
        let parsed = self.graph.parse(input);
        if let Some(error) = parsed.error(input) {
            return Err(error.contextual(input));
        }
        if let Some(flag) = self.refused_preset(&parsed.context.nodes) {
            return Err(format!(
                "This command doesn't take --{flag}, which the alias sets"
            ));
        }
        let preset = self.preset(&parsed.context.nodes);
        let executor = parsed
            .context
            .executor
            .expect("Successful parse must have an executor");

        Ok(Invocation {
            executor,
            arguments: Arguments::new(parsed.context.arguments, preset),
        })
    }
}
