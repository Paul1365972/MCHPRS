use super::argument::{ClientArgument, Parameter};
use super::graph::NodeKind;
use super::registry::CommandRegistry;
use mchprs_network::packets::clientbound::{
    CCommandSuggestionsResponse, CCommandSuggestionsResponseMatch,
};
use mchprs_network::text::TextComponentBuilder;
use mchprs_syntax::flag::FlagValues;
use mchprs_syntax::reader::Span;
use mchprs_syntax::suggestion::{self, is_completion, Suggestion};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SuggestionSource {
    PlayerNames,
    SchematicFiles,
}

#[derive(Debug)]
pub struct CommandSuggestions {
    text: String,
    transaction_id: i32,
    suggestions: Vec<Suggestion>,
    requests: Vec<SuggestionRequest>,
}

impl CommandSuggestions {
    pub fn needs(&self, source: SuggestionSource) -> bool {
        self.requests.iter().any(|request| request.source == source)
    }

    pub fn resolve(
        &mut self,
        source: SuggestionSource,
        values: impl IntoIterator<Item = impl AsRef<str>>,
    ) {
        let values: Vec<_> = values.into_iter().collect();
        self.requests.retain(|request| {
            if request.source != source {
                return true;
            }
            let typed = &self.text[request.span.start..request.span.end];
            self.suggestions.extend(
                values
                    .iter()
                    .map(AsRef::as_ref)
                    .filter(|value| is_completion(value, typed))
                    .map(|value| Suggestion {
                        span: request.span,
                        text: value.to_owned(),
                        tooltip: None,
                    }),
            );
            false
        });
    }

    pub fn into_response(self) -> CCommandSuggestionsResponse {
        assert!(
            self.requests.is_empty(),
            "Suggestion sources must be resolved before encoding"
        );
        let (span, suggestions) = suggestion::merge(&self.text, self.suggestions);
        let (start, length) = span
            .utf16(&self.text)
            .expect("Suggestion ranges follow UTF-8 boundaries");
        CCommandSuggestionsResponse {
            id: self.transaction_id,
            start: start as i32,
            length: length as i32,
            matches: suggestions
                .into_iter()
                .take(1000)
                .map(|suggestion| CCommandSuggestionsResponseMatch {
                    match_: suggestion.text,
                    tooltip: suggestion
                        .tooltip
                        .map(|tooltip| TextComponentBuilder::new(tooltip).finish()),
                })
                .collect(),
        }
    }
}

#[derive(Debug)]
struct SuggestionRequest {
    source: SuggestionSource,
    span: Span,
}

impl CommandRegistry {
    pub fn suggestions(&self, text: &str, transaction_id: i32) -> CommandSuggestions {
        let mut result = CommandSuggestions {
            text: text.to_owned(),
            transaction_id,
            suggestions: Vec::new(),
            requests: Vec::new(),
        };
        let Some(input) = text.strip_prefix('/') else {
            return result;
        };
        let in_text = |span: Span| Span::new(span.start + 1, span.end + 1);
        for offered in self.graph.suggest(input) {
            let offers = offered.offers.into_iter().map(|offer| Suggestion {
                span: in_text(offer.span),
                ..offer
            });
            let NodeKind::Argument { parameter, .. } = &self.graph.node(offered.node).kind else {
                result.suggestions.extend(offers);
                continue;
            };
            let server_only = offered.path.iter().any(|&id| {
                matches!(&self.graph.node(id).kind, NodeKind::Argument { parameter, .. }
                    if matches!(parameter.client(), ClientArgument::Opaque))
            });
            if !server_only && matches!(parameter.client(), ClientArgument::Client(_)) {
                continue;
            }
            let preset = self.preset(&offered.path);
            result
                .suggestions
                .extend(offers.filter(|offer| !names_flag_in(&preset, parameter, &offer.text)));
            if let Some(source) = parameter.source() {
                result.requests.push(SuggestionRequest {
                    source,
                    span: in_text(Span::new(offered.start, input.len())),
                });
            }
        }
        result
    }
}

fn names_flag_in(preset: &FlagValues, parameter: &Parameter, text: &str) -> bool {
    let Parameter::Flags(flags) = parameter else {
        return false;
    };
    flags.iter().filter(|flag| preset.is_on(flag)).any(|flag| {
        text.strip_prefix("--") == Some(flag.name())
            || flag
                .short()
                .is_some_and(|short| text == format!("-{short}"))
    })
}
