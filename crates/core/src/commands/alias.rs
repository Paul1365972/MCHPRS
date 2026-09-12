use mchprs_commands::reader::Span;
use std::collections::BTreeMap;

/// The input with its alias replaced by the alias's template, mapped back to the input
pub(super) struct Expansion {
    pub(super) text: String,
    mappings: Vec<SourceMapping>,
}

struct SourceMapping {
    expanded: Span,
    origin: MappingOrigin,
}

enum MappingOrigin {
    Copied { original: Span },
    Template { invocation: Span },
}

impl Expansion {
    pub(super) fn new(input: &str, aliases: &BTreeMap<String, String>) -> Expansion {
        let mut expansion = Expansion {
            text: String::new(),
            mappings: Vec::new(),
        };
        let name_end = input.find(' ').unwrap_or(input.len());
        let Some(template) = aliases.get(&input[..name_end]) else {
            expansion.append_copied(input, Span::new(0, input.len()));
            return expansion;
        };
        let invocation = Span::new(0, name_end);
        let rest_start = (name_end + 1).min(input.len());
        let rest = &input[rest_start..];
        if template.contains("{}") {
            for (index, mut part) in template.split("{}").enumerate() {
                if index > 0 {
                    expansion.append_copied(rest, Span::new(rest_start, input.len()));
                    if rest.is_empty() {
                        part = part.strip_prefix(' ').unwrap_or(part);
                    }
                }
                expansion.append_template(part, invocation);
            }
        } else {
            expansion.append_template(template, invocation);
            if name_end < input.len() {
                expansion.append_copied(&input[name_end..], Span::new(name_end, input.len()));
            }
        }
        if name_end == input.len() {
            let end = expansion.text.trim_end().len();
            expansion.text.truncate(end);
            for mapping in &mut expansion.mappings {
                mapping.expanded.start = mapping.expanded.start.min(end);
                mapping.expanded.end = mapping.expanded.end.min(end);
            }
        }
        expansion
    }

    fn append_copied(&mut self, text: &str, original: Span) {
        self.append(text, MappingOrigin::Copied { original });
    }

    fn append_template(&mut self, text: &str, invocation: Span) {
        self.append(text, MappingOrigin::Template { invocation });
    }

    fn append(&mut self, text: &str, origin: MappingOrigin) {
        let start = self.text.len();
        self.text.push_str(text);
        let expanded = Span::new(start, self.text.len());
        self.mappings.push(SourceMapping { expanded, origin });
    }

    pub(super) fn original_cursor(&self, cursor: usize) -> usize {
        let mapping = self
            .mappings
            .iter()
            .find(|mapping| cursor >= mapping.expanded.start && cursor < mapping.expanded.end)
            .or_else(|| {
                let mut mappings = self.mappings.iter().rev();
                mappings.find(|mapping| cursor == mapping.expanded.end)
            });
        let Some(mapping) = mapping else {
            return 0;
        };
        match mapping.origin {
            MappingOrigin::Copied { original } => {
                let offset = cursor - mapping.expanded.start;
                original.start + offset.min(original.end - original.start)
            }
            MappingOrigin::Template { invocation } => invocation.start,
        }
    }

    pub(super) fn original_span(&self, span: Span) -> Option<Span> {
        self.copied().find_map(|(expanded, original)| {
            (span.start >= expanded.start && span.end <= expanded.end).then(|| {
                Span::new(
                    original.start + span.start - expanded.start,
                    original.start + span.end - expanded.start,
                )
            })
        })
    }

    pub(super) fn expanded_cursor(&self, cursor: usize) -> Option<usize> {
        self.copied().find_map(|(expanded, original)| {
            (cursor >= original.start && cursor <= original.end)
                .then(|| expanded.start + cursor - original.start)
        })
    }

    fn copied(&self) -> impl Iterator<Item = (Span, Span)> + '_ {
        self.mappings
            .iter()
            .filter_map(|mapping| match mapping.origin {
                MappingOrigin::Copied { original } => Some((mapping.expanded, original)),
                MappingOrigin::Template { .. } => None,
            })
    }
}
