use crate::reader::Span;

#[derive(Debug)]
pub struct Suggestion {
    pub span: Span,
    pub text: String,
    pub tooltip: Option<String>,
}

pub fn is_completion(candidate: &str, typed: &str) -> bool {
    if candidate.is_ascii() && typed.is_ascii() {
        candidate
            .get(..typed.len())
            .is_some_and(|head| head.eq_ignore_ascii_case(typed))
    } else {
        candidate.to_lowercase().starts_with(&typed.to_lowercase())
    }
}

pub fn merge(input: &str, suggestions: Vec<Suggestion>) -> (Span, Vec<Suggestion>) {
    let Some(first) = suggestions.first() else {
        return (Span::new(input.len(), input.len()), Vec::new());
    };
    let span = suggestions.iter().fold(first.span, |span, suggestion| {
        Span::new(
            span.start.min(suggestion.span.start),
            span.end.max(suggestion.span.end),
        )
    });
    let mut merged: Vec<_> = suggestions
        .into_iter()
        .map(|suggestion| Suggestion {
            span,
            text: format!(
                "{}{}{}",
                &input[span.start..suggestion.span.start],
                suggestion.text,
                &input[suggestion.span.end..span.end]
            ),
            tooltip: suggestion.tooltip,
        })
        .collect();
    merged.sort_by(|left, right| {
        left.text
            .cmp(&right.text)
            .then(left.tooltip.is_none().cmp(&right.tooltip.is_none()))
    });
    merged.dedup_by(|later, earlier| later.text == earlier.text);
    (span, merged)
}
