use mchprs_commands::flag::{self, Flag};
use mchprs_commands::graph::Argument;
use mchprs_commands::reader::Reader;
use mchprs_commands::suggestion::SuggestionsBuilder;
use std::iter;

pub(super) fn suggest_flags(flags: &[Flag], builder: &mut SuggestionsBuilder<'_, '_>) {
    let typed = builder.remaining();
    let token_start = typed.rfind(' ').map_or(0, |index| index + 1);
    let mut reader = Reader::new(&typed[..token_start]);
    let Ok(given) = flag::parse(&mut reader, flags) else {
        return;
    };
    let offset = builder.span().start + token_start;
    let mut token = builder.at(offset);
    for flag in flags
        .iter()
        .filter(|flag| !given.names().any(|name| name == flag.name()))
    {
        let names = iter::once(format!("--{}", flag.name()))
            .chain(flag.short().map(|short| format!("-{short}")));
        for name in names {
            if token.matches(&name) {
                token.suggest_with_tooltip(name, flag.description());
            }
        }
    }
}

pub(super) fn suggest_coordinates(
    parser: &impl Argument,
    templates: &[&str],
    builder: &mut SuggestionsBuilder<'_, '_>,
) {
    let entered: Vec<_> = builder.remaining().split(' ').collect();
    for template in templates {
        let mut coordinates: Vec<_> = template.split(' ').collect();
        if entered.len() > coordinates.len() {
            continue;
        }
        for (coordinate, entered) in coordinates.iter_mut().zip(&entered) {
            if !entered.is_empty() {
                *coordinate = entered;
            }
        }
        let suggestion = coordinates.join(" ");
        let mut reader = Reader::new(&suggestion);
        if parser.parse(&mut reader).is_ok() && !reader.can_read() {
            builder.suggest_matching([suggestion]);
        }
    }
}
