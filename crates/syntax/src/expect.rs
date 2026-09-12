use crate::reader::{Reader, Span, SyntaxError};
use crate::suggestion::{is_completion, Suggestion};
use std::mem;

pub struct Expect {
    furthest: Option<SyntaxError>,
    suggesting: bool,
    completion_start: Option<usize>,
    offers: Vec<Suggestion>,
}

impl Expect {
    pub fn executing() -> Expect {
        Expect::new(false)
    }

    pub fn suggesting() -> Expect {
        Expect::new(true)
    }

    fn new(suggesting: bool) -> Expect {
        Expect {
            furthest: None,
            suggesting,
            completion_start: None,
            offers: Vec::new(),
        }
    }

    pub fn is_suggesting(&self) -> bool {
        self.suggesting
    }

    pub fn fail(&mut self, at: usize, message: impl Into<String>) -> SyntaxError {
        self.record(SyntaxError::new(at, message))
    }

    pub fn fail_recognized(&mut self, at: usize, message: impl Into<String>) -> SyntaxError {
        self.record(SyntaxError::recognized(at, message))
    }

    fn record(&mut self, error: SyntaxError) -> SyntaxError {
        if self
            .furthest
            .as_ref()
            .is_none_or(|furthest| furthest.cursor <= error.cursor)
        {
            self.furthest = Some(error.clone());
        }
        error
    }

    pub fn complete_from(&mut self, reader: &Reader<'_>, start: usize) -> bool {
        if !self.suggesting || reader.can_read() {
            return false;
        }
        self.completion_start = Some(
            self.completion_start
                .map_or(start, |known| known.min(start)),
        );
        true
    }

    pub fn offer<I>(&mut self, reader: &Reader<'_>, start: usize, candidates: impl FnOnce() -> I)
    where
        I: IntoIterator,
        I::Item: Into<String>,
    {
        self.offer_with_tooltips(reader, start, || {
            candidates()
                .into_iter()
                .map(|candidate| (candidate, None::<String>))
        });
    }

    pub fn offer_with_tooltips<I, T, D>(
        &mut self,
        reader: &Reader<'_>,
        start: usize,
        candidates: impl FnOnce() -> I,
    ) where
        I: IntoIterator<Item = (T, Option<D>)>,
        T: Into<String>,
        D: Into<String>,
    {
        if !self.complete_from(reader, start) {
            return;
        }
        let input = reader.input();
        let typed = &input[start..];
        let span = Span::new(start, input.len());
        for (text, tooltip) in candidates() {
            let text = text.into();
            if is_completion(&text, typed) {
                self.offers.push(Suggestion {
                    span,
                    text,
                    tooltip: tooltip.map(Into::into),
                });
            }
        }
    }

    pub fn error(&self, returned: SyntaxError) -> SyntaxError {
        match &self.furthest {
            Some(furthest) if furthest.cursor > returned.cursor => furthest.clone(),
            _ => returned,
        }
    }

    pub fn into_completion(self) -> Option<(usize, Vec<Suggestion>)> {
        Some((self.completion_start?, self.offers))
    }

    pub fn offered(&self) -> usize {
        self.offers.len()
    }

    pub fn silenced<T>(&mut self, parse: impl FnOnce(&mut Expect) -> T) -> T {
        let suggesting = mem::replace(&mut self.suggesting, false);
        let result = parse(self);
        self.suggesting = suggesting;
        result
    }

    pub fn offers_since(&mut self, mark: usize) -> &mut [Suggestion] {
        &mut self.offers[mark..]
    }
}
