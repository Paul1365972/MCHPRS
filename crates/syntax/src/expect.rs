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

    fn offered(&self) -> usize {
        self.offers.len()
    }

    fn silenced<T>(&mut self, parse: impl FnOnce(&mut Expect) -> T) -> T {
        let suggesting = mem::replace(&mut self.suggesting, false);
        let result = parse(self);
        self.suggesting = suggesting;
        result
    }

    fn offers_since(&mut self, mark: usize) -> &mut [Suggestion] {
        &mut self.offers[mark..]
    }
}

pub fn quotable<T>(
    reader: &mut Reader<'_>,
    expect: &mut Expect,
    parse: impl Fn(&mut Reader<'_>, &mut Expect, bool) -> Result<T, SyntaxError>,
) -> Result<T, SyntaxError> {
    let Some(quote @ ('\'' | '"')) = reader.peek() else {
        return parse(reader, expect, false);
    };
    let input = reader.input();
    let open = reader.cursor();
    let content = open + quote.len_utf8();
    let close = input[content..].find(quote).map(|length| content + length);
    if let Some(close) = close
        && input[content..close].contains('\\')
    {
        let text = reader.quoted()?;
        let mut inner = Reader::new(&text);
        let value = parse(&mut inner, &mut Expect::executing(), true)
            .map_err(|error| expect.fail(open, error.message))?;
        if inner.can_read() {
            return Err(expect.fail(open, "Expected the closing quote"));
        }
        return Ok(value);
    }
    let end = close.unwrap_or(input.len());
    let after = close.map_or(end, |close| close + quote.len_utf8());
    let mut inner = Reader::at(&input[..end], content);
    let mark = expect.offered();
    let parsed = if after == input.len() {
        parse(&mut inner, expect, true)
    } else {
        expect.silenced(|expect| parse(&mut inner, expect, true))
    };
    for offer in expect.offers_since(mark) {
        let completed = format!("{}{}", &input[content..offer.span.start], offer.text);
        let mut reader = Reader::new(&completed);
        if parse(&mut reader, &mut Expect::executing(), true).is_ok() && !reader.can_read() {
            offer.text.push(quote);
        }
        offer.span.end = input.len();
    }
    let value = parsed?;
    if inner.can_read() {
        return Err(expect.fail(inner.cursor(), "Expected the closing quote"));
    }
    if close.is_none() {
        return Err(expect.fail(input.len(), "Unclosed quoted string"));
    }
    *reader = Reader::at(input, after);
    Ok(value)
}
