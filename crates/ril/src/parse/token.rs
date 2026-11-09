use super::{Parsed, Parser};
use mchprs_syntax::reader::SyntaxError;
use std::str::FromStr;

impl<'a> Parser<'a> {
    pub fn global_name(&mut self) -> Parsed<String> {
        self.require('@')?;
        let at = self.reader.cursor();
        let name = self.name("a name")?.to_owned();
        if self.globals.contains_key(&name) {
            return Err(self.fail(at, format!("duplicate name @{name}")));
        }
        Ok(name)
    }

    pub fn string(&mut self) -> Parsed<String> {
        self.skip();
        self.reader.quoted()
    }

    pub fn integer<T: FromStr>(&mut self, what: &str) -> Parsed<T> {
        self.skip();
        let at = self.reader.cursor();
        let text = self
            .reader
            .take_while(|ch| ch.is_ascii_digit() || ch == '-');
        text.parse()
            .map_err(|_| self.fail(at, format!("expected {what}")))
    }

    pub fn name(&mut self, what: &str) -> Parsed<&'a str> {
        let at = self.reader.cursor();
        let name = self.reader.take_while(is_name_char);
        if name.is_empty() {
            return Err(self.fail(at, format!("expected {what}")));
        }
        Ok(name)
    }

    pub fn keyword<T: Copy>(&mut self, table: &[(&str, T)], what: &str) -> Parsed<T> {
        let at = self.reader.cursor();
        let word = self.reader.take_while(is_name_char);
        match table.iter().find(|(name, _)| *name == word) {
            Some(&(_, value)) => Ok(value),
            None => Err(self.fail(at, format!("expected {what}"))),
        }
    }

    pub fn eat_word(&mut self, word: &str) -> bool {
        self.skip();
        let mut ahead = self.reader;
        if ahead.take_while(is_name_char) == word {
            self.reader = ahead;
            return true;
        }
        false
    }

    pub fn eat(&mut self, ch: char) -> bool {
        self.skip();
        if self.reader.peek() == Some(ch) {
            self.reader.read();
            return true;
        }
        false
    }

    pub fn require(&mut self, ch: char) -> Parsed<()> {
        if self.eat(ch) {
            return Ok(());
        }
        let at = self.reader.cursor();
        Err(self.fail(at, format!("expected '{ch}'")))
    }

    pub fn skip(&mut self) {
        loop {
            self.reader.take_while(char::is_whitespace);
            if self.reader.peek() != Some('#') {
                return;
            }
            self.reader.take_while(|ch| ch != '\n');
        }
    }

    pub fn fail(&self, at: usize, message: impl Into<String>) -> SyntaxError {
        SyntaxError::new(at, message)
    }
}

pub fn is_name_char(ch: char) -> bool {
    ch.is_ascii_alphanumeric() || ch == '_'
}
