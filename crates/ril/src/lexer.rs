use crate::Error;
use mchprs_blocks::blocks::{ComparatorMode, Instrument};
use std::iter::Peekable;
use std::str::CharIndices;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum ComponentType {
    Repeater,
    Torch,
    Comparator,
    Lamp,
    Button,
    Lever,
    PressurePlate,
    Trapdoor,
    Wire,
    Constant,
    NoteBlock,
}

#[derive(PartialEq, Debug, Clone)]
pub(crate) enum TokenKind {
    /// The `circuit` keyword
    Circuit,
    /// The `test` keyword
    Test,
    /// The `test_args` keyword
    TestArgs,
    /// The `schematic` keyword
    Schematic,
    ComponentType(ComponentType),
    Instrument(Instrument),
    ComparatorMode(ComparatorMode),
    Bool(bool),
    Int(u32),
    String(String),
    /// %name
    Value(String),
    /// @name
    GlobalValue(String),
    /// The `none` keyword
    None,
    /// The `side` keyword
    Side,
    /// `->`
    Arrow,
    Colon,
    RightBracket,
    LeftBracket,
    RightCurlyBrace,
    LeftCurlyBrace,
    LeftParens,
    RightParens,
    Comma,
    Equals,
}

impl TokenKind {
    pub(crate) fn friendly_name(&self) -> &'static str {
        use TokenKind::*;
        match self {
            Circuit => "circuit keyword",
            Test => "test keyword",
            TestArgs => "test_args keyword",
            Schematic => "schematic keyword",
            ComponentType(_) => "component type",
            Instrument(_) => "instrument",
            ComparatorMode(_) => "comparator mode",
            Bool(_) => "boolean",
            Int(_) => "int",
            String(_) => "string",
            Value(_) => "value",
            GlobalValue(_) => "global value",
            None => "none keyword",
            Side => "side keyword",
            Arrow => "'->'",
            Colon => "':'",
            LeftBracket => "'['",
            RightBracket => "']'",
            LeftCurlyBrace => "'{'",
            RightCurlyBrace => "'}'",
            LeftParens => "'('",
            RightParens => "')'",
            Comma => "','",
            Equals => "'='",
        }
    }
}

#[derive(PartialEq, Debug)]
pub(crate) struct Token {
    /// Byte offset in the source
    pub(crate) offset: usize,
    pub(crate) kind: TokenKind,
}

impl Token {
    fn new(offset: usize, kind: TokenKind) -> Self {
        Self { offset, kind }
    }
}

pub(crate) struct Lexer<'a> {
    source: &'a str,
    chars: Peekable<CharIndices<'a>>,
    tokens: Vec<Token>,
}

impl<'a> Lexer<'a> {
    pub(crate) fn lex(source: &'a str) -> Result<Vec<Token>, Error> {
        let mut lexer = Lexer {
            source,
            chars: source.char_indices().peekable(),
            tokens: Vec::new(),
        };
        while let Some(&(offset, c)) = lexer.chars.peek() {
            let kind = match c {
                ':' => TokenKind::Colon,
                '[' => TokenKind::LeftBracket,
                ']' => TokenKind::RightBracket,
                '{' => TokenKind::LeftCurlyBrace,
                '}' => TokenKind::RightCurlyBrace,
                '(' => TokenKind::LeftParens,
                ')' => TokenKind::RightParens,
                ',' => TokenKind::Comma,
                '=' => TokenKind::Equals,
                '#' => {
                    lexer.skip_line();
                    continue;
                }
                '-' => {
                    lexer.chars.next();
                    if lexer.chars.next_if(|&(_, c)| c == '>').is_none() {
                        return Err(lexer.error(offset, "expected '>' after '-'"));
                    }
                    lexer.tokens.push(Token::new(offset, TokenKind::Arrow));
                    continue;
                }
                '%' | '@' => {
                    lexer.chars.next();
                    let name = lexer.read_name();
                    if name.is_empty() {
                        return Err(lexer.error(offset, format!("expected a name after {c}")));
                    }
                    let kind = match c {
                        '%' => TokenKind::Value(name),
                        _ => TokenKind::GlobalValue(name),
                    };
                    lexer.tokens.push(Token::new(offset, kind));
                    continue;
                }
                '"' => {
                    let string = lexer.read_string(offset)?;
                    lexer
                        .tokens
                        .push(Token::new(offset, TokenKind::String(string)));
                    continue;
                }
                _ if c.is_whitespace() => {
                    lexer.chars.next();
                    continue;
                }
                _ if c.is_ascii_digit() => {
                    lexer.read_int(offset)?;
                    continue;
                }
                _ if c.is_ascii_alphabetic() => {
                    lexer.read_keyword(offset)?;
                    continue;
                }
                _ => return Err(lexer.error(offset, format!("unexpected character {c}"))),
            };
            lexer.tokens.push(Token::new(offset, kind));
            lexer.chars.next();
        }
        Ok(lexer.tokens)
    }

    fn error(&self, offset: usize, message: impl Into<String>) -> Error {
        Error::at(self.source, offset, message)
    }

    fn read_while(&mut self, accepted: impl Fn(char) -> bool) -> &'a str {
        let start = self
            .chars
            .peek()
            .map_or(self.source.len(), |&(offset, _)| offset);
        while self.chars.next_if(|&(_, c)| accepted(c)).is_some() {}
        let end = self
            .chars
            .peek()
            .map_or(self.source.len(), |&(offset, _)| offset);
        &self.source[start..end]
    }

    fn read_string(&mut self, offset: usize) -> Result<String, Error> {
        self.chars.next();
        let string = self.read_while(|c| c != '"').to_owned();
        if self.chars.next().is_none() {
            return Err(self.error(offset, "unterminated string"));
        }
        Ok(string)
    }

    fn read_name(&mut self) -> String {
        self.read_while(|c| c.is_ascii_alphanumeric() || c == '_')
            .to_owned()
    }

    fn read_int(&mut self, offset: usize) -> Result<(), Error> {
        let digits = self.read_while(|c| c.is_ascii_digit());
        let int = digits
            .parse()
            .map_err(|_| self.error(offset, format!("{digits} is too large")))?;
        self.tokens.push(Token::new(offset, TokenKind::Int(int)));
        Ok(())
    }

    fn read_keyword(&mut self, offset: usize) -> Result<(), Error> {
        let word = self.read_while(|c| c.is_ascii_alphabetic() || c == '_');
        let kind = match word {
            "circuit" => TokenKind::Circuit,
            "test" => TokenKind::Test,
            "test_args" => TokenKind::TestArgs,
            "schematic" => TokenKind::Schematic,
            "none" => TokenKind::None,
            "side" => TokenKind::Side,
            "repeater" => TokenKind::ComponentType(ComponentType::Repeater),
            "torch" => TokenKind::ComponentType(ComponentType::Torch),
            "comparator" => TokenKind::ComponentType(ComponentType::Comparator),
            "lamp" => TokenKind::ComponentType(ComponentType::Lamp),
            "button" => TokenKind::ComponentType(ComponentType::Button),
            "lever" => TokenKind::ComponentType(ComponentType::Lever),
            "pressure_plate" => TokenKind::ComponentType(ComponentType::PressurePlate),
            "trapdoor" => TokenKind::ComponentType(ComponentType::Trapdoor),
            "wire" => TokenKind::ComponentType(ComponentType::Wire),
            "constant" => TokenKind::ComponentType(ComponentType::Constant),
            "note_block" => TokenKind::ComponentType(ComponentType::NoteBlock),
            // Note Block Instruments
            "harp" => TokenKind::Instrument(Instrument::Harp),
            "basedrum" => TokenKind::Instrument(Instrument::Basedrum),
            "snare" => TokenKind::Instrument(Instrument::Snare),
            "hat" => TokenKind::Instrument(Instrument::Hat),
            "bass" => TokenKind::Instrument(Instrument::Bass),
            "flute" => TokenKind::Instrument(Instrument::Flute),
            "bell" => TokenKind::Instrument(Instrument::Bell),
            "guitar" => TokenKind::Instrument(Instrument::Guitar),
            "chime" => TokenKind::Instrument(Instrument::Chime),
            "xylophone" => TokenKind::Instrument(Instrument::Xylophone),
            "iron_xylophone" => TokenKind::Instrument(Instrument::IronXylophone),
            "cow_bell" => TokenKind::Instrument(Instrument::CowBell),
            "didgeridoo" => TokenKind::Instrument(Instrument::Didgeridoo),
            "bit" => TokenKind::Instrument(Instrument::Bit),
            "banjo" => TokenKind::Instrument(Instrument::Banjo),
            "pling" => TokenKind::Instrument(Instrument::Pling),
            // Bool
            "false" => TokenKind::Bool(false),
            "true" => TokenKind::Bool(true),
            // Comparator Mode
            "compare" => TokenKind::ComparatorMode(ComparatorMode::Compare),
            "subtract" => TokenKind::ComparatorMode(ComparatorMode::Subtract),
            _ => return Err(self.error(offset, format!("unknown keyword {word}"))),
        };
        self.tokens.push(Token::new(offset, kind));
        Ok(())
    }

    fn skip_line(&mut self) {
        for (_, c) in self.chars.by_ref() {
            if c == '\n' {
                break;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lexer_comparator() {
        use TokenKind::*;
        let src = r"
            circuit {
              %comp = comparator subtract, none, false, 0 -> [%10:2, side %11:1]  # Loc: (126, 8, 118)
            }
        ";
        let actual_tokens = Lexer::lex(src).unwrap();
        let expected_tokens = &[
            Token::new(13, Circuit),
            Token::new(21, LeftCurlyBrace),
            Token::new(37, Value("comp".to_owned())),
            Token::new(43, Equals),
            Token::new(45, ComponentType(super::ComponentType::Comparator)),
            Token::new(56, ComparatorMode(super::ComparatorMode::Subtract)),
            Token::new(64, Comma),
            Token::new(66, None),
            Token::new(70, Comma),
            Token::new(72, Bool(false)),
            Token::new(77, Comma),
            Token::new(79, Int(0)),
            Token::new(81, Arrow),
            Token::new(84, LeftBracket),
            Token::new(85, Value("10".to_owned())),
            Token::new(88, Colon),
            Token::new(89, Int(2)),
            Token::new(90, Comma),
            Token::new(92, Side),
            Token::new(97, Value("11".to_owned())),
            Token::new(100, Colon),
            Token::new(101, Int(1)),
            Token::new(102, RightBracket),
            Token::new(138, RightCurlyBrace),
        ];
        assert_eq!(&actual_tokens, expected_tokens);
    }
}
