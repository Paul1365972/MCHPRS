use crate::compile_graph::{
    CompileGraph, CompileLink, CompileNode, Direction, LinkType, NodeIdx, NodeState, NodeType,
};
use crate::string_replacer::StringReplacer;
use crate::CompilerOptions;
use indexmap::IndexMap;
use itertools::Itertools;
use mchprs_blocks::blocks::{ComparatorMode, Instrument};
use mchprs_blocks::signal_strength::SignalStrength;
use rustc_hash::FxHashMap;
use std::iter::Peekable;
use std::str::CharIndices;
use std::{fmt, vec};

fn dump_node_name(f: &mut impl fmt::Write, graph: &CompileGraph, node_idx: NodeIdx) -> fmt::Result {
    write!(f, "%")?;
    if let Some(name) = &graph[node_idx].name {
        write!(f, "{}", name)
    } else {
        write!(f, "{}", node_idx.index())
    }
}

struct FarInputFormatter(Option<SignalStrength>);

impl fmt::Display for FarInputFormatter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            Some(power) => write!(f, "{}", power),
            None => write!(f, "none"),
        }
    }
}

fn dump_outputs(f: &mut impl fmt::Write, graph: &CompileGraph, node_idx: NodeIdx) -> fmt::Result {
    let mut outputs = graph.edges(node_idx, Direction::Outgoing).peekable();
    if outputs.peek().is_none() {
        return Ok(());
    }
    write!(f, " -> [")?;
    for (idx, edge) in outputs.enumerate() {
        if idx > 0 {
            write!(f, ", ")?;
        }
        if edge.weight().ty == LinkType::Side {
            write!(f, "side ")?;
        }
        dump_node_name(f, graph, edge.target())?;
        write!(f, ":{}", edge.weight().weight)?;
    }
    write!(f, "]")
}

fn dump_node(f: &mut impl fmt::Write, graph: &CompileGraph, node_idx: NodeIdx) -> fmt::Result {
    let node = &graph[node_idx];

    write!(f, "  ")?;
    dump_node_name(f, graph, node_idx)?;
    write!(f, " = ")?;

    match node.ty {
        NodeType::Repeater { delay, prioritized } => write!(
            f,
            "repeater {}, {}, {}, {}",
            delay,
            prioritized,
            node.state.repeater_locked,
            node.state.is_powered(),
        ),
        NodeType::Torch => write!(f, "torch {}", node.state.is_powered()),
        NodeType::Comparator {
            mode,
            far_input,
            prioritized,
        } => write!(
            f,
            "comparator {}, {}, {}, {}",
            mode,
            FarInputFormatter(far_input),
            prioritized,
            node.state.power,
        ),
        NodeType::Lamp => write!(f, "lamp {}", node.state.is_powered()),
        NodeType::Button => write!(f, "button {}", node.state.is_powered()),
        NodeType::Lever => write!(f, "lever {}", node.state.is_powered()),
        NodeType::PressurePlate => write!(f, "pressure_plate {}", node.state.is_powered()),
        NodeType::Trapdoor => write!(f, "trapdoor {}", node.state.is_powered()),
        NodeType::Wire => write!(f, "wire {}", node.state.power),
        NodeType::Constant => write!(f, "constant {}", node.state.power),
        NodeType::NoteBlock { instrument, note } => write!(
            f,
            "note_block {}, {}, {}",
            instrument,
            note,
            node.state.is_powered()
        ),
    }?;
    dump_outputs(f, graph, node_idx)?;

    if !node.block.is_empty() {
        write!(f, "  # Loc: ")?;
        for (idx, (pos, _)) in node.block.iter().copied().enumerate() {
            if idx != 0 {
                write!(f, ", ")?;
            }
            write!(f, "{}", pos)?;
        }
    }

    Ok(())
}

pub fn dump_graph(f: &mut impl fmt::Write, graph: &CompileGraph, name: &str) -> fmt::Result {
    writeln!(f, "circuit @{} {{", name)?;
    for node_idx in graph.node_indices() {
        dump_node(f, graph, node_idx)?;
        writeln!(f)?;
    }
    write!(f, "}}")
}

struct GraphDumper<'a> {
    graph: &'a CompileGraph,
}

impl<'a> fmt::Display for GraphDumper<'a> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        dump_graph(f, self.graph, "redpiler_dump")
    }
}

pub trait DumpGraph {
    fn dump(&self);
}

impl DumpGraph for CompileGraph {
    fn dump(&self) {
        eprintln!("{}", GraphDumper { graph: self });
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum ComponentType {
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

impl ComponentType {
    fn simple_node_type(self) -> NodeType {
        match self {
            ComponentType::Torch => NodeType::Torch,
            ComponentType::Lamp => NodeType::Lamp,
            ComponentType::Button => NodeType::Button,
            ComponentType::Lever => NodeType::Lever,
            ComponentType::PressurePlate => NodeType::PressurePlate,
            ComponentType::Trapdoor => NodeType::Trapdoor,
            ComponentType::Wire => NodeType::Wire,
            ComponentType::Constant => NodeType::Constant,
            _ => panic!("not a simple type"),
        }
    }
}

#[derive(PartialEq, Debug, Clone)]
enum TokenType {
    /// The `circuit` keyword
    Circuit,
    /// The `backend_circuit` keyword
    BackendCircuit,
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

impl TokenType {
    fn unwrap_value(self) -> String {
        match self {
            TokenType::Value(name) => name,
            _ => unreachable!(),
        }
    }

    fn unwrap_global_value(self) -> String {
        match self {
            TokenType::GlobalValue(name) => name,
            _ => unreachable!(),
        }
    }

    fn unwrap_component_ty(self) -> ComponentType {
        match self {
            TokenType::ComponentType(ty) => ty,
            _ => unreachable!(),
        }
    }

    fn unwrap_int(&self) -> u32 {
        match self {
            TokenType::Int(val) => *val,
            _ => unreachable!(),
        }
    }

    fn unwrap_comparator_mode(self) -> ComparatorMode {
        match self {
            TokenType::ComparatorMode(val) => val,
            _ => unreachable!(),
        }
    }

    fn unwrap_bool(&self) -> bool {
        match self {
            TokenType::Bool(val) => *val,
            _ => unreachable!(),
        }
    }

    fn unwrap_instrument(&self) -> Instrument {
        match self {
            TokenType::Instrument(val) => *val,
            _ => unreachable!(),
        }
    }

    fn unwrap_string(self) -> String {
        match self {
            TokenType::String(val) => val,
            _ => unreachable!(),
        }
    }

    fn friendy_name(&self) -> &'static str {
        use TokenType::*;
        match self {
            Circuit => "circuit keyword",
            BackendCircuit => "backend_circuit keyword",
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
struct Token {
    pos: usize,
    ty: TokenType,
}

impl Token {
    fn new(pos: usize, ty: TokenType) -> Self {
        Self { pos, ty }
    }
}

#[derive(Debug)]
pub struct RILParserError {
    /// Byte position from source file
    pub pos: usize,
    pub message: String,
}

impl RILParserError {
    fn new<S: ToString>(pos: usize, message: S) -> Self {
        Self {
            pos,
            message: message.to_string(),
        }
    }

    fn new_expected<S: ToString>(pos: usize, message: S, expected: &[TokenType]) -> Self {
        let expected = expected.iter().map(|token| token.friendy_name()).join(", ");
        Self::new(
            pos,
            format!("{}, expected one of: {}", message.to_string(), expected),
        )
    }
}

type RILParserResult<T> = Result<T, RILParserError>;

struct Lexer<'a> {
    src_iter: Peekable<CharIndices<'a>>,
    tokens: Vec<Token>,
}

impl<'a> Lexer<'a> {
    fn lex(input: &str) -> RILParserResult<Vec<Token>> {
        let mut lexer = Lexer {
            src_iter: input.char_indices().peekable(),
            tokens: Vec::new(),
        };
        while let Some((pos, c)) = lexer.src_iter.peek().cloned() {
            let ty = match c {
                ':' => TokenType::Colon,
                '[' => TokenType::LeftBracket,
                ']' => TokenType::RightBracket,
                '{' => TokenType::LeftCurlyBrace,
                '}' => TokenType::RightCurlyBrace,
                '(' => TokenType::LeftParens,
                ')' => TokenType::RightParens,
                ',' => TokenType::Comma,
                '=' => TokenType::Equals,
                '#' => {
                    lexer.skip_line();
                    continue;
                }
                '-' => {
                    lexer.src_iter.next();
                    if lexer.src_iter.next_if(|&(_, c)| c == '>').is_none() {
                        return Err(RILParserError::new(pos, "expected '>' after '-'"));
                    }
                    lexer.tokens.push(Token::new(pos, TokenType::Arrow));
                    continue;
                }
                '%' | '@' => {
                    lexer.src_iter.next();
                    let name = lexer.read_value_ident();
                    let ty = match c {
                        '%' => TokenType::Value(name),
                        '@' => TokenType::GlobalValue(name),
                        _ => unreachable!(),
                    };
                    lexer.tokens.push(Token::new(pos, ty));
                    continue;
                }
                '"' => {
                    let str = lexer.read_string();
                    lexer.tokens.push(Token::new(pos, TokenType::String(str)));
                    continue;
                }
                _ if c.is_whitespace() => {
                    lexer.src_iter.next();
                    continue;
                }
                _ if c.is_ascii_digit() => {
                    lexer.read_int();
                    continue;
                }
                _ if c.is_ascii_alphabetic() => {
                    lexer.read_keyword()?;
                    continue;
                }
                _ => {
                    return Err(RILParserError::new(
                        pos,
                        format!("unexpected character: {}", c),
                    ))
                }
            };
            let token = Token::new(pos, ty);
            lexer.tokens.push(token);
            lexer.src_iter.next();
        }
        Ok(lexer.tokens)
    }

    fn read_string(&mut self) -> String {
        self.src_iter.next();
        let mut str = String::new();
        while let Some(&(_, c)) = self.src_iter.peek() {
            if c == '"' {
                self.src_iter.next();
                break;
            }
            str.push(c);
            self.src_iter.next();
        }
        str
    }

    fn read_value_ident(&mut self) -> String {
        let mut str = String::new();
        while let Some(&(_, c)) = self.src_iter.peek() {
            if !c.is_ascii_digit() && !c.is_ascii_alphabetic() && c != '_' {
                break;
            }
            str.push(c);
            self.src_iter.next();
        }
        str
    }

    fn read_int(&mut self) {
        let pos = self.src_iter.peek().unwrap().0;
        let mut num_string = String::new();
        while let Some((_, c)) = self.src_iter.peek() {
            if !c.is_ascii_digit() {
                break;
            }
            num_string.push(*c);
            self.src_iter.next();
        }
        let num: u32 = num_string.parse().unwrap();
        let token = Token::new(pos, TokenType::Int(num));
        self.tokens.push(token);
    }

    fn read_keyword(&mut self) -> RILParserResult<()> {
        let pos = self.src_iter.peek().unwrap().0;
        let mut word = String::new();
        while let Some(&(_, c)) = self.src_iter.peek() {
            if !c.is_ascii_alphabetic() && c != '_' {
                break;
            }
            word.push(c);
            self.src_iter.next();
        }

        let ty = match word.as_str() {
            "circuit" => TokenType::Circuit,
            "backend_circuit" => TokenType::BackendCircuit,
            "test" => TokenType::Test,
            "test_args" => TokenType::TestArgs,
            "schematic" => TokenType::Schematic,
            "none" => TokenType::None,
            "side" => TokenType::Side,
            "repeater" => TokenType::ComponentType(ComponentType::Repeater),
            "torch" => TokenType::ComponentType(ComponentType::Torch),
            "comparator" => TokenType::ComponentType(ComponentType::Comparator),
            "lamp" => TokenType::ComponentType(ComponentType::Lamp),
            "button" => TokenType::ComponentType(ComponentType::Button),
            "lever" => TokenType::ComponentType(ComponentType::Lever),
            "pressure_plate" => TokenType::ComponentType(ComponentType::PressurePlate),
            "trapdoor" => TokenType::ComponentType(ComponentType::Trapdoor),
            "wire" => TokenType::ComponentType(ComponentType::Wire),
            "constant" => TokenType::ComponentType(ComponentType::Constant),
            "note_block" => TokenType::ComponentType(ComponentType::NoteBlock),
            // Note Block Instruments
            "harp" => TokenType::Instrument(Instrument::Harp),
            "basedrum" => TokenType::Instrument(Instrument::Basedrum),
            "snare" => TokenType::Instrument(Instrument::Snare),
            "hat" => TokenType::Instrument(Instrument::Hat),
            "bass" => TokenType::Instrument(Instrument::Bass),
            "flute" => TokenType::Instrument(Instrument::Flute),
            "bell" => TokenType::Instrument(Instrument::Bell),
            "guitar" => TokenType::Instrument(Instrument::Guitar),
            "chime" => TokenType::Instrument(Instrument::Chime),
            "xylophone" => TokenType::Instrument(Instrument::Xylophone),
            "iron_xylophone" => TokenType::Instrument(Instrument::IronXylophone),
            "cow_bell" => TokenType::Instrument(Instrument::CowBell),
            "didgeridoo" => TokenType::Instrument(Instrument::Didgeridoo),
            "bit" => TokenType::Instrument(Instrument::Bit),
            "banjo" => TokenType::Instrument(Instrument::Banjo),
            "pling" => TokenType::Instrument(Instrument::Pling),
            // Bool
            "false" => TokenType::Bool(false),
            "true" => TokenType::Bool(true),
            // Comparator Mode
            "compare" => TokenType::ComparatorMode(ComparatorMode::Compare),
            "subtract" => TokenType::ComparatorMode(ComparatorMode::Subtract),
            _ => {
                return Err(RILParserError::new(
                    pos,
                    format!("unrecognized keyword: {}", word),
                ))
            }
        };
        self.tokens.push(Token::new(pos, ty));
        Ok(())
    }

    fn skip_line(&mut self) {
        for (_, c) in self.src_iter.by_ref() {
            if c == '\n' {
                break;
            }
        }
    }
}

pub struct RILTest {
    pub name: String,
    pub schematic_path: Option<String>,
    pub graph: CompileGraph,
    pub options: CompilerOptions,
}

#[derive(Default)]
pub struct RILModule {
    // The order of the globals are preserved so the test results can more easily be replaced
    pub globals: IndexMap<String, ast::Global>,
    pub test_args: Option<ast::TestArgs>,
}

impl RILModule {
    pub fn parse_from_string(src: &str) -> RILParserResult<Self> {
        let tokens = Lexer::lex(src)?;
        Parser::parse(tokens)
    }

    pub fn get_graph(&self, circuit: &ast::Circuit) -> CompileGraph {
        let mut graph = CompileGraph::default();
        let mut name_map = FxHashMap::default();
        for component in &circuit.components {
            let node_idx = graph.add_node(CompileNode {
                ty: component.node_ty.clone(),
                state: component.node_state.clone(),
                name: Some(component.name.clone()),
                block: Default::default(),
                is_input: component.node_ty.is_normally_input(),
                is_output: component.node_ty.is_normally_output(),
            });
            name_map.insert(component.name.clone(), node_idx);
        }
        for component in &circuit.components {
            let from = name_map[&component.name];
            for output in &component.outputs {
                graph.add_edge(from, name_map[&output.target], output.link);
            }
        }
        graph
    }

    fn get_test(&self, name: &str, test: &ast::Test) -> RILTest {
        let (graph, schematic_path) = match self.globals.get(&test.input) {
            Some(ast::Global::Circuit(circuit)) => (self.get_graph(circuit), None),
            Some(ast::Global::Schematic(schematic)) => {
                (CompileGraph::default(), Some(schematic.path.clone()))
            }
            Some(_) => panic!("invalid test input"),
            None => panic!("could not find test input with name: {}", test.input),
        };
        let options = test
            .test_args
            .as_ref()
            .or(self.test_args.as_ref().map(|args| &args.args))
            .expect("could not determine test arguments")
            .clone();
        RILTest {
            name: name.to_owned(),
            graph,
            schematic_path,
            options,
        }
    }

    pub fn get_tests(&self) -> Vec<RILTest> {
        self.globals
            .iter()
            .filter_map(|(name, global)| match global {
                ast::Global::Test(test) => Some(self.get_test(name, test)),
                _ => None,
            })
            .collect()
    }

    pub fn compare_test_result(&self, name: &str, result: &CompileGraph) -> bool {
        let Some(ast::Global::Test(test)) = self.globals.get(name) else {
            panic!("could not find test: {}", name);
        };
        let ast::TestResult::Circuit(circuit) = &test.result else {
            todo!("non-circuit test result types")
        };

        let mut expected_map = FxHashMap::default();
        for (idx, component) in circuit.components.iter().enumerate() {
            expected_map.insert(&component.name, idx);
        }

        for node_idx in result.node_indices() {
            if !result.contains_node(node_idx) {
                continue;
            }
            let node = &result[node_idx];
            let name = node
                .name
                .clone()
                .unwrap_or_else(|| node_idx.index().to_string());
            let Some(expected_idx) = expected_map.get(&name) else {
                return false;
            };
            let expected = &circuit.components[*expected_idx];
            if expected.node_ty != node.ty || expected.node_state != node.state {
                return false;
            }
            let outputs: Vec<ast::Output> = result
                .edges(node_idx, Direction::Outgoing)
                .map(|edge| ast::Output {
                    link: *edge.weight(),
                    target: result[edge.target()]
                        .name
                        .clone()
                        .unwrap_or_else(|| edge.target().index().to_string()),
                })
                .collect();
            if outputs != expected.outputs {
                return false;
            }
            expected_map.remove(&name);
        }

        expected_map.is_empty()
    }

    pub fn update_test(&self, src: &mut StringReplacer, name: &str, new_result: &str) {
        let Some(ast::Global::Test(test)) = self.globals.get(name) else {
            panic!("could not find test: {}", name);
        };
        let ast::TestResult::Circuit(circuit) = &test.result else {
            todo!("non-circuit test result types")
        };
        src.replace_range(circuit.src_begin..circuit.src_end, new_result);
    }
}

struct Parser {
    module: RILModule,
    tokens: Peekable<vec::IntoIter<Token>>,
}

impl Parser {
    fn parse(tokens: Vec<Token>) -> RILParserResult<RILModule> {
        let mut parser = Parser {
            module: Default::default(),
            tokens: tokens.into_iter().peekable(),
        };

        loop {
            if parser.tokens.peek().is_none() {
                break;
            }

            let keyword_token = parser.expect_token(&[
                TokenType::Circuit,
                TokenType::BackendCircuit,
                TokenType::Schematic,
                TokenType::Test,
                TokenType::TestArgs,
            ])?;
            match keyword_token.ty {
                TokenType::Circuit => {
                    let circuit = parser.parse_circuit(keyword_token.pos)?;
                    parser
                        .module
                        .globals
                        .insert(circuit.name.clone(), ast::Global::Circuit(circuit));
                }
                TokenType::Schematic => {
                    let schematic = parser.parse_schematic()?;
                    parser
                        .module
                        .globals
                        .insert(schematic.name.clone(), ast::Global::Schematic(schematic));
                }
                TokenType::BackendCircuit => todo!("backend circuit parsing"),
                TokenType::Test => parser.parse_test()?,
                TokenType::TestArgs => parser.parse_test_args()?,
                _ => unreachable!(),
            }
        }

        Ok(parser.module)
    }

    fn parse_schematic(&mut self) -> RILParserResult<ast::Schematic> {
        let name = self
            .expect_token_with(
                |token| matches!(token.ty, TokenType::GlobalValue(_)),
                &[TokenType::GlobalValue(Default::default())],
            )?
            .ty
            .unwrap_global_value();
        let (_, path) = self.expect_string()?;
        Ok(ast::Schematic { name, path })
    }

    fn parse_circuit(&mut self, src_begin: usize) -> RILParserResult<ast::Circuit> {
        let name = self
            .expect_token_with(
                |token| matches!(token.ty, TokenType::GlobalValue(_)),
                &[TokenType::GlobalValue(Default::default())],
            )?
            .ty
            .unwrap_global_value();
        self.expect_token(&[TokenType::LeftCurlyBrace])?;

        let mut components = Vec::new();

        let src_end = loop {
            let value_or_brace = self.expect_token_with(
                |token| matches!(token.ty, TokenType::Value(_) | TokenType::RightCurlyBrace),
                &[
                    TokenType::Value(Default::default()),
                    TokenType::RightCurlyBrace,
                ],
            )?;
            if value_or_brace.ty == TokenType::RightCurlyBrace {
                break value_or_brace.pos + 1;
            } else {
                components.push(self.parse_component(value_or_brace)?);
            }
        };

        Ok(ast::Circuit {
            name,
            components,
            src_begin,
            src_end,
        })
    }

    fn parse_component(&mut self, value: Token) -> RILParserResult<ast::Component> {
        let name = value.ty.unwrap_value();
        self.expect_token(&[TokenType::Equals])?;
        let component_ty = self
            .expect_token_with(
                |token| matches!(token.ty, TokenType::ComponentType(_)),
                &[TokenType::ComponentType(ComponentType::Wire)],
            )?
            .ty
            .unwrap_component_ty();
        let (node_ty, node_state) = match component_ty {
            ComponentType::Repeater => {
                let (delay_token, delay) = self.expect_int()?;
                if !(1..=4).contains(&delay) {
                    return Err(RILParserError::new(
                        delay_token.pos,
                        "repeater delay out of range",
                    ));
                }
                self.expect_token(&[TokenType::Comma])?;
                let (_, prioritized) = self.expect_bool()?;
                self.expect_token(&[TokenType::Comma])?;
                let (_, locked) = self.expect_bool()?;
                self.expect_token(&[TokenType::Comma])?;
                let (_, powered) = self.expect_bool()?;
                let ty = NodeType::Repeater {
                    delay: delay as u8,
                    prioritized,
                };
                (ty, NodeState::repeater(powered, locked))
            }
            ComponentType::Comparator => {
                let mode = self
                    .expect_token_with(
                        |token| matches!(token.ty, TokenType::ComparatorMode(_)),
                        &[TokenType::ComparatorMode(ComparatorMode::Subtract)],
                    )?
                    .ty
                    .unwrap_comparator_mode();
                self.expect_token(&[TokenType::Comma])?;
                let far_input = self.parse_comparator_far_input()?;
                self.expect_token(&[TokenType::Comma])?;
                let (_, prioritized) = self.expect_bool()?;
                self.expect_token(&[TokenType::Comma])?;
                let power = self.expect_power()?;
                let ty = NodeType::Comparator {
                    mode,
                    far_input,
                    prioritized,
                };
                (ty, NodeState::from_power(power))
            }
            ComponentType::Torch
            | ComponentType::Lamp
            | ComponentType::Trapdoor
            | ComponentType::Button
            | ComponentType::Lever
            | ComponentType::PressurePlate => {
                let (_, powered) = self.expect_bool()?;
                (
                    component_ty.simple_node_type(),
                    NodeState::from_powered(powered),
                )
            }
            ComponentType::Wire | ComponentType::Constant => {
                let power = self.expect_power()?;
                (
                    component_ty.simple_node_type(),
                    NodeState::from_power(power),
                )
            }
            ComponentType::NoteBlock => {
                let instrument = self
                    .expect_token_with(
                        |token| matches!(token.ty, TokenType::Instrument(_)),
                        &[TokenType::Instrument(Instrument::Harp)],
                    )?
                    .ty
                    .unwrap_instrument();
                self.expect_token(&[TokenType::Comma])?;
                let (token, note) = self.expect_int()?;
                if note > 24 {
                    return Err(RILParserError::new(
                        token.pos,
                        "note must be between 0 and 24",
                    ));
                }
                self.expect_token(&[TokenType::Comma])?;
                let (_, powered) = self.expect_bool()?;
                let ty = NodeType::NoteBlock {
                    instrument,
                    note: note as u8,
                };
                (ty, NodeState::from_powered(powered))
            }
        };
        Ok(ast::Component {
            name,
            node_ty,
            node_state,
            outputs: self.parse_outputs()?,
        })
    }

    fn parse_comparator_far_input(&mut self) -> RILParserResult<Option<SignalStrength>> {
        let token = self.expect_token_with(
            |token| matches!(token.ty, TokenType::Int(_) | TokenType::None),
            &[TokenType::Int(0), TokenType::None],
        )?;
        if token.ty == TokenType::None {
            return Ok(None);
        }
        SignalStrength::try_from(token.ty.unwrap_int())
            .map(Some)
            .map_err(|error| RILParserError::new(token.pos, error.to_string()))
    }

    fn parse_test(&mut self) -> RILParserResult<()> {
        self.expect_token(&[TokenType::LeftParens])?;
        let input = self
            .expect_token_with(
                |token| matches!(token.ty, TokenType::GlobalValue(_)),
                &[TokenType::GlobalValue(String::new())],
            )?
            .ty
            .unwrap_global_value();
        let parens_or_comma = self.expect_token(&[TokenType::RightParens, TokenType::Comma])?;
        let test_args = match parens_or_comma.ty {
            TokenType::RightParens => None,
            TokenType::Comma => {
                let (_, args) = self.expect_string()?;
                self.expect_token(&[TokenType::RightParens])?;
                Some(CompilerOptions::parse(&args))
            }
            _ => unreachable!(),
        };

        let result_ty = self.expect_token(&[TokenType::Circuit, TokenType::BackendCircuit])?;
        let (name, result) = match result_ty.ty {
            TokenType::Circuit => {
                let circuit = self.parse_circuit(result_ty.pos)?;
                (circuit.name.clone(), ast::TestResult::Circuit(circuit))
            }
            TokenType::BackendCircuit => todo!("backend circuit parsing"),
            _ => unreachable!(),
        };

        self.module.globals.insert(
            name,
            ast::Global::Test(ast::Test {
                input,
                result,
                test_args,
            }),
        );

        Ok(())
    }

    fn parse_test_args(&mut self) -> RILParserResult<()> {
        let (_, args) = self.expect_string()?;
        let args = CompilerOptions::parse(&args);
        self.module.test_args = Some(ast::TestArgs { args });
        Ok(())
    }

    fn parse_outputs(&mut self) -> RILParserResult<Vec<ast::Output>> {
        if self
            .tokens
            .next_if(|token| token.ty == TokenType::Arrow)
            .is_none()
        {
            return Ok(Vec::new());
        }
        self.expect_token(&[TokenType::LeftBracket])?;
        let mut outputs = Vec::new();
        loop {
            outputs.push(self.parse_output()?);
            let comma_or_bracket =
                self.expect_token(&[TokenType::Comma, TokenType::RightBracket])?;
            if comma_or_bracket.ty == TokenType::RightBracket {
                return Ok(outputs);
            }
        }
    }

    fn parse_output(&mut self) -> RILParserResult<ast::Output> {
        let ty = match self.tokens.next_if(|token| token.ty == TokenType::Side) {
            Some(_) => LinkType::Side,
            None => LinkType::Default,
        };
        let target = self
            .expect_token_with(
                |token| matches!(token.ty, TokenType::Value(_)),
                &[TokenType::Side, TokenType::Value(Default::default())],
            )?
            .ty
            .unwrap_value();
        self.expect_token(&[TokenType::Colon])?;
        let (token, weight) = self.expect_int()?;
        let weight = u8::try_from(weight).map_err(|_| {
            RILParserError::new(
                token.pos,
                format!("link weight {weight} is outside 0..=255"),
            )
        })?;
        Ok(ast::Output {
            link: CompileLink::new(ty, weight),
            target,
        })
    }

    fn expect_power(&mut self) -> RILParserResult<SignalStrength> {
        let (token, power) = self.expect_int()?;
        SignalStrength::try_from(power)
            .map_err(|error| RILParserError::new(token.pos, error.to_string()))
    }

    fn expect_int(&mut self) -> RILParserResult<(Token, u32)> {
        let token = self.expect_token_with(
            |token| matches!(token.ty, TokenType::Int(_)),
            &[TokenType::Int(0)],
        )?;
        let val = token.ty.unwrap_int();
        Ok((token, val))
    }

    fn expect_bool(&mut self) -> RILParserResult<(Token, bool)> {
        let token = self.expect_token_with(
            |token| matches!(token.ty, TokenType::Bool(_)),
            &[TokenType::Bool(false)],
        )?;
        let val = token.ty.unwrap_bool();
        Ok((token, val))
    }

    fn expect_string(&mut self) -> RILParserResult<(Token, String)> {
        let token = self.expect_token_with(
            |token| matches!(token.ty, TokenType::String(_)),
            &[TokenType::String(String::new())],
        )?;
        let val = token.ty.clone().unwrap_string();
        Ok((token, val))
    }

    /// Consume one token of any type. The `expected` parameter is only used to create the error message.
    fn expect_any_token(&mut self, expected: &[TokenType]) -> RILParserResult<Token> {
        if let Some(token) = self.tokens.next() {
            Ok(token)
        } else {
            // TODO: Get position of end of file
            Err(RILParserError::new_expected(
                0,
                "reached end of file",
                expected,
            ))
        }
    }

    fn expect_token(&mut self, valid_types: &[TokenType]) -> RILParserResult<Token> {
        let token = self.expect_any_token(valid_types)?;
        if valid_types.contains(&token.ty) {
            Ok(token)
        } else {
            Err(RILParserError::new_expected(
                token.pos,
                format!("found {}", token.ty.friendy_name()),
                valid_types,
            ))
        }
    }

    fn expect_token_with(
        &mut self,
        matcher: fn(&Token) -> bool,
        valid_types: &[TokenType],
    ) -> RILParserResult<Token> {
        let token = self.expect_any_token(valid_types)?;
        if matcher(&token) {
            Ok(token)
        } else {
            Err(RILParserError::new_expected(
                token.pos,
                format!("found {}", token.ty.friendy_name()),
                valid_types,
            ))
        }
    }
}

pub mod ast {
    use crate::{
        compile_graph::{CompileLink, NodeState, NodeType},
        CompilerOptions,
    };

    pub enum Global {
        Circuit(Circuit),
        BackendCircuit(BackendCircuit),
        Test(Test),
        Schematic(Schematic),
    }

    pub struct Schematic {
        pub name: String,
        pub path: String,
    }

    pub struct Circuit {
        pub name: String,
        pub components: Vec<Component>,
        pub src_begin: usize,
        pub src_end: usize,
    }

    pub struct BackendCircuit {
        pub name: String,
        pub backend: String,
        // TODO
    }

    pub enum TestResult {
        Circuit(Circuit),
        BackendCircuit,
    }

    pub struct Test {
        pub input: String,
        pub test_args: Option<CompilerOptions>,
        pub result: TestResult,
    }

    pub struct TestArgs {
        pub args: CompilerOptions,
    }

    #[derive(Clone, PartialEq)]
    pub struct Output {
        pub link: CompileLink,
        pub target: String,
    }

    pub struct Component {
        pub name: String,
        pub node_ty: NodeType,
        pub node_state: NodeState,
        pub outputs: Vec<Output>,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lexer_comparator() {
        use TokenType::*;
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
