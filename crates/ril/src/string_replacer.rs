pub struct StringReplacer<'a> {
    src: &'a str,
    read: usize,
    output: String,
}

impl<'a> StringReplacer<'a> {
    pub fn new(src: &'a str) -> Self {
        Self {
            src,
            read: 0,
            output: String::new(),
        }
    }

    pub fn replace_range(&mut self, range: std::ops::Range<usize>, with: &str) {
        assert!(
            range.start >= self.read,
            "Replacements should be done in order {} < {}",
            range.start,
            self.read,
        );
        self.output += &self.src[self.read..range.start];
        self.output += with;
        self.read = range.end;
    }

    pub fn finish(self) -> String {
        self.output + &self.src[self.read..]
    }
}

#[test]
fn test() {
    let mut sr = StringReplacer::new("hello there");

    let hello = sr.src.find("hello").unwrap();
    let her = sr.src.find("her").unwrap();

    sr.replace_range(hello..hello + 5, "bye");
    sr.replace_range(her..her + 3, "elephon");

    assert_eq!(sr.finish(), "bye telephone");
}
