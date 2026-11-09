pub fn grouped(count: u64) -> String {
    let digits = count.to_string();
    let mut grouped = String::with_capacity(digits.len() + digits.len() / 3);
    for (index, digit) in digits.chars().enumerate() {
        if index > 0 && (digits.len() - index).is_multiple_of(3) {
            grouped.push(',');
        }
        grouped.push(digit);
    }
    grouped
}

pub fn counted(count: u64, noun: &str) -> String {
    let plural = if count == 1 { "" } else { "s" };
    format!("{} {noun}{plural}", grouped(count))
}

pub fn change(change: i64, noun: &str) -> String {
    let sign = if change < 0 { '-' } else { '+' };
    format!("{sign}{}", counted(change.unsigned_abs(), noun))
}
