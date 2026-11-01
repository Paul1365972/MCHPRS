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

pub fn rate(per_second: f64) -> String {
    let (scaled, prefix) = [(1e9, " G"), (1e6, " M"), (1e3, " k")]
        .into_iter()
        .find(|&(scale, _)| per_second >= scale)
        .map_or((per_second, ""), |(scale, prefix)| {
            (per_second / scale, prefix)
        });
    let decimals = match scaled {
        100.0.. => 0,
        10.0.. => 1,
        _ => 2,
    };
    format!("{scaled:.decimals$}{prefix}")
}
