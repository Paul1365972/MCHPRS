use criterion::*;
use realtime_channel::RingBuffer;

// Note: Result ist < 200ns on my laptop
fn roundtrip_bench(c: &mut Criterion) {
    let (mut sender1, mut receiver1) = RingBuffer::new(1024);
    let (mut sender2, mut receiver2) = RingBuffer::new(1024);

    std::thread::spawn(move || {
        sender1.send(0u64);
        loop {
            let value = receiver2.recv();
            sender1.send(value + 1);
        }
    });

    c.bench_function("roundtrip", move |b| {
        b.iter(|| {
            let value = receiver1.recv();
            sender2.send(value + 1);
        })
    });
}

criterion_group!(roundtrip, roundtrip_bench);
criterion_main!(roundtrip);
