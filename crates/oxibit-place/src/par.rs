//! The one parallel primitive the packer and placer use: a read-only map
//! over an index range on scoped threads. No locks; the closure borrows
//! shared state immutably and the results come back in order.

/// `f(0..n)` in order, on scoped threads when there is enough work.
pub fn par_map<T: Send>(n: usize, f: impl Fn(u32) -> T + Sync) -> Vec<T> {
    const GRAIN: usize = 2048;
    let threads = std::thread::available_parallelism().map_or(1, |t| t.get()).min(n / GRAIN).max(1);
    if threads == 1 {
        return (0..n as u32).map(f).collect();
    }
    let chunk = n.div_ceil(threads);
    let f = &f;
    std::thread::scope(|s| {
        let hs: Vec<_> = (0..threads)
            .map(|t| {
                let (lo, hi) = (t * chunk, ((t + 1) * chunk).min(n));
                s.spawn(move || (lo as u32..hi as u32).map(f).collect::<Vec<T>>())
            })
            .collect();
        hs.into_iter().flat_map(|h| h.join().expect("worker thread")).collect()
    })
}
