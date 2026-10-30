//! Profiling for oxibit.
//!
//! Annotate with [`hot`] (functions) and [`hot_block!`] (regions); both
//! compile to nothing unless the annotated crate is built with its
//! `profile` feature, which pulls in `tracing` and this crate's
//! collectors. A binary calls [`init`] once and keeps the guard alive;
//! `OXIBIT_PROFILE` picks the outputs, comma-separated:
//!
//! - `flame=out.svg`: a flame graph of span time (`out.svg.folded` keeps
//!   the folded stacks).
//! - `chrome=out.json`: a timeline for Perfetto (ui.perfetto.dev) or
//!   chrome://tracing.
//! - `table`: per-span count, total, self time and percentiles on stderr
//!   at exit.
//!
//! For time inside code without spans (inner loops), sample instead:
//! `cargo build --profile profiling` keeps line tables and symbols, then
//! `samply record ./target/profiling/...` (macOS and Linux, no root on
//! macOS) opens the Firefox Profiler; on Linux `perf record -g` works too.
//!
//! A crate opts in with
//!
//! ```toml
//! [features]
//! profile = ["dep:tracing", "oxibit-prof/profile"]
//! [dependencies]
//! oxibit-prof.workspace = true
//! tracing = { workspace = true, optional = true }
//! ```

pub use oxibit_prof_macros::hot;

/// A span around a block, only with the calling crate's `profile` feature.
///
/// ```ignore
/// hot_block!("json.cells", { ... });
/// hot_block!("route.net", [net = %name], { ... });
/// ```
#[macro_export]
macro_rules! hot_block {
    ($name:literal, $body:block) => {{
        #[cfg(feature = "profile")]
        let _hot_span = ::tracing::info_span!($name).entered();
        $body
    }};
    ($name:literal, [$($field:tt)+], $body:block) => {{
        #[cfg(feature = "profile")]
        let _hot_span = ::tracing::info_span!($name, $($field)+).entered();
        $body
    }};
}

/// A span from here to the end of the enclosing block, only with the
/// calling crate's `profile` feature. For a loop body:
///
/// ```ignore
/// for iter in 1..=n {
///     hot_scope!("route.iter", iter);
///     ...
/// }
/// ```
#[macro_export]
macro_rules! hot_scope {
    ($name:literal) => {
        #[cfg(feature = "profile")]
        let _hot_scope = ::tracing::info_span!($name).entered();
    };
    ($name:literal, $($field:tt)+) => {
        #[cfg(feature = "profile")]
        let _hot_scope = ::tracing::info_span!($name, $($field)+).entered();
    };
}

/// Set a field declared `tracing::field::Empty` on the current span.
#[macro_export]
macro_rules! hot_record {
    ($field:ident, $value:expr) => {{
        #[cfg(feature = "profile")]
        ::tracing::Span::current().record(stringify!($field), $value);
    }};
}

/// Keeps the collectors alive; dropping it writes the outputs.
#[must_use = "profiles are written when the guard drops"]
pub struct Guard {
    #[cfg(feature = "profile")]
    inner: Option<imp::Inner>,
}

impl Drop for Guard {
    fn drop(&mut self) {
        #[cfg(feature = "profile")]
        if let Some(i) = self.inner.take() {
            i.finish();
        }
    }
}

/// Install the collectors `OXIBIT_PROFILE` asks for.
pub fn init() -> Guard {
    #[cfg(feature = "profile")]
    {
        Guard { inner: imp::init() }
    }
    #[cfg(not(feature = "profile"))]
    {
        if std::env::var_os("OXIBIT_PROFILE").is_some() {
            eprintln!("oxibit-prof: OXIBIT_PROFILE is set but this binary was built without the `profile` feature");
        }
        Guard {}
    }
}

#[cfg(feature = "profile")]
mod imp {
    use std::collections::HashMap;
    use std::fs::File;
    use std::io::{BufWriter, Write};
    use std::path::PathBuf;
    use std::sync::{Arc, Mutex};
    use std::time::Instant;

    use tracing::{Subscriber, span};
    use tracing_subscriber::layer::{Context, Layer, SubscriberExt};
    use tracing_subscriber::registry::LookupSpan;

    pub struct Inner {
        flame: Option<(PathBuf, tracing_flame::FlushGuard<BufWriter<File>>)>,
        chrome: Option<(PathBuf, tracing_chrome::FlushGuard)>,
        table: Option<Arc<Mutex<HashMap<String, Stat>>>>,
    }

    pub fn init() -> Option<Inner> {
        let spec = std::env::var("OXIBIT_PROFILE").ok()?;
        let (mut flame_p, mut chrome_p, mut table) = (None, None, false);
        for part in spec.split(',').map(str::trim).filter(|s| !s.is_empty()) {
            match part.split_once('=') {
                Some(("flame", p)) => flame_p = Some(PathBuf::from(p)),
                Some(("chrome", p)) => chrome_p = Some(PathBuf::from(p)),
                None if part == "table" => table = true,
                _ => eprintln!("oxibit-prof: ignoring {part:?} (flame=FILE.svg, chrome=FILE.json, table)"),
            }
        }
        let mut flame_guard = None;
        let flame = flame_p.and_then(|svg| {
            let folded = folded_path(&svg);
            match tracing_flame::FlameLayer::with_file(&folded) {
                Ok((layer, g)) => {
                    flame_guard = Some((svg, g));
                    Some(layer.with_threads_collapsed(true).with_file_and_line(false))
                }
                Err(e) => {
                    eprintln!("oxibit-prof: {}: {e}", folded.display());
                    None
                }
            }
        });
        let mut chrome_guard = None;
        let chrome = chrome_p.map(|p| {
            let (layer, g) = tracing_chrome::ChromeLayerBuilder::new().file(&p).include_args(true).build();
            chrome_guard = Some((p, g));
            layer
        });
        let stats = table.then(|| Arc::new(Mutex::new(HashMap::new())));
        let table_layer = stats.clone().map(|stats| TableLayer { stats });
        let sub = tracing_subscriber::registry().with(flame).with(chrome).with(table_layer);
        if tracing::subscriber::set_global_default(sub).is_err() {
            eprintln!("oxibit-prof: a tracing subscriber is already installed");
            return None;
        }
        Some(Inner { flame: flame_guard, chrome: chrome_guard, table: stats })
    }

    fn folded_path(svg: &std::path::Path) -> PathBuf {
        let mut s = svg.as_os_str().to_owned();
        s.push(".folded");
        PathBuf::from(s)
    }

    impl Inner {
        pub fn finish(self) {
            if let Some((p, g)) = self.chrome {
                drop(g);
                eprintln!("oxibit-prof: timeline {}", p.display());
            }
            if let Some((svg, g)) = self.flame {
                if let Err(e) = g.flush() {
                    eprintln!("oxibit-prof: flush: {e}");
                }
                drop(g);
                let folded = folded_path(&svg);
                let mut opts = inferno::flamegraph::Options::default();
                opts.title = "oxibit".into();
                opts.count_name = "ns".into();
                let r = File::create(&svg).map_err(|e| e.to_string()).and_then(|f| {
                    inferno::flamegraph::from_files(&mut opts, &[folded.clone()], BufWriter::new(f)).map_err(|e| e.to_string())
                });
                match r {
                    Ok(()) => eprintln!("oxibit-prof: flame graph {}", svg.display()),
                    Err(e) => eprintln!("oxibit-prof: flame graph {}: {e}", svg.display()),
                }
            }
            if let Some(stats) = self.table {
                print_table(&stats.lock().unwrap());
            }
        }
    }

    /// Per span name; times in ns. `hist[k]` counts spans of busy time
    /// in [2^(k-1), 2^k).
    pub struct Stat {
        count: u64,
        total: u64,
        self_ns: u64,
        max: u64,
        hist: [u64; 64],
    }

    impl Default for Stat {
        fn default() -> Self {
            Stat { count: 0, total: 0, self_ns: 0, max: 0, hist: [0; 64] }
        }
    }

    impl Stat {
        /// Upper bound of the bucket holding quantile `q`.
        fn quantile(&self, q: f64) -> u64 {
            let want = (self.count as f64 * q).ceil().max(1.0) as u64;
            let mut seen = 0;
            for (k, n) in self.hist.iter().enumerate() {
                seen += n;
                if seen >= want {
                    return 1u64 << k;
                }
            }
            self.max
        }
    }

    fn ms(ns: u64) -> String {
        format!("{:.3}", ns as f64 / 1e6)
    }

    fn print_table(stats: &HashMap<String, Stat>) {
        let mut rows: Vec<_> = stats.iter().collect();
        rows.sort_by(|a, b| b.1.self_ns.cmp(&a.1.self_ns));
        let mut e = std::io::stderr().lock();
        let _ = writeln!(e, "{:<44} {:>10} {:>12} {:>12} {:>10} {:>10} {:>10}", "span", "count", "total_ms", "self_ms", "p50<=ms", "p99<=ms", "max_ms");
        for (name, s) in rows {
            let _ = writeln!(
                e,
                "{:<44} {:>10} {:>12} {:>12} {:>10} {:>10} {:>10}",
                name,
                s.count,
                ms(s.total),
                ms(s.self_ns),
                ms(s.quantile(0.5)),
                ms(s.quantile(0.99)),
                ms(s.max)
            );
        }
    }

    struct Timing {
        busy: u64,
        child: u64,
        entered: Option<Instant>,
    }

    struct TableLayer {
        stats: Arc<Mutex<HashMap<String, Stat>>>,
    }

    impl<S: Subscriber + for<'a> LookupSpan<'a>> Layer<S> for TableLayer {
        fn on_new_span(&self, _a: &span::Attributes<'_>, id: &span::Id, ctx: Context<'_, S>) {
            if let Some(s) = ctx.span(id) {
                s.extensions_mut().insert(Timing { busy: 0, child: 0, entered: None });
            }
        }
        fn on_enter(&self, id: &span::Id, ctx: Context<'_, S>) {
            if let Some(s) = ctx.span(id) {
                if let Some(t) = s.extensions_mut().get_mut::<Timing>() {
                    t.entered = Some(Instant::now());
                }
            }
        }
        fn on_exit(&self, id: &span::Id, ctx: Context<'_, S>) {
            let Some(s) = ctx.span(id) else { return };
            let d = match s.extensions_mut().get_mut::<Timing>() {
                Some(t) => {
                    let d = t.entered.take().map(|e| e.elapsed().as_nanos() as u64).unwrap_or(0);
                    t.busy += d;
                    d
                }
                None => return,
            };
            if let Some(p) = s.parent() {
                if let Some(t) = p.extensions_mut().get_mut::<Timing>() {
                    t.child += d;
                }
            }
        }
        fn on_close(&self, id: span::Id, ctx: Context<'_, S>) {
            let Some(s) = ctx.span(&id) else { return };
            let ext = s.extensions();
            let Some(t) = ext.get::<Timing>() else { return };
            let mut st = self.stats.lock().unwrap();
            // "module::name": the last segment of the span's module path.
            let m = s.metadata();
            let module = m.target().rsplit("::").next().unwrap_or("");
            let e = st.entry(format!("{module}::{}", m.name())).or_default();
            e.count += 1;
            e.total += t.busy;
            e.self_ns += t.busy.saturating_sub(t.child);
            e.max = e.max.max(t.busy);
            e.hist[((64 - t.busy.leading_zeros()) as usize).min(63)] += 1;
        }
    }
}
