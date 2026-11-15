//! The standard normal CDF, on the platform C library's `erf` (the same
//! function the reference analysis used, so fits agree to the last bit the
//! grid search can see).

unsafe extern "C" {
    safe fn erf(x: f64) -> f64;
}

/// Standard normal CDF.
pub fn phi(x: f64) -> f64 {
    0.5 * (1.0 + erf(x / std::f64::consts::SQRT_2))
}

#[cfg(test)]
mod tests {
    use super::phi;

    #[test]
    fn known_values() {
        assert_eq!(phi(0.0), 0.5);
        assert!((phi(1.0) - 0.841_344_746_068_542_9).abs() < 1e-15);
        assert!((phi(-1.959_963_984_540_054) - 0.025).abs() < 1e-15);
        assert_eq!(phi(40.0), 1.0);
        assert_eq!(phi(-40.0), 0.0);
    }
}
