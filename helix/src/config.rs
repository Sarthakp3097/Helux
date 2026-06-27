/// Runtime-tunable parameters for the Helix fusion engine.
/// All fields have sensible defaults; override via environment variables
/// or by constructing `EngineConfig` directly in tests.
#[derive(Debug, Clone)]
pub struct EngineConfig {
    /// UDP port for GPS packets (10 Hz)
    pub gps_port: u16,
    /// UDP port for IMU packets (200 Hz)
    pub imu_port: u16,
    /// UDP port for barometer packets (50 Hz)
    pub baro_port: u16,

    /// Complementary-filter blending coefficient.
    ///
    /// Range [0.0, 1.0]:
    ///   0.0 → trust only IMU integration (pure dead-reckoning)
    ///   1.0 → trust only absolute altitude (GPS / baro)
    ///
    /// Recommended operating points:
    ///   0.98 — slow-moving platforms (weather balloons, buoys)
    ///   0.85 — general-purpose default
    ///   0.60 — fast-maneuvering platforms (racing drones, rockets)
    pub filter_alpha: f32,

    /// Crossbeam channel capacity shared by all sensor producers.
    /// Bounded at this size to apply backpressure when fusion falls behind.
    pub channel_depth: usize,

    /// Minimum interval (µs) between consecutive state-estimate log lines.
    /// Set to 0 to log every cycle.
    pub log_throttle_us: u64,
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self {
            gps_port: 5001,
            imu_port: 5002,
            baro_port: 5003,
            filter_alpha: 0.85,
            channel_depth: 1024,
            log_throttle_us: 2_000, // ~500 Hz log ceiling
        }
    }
}

impl EngineConfig {
    /// Construct from environment variables, falling back to defaults.
    pub fn from_env() -> Self {
        let base = Self::default();
        Self {
            gps_port: env_u16("HELIX_GPS_PORT", base.gps_port),
            imu_port: env_u16("HELIX_IMU_PORT", base.imu_port),
            baro_port: env_u16("HELIX_BARO_PORT", base.baro_port),
            filter_alpha: env_f32("HELIX_ALPHA", base.filter_alpha),
            channel_depth: env_usize("HELIX_CHANNEL_DEPTH", base.channel_depth),
            log_throttle_us: env_u64("HELIX_LOG_THROTTLE_US", base.log_throttle_us),
        }
    }
}

fn env_u16(key: &str, fallback: u16) -> u16 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(fallback)
}

fn env_f32(key: &str, fallback: f32) -> f32 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(fallback)
}

fn env_usize(key: &str, fallback: usize) -> usize {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(fallback)
}

fn env_u64(key: &str, fallback: u64) -> u64 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(fallback)
}
