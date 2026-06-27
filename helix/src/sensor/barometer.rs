/// Barometer packet received at ~50 Hz over UDP port 5003.
///
/// Wire layout: little-endian bincode — exactly 16 bytes.
/// Field order matches the Python simulator's `struct.pack('<Qff')`.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
#[repr(C)]
pub struct BaroPacket {
    /// Microseconds since Unix epoch.
    pub timestamp_us: u64,
    /// Absolute static pressure in Pascals.
    pub pressure_pa: f32,
    /// Ambient temperature in Celsius (used for altitude correction via ISA model).
    pub temp_c: f32,
}

/// Expected on-wire size in bytes.
pub const BARO_WIRE_BYTES: usize = 16;

/// International Standard Atmosphere (ISA) barometric altitude formula.
///
/// Converts a pressure/temperature pair into an estimated altitude above MSL.
/// Valid for the troposphere (0 – 11 km). Outside that range the ISA lapse
/// rate no longer holds and the result degrades gracefully (monotonic but
/// non-linear error).
///
/// # Arguments
/// * `pressure_pa`  — measured static pressure in Pascals
/// * `temp_c`       — local temperature in Celsius
///
/// # Returns
/// Altitude in metres above mean sea level.
pub fn barometric_altitude(pressure_pa: f32, temp_c: f32) -> f32 {
    /// Sea-level standard pressure (Pa).
    const SEA_LEVEL_PA: f32 = 101_325.0;
    /// Temperature lapse rate (K/m) — troposphere.
    const LAPSE_RATE_K_PER_M: f32 = 0.0065;
    /// Universal gas constant (J/(mol·K)).
    const GAS_CONSTANT_R: f32 = 8.314;
    /// Standard gravity (m/s²).
    const GRAVITY_G: f32 = 9.80665;
    /// Molar mass of dry air (kg/mol).
    const MOLAR_MASS_AIR: f32 = 0.0289644;

    let temp_k = temp_c + 273.15;
    let exponent = GAS_CONSTANT_R * LAPSE_RATE_K_PER_M / (GRAVITY_G * MOLAR_MASS_AIR);
    (temp_k / LAPSE_RATE_K_PER_M)
        * (1.0 - (pressure_pa / SEA_LEVEL_PA).powf(exponent))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sea_level_pressure_gives_zero_altitude() {
        let alt = barometric_altitude(101_325.0, 15.0);
        assert!(alt.abs() < 1.0, "expected ~0 m, got {alt:.2} m");
    }

    #[test]
    fn standard_cruising_altitude() {
        // ~35 000 ft ≈ 10 668 m, standard pressure ~26 500 Pa, ISA temp ~−50 °C
        let alt = barometric_altitude(26_500.0, -50.0);
        assert!((alt - 10_668.0).abs() < 200.0, "expected ~10668 m, got {alt:.1} m");
    }
}
