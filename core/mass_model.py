"""
core/mass_model.py
==================
Mass breakdown model for multirotor drone sizing.

This module computes the four canonical mass quantities used throughout
the sizing tool, starting from the two user-supplied input masses and the
battery mass determined by the battery model.

Mass Definitions
----------------
The four masses are strictly defined as follows:

    m_struct            Structural mass — airframe, motors, ESCs, electronics,
                        wiring. Excludes battery and payload.
                        Source: user input (drone.m_struct_kg)

    m_payload           Payload mass — inspection sensors, gimbal, cameras.
                        Excludes battery.
                        Source: user input (drone.m_payload_kg)

    m_bat               Battery mass — optimal value computed by battery_model.
                        Source: computed (BatteryResult.m_bat_optimal_kg)

    m_charge_sans_bat   Drone with payload, without battery.
                        = m_struct + m_payload

    m_vol_sans_charge   Drone ready to fly without payload (ferry configuration).
                        = m_struct + m_bat

    MTOW                Maximum Take-Off Weight — full operational mass.
                        = m_struct + m_bat + m_payload

Dependency
----------
This module depends on battery_model.py for the battery mass value.
The call sequence must be:

    1. battery_model.compute_battery(...)  -> BatteryResult
    2. mass_model.compute_masses(...)      -> MassBreakdown

Usage:
    from core.battery_model import compute_battery, BatteryResult
    from core.mass_model import compute_masses

    battery = compute_battery(m_struct, m_payload, n_rotors_eff, diameter_m, phys)
    masses  = compute_masses(m_struct, m_payload, battery.m_bat_optimal_kg)
"""

from typing import NamedTuple


# ---------------------------------------------------------------------------
# Result container
# ---------------------------------------------------------------------------

class MassBreakdown(NamedTuple):
    """Complete mass breakdown for a multirotor drone configuration.

    All masses are in kilograms.

    Attributes:
        m_struct_kg (float): Structural mass — airframe, motors, ESCs,
                             electronics. No battery, no payload.
        m_payload_kg (float): Payload mass — sensors, gimbal, cameras.
                              No battery.
        m_bat_kg (float): Battery mass — optimal value from battery model.
        m_charge_sans_bat_kg (float): Drone with payload, without battery.
                                      = m_struct + m_payload.
        m_vol_sans_charge_kg (float): Drone ready to fly, without payload.
                                      = m_struct + m_bat.
        mtow_kg (float): Maximum Take-Off Weight — full operational mass.
                         = m_struct + m_bat + m_payload.
    """
    m_struct_kg: float
    m_payload_kg: float
    m_bat_kg: float
    m_charge_sans_bat_kg: float
    m_vol_sans_charge_kg: float
    mtow_kg: float


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def compute_masses(
    m_struct_kg: float,
    m_payload_kg: float,
    m_bat_kg: float
) -> MassBreakdown:
    """Compute the complete drone mass breakdown from the three base masses.

    Derives all composite mass quantities from the two input masses and the
    battery mass computed by the battery model. Validates physical consistency
    before returning results.

    Args:
        m_struct_kg (float): Structural mass in kilograms. Must be strictly
                             positive. Represents the airframe, motors, ESCs,
                             and onboard electronics, excluding battery and
                             payload.
        m_payload_kg (float): Payload mass in kilograms. Must be >= 0.
                              Represents mission-specific equipment (sensors,
                              gimbal, cameras).
        m_bat_kg (float): Battery mass in kilograms. Must be strictly positive.
                          This value is the optimal battery mass as returned by
                          battery_model.compute_battery().

    Returns:
        MassBreakdown: Named tuple with all six mass quantities:
            - m_struct_kg
            - m_payload_kg
            - m_bat_kg
            - m_charge_sans_bat_kg  (= m_struct + m_payload)
            - m_vol_sans_charge_kg  (= m_struct + m_bat)
            - mtow_kg               (= m_struct + m_bat + m_payload)

    Raises:
        ValueError: If m_struct_kg <= 0, m_payload_kg < 0, or m_bat_kg <= 0.

    Example:
        >>> masses = compute_masses(
        ...     m_struct_kg=7.0,
        ...     m_payload_kg=2.0,
        ...     m_bat_kg=3.6
        ... )
        >>> masses.mtow_kg
        12.6
        >>> masses.m_vol_sans_charge_kg
        10.6
        >>> masses.m_charge_sans_bat_kg
        9.0
    """
    # --- Input validation ---
    if m_struct_kg <= 0:
        raise ValueError(
            f"m_struct_kg must be strictly positive, got {m_struct_kg}."
        )
    if m_payload_kg < 0:
        raise ValueError(
            f"m_payload_kg must be >= 0, got {m_payload_kg}."
        )
    if m_bat_kg <= 0:
        raise ValueError(
            f"m_bat_kg must be strictly positive, got {m_bat_kg}."
        )

    # --- Derived masses ---
    m_charge_sans_bat = m_struct_kg + m_payload_kg
    m_vol_sans_charge = m_struct_kg + m_bat_kg
    mtow = m_struct_kg + m_bat_kg + m_payload_kg

    return MassBreakdown(
        m_struct_kg=m_struct_kg,
        m_payload_kg=m_payload_kg,
        m_bat_kg=m_bat_kg,
        m_charge_sans_bat_kg=m_charge_sans_bat,
        m_vol_sans_charge_kg=m_vol_sans_charge,
        mtow_kg=mtow
    )


def format_mass_summary(masses: MassBreakdown) -> str:
    """Format the mass breakdown as a human-readable summary string.

    Intended for logging, CLI output, and report generation.

    Args:
        masses (MassBreakdown): Computed mass breakdown as returned by
                                compute_masses().

    Returns:
        str: Multi-line formatted summary of all six mass quantities.

    Example:
        >>> print(format_mass_summary(masses))
        Mass Breakdown
        --------------
        Structural mass (m_struct)          :   7.000 kg
        Battery mass (m_bat)                :   3.600 kg
        Payload mass (m_payload)            :   2.000 kg
        --------------
        With payload, no battery            :   9.000 kg
        Ferry config (no payload)           :  10.600 kg
        MTOW (full operational)             :  12.600 kg
    """
    lines = [
        "Mass Breakdown",
        "-" * 46,
        f"{'Structural mass (m_struct)':<40}: {masses.m_struct_kg:>7.3f} kg",
        f"{'Battery mass (m_bat)':<40}: {masses.m_bat_kg:>7.3f} kg",
        f"{'Payload mass (m_payload)':<40}: {masses.m_payload_kg:>7.3f} kg",
        "-" * 46,
        f"{'With payload, no battery':<40}: {masses.m_charge_sans_bat_kg:>7.3f} kg",
        f"{'Ferry config (no payload)':<40}: {masses.m_vol_sans_charge_kg:>7.3f} kg",
        f"{'MTOW (full operational)':<40}: {masses.mtow_kg:>7.3f} kg",
    ]
    return "\n".join(lines)
