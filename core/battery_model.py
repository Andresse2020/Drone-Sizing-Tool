"""
core/battery_model.py
=====================
Battery sizing and endurance model for multirotor drone design.

This module computes:
- The effective battery voltage from cell count
- The optimal battery mass that maximizes flight endurance
- The maximum achievable endurance at the optimal battery mass
- The endurance as a function of battery mass (used for curve generation)

Physical Model
--------------
The endurance model is derived from the actuator disk theory combined with
the specific energy of the battery pack.

Hover power (from actuator disk theory):
    P_hover = (MTOW * g)^(3/2) / sqrt(2 * rho * A_disk * N_rotors) / eta_global

where:
    MTOW        = m_struct + m_bat + m_payload  [kg]
    A_disk      = pi * (D/2)^2                  [m^2] per rotor
    N_rotors    = number of independent rotor disks
    eta_global  = global drivetrain efficiency

For coaxial configurations (x8), the coaxial correction is applied to the
effective number of rotor disks:
    N_eff = N_motors * thrust_loss_factor

Available energy from battery:
    E_avail = m_bat * e_sp * DoD   [Wh]

Endurance:
    t = E_avail / P_hover          [hours]  ->  converted to minutes

Optimal battery mass
--------------------
The endurance function t(m_bat) has a single maximum. As m_bat increases:
- Available energy increases linearly
- Hover power increases (heavier MTOW) — but sublinearly

The optimum is found numerically over the physically constrained range:
    [m_bat_min, m_bat_max] defined by mass_fraction bounds in the config.

Usage:
    from core.battery_model import compute_battery

    result = compute_battery(
        m_struct=7.0,
        m_payload=2.0,
        n_rotors_eff=8.0,
        diameter_m=0.4064,
        phys=physical_config
    )
"""

import math
from typing import NamedTuple
import numpy as np


# ---------------------------------------------------------------------------
# Result container
# ---------------------------------------------------------------------------

class BatteryResult(NamedTuple):
    """Results of the battery sizing computation.

    Attributes:
        voltage_V (float): Effective battery pack voltage in Volts.
        m_bat_optimal_kg (float): Battery mass that maximizes endurance [kg].
        endurance_max_min (float): Maximum endurance at optimal battery mass
                                   [minutes].
        m_bat_range_kg (np.ndarray): Array of battery mass values used for
                                     curve generation [kg].
        endurance_curve_min (np.ndarray): Endurance values corresponding to
                                          m_bat_range_kg [minutes].
    """
    voltage_V: float
    m_bat_optimal_kg: float
    endurance_max_min: float
    m_bat_range_kg: "np.ndarray"
    endurance_curve_min: "np.ndarray"


# ---------------------------------------------------------------------------
# Core physical functions
# ---------------------------------------------------------------------------

def compute_effective_voltage(cell_count_S: int, cell_voltage_V: float) -> float:
    """Compute the effective battery pack voltage from cell count.

    The nominal voltage of a LiPo pack is the product of the number of cells
    in series and the nominal voltage per cell (3.7V for standard LiPo).

    Args:
        cell_count_S (int): Number of cells in series (e.g. 6 for 6S, 12 for 12S).
        cell_voltage_V (float): Nominal voltage per cell in Volts. Typically 3.7V.

    Returns:
        float: Effective pack voltage in Volts.

    Raises:
        ValueError: If cell_count_S or cell_voltage_V is not positive.

    Example:
        >>> compute_effective_voltage(12, 3.7)
        44.4
    """
    if cell_count_S <= 0:
        raise ValueError(
            f"cell_count_S must be a positive integer, got {cell_count_S}."
        )
    if cell_voltage_V <= 0:
        raise ValueError(
            f"cell_voltage_V must be strictly positive, got {cell_voltage_V}."
        )
    return float(cell_count_S) * cell_voltage_V


def compute_disk_area(diameter_m: float) -> float:
    """Compute the rotor disk area from the propeller diameter.

    Args:
        diameter_m (float): Propeller diameter in meters.

    Returns:
        float: Rotor disk area in square meters.

    Raises:
        ValueError: If diameter_m is not strictly positive.
    """
    if diameter_m <= 0:
        raise ValueError(
            f"diameter_m must be strictly positive, got {diameter_m}."
        )
    return math.pi * (diameter_m / 2.0) ** 2


def compute_hover_power(
    mtow_kg: float,
    n_rotors_eff: float,
    disk_area_m2: float,
    rho: float,
    g: float,
    eta_global: float
) -> float:
    """Compute the mechanical power required for steady hover flight.

    Based on the actuator disk theory (momentum theory), the ideal induced
    power required to hover is:

        P_ideal = (MTOW * g)^(3/2) / sqrt(2 * rho * A_total)

    where A_total = n_rotors_eff * disk_area_m2 is the total effective
    rotor disk area.

    The actual shaft power accounts for drivetrain losses:

        P_hover = P_ideal / eta_global

    For coaxial configurations, n_rotors_eff already embeds the thrust loss
    correction (see compute_battery for details).

    Args:
        mtow_kg (float): Total take-off mass in kilograms.
        n_rotors_eff (float): Effective number of rotor disks. For flat
            configurations this equals the number of motors. For coaxial (x8),
            this is adjusted by the thrust loss factor.
        disk_area_m2 (float): Area of a single rotor disk in square meters.
        rho (float): Air density in kg/m^3.
        g (float): Gravitational acceleration in m/s^2.
        eta_global (float): Global drivetrain efficiency in (0, 1).

    Returns:
        float: Required hover power in Watts.

    Raises:
        ValueError: If any argument is non-positive.

    Physical assumptions:
        - Steady, level hover (no translational velocity).
        - Uniform inflow velocity across the rotor disk.
        - Constant drivetrain efficiency across the operating range.
    """
    if mtow_kg <= 0:
        raise ValueError(f"mtow_kg must be strictly positive, got {mtow_kg}.")
    if n_rotors_eff <= 0:
        raise ValueError(
            f"n_rotors_eff must be strictly positive, got {n_rotors_eff}."
        )
    if disk_area_m2 <= 0:
        raise ValueError(
            f"disk_area_m2 must be strictly positive, got {disk_area_m2}."
        )
    if rho <= 0:
        raise ValueError(f"rho must be strictly positive, got {rho}.")
    if g <= 0:
        raise ValueError(f"g must be strictly positive, got {g}.")
    if not (0 < eta_global < 1):
        raise ValueError(
            f"eta_global must be in (0, 1), got {eta_global}."
        )

    total_disk_area = n_rotors_eff * disk_area_m2
    weight = mtow_kg * g
    p_ideal = (weight ** 1.5) / math.sqrt(2.0 * rho * total_disk_area)
    return p_ideal / eta_global


def compute_endurance_minutes(
    m_bat_kg: float,
    m_struct_kg: float,
    m_payload_kg: float,
    n_rotors_eff: float,
    disk_area_m2: float,
    energy_density_Wh_kg: float,
    depth_of_discharge: float,
    rho: float,
    g: float,
    eta_global: float
) -> float:
    """Compute the flight endurance in minutes for a given battery mass.

    Endurance is defined as the ratio of available battery energy to hover
    power consumption:

        t = (m_bat * e_sp * DoD) / P_hover(MTOW)   [hours]
        t_min = t * 60                               [minutes]

    where MTOW = m_struct + m_bat + m_payload.

    Args:
        m_bat_kg (float): Battery mass in kilograms.
        m_struct_kg (float): Structural mass (airframe, motors, electronics)
                             in kilograms. Excludes battery and payload.
        m_payload_kg (float): Payload mass in kilograms.
        n_rotors_eff (float): Effective number of rotor disks (coaxial-adjusted).
        disk_area_m2 (float): Area of a single rotor disk in square meters.
        energy_density_Wh_kg (float): Battery specific energy in Wh/kg.
        depth_of_discharge (float): Usable fraction of total battery energy,
                                    in range (0, 1).
        rho (float): Air density in kg/m^3.
        g (float): Gravitational acceleration in m/s^2.
        eta_global (float): Global drivetrain efficiency in (0, 1).

    Returns:
        float: Estimated flight endurance in minutes. Returns 0.0 if
               m_bat_kg is zero or negative.

    Physical assumptions:
        - Hover flight only (worst-case power consumption).
        - Constant power draw throughout the flight.
        - Battery energy fully available within the DoD limit.
    """
    if m_bat_kg <= 0:
        return 0.0

    mtow = m_struct_kg + m_bat_kg + m_payload_kg
    p_hover = compute_hover_power(
        mtow_kg=mtow,
        n_rotors_eff=n_rotors_eff,
        disk_area_m2=disk_area_m2,
        rho=rho,
        g=g,
        eta_global=eta_global
    )

    e_avail_Wh = m_bat_kg * energy_density_Wh_kg * depth_of_discharge
    endurance_hours = e_avail_Wh / p_hover
    return endurance_hours * 60.0


# ---------------------------------------------------------------------------
# Optimal battery mass solver
# ---------------------------------------------------------------------------

def _find_optimal_battery_mass(
    m_struct_kg: float,
    m_payload_kg: float,
    n_rotors_eff: float,
    disk_area_m2: float,
    energy_density_Wh_kg: float,
    depth_of_discharge: float,
    rho: float,
    g: float,
    eta_global: float,
    m_bat_min_kg: float,
    m_bat_max_kg: float,
    resolution: int = 1000
) -> tuple[float, float]:
    """Find the battery mass that maximizes endurance via numerical search.

    Evaluates the endurance function over a uniformly spaced grid of battery
    mass values between m_bat_min_kg and m_bat_max_kg, and returns the mass
    at which endurance is maximum.

    The endurance function t(m_bat) is unimodal within physical bounds:
    it rises as energy increases and falls once the added weight penalty
    dominates. The numerical resolution of 1000 points provides sub-gram
    accuracy for typical drone battery mass ranges.

    Args:
        m_struct_kg (float): Structural mass in kilograms.
        m_payload_kg (float): Payload mass in kilograms.
        n_rotors_eff (float): Effective number of rotor disks.
        disk_area_m2 (float): Area of a single rotor disk in square meters.
        energy_density_Wh_kg (float): Battery specific energy in Wh/kg.
        depth_of_discharge (float): Usable fraction of total battery energy.
        rho (float): Air density in kg/m^3.
        g (float): Gravitational acceleration in m/s^2.
        eta_global (float): Global drivetrain efficiency.
        m_bat_min_kg (float): Minimum battery mass to consider in kg.
        m_bat_max_kg (float): Maximum battery mass to consider in kg.
        resolution (int): Number of evaluation points. Default 1000.

    Returns:
        tuple[float, float]: (optimal_battery_mass_kg, max_endurance_minutes)
    """
    masses = np.linspace(m_bat_min_kg, m_bat_max_kg, resolution)
    endurances = np.array([
        compute_endurance_minutes(
            m_bat_kg=m,
            m_struct_kg=m_struct_kg,
            m_payload_kg=m_payload_kg,
            n_rotors_eff=n_rotors_eff,
            disk_area_m2=disk_area_m2,
            energy_density_Wh_kg=energy_density_Wh_kg,
            depth_of_discharge=depth_of_discharge,
            rho=rho,
            g=g,
            eta_global=eta_global
        )
        for m in masses
    ])

    idx_max = int(np.argmax(endurances))
    return float(masses[idx_max]), float(endurances[idx_max])


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def compute_battery(
    m_struct_kg: float,
    m_payload_kg: float,
    n_rotors_eff: float,
    diameter_m: float,
    phys: dict
) -> BatteryResult:
    """Compute battery sizing results for a multirotor drone.

    This is the main entry point of the battery model. It:
    1. Computes the effective battery voltage from cell count.
    2. Determines the physical battery mass bounds from config fractions.
    3. Finds the optimal battery mass that maximizes hover endurance.
    4. Generates the full endurance-vs-battery-mass curve for reporting.

    Coaxial handling:
        n_rotors_eff should already account for coaxial thrust loss when
        passed from the calling module. For x8 configurations, the effective
        number of rotor disks is:
            n_rotors_eff = N_motors * thrust_loss_factor
        This adjustment is applied in the calling context (mass_model or
        propeller_model), not here.

    Args:
        m_struct_kg (float): Structural mass in kilograms (no battery,
                             no payload).
        m_payload_kg (float): Payload mass in kilograms.
        n_rotors_eff (float): Effective number of independent rotor disks.
                              For x8, this is 8 * thrust_loss_factor.
                              For quad/hexa/octo, this equals motor count.
        diameter_m (float): Propeller diameter in meters. Used to compute
                            the rotor disk area.
        phys (dict): Validated physical configuration dictionary as returned
                     by load_physical_config().

    Returns:
        BatteryResult: Named tuple containing:
            - voltage_V (float): Effective pack voltage [V].
            - m_bat_optimal_kg (float): Optimal battery mass [kg].
            - endurance_max_min (float): Maximum endurance [minutes].
            - m_bat_range_kg (np.ndarray): Battery mass sweep array [kg].
            - endurance_curve_min (np.ndarray): Endurance curve values [min].

    Raises:
        ValueError: If input values are physically inconsistent or if
                    battery mass bounds are invalid.

    Example:
        >>> result = compute_battery(
        ...     m_struct_kg=7.0,
        ...     m_payload_kg=2.0,
        ...     n_rotors_eff=6.8,
        ...     diameter_m=0.4064,
        ...     phys=physical_config
        ... )
        >>> print(f"Optimal battery: {result.m_bat_optimal_kg:.2f} kg")
        >>> print(f"Max endurance:   {result.endurance_max_min:.1f} min")
    """
    # --- Extract physical parameters ---
    atm = phys["atmosphere"]
    bat = phys["battery"]
    drive = phys["drivetrain"]

    rho = float(atm["rho_kg_m3"])
    g = float(atm["g_m_s2"])
    e_sp = float(bat["energy_density_Wh_kg"])
    dod = float(bat["depth_of_discharge"])
    v_cell = float(bat["cell_voltage_nominal_V"])
    mf_min = float(bat["mass_fraction_min"])
    mf_max = float(bat["mass_fraction_max"])

    # Retrieve cell count from inputs — passed via phys extension or separately.
    # Note: cell count is an input parameter stored outside phys.
    # The caller must have injected it as phys["battery"]["cell_count_S"].
    cell_count_S = int(bat["cell_count_S"])

    from core.config_loader import get_effective_eta
    eta = get_effective_eta(drive)

    # --- Effective voltage ---
    voltage_V = compute_effective_voltage(cell_count_S, v_cell)

    # --- Rotor disk area ---
    disk_area = compute_disk_area(diameter_m)

    # --- Battery mass bounds ---
    m_base = m_struct_kg + m_payload_kg
    m_bat_min = mf_min * m_base
    m_bat_max = mf_max * m_base

    if m_bat_min <= 0:
        raise ValueError(
            f"Computed m_bat_min ({m_bat_min:.3f} kg) is not positive. "
            f"Check mass_fraction_min and input masses."
        )
    if m_bat_min >= m_bat_max:
        raise ValueError(
            f"Battery mass bounds are invalid: "
            f"m_bat_min={m_bat_min:.3f} kg >= m_bat_max={m_bat_max:.3f} kg."
        )

    # --- Optimal battery mass ---
    m_bat_opt, endurance_max = _find_optimal_battery_mass(
        m_struct_kg=m_struct_kg,
        m_payload_kg=m_payload_kg,
        n_rotors_eff=n_rotors_eff,
        disk_area_m2=disk_area,
        energy_density_Wh_kg=e_sp,
        depth_of_discharge=dod,
        rho=rho,
        g=g,
        eta_global=eta,
        m_bat_min_kg=m_bat_min,
        m_bat_max_kg=m_bat_max
    )

    # --- Endurance curve (for report) ---
    m_bat_range = np.linspace(m_bat_min, m_bat_max, 500)
    endurance_curve = np.array([
        compute_endurance_minutes(
            m_bat_kg=m,
            m_struct_kg=m_struct_kg,
            m_payload_kg=m_payload_kg,
            n_rotors_eff=n_rotors_eff,
            disk_area_m2=disk_area,
            energy_density_Wh_kg=e_sp,
            depth_of_discharge=dod,
            rho=rho,
            g=g,
            eta_global=eta
        )
        for m in m_bat_range
    ])

    return BatteryResult(
        voltage_V=voltage_V,
        m_bat_optimal_kg=m_bat_opt,
        endurance_max_min=endurance_max,
        m_bat_range_kg=m_bat_range,
        endurance_curve_min=endurance_curve
    )
