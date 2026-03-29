"""
core/power_model.py
===================
Power consumption model for multirotor drone sizing.

This module computes the electrical power required for hover and forward
flight, as well as the thrust-to-weight ratio (TWR) for a given drone
configuration and mass breakdown.

Physical Model
--------------

Hover Power
~~~~~~~~~~~
Based on the actuator disk theory (momentum theory), the ideal mechanical
power to sustain hover is:

    P_ideal = (MTOW * g)^(3/2) / sqrt(2 * rho * A_total)

where:
    A_total = n_rotors_eff * A_disk     total effective disk area [m^2]
    A_disk  = pi * (D/2)^2              single rotor disk area [m^2]
    n_rotors_eff                        effective rotor count (coaxial-adjusted)

The shaft (electrical input) power is:

    P_hover = P_ideal / eta_global

Forward Flight Power
~~~~~~~~~~~~~~~~~~~~
In forward flight at speed V, the total power includes:
1. Induced power (reduced vs hover due to increased inflow):
        P_induced = P_hover / (1 + (V / v_h)^2)^(1/2)   (Glauert approximation)
   where v_h is the hover-induced velocity.

2. Parasite (profile drag) power — modelled via a flat-plate drag coefficient:
        P_parasite = 0.5 * rho * V^3 * Cd * A_front

   For a generic sizing tool without airframe geometry, we approximate the
   parasite power as a fixed fraction of hover power scaled by (V/V_ref)^3.
   This is the standard approach for preliminary sizing.

        P_forward = P_induced + P_parasite

Thrust-to-Weight Ratio
~~~~~~~~~~~~~~~~~~~~~~~
The available thrust at hover for a given configuration is:

    T_total = N_motors * T_single_motor

For a coaxial x8 configuration:
    T_total = 4 * 2 * T_single * thrust_loss_factor
            = N_motors * T_single * thrust_loss_factor

The TWR at MTOW is:
    TWR = T_total / (MTOW * g)

This module does not compute T_single — it takes the thrust per motor as
an input from the propeller model. The TWR is used for constraint validation
and for the TWR-vs-battery-mass curve.

Coaxial Correction
~~~~~~~~~~~~~~~~~~
For x8 configurations, n_rotors_eff is passed as:
    n_rotors_eff = N_motors * thrust_loss_factor

This value is computed upstream (in the orchestration layer) and passed
directly to all power functions.

Usage:
    from core.power_model import compute_power_hover, compute_twr, PowerResult

    result = compute_power_hover(
        mtow_kg=12.6,
        n_rotors_eff=6.8,
        diameter_m=0.4064,
        phys=physical_config
    )
"""

import math
from typing import NamedTuple
import numpy as np

from core.config_loader import get_effective_eta


# ---------------------------------------------------------------------------
# Result containers
# ---------------------------------------------------------------------------

class PowerResult(NamedTuple):
    """Results of the hover power computation.

    Attributes:
        p_hover_W (float): Electrical power required for hover [W].
        p_ideal_W (float): Ideal mechanical hover power (no losses) [W].
        v_induced_ms (float): Hover-induced velocity through the rotor disk
                              [m/s]. Used as reference for forward flight.
        eta_global (float): Effective global drivetrain efficiency used
                            in the computation.
    """
    p_hover_W: float
    p_ideal_W: float
    v_induced_ms: float
    eta_global: float


class TWRResult(NamedTuple):
    """Results of the TWR computation.

    Attributes:
        twr (float): Thrust-to-weight ratio at the given MTOW [-].
        thrust_total_N (float): Total available thrust [N].
        weight_N (float): Drone weight at MTOW [N].
        is_compliant (bool): True if twr is within [TWR_min, TWR_max]
                              from the physical config constraints.
    """
    twr: float
    thrust_total_N: float
    weight_N: float
    is_compliant: bool


# ---------------------------------------------------------------------------
# Core physical functions
# ---------------------------------------------------------------------------

def compute_power_hover(
    mtow_kg: float,
    n_rotors_eff: float,
    diameter_m: float,
    phys: dict
) -> PowerResult:
    """Compute the electrical power required for steady hover flight.

    Applies the actuator disk theory to compute ideal mechanical power,
    then divides by the effective global drivetrain efficiency to obtain
    the required electrical input power.

    Args:
        mtow_kg (float): Total take-off mass in kilograms. Must be > 0.
        n_rotors_eff (float): Effective number of independent rotor disks.
                              For flat configurations (quad/hexa/octo), this
                              equals the motor count. For coaxial x8, this
                              is N_motors * thrust_loss_factor.
        diameter_m (float): Propeller diameter in meters. Must be > 0.
        phys (dict): Validated physical configuration as returned by
                     load_physical_config().

    Returns:
        PowerResult: Named tuple with hover power, ideal power,
                     induced velocity, and effective efficiency.

    Raises:
        ValueError: If mtow_kg, n_rotors_eff, or diameter_m is non-positive.

    Physical assumptions:
        - Steady, level hover with zero translational velocity.
        - Uniform inflow velocity distribution across each rotor disk.
        - Constant drivetrain efficiency across the operating range.
        - No ground effect (altitude >> rotor diameter).
    """
    if mtow_kg <= 0:
        raise ValueError(f"mtow_kg must be strictly positive, got {mtow_kg}.")
    if n_rotors_eff <= 0:
        raise ValueError(
            f"n_rotors_eff must be strictly positive, got {n_rotors_eff}."
        )
    if diameter_m <= 0:
        raise ValueError(
            f"diameter_m must be strictly positive, got {diameter_m}."
        )

    rho = float(phys["atmosphere"]["rho_kg_m3"])
    g = float(phys["atmosphere"]["g_m_s2"])
    eta = get_effective_eta(phys["drivetrain"])

    disk_area = math.pi * (diameter_m / 2.0) ** 2
    total_disk_area = n_rotors_eff * disk_area
    weight_N = mtow_kg * g

    # Ideal mechanical hover power (actuator disk theory)
    p_ideal = (weight_N ** 1.5) / math.sqrt(2.0 * rho * total_disk_area)

    # Hover-induced velocity through the disk
    # From momentum theory: T = 2 * rho * A * v_h^2  =>  v_h = sqrt(T / (2*rho*A))
    # Here T = weight_N / n_rotors_eff per rotor
    thrust_per_rotor = weight_N / n_rotors_eff
    v_induced = math.sqrt(thrust_per_rotor / (2.0 * rho * disk_area))

    # Electrical input power
    p_hover = p_ideal / eta

    return PowerResult(
        p_hover_W=p_hover,
        p_ideal_W=p_ideal,
        v_induced_ms=v_induced,
        eta_global=eta
    )


def compute_power_forward(
    mtow_kg: float,
    n_rotors_eff: float,
    diameter_m: float,
    velocity_ms: float,
    parasite_power_fraction: float,
    phys: dict
) -> float:
    """Compute the total electrical power required for forward flight.

    Uses the Glauert inflow model to compute the reduced induced power in
    forward flight, then adds a parasite power term modelled as a fraction
    of hover power scaled by (V / V_ref)^3.

    The Glauert approximation for induced power in forward flight:

        P_induced = P_ideal_hover * v_h / sqrt(V^2/2 + sqrt(V^4/4 + v_h^4))

    where:
        v_h     = hover-induced velocity [m/s]
        V       = forward flight speed [m/s]

    Parasite power:
        P_parasite = parasite_power_fraction * P_hover * (V / v_h)^3

    Total forward flight power:
        P_forward = (P_induced + P_parasite) / eta_global

    Args:
        mtow_kg (float): Total take-off mass in kilograms.
        n_rotors_eff (float): Effective number of independent rotor disks.
        diameter_m (float): Propeller diameter in meters.
        velocity_ms (float): Forward flight speed in m/s. Must be >= 0.
        parasite_power_fraction (float): Parasite drag scaling factor relative
                                         to hover power. Typical value: 0.03
                                         to 0.08 for inspection drones.
                                         Dimensionless.
        phys (dict): Validated physical configuration dictionary.

    Returns:
        float: Total electrical power for forward flight in Watts.

    Raises:
        ValueError: If velocity_ms < 0 or parasite_power_fraction < 0.

    Physical assumptions:
        - Quasi-steady forward flight at constant altitude and speed.
        - Glauert inflow model valid for V > 0 (hover is handled separately).
        - Parasite drag modelled as a cube-law scaling of forward speed.
        - Constant drivetrain efficiency.
    """
    if velocity_ms < 0:
        raise ValueError(
            f"velocity_ms must be >= 0, got {velocity_ms}."
        )
    if parasite_power_fraction < 0:
        raise ValueError(
            f"parasite_power_fraction must be >= 0, got {parasite_power_fraction}."
        )

    hover = compute_power_hover(mtow_kg, n_rotors_eff, diameter_m, phys)
    eta = hover.eta_global
    v_h = hover.v_induced_ms
    p_ideal = hover.p_ideal_W

    if velocity_ms == 0.0:
        return hover.p_hover_W

    # Glauert induced power in forward flight
    v2 = velocity_ms ** 2
    v_h2 = v_h ** 2
    inflow_denom = math.sqrt(v2 / 2.0 + math.sqrt((v2 / 2.0) ** 2 + v_h2 ** 2))
    p_induced_ideal = p_ideal * v_h / inflow_denom

    # Parasite power (cube-law scaling)
    p_parasite_ideal = parasite_power_fraction * p_ideal * (velocity_ms / v_h) ** 3

    return (p_induced_ideal + p_parasite_ideal) / eta


def compute_twr(
    mtow_kg: float,
    thrust_total_N: float,
    phys: dict
) -> TWRResult:
    """Compute the thrust-to-weight ratio and assess constraint compliance.

    The TWR is the ratio of total available thrust to the drone's weight:
        TWR = T_total / (MTOW * g)

    A TWR >= 2.0 is typically required for safe outdoor multirotor operation,
    providing sufficient authority to maneuver and reject wind disturbances.

    Args:
        mtow_kg (float): Total take-off mass in kilograms.
        thrust_total_N (float): Total available thrust from all motors
                                in Newtons. This value is computed by
                                the propeller model.
        phys (dict): Validated physical configuration dictionary.
                     Used to read TWR_min and TWR_max constraints.

    Returns:
        TWRResult: Named tuple with twr, thrust_total_N, weight_N,
                   and is_compliant flag.

    Raises:
        ValueError: If mtow_kg or thrust_total_N is non-positive.
    """
    if mtow_kg <= 0:
        raise ValueError(f"mtow_kg must be strictly positive, got {mtow_kg}.")
    if thrust_total_N <= 0:
        raise ValueError(
            f"thrust_total_N must be strictly positive, got {thrust_total_N}."
        )

    g = float(phys["atmosphere"]["g_m_s2"])
    twr_min = float(phys["constraints"]["TWR_min"])
    twr_max = float(phys["constraints"]["TWR_max"])

    weight_N = mtow_kg * g
    twr = thrust_total_N / weight_N
    is_compliant = twr_min <= twr <= twr_max

    return TWRResult(
        twr=twr,
        thrust_total_N=thrust_total_N,
        weight_N=weight_N,
        is_compliant=is_compliant
    )


# ---------------------------------------------------------------------------
# Curve generation helpers
# ---------------------------------------------------------------------------

def power_curve_vs_mtow(
    mtow_range_kg: "np.ndarray",
    n_rotors_eff: float,
    diameter_m: float,
    phys: dict
) -> "np.ndarray":
    """Compute hover power for a range of MTOW values.

    Used to generate the power-vs-MTOW curve in the output report.
    The current mission MTOW is highlighted as a reference point by
    the curve rendering module.

    Args:
        mtow_range_kg (np.ndarray): Array of MTOW values in kilograms.
                                    Typically a linspace from a fraction of
                                    the nominal MTOW to twice its value.
        n_rotors_eff (float): Effective number of rotor disks.
        diameter_m (float): Propeller diameter in meters.
        phys (dict): Validated physical configuration dictionary.

    Returns:
        np.ndarray: Array of hover power values in Watts, same length as
                    mtow_range_kg.
    """
    return np.array([
        compute_power_hover(m, n_rotors_eff, diameter_m, phys).p_hover_W
        for m in mtow_range_kg
    ])


def twr_curve_vs_bat_mass(
    m_bat_range_kg: "np.ndarray",
    m_struct_kg: float,
    m_payload_kg: float,
    thrust_per_motor_N: float,
    n_motors: int,
    coaxial: bool,
    thrust_loss_factor: float,
    phys: dict
) -> "np.ndarray":
    """Compute the effective TWR for a range of battery masses.

    As battery mass increases, MTOW increases and TWR decreases.
    This curve identifies the battery mass at which TWR drops below the
    minimum acceptable threshold.

    Args:
        m_bat_range_kg (np.ndarray): Array of battery mass values [kg].
        m_struct_kg (float): Structural mass [kg].
        m_payload_kg (float): Payload mass [kg].
        thrust_per_motor_N (float): Thrust produced by a single motor [N].
                                    Computed by propeller_model.
        n_motors (int): Total number of motors.
        coaxial (bool): True if the configuration is coaxial (x8).
        thrust_loss_factor (float): Coaxial thrust loss factor. Applied only
                                    if coaxial is True.
        phys (dict): Validated physical configuration dictionary.

    Returns:
        np.ndarray: Array of TWR values, same length as m_bat_range_kg.
    """
    g = float(phys["atmosphere"]["g_m_s2"])
    twr_values = []

    for m_bat in m_bat_range_kg:
        mtow = m_struct_kg + m_bat + m_payload_kg
        weight_N = mtow * g

        if coaxial:
            thrust_total = n_motors * thrust_per_motor_N * thrust_loss_factor
        else:
            thrust_total = n_motors * thrust_per_motor_N

        twr = thrust_total / weight_N
        twr_values.append(twr)

    return np.array(twr_values)
