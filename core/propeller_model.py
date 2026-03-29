"""
core/propeller_model.py
=======================
Propeller sizing model for multirotor drone design.

This module computes the optimal propeller diameter and pitch for a given
drone configuration, using a hybrid analytical model combining actuator disk
theory with empirical corrective coefficients (CT, CP) from the physical
configuration file.

Physical Model
--------------

Thrust and Power Coefficients
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The non-dimensional thrust and power produced by a single rotor are:

    T = CT * rho * n^2 * D^4
    P = CP * rho * n^3 * D^5

where:
    CT      thrust coefficient (from physical_config.json)
    CP      power coefficient  (from physical_config.json)
    rho     air density [kg/m^3]
    n       rotational speed [rev/s]
    D       propeller diameter [m]

These coefficients calibrate the purely analytical model to the target
propeller family, bridging theory and real-world performance.

Required Thrust per Motor
~~~~~~~~~~~~~~~~~~~~~~~~~
At hover, each motor must produce:

    T_required = MTOW * g / N_motors_eff

For coaxial configurations (x8), the thrust loss factor is applied:

    T_required = MTOW * g / (N_motors * thrust_loss_factor)

Optimal Diameter (from actuator disk theory)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
From momentum theory, the thrust of a single rotor is:

    T = 2 * rho * A * v_h^2    where A = pi * (D/2)^2

Combining with the CT model and solving for D:

    D = ( T / (CT * rho * n^2) )^(1/4)

Since n is unknown at this stage, the diameter is derived directly from the
required thrust using the momentum theory form:

    D = sqrt( 2 * T / (rho * pi * v_h^2) )

This is solved iteratively: an initial diameter estimate is obtained from the
thrust coefficient relation, and refined by matching the momentum theory
constraint. In practice, for the sizing stage, we use the simplified closed-
form expression derived from momentum theory only:

    D_opt = ( (2 * T_required) / (CT * rho * pi) )^(1/3) * (2/pi)^(1/3)

The result is clamped to [diameter_min_inch, diameter_max_inch] and
[constraints.propeller_diameter_max_inch] from the configuration file.

Optimal Pitch
~~~~~~~~~~~~~
Pitch is determined by the flight regime:

    Pitch_hover = pitch_to_diameter_ratio_hover * D
    Pitch_speed = pitch_to_diameter_ratio_speed * D

The reported optimal pitch is the hover pitch (maximizes hover efficiency).
The speed pitch is reported separately for reference.

Thrust per Motor (back-calculation)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Once the diameter is determined, the thrust per motor at the required
TWR is back-calculated:

    T_per_motor = MTOW * g * TWR_cible / N_motors

This value is passed to power_model.twr_curve_vs_bat_mass() for TWR curve
generation.

Usage:
    from core.propeller_model import compute_propeller, PropellerResult

    result = compute_propeller(
        mtow_kg=12.6,
        n_motors=8,
        twr_cible=2.0,
        velocity_ms=16.67,
        phys=physical_config
    )
"""

import math
from typing import NamedTuple

import numpy as np


# ---------------------------------------------------------------------------
# Result container
# ---------------------------------------------------------------------------

class PropellerResult(NamedTuple):
    """Results of the propeller sizing computation.

    Attributes:
        diameter_m (float): Propeller diameter in meters (from input).
        diameter_inch (float): Propeller diameter in inches (from input).
        pitch_hover_m (float): Optimal propeller pitch for hover [m].
        pitch_hover_inch (float): Optimal propeller pitch for hover [inches].
        pitch_speed_m (float): Optimal propeller pitch for forward flight [m].
        pitch_speed_inch (float): Optimal propeller pitch for forward flight
                                  [inches].
        thrust_per_motor_N (float): Required thrust per motor at target TWR
                                    [N].
        thrust_total_N (float): Total available thrust at target TWR [N].
        n_rotors_eff (float): Effective rotor count used in calculations.
                              Coaxial-adjusted for x8 configurations.
        rpm_hover (float): Estimated rotor RPM in hover, derived from
                           momentum theory and pitch. [rev/min].
        diameter_clamped (bool): Always False (diameter is user-supplied).
                                 Kept for API compatibility.
        warnings (list[str]): List of warning messages for constraint
                              violations.
    """
    diameter_m: float
    diameter_inch: float
    pitch_hover_m: float
    pitch_hover_inch: float
    pitch_speed_m: float
    pitch_speed_inch: float
    thrust_per_motor_N: float
    thrust_total_N: float
    n_rotors_eff: float
    rpm_hover: float
    diameter_clamped: bool
    warnings: list


# ---------------------------------------------------------------------------
# Unit conversion helpers
# ---------------------------------------------------------------------------

def meters_to_inches(value_m: float) -> float:
    """Convert a length from meters to inches.

    Args:
        value_m (float): Length in meters.

    Returns:
        float: Length in inches.
    """
    return value_m / 0.0254


def inches_to_meters(value_inch: float) -> float:
    """Convert a length from inches to meters.

    Args:
        value_inch (float): Length in inches.

    Returns:
        float: Length in meters.
    """
    return value_inch * 0.0254


# ---------------------------------------------------------------------------
# Core physical functions
# ---------------------------------------------------------------------------

def compute_required_thrust_per_motor(
    mtow_kg: float,
    twr_cible: float,
    n_motors: int,
    coaxial: bool,
    thrust_loss_factor: float,
    g: float
) -> float:
    """Compute the thrust each motor must produce to achieve the target TWR.

    The total required thrust is:
        T_total = MTOW * g * TWR_cible

    For a flat configuration:
        T_per_motor = T_total / N_motors

    For a coaxial configuration (x8), the thrust loss factor accounts for
    the aerodynamic interference between rotor pairs. Each motor effectively
    produces less thrust, so a higher single-motor thrust is needed:
        T_per_motor = T_total / (N_motors * thrust_loss_factor)

    Args:
        mtow_kg (float): Total take-off mass [kg].
        twr_cible (float): Target thrust-to-weight ratio [-].
        n_motors (int): Total number of motors.
        coaxial (bool): True if the configuration uses coaxial rotor pairs.
        thrust_loss_factor (float): Coaxial thrust loss factor. Ignored if
                                    coaxial is False.
        g (float): Gravitational acceleration [m/s^2].

    Returns:
        float: Required thrust per motor in Newtons.

    Raises:
        ValueError: If mtow_kg, twr_cible, or n_motors is non-positive.
    """
    if mtow_kg <= 0:
        raise ValueError(f"mtow_kg must be > 0, got {mtow_kg}.")
    if twr_cible <= 0:
        raise ValueError(f"twr_cible must be > 0, got {twr_cible}.")
    if n_motors <= 0:
        raise ValueError(f"n_motors must be > 0, got {n_motors}.")

    t_total = mtow_kg * g * twr_cible

    if coaxial:
        effective_motors = n_motors * thrust_loss_factor
    else:
        effective_motors = float(n_motors)

    return t_total / effective_motors


def compute_optimal_diameter(
    thrust_required_N: float,
    CT: float,
    rho: float,
    diameter_min_m: float,
    diameter_max_m: float
) -> tuple[float, bool]:
    """Compute the optimal propeller diameter from the required thrust.

    Uses a hybrid approach combining the CT coefficient model with the
    actuator disk thrust equation. The diameter is derived by equating the
    thrust produced by a rotor of diameter D at the design point with the
    required thrust per motor.

    From actuator disk momentum theory and the CT model, the optimal diameter
    is derived as:

        D_opt = ( (2 * T) / (CT * rho * pi) )^(1/3)

    This expression gives the diameter at which the rotor operates at its
    design CT, producing exactly the required thrust.

    The result is clamped to [diameter_min_m, diameter_max_m].

    Args:
        thrust_required_N (float): Required thrust per motor [N].
        CT (float): Non-dimensional thrust coefficient from config.
        rho (float): Air density [kg/m^3].
        diameter_min_m (float): Minimum allowable diameter [m].
        diameter_max_m (float): Maximum allowable diameter [m].

    Returns:
        tuple[float, bool]: (optimal_diameter_m, was_clamped)
            - optimal_diameter_m: Diameter in meters.
            - was_clamped: True if the value was constrained by bounds.

    Raises:
        ValueError: If thrust_required_N, CT, or rho is non-positive.
    """
    if thrust_required_N <= 0:
        raise ValueError(
            f"thrust_required_N must be > 0, got {thrust_required_N}."
        )
    if CT <= 0:
        raise ValueError(f"CT must be > 0, got {CT}.")
    if rho <= 0:
        raise ValueError(f"rho must be > 0, got {rho}.")

    # Derived from: T = CT * rho * (n^2 * D^4)
    # Combined with momentum theory: T = 0.5 * rho * pi * D^2 * v_h^2 * 2
    # Solving for D using the simplified form:
    d_opt = (2.0 * thrust_required_N / (CT * rho * math.pi)) ** (1.0 / 3.0)

    clamped = False
    if d_opt < diameter_min_m:
        d_opt = diameter_min_m
        clamped = True
    elif d_opt > diameter_max_m:
        d_opt = diameter_max_m
        clamped = True

    return d_opt, clamped


def compute_effective_rotor_count(
    n_motors: int,
    coaxial: bool,
    thrust_loss_factor: float
) -> float:
    """Compute the effective number of independent rotor disks.

    For flat configurations, all motors contribute independently:
        n_eff = n_motors

    For coaxial configurations (x8), rotor pairs share a disk area and the
    lower rotor operates in the wake of the upper one. The effective rotor
    count is adjusted by the thrust loss factor:
        n_eff = n_motors * thrust_loss_factor

    This value is used in power and endurance calculations to correctly
    account for the aerodynamic efficiency loss.

    Args:
        n_motors (int): Total number of motors.
        coaxial (bool): True if the configuration is coaxial (x8).
        thrust_loss_factor (float): Coaxial efficiency factor in (0, 1].
                                    Ignored if coaxial is False.

    Returns:
        float: Effective number of rotor disks for actuator disk calculations.
    """
    if coaxial:
        return float(n_motors) * thrust_loss_factor
    return float(n_motors)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def compute_propeller(
    mtow_kg: float,
    n_motors: int,
    twr_cible: float,
    diameter_inch: float,
    pitch_inch: float,
    phys: dict
) -> PropellerResult:
    """Compute propeller sizing results for a drone configuration.

    Both diameter and pitch are supplied directly from the mission inputs
    (manufacturer datasheet values). No analytical pitch derivation is
    performed.

    This is the main entry point of the propeller model. It:
    1. Determines coaxial status from the physical config.
    2. Computes the required thrust per motor at the target TWR.
    3. Uses the user-supplied diameter and pitch directly.
    4. Validates diameter and pitch against configured bounds.
    5. Back-calculates total thrust and n_rotors_eff for downstream use.
    6. Estimates hover RPM from momentum theory and supplied pitch.
    7. Collects constraint warnings for the output report.

    Args:
        mtow_kg (float): Total take-off mass in kilograms.
        n_motors (int): Total number of motors (from drone configuration).
        twr_cible (float): Target thrust-to-weight ratio at MTOW.
        diameter_inch (float): Propeller diameter in inches, as provided
                               in the mission input file (manufacturer value).
        pitch_inch (float): Propeller pitch in inches, as provided in the
                            mission input file (manufacturer value).
        phys (dict): Validated physical configuration as returned by
                     load_physical_config(). Must contain 'propeller_model',
                     'atmosphere', 'coaxial_correction', and 'constraints'.

    Returns:
        PropellerResult: Named tuple with diameter, pitch (hover and speed),
                         thrust values, effective rotor count, and warnings.

    Raises:
        ValueError: If mtow_kg, n_motors, twr_cible, or diameter_inch
                    is non-positive.

    Example:
        >>> result = compute_propeller(
        ...     mtow_kg=12.6,
        ...     n_motors=8,
        ...     twr_cible=2.0,
        ...     velocity_ms=16.67,
        ...     diameter_inch=16.0,
        ...     phys=physical_config
        ... )
        >>> print(f"Diameter : {result.diameter_inch:.1f} inches")
        >>> print(f"Pitch    : {result.pitch_hover_inch:.1f} inches")
    """
    if mtow_kg <= 0:
        raise ValueError(f"mtow_kg must be > 0, got {mtow_kg}.")
    if n_motors <= 0:
        raise ValueError(f"n_motors must be > 0, got {n_motors}.")
    if twr_cible <= 0:
        raise ValueError(f"twr_cible must be > 0, got {twr_cible}.")
    if diameter_inch <= 0:
        raise ValueError(f"diameter_inch must be > 0, got {diameter_inch}.")
    if pitch_inch <= 0:
        raise ValueError(f"pitch_inch must be > 0, got {pitch_inch}.")

    # --- Extract physical parameters ---
    atm = phys["atmosphere"]
    prop = phys["propeller_model"]
    coax = phys["coaxial_correction"]
    constraints = phys["constraints"]

    g = float(atm["g_m_s2"])
    d_min_inch_cfg = float(prop["diameter_min_inch"])
    d_constraint_max_inch = float(constraints["propeller_diameter_max_inch"])

    coaxial = bool(coax["enabled"])
    thrust_loss = float(coax["thrust_loss_factor"])

    warnings = []

    # --- Diameter and pitch: use user-supplied values directly ---
    d_opt_m = inches_to_meters(diameter_inch)
    pitch_m = inches_to_meters(pitch_inch)
    was_clamped = False

    # Validate diameter against bounds
    if diameter_inch < d_min_inch_cfg:
        warnings.append(
            f"WARNING: Supplied diameter ({diameter_inch:.1f} in) is below "
            f"the configured minimum ({d_min_inch_cfg:.1f} in)."
        )
    if diameter_inch > d_constraint_max_inch:
        warnings.append(
            f"WARNING: Supplied diameter ({diameter_inch:.1f} in) exceeds "
            f"the structural maximum ({d_constraint_max_inch:.1f} in)."
        )

    # --- Effective rotor count ---
    n_rotors_eff = compute_effective_rotor_count(n_motors, coaxial, thrust_loss)

    # --- Required thrust per motor at target TWR ---
    t_per_motor = compute_required_thrust_per_motor(
        mtow_kg=mtow_kg,
        twr_cible=twr_cible,
        n_motors=n_motors,
        coaxial=coaxial,
        thrust_loss_factor=thrust_loss,
        g=g
    )

    # --- Pitch: user-supplied value used for both hover and speed ---
    pitch_hover_m = pitch_m
    pitch_speed_m = pitch_m

    # --- Total thrust at target TWR ---
    if coaxial:
        thrust_total_N = n_motors * t_per_motor * thrust_loss
    else:
        thrust_total_N = n_motors * t_per_motor

    # --- TWR constraint check ---
    weight_N = mtow_kg * g
    twr_effective = thrust_total_N / weight_N
    twr_min = float(constraints["TWR_min"])
    twr_max = float(constraints["TWR_max"])

    if twr_effective < twr_min:
        warnings.append(
            f"WARNING: Effective TWR ({twr_effective:.2f}) is below "
            f"minimum constraint ({twr_min})."
        )
    elif twr_effective > twr_max:
        warnings.append(
            f"WARNING: Effective TWR ({twr_effective:.2f}) exceeds "
            f"maximum constraint ({twr_max}). Drone may be oversized."
        )

    # --- Hover RPM estimate ---
    # From momentum theory: T = 2 * rho * A * v_h^2
    # => v_h = sqrt(T / (2 * rho * A))
    # From propeller advance: v_h = n * pitch * (1 - slip)
    # => n [rev/s] = v_h / (pitch_hover_m * (1 - slip))
    rho = float(atm["rho_kg_m3"])
    slip = float(prop["slip_factor"])
    disk_area = math.pi * (d_opt_m / 2.0) ** 2
    v_h = math.sqrt(t_per_motor / (2.0 * rho * disk_area))
    n_rev_s = v_h / (pitch_hover_m * (1.0 - slip))
    rpm_hover = n_rev_s * 60.0

    return PropellerResult(
        diameter_m=d_opt_m,
        diameter_inch=float(diameter_inch),
        pitch_hover_m=pitch_hover_m,
        pitch_hover_inch=float(pitch_inch),
        pitch_speed_m=pitch_speed_m,
        pitch_speed_inch=float(pitch_inch),
        thrust_per_motor_N=t_per_motor,
        thrust_total_N=thrust_total_N,
        n_rotors_eff=n_rotors_eff,
        rpm_hover=rpm_hover,
        diameter_clamped=was_clamped,
        warnings=warnings
    )