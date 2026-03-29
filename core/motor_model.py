"""
core/motor_model.py
===================
Motor sizing model for multirotor drone design.

This module computes the electrical and mechanical characteristics at two
distinct operating points, allowing unambiguous motor selection:

Operating Points
----------------
1. **Hover** — sustained level flight at mission MTOW.
   Power consumption is determined by actuator disk theory.
   This is the continuous rating the motor must sustain indefinitely.

2. **TWR cible** — maximum thrust operating point at the target TWR.
   This represents the peak demand (climb, wind rejection, maneuver).
   Motors must handle this transiently without overheating.

Physical Model
--------------

Power per motor
~~~~~~~~~~~~~~~
At hover, total electrical power comes from power_model:
    P_elec_hover = P_hover_total / N_motors

At TWR cible, total thrust is:
    T_total_TWR = MTOW * g * TWR_cible
    T_per_motor_TWR = T_total_TWR / N_eff_motors

Shaft power at TWR cible is derived from the propeller power coefficient:
    P_shaft_TWR = CP * rho * (n_TWR)^3 * D^5

Since n_TWR = v_h_TWR / (pitch * (1-slip)), and T = CT * rho * n^2 * D^4:
    n_TWR = sqrt(T_per_motor_TWR / (CT * rho * D^4))
    P_shaft_TWR = CP * rho * n_TWR^3 * D^5

Electrical input power:
    P_elec_TWR = P_shaft_TWR / eta_motor

Current per motor
~~~~~~~~~~~~~~~~~
    I = P_elec / V_pack

Torque per motor
~~~~~~~~~~~~~~~~
    Q = P_shaft / omega     where omega = n * 2*pi  [rad/s, n in rev/s]

RPM
~~~
At hover:   from propeller_model (momentum theory + pitch)
At TWR:     n_TWR = sqrt(T_per_motor / (CT * rho * D^4))  [rev/s -> RPM]

KV range recommendation
~~~~~~~~~~~~~~~~~~~~~~~~
Based on TWR operating point (peak demand):
    KV_recommended = RPM_TWR / (V_pack * KV_load_factor)

A ±20% range accounts for motor winding variants.

Thrust per motor
~~~~~~~~~~~~~~~~
At hover:  T_hover = MTOW * g / N_eff_motors
At TWR:    T_TWR   = T_per_motor_N  (from propeller_model)
In grams:  T_g = T_N * 1000 / g

Usage:
    from core.motor_model import compute_motor_sizing, MotorSizingResult

    result = compute_motor_sizing(
        p_hover_W=852.0,
        mtow_kg=12.6,
        n_motors=8,
        n_rotors_eff=6.8,
        rpm_hover=3096.0,
        thrust_per_motor_twr_N=36.35,
        twr_cible=2.0,
        diameter_m=0.4064,
        pitch_m=0.24384,
        voltage_V=44.4,
        phys=physical_config
    )
"""

import math
from typing import NamedTuple


# ---------------------------------------------------------------------------
# Result container
# ---------------------------------------------------------------------------

class MotorSizingResult(NamedTuple):
    """Motor sizing results at two operating points: hover and TWR cible.

    All per-motor values are for a single motor.

    Hover operating point — continuous rating:
        p_elec_hover_W      Electrical input power per motor [W]
        p_shaft_hover_W     Shaft (mechanical) power per motor [W]
        current_hover_A     Current draw per motor [A]
        torque_hover_Nm     Shaft torque per motor [N·m]
        thrust_hover_N      Thrust per motor [N]
        thrust_hover_g      Thrust per motor [g]
        rpm_hover           Rotor speed [RPM]

    TWR cible operating point — peak / transient rating:
        p_elec_twr_W        Electrical input power per motor [W]
        p_shaft_twr_W       Shaft (mechanical) power per motor [W]
        current_twr_A       Current draw per motor [A]
        torque_twr_Nm       Shaft torque per motor [N·m]
        thrust_twr_N        Thrust per motor [N]
        thrust_twr_g        Thrust per motor [g]
        rpm_twr             Rotor speed [RPM]

    KV recommendation (based on TWR peak point):
        kv_recommended      Nominal KV constant [RPM/V]
        kv_range_min        Lower bound ±20% [RPM/V]
        kv_range_max        Upper bound ±20% [RPM/V]

    Common:
        voltage_V           Battery pack voltage [V]
    """
    # --- Hover ---
    p_elec_hover_W: float
    p_shaft_hover_W: float
    current_hover_A: float
    torque_hover_Nm: float
    thrust_hover_N: float
    thrust_hover_g: float
    rpm_hover: float

    # --- TWR cible ---
    p_elec_twr_W: float
    p_shaft_twr_W: float
    current_twr_A: float
    torque_twr_Nm: float
    thrust_twr_N: float
    thrust_twr_g: float
    rpm_twr: float

    # --- KV recommendation ---
    kv_recommended: float
    kv_range_min: float
    kv_range_max: float

    # --- Common ---
    voltage_V: float


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _rpm_from_thrust(
    thrust_N: float,
    CT: float,
    rho: float,
    diameter_m: float
) -> float:
    """Estimate rotor RPM from thrust using the CT coefficient model.

    From the thrust coefficient definition:
        T = CT * rho * n^2 * D^4
        n = sqrt(T / (CT * rho * D^4))   [rev/s]

    Args:
        thrust_N (float): Thrust produced by one rotor [N].
        CT (float): Non-dimensional thrust coefficient.
        rho (float): Air density [kg/m^3].
        diameter_m (float): Propeller diameter [m].

    Returns:
        float: Rotor speed in RPM.
    """
    n_rev_s = math.sqrt(thrust_N / (CT * rho * diameter_m ** 4))
    return n_rev_s * 60.0


def _shaft_power_from_rpm(
    rpm: float,
    CP: float,
    rho: float,
    diameter_m: float
) -> float:
    """Compute shaft power from RPM using the CP coefficient model.

    From the power coefficient definition:
        P = CP * rho * n^3 * D^5

    Args:
        rpm (float): Rotor speed [RPM].
        CP (float): Non-dimensional power coefficient.
        rho (float): Air density [kg/m^3].
        diameter_m (float): Propeller diameter [m].

    Returns:
        float: Shaft (mechanical) power [W].
    """
    n_rev_s = rpm / 60.0
    return CP * rho * (n_rev_s ** 3) * (diameter_m ** 5)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def compute_motor_sizing(
    p_hover_W: float,
    mtow_kg: float,
    n_motors: int,
    n_rotors_eff: float,
    rpm_hover: float,
    thrust_per_motor_twr_N: float,
    twr_cible: float,
    diameter_m: float,
    pitch_m: float,
    voltage_V: float,
    phys: dict,
    kv_range_percent: float = 0.20
) -> MotorSizingResult:
    """Compute motor sizing requirements at hover and TWR cible operating points.

    Derives all per-motor electrical and mechanical parameters at two
    operating points to enable unambiguous motor selection:
    - Hover: continuous rating (sustained level flight at MTOW)
    - TWR cible: peak/transient rating (climb, wind rejection, maneuver)

    Args:
        p_hover_W (float): Total electrical hover power for all motors [W].
                           As returned by power_model.compute_power_hover().
        mtow_kg (float): Total take-off mass [kg].
        n_motors (int): Total number of motors.
        n_rotors_eff (float): Effective rotor count (coaxial-adjusted).
                              As returned by propeller_model.
        rpm_hover (float): Estimated hover RPM per motor [RPM].
                           As returned by propeller_model.
        thrust_per_motor_twr_N (float): Thrust per motor at TWR cible [N].
                                        As returned by propeller_model.
        twr_cible (float): Target thrust-to-weight ratio [-].
        diameter_m (float): Propeller diameter [m].
        pitch_m (float): Propeller pitch [m].
        voltage_V (float): Effective battery pack voltage [V].
        phys (dict): Validated physical configuration dictionary. Used to
                     read eta_motor, CT, CP, rho, g, slip_factor.
        kv_range_percent (float): Half-width of the KV range as a fraction
                                  of the nominal KV. Default 0.20 (±20%).

    Returns:
        MotorSizingResult: Named tuple with all motor sizing parameters
                           at both operating points.

    Raises:
        ValueError: If any required input value is non-positive.

    Physical assumptions:
        - Hover: continuous operation, derived from actuator disk power.
        - TWR cible: transient peak, derived from CT/CP model at peak thrust.
        - Constant motor efficiency (eta_motor) at both operating points.
        - KV recommendation based on TWR peak point (most demanding).
        - KV load factor of 0.85 (actual load RPM = 85% of no-load RPM).

    Example:
        >>> result = compute_motor_sizing(
        ...     p_hover_W=852.0,
        ...     mtow_kg=12.6,
        ...     n_motors=8,
        ...     n_rotors_eff=6.8,
        ...     rpm_hover=3096.0,
        ...     thrust_per_motor_twr_N=36.35,
        ...     twr_cible=2.0,
        ...     diameter_m=0.4064,
        ...     pitch_m=0.24384,
        ...     voltage_V=44.4,
        ...     phys=physical_config
        ... )
        >>> print(f"Hover current  : {result.current_hover_A:.1f} A")
        >>> print(f"TWR current    : {result.current_twr_A:.1f} A")
        >>> print(f"KV recommended : {result.kv_recommended:.0f} RPM/V")
    """
    if p_hover_W <= 0:
        raise ValueError(f"p_hover_W must be > 0, got {p_hover_W}.")
    if mtow_kg <= 0:
        raise ValueError(f"mtow_kg must be > 0, got {mtow_kg}.")
    if n_motors <= 0:
        raise ValueError(f"n_motors must be > 0, got {n_motors}.")
    if n_rotors_eff <= 0:
        raise ValueError(f"n_rotors_eff must be > 0, got {n_rotors_eff}.")
    if rpm_hover <= 0:
        raise ValueError(f"rpm_hover must be > 0, got {rpm_hover}.")
    if thrust_per_motor_twr_N <= 0:
        raise ValueError(
            f"thrust_per_motor_twr_N must be > 0, got {thrust_per_motor_twr_N}."
        )
    if diameter_m <= 0:
        raise ValueError(f"diameter_m must be > 0, got {diameter_m}.")
    if pitch_m <= 0:
        raise ValueError(f"pitch_m must be > 0, got {pitch_m}.")
    if voltage_V <= 0:
        raise ValueError(f"voltage_V must be > 0, got {voltage_V}.")

    g       = float(phys["atmosphere"]["g_m_s2"])
    rho     = float(phys["atmosphere"]["rho_kg_m3"])
    eta_m   = float(phys["drivetrain"]["eta_motor"])
    CT      = float(phys["propeller_model"]["CT"])
    CP      = float(phys["propeller_model"]["CP"])

    _KV_LOAD_FACTOR = 0.85

    # -----------------------------------------------------------------------
    # Hover operating point
    # -----------------------------------------------------------------------
    # Thrust per motor at hover (equal distribution across effective rotors)
    thrust_hover_N = (mtow_kg * g) / n_rotors_eff

    # Electrical power per motor at hover
    p_elec_hover   = p_hover_W / n_motors
    p_shaft_hover  = p_elec_hover * eta_m

    # Current at hover
    current_hover  = p_elec_hover / voltage_V

    # Torque at hover
    omega_hover    = rpm_hover * 2.0 * math.pi / 60.0
    torque_hover   = p_shaft_hover / omega_hover

    # Thrust in grams at hover
    thrust_hover_g = thrust_hover_N * 1000.0 / g

    # -----------------------------------------------------------------------
    # TWR cible operating point
    # -----------------------------------------------------------------------
    # RPM at TWR cible — scaled from hover RPM using momentum theory.
    # From momentum theory: T ∝ n^2  =>  n_TWR = n_hover * sqrt(T_TWR / T_hover)
    # This is consistent with the hover RPM derived from pitch + momentum theory
    # in propeller_model, avoiding the divergence between CT and momentum models.
    rpm_twr        = rpm_hover * (thrust_per_motor_twr_N / thrust_hover_N) ** 0.5

    # Shaft power at TWR cible.
    # From momentum theory: P ∝ T^(3/2)  =>  P_TWR = P_hover * (T_TWR/T_hover)^(3/2)
    p_shaft_twr    = p_shaft_hover * (thrust_per_motor_twr_N / thrust_hover_N) ** 1.5
    p_elec_twr     = p_shaft_twr / eta_m

    # Current at TWR cible
    current_twr    = p_elec_twr / voltage_V

    # Torque at TWR cible
    omega_twr      = rpm_twr * 2.0 * math.pi / 60.0
    torque_twr     = p_shaft_twr / omega_twr

    # Thrust in grams at TWR cible
    thrust_twr_g   = thrust_per_motor_twr_N * 1000.0 / g

    # -----------------------------------------------------------------------
    # KV recommendation — based on TWR peak point
    # -----------------------------------------------------------------------
    kv_recommended = rpm_twr / (voltage_V * _KV_LOAD_FACTOR)
    kv_range_min   = kv_recommended * (1.0 - kv_range_percent)
    kv_range_max   = kv_recommended * (1.0 + kv_range_percent)

    return MotorSizingResult(
        # Hover
        p_elec_hover_W=p_elec_hover,
        p_shaft_hover_W=p_shaft_hover,
        current_hover_A=current_hover,
        torque_hover_Nm=torque_hover,
        thrust_hover_N=thrust_hover_N,
        thrust_hover_g=thrust_hover_g,
        rpm_hover=rpm_hover,
        # TWR cible
        p_elec_twr_W=p_elec_twr,
        p_shaft_twr_W=p_shaft_twr,
        current_twr_A=current_twr,
        torque_twr_Nm=torque_twr,
        thrust_twr_N=thrust_per_motor_twr_N,
        thrust_twr_g=thrust_twr_g,
        rpm_twr=rpm_twr,
        # KV
        kv_recommended=kv_recommended,
        kv_range_min=kv_range_min,
        kv_range_max=kv_range_max,
        # Common
        voltage_V=voltage_V,
    )