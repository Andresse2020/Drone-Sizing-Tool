"""
report/builder.py
=================
Orchestration module for the drone sizing report.

This module is the single entry point that:
1. Loads and validates both configuration files
2. Runs all sizing computations in the correct dependency order
3. Generates all four curves as SVG strings
4. Collects all warnings from each model
5. Assembles a single ReportData object passed to the renderer

Computation pipeline
--------------------
The modules must be called in the following order due to data dependencies:

    Step 1 — config_loader     : Load and validate JSON files
    Step 2 — propeller_model   : First pass with estimated diameter
                                 (needed to compute n_rotors_eff)
    Step 3 — battery_model     : Compute optimal battery mass
                                 (needs n_rotors_eff and diameter)
    Step 4 — mass_model        : Compute all four masses
                                 (needs m_bat from battery_model)
    Step 5 — propeller_model   : Second pass with actual MTOW
                                 (refines diameter at true MTOW)
    Step 6 — battery_model     : Second pass with refined diameter
    Step 7 — mass_model        : Final mass breakdown
    Step 8 — power_model       : Hover and forward flight power
    Step 9 — curves/           : Generate all four SVG curves

The two-pass approach on propeller and battery models is necessary because:
- The propeller diameter depends on MTOW
- MTOW depends on the battery mass
- The battery mass depends on the propeller diameter (disk area)

Two iterations are sufficient for convergence in all practical cases.

Usage:
    from report.builder import build

    data = build(inputs, phys)
    # data is a ReportData ready to pass to a renderer
"""

import json
import os
from dataclasses import dataclass, field
from datetime import date
from typing import Optional

from core.config_loader import (
    load_physical_config,
    load_mission_inputs,
    get_motor_count,
    is_coaxial,
    get_effective_eta,
)
from core.battery_model import compute_battery, BatteryResult
from core.mass_model import compute_masses, MassBreakdown, format_mass_summary
from core.propeller_model import compute_propeller, PropellerResult
from core.power_model import compute_power_hover, compute_power_forward, compute_twr

from curves.curve_power import generate as gen_curve_power
from curves.curve_endurance_payload import generate as gen_curve_endurance_payload
from curves.curve_endurance_battery import generate as gen_curve_endurance_battery
from curves.curve_twr import generate as gen_curve_twr

from core.motor_model import compute_motor_sizing, MotorSizingResult


# ---------------------------------------------------------------------------
# Report data container
# ---------------------------------------------------------------------------

@dataclass
class ReportData:
    """Complete data package passed from the builder to a renderer.

    All computed values, SVG curves, and metadata are stored here.
    The renderer consumes this object without performing any physics.

    Attributes:
        mission_name (str): Name of the mission from inputs.json.
        mission_description (str): Description from inputs.json.
        report_date (str): ISO date string of report generation (YYYY-MM-DD).
        config_version (str): Version string from physical_config.json.

        inputs (dict): Raw validated inputs dictionary (stripped of comments).
        phys (dict): Raw validated physical config dictionary.

        masses (MassBreakdown): All six mass quantities [kg].
        battery (BatteryResult): Battery sizing results.
        prop (PropellerResult): Propeller sizing results.

        p_hover_W (float): Hover power at mission MTOW [W].
        p_forward_W (float): Forward flight power at cruise speed [W].
        twr_result: TWR result at mission MTOW.
        eta_global (float): Effective global drivetrain efficiency.
        voltage_V (float): Effective battery pack voltage [V].
        velocity_kmh (float): Cruise speed from inputs [km/h].

        svg_power (str): SVG string — hover power vs MTOW curve.
        svg_endurance_payload (str): SVG string — endurance vs payload curve.
        svg_endurance_battery (str): SVG string — endurance vs battery mass.
        svg_twr (str): SVG string — TWR vs battery mass curve.

        warnings (list[str]): All warnings collected from all models.
    """
    mission_name: str
    mission_description: str
    report_date: str
    config_version: str

    inputs: dict
    phys: dict

    masses: MassBreakdown
    battery: BatteryResult
    prop: PropellerResult

    p_hover_W: float
    p_forward_W: float
    twr_result: object
    eta_global: float
    voltage_V: float
    velocity_kmh: float

    svg_power: str
    svg_endurance_payload: str
    svg_endurance_battery: str
    svg_twr: str

    motor: MotorSizingResult
    n_motors: int

    warnings: list = field(default_factory=list)


# ---------------------------------------------------------------------------
# Builder
# ---------------------------------------------------------------------------

def build(inputs: dict, phys: dict) -> ReportData:
    """Orchestrate all sizing computations and assemble the report data.

    Runs the full two-pass computation pipeline and generates all four
    SVG curves. Returns a ReportData object ready to pass to a renderer.

    Args:
        inputs (dict): Validated mission inputs as returned by
                       load_mission_inputs(). Must contain 'mission',
                       'drone', and 'output' keys.
        phys (dict): Validated physical configuration as returned by
                     load_physical_config().

    Returns:
        ReportData: Fully populated report data object.

    Raises:
        ValueError: If any computation step encounters an inconsistent
                    physical configuration.

    Pipeline:
        Pass 1 uses a 16-inch diameter estimate to bootstrap battery sizing.
        Pass 2 refines with the actual MTOW-derived diameter.
        Two passes are sufficient for convergence in all practical cases.
    """
    warnings: list[str] = []

    # --- Extract drone inputs ---
    drone = inputs["drone"]
    m_struct = float(drone["m_struct_kg"])
    m_payload = float(drone["m_payload_kg"])
    configuration = str(drone["configuration"])
    twr_cible = float(drone["TWR_cible"])
    velocity_kmh = float(drone["vitesse_kmh"])
    velocity_ms = velocity_kmh / 3.6
    cell_count_S = int(drone["batterie_S"])
    diameter_inch = float(drone["helice_diametre_inch"])
    pitch_inch    = float(drone["helice_pas_inch"])

    # Inject cell count into phys so battery_model can access it
    phys["battery"]["cell_count_S"] = cell_count_S

    n_motors = get_motor_count(configuration)
    coaxial = is_coaxial(configuration)
    tlf = float(phys["coaxial_correction"]["thrust_loss_factor"])

    # Initial diameter estimate (16 inches — typical for inspection drones)
    _D_INIT_M = 16 * 0.0254

    # -----------------------------------------------------------------------
    # Pass 1 — bootstrap with estimated diameter
    # -----------------------------------------------------------------------
    n_eff_init = n_motors * tlf if coaxial else float(n_motors)

    battery_p1 = compute_battery(
        m_struct_kg=m_struct,
        m_payload_kg=m_payload,
        n_rotors_eff=n_eff_init,
        diameter_m=_D_INIT_M,
        phys=phys
    )
    masses_p1 = compute_masses(m_struct, m_payload, battery_p1.m_bat_optimal_kg)

    prop_p1 = compute_propeller(
        mtow_kg=masses_p1.mtow_kg,
        n_motors=n_motors,
        twr_cible=twr_cible,
        diameter_inch=diameter_inch,
        pitch_inch=pitch_inch,
        phys=phys
    )

    # -----------------------------------------------------------------------
    # Pass 2 — refine with actual diameter from pass 1
    # -----------------------------------------------------------------------
    battery = compute_battery(
        m_struct_kg=m_struct,
        m_payload_kg=m_payload,
        n_rotors_eff=prop_p1.n_rotors_eff,
        diameter_m=prop_p1.diameter_m,
        phys=phys
    )
    masses = compute_masses(m_struct, m_payload, battery.m_bat_optimal_kg)

    prop = compute_propeller(
        mtow_kg=masses.mtow_kg,
        n_motors=n_motors,
        twr_cible=twr_cible,
        diameter_inch=diameter_inch,
        pitch_inch=pitch_inch,
        phys=phys
    )

    # Collect propeller warnings
    warnings.extend(prop.warnings)

    # -----------------------------------------------------------------------
    # Power and TWR
    # -----------------------------------------------------------------------
    power_hover = compute_power_hover(
        mtow_kg=masses.mtow_kg,
        n_rotors_eff=prop.n_rotors_eff,
        diameter_m=prop.diameter_m,
        phys=phys
    )

    p_forward_W = compute_power_forward(
        mtow_kg=masses.mtow_kg,
        n_rotors_eff=prop.n_rotors_eff,
        diameter_m=prop.diameter_m,
        velocity_ms=velocity_ms,
        parasite_power_fraction=0.05,
        phys=phys
    )

    twr_result = compute_twr(
        mtow_kg=masses.mtow_kg,
        thrust_total_N=prop.thrust_total_N,
        phys=phys
    )

    if not twr_result.is_compliant:
        warnings.append(
            f"TWR {twr_result.twr:.3f} is outside the acceptable range "
            f"[{phys['constraints']['TWR_min']}, "
            f"{phys['constraints']['TWR_max']}]."
        )

    # Battery mass constraint check
    bat_max = float(phys["constraints"]["battery_mass_kg_max"])
    if masses.m_bat_kg > bat_max:
        warnings.append(
            f"Optimal battery mass ({masses.m_bat_kg:.3f} kg) exceeds "
            f"the structural maximum ({bat_max:.1f} kg)."
        )

    eta_global = get_effective_eta(phys["drivetrain"])

    # -----------------------------------------------------------------------
    # Motor sizing
    # -----------------------------------------------------------------------
    motor = compute_motor_sizing(
        p_hover_W=power_hover.p_hover_W,
        mtow_kg=masses.mtow_kg,
        n_motors=n_motors,
        n_rotors_eff=prop.n_rotors_eff,
        rpm_hover=prop.rpm_hover,
        thrust_per_motor_twr_N=prop.thrust_per_motor_N,
        twr_cible=twr_cible,
        diameter_m=prop.diameter_m,
        pitch_m=prop.pitch_hover_m,
        voltage_V=battery.voltage_V,
        phys=phys
    )

    # -----------------------------------------------------------------------
    # Curve generation
    # -----------------------------------------------------------------------
    svg_power = gen_curve_power(masses, prop, phys)
    svg_endurance_payload = gen_curve_endurance_payload(masses, battery, prop, phys)
    svg_endurance_battery = gen_curve_endurance_battery(masses, battery, prop, phys)
    svg_twr = gen_curve_twr(masses, battery, prop, phys)

    # -----------------------------------------------------------------------
    # Metadata
    # -----------------------------------------------------------------------
    try:
        from core.config_loader import _PHYSICAL_CONFIG_PATH
        with open(_PHYSICAL_CONFIG_PATH, "r", encoding="utf-8") as _f:
            _raw = json.load(_f)
        config_version = _raw.get("_metadata", {}).get("version", "N.A")
    except Exception:
        config_version = "N.A"
    report_date = date.today().isoformat()

    return ReportData(
        mission_name=inputs["mission"]["name"],
        mission_description=inputs["mission"]["description"],
        report_date=report_date,
        config_version=config_version,

        inputs=inputs,
        phys=phys,

        masses=masses,
        battery=battery,
        prop=prop,

        p_hover_W=power_hover.p_hover_W,
        p_forward_W=p_forward_W,
        twr_result=twr_result,
        eta_global=eta_global,
        voltage_V=battery.voltage_V,
        velocity_kmh=velocity_kmh,

        svg_power=svg_power,
        svg_endurance_payload=svg_endurance_payload,
        svg_endurance_battery=svg_endurance_battery,
        svg_twr=svg_twr,

        motor=motor,
        n_motors=n_motors,

        warnings=warnings,
    )