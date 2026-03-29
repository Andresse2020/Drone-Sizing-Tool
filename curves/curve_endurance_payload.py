"""
curves/curve_endurance_payload.py
===================================
Endurance vs payload mass curve for the drone sizing report.

Generates a matplotlib figure showing how flight endurance decreases as
payload increases, with battery mass held at its optimal value.

Physical background:
    With fixed optimal battery mass, any increase in payload increases MTOW
    and hover power, reducing endurance. This curve quantifies the payload
    penalty and helps identify the maximum useful payload.

Usage:
    from curves.curve_endurance_payload import generate

    svg = generate(masses, battery, prop, phys)
"""

import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from core.mass_model import MassBreakdown
from core.battery_model import BatteryResult, compute_endurance_minutes
from core.propeller_model import PropellerResult
from core.config_loader import get_effective_eta
from curves.curve_utils import (
    apply_style, fig_to_svg, PALETTE, FIGURE_SIZE, FONT_SIZE_ANNOTATION
)


def generate(
    masses: MassBreakdown,
    battery: BatteryResult,
    prop: PropellerResult,
    phys: dict,
    payload_range_factor: float = 3.0,
    n_points: int = 300
) -> str:
    """Generate the endurance vs payload mass curve as an SVG string.

    Sweeps payload from 0 kg to payload_range_factor times the nominal
    mission payload, with battery mass fixed at its optimal value.

    Args:
        masses (MassBreakdown): Computed mass breakdown from mass_model.
        battery (BatteryResult): Battery sizing result from battery_model.
                                 Provides optimal battery mass.
        prop (PropellerResult): Propeller sizing result. Provides diameter
                                and n_rotors_eff.
        phys (dict): Validated physical configuration dictionary.
        payload_range_factor (float): Upper bound of the payload sweep as
                                      a multiple of the nominal payload.
                                      Default 3.0. Minimum sweep is 10 kg.
        n_points (int): Number of evaluation points. Default 300.

    Returns:
        str: Self-contained SVG string ready for HTML embedding.
    """
    atm = phys["atmosphere"]
    bat = phys["battery"]
    rho = float(atm["rho_kg_m3"])
    g = float(atm["g_m_s2"])
    e_sp = float(bat["energy_density_Wh_kg"])
    dod = float(bat["depth_of_discharge"])
    eta = get_effective_eta(phys["drivetrain"])
    disk_area = math.pi * (prop.diameter_m / 2.0) ** 2

    payload_max = max(masses.m_payload_kg * payload_range_factor, 10.0)
    payload_range = np.linspace(0.0, payload_max, n_points)

    endurance_curve = np.array([
        compute_endurance_minutes(
            m_bat_kg=battery.m_bat_optimal_kg,
            m_struct_kg=masses.m_struct_kg,
            m_payload_kg=float(mp),
            n_rotors_eff=prop.n_rotors_eff,
            disk_area_m2=disk_area,
            energy_density_Wh_kg=e_sp,
            depth_of_discharge=dod,
            rho=rho, g=g, eta_global=eta
        )
        for mp in payload_range
    ])

    mission_endurance = compute_endurance_minutes(
        m_bat_kg=battery.m_bat_optimal_kg,
        m_struct_kg=masses.m_struct_kg,
        m_payload_kg=masses.m_payload_kg,
        n_rotors_eff=prop.n_rotors_eff,
        disk_area_m2=disk_area,
        energy_density_Wh_kg=e_sp,
        depth_of_discharge=dod,
        rho=rho, g=g, eta_global=eta
    )

    fig, ax = plt.subplots(figsize=FIGURE_SIZE)

    ax.plot(payload_range, endurance_curve,
            color=PALETTE["green"], linewidth=2.5,
            label="Endurance", zorder=4)

    ax.scatter([masses.m_payload_kg], [mission_endurance],
               color=PALETTE["red"], s=80, zorder=5,
               label=f"Mission payload: {masses.m_payload_kg:.2f} kg "
                     f"→ {mission_endurance:.1f} min")

    ax.annotate(
        f" {mission_endurance:.1f} min",
        xy=(masses.m_payload_kg, mission_endurance),
        fontsize=FONT_SIZE_ANNOTATION,
        color=PALETTE["red"],
        va="bottom"
    )

    ax.set_ylim(bottom=0)

    apply_style(
        fig, ax,
        title="Endurance vs Payload Mass  (battery at optimal mass)",
        xlabel="Payload mass [kg]",
        ylabel="Endurance [min]"
    )

    svg = fig_to_svg(fig)
    plt.close(fig)
    return svg
