"""
curves/curve_power.py
=====================
Hover power vs MTOW curve for the drone sizing report.

Generates a matplotlib figure showing how required hover power scales with
total take-off mass. The current mission MTOW is marked as a reference point.

Physical background:
    From actuator disk theory, hover power scales as MTOW^(3/2):

        P_hover ∝ MTOW^(3/2)

    Doubling the MTOW increases hover power by ~2.83x, making weight
    reduction highly impactful for endurance.

Usage:
    from curves.curve_power import generate

    svg = generate(masses, prop, phys)
    # svg is a self-contained SVG string for HTML embedding
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from core.mass_model import MassBreakdown
from core.propeller_model import PropellerResult
from core.power_model import power_curve_vs_mtow, compute_power_hover
from curves.curve_utils import (
    apply_style, fig_to_svg, PALETTE, FIGURE_SIZE, FONT_SIZE_ANNOTATION
)


def generate(
    masses: MassBreakdown,
    prop: PropellerResult,
    phys: dict,
    mtow_range_factor: float = 2.0,
    n_points: int = 300
) -> str:
    """Generate the hover power vs MTOW curve as an SVG string.

    Sweeps MTOW from half the structural mass up to mtow_range_factor times
    the current mission MTOW, and marks the mission operating point.

    Args:
        masses (MassBreakdown): Computed mass breakdown from mass_model.
        prop (PropellerResult): Propeller sizing result. Provides diameter
                                and n_rotors_eff.
        phys (dict): Validated physical configuration dictionary.
        mtow_range_factor (float): Upper bound multiplier on the current
                                   MTOW for the sweep range. Default 2.0.
        n_points (int): Number of evaluation points. Default 300.

    Returns:
        str: Self-contained SVG string ready for HTML embedding.
    """
    mtow_min = masses.m_struct_kg * 0.5
    mtow_max = masses.mtow_kg * mtow_range_factor
    mtow_range = np.linspace(mtow_min, mtow_max, n_points)

    power_w = power_curve_vs_mtow(
        mtow_range_kg=mtow_range,
        n_rotors_eff=prop.n_rotors_eff,
        diameter_m=prop.diameter_m,
        phys=phys
    )
    power_kw = power_w / 1000.0

    mission_power_w = compute_power_hover(
        mtow_kg=masses.mtow_kg,
        n_rotors_eff=prop.n_rotors_eff,
        diameter_m=prop.diameter_m,
        phys=phys
    ).p_hover_W
    mission_power_kw = mission_power_w / 1000.0

    fig, ax = plt.subplots(figsize=FIGURE_SIZE)

    ax.plot(mtow_range, power_kw,
            color=PALETTE["blue"], linewidth=2.5,
            label="Hover power", zorder=4)

    ax.scatter([masses.mtow_kg], [mission_power_kw],
               color=PALETTE["red"], s=80, zorder=5,
               label=f"Mission MTOW: {masses.mtow_kg:.2f} kg "
                     f"→ {mission_power_kw:.2f} kW")

    ax.annotate(
        f" {mission_power_kw:.2f} kW",
        xy=(masses.mtow_kg, mission_power_kw),
        fontsize=FONT_SIZE_ANNOTATION,
        color=PALETTE["red"],
        va="bottom"
    )

    apply_style(
        fig, ax,
        title="Hover Power vs Total Take-Off Mass",
        xlabel="MTOW [kg]",
        ylabel="Hover Power [kW]"
    )

    svg = fig_to_svg(fig)
    plt.close(fig)
    return svg
