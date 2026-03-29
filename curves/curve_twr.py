"""
curves/curve_twr.py
====================
Effective TWR vs battery mass curve for the drone sizing report.

Generates a matplotlib figure showing how the effective thrust-to-weight
ratio decreases as battery mass increases. Constraint bounds (TWR_min,
TWR_max) are shown as horizontal reference lines with colored shading.

Physical background:
    Since motor thrust is fixed at design point, adding battery mass
    increases MTOW linearly, causing TWR to decrease monotonically:

        TWR(m_bat) = T_total / ((m_struct + m_bat + m_payload) * g)

    The intersection of the TWR curve with TWR_min defines the maximum
    battery mass that still provides adequate maneuverability.

Usage:
    from curves.curve_twr import generate

    svg = generate(masses, battery, prop, phys)
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from core.mass_model import MassBreakdown
from core.battery_model import BatteryResult
from core.propeller_model import PropellerResult
from core.power_model import twr_curve_vs_bat_mass
from curves.curve_utils import (
    apply_style, fig_to_svg,
    PALETTE, FIGURE_SIZE, FONT_SIZE_ANNOTATION
)


def generate(
    masses: MassBreakdown,
    battery: BatteryResult,
    prop: PropellerResult,
    phys: dict
) -> str:
    """Generate the effective TWR vs battery mass curve as an SVG string.

    Uses the same battery mass range as the endurance curve for visual
    consistency across report sections.

    Args:
        masses (MassBreakdown): Computed mass breakdown from mass_model.
        battery (BatteryResult): Battery sizing result from battery_model.
                                 Provides the battery mass range array and
                                 optimal battery mass.
        prop (PropellerResult): Propeller sizing result. Provides thrust per
                                motor values for TWR calculation.
        phys (dict): Validated physical configuration dictionary.
                     Used to read TWR_min and TWR_max constraints.

    Returns:
        str: Self-contained SVG string ready for HTML embedding.
    """
    import numpy as _np

    m_range_base = battery.m_bat_range_kg
    m_opt = battery.m_bat_optimal_kg

    # Extend range 40% beyond optimal point to show the declining trend
    m_extended_end = max(float(m_range_base[-1]), m_opt * 1.4)
    if m_extended_end > float(m_range_base[-1]):
        m_extra = _np.linspace(float(m_range_base[-1]), m_extended_end, 80)[1:]
        m_range = _np.concatenate([m_range_base, m_extra])
    else:
        m_range = m_range_base
    twr_min = float(phys["constraints"]["TWR_min"])
    twr_max = float(phys["constraints"]["TWR_max"])
    g = float(phys["atmosphere"]["g_m_s2"])

    coaxial = bool(phys["coaxial_correction"]["enabled"])
    thrust_loss = float(phys["coaxial_correction"]["thrust_loss_factor"])

    # Infer motor count from effective rotor count
    if coaxial:
        n_motors = int(round(prop.n_rotors_eff / thrust_loss))
    else:
        n_motors = int(round(prop.n_rotors_eff))

    twr_values = twr_curve_vs_bat_mass(
        m_bat_range_kg=m_range,
        m_struct_kg=masses.m_struct_kg,
        m_payload_kg=masses.m_payload_kg,
        thrust_per_motor_N=prop.thrust_per_motor_N,
        n_motors=n_motors,
        coaxial=coaxial,
        thrust_loss_factor=thrust_loss,
        phys=phys
    )

    # Mission operating point
    if coaxial:
        t_total = n_motors * prop.thrust_per_motor_N * thrust_loss
    else:
        t_total = n_motors * prop.thrust_per_motor_N
    twr_mission = t_total / (masses.mtow_kg * g)

    y_max = max(float(np.max(twr_values)) * 1.15, twr_max + 0.5)

    fig, ax = plt.subplots(figsize=FIGURE_SIZE)

    # Unsafe region shading (below TWR_min)
    ax.axhspan(0, twr_min,
               alpha=0.07, color=PALETTE["red"], zorder=1)
    ax.text(
        float(m_range[-1]) * 0.98, twr_min * 0.5,
        "Unsafe region",
        fontsize=FONT_SIZE_ANNOTATION, color=PALETTE["red"],
        ha="right", va="center"
    )

    # Oversized region shading (above TWR_max)
    ax.axhspan(twr_max, y_max,
               alpha=0.06, color=PALETTE["light_blue"], zorder=1)
    ax.text(
        float(m_range[-1]) * 0.98, twr_max + (y_max - twr_max) * 0.5,
        "Oversized region",
        fontsize=FONT_SIZE_ANNOTATION, color="#6baed6",
        ha="right", va="center"
    )

    # Main TWR curve
    ax.plot(m_range, twr_values,
            color=PALETTE["purple"], linewidth=2.5,
            label="Effective TWR", zorder=4)

    # TWR_min line
    ax.axhline(y=twr_min, color=PALETTE["red"],
               linestyle="--", linewidth=1.4, zorder=3,
               label=f"TWR_min = {twr_min}")

    # TWR_max line
    ax.axhline(y=twr_max, color=PALETTE["light_blue"],
               linestyle=":", linewidth=1.4, zorder=3,
               label=f"TWR_max = {twr_max}")

    # Optimal battery mass vertical reference
    ax.axvline(x=m_opt, color=PALETTE["orange"],
               linestyle="--", linewidth=1.3, zorder=3)
    ax.annotate(
        f"  opt bat\n  {m_opt:.2f} kg",
        xy=(m_opt, twr_min + (twr_max - twr_min) * 0.15),
        fontsize=FONT_SIZE_ANNOTATION,
        color=PALETTE["orange"], va="bottom"
    )

    # Mission operating point
    ax.scatter([masses.m_bat_kg], [twr_mission],
               color=PALETTE["red"], s=80, zorder=6,
               label=f"Mission: {masses.m_bat_kg:.3f} kg → TWR {twr_mission:.2f}")

    ax.set_ylim(0, y_max)

    apply_style(
        fig, ax,
        title="Effective TWR vs Battery Mass",
        xlabel="Battery mass [kg]",
        ylabel="Thrust-to-Weight Ratio [-]"
    )

    svg = fig_to_svg(fig)
    plt.close(fig)
    return svg