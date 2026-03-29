"""
curves/curve_endurance_battery.py
===================================
Endurance vs battery mass curve with optimal point marker.

Generates a matplotlib figure showing the unimodal endurance curve as a
function of battery mass. The optimal battery mass (peak endurance) is
highlighted with a star marker, a vertical dashed line, and an annotation.

Physical background:
    The endurance function t(m_bat) has a single maximum:
    - Left of the peak: adding battery mass increases energy faster than
      the MTOW penalty increases power consumption.
    - Right of the peak: the MTOW penalty dominates and endurance falls.

    The optimal battery mass is where the marginal energy gain equals the
    marginal power increase due to added weight.

Usage:
    from curves.curve_endurance_battery import generate

    svg = generate(masses, battery, prop, phys)
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from core.mass_model import MassBreakdown
from core.battery_model import BatteryResult
from core.propeller_model import PropellerResult
from curves.curve_utils import (
    apply_style, fig_to_svg, add_vline,
    PALETTE, FIGURE_SIZE, FONT_SIZE_ANNOTATION
)


def generate(
    masses: MassBreakdown,
    battery: BatteryResult,
    prop: PropellerResult,
    phys: dict
) -> str:
    """Generate the endurance vs battery mass curve as an SVG string.

    Uses the pre-computed curve arrays from the battery model result and
    extends the sweep 40% beyond the optimal battery mass to clearly show
    the declining endurance trend.

    Args:
        masses (MassBreakdown): Computed mass breakdown from mass_model.
                                Used to compute extended curve values.
        battery (BatteryResult): Battery sizing result. Provides the
                                 pre-computed curve arrays (m_bat_range_kg,
                                 endurance_curve_min) and optimal values.
        prop (PropellerResult): Propeller sizing result. Provides diameter
                                and n_rotors_eff for extended curve computation.
        phys (dict): Validated physical configuration dictionary.
                     Used to read battery_mass_kg_max constraint.

    Returns:
        str: Self-contained SVG string ready for HTML embedding.
    """
    import math as _math
    import numpy as _np
    from core.battery_model import compute_endurance_minutes
    from core.config_loader import get_effective_eta

    m_range_base = battery.m_bat_range_kg
    e_curve_base = battery.endurance_curve_min
    m_opt = battery.m_bat_optimal_kg
    e_max = battery.endurance_max_min
    bat_max = float(phys["constraints"]["battery_mass_kg_max"])

    # Extend range 40% beyond optimal point to show the declining trend
    m_extended_end = max(float(m_range_base[-1]), m_opt * 1.4)
    if m_extended_end > float(m_range_base[-1]):
        eta = get_effective_eta(phys["drivetrain"])
        disk_area = _math.pi * (prop.diameter_m / 2.0) ** 2
        m_extra = _np.linspace(float(m_range_base[-1]), m_extended_end, 80)[1:]
        e_extra = _np.array([
            compute_endurance_minutes(
                m_bat_kg=float(m),
                m_struct_kg=masses.m_struct_kg,
                m_payload_kg=masses.m_payload_kg,
                n_rotors_eff=prop.n_rotors_eff,
                disk_area_m2=disk_area,
                energy_density_Wh_kg=float(phys["battery"]["energy_density_Wh_kg"]),
                depth_of_discharge=float(phys["battery"]["depth_of_discharge"]),
                rho=float(phys["atmosphere"]["rho_kg_m3"]),
                g=float(phys["atmosphere"]["g_m_s2"]),
                eta_global=eta
            ) for m in m_extra
        ])
        m_range = _np.concatenate([m_range_base, m_extra])
        e_curve = _np.concatenate([e_curve_base, e_extra])
    else:
        m_range = m_range_base
        e_curve = e_curve_base

    fig, ax = plt.subplots(figsize=FIGURE_SIZE)

    # Filled area under curve
    ax.fill_between(m_range, e_curve,
                    alpha=0.10, color=PALETTE["orange"], zorder=1)

    # Main curve
    ax.plot(m_range, e_curve,
            color=PALETTE["orange"], linewidth=2.5,
            label="Endurance", zorder=4)

    # Optimal point
    ax.scatter([m_opt], [e_max],
               color=PALETTE["red"], s=120,
               marker="*", zorder=6,
               label=f"Optimal: {m_opt:.3f} kg → {e_max:.1f} min")

    # Vertical line at optimal mass
    ax.axvline(x=m_opt, color=PALETTE["red"],
               linestyle="--", linewidth=1.4, zorder=3)
    ax.annotate(
        f"  {m_opt:.2f} kg\n  {e_max:.1f} min",
        xy=(m_opt, e_max * 0.60),
        fontsize=FONT_SIZE_ANNOTATION,
        color=PALETTE["red"], va="center"
    )

    # Battery mass constraint line (if in range)
    if float(m_range[0]) <= bat_max <= float(m_range[-1]):
        ax.axvline(x=bat_max, color=PALETTE["gray"],
                   linestyle=":", linewidth=1.2, zorder=3)
        ax.annotate(
            f"  max {bat_max:.1f} kg",
            xy=(bat_max, e_curve.max() * 0.15),
            fontsize=FONT_SIZE_ANNOTATION,
            color=PALETTE["gray"], va="bottom"
        )

    ax.set_ylim(bottom=0)

    apply_style(
        fig, ax,
        title="Endurance vs Battery Mass",
        xlabel="Battery mass [kg]",
        ylabel="Endurance [min]"
    )

    svg = fig_to_svg(fig)
    plt.close(fig)
    return svg