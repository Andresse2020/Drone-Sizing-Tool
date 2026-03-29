"""
curves/curve_utils.py
=====================
Shared style utilities for all curve generation modules.

All curves in the drone sizing report use a consistent visual style:
- Clean white background with subtle grid
- Consistent color palette
- Standardized fonts, margins, and figure size
- SVG export for HTML embedding (no raster quality loss)

This module provides:
- apply_style(): Apply consistent style to a matplotlib Figure and Axes
- fig_to_svg(): Export a matplotlib Figure to an SVG string for HTML embedding
- PALETTE: Shared color constants

Usage:
    from curves.curve_utils import apply_style, fig_to_svg, PALETTE
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()
    # ... plot content ...
    apply_style(fig, ax, title="My Chart", xlabel="X", ylabel="Y")
    svg_str = fig_to_svg(fig)
    plt.close(fig)
"""

import io
import matplotlib
matplotlib.use("Agg")   # Non-interactive backend — no display required
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


# ---------------------------------------------------------------------------
# Shared color palette
# ---------------------------------------------------------------------------
PALETTE = {
    "blue":       "#1f77b4",
    "orange":     "#ff7f0e",
    "green":      "#2ca02c",
    "red":        "#d62728",
    "purple":     "#9467bd",
    "gray":       "#7f7f7f",
    "light_blue": "#aec7e8",
    "grid":       "#e8e8e8",
    "bg":         "#ffffff",
    "text":       "#222222",
    "subtext":    "#555555",
}

# Figure dimensions (width x height in inches)
FIGURE_SIZE = (9.0, 4.5)

# Font settings
FONT_FAMILY = "DejaVu Sans"
FONT_SIZE_TITLE = 13
FONT_SIZE_LABEL = 11
FONT_SIZE_TICK = 10
FONT_SIZE_LEGEND = 10
FONT_SIZE_ANNOTATION = 9


# ---------------------------------------------------------------------------
# Style application
# ---------------------------------------------------------------------------

def apply_style(
    fig: "plt.Figure",
    ax: "plt.Axes",
    title: str,
    xlabel: str,
    ylabel: str,
    legend: bool = True
) -> None:
    """Apply the standard drone sizing report style to a figure.

    Sets background colors, grid, fonts, labels, and tick formatting.
    Should be called after all plot traces have been added to the axes.

    Args:
        fig (plt.Figure): Matplotlib figure object.
        ax (plt.Axes): Matplotlib axes object to style.
        title (str): Chart title displayed above the axes.
        xlabel (str): X-axis label.
        ylabel (str): Y-axis label.
        legend (bool): Whether to display the legend. Default True.
    """
    fig.patch.set_facecolor(PALETTE["bg"])
    ax.set_facecolor(PALETTE["bg"])

    ax.set_title(title, fontsize=FONT_SIZE_TITLE, fontweight="bold",
                 color=PALETTE["text"], pad=12, fontfamily=FONT_FAMILY)
    ax.set_xlabel(xlabel, fontsize=FONT_SIZE_LABEL, color=PALETTE["subtext"],
                  fontfamily=FONT_FAMILY)
    ax.set_ylabel(ylabel, fontsize=FONT_SIZE_LABEL, color=PALETTE["subtext"],
                  fontfamily=FONT_FAMILY)

    ax.grid(True, color=PALETTE["grid"], linewidth=0.8, linestyle="-")
    ax.set_axisbelow(True)

    for spine in ax.spines.values():
        spine.set_edgecolor(PALETTE["grid"])
        spine.set_linewidth(0.8)

    ax.tick_params(
        axis="both",
        labelsize=FONT_SIZE_TICK,
        labelcolor=PALETTE["subtext"],
        colors=PALETTE["subtext"]
    )

    if legend:
        leg = ax.legend(
            fontsize=FONT_SIZE_LEGEND,
            frameon=True,
            framealpha=0.9,
            edgecolor=PALETTE["grid"],
            facecolor=PALETTE["bg"]
        )
        for text in leg.get_texts():
            text.set_color(PALETTE["subtext"])

    fig.tight_layout(pad=1.5)


def fig_to_svg(fig: "plt.Figure") -> str:
    """Export a matplotlib figure to an SVG string.

    The SVG string can be embedded directly in an HTML file without any
    external file references or base64 encoding. This makes the output
    report fully self-contained.

    Args:
        fig (plt.Figure): Matplotlib figure to export.

    Returns:
        str: SVG content as a UTF-8 string, ready for HTML embedding.
             Includes the full <svg>...</svg> element.
    """
    buffer = io.BytesIO()
    fig.savefig(buffer, format="svg", bbox_inches="tight",
                facecolor=fig.get_facecolor())
    buffer.seek(0)
    svg_bytes = buffer.read()
    buffer.close()
    return svg_bytes.decode("utf-8")


def add_vline(
    ax: "plt.Axes",
    x: float,
    label: str,
    color: str,
    linestyle: str = "--",
    linewidth: float = 1.4
) -> None:
    """Add a labeled vertical reference line to an axes.

    Args:
        ax (plt.Axes): Target axes.
        x (float): X position of the vertical line.
        label (str): Text label displayed at the top of the line.
        color (str): Line and label color (hex or named color).
        linestyle (str): Matplotlib linestyle string. Default '--'.
        linewidth (float): Line width. Default 1.4.
    """
    ax.axvline(x=x, color=color, linestyle=linestyle,
               linewidth=linewidth, zorder=3)
    ylim = ax.get_ylim()
    ax.text(
        x, ylim[1] * 0.97, f" {label}",
        color=color, fontsize=FONT_SIZE_ANNOTATION,
        va="top", ha="left", fontfamily=FONT_FAMILY
    )


def add_hline(
    ax: "plt.Axes",
    y: float,
    label: str,
    color: str,
    linestyle: str = "--",
    linewidth: float = 1.4
) -> None:
    """Add a labeled horizontal reference line to an axes.

    Args:
        ax (plt.Axes): Target axes.
        y (float): Y position of the horizontal line.
        label (str): Text label displayed at the right of the line.
        color (str): Line and label color (hex or named color).
        linestyle (str): Matplotlib linestyle string. Default '--'.
        linewidth (float): Line width. Default 1.4.
    """
    ax.axhline(y=y, color=color, linestyle=linestyle,
               linewidth=linewidth, zorder=3)
    xlim = ax.get_xlim()
    ax.text(
        xlim[1] * 0.98, y, f"{label} ",
        color=color, fontsize=FONT_SIZE_ANNOTATION,
        va="bottom", ha="right", fontfamily=FONT_FAMILY
    )
