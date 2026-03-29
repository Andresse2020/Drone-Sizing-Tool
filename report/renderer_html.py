"""
report/renderer_html.py
=======================
HTML report renderer for the drone sizing tool.

This module generates a fully self-contained HTML report from a ReportData
object. The output file:
- Contains no external dependencies (CSS, fonts, scripts are all inline)
- Embeds all four SVG curves directly in the HTML body
- Uses a clean, professional layout suitable for engineering review
- Is named automatically as {mission_name}_{YYYY-MM-DD}.html

Report sections:
    1. Page header       — mission name, date, config version
    2. Input summary     — all mission parameters in a two-column table
    3. Synthesis table   — all six masses + key results with compliance badges
    4. Propeller results — diameter and pitch
    5. Battery results   — optimal mass, voltage, autonomie
    6. Curves (4)        — power, endurance/payload, endurance/battery, TWR
    7. Model hypotheses  — assumptions and limitations
    8. Warnings          — constraint violations and clamping notices

Usage:
    from report.renderer_html import render

    output_path = render(data, output_dir=".")
    print(f"Report written to: {output_path}")
"""

import os
from datetime import date

from report.builder import ReportData


# ---------------------------------------------------------------------------
# Badge helpers
# ---------------------------------------------------------------------------

def _badge_ok(text: str) -> str:
    """Return an HTML green OK badge."""
    return (
        f'<span style="background:#22c55e;color:#fff;padding:2px 8px;'
        f'border-radius:4px;font-size:0.78em;font-weight:600">{text}</span>'
    )


def _badge_warn(text: str) -> str:
    """Return an HTML orange WARNING badge."""
    return (
        f'<span style="background:#f59e0b;color:#fff;padding:2px 8px;'
        f'border-radius:4px;font-size:0.78em;font-weight:600">{text}</span>'
    )


def _badge_error(text: str) -> str:
    """Return an HTML red ERROR badge."""
    return (
        f'<span style="background:#ef4444;color:#fff;padding:2px 8px;'
        f'border-radius:4px;font-size:0.78em;font-weight:600">{text}</span>'
    )


def _compliance_badge(ok: bool, ok_label: str = "OK",
                      fail_label: str = "VIOLATION") -> str:
    """Return a compliance badge based on a boolean condition."""
    return _badge_ok(ok_label) if ok else _badge_error(fail_label)


# ---------------------------------------------------------------------------
# CSS
# ---------------------------------------------------------------------------

_CSS = """
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
    font-family: 'Segoe UI', Arial, sans-serif;
    font-size: 13px;
    color: #222;
    background: #f5f6fa;
    padding: 24px 16px;
}
.page { max-width: 960px; margin: 0 auto; }

/* Header */
.header {
    background: #1e293b;
    color: #f8fafc;
    border-radius: 8px;
    padding: 24px 28px;
    margin-bottom: 20px;
}
.header h1 { font-size: 1.45em; font-weight: 700; letter-spacing: 0.01em; }
.header .subtitle { font-size: 0.9em; color: #94a3b8; margin-top: 6px; }
.header .meta { font-size: 0.82em; color: #64748b; margin-top: 10px; }
.header .meta span { margin-right: 18px; }

/* Sections */
.section {
    background: #fff;
    border-radius: 8px;
    box-shadow: 0 1px 3px rgba(0,0,0,0.07);
    margin-bottom: 18px;
    overflow: hidden;
}
.section-title {
    background: #f1f5f9;
    border-bottom: 1px solid #e2e8f0;
    padding: 10px 18px;
    font-size: 0.92em;
    font-weight: 700;
    color: #1e293b;
    letter-spacing: 0.03em;
    text-transform: uppercase;
}
.section-body { padding: 16px 18px; }

/* Tables */
table {
    width: 100%;
    border-collapse: collapse;
    font-size: 0.9em;
}
th {
    background: #f8fafc;
    color: #475569;
    text-align: left;
    padding: 7px 12px;
    border-bottom: 1px solid #e2e8f0;
    font-weight: 600;
    font-size: 0.82em;
    letter-spacing: 0.04em;
    text-transform: uppercase;
}
td {
    padding: 7px 12px;
    border-bottom: 1px solid #f1f5f9;
    color: #334155;
}
tr:last-child td { border-bottom: none; }
tr:nth-child(even) td { background: #fafbfc; }
.td-value { font-weight: 600; color: #1e293b; font-variant-numeric: tabular-nums; }
.td-unit  { color: #94a3b8; font-size: 0.85em; }

/* Two-column layout */
.two-col { display: grid; grid-template-columns: 1fr 1fr; gap: 18px; }

/* Curves */
.curve-container { margin-bottom: 10px; }
.curve-container svg { width: 100%; height: auto; display: block; }

/* Warnings */
.warning-list { list-style: none; }
.warning-list li {
    padding: 8px 12px;
    margin-bottom: 6px;
    border-radius: 5px;
    font-size: 0.88em;
    background: #fef3c7;
    border-left: 3px solid #f59e0b;
    color: #78350f;
}
.warning-list li.error {
    background: #fee2e2;
    border-left-color: #ef4444;
    color: #7f1d1d;
}
.no-warnings {
    color: #16a34a;
    font-size: 0.9em;
    padding: 6px 0;
}

/* Hypotheses */
.hypothesis-list { list-style: none; }
.hypothesis-list li {
    padding: 5px 0 5px 14px;
    border-left: 2px solid #e2e8f0;
    margin-bottom: 4px;
    color: #475569;
    font-size: 0.88em;
}

/* Footer */
.footer {
    text-align: center;
    color: #94a3b8;
    font-size: 0.78em;
    margin-top: 24px;
    padding-top: 12px;
    border-top: 1px solid #e2e8f0;
}
"""


# ---------------------------------------------------------------------------
# Section builders
# ---------------------------------------------------------------------------

def _section(title: str, body: str) -> str:
    """Wrap content in a styled section block."""
    return (
        f'<div class="section">'
        f'<div class="section-title">{title}</div>'
        f'<div class="section-body">{body}</div>'
        f'</div>'
    )


def _table_row(label: str, value: str, unit: str = "",
               badge: str = "") -> str:
    """Return a single HTML table row."""
    badge_html = f"&nbsp;{badge}" if badge else ""
    return (
        f"<tr>"
        f"<td>{label}</td>"
        f'<td class="td-value">{value}{badge_html}</td>'
        f'<td class="td-unit">{unit}</td>'
        f"</tr>"
    )


def _build_header(data: ReportData) -> str:
    return (
        f'<div class="header">'
        f'<h1>{data.mission_name}</h1>'
        f'<div class="subtitle">{data.mission_description}</div>'
        f'<div class="meta">'
        f'<span>📅 {data.report_date}</span>'
        f'<span>⚙️ physical_config v{data.config_version}</span>'
        f'</div>'
        f'</div>'
    )


def _build_inputs(data: ReportData) -> str:
    drone = data.inputs["drone"]
    rows = (
        _table_row("Structural mass", f"{drone['m_struct_kg']:.3f}", "kg") +
        _table_row("Payload mass", f"{drone['m_payload_kg']:.3f}", "kg") +
        _table_row("Configuration", str(drone["configuration"])) +
        _table_row("Target TWR", f"{drone['TWR_cible']:.2f}", "-") +
        _table_row("Cruise speed", f"{drone['vitesse_kmh']:.1f}", "km/h") +
        _table_row("Battery", f"{drone['batterie_S']}S",
                   f"({data.voltage_V:.1f} V nominal)")
    )
    return _section(
        "Mission Inputs",
        f"<table><thead><tr><th>Parameter</th><th>Value</th>"
        f"<th>Unit</th></tr></thead><tbody>{rows}</tbody></table>"
    )


def _build_synthesis(data: ReportData) -> str:
    m = data.masses
    twr = data.twr_result
    bat_max = float(data.phys["constraints"]["battery_mass_kg_max"])

    rows = (
        _table_row("Structural mass (m_struct)",
                   f"{m.m_struct_kg:.3f}", "kg") +
        _table_row("Battery mass — optimal (m_bat)",
                   f"{m.m_bat_kg:.3f}", "kg",
                   _compliance_badge(m.m_bat_kg <= bat_max,
                                     "OK", "EXCEEDS MAX")) +
        _table_row("Payload mass (m_payload)",
                   f"{m.m_payload_kg:.3f}", "kg") +
        "<tr><td colspan='3' style='padding:4px 0'></td></tr>" +
        _table_row("With payload, no battery (m_charge_sans_bat)",
                   f"{m.m_charge_sans_bat_kg:.3f}", "kg") +
        _table_row("Ferry config — no payload (m_vol_sans_charge)",
                   f"{m.m_vol_sans_charge_kg:.3f}", "kg") +
        _table_row("MTOW — full operational",
                   f"{m.mtow_kg:.3f}", "kg") +
        "<tr><td colspan='3' style='padding:4px 0'></td></tr>" +
        _table_row("Hover power",
                   f"{data.p_hover_W:.0f}", "W") +
        _table_row(f"Forward flight power ({data.velocity_kmh:.0f} km/h)",
                   f"{data.p_forward_W:.0f}", "W") +
        _table_row("Max endurance",
                   f"{data.battery.endurance_max_min:.1f}", "min") +
        _table_row("Effective TWR",
                   f"{twr.twr:.3f}", "-",
                   _compliance_badge(twr.is_compliant)) +
        _table_row("Global drivetrain efficiency (η)",
                   f"{data.eta_global:.4f}", "-")
    )
    return _section(
        "Sizing Results — Summary",
        f"<table><thead><tr><th>Parameter</th><th>Value</th>"
        f"<th>Unit / Status</th></tr></thead><tbody>{rows}</tbody></table>"
    )


def _build_propeller(data: ReportData) -> str:
    p = data.prop
    rows = (
        _table_row("Propeller diameter",
                   f"{p.diameter_inch:.2f}", "inches",
                   _badge_warn("CLAMPED") if p.diameter_clamped else "") +
        _table_row("Propeller diameter (SI)",
                   f"{p.diameter_m * 100:.1f}", "cm") +
        _table_row("Optimal pitch — hover",
                   f"{p.pitch_hover_inch:.2f}", "inches") +
        _table_row("Optimal pitch — cruise speed",
                   f"{p.pitch_speed_inch:.2f}", "inches") +
        _table_row("Thrust per motor (at target TWR)",
                   f"{p.thrust_per_motor_N:.2f}", "N") +
        _table_row("Total thrust (at target TWR)",
                   f"{p.thrust_total_N:.2f}", "N") +
        _table_row("Effective rotor count",
                   f"{p.n_rotors_eff:.2f}", "-")
    )
    return _section(
        "Propeller Sizing",
        f"<table><thead><tr><th>Parameter</th><th>Value</th>"
        f"<th>Unit</th></tr></thead><tbody>{rows}</tbody></table>"
    )


def _build_battery(data: ReportData) -> str:
    b = data.battery
    bat = data.phys["battery"]
    rows = (
        _table_row("Optimal battery mass",
                   f"{b.m_bat_optimal_kg:.3f}", "kg") +
        _table_row("Maximum endurance",
                   f"{b.endurance_max_min:.1f}", "min") +
        _table_row("Effective pack voltage",
                   f"{b.voltage_V:.1f}", "V") +
        _table_row("Cell count",
                   f"{bat['cell_count_S']}S", "") +
        _table_row("Specific energy",
                   f"{bat['energy_density_Wh_kg']:.0f}", "Wh/kg") +
        _table_row("Depth of discharge",
                   f"{bat['depth_of_discharge'] * 100:.0f}", "%") +
        _table_row("Available energy",
                   f"{b.m_bat_optimal_kg * bat['energy_density_Wh_kg'] * bat['depth_of_discharge']:.1f}",
                   "Wh")
    )
    return _section(
        "Battery Sizing",
        f"<table><thead><tr><th>Parameter</th><th>Value</th>"
        f"<th>Unit</th></tr></thead><tbody>{rows}</tbody></table>"
    )



def _build_motor_sizing(data: ReportData) -> str:
    """Build the motor sizing HTML section with hover vs TWR columns."""
    m = data.motor

    def row3(label: str, v_hover: str, v_twr: str, unit: str = "") -> str:
        """Return a three-column table row: label | hover | TWR | unit."""
        return (
            f"<tr>"
            f"<td>{label}</td>"
            f'<td class="td-value">{v_hover}</td>'
            f'<td class="td-value">{v_twr}</td>'
            f'<td class="td-unit">{unit}</td>'
            f"</tr>"
        )

    rows = (
        row3("Electrical power per motor",
             f"{m.p_elec_hover_W:.1f}",
             f"{m.p_elec_twr_W:.1f}", "W") +
        row3("Shaft power per motor",
             f"{m.p_shaft_hover_W:.1f}",
             f"{m.p_shaft_twr_W:.1f}", "W") +
        row3("Current per motor",
             f"{m.current_hover_A:.1f}",
             f"{m.current_twr_A:.1f}", "A") +
        row3("Torque per motor",
             f"{m.torque_hover_Nm:.3f}",
             f"{m.torque_twr_Nm:.3f}", "N·m") +
        row3("Thrust per motor",
             f"{m.thrust_hover_N:.2f} N / {m.thrust_hover_g:.0f} g",
             f"{m.thrust_twr_N:.2f} N / {m.thrust_twr_g:.0f} g", "") +
        row3("RPM",
             f"{m.rpm_hover:.0f}",
             f"{m.rpm_twr:.0f}", "RPM")
    )

    kv_row = (
        f'<tr><td colspan="4" style="padding:10px 12px 4px;'
        f'font-weight:600;color:#1e293b;font-size:0.85em;'
        f'text-transform:uppercase;letter-spacing:0.04em;">'
        f'KV Recommendation (based on TWR peak)</td></tr>'
        f'<tr>'
        f'<td>Recommended KV</td>'
        f'<td class="td-value" colspan="2">{m.kv_recommended:.0f} '
        f'<span style="color:#94a3b8;font-size:0.85em;font-weight:400">'
        f'range [{m.kv_range_min:.0f} — {m.kv_range_max:.0f}]</span></td>'
        f'<td class="td-unit">RPM/V</td>'
        f'</tr>'
        f'<tr>'
        f'<td>Pack voltage</td>'
        f'<td class="td-value" colspan="2">{m.voltage_V:.1f}</td>'
        f'<td class="td-unit">V</td>'
        f'</tr>'
    )

    note = (
        '<p style="margin-top:10px;font-size:0.82em;color:#64748b;">'
        '<strong>Hover</strong> = continuous rating (sustained level flight at MTOW). '
        '<strong>TWR cible</strong> = peak / transient rating (climb, wind rejection). '
        'Select a motor rated ≥ 20% above the TWR peak values. '
        'KV assumes 85% no-load to operating RPM ratio.'
        '</p>'
    )

    header = (
        "<thead><tr>"
        "<th>Parameter</th>"
        '<th style="background:#dbeafe;color:#1e40af">Hover</th>'
        '<th style="background:#fce7f3;color:#9d174d">TWR cible</th>'
        "<th>Unit</th>"
        "</tr></thead>"
    )

    return _section(
        "Motor Sizing",
        f"<table>{header}<tbody>{rows}{kv_row}</tbody></table>{note}"
    )

def _build_curves(data: ReportData) -> str:
    curves = [
        ("Hover Power vs MTOW", data.svg_power),
        ("Endurance vs Payload Mass", data.svg_endurance_payload),
        ("Endurance vs Battery Mass", data.svg_endurance_battery),
        ("Effective TWR vs Battery Mass", data.svg_twr),
    ]
    html = '<div class="two-col">'
    for title, svg in curves:
        html += (
            f'<div class="section">'
            f'<div class="section-title">{title}</div>'
            f'<div class="section-body">'
            f'<div class="curve-container">{svg}</div>'
            f'</div></div>'
        )
    html += "</div>"
    return html


def _build_hypotheses(data: ReportData) -> str:
    coaxial = bool(data.phys["coaxial_correction"]["enabled"])
    tlf = data.phys["coaxial_correction"]["thrust_loss_factor"]
    alt = data.phys["atmosphere"]["altitude_m"]

    items = [
        "Hover flight only for power and endurance calculations (conservative worst-case).",
        f"Standard ISA atmosphere at {alt} m altitude "
        f"(ρ = {data.phys['atmosphere']['rho_kg_m3']} kg/m³).",
        f"Global drivetrain efficiency η = {data.eta_global:.4f} "
        f"(constant over the full operating range).",
        f"Battery specific energy = {data.phys['battery']['energy_density_Wh_kg']} Wh/kg "
        f"(usable DoD = {data.phys['battery']['depth_of_discharge']*100:.0f}%).",
        "Optimal battery mass determined by numerical endurance maximisation "
        "within configured mass fraction bounds.",
        f"Propeller model: hybrid analytical (actuator disk) + CT/CP coefficients "
        f"(CT = {data.phys['propeller_model']['CT']}, "
        f"CP = {data.phys['propeller_model']['CP']}).",
    ]
    if coaxial:
        items.append(
            f"Coaxial configuration: lower rotor thrust loss factor = {tlf} "
            f"(applied to effective rotor count and thrust calculations)."
        )

    li_html = "".join(f"<li>{item}</li>" for item in items)
    return _section(
        "Model Hypotheses and Limitations",
        f'<ul class="hypothesis-list">{li_html}</ul>'
    )


def _build_warnings(data: ReportData) -> str:
    if not data.warnings:
        body = '<p class="no-warnings">✅ No warnings — all constraints satisfied.</p>'
    else:
        items = ""
        for w in data.warnings:
            css_class = "error" if "WARNING" in w.upper() else ""
            items += f'<li class="{css_class}">{w}</li>'
        body = f'<ul class="warning-list">{items}</ul>'
    return _section("Warnings and Constraint Violations", body)


def _build_footer() -> str:
    return (
        f'<div class="footer">'
        f'Generated by drone_sizing — '
        f'physical_config.json + mission inputs · {date.today().isoformat()}'
        f'</div>'
    )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def render(data: ReportData, output_dir: str = ".") -> str:
    """Render the sizing report as a self-contained HTML file.

    Assembles all report sections into a single HTML file with inline CSS
    and embedded SVG curves. No external resources are required.

    Args:
        data (ReportData): Fully populated report data from builder.build().
        output_dir (str): Directory where the HTML file will be written.
                          Default is the current working directory.

    Returns:
        str: Absolute path to the generated HTML file.

    Output filename:
        {mission_name}_{YYYY-MM-DD}.html
        Example: HELIOX_X1_baseline_2026-03-22.html
    """
    os.makedirs(output_dir, exist_ok=True)

    filename = f"{data.mission_name}_{data.report_date}.html"
    output_path = os.path.join(output_dir, filename)

    body = (
        _build_header(data) +
        _build_inputs(data) +
        _build_synthesis(data) +
        '<div class="two-col">' +
        _build_propeller(data) +
        _build_battery(data) +
        '</div>' +
        _build_motor_sizing(data) +
        _build_curves(data) +
        _build_hypotheses(data) +
        _build_warnings(data) +
        _build_footer()
    )

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Drone Sizing Report — {data.mission_name}</title>
<style>
{_CSS}
</style>
</head>
<body>
<div class="page">
{body}
</div>
</body>
</html>"""

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(html)

    return os.path.abspath(output_path)