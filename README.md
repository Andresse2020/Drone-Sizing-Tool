# Drone Sizing Tool

A command-line tool for sizing multirotor drones. Given a mission input file, it runs a full physics-based sizing pipeline (propulsion, battery, mass, power) and produces a self-contained HTML or PDF report with four performance curves.

---

## Requirements

- Python 3.10 or higher
- `numpy` and `matplotlib` (see `requirements.txt`)
- **PDF output only:** `wkhtmltopdf` must be installed on your system

Install Python dependencies:

```bash
pip install -r requirements.txt
```

Install `wkhtmltopdf` (optional, PDF only):

```bash
# Ubuntu / Debian
sudo apt install wkhtmltopdf

# macOS (Homebrew)
brew install wkhtmltopdf
```

---

## Project Structure

```
drone_sizing/
├── drone_sizing.py          ← Entry point (run this)
├── physical_config.json     ← Physical model parameters
├── requirements.txt
├── inputs/
│   ├── heliox_x1.json       ← Example: X8 coaxial octorotor
│   └── tundra_2.json        ← Example: quadcopter
├── output/                  ← Generated reports land here
├── core/                    ← Physics models (battery, propeller, power…)
├── curves/                  ← SVG curve generators (matplotlib)
└── report/                  ← Report builder and HTML/PDF renderers
```

---

## Quick Start

```bash
# Generate the HELIOX X1 report (HTML)
python drone_sizing.py inputs/heliox_x1.json --output-dir output/

# Generate the TUNDRA 2 report (HTML)
python drone_sizing.py inputs/tundra_2.json --output-dir output/
```

The report is written to the `output/` directory. Open the `.html` file in any browser.

---

## Command Reference

### Show help

```bash
python drone_sizing.py --help
```

Prints the full list of arguments and usage examples, then exits. Use this any time you are unsure about the syntax.

```
python drone_sizing.py -h
```

Short form, identical result.

---

### Run a sizing

```bash
python drone_sizing.py <inputs_file> [--output-dir <dir>]
```

| Argument | Required | Description |
|---|---|---|
| `inputs_file` | **Yes** | Path to your mission JSON file. |
| `--output-dir DIR` | No | Directory where the report will be written. <br> Defaults to the current directory (`.`) if not specified. |

**Examples:**

```bash
# Minimal — report written to the current directory
python drone_sizing.py inputs/heliox_x1.json

# With explicit output directory
python drone_sizing.py inputs/heliox_x1.json --output-dir output/

# Different mission, different output folder
python drone_sizing.py inputs/tundra_2.json --output-dir reports/tundra/
```

> The output directory is created automatically if it does not exist.

---

### Generate a PDF instead of HTML

Set `"format": "pdf"` in the `output` block of your mission JSON file, then run the command as usual:

```bash
python drone_sizing.py inputs/heliox_x1.json --output-dir output/
```

> `wkhtmltopdf` must be installed on your system for PDF output to work (see Requirements above).

---

## Creating Your Own Mission File

Create a `.json` file in the `inputs/` folder. Use one of the provided examples as a template.

```json
{
  "mission": {
    "name": "MY_DRONE",
    "description": "Short description of the mission."
  },

  "drone": {
    "m_struct_kg": 5.0,
    "m_payload_kg": 2.0,
    "configuration": "x8",
    "TWR_cible": 2.0,
    "vitesse_kmh": 45,
    "batterie_S": 6,
    "helice_diametre_inch": 22,
    "helice_pas_inch": 7.0
  },

  "output": {
    "format": "html"
  }
}
```

### Field Reference

| Field | Type | Description |
|---|---|---|
| `m_struct_kg` | float | Structural mass (airframe, motors, ESCs, electronics). <br> **Excludes** battery and payload. |
| `m_payload_kg` | float | Payload mass (sensors, gimbal, cameras). **Excludes** battery. |
| `configuration` | string | Rotor layout: `"quad"`, `"hexa"`, `"octo"`, or `"x8"` (coaxial). |
| `TWR_cible` | float | Target thrust-to-weight ratio at MTOW. Typical: `1.8` to `2.5`. |
| `vitesse_kmh` | float | Nominal cruise speed in km/h. |
| `batterie_S` | int | Number of LiPo cells in series (e.g. `6` for 6S → 22.2 V). |
| `helice_diametre_inch` | float | Propeller diameter in inches (from manufacturer datasheet). |
| `helice_pas_inch` | float | Propeller pitch in inches (from manufacturer datasheet). |
| `output.format` | string | Report format: `"html"` or `"pdf"`. |

---

## Output Report

The report includes:

- **Mass breakdown** — structural, battery, payload, MTOW
- **Propulsion summary** — diameter, pitch, thrust per motor, effective TWR
- **Battery summary** — voltage, optimal mass, maximum endurance
- **Motor sizing** — power, current, torque, and KV recommendation at two operating points (hover + peak TWR)
- **4 performance curves:**
  - Hover power vs MTOW
  - Endurance vs payload mass
  - Endurance vs battery mass (with optimal point)
  - Effective TWR vs battery mass

Any constraint violations are listed as warnings at the top of the report.

---

## Tuning the Physical Model

All physical constants are stored in `physical_config.json` at the project root. You can adjust:

| Parameter | Location | Description |
|---|---|---|
| `CT`, `CP` | `propeller_model` | Thrust and power coefficients. Calibrate to your propeller family. |
| `energy_density_Wh_kg` | `battery` | Battery specific energy. Typical LiPo: 200–260 Wh/kg. |
| `depth_of_discharge` | `battery` | Usable fraction of battery energy (default: 0.8). |
| `eta_motor`, `eta_esc`, `eta_propeller` | `drivetrain` | Individual efficiencies. Or set `eta_global` to override all three. |
| `thrust_loss_factor` | `coaxial_correction` | Coaxial thrust efficiency (default: 0.85). Applied to `x8` only. |
| `TWR_min`, `TWR_max` | `constraints` | Acceptable TWR range for constraint checking. |

> **Note:** Do not modify `physical_config.json` unless you understand the physical model. Wrong values will silently produce incorrect results.

---

## Supported Configurations

| Value | Motors | Layout |
|---|---|---|
| `"quad"` | 4 | Flat quadcopter |
| `"hexa"` | 6 | Flat hexacopter |
| `"octo"` | 8 | Flat octocopter |
| `"x8"` | 8 | Coaxial X8 (4 pairs) |

For `x8`, a coaxial thrust loss correction is applied automatically (configurable via `thrust_loss_factor`).

---

## Exit Codes

| Code | Meaning |
|---|---|
| `0` | Success |
| `1` | Computation or configuration error |
| `2` | Usage error (wrong arguments) |

---

## Example Output (CLI)

```
=== Drone Sizing Tool ===

[ 1/5 ] Loading configuration files...
        ✓ physical_config.json loaded
        ✓ inputs/heliox_x1.json loaded  [HELIOX_X1_5030]
[ 2/5 ] Running sizing computations...
        ✓ MTOW           = 12.700 kg
        ✓ m_bat optimal  = 3.000 kg
        ✓ Diameter       = 22.0 in
        ✓ Endurance max  = 38.4 min
        ✓ TWR effective  = 1.984
[ 3/5 ] No warnings — all constraints satisfied.
[ 4/5 ] Curves generated (4/4):
        ✓ Hover power vs MTOW
        ✓ Endurance vs payload mass
        ✓ Endurance vs battery mass
        ✓ Effective TWR vs battery mass
[ 5/5 ] Rendering HTML report...
        ✓ Report written: output/HELIOX_X1_5030_2026-03-29.html  (312.4 kB)

Done.
```