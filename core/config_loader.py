"""
core/config_loader.py
=====================
Loading and validation module for drone sizing tool configuration files.

This module is responsible for:
- Loading the physical configuration file (physical_config.json)
- Loading the mission input file provided via CLI
- Validating the presence and type of all required fields
- Validating physical consistency and constraint bounds
- Exposing clean, validated Python dictionaries to the rest of the application

All keys starting with '_' (underscore) are treated as documentation/comments
and are stripped from the loaded data before it is returned.

Usage:
    from core.config_loader import load_physical_config, load_mission_inputs

    phys = load_physical_config()
    inputs = load_mission_inputs("inputs/heliox_x1_baseline.json")

Raises:
    FileNotFoundError: If a required JSON file cannot be found.
    ValueError: If a required field is missing, has the wrong type,
                or violates a physical consistency rule.
"""

import json
import os
from typing import Any

# ---------------------------------------------------------------------------
# Path to the physical configuration file — fixed in source code.
# ---------------------------------------------------------------------------
_PHYSICAL_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "physical_config.json"
)

# ---------------------------------------------------------------------------
# Supported drone configurations
# ---------------------------------------------------------------------------
SUPPORTED_CONFIGURATIONS = ("quad", "hexa", "octo", "x8")

# Number of motors per configuration
MOTORS_PER_CONFIGURATION: dict[str, int] = {
    "quad": 4,
    "hexa": 6,
    "octo": 8,
    "x8": 8,
}

# Configurations that use coaxial rotor pairs
COAXIAL_CONFIGURATIONS = ("x8",)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _load_json(filepath: str) -> dict:
    """Load a JSON file from disk and return its content as a dictionary.

    Args:
        filepath (str): Absolute or relative path to the JSON file.

    Returns:
        dict: Parsed JSON content.

    Raises:
        FileNotFoundError: If the file does not exist at the given path.
        ValueError: If the file content is not valid JSON.
    """
    if not os.path.isfile(filepath):
        raise FileNotFoundError(
            f"Configuration file not found: '{filepath}'"
        )
    with open(filepath, "r", encoding="utf-8") as f:
        try:
            return json.load(f)
        except json.JSONDecodeError as e:
            raise ValueError(
                f"Invalid JSON in file '{filepath}': {e}"
            ) from e


def _strip_comments(data: Any) -> Any:
    """Recursively remove all documentation keys from a loaded JSON object.

    Keys starting with '_' are treated as inline comments or metadata
    and are stripped before the data is used by the application.

    Args:
        data (Any): A parsed JSON value (dict, list, or scalar).

    Returns:
        Any: The same structure with all underscore-prefixed keys removed.
    """
    if isinstance(data, dict):
        return {
            k: _strip_comments(v)
            for k, v in data.items()
            if not k.startswith("_")
        }
    if isinstance(data, list):
        return [_strip_comments(item) for item in data]
    return data


def _require_field(data: dict, path: str, expected_type) -> Any:
    """Extract and type-check a required field from a nested dictionary.

    The field path uses dot notation to navigate nested keys.
    Example: 'atmosphere.rho_kg_m3' maps to data['atmosphere']['rho_kg_m3'].

    Args:
        data (dict): The dictionary to search in.
        path (str): Dot-separated key path to the required field.
        expected_type: The expected Python type or tuple of types.

    Returns:
        Any: The validated field value.

    Raises:
        ValueError: If the field is missing or has an unexpected type.
    """
    keys = path.split(".")
    current = data
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            raise ValueError(
                f"Required field '{path}' is missing from configuration."
            )
        current = current[key]
    if not isinstance(current, expected_type):
        type_name = (
            expected_type.__name__
            if isinstance(expected_type, type)
            else str(expected_type)
        )
        raise ValueError(
            f"Field '{path}' must be of type {type_name}, "
            f"got {type(current).__name__} instead."
        )
    return current


def _require_positive(value: float, field_name: str) -> None:
    """Assert that a numeric value is strictly positive.

    Args:
        value (float): The value to check.
        field_name (str): Field name used in the error message.

    Raises:
        ValueError: If the value is zero or negative.
    """
    if value <= 0:
        raise ValueError(
            f"Field '{field_name}' must be strictly positive, got {value}."
        )


def _require_in_range(
    value: float,
    field_name: str,
    low: float,
    high: float,
    inclusive: bool = True
) -> None:
    """Assert that a numeric value falls within a specified range.

    Args:
        value (float): The value to check.
        field_name (str): Field name used in the error message.
        low (float): Lower bound of the acceptable range.
        high (float): Upper bound of the acceptable range.
        inclusive (bool): Whether the bounds are inclusive. Default True.

    Raises:
        ValueError: If the value is outside the specified range.
    """
    in_range = (low <= value <= high) if inclusive else (low < value < high)
    if not in_range:
        bounds = f"[{low}, {high}]" if inclusive else f"({low}, {high})"
        raise ValueError(
            f"Field '{field_name}' must be in range {bounds}, got {value}."
        )


# ---------------------------------------------------------------------------
# Physical configuration loader
# ---------------------------------------------------------------------------

def load_physical_config() -> dict:
    """Load and validate the physical configuration file.

    Reads the file located at the fixed path defined by _PHYSICAL_CONFIG_PATH,
    strips all documentation keys (underscore-prefixed), and validates all
    required fields for type, presence, and physical consistency.

    Returns:
        dict: Validated physical configuration with the following top-level keys:
            - atmosphere (dict): rho_kg_m3, g_m_s2, altitude_m
            - propeller_model (dict): CT, CP, slip_factor, pitch ratios,
                                      diameter bounds
            - battery (dict): energy_density_Wh_kg, cell_voltage_nominal_V,
                              depth_of_discharge, mass_fraction bounds
            - drivetrain (dict): eta_motor, eta_esc, eta_propeller, eta_global
            - coaxial_correction (dict): enabled, thrust_loss_factor
            - constraints (dict): TWR bounds, battery mass max,
                                  propeller diameter max

    Raises:
        FileNotFoundError: If physical_config.json cannot be found.
        ValueError: If any required field is missing, wrongly typed,
                    or physically inconsistent.
    """
    raw = _load_json(_PHYSICAL_CONFIG_PATH)
    config = _strip_comments(raw)

    # --- atmosphere ---
    rho = _require_field(config, "atmosphere.rho_kg_m3", (int, float))
    _require_positive(rho, "atmosphere.rho_kg_m3")

    g = _require_field(config, "atmosphere.g_m_s2", (int, float))
    _require_positive(g, "atmosphere.g_m_s2")

    alt = _require_field(config, "atmosphere.altitude_m", (int, float))
    if alt < 0:
        raise ValueError("Field 'atmosphere.altitude_m' must be >= 0.")

    # --- propeller model ---
    CT = _require_field(config, "propeller_model.CT", (int, float))
    _require_in_range(CT, "propeller_model.CT", 0.005, 0.025)

    CP = _require_field(config, "propeller_model.CP", (int, float))
    _require_in_range(CP, "propeller_model.CP", 0.002, 0.015)

    slip = _require_field(config, "propeller_model.slip_factor", (int, float))
    _require_in_range(slip, "propeller_model.slip_factor", 0.0, 0.5)

    p_hover = _require_field(
        config, "propeller_model.pitch_to_diameter_ratio_hover", (int, float)
    )
    _require_in_range(
        p_hover, "propeller_model.pitch_to_diameter_ratio_hover", 0.3, 1.0
    )

    p_speed = _require_field(
        config, "propeller_model.pitch_to_diameter_ratio_speed", (int, float)
    )
    _require_in_range(
        p_speed, "propeller_model.pitch_to_diameter_ratio_speed", 0.3, 1.5
    )

    d_min = _require_field(config, "propeller_model.diameter_min_inch", (int, float))
    d_max = _require_field(config, "propeller_model.diameter_max_inch", (int, float))
    _require_positive(d_min, "propeller_model.diameter_min_inch")
    _require_positive(d_max, "propeller_model.diameter_max_inch")
    if d_min >= d_max:
        raise ValueError(
            f"'propeller_model.diameter_min_inch' ({d_min}) must be "
            f"strictly less than 'diameter_max_inch' ({d_max})."
        )

    # --- battery ---
    e_sp = _require_field(config, "battery.energy_density_Wh_kg", (int, float))
    _require_positive(e_sp, "battery.energy_density_Wh_kg")

    v_cell = _require_field(config, "battery.cell_voltage_nominal_V", (int, float))
    _require_positive(v_cell, "battery.cell_voltage_nominal_V")

    dod = _require_field(config, "battery.depth_of_discharge", (int, float))
    _require_in_range(dod, "battery.depth_of_discharge", 0.0, 1.0, inclusive=False)

    mf_min = _require_field(config, "battery.mass_fraction_min", (int, float))
    mf_max = _require_field(config, "battery.mass_fraction_max", (int, float))
    _require_in_range(mf_min, "battery.mass_fraction_min", 0.0, 1.0, inclusive=False)
    _require_in_range(mf_max, "battery.mass_fraction_max", 0.0, 1.0, inclusive=False)
    if mf_min >= mf_max:
        raise ValueError(
            f"'battery.mass_fraction_min' ({mf_min}) must be "
            f"strictly less than 'battery.mass_fraction_max' ({mf_max})."
        )

    # --- drivetrain ---
    eta_motor = _require_field(config, "drivetrain.eta_motor", (int, float))
    _require_in_range(eta_motor, "drivetrain.eta_motor", 0.0, 1.0, inclusive=False)

    eta_esc = _require_field(config, "drivetrain.eta_esc", (int, float))
    _require_in_range(eta_esc, "drivetrain.eta_esc", 0.0, 1.0, inclusive=False)

    eta_prop = _require_field(config, "drivetrain.eta_propeller", (int, float))
    _require_in_range(eta_prop, "drivetrain.eta_propeller", 0.0, 1.0, inclusive=False)

    eta_global = config.get("drivetrain", {}).get("eta_global")
    if eta_global is not None:
        if not isinstance(eta_global, (int, float)):
            raise ValueError("'drivetrain.eta_global' must be a float or null.")
        _require_in_range(
            eta_global, "drivetrain.eta_global", 0.0, 1.0, inclusive=False
        )

    # --- coaxial correction ---
    _require_field(config, "coaxial_correction.enabled", bool)
    tlf = _require_field(
        config, "coaxial_correction.thrust_loss_factor", (int, float)
    )
    _require_in_range(tlf, "coaxial_correction.thrust_loss_factor", 0.5, 1.0)

    # --- constraints ---
    twr_min = _require_field(config, "constraints.TWR_min", (int, float))
    twr_max = _require_field(config, "constraints.TWR_max", (int, float))
    _require_positive(twr_min, "constraints.TWR_min")
    _require_positive(twr_max, "constraints.TWR_max")
    if twr_min >= twr_max:
        raise ValueError(
            f"'constraints.TWR_min' ({twr_min}) must be "
            f"strictly less than 'constraints.TWR_max' ({twr_max})."
        )

    bat_max = _require_field(
        config, "constraints.battery_mass_kg_max", (int, float)
    )
    _require_positive(bat_max, "constraints.battery_mass_kg_max")

    prop_max = _require_field(
        config, "constraints.propeller_diameter_max_inch", (int, float)
    )
    _require_positive(prop_max, "constraints.propeller_diameter_max_inch")
    if prop_max > d_max:
        raise ValueError(
            f"'constraints.propeller_diameter_max_inch' ({prop_max}) exceeds "
            f"'propeller_model.diameter_max_inch' ({d_max})."
        )

    return config


# ---------------------------------------------------------------------------
# Mission input loader
# ---------------------------------------------------------------------------

def load_mission_inputs(filepath: str) -> dict:
    """Load and validate the mission input file provided via CLI.

    Reads the mission JSON file, strips all documentation keys, and validates
    all required fields for type, presence, and physical consistency.

    Args:
        filepath (str): Path to the mission input JSON file, as provided
                        by the user on the command line.

    Returns:
        dict: Validated mission inputs with the following top-level keys:
            - mission (dict): name, description
            - drone (dict): m_struct_kg, m_payload_kg, configuration,
                            TWR_cible, vitesse_kmh, batterie_S
            - output (dict): format

    Raises:
        FileNotFoundError: If the mission input file cannot be found.
        ValueError: If any required field is missing, wrongly typed,
                    or physically inconsistent.
    """
    raw = _load_json(filepath)
    inputs = _strip_comments(raw)

    # --- mission ---
    _require_field(inputs, "mission.name", str)
    _require_field(inputs, "mission.description", str)

    # --- drone ---
    m_struct = _require_field(inputs, "drone.m_struct_kg", (int, float))
    _require_positive(m_struct, "drone.m_struct_kg")

    m_payload = _require_field(inputs, "drone.m_payload_kg", (int, float))
    if m_payload < 0:
        raise ValueError("Field 'drone.m_payload_kg' must be >= 0.")

    configuration = _require_field(inputs, "drone.configuration", str)
    if configuration not in SUPPORTED_CONFIGURATIONS:
        raise ValueError(
            f"Field 'drone.configuration' must be one of "
            f"{SUPPORTED_CONFIGURATIONS}, got '{configuration}'."
        )

    twr = _require_field(inputs, "drone.TWR_cible", (int, float))
    _require_in_range(twr, "drone.TWR_cible", 1.0, 5.0)

    speed = _require_field(inputs, "drone.vitesse_kmh", (int, float))
    _require_positive(speed, "drone.vitesse_kmh")

    s_count = _require_field(inputs, "drone.batterie_S", int)
    _require_in_range(s_count, "drone.batterie_S", 1, 24)

    # --- propeller geometry (mandatory, from manufacturer datasheet) ---
    d_inch = _require_field(inputs, "drone.helice_diametre_inch", (int, float))
    _require_in_range(d_inch, "drone.helice_diametre_inch", 3.0, 50.0)

    p_inch = _require_field(inputs, "drone.helice_pas_inch", (int, float))
    _require_in_range(p_inch, "drone.helice_pas_inch", 1.0, 40.0)

    # --- output ---
    fmt = _require_field(inputs, "output.format", str)
    if fmt not in ("html", "pdf"):
        raise ValueError(
            f"Field 'output.format' must be 'html' or 'pdf', got '{fmt}'."
        )

    return inputs


# ---------------------------------------------------------------------------
# Convenience accessors
# ---------------------------------------------------------------------------

def get_motor_count(configuration: str) -> int:
    """Return the number of motors for a given drone configuration.

    Args:
        configuration (str): One of 'quad', 'hexa', 'octo', 'x8'.

    Returns:
        int: Number of motors.

    Raises:
        ValueError: If the configuration is not supported.
    """
    if configuration not in MOTORS_PER_CONFIGURATION:
        raise ValueError(
            f"Unsupported configuration '{configuration}'. "
            f"Must be one of {SUPPORTED_CONFIGURATIONS}."
        )
    return MOTORS_PER_CONFIGURATION[configuration]


def is_coaxial(configuration: str) -> bool:
    """Return whether a given configuration uses coaxial rotor pairs.

    Coaxial configurations apply a thrust loss correction factor to account
    for the aerodynamic interference between upper and lower rotors.

    Args:
        configuration (str): One of 'quad', 'hexa', 'octo', 'x8'.

    Returns:
        bool: True if coaxial (x8), False otherwise.
    """
    return configuration in COAXIAL_CONFIGURATIONS


def get_effective_eta(drivetrain: dict) -> float:
    """Compute the effective global drivetrain efficiency.

    If 'eta_global' is explicitly set (non-null) in the drivetrain config,
    it takes priority over the product of individual efficiencies.
    Otherwise:
        eta_global = eta_motor x eta_esc x eta_propeller

    Args:
        drivetrain (dict): The 'drivetrain' block from physical_config.json,
                           after stripping comment keys.

    Returns:
        float: Effective global drivetrain efficiency in range (0, 1).

    Example:
        >>> dt = {"eta_motor": 0.88, "eta_esc": 0.95, "eta_propeller": 0.75,
        ...       "eta_global": None}
        >>> get_effective_eta(dt)
        0.6270000000000001

        >>> dt["eta_global"] = 0.70
        >>> get_effective_eta(dt)
        0.70
    """
    if drivetrain.get("eta_global") is not None:
        return float(drivetrain["eta_global"])
    return (
        float(drivetrain["eta_motor"])
        * float(drivetrain["eta_esc"])
        * float(drivetrain["eta_propeller"])
    )