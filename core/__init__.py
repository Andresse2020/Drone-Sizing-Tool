"""
core/
=====
Physical sizing models for the multirotor drone sizing tool.

This package contains all the physics-based computation modules.
Each module is independent and can be used individually or through
the report/builder.py orchestrator.

Modules:
    config_loader     : JSON loading, validation, and convenience accessors
    battery_model     : Battery sizing, voltage, and endurance computation
    mass_model        : Mass breakdown from struct + payload + battery
    propeller_model   : Optimal propeller diameter and pitch (hybrid model)
    power_model       : Hover and forward flight power, TWR computation

Dependency order (must be respected):
    config_loader
        └── battery_model   (needs rho, g, eta, battery params)
            └── mass_model  (needs m_bat from battery_model)
                └── propeller_model  (needs MTOW from mass_model)
                    └── power_model  (needs diameter, n_rotors_eff)
"""
