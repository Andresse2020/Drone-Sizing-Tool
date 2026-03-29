"""
curves/
=======
Curve generation modules for the drone sizing report.

Each module exposes a single public function ``generate(...)`` that returns
a self-contained SVG string ready for HTML embedding.

Available modules:
    curve_utils               : Shared matplotlib style helpers and constants
    curve_power               : Hover power vs MTOW
    curve_endurance_payload   : Endurance vs payload mass
    curve_endurance_battery   : Endurance vs battery mass (with optimal point)
    curve_twr                 : Effective TWR vs battery mass
"""
