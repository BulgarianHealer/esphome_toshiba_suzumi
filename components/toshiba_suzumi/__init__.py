from esphome.components import select
import importlib.metadata
from packaging.version import parse as V

def _select_schema_base():
    """
    Return the right cv.Schema object regardless of ESPHome version.
    """
    # 1.  Feature-detection (fast, works inside HA add-on containers too)
    if hasattr(select, "select_schema"):
        return select.select_schema()        # New helper exists – use it.

    # 2.  Fallback: we are on an older build that still has the constant.
    return select.SELECT_SCHEMA

ESP_VERSION = V(importlib.metadata.version("esphome"))