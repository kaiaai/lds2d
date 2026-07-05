"""Registry-wide policy tests: every model is selectable by a manufacturer-
qualified name, and the ambiguous bare model codes (X4, A1, …) that could clash
across vendors are NOT registered on their own — they require a prefix."""
import pytest

from lds2d.core import _REGISTRY, driver_for

# Manufacturer prefixes that make a name unambiguous.
_PREFIXES = ("LDROBOT-", "LDROBOT_", "3IROBOTIX-", "3IROBOTIX_", "XIAOMI-",
             "XIAOMI_", "NEATO-", "NEATO_", "YDLIDAR-", "YDLIDAR_",
             "RPLIDAR-", "RPLIDAR_", "CAMSENSE-", "CAMSENSE_", "HLS-", "HLS_",
             "COIN-", "COIN_")


def test_every_model_has_a_manufacturer_qualified_name():
    # Invert the registry to class -> [names] and check each class is reachable
    # by at least one manufacturer-qualified alias.
    names_by_cls = {}
    for name, cls in _REGISTRY.items():
        names_by_cls.setdefault(cls, []).append(name)
    for cls, names in names_by_cls.items():
        assert any(n.startswith(_PREFIXES) for n in names), \
            f"{cls.MODEL_NAME} has no manufacturer-qualified name: {names}"


@pytest.mark.parametrize("bare", [
    "X1", "X2", "X2L", "X3", "X4", "X3-PRO", "X3_PRO", "X4-PRO", "X4_PRO",
    "A1", "C1", "SCL", "TMINI", "T-MINI",
])
def test_ambiguous_bare_codes_are_not_registered(bare):
    # These generic codes are reused across vendors; they must be qualified
    # (e.g. YDLIDAR-X4, RPLIDAR-A1, CAMSENSE-X1) to avoid silent clashes.
    assert driver_for(bare) is None


@pytest.mark.parametrize("name,model", [
    ("LDROBOT-LD14P", "LDROBOT LD14P"),
    ("3IROBOTIX-DELTA-2A", "3irobotix Delta-2A"),
    ("XIAOMI-LDS02RR", "Xiaomi LDS02RR"),
    ("YDLIDAR-X4", "YDLIDAR X4"),
    ("RPLIDAR-A1", "RPLIDAR A1"),
    ("CAMSENSE-X1", "Camsense X1"),
    ("HLS-LFCD2", "Hitachi-LG HLS-LFCD2"),
])
def test_canonical_names_resolve(name, model):
    cls = driver_for(name)
    assert cls is not None and cls.MODEL_NAME == model


def test_names_are_case_insensitive():
    assert driver_for("ldrobot-ld14p") is driver_for("LDROBOT-LD14P")
