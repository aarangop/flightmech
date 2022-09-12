import pytest as pytest

from flightmech.utils import c, meters


def test_c():
    assert c(0) == pytest.approx(661 * 0.5144, 1)
    assert c(meters(40e3)) == pytest.approx(573 * 0.5144, 1)
