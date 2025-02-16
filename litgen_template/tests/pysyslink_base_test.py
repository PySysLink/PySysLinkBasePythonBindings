import pysyslink_base


def test_add():
    assert pysyslink_base.add(1, 2) == 3
    assert pysyslink_base.add(4, 5, 6) == 15


def test_version():
    assert pysyslink_base.__version__ == "0.0.1"
