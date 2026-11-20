import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SCEN = os.path.join(HERE, '..', 'scenarios', 'lio_faults.yaml')


def test_scenario_codes_exist_in_lio_doctor():
    pytest.importorskip('yaml')
    import yaml
    from lio_doctor import core
    src = open(core.__file__).read()
    for f in yaml.safe_load(open(SCEN))['faults']:
        for code in f['expect']['detect']:
            assert f"'{code}'" in src, f'{code} is not a lio_doctor finding'


def test_scenario_parses_with_fault_inject():
    pytest.importorskip('yaml')
    fi = pytest.importorskip('nav2_fault_inject.faults')
    from nav2_fault_inject.check import normalize
    faults = fi.parse_scenario_full(open(SCEN).read())
    assert len(faults) == 3
    for f in faults:
        assert normalize(f['expect'])['detect']
