import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('guard', Path(__file__).parents[1]/'src/localization_guard.py')
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


def test_only_fresh_localized_inputs_allow_motion():
    ready = guard.localization_ready
    assert ready('LOCALIZED', .1, .1, .1, 1., .5)
    for state in [None, 'LOST', 'WAITING_FOR_POSE', 'NO_MAP', 'UNKNOWN']:
        assert not ready(state, .1, .1, .1, 1., .5)
    for ages in [(1.1,.1,.1),(.1,.6,.1),(.1,.1,.6),(.1,.1,-1),(.1,.1,float('nan'))]:
        assert not ready('LOCALIZED', *ages, 1., .5)
