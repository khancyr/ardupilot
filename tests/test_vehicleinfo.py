"""Checks for Tools/autotest/pysim/vehicleinfo.json.

sim_vehicle.py and autotest read it from the source tree, and SITL embeds
it in ROMFS and looks up a model's default parameter files in it at startup
(SITL_State::resolve_defaults_from_romfs).  Check that it is strict JSON,
that it has the shape they rely on, and that the files it names exist and,
for SITL's defaults, are embedded in ROMFS.

AP_FLAKE8_CLEAN
"""

import fnmatch
import json
import os

import pytest

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
AUTOTEST = os.path.join(ROOT, 'Tools', 'autotest')
VEHICLEINFO = os.path.join(AUTOTEST, 'pysim', 'vehicleinfo.json')
SCRIPTING_EXAMPLES = os.path.join(ROOT, 'libraries', 'AP_Scripting', 'examples')

# the files SITL embeds in ROMFS from these Tools/autotest directories, see
# SITLBoard.configure_env() in Tools/ardupilotwaf/boards.py
SITL_ROMFS_PATTERNS = {
    'default_params': ['*.parm'],
    'models': ['*.json', '*.parm', 'mt11_*.h264'],
}

# the frame keys in use, and the types they must have.  A key missing from
# here is most likely a typo, which every reader would silently ignore
STR = (str,)
FRAME_KEYS = {
    'waf_target': STR,
    'model': STR,
    'configure_target': STR,
    'extra_mavlink_cmds': STR,
    'frame_example_script': STR,
    'periph_board': STR,
    'external': (bool,),
    'configure_args': (list,),
    'periph_extra_args': (list,),
    'default_params_filename': (str, list),
    'periph_params_filename': (str, list),
}


def _reject_constant(name):
    # AP_JSON, which SITL uses, rejects NaN and Infinity
    raise ValueError("not JSON: %s" % name)


def _reject_duplicates(pairs):
    keys = [key for key, _ in pairs]
    duplicates = sorted({key for key in keys if keys.count(key) > 1})
    if duplicates:
        raise ValueError("duplicate keys: %s" % duplicates)
    return dict(pairs)


@pytest.fixture(scope='module')
def vehicleinfo():
    with open(VEHICLEINFO) as f:
        return json.load(f, parse_constant=_reject_constant, object_pairs_hook=_reject_duplicates)


def all_frames(info):
    for vehicle_name, vehicle in info.items():
        for frame_name, frame in vehicle['frames'].items():
            yield "%s %s" % (vehicle_name, frame_name), frame


def file_names(value):
    '''a *_params_filename value: one name or a list of them'''
    return [value] if isinstance(value, str) else value


def test_vehicles_have_frames(vehicleinfo):
    assert isinstance(vehicleinfo, dict)
    for name, vehicle in vehicleinfo.items():
        assert isinstance(vehicle, dict), name
        assert isinstance(vehicle.get('frames'), dict), name
        assert vehicle['frames'], name


def test_default_frame_exists_and_has_a_waf_target(vehicleinfo):
    # VehicleInfo.default_waf_target() looks the default frame up
    for name, vehicle in vehicleinfo.items():
        if 'default_frame' not in vehicle:
            continue
        default_frame = vehicle['default_frame']
        assert isinstance(default_frame, str), name
        assert default_frame in vehicle['frames'], name
        assert isinstance(vehicle['frames'][default_frame].get('waf_target'), str), name


def test_frame_keys_and_types(vehicleinfo):
    for where, frame in all_frames(vehicleinfo):
        assert isinstance(frame, dict), where
        for key, value in frame.items():
            assert key in FRAME_KEYS, "%s: unknown key %s" % (where, key)
            assert isinstance(value, FRAME_KEYS[key]), "%s: %s" % (where, key)
            if isinstance(value, list):
                assert all(isinstance(item, str) for item in value), "%s: %s" % (where, key)


def test_periph_board_exists(vehicleinfo):
    # sim_vehicle.py runs the peripheral with the board's "universal" frame
    for where, frame in all_frames(vehicleinfo):
        if 'periph_board' in frame:
            board = vehicleinfo.get(frame['periph_board'])
            assert board is not None, where
            assert 'universal' in board['frames'], where


def test_params_files_exist(vehicleinfo):
    # sim_vehicle.py and autotest load them from Tools/autotest
    for where, frame in all_frames(vehicleinfo):
        for key in 'default_params_filename', 'periph_params_filename':
            for name in file_names(frame.get(key, [])):
                assert os.path.isfile(os.path.join(AUTOTEST, name)), "%s: %s" % (where, name)


def test_default_params_are_embedded_in_sitl_romfs(vehicleinfo):
    # SITL loads them as @ROMFS/<name>, so each must be one SITL embeds
    for where, frame in all_frames(vehicleinfo):
        for name in file_names(frame.get('default_params_filename', [])):
            directory, _, filename = name.partition('/')
            patterns = SITL_ROMFS_PATTERNS.get(directory, [])
            assert any(fnmatch.fnmatch(filename, p) for p in patterns), \
                "%s: %s is not embedded in SITL's ROMFS" % (where, name)


def test_frame_example_scripts_exist(vehicleinfo):
    for where, frame in all_frames(vehicleinfo):
        if 'frame_example_script' in frame:
            path = os.path.join(SCRIPTING_EXAMPLES, frame['frame_example_script'])
            assert os.path.isfile(path), where
