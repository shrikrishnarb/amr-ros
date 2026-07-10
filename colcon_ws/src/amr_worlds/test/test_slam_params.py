"""Validate the config/slam_params.yaml slam_toolbox template."""

import os

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SLAM_FILE = os.path.join(PKG_DIR, 'config', 'slam_params.yaml')


def _load(namespace='butler1'):
    """Load the template with {NS} substituted, as mapping.launch.py does."""
    with open(SLAM_FILE, 'r') as f:
        content = f.read()
    assert '{NS}' in content, 'template lost its {NS} placeholders'
    return yaml.safe_load(content.replace('{NS}', namespace))


def _params(namespace='butler1'):
    config = _load(namespace)
    assert 'slam_toolbox' in config
    assert 'ros__parameters' in config['slam_toolbox']
    return config['slam_toolbox']['ros__parameters']


def test_yaml_parses():
    """The substituted template must parse into the slam_toolbox param tree."""
    params = _params()
    assert isinstance(params, dict)
    assert len(params) > 10


def test_namespaced_frames_and_scan_topic():
    """Frames must follow the repo namespacing convention (CLAUDE.md 'TF')."""
    params = _params()
    assert params['base_frame'] == 'butler1/base_footprint'
    assert params['odom_frame'] == 'butler1/odom'
    assert params['map_frame'] == 'map'
    assert params['scan_topic'] == '/butler1/scan'


def test_mapping_mode_and_sim_time():
    """Mapping mode with sim time; laser range matches the butler lidar (8 m)."""
    params = _params()
    assert params['use_sim_time'] is True
    assert params['mode'] == 'mapping'
    assert params['max_laser_range'] == 8.0


def test_no_unsubstituted_placeholders():
    """After substitution no {NS} may remain anywhere in the param tree."""
    assert '{NS}' not in yaml.safe_dump(_load('butler2'))
