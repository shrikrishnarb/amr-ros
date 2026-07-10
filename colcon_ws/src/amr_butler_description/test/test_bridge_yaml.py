"""Validate the config/bridge.yaml ros_gz_bridge template."""

import os

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BRIDGE_FILE = os.path.join(PKG_DIR, 'config', 'bridge.yaml')

REQUIRED_KEYS = {
    'ros_topic_name', 'gz_topic_name', 'ros_type_name', 'gz_type_name', 'direction',
}
VALID_DIRECTIONS = {'GZ_TO_ROS', 'ROS_TO_GZ', 'BIDIRECTIONAL'}

EXPECTED_ROS_TOPICS = {
    '/clock',
    '/tf',
    '/butler1/cmd_vel',
    '/butler1/odom',
    '/butler1/joint_states',
    '/butler1/scan',
    '/butler1/camera/image_raw',
    '/butler1/camera/depth/image_raw',
    '/butler1/camera/camera_info',
}


def _load(namespace='butler1'):
    """Load the template with {NS} substituted, as the launch file does."""
    with open(BRIDGE_FILE, 'r') as f:
        content = f.read()
    return yaml.safe_load(content.replace('{NS}', namespace))


def test_bridge_yaml_is_valid_yaml():
    """The substituted template must parse into a non-empty list of mappings."""
    entries = _load()
    assert isinstance(entries, list)
    assert len(entries) >= 9


def test_bridge_entries_have_required_keys():
    """Each entry needs the full parameter_bridge key set and a valid direction."""
    for entry in _load():
        assert REQUIRED_KEYS <= set(entry), f'missing keys in {entry}'
        assert entry['direction'] in VALID_DIRECTIONS
        assert '{NS}' not in entry['ros_topic_name'], 'unsubstituted placeholder'
        assert '{NS}' not in entry['gz_topic_name'], 'unsubstituted placeholder'


def test_expected_topic_contract():
    """ROS-side names must match the /<ns>/... contract used by the Classic stack."""
    entries = _load()
    ros_topics = {e['ros_topic_name'] for e in entries}
    assert EXPECTED_ROS_TOPICS <= ros_topics, (
        f'missing topics: {EXPECTED_ROS_TOPICS - ros_topics}'
    )
    by_topic = {e['ros_topic_name']: e for e in entries}
    assert by_topic['/clock']['direction'] == 'GZ_TO_ROS'
    assert by_topic['/butler1/cmd_vel']['direction'] == 'ROS_TO_GZ'
    assert by_topic['/butler1/scan']['direction'] == 'GZ_TO_ROS'


def test_gz_topics_are_model_scoped():
    """gz-side names (except /clock) must be model-scoped for multi-robot use."""
    for entry in _load('butler2'):
        gz_topic = entry['gz_topic_name']
        if gz_topic == '/clock':
            continue
        assert gz_topic.startswith('/model/butler2/'), (
            f'gz topic {gz_topic} is not model-scoped'
        )
