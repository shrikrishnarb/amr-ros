"""Validate that amr_butler.urdf.xacro expands to a well-formed, namespaced URDF."""

import os
import shutil
import subprocess
from xml.etree import ElementTree

import pytest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XACRO_FILE = os.path.join(PKG_DIR, 'urdf', 'amr_butler.urdf.xacro')

EXPECTED_LINKS = [
    'base_footprint', 'base_link', 'left_wheel', 'right_wheel',
    'front_caster', 'rear_caster', 'pillar', 'lower_tray', 'upper_tray',
    'head', 'lidar_link', 'camera_link', 'camera_link_optical',
]


def _expand(namespace):
    """Run xacro and return the parsed URDF root element."""
    if shutil.which('xacro') is None:
        pytest.skip('xacro executable not available')
    result = subprocess.run(
        ['xacro', XACRO_FILE, f'namespace:={namespace}'],
        capture_output=True, text=True, check=False,
    )
    assert result.returncode == 0, f'xacro failed:\n{result.stderr}'
    return ElementTree.fromstring(result.stdout)


def test_xacro_expands_to_urdf():
    """The xacro must produce a <robot> document with namespaced links."""
    root = _expand('butler1')
    assert root.tag == 'robot'
    link_names = {link.get('name') for link in root.findall('link')}
    for expected in EXPECTED_LINKS:
        assert f'butler1/{expected}' in link_names, f'missing link butler1/{expected}'


def test_urdf_tree_is_valid():
    """Every joint must reference declared links and the tree must have one root."""
    root = _expand('butler1')
    link_names = {link.get('name') for link in root.findall('link')}
    children = set()
    for joint in root.findall('joint'):
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        assert parent in link_names, f'joint parent {parent} is not a declared link'
        assert child in link_names, f'joint child {child} is not a declared link'
        assert child not in children, f'link {child} has two parent joints'
        children.add(child)
    roots = link_names - children
    assert roots == {'butler1/base_footprint'}, f'unexpected root links: {roots}'


def test_inertias_are_physical():
    """Inertial matrices must be diagonal-positive and not identity placeholders."""
    root = _expand('butler1')
    checked = 0
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        assert float(inertial.find('mass').get('value')) > 0.0
        inertia = inertial.find('inertia')
        diag = [float(inertia.get(k)) for k in ('ixx', 'iyy', 'izz')]
        assert all(v > 0.0 for v in diag), f'non-positive inertia in {link.get("name")}'
        assert diag != [1.0, 1.0, 1.0], f'identity inertia in {link.get("name")}'
        checked += 1
    assert checked >= 10, 'expected inertials on all physical links'


def test_namespace_argument_is_respected():
    """Frames and gz topics must follow the namespace argument, with no leftovers."""
    if shutil.which('xacro') is None:
        pytest.skip('xacro executable not available')
    result = subprocess.run(
        ['xacro', XACRO_FILE, 'namespace:=butler2'],
        capture_output=True, text=True, check=False,
    )
    assert result.returncode == 0, f'xacro failed:\n{result.stderr}'
    assert 'butler1' not in result.stdout, 'default namespace leaked into output'
    assert 'butler2/base_link' in result.stdout


def test_gz_topics_are_model_scoped():
    """All gz plugin/sensor topics must be under /model/<ns>/ for multi-robot use."""
    root = _expand('butler1')
    topics = [t.text for t in root.iter('topic')]
    topics += [t.text for t in root.iter('odom_topic')]
    topics += [t.text for t in root.iter('tf_topic')]
    assert len(topics) >= 5, 'expected topics for diff drive, joint states, sensors'
    for topic in topics:
        assert topic.startswith('/model/butler1/'), f'topic {topic} is not model-scoped'


def test_diff_drive_frames_follow_rep105():
    """Check DiffDrive publishes odom -> base_footprint with namespaced frames."""
    root = _expand('butler1')
    plugins = {p.get('name'): p for p in root.iter('plugin')}
    diff_drive = plugins.get('gz::sim::systems::DiffDrive')
    assert diff_drive is not None, 'DiffDrive plugin missing'
    assert diff_drive.find('frame_id').text == 'butler1/odom'
    assert diff_drive.find('child_frame_id').text == 'butler1/base_footprint'
    assert plugins.get('gz::sim::systems::JointStatePublisher') is not None
