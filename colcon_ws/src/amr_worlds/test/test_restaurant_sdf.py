"""Validate the worlds/restaurant.sdf structure (and `gz sdf --check` when usable)."""

import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_FILE = os.path.join(PKG_DIR, 'worlds', 'restaurant.sdf')

REQUIRED_PLUGINS = {
    'gz::sim::systems::Physics',
    'gz::sim::systems::UserCommands',
    'gz::sim::systems::SceneBroadcaster',
    'gz::sim::systems::Sensors',
}
REQUIRED_MODELS = {
    'table_1', 'table_2', 'table_3', 'table_4', 'table_5', 'table_6',
    'kitchen_counter', 'bar_counter', 'charging_dock',
}
LIDAR_Z = 0.25  # butler lidar scan plane height [m] (amr_butler.urdf.xacro)


def _world():
    root = ET.parse(WORLD_FILE).getroot()
    world = root.find('world')
    assert world is not None, 'no <world> element'
    return world


def test_sdf_is_valid_xml():
    """The world file must be well-formed XML with an SDF root."""
    root = ET.parse(WORLD_FILE).getroot()
    assert root.tag == 'sdf'
    assert root.get('version')


def test_world_plugin_set():
    """The world needs the same plugin set as butler_test.sdf, ogre2 rendering."""
    world = _world()
    plugins = {p.get('name'): p for p in world.findall('plugin')}
    assert REQUIRED_PLUGINS <= set(plugins), (
        f'missing plugins: {REQUIRED_PLUGINS - set(plugins)}'
    )
    sensors = plugins['gz::sim::systems::Sensors']
    engine = sensors.find('render_engine')
    assert engine is not None and engine.text == 'ogre2', (
        'Sensors plugin must use the verified ogre2 render engine'
    )


def test_named_models_exist():
    """All semantic locations must exist as named models."""
    names = {m.get('name') for m in _world().findall('model')}
    assert REQUIRED_MODELS <= names, f'missing models: {REQUIRED_MODELS - names}'


def test_all_models_are_static():
    """Furniture must be static so physics stays cheap and nothing drifts."""
    for model in _world().findall('model'):
        static = model.find('static')
        assert static is not None and static.text == 'true', (
            f"model {model.get('name')} is not static"
        )


def test_tables_have_four_legs_at_lidar_height():
    """Each table needs >= 4 cylinder legs whose extent crosses the lidar plane."""
    models = {m.get('name'): m for m in _world().findall('model')}
    for name in (f'table_{i}' for i in range(1, 7)):
        table_link = models[name].find("link[@name='table']")
        assert table_link is not None, f'{name} has no table link'
        legs = 0
        for col in table_link.findall('collision'):
            cyl = col.find('geometry/cylinder')
            if cyl is None:
                continue
            length = float(cyl.find('length').text)
            pose = col.find('pose')
            z = float(pose.text.split()[2]) if pose is not None else 0.0
            if z - length / 2 < LIDAR_Z < z + length / 2:
                legs += 1
        assert legs >= 4, f'{name}: only {legs} legs cross the lidar plane'


def test_tables_have_chairs():
    """Each table needs 2-4 chair links (seat + legs)."""
    models = {m.get('name'): m for m in _world().findall('model')}
    for name in (f'table_{i}' for i in range(1, 7)):
        chairs = [
            link for link in models[name].findall('link')
            if link.get('name', '').startswith('chair_')
        ]
        assert 2 <= len(chairs) <= 4, f'{name}: {len(chairs)} chairs'


def test_gz_sdf_check():
    """
    Run `gz sdf --check` when a Harmonic-capable gz CLI is available.

    The Classic container's `gz` tool only understands SDF <= 1.7, so the
    check is skipped (not failed) when the CLI cannot convert this file;
    structural validation is covered by the XML-based tests above.
    """
    gz = shutil.which('gz')
    if gz is None:
        pytest.skip('gz CLI not available')
    result = subprocess.run(
        [gz, 'sdf', '--check', WORLD_FILE],
        capture_output=True, text=True, timeout=60,
    )
    output = result.stdout + result.stderr
    if 'Unable to convert' in output:
        pytest.skip('gz CLI does not support this SDF version (gazebo-classic gz)')
    assert result.returncode == 0, output
    assert 'Error' not in output, output
