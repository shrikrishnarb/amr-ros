"""Smoke-test that the amr_worlds launch files import and build."""

import importlib.util
import os

import pytest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_FILES = {
    'restaurant': os.path.join(PKG_DIR, 'launch', 'restaurant.launch.py'),
    'mapping': os.path.join(PKG_DIR, 'launch', 'mapping.launch.py'),
}


def _import_launch_module(name):
    spec = importlib.util.spec_from_file_location(
        f'{name}_launch', LAUNCH_FILES[name]
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('name', sorted(LAUNCH_FILES))
def test_launch_file_imports(name):
    """Each launch module must import and expose generate_launch_description."""
    pytest.importorskip('launch')
    module = _import_launch_module(name)
    assert callable(module.generate_launch_description)


@pytest.mark.parametrize('name', sorted(LAUNCH_FILES))
def test_generate_launch_description(name):
    """
    Build each launch description (needs the installed packages).

    Skipped when run outside the amr-butler-dev container (e.g. in the Classic
    container, which has no amr_butler_description / ros_gz packages).
    """
    pytest.importorskip('launch')
    from ament_index_python.packages import (
        PackageNotFoundError,
        get_package_share_directory,
    )
    try:
        get_package_share_directory('amr_worlds')
        get_package_share_directory('amr_butler_description')
        get_package_share_directory('ros_gz_sim')
    except PackageNotFoundError as e:
        pytest.skip(f'required package not installed here: {e}')

    from launch import LaunchDescription
    module = _import_launch_module(name)
    ld = module.generate_launch_description()
    assert isinstance(ld, LaunchDescription)
    argument_names = {
        action.name for action in ld.entities
        if action.__class__.__name__ == 'DeclareLaunchArgument'
    }
    assert {'namespace', 'gui', 'rviz'} <= argument_names
