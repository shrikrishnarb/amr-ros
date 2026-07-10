"""Smoke-test that butler_sim.launch.py imports and builds a LaunchDescription."""

import importlib.util
import os

import pytest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_FILE = os.path.join(PKG_DIR, 'launch', 'butler_sim.launch.py')


def _import_launch_module():
    spec = importlib.util.spec_from_file_location('butler_sim_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_file_imports():
    """The launch module must import and expose generate_launch_description."""
    pytest.importorskip('launch')
    module = _import_launch_module()
    assert callable(module.generate_launch_description)


def test_generate_launch_description():
    """
    Build the launch description (needs the installed package + ros_gz_sim).

    Skipped when run outside the amr-butler-dev container (e.g. in the Classic
    container, which has no ros_gz packages).
    """
    pytest.importorskip('launch')
    from ament_index_python.packages import (
        PackageNotFoundError,
        get_package_share_directory,
    )
    try:
        get_package_share_directory('ros_gz_sim')
        get_package_share_directory('amr_butler_description')
    except PackageNotFoundError as e:
        pytest.skip(f'required package not installed here: {e}')

    from launch import LaunchDescription
    module = _import_launch_module()
    ld = module.generate_launch_description()
    assert isinstance(ld, LaunchDescription)
    argument_names = {
        action.name for action in ld.entities
        if action.__class__.__name__ == 'DeclareLaunchArgument'
    }
    assert {'namespace', 'gui', 'rviz', 'bridge_clock', 'world'} <= argument_names
