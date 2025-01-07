"""Defines basic visualization tests."""

from pathlib import Path

import pytest

from krecviz.visualize import visualize_krec

BASE_PATH = Path(__file__).parent

URDF_BASE_PATH = BASE_PATH / "assets" / "urdf_examples"

EXAMPLE_KREC_PATHS = {
    "gpr": BASE_PATH / "assets" / "krec_examples" / "actuator_22_right_arm_shoulder_roll_movement.krec",
}

EXAMPLE_URDF_PATHS = {
    "gpr": URDF_BASE_PATH / "gpr" / "robot.urdf",
    "simple": URDF_BASE_PATH / "simple" / "example.urdf",
    "simple_onshape_2_joints": URDF_BASE_PATH / "simple_onshape_2_joints" / "assembly_1.urdf",
    "simple_onshape_2_joints_asymmetrical": URDF_BASE_PATH / "simple_onshape_2_joints_asymmetrical" / "assembly_1.urdf",
    "simple_onshape_4_joints": URDF_BASE_PATH / "simple_onshape_4_joints" / "assembly_1.urdf",
    "xbot": URDF_BASE_PATH / "XBot" / "urdf" / "XBot-L.urdf",
}


@pytest.mark.slow
def test_run_visualization(tmpdir: Path) -> None:
    """Simply confirm code runs and output file is created."""
    output_path = Path(tmpdir / "output.rrd")

    visualize_krec(
        krec_path=EXAMPLE_KREC_PATHS["gpr"],
        urdf_path=EXAMPLE_URDF_PATHS["gpr"],
        output_path=output_path,
    )

    # Verify that this is a valid file.
    assert output_path.exists()
    assert output_path.stat().st_size > 0


def _test_run_visualization_gpr_adhoc() -> None:
    """Run visualization on an ad-hoc basis, showing the GUI."""
    visualize_krec(
        krec_path=EXAMPLE_KREC_PATHS["gpr"],
        urdf_path=EXAMPLE_URDF_PATHS["gpr"],
    )


def _test_run_visualization_simple_onshape_2_joints_asymmetrical_adhoc() -> None:
    """Run visualization on an ad-hoc basis, showing the GUI."""
    visualize_krec(
        krec_path=None,
        urdf_path=EXAMPLE_URDF_PATHS["simple_onshape_2_joints_asymmetrical"],
    )


def _test_run_visualization_simple_adhoc() -> None:
    """Run visualization on an ad-hoc basis, showing the GUI."""
    visualize_krec(
        krec_path=None,
        urdf_path=EXAMPLE_URDF_PATHS["simple"],
    )


def _test_run_visualization_simple_onshape_2_joints_adhoc() -> None:
    """Run visualization on an ad-hoc basis, showing the GUI."""
    visualize_krec(
        krec_path=None,
        urdf_path=EXAMPLE_URDF_PATHS["simple_onshape_2_joints"],
    )


def _test_run_visualization_simple_onshape_4_joints_adhoc() -> None:
    """Run visualization on an ad-hoc basis, showing the GUI."""
    visualize_krec(
        krec_path=None,
        urdf_path=EXAMPLE_URDF_PATHS["simple_onshape_4_joints"],
    )


def _test_run_visualization_xbot_adhoc() -> None:
    """Run visualization on an ad-hoc basis, showing the GUI."""
    visualize_krec(
        krec_path=None,
        urdf_path=EXAMPLE_URDF_PATHS["xbot"],
    )


if __name__ == "__main__":
    # python -m tests.test_visualize

    # Choose which test to run here, running both at the same time doesn't work with rerun.

    # Full gpr robot
    # _test_run_visualization_gpr_adhoc()

    # Simple 2 joint URDF
    _test_run_visualization_simple_onshape_2_joints_asymmetrical_adhoc()

    # Simple URDF
    # _test_run_visualization_simple_adhoc()

    # Simple 2 joint URDF (symmetrical)
    # _test_run_visualization_simple_onshape_2_joints_adhoc()

    # Simple 4 joint URDF
    # _test_run_visualization_simple_onshape_4_joints_adhoc()

    # XBot URDF
    # _test_run_visualization_xbot_adhoc()
