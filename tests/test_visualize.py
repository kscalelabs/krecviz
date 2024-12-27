"""Defines a dummy test."""

from pathlib import Path

import pytest

from krecviz.visualize import visualize_krec

BASE_PATH = Path(__file__).parent

KREC_PATH = BASE_PATH / "assets" / "krec_examples" / "actuator_22_right_arm_shoulder_roll_movement.krec"
URDF_PATH = BASE_PATH / "assets" / "urdf_examples" / "gpr" / "robot.urdf"


@pytest.mark.slow
def test_run_visualization(tmpdir: Path) -> None:
    output_path = Path(tmpdir / "output.rrd")

    visualize_krec(
        krec_path=KREC_PATH,
        urdf_path=URDF_PATH,
        output_path=output_path,
    )

    # Verify that this is a valid file.
    assert output_path.exists()
    assert output_path.stat().st_size > 0


def _test_run_visualization_adhoc() -> None:
    """Run visualization on an ad-hoc basis, showing the GUI."""
    visualize_krec(
        krec_path=KREC_PATH,
        urdf_path=URDF_PATH,
    )


if __name__ == "__main__":
    # python -m tests.test_visualize
    _test_run_visualization_adhoc()
