"""Visualize KREC recordings with URDF models using Rerun.

This script loads a KREC recording and animates it on a URDF model, displaying
joint positions, velocities, and torques over time.

NOTE: You cannot both save the visualization to a file and log to an open Rerun viewer.

Usage:
    # Basic usage with default paths
    python visualize.py

    # Specify custom URDF and KREC files
    python visualize.py --urdf path/to/robot.urdf --krec path/to/recording.krec

    # Save the visualization to a Rerun file
    python visualize.py --output my_animation.rrd
"""

import argparse
import logging
from pathlib import Path

import krec
import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation
from tqdm import tqdm

from krecviz.urdf_logger import URDFLogger

# 1. Define mappings between actuator IDs and URDF joints
actuator_to_urdf_joint = {
    # Left Arm
    11: "Revolute_2",
    12: "Revolute_4",
    13: "Revolute_5",
    14: "Revolute_8",
    15: "Revolute_16",
    # Right Arm
    21: "Revolute_1",
    22: "Revolute_3",
    23: "Revolute_6",
    24: "Revolute_7",
    25: "Revolute_15",
    # Left Leg
    31: "L_hip_y",
    32: "L_hip_x",
    33: "L_hip_z",
    34: "L_knee",
    35: "L_ankle_y",
    # Right Leg
    41: "R_hip_y",
    42: "R_hip_x",
    43: "R_hip_z",
    44: "R_knee",
    45: "R_ankle_y",
}


def load_krec(file_path: Path, verbose: bool = False) -> krec.KRec:
    """Smart loader that handles both direct KREC and MKV-embedded KREC files."""
    if file_path.suffix == ".krec":
        return krec.KRec.load(str(file_path))
    elif file_path.suffixes == [".krec", ".mkv"]:
        return krec.extract_from_video(str(file_path), verbose=verbose)
    else:
        raise RuntimeError(f"Invalid file extension. Expected '.krec' or '.krec.mkv', got: {file_path}")


def update_robot_pose(
    entity_to_transform: dict[str, tuple[list[float], list[list[float]]]],
    actuator_states: list[krec.ActuatorState],
    joint_name_to_entity_path: dict[str, str],
) -> None:
    """Updates robot joint positions based on actuator states."""
    joint_angles = {}
    for state in actuator_states:
        if state.actuator_id in actuator_to_urdf_joint and state.position is not None:
            joint_name = actuator_to_urdf_joint[state.actuator_id]
            # Convert degrees to radians
            angle_rad = np.deg2rad(float(state.position))
            joint_angles[joint_name] = angle_rad

    # Update each joint's transform
    for joint_name, angle in joint_angles.items():
        if joint_name not in joint_name_to_entity_path:
            logging.warning("No entity path found for joint %s", joint_name)
            continue

        full_path = joint_name_to_entity_path[joint_name]
        if full_path not in entity_to_transform:
            logging.warning("Transform not found for path %s", full_path)
            continue

        # Get initial transform and rotation axis
        translation, base_rotation = entity_to_transform[full_path]
        axis = np.array([0, 0, 1])

        # Compute new rotation (angle is already in radians)
        rot_mat = Rotation.from_rotvec(axis * angle).as_matrix()
        new_rotation = base_rotation @ rot_mat

        # Log updated transform
        rr.log(
            f"/{full_path}",
            rr.Transform3D(translation=translation, mat3x3=new_rotation),
        )


def log_frame_data(frame: krec.KRecFrame, frame_idx: int) -> None:
    """Log actuator states for each frame."""
    rr.set_time_sequence("frame_idx", frame_idx)

    for state in frame.get_actuator_states():
        prefix = f"actuators/actuator_{state.actuator_id}/state"
        if state.position is not None:
            rr.log(f"{prefix}/position", rr.Scalar(float(state.position)))
        if state.velocity is not None:
            rr.log(f"{prefix}/velocity", rr.Scalar(float(state.velocity)))
        if state.torque is not None:
            rr.log(f"{prefix}/torque", rr.Scalar(float(state.torque)))


def visualize_krec(
    krec_path: Path | str,
    urdf_path: Path | str | None = None,
    output_path: Path | str | None = None,
) -> None:
    """Visualize a KREC recording with a URDF model using Rerun.

    Args:
        krec_path: Path to the KREC file (.krec or .krec.mkv)
        urdf_path: Path to the URDF file
        output_path: Optional path to save the visualization as .rrd file
    """
    # Convert string paths to Path objects
    krec_path = Path(krec_path) if isinstance(krec_path, str) else krec_path
    urdf_path = Path(urdf_path) if isinstance(urdf_path, str) else urdf_path
    output_path = Path(output_path) if isinstance(output_path, str) else output_path

    # Initialize Rerun
    if output_path:
        rr.init("urdf_basic_logging", spawn=False)
        rr.save(str(output_path))
    else:
        rr.init("urdf_basic_logging", spawn=True)

    if urdf_path:
        # Load and log URDF
        urdf_logger = URDFLogger(str(urdf_path))
        urdf_logger.log()

        # Build mapping from joint names to entity paths
        joint_name_to_entity_path = {}
        for joint in urdf_logger.urdf.joints:
            entity_path = urdf_logger.joint_entity_path(joint)
            joint_name_to_entity_path[joint.name] = entity_path

    # Load KREC file
    krec_data = load_krec(krec_path)
    logging.info("Processing %d frames...", len(krec_data))

    # Process frames
    try:
        for idx, frame in enumerate(tqdm(krec_data, desc="Processing frames")):
            if frame is not None:
                log_frame_data(frame, idx)
                if urdf_path:
                    update_robot_pose(
                        urdf_logger.entity_to_transform,
                        frame.get_actuator_states(),
                        joint_name_to_entity_path,
                    )

        if output_path:
            logging.info("Saved animation to: %s", output_path)
        else:
            logging.info("Animation complete. Press Ctrl+C to exit.")

    except Exception as e:
        logging.error("Error during animation: %s", e)
        raise


def main() -> None:
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(description="Basic URDF and KREC logging with Rerun.")
    parser.add_argument(
        "--urdf",
        type=Path,
        default="data/urdf_examples/gpr/robot.urdf",
        help="Path to the URDF file of the robot.",
    )
    parser.add_argument(
        "--krec",
        type=Path,
        default="data/krec_examples/actuator_22_right_arm_shoulder_roll_movement.krec",
        help="Input KREC file (either .krec or .krec.mkv).",
    )
    parser.add_argument("--output", type=Path, help="Output RRD file.")
    args = parser.parse_args()

    # Set up logging
    logging.basicConfig(level=logging.INFO)

    visualize_krec(args.krec, args.urdf, args.output)


if __name__ == "__main__":
    main()
