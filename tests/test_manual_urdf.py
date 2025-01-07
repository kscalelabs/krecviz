"""Tests a generated (in-code) simplified URDF by calling 'visualize_krec'."""

import xml.etree.ElementTree as ET
from pathlib import Path

# Import your visualization function. Adjust the import path if needed.
from krecviz.visualize import visualize_krec

# 1) Figure out base path of this test file
BASE_PATH = Path(__file__).parent.resolve()

# 2) Create or confirm the manual_urdf folder
URDF_BASE_PATH = BASE_PATH / "assets" / "urdf_examples" / "manual_urdf"
URDF_BASE_PATH.mkdir(parents=True, exist_ok=True)


def create_simplified_urdf(urdf_file: Path) -> None:
    """Create a minimal URDF for visualization.

    Create a minimal URDF with:
    - 4 links (base, Part_1, Part_1_2, Part_1_3)
    - 1 fixed joint (floating_base)
    - 2 revolute joints (Revolute_2 and Revolute_3)

    All links have simple <visual> blocks referencing STL meshes, and no inertial
    or collision data. This is intended for quick demonstration or visualization
    in typical URDF-based tools.

    Format is as follows:
    robot "generated_simplified_robot"
    ├─ base
    ├─ floating_base (fixed)      base → Part_1
    ├─ Revolute_2 (revolute)      Part_1 → Part_1_2
    └─ Revolute_3 (revolute)      Part_1 → Part_1_3
    """
    root = ET.Element("robot", {"name": "generated_simplified_robot"})

    # 1) base link
    ET.SubElement(root, "link", {"name": "base"})

    # 2) fixed joint from base -> Part_1
    joint_fixed = ET.SubElement(root, "joint", {"name": "floating_base", "type": "fixed"})
    ET.SubElement(joint_fixed, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(joint_fixed, "parent", {"link": "base"})
    ET.SubElement(joint_fixed, "child", {"link": "Part_1"})

    # 3) Part_1 link (visual only)
    link_part1 = ET.SubElement(root, "link", {"name": "Part_1"})
    visual1 = ET.SubElement(link_part1, "visual")
    ET.SubElement(visual1, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    geom1 = ET.SubElement(visual1, "geometry")
    ET.SubElement(geom1, "mesh", {"filename": "meshes/Part_1.stl"})
    mat1 = ET.SubElement(visual1, "material", {"name": "Part_1_color"})
    ET.SubElement(mat1, "color", {"rgba": "0.8 0.4 0.1 1.0"})

    # 4) Revolute_2 joint
    joint_rev2 = ET.SubElement(root, "joint", {"name": "Revolute_2", "type": "revolute"})
    ET.SubElement(joint_rev2, "origin", {"xyz": "-0.015 0.0025 -0.0012", "rpy": "1.5708 0 1.5708"})
    ET.SubElement(joint_rev2, "parent", {"link": "Part_1"})
    ET.SubElement(joint_rev2, "child", {"link": "Part_1_2"})
    ET.SubElement(joint_rev2, "axis", {"xyz": "0 0 1"})
    ET.SubElement(joint_rev2, "limit", {"effort": "80", "velocity": "5", "lower": "-1.5707963", "upper": "1.5707963"})

    # 5) Part_1_2 link (visual only)
    link_part1_2 = ET.SubElement(root, "link", {"name": "Part_1_2"})
    visual2 = ET.SubElement(link_part1_2, "visual")
    ET.SubElement(visual2, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    geom2 = ET.SubElement(visual2, "geometry")
    ET.SubElement(geom2, "mesh", {"filename": "meshes/Part_1_2.stl"})
    mat2 = ET.SubElement(visual2, "material", {"name": "Part_1_2_color"})
    ET.SubElement(mat2, "color", {"rgba": "0.7 0.6 0.3 1.0"})

    # 6) Revolute_3 joint
    joint_rev3 = ET.SubElement(root, "joint", {"name": "Revolute_3", "type": "revolute"})
    ET.SubElement(joint_rev3, "origin", {"xyz": "0 0.0025 -0.014", "rpy": "3.1415927 0 0"})
    ET.SubElement(joint_rev3, "parent", {"link": "Part_1"})
    ET.SubElement(joint_rev3, "child", {"link": "Part_1_3"})
    ET.SubElement(joint_rev3, "axis", {"xyz": "0 0 -1"})
    ET.SubElement(joint_rev3, "limit", {"effort": "80", "velocity": "5", "lower": "-1.5707963", "upper": "1.5707963"})

    # 7) Part_1_3 link (visual only)
    link_part1_3 = ET.SubElement(root, "link", {"name": "Part_1_3"})
    visual3 = ET.SubElement(link_part1_3, "visual")
    ET.SubElement(visual3, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    geom3 = ET.SubElement(visual3, "geometry")
    ET.SubElement(geom3, "mesh", {"filename": "meshes/Part_1_3.stl"})
    mat3 = ET.SubElement(visual3, "material", {"name": "Part_1_3_color"})
    ET.SubElement(mat3, "color", {"rgba": "0.5 0.5 0.8 1.0"})

    # Pretty-print (Python >=3.9). If older, comment out or remove.
    ET.indent(root, space="  ", level=0)
    tree = ET.ElementTree(root)
    tree.write(urdf_file, encoding="unicode", xml_declaration=True)


def _test_run_visualization_generated_simplified_adhoc() -> None:
    """Run ad-hoc test with interactive visualization.

    Directly call visualize_krec in interactive mode (no output path).
    Launches the GUI so you can confirm the model visually.
    """
    # Create a local URDF file
    temp_urdf = Path(URDF_BASE_PATH / "manual_example.urdf")
    create_simplified_urdf(temp_urdf)

    visualize_krec(krec_path=None, urdf_path=temp_urdf)


if __name__ == "__main__":
    # Run an ad-hoc test that shows the GUI (rather than the auto test).
    _test_run_visualization_generated_simplified_adhoc()
