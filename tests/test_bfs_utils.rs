// tests/test_utils.rs

#[cfg(test)]
mod test_urdf_bfs_utils {
    use std::path::PathBuf;
    use krecviz::utils::urdf_bfs_utils::{build_link_bfs_map, find_root_link_name};
    use urdf_rs::{read_from_string, read_file};

    #[test]
    fn test_minimal_urdf_bfs() {
        // A minimal URDF with two links (base_link -> link_1) via one fixed joint
        let urdf_str = r#"
        <robot name="TestRobot">
          <link name="base_link"/>
          <link name="link_1"/>
          <joint name="joint_1" type="fixed">
            <origin xyz="0 1 0" rpy="0 0 0"/>
            <parent link="base_link"/>
            <child link="link_1"/>
          </joint>
        </robot>
        "#;

        let robot = read_from_string(urdf_str).expect("Failed to parse minimal URDF string");

        // 1) Confirm root link is 'base_link'
        let root_name = find_root_link_name(&robot.links, &robot.joints)
            .expect("No root link found?");
        assert_eq!(root_name, "base_link");

        // 2) Build BFS data
        let (bfs_map, bfs_order) = build_link_bfs_map(&robot);

        // Should have 2 entries: 'base_link' and 'link_1'
        assert_eq!(bfs_map.len(), 2, "We should have BFS data for two links");

        // BFS order should be [ "base_link", "link_1" ]
        assert_eq!(bfs_order, vec!["base_link", "link_1"]);

        // Check the data for 'link_1'
        let link1_data = bfs_map.get("link_1").expect("Missing BFS data for 'link_1'");
        assert_eq!(link1_data.link_name, "link_1");
        assert_eq!(link1_data.link_only_path, "base_link/link_1");

        // local_transform: row-major => indices [3,7,11] are translation X, Y, Z
        // We expect Y=1.0, so index 7 => 1.0
        let y_translation = link1_data.local_transform[7];
        assert!(
            (y_translation - 1.0).abs() < 1e-6,
            "Link_1 transform Y-translation should be 1.0"
        );
    }

    #[test]
    fn test_manual_urdf_bfs() {
        // Path to your manual_example.urdf file.
        // Adjust if your directory structure is different!
        let urdf_path = PathBuf::from("tests/assets/urdf_examples/manual_urdf/manual_example.urdf");
        assert!(
            urdf_path.exists(),
            "Could not find manual_example.urdf at: {:?}",
            urdf_path
        );

        // 1) Parse the URDF from disk
        let robot = read_file(urdf_path).expect("Failed to parse manual_example.urdf");

        // 2) Identify the root link
        let root = find_root_link_name(&robot.links, &robot.joints)
            .expect("No root link found in URDF?");
        assert_eq!(root, "base", "Root link should be 'base'.");

        // 3) Build BFS map & BFS order
        let (bfs_map, bfs_order) = build_link_bfs_map(&robot);

        // We expect these four links in BFS order:
        //    base -> Part_1 -> Part_1_2 -> Part_1_3
        // BFS sorts child link names alphabetically, so "Part_1_2" < "Part_1_3".
        let expected_order = vec!["base", "Part_1", "Part_1_2", "Part_1_3"];
        assert_eq!(
            bfs_order, expected_order,
            "BFS link order differs from expected."
        );

        // 4) Verify local transforms from each joint's <origin>:
        //   (a) floating_base => base -> Part_1
        //       origin xyz="0 0 0", rpy="0 0 0"
        let part1_data = bfs_map.get("Part_1").expect("Missing BFS data for Part_1");
        let tf_part1 = part1_data.local_transform;
        // Indices 3,7,11 => X,Y,Z
        assert!(
            tf_part1[3].abs() < 1e-9 && tf_part1[7].abs() < 1e-9 && tf_part1[11].abs() < 1e-9,
            "Part_1 local transform translation should be (0,0,0)"
        );

        //   (b) Revolute_2 => Part_1 -> Part_1_2
        //       origin xyz="-0.015 0.0025 -0.0012", rpy="1.5708 0 1.5708"
        let part1_2_data = bfs_map.get("Part_1_2").expect("Missing BFS for Part_1_2");
        let tf_part1_2 = part1_2_data.local_transform;
        let (x_2, y_2, z_2) = (tf_part1_2[3], tf_part1_2[7], tf_part1_2[11]);
        assert!(
            (x_2 + 0.015).abs() < 1e-6,
            "Part_1_2 local X should be -0.015, got {}",
            x_2
        );
        assert!(
            (y_2 - 0.0025).abs() < 1e-6,
            "Part_1_2 local Y should be 0.0025, got {}",
            y_2
        );
        assert!(
            (z_2 + 0.0012).abs() < 1e-6,
            "Part_1_2 local Z should be -0.0012, got {}",
            z_2
        );

        //   (c) Revolute_3 => Part_1 -> Part_1_3
        //       origin xyz="0 0.0025 -0.014", rpy="3.1415927 0 0"
        let part1_3_data = bfs_map.get("Part_1_3").expect("Missing BFS for Part_1_3");
        let tf_part1_3 = part1_3_data.local_transform;
        let (x_3, y_3, z_3) = (tf_part1_3[3], tf_part1_3[7], tf_part1_3[11]);
        assert!(
            x_3.abs() < 1e-6,
            "Part_1_3 local X should be 0, got {}",
            x_3
        );
        assert!(
            (y_3 - 0.0025).abs() < 1e-6,
            "Part_1_3 local Y should be 0.0025, got {}",
            y_3
        );
        assert!(
            (z_3 + 0.014).abs() < 1e-6,
            "Part_1_3 local Z should be -0.014, got {}",
            z_3
        );

        // If we reach here, BFS & transforms match expected for this manual URDF
    }
}
