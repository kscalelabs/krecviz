use std::collections::{HashMap, HashSet, VecDeque};
use urdf_rs::{Joint, Link, Robot};
use anyhow::Result;

use crate::spatial_transform_utils::{
    mat4x4_mul, identity_4x4, build_4x4_from_xyz_rpy, rotation_from_euler_xyz
};
use crate::debug_log_utils::debug_log_bfs_insertion;

/// Complete information about a link's position in the hierarchy and transforms
#[derive(Debug, Clone)]
pub struct LinkBfsData {
    /// The name of this link (same as the map key)
    pub link_name: String,
    
    /// A slash-separated path *of links only*, ignoring joints
    /// e.g. "base_link/child_link/gripper_link"
    pub link_only_path: String,
    
    /// A slash-separated path *including* joints for debugging
    /// e.g. "base_link/joint1/child_link/joint2/gripper_link"
    pub link_full_path: String,
    
    /// The local RPY from the last joint that connects to parent
    /// For root link, this will be [0.0, 0.0, 0.0]
    pub local_rpy: [f64; 3],
    
    /// The local translation (XYZ) from the last joint
    /// For root link, this will be [0.0, 0.0, 0.0]
    pub local_translation: [f64; 3],
    
    /// The local transform relative to parent (row-major)
    /// For root link, this will be identity
    pub local_transform: [f32; 16],
    
    /// The global (accumulated) transform from root link (row-major)
    /// For root link, this will be identity
    pub global_transform: [f32; 16],
}

/// Build adjacency map from joints to their child links
pub fn build_adjacency(joints: &[Joint]) -> HashMap<String, Vec<(Joint, String)>> {
    let mut adj = HashMap::new();
    for j in joints {
        let parent_link = j.parent.link.clone();
        let child_link = j.child.link.clone();
        adj.entry(parent_link)
            .or_insert_with(Vec::new)
            .push((j.clone(), child_link));
    }
    adj
}

/// Find the root link name by identifying the link that isn't a child of any joint
pub fn find_root_link_name(links: &[Link], joints: &[Joint]) -> Option<String> {
    let mut all_links = HashSet::new();
    let mut child_links = HashSet::new();

    for l in links {
        all_links.insert(l.name.clone());
    }
    for j in joints {
        child_links.insert(j.child.link.clone());
    }

    all_links.difference(&child_links).next().cloned()
}

/// Build a BFS-based map from each link's name => LinkBfsData.
/// This is the main function that replaces multiple separate BFS passes.
pub fn build_link_bfs_map(robot: &Robot) -> HashMap<String, LinkBfsData> {
    let adjacency = build_adjacency(&robot.joints);

    // 1) Find root link
    let root_link = find_root_link_name(&robot.links, &robot.joints)
        .unwrap_or_else(|| "base".to_string());

    // 2) BFS from root to discover all link paths and transforms
    let mut link_bfs_map = HashMap::new();
    let mut queue = VecDeque::new();

    // Root link has no parent joint => zero local transforms
    let root_data = LinkBfsData {
        link_name: root_link.clone(),
        link_only_path: root_link.clone(),
        link_full_path: root_link.clone(),
        local_rpy: [0.0, 0.0, 0.0],
        local_translation: [0.0, 0.0, 0.0],
        local_transform: identity_4x4(),
        global_transform: identity_4x4(),
    };
    link_bfs_map.insert(root_link.clone(), root_data);
    queue.push_back(root_link);

    // 3) BFS traversal
    while let Some(parent_link_name) = queue.pop_front() {
        // Clone all needed data from parent_info before the mutable borrow
        let (parent_link_only_path, parent_link_full_path, parent_final_transform) = {
            let parent_info = link_bfs_map
                .get(&parent_link_name)
                .expect("Parent link BFS info missing");
            (
                parent_info.link_only_path.clone(),
                parent_info.link_full_path.clone(),
                parent_info.global_transform,
            )
        };

        if let Some(children) = adjacency.get(&parent_link_name) {
            for (joint, child_link_name) in children {
                let child_link_only_path = format!("{}/{}", parent_link_only_path, child_link_name);
                let child_link_full_path = format!(
                    "{}/{}/{}", 
                    parent_link_full_path,
                    joint.name,
                    child_link_name
                );

                // Get local transform from joint's origin
                let local_xyz = [
                    joint.origin.xyz[0],
                    joint.origin.xyz[1],
                    joint.origin.xyz[2],
                ];
                let local_rpy = [
                    joint.origin.rpy[0],
                    joint.origin.rpy[1],
                    joint.origin.rpy[2],
                ];

                let local_tf_4x4 = build_4x4_from_xyz_rpy(local_xyz, local_rpy);
                let child_global_tf = mat4x4_mul(parent_final_transform, local_tf_4x4);

                let child_data = LinkBfsData {
                    link_name: child_link_name.clone(),
                    link_only_path: child_link_only_path,
                    link_full_path: child_link_full_path,
                    local_rpy,
                    local_translation: local_xyz,
                    local_transform: local_tf_4x4,
                    global_transform: child_global_tf,
                };

                debug_log_bfs_insertion(&child_data);
                link_bfs_map.insert(child_link_name.clone(), child_data);
                queue.push_back(child_link_name.clone());
            }
        }
    }

    link_bfs_map
}

/// Information about a joint's position and transforms
#[derive(Debug, Clone)]
pub struct JointInfo {
    pub entity_path: String,
    pub origin_translation: [f32; 3],
    pub base_rotation: [f32; 9],
}

/// Build joint info map using the BFS data
pub fn build_joint_name_to_joint_info(urdf_path: &str) -> Result<HashMap<String, JointInfo>> {
    let robot = urdf_rs::read_file(urdf_path)?;
    let link_bfs_map = build_link_bfs_map(&robot);
    
    let mut joint_info_map = HashMap::new();
    
    for joint in &robot.joints {
        if let Some(child_data) = link_bfs_map.get(&joint.child.link) {
            let entity_path = child_data.link_only_path.clone();
            
            let translation = [
                joint.origin.xyz[0] as f32,
                joint.origin.xyz[1] as f32,
                joint.origin.xyz[2] as f32,
            ];
            
            let base_rotation = rotation_from_euler_xyz(
                joint.origin.rpy[0],
                joint.origin.rpy[1],
                joint.origin.rpy[2],
            );
            
            let info = JointInfo {
                entity_path,
                origin_translation: translation,
                base_rotation,
            };
            
            joint_info_map.insert(joint.name.clone(), info);
        }
    }
    
    Ok(joint_info_map)
}
