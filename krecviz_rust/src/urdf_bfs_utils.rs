use std::collections::{HashMap, HashSet, VecDeque};
use urdf_rs::{Joint, Link};
use anyhow::Result;
use crate::spatial_transform_utils::rotation_from_euler_xyz;

/// Build adjacency map from joints to their child links.
pub fn build_adjacency(joints: &[Joint]) -> HashMap<String, Vec<(Joint, String)>> {
    let mut adj = HashMap::new();
    for j in joints {
        let p = j.parent.link.clone();
        let c = j.child.link.clone();
        adj.entry(p).or_insert_with(Vec::new).push((j.clone(), c));
    }
    adj
}

/// Find the root link name by identifying the link that isn't a child of any joint.
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

/// Build a map of `link_name -> full chain of [link, joint, link, joint, ...]`.
/// This is done by one BFS traversal from the given `root_link`.
pub fn build_link_paths_map(
    adjacency: &HashMap<String, Vec<(Joint, String)>>,
    root_link: &str,
) -> HashMap<String, Vec<String>> {
    let mut map = HashMap::new();
    let mut queue = VecDeque::new();

    // Start BFS from the root link. Its path is just [root_link] initially.
    queue.push_back((root_link.to_owned(), vec![root_link.to_owned()]));

    while let Some((cur_link, path_so_far)) = queue.pop_front() {
        // Store the full path to `cur_link`.
        map.insert(cur_link.clone(), path_so_far.clone());

        // Enqueue child links, building extended paths.
        if let Some(kids) = adjacency.get(&cur_link) {
            for (j, c) in kids {
                let mut new_path = path_so_far.clone();
                new_path.push(j.name.clone());
                new_path.push(c.clone());
                queue.push_back((c.clone(), new_path));
            }
        }
    }

    map
}

/// Get the chain of links+joint-names for `target` link from the BFS map.
/// Returns something like [rootLink, jointA, link2, jointB, targetLink].
pub fn get_link_chain(
    link_paths_map: &HashMap<String, Vec<String>>,
    target: &str,
) -> Option<Vec<String>> {
    link_paths_map.get(target).cloned()
}

/// Convert a chain of [link, joint, link, joint, link, ...]
/// into a slash-separated path *of link-names only*.
/// Example:
///   chain = [ "base_link", "joint1", "link2", "joint2", "hand" ]
///   => we extract link indices (0, 2, 4, ...) => "base_link/link2/hand"
pub fn link_entity_path(
    link_paths_map: &HashMap<String, Vec<String>>,
    link_name: &str,
) -> Option<String> {
    if let Some(chain) = get_link_chain(link_paths_map, link_name) {
        let link_only: Vec<_> = chain
            .iter()
            .enumerate()
            .filter_map(|(i, nm)| if i % 2 == 0 { Some(nm.clone()) } else { None })
            .collect();
        Some(link_only.join("/"))
    } else {
        None
    }
}

/// Build a mapping from joint names to their corresponding entity paths, using the BFS map
/// rather than re-BFSing for each joint.
pub fn build_joint_name_to_entity_path(urdf_path: &str) -> anyhow::Result<HashMap<String, String>> {
    // 1) Parse the URDF
    let robot_model = urdf_rs::read_file(urdf_path)?;

    // 2) Build adjacency
    let adjacency = build_adjacency(&robot_model.joints);

    // 3) Find root link
    let root_link_name = find_root_link_name(&robot_model.links, &robot_model.joints)
        .unwrap_or_else(|| "base".to_string());

    // 4) Build a BFS-based map from link_name => chain
    let link_paths_map = build_link_paths_map(&adjacency, &root_link_name);

    // 5) For each joint, look up the path from root to `j.child.link`,
    //    extract only the link names, and insert into `map`.
    let mut map = HashMap::new();
    for j in &robot_model.joints {
        if let Some(chain) = link_paths_map.get(&j.child.link) {
            // keep only even indices => link names
            let link_only: Vec<_> = chain
                .iter()
                .enumerate()
                .filter_map(|(i, nm)| if i % 2 == 0 { Some(nm.clone()) } else { None })
                .collect();
            let path = link_only.join("/");
            map.insert(j.name.clone(), path);
        }
    }

    Ok(map)
}

pub fn build_joint_name_to_joint_info(urdf_path: &str) -> Result<HashMap<String, JointInfo>, anyhow::Error> {
    // 1) Parse the URDF
    let robot = urdf_rs::read_file(urdf_path)?;
    
    // 2) Build adjacency
    let adjacency = build_adjacency(&robot.joints);

    // 3) Find root link
    let root_link_name = find_root_link_name(&robot.links, &robot.joints)
        .unwrap_or_else(|| "base".to_string());

    // 4) Build BFS-based map from link_name => chain
    let link_paths_map = build_link_paths_map(&adjacency, &root_link_name);
    
    let mut joint_info_map = HashMap::new();
    
    // 5) For each joint, get the proper entity path and origin translation
    for joint in &robot.joints {
        // Get the proper entity path using our existing BFS logic
        if let Some(chain) = link_paths_map.get(&joint.child.link) {
            // Extract only the link names (even indices)
            let link_only: Vec<_> = chain
                .iter()
                .enumerate()
                .filter_map(|(i, nm)| if i % 2 == 0 { Some(nm.clone()) } else { None })
                .collect();
            
            let entity_path = link_only.join("/");
            
            // Get the translation from the joint's origin
            let translation = [
                joint.origin.xyz[0] as f32,
                joint.origin.xyz[1] as f32, 
                joint.origin.xyz[2] as f32
            ];
            
            let base_rotation = rotation_from_euler_xyz(
                joint.origin.rpy[0],
                joint.origin.rpy[1],
                joint.origin.rpy[2]
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

pub struct JointInfo {
    pub entity_path: String,
    pub origin_translation: [f32; 3],
    pub base_rotation: [f32; 9],
}
