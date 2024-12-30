use std::collections::{HashMap, HashSet, VecDeque};
use urdf_rs::{Joint, Link};

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

/// Get the chain of links and joints from root to target using BFS.
pub fn get_link_chain(
    adjacency: &HashMap<String, Vec<(Joint, String)>>,
    root_link: &str,
    target: &str,
) -> Option<Vec<String>> {
    let mut queue = VecDeque::new();
    queue.push_back((root_link.to_owned(), vec![root_link.to_owned()]));

    while let Some((cur_link, path_so_far)) = queue.pop_front() {
        if cur_link == target {
            return Some(path_so_far);
        }
        if let Some(kids) = adjacency.get(&cur_link) {
            for (j, c) in kids {
                let mut new_path = path_so_far.clone();
                new_path.push(j.name.clone());
                new_path.push(c.clone());
                queue.push_back((c.clone(), new_path));
            }
        }
    }
    None
}

/// Convert a chain of links and joints into a slash-separated path of link names.
pub fn link_entity_path(
    adjacency: &HashMap<String, Vec<(Joint, String)>>,
    root_link: &str,
    link_name: &str,
) -> Option<String> {
    if let Some(chain) = get_link_chain(adjacency, root_link, link_name) {
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

/// Build a mapping from joint names to their corresponding entity paths.
pub fn build_joint_name_to_entity_path(urdf_path: &str) -> anyhow::Result<HashMap<String, String>> {
    // 1) Parse the URDF
    let robot_model = urdf_rs::read_file(urdf_path)?;

    // 2) Build adjacency
    let adjacency = build_adjacency(&robot_model.joints);

    // 3) Find root link
    let root_link_name = find_root_link_name(&robot_model.links, &robot_model.joints)
        .unwrap_or_else(|| "base".to_string());

    // 4) For each joint, do a BFS to get the path of link-names only
    let mut map = HashMap::new();
    for j in &robot_model.joints {
        // BFS from root_link_name -> j.child.link
        if let Some(chain) = get_link_chain(&adjacency, &root_link_name, &j.child.link) {
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
