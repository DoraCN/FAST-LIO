//! Incremental k-d tree (ikd-Tree). Ported from
//! `fast_lio/include/ikd-Tree/ikd_Tree.cpp` / `ikd_Tree.h`.
//!
//! The C++ version performs large-subtree rebuilds on a background thread. This
//! port keeps the exact tree semantics but rebuilds inline (single-threaded),
//! which is equivalent from the correctness point of view.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::types::PointType;

/// `EPSS` from ikd_Tree.h.
const EPSS: f32 = 1e-6;
/// `Minimal_Unbalanced_Tree_Size`.
const MINIMAL_UNBALANCED_TREE_SIZE: i32 = 10;
/// `DOWNSAMPLE_SWITCH`.
const DOWNSAMPLE_SWITCH: bool = true;

/// `BoxPointType`.
#[derive(Clone, Copy, Debug, Default)]
pub struct BoxPointType {
    pub vertex_min: [f32; 3],
    pub vertex_max: [f32; 3],
}

/// A k-d tree node (`KD_TREE_NODE`).
#[derive(Clone, Debug)]
pub struct KdTreeNode {
    pub point: PointType,
    pub division_axis: usize,
    pub tree_size: i32,
    pub invalid_point_num: i32,
    pub down_del_num: i32,
    pub point_deleted: bool,
    pub tree_deleted: bool,
    pub point_downsample_deleted: bool,
    pub tree_downsample_deleted: bool,
    pub need_push_down_to_left: bool,
    pub need_push_down_to_right: bool,
    pub node_range_x: [f32; 2],
    pub node_range_y: [f32; 2],
    pub node_range_z: [f32; 2],
    pub radius_sq: f32,
    pub left_son: Option<Box<KdTreeNode>>,
    pub right_son: Option<Box<KdTreeNode>>,
    pub alpha_del: f32,
    pub alpha_bal: f32,
}

impl KdTreeNode {
    fn new() -> Self {
        Self {
            point: PointType::default(),
            division_axis: 0,
            tree_size: 0,
            invalid_point_num: 0,
            down_del_num: 0,
            point_deleted: false,
            tree_deleted: false,
            point_downsample_deleted: false,
            tree_downsample_deleted: false,
            need_push_down_to_left: false,
            need_push_down_to_right: false,
            node_range_x: [0.0; 2],
            node_range_y: [0.0; 2],
            node_range_z: [0.0; 2],
            radius_sq: 0.0,
            left_son: None,
            right_son: None,
            alpha_del: 0.0,
            alpha_bal: 0.0,
        }
    }
}

/// Mutable scratch buffers shared across tree operations.
#[derive(Default)]
struct Scratch {
    points_deleted: Vec<PointType>,
    downsample_storage: Vec<PointType>,
    pcl_storage: Vec<PointType>,
}

/// The incremental k-d tree (`KD_TREE`).
pub struct KdTree {
    pub root: Option<Box<KdTreeNode>>,
    pub pcl_storage: Vec<PointType>,
    delete_criterion_param: f32,
    balance_criterion_param: f32,
    downsample_size: f32,
    scratch: Scratch,
}

impl Default for KdTree {
    fn default() -> Self {
        Self::new()
    }
}

impl KdTree {
    pub fn new() -> Self {
        Self {
            root: None,
            pcl_storage: Vec::new(),
            delete_criterion_param: 0.5,
            balance_criterion_param: 0.6,
            downsample_size: 0.2,
            scratch: Scratch::default(),
        }
    }

    pub fn set_delete_criterion_param(&mut self, v: f32) {
        self.delete_criterion_param = v;
    }

    pub fn set_balance_criterion_param(&mut self, v: f32) {
        self.balance_criterion_param = v;
    }

    pub fn set_downsample_param(&mut self, v: f32) {
        self.downsample_size = v;
    }

    pub fn initialize(&mut self, delete_param: f32, balance_param: f32, box_length: f32) {
        self.set_delete_criterion_param(delete_param);
        self.set_balance_criterion_param(balance_param);
        self.set_downsample_param(box_length);
    }

    pub fn size(&self) -> i32 {
        self.root.as_ref().map(|n| n.tree_size).unwrap_or(0)
    }

    pub fn validnum(&self) -> i32 {
        self.root
            .as_ref()
            .map(|n| n.tree_size - n.invalid_point_num)
            .unwrap_or(0)
    }

    /// Build the tree from a point cloud (`Build`).
    pub fn build(&mut self, point_cloud: Vec<PointType>) {
        if self.root.is_some() {
            self.root = None;
        }
        if point_cloud.is_empty() {
            return;
        }
        let mut storage = point_cloud;
        build_tree(&mut self.root, 0, storage.len() - 1, &mut storage);
    }

    /// k-NN search (`Nearest_Search`). Results are sorted by distance ascending.
    pub fn nearest_search(
        &mut self,
        point: &PointType,
        k_nearest: i32,
        nearest_points: &mut Vec<PointType>,
        point_distance: &mut Vec<f32>,
        max_dist: f32,
    ) {
        let mut q = BinaryHeap::with_capacity(2 * k_nearest as usize);
        point_distance.clear();
        search(&mut self.root, k_nearest, point, &mut q, max_dist);
        nearest_points.clear();
        point_distance.clear();
        let k_found = k_nearest.min(q.len() as i32);
        let mut np = Vec::with_capacity(k_found as usize);
        let mut pd = Vec::with_capacity(k_found as usize);
        for _ in 0..k_found {
            let top = q.pop().unwrap();
            np.push(top.point);
            pd.push(top.dist);
        }
        np.reverse();
        pd.reverse();
        *nearest_points = np;
        *point_distance = pd;
    }

    /// Box search (`Box_Search`).
    pub fn box_search(&mut self, boxpoint: &BoxPointType, storage: &mut Vec<PointType>) {
        storage.clear();
        search_by_range(&mut self.root, boxpoint, storage);
    }

    /// Radius search (`Radius_Search`).
    pub fn radius_search(&mut self, point: &PointType, radius: f32, storage: &mut Vec<PointType>) {
        storage.clear();
        search_by_radius(&mut self.root, point, radius, storage);
    }

    /// Add points (`Add_Points`), returns the number of points actually added.
    pub fn add_points(&mut self, points: &mut [PointType], downsample_on: bool) -> i32 {
        let downsample_switch = downsample_on && DOWNSAMPLE_SWITCH;
        let mut tmp_counter = 0i32;
        for pt in points {
            let pt = *pt;
            if downsample_switch {
                let mut box_of_point = BoxPointType::default();
                for d in 0..3 {
                    let v = match d {
                        0 => pt.x,
                        1 => pt.y,
                        _ => pt.z,
                    };
                    let lo = (v / self.downsample_size).floor() * self.downsample_size;
                    box_of_point.vertex_min[d] = lo;
                    box_of_point.vertex_max[d] = lo + self.downsample_size;
                }
                let mid_point = PointType::new(
                    box_of_point.vertex_min[0]
                        + (box_of_point.vertex_max[0] - box_of_point.vertex_min[0]) / 2.0,
                    box_of_point.vertex_min[1]
                        + (box_of_point.vertex_max[1] - box_of_point.vertex_min[1]) / 2.0,
                    box_of_point.vertex_min[2]
                        + (box_of_point.vertex_max[2] - box_of_point.vertex_min[2]) / 2.0,
                );
                self.scratch.downsample_storage.clear();
                search_by_range(&mut self.root, &box_of_point, &mut self.scratch.downsample_storage);
                let mut min_dist = calc_dist(&pt, &mid_point);
                let mut downsample_result = pt;
                for ds in &self.scratch.downsample_storage {
                    let tmp_dist = calc_dist(ds, &mid_point);
                    if tmp_dist < min_dist {
                        min_dist = tmp_dist;
                        downsample_result = *ds;
                    }
                }
                if self.scratch.downsample_storage.len() > 1 || same_point(&pt, &downsample_result) {
                    if !self.scratch.downsample_storage.is_empty() {
                        self.delete_by_range(&box_of_point, true, true);
                    }
                    self.add_by_point(&downsample_result, true, self.root.as_ref().map(|n| n.division_axis).unwrap_or(0));
                    tmp_counter += 1;
                }
            } else {
                self.add_by_point(&pt, true, self.root.as_ref().map(|n| n.division_axis).unwrap_or(0));
            }
        }
        tmp_counter
    }

    /// Delete all points within boxes (`Delete_Point_Boxes`), returns the count.
    pub fn delete_point_boxes(&mut self, boxes: &[BoxPointType]) -> i32 {
        let mut tmp_counter = 0i32;
        for b in boxes {
            tmp_counter += self.delete_by_range(b, true, false);
        }
        tmp_counter
    }

    /// Drain the points removed during rebuilds (`acquire_removed_points`).
    pub fn acquire_removed_points(&mut self) -> Vec<PointType> {
        let mut removed = Vec::new();
        removed.append(&mut self.scratch.points_deleted);
        removed
    }

    /// Collect all valid points into `pcl_storage` (used for debug / flatten).
    pub fn flatten_to_storage(&mut self) {
        self.pcl_storage.clear();
        flatten(&mut self.root, &mut self.pcl_storage);
    }

    fn delete_by_range(&mut self, boxpoint: &BoxPointType, allow_rebuild: bool, is_downsample: bool) -> i32 {
        let root = &mut self.root;
        let params = (self.delete_criterion_param, self.balance_criterion_param);
        let scratch = &mut self.scratch;
        delete_by_range_impl(root, boxpoint, allow_rebuild, is_downsample, params, scratch)
    }

    fn add_by_point(&mut self, point: &PointType, allow_rebuild: bool, father_axis: usize) {
        let root = &mut self.root;
        let params = (self.delete_criterion_param, self.balance_criterion_param);
        let scratch = &mut self.scratch;
        add_by_point_impl(root, point, allow_rebuild, father_axis, params, scratch);
    }
}

// ---------------------------------------------------------------------------
// heap element
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
struct PointCmp {
    point: PointType,
    dist: f32,
}

impl PartialEq for PointCmp {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}
impl Eq for PointCmp {}
impl PartialOrd for PointCmp {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for PointCmp {
    fn cmp(&self, other: &Self) -> Ordering {
        // BinaryHeap is a max-heap; the top must be the point with the largest
        // distance (the worst neighbour to evict), matching C++ MANUAL_HEAP.
        if (self.dist - other.dist).abs() < 1e-10 {
            self.point.x.partial_cmp(&other.point.x).unwrap()
        } else {
            self.dist.partial_cmp(&other.dist).unwrap()
        }
    }
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

fn same_point(a: &PointType, b: &PointType) -> bool {
    (a.x - b.x).abs() < EPSS && (a.y - b.y).abs() < EPSS && (a.z - b.z).abs() < EPSS
}

fn calc_dist(a: &PointType, b: &PointType) -> f32 {
    (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z)
}

fn calc_box_dist(node: &KdTreeNode, point: &PointType) -> f32 {
    let mut min_dist = 0.0f32;
    if point.x < node.node_range_x[0] {
        min_dist += (point.x - node.node_range_x[0]) * (point.x - node.node_range_x[0]);
    }
    if point.x > node.node_range_x[1] {
        min_dist += (point.x - node.node_range_x[1]) * (point.x - node.node_range_x[1]);
    }
    if point.y < node.node_range_y[0] {
        min_dist += (point.y - node.node_range_y[0]) * (point.y - node.node_range_y[0]);
    }
    if point.y > node.node_range_y[1] {
        min_dist += (point.y - node.node_range_y[1]) * (point.y - node.node_range_y[1]);
    }
    if point.z < node.node_range_z[0] {
        min_dist += (point.z - node.node_range_z[0]) * (point.z - node.node_range_z[0]);
    }
    if point.z > node.node_range_z[1] {
        min_dist += (point.z - node.node_range_z[1]) * (point.z - node.node_range_z[1]);
    }
    min_dist
}

fn cmp_x(a: &PointType, b: &PointType) -> Ordering {
    a.x.partial_cmp(&b.x).unwrap()
}
fn cmp_y(a: &PointType, b: &PointType) -> Ordering {
    a.y.partial_cmp(&b.y).unwrap()
}
fn cmp_z(a: &PointType, b: &PointType) -> Ordering {
    a.z.partial_cmp(&b.z).unwrap()
}

// ---------------------------------------------------------------------------
// push_down / update
// ---------------------------------------------------------------------------

fn push_down(root: &mut Option<Box<KdTreeNode>>) {
    let node = match root {
        Some(n) => n,
        None => return,
    };
    let tree_deleted = node.tree_deleted;
    let tree_downsample_deleted = node.tree_downsample_deleted;

    if node.need_push_down_to_left {
        if let Some(left) = node.left_son.as_mut() {
            left.tree_downsample_deleted |= tree_downsample_deleted;
            left.point_downsample_deleted |= tree_downsample_deleted;
            left.tree_deleted = tree_deleted || left.tree_downsample_deleted;
            left.point_deleted = left.tree_deleted || left.point_downsample_deleted;
            if tree_downsample_deleted {
                left.down_del_num = left.tree_size;
            }
            if tree_deleted {
                left.invalid_point_num = left.tree_size;
            } else {
                left.invalid_point_num = left.down_del_num;
            }
            left.need_push_down_to_left = true;
            left.need_push_down_to_right = true;
        }
        node.need_push_down_to_left = false;
    }
    if node.need_push_down_to_right {
        if let Some(right) = node.right_son.as_mut() {
            right.tree_downsample_deleted |= tree_downsample_deleted;
            right.point_downsample_deleted |= tree_downsample_deleted;
            right.tree_deleted = tree_deleted || right.tree_downsample_deleted;
            right.point_deleted = right.tree_deleted || right.point_downsample_deleted;
            if tree_downsample_deleted {
                right.down_del_num = right.tree_size;
            }
            if tree_deleted {
                right.invalid_point_num = right.tree_size;
            } else {
                right.invalid_point_num = right.down_del_num;
            }
            right.need_push_down_to_left = true;
            right.need_push_down_to_right = true;
        }
        node.need_push_down_to_right = false;
    }
}

/// (tree_size, invalid, down_del, tree_downsample_deleted, tree_deleted, ranges)
type ChildInfo = (i32, i32, i32, bool, bool, [f32; 2], [f32; 2], [f32; 2]);

fn child_info(node: &KdTreeNode) -> Option<ChildInfo> {
    node.left_son.as_ref().map(|n| {
        (
            n.tree_size,
            n.invalid_point_num,
            n.down_del_num,
            n.tree_downsample_deleted,
            n.tree_deleted,
            n.node_range_x,
            n.node_range_y,
            n.node_range_z,
        )
    })
}

fn update(node: &mut KdTreeNode) {
    let left_info = child_info(node);
    let right_info = node
        .right_son
        .as_ref()
        .map(|n| {
            (
                n.tree_size,
                n.invalid_point_num,
                n.down_del_num,
                n.tree_downsample_deleted,
                n.tree_deleted,
                n.node_range_x,
                n.node_range_y,
                n.node_range_z,
            )
        });

    let (l, r) = (left_info, right_info);
    let mut tmp_range_x = [f32::INFINITY, f32::NEG_INFINITY];
    let mut tmp_range_y = [f32::INFINITY, f32::NEG_INFINITY];
    let mut tmp_range_z = [f32::INFINITY, f32::NEG_INFINITY];

    let (tree_size, invalid, down_del, tree_ds, tree_del);
    match (l, r) {
        (Some(l), Some(r)) => {
            tree_size = l.0 + r.0 + 1;
            invalid = l.1 + r.1 + (node.point_deleted as i32);
            down_del = l.2 + r.2 + (node.point_downsample_deleted as i32);
            tree_ds = l.3 & r.3 & node.point_downsample_deleted;
            tree_del = l.4 && r.4 && node.point_deleted;
            if tree_del || (!l.4 && !r.4 && !node.point_deleted) {
                tmp_range_x[0] = l.5[0].min(r.5[0]).min(node.point.x);
                tmp_range_x[1] = l.5[1].max(r.5[1]).max(node.point.x);
                tmp_range_y[0] = l.6[0].min(r.6[0]).min(node.point.y);
                tmp_range_y[1] = l.6[1].max(r.6[1]).max(node.point.y);
                tmp_range_z[0] = l.7[0].min(r.7[0]).min(node.point.z);
                tmp_range_z[1] = l.7[1].max(r.7[1]).max(node.point.z);
            } else {
                if !l.4 {
                    tmp_range_x[0] = tmp_range_x[0].min(l.5[0]);
                    tmp_range_x[1] = tmp_range_x[1].max(l.5[1]);
                    tmp_range_y[0] = tmp_range_y[0].min(l.6[0]);
                    tmp_range_y[1] = tmp_range_y[1].max(l.6[1]);
                    tmp_range_z[0] = tmp_range_z[0].min(l.7[0]);
                    tmp_range_z[1] = tmp_range_z[1].max(l.7[1]);
                }
                if !r.4 {
                    tmp_range_x[0] = tmp_range_x[0].min(r.5[0]);
                    tmp_range_x[1] = tmp_range_x[1].max(r.5[1]);
                    tmp_range_y[0] = tmp_range_y[0].min(r.6[0]);
                    tmp_range_y[1] = tmp_range_y[1].max(r.6[1]);
                    tmp_range_z[0] = tmp_range_z[0].min(r.7[0]);
                    tmp_range_z[1] = tmp_range_z[1].max(r.7[1]);
                }
                if !node.point_deleted {
                    tmp_range_x[0] = tmp_range_x[0].min(node.point.x);
                    tmp_range_x[1] = tmp_range_x[1].max(node.point.x);
                    tmp_range_y[0] = tmp_range_y[0].min(node.point.y);
                    tmp_range_y[1] = tmp_range_y[1].max(node.point.y);
                    tmp_range_z[0] = tmp_range_z[0].min(node.point.z);
                    tmp_range_z[1] = tmp_range_z[1].max(node.point.z);
                }
            }
        }
        (Some(l), None) => {
            tree_size = l.0 + 1;
            invalid = l.1 + (node.point_deleted as i32);
            down_del = l.2 + (node.point_downsample_deleted as i32);
            tree_ds = l.3 & node.point_downsample_deleted;
            tree_del = l.4 && node.point_deleted;
            if tree_del || (!l.4 && !node.point_deleted) {
                tmp_range_x[0] = l.5[0].min(node.point.x);
                tmp_range_x[1] = l.5[1].max(node.point.x);
                tmp_range_y[0] = l.6[0].min(node.point.y);
                tmp_range_y[1] = l.6[1].max(node.point.y);
                tmp_range_z[0] = l.7[0].min(node.point.z);
                tmp_range_z[1] = l.7[1].max(node.point.z);
            } else {
                if !l.4 {
                    tmp_range_x[0] = tmp_range_x[0].min(l.5[0]);
                    tmp_range_x[1] = tmp_range_x[1].max(l.5[1]);
                    tmp_range_y[0] = tmp_range_y[0].min(l.6[0]);
                    tmp_range_y[1] = tmp_range_y[1].max(l.6[1]);
                    tmp_range_z[0] = tmp_range_z[0].min(l.7[0]);
                    tmp_range_z[1] = tmp_range_z[1].max(l.7[1]);
                }
                if !node.point_deleted {
                    tmp_range_x[0] = tmp_range_x[0].min(node.point.x);
                    tmp_range_x[1] = tmp_range_x[1].max(node.point.x);
                    tmp_range_y[0] = tmp_range_y[0].min(node.point.y);
                    tmp_range_y[1] = tmp_range_y[1].max(node.point.y);
                    tmp_range_z[0] = tmp_range_z[0].min(node.point.z);
                    tmp_range_z[1] = tmp_range_z[1].max(node.point.z);
                }
            }
        }
        (None, Some(r)) => {
            tree_size = r.0 + 1;
            invalid = r.1 + (node.point_deleted as i32);
            down_del = r.2 + (node.point_downsample_deleted as i32);
            tree_ds = r.3 & node.point_downsample_deleted;
            tree_del = r.4 && node.point_deleted;
            if tree_del || (!r.4 && !node.point_deleted) {
                tmp_range_x[0] = r.5[0].min(node.point.x);
                tmp_range_x[1] = r.5[1].max(node.point.x);
                tmp_range_y[0] = r.6[0].min(node.point.y);
                tmp_range_y[1] = r.6[1].max(node.point.y);
                tmp_range_z[0] = r.7[0].min(node.point.z);
                tmp_range_z[1] = r.7[1].max(node.point.z);
            } else {
                if !r.4 {
                    tmp_range_x[0] = tmp_range_x[0].min(r.5[0]);
                    tmp_range_x[1] = tmp_range_x[1].max(r.5[1]);
                    tmp_range_y[0] = tmp_range_y[0].min(r.6[0]);
                    tmp_range_y[1] = tmp_range_y[1].max(r.6[1]);
                    tmp_range_z[0] = tmp_range_z[0].min(r.7[0]);
                    tmp_range_z[1] = tmp_range_z[1].max(r.7[1]);
                }
                if !node.point_deleted {
                    tmp_range_x[0] = tmp_range_x[0].min(node.point.x);
                    tmp_range_x[1] = tmp_range_x[1].max(node.point.x);
                    tmp_range_y[0] = tmp_range_y[0].min(node.point.y);
                    tmp_range_y[1] = tmp_range_y[1].max(node.point.y);
                    tmp_range_z[0] = tmp_range_z[0].min(node.point.z);
                    tmp_range_z[1] = tmp_range_z[1].max(node.point.z);
                }
            }
        }
        (None, None) => {
            tree_size = 1;
            invalid = node.point_deleted as i32;
            down_del = node.point_downsample_deleted as i32;
            tree_ds = node.point_downsample_deleted;
            tree_del = node.point_deleted;
            tmp_range_x[0] = node.point.x;
            tmp_range_x[1] = node.point.x;
            tmp_range_y[0] = node.point.y;
            tmp_range_y[1] = node.point.y;
            tmp_range_z[0] = node.point.z;
            tmp_range_z[1] = node.point.z;
        }
    }
    node.tree_size = tree_size;
    node.invalid_point_num = invalid;
    node.down_del_num = down_del;
    node.tree_downsample_deleted = tree_ds;
    node.tree_deleted = tree_del;
    node.node_range_x = tmp_range_x;
    node.node_range_y = tmp_range_y;
    node.node_range_z = tmp_range_z;
    let x_l = (node.node_range_x[1] - node.node_range_x[0]) * 0.5;
    let y_l = (node.node_range_y[1] - node.node_range_y[0]) * 0.5;
    let z_l = (node.node_range_z[1] - node.node_range_z[0]) * 0.5;
    node.radius_sq = x_l * x_l + y_l * y_l + z_l * z_l;
}

// ---------------------------------------------------------------------------
// criterion / rebuild
// ---------------------------------------------------------------------------

fn criterion_check(node: &KdTreeNode, delete_criterion: f32, balance_criterion: f32) -> bool {
    if node.tree_size <= MINIMAL_UNBALANCED_TREE_SIZE {
        return false;
    }
    let son_ptr = node
        .left_son
        .as_ref()
        .or(node.right_son.as_ref())
        .unwrap();
    let delete_evaluation = node.invalid_point_num as f32 / node.tree_size as f32;
    let balance_evaluation = son_ptr.tree_size as f32 / (node.tree_size - 1) as f32;
    if delete_evaluation > delete_criterion {
        return true;
    }
    if balance_evaluation > balance_criterion || balance_evaluation < 1.0 - balance_criterion {
        return true;
    }
    false
}

fn flatten(root: &mut Option<Box<KdTreeNode>>, storage: &mut Vec<PointType>) {
    if root.is_none() {
        return;
    }
    push_down(root);
    let node = root.as_mut().unwrap();
    let point = node.point;
    let point_deleted = node.point_deleted;
    if !point_deleted {
        storage.push(point);
    }
    flatten(&mut node.left_son, storage);
    flatten(&mut node.right_son, storage);
}

fn build_tree(root: &mut Option<Box<KdTreeNode>>, l: usize, r: usize, storage: &mut [PointType]) {
    if l > r {
        return;
    }
    let mid = (l + r) >> 1;
    let mut min_value = [f32::INFINITY; 3];
    let mut max_value = [f32::NEG_INFINITY; 3];
    for p in &storage[l..=r] {
        min_value[0] = min_value[0].min(p.x);
        min_value[1] = min_value[1].min(p.y);
        min_value[2] = min_value[2].min(p.z);
        max_value[0] = max_value[0].max(p.x);
        max_value[1] = max_value[1].max(p.y);
        max_value[2] = max_value[2].max(p.z);
    }
    let mut div_axis = 0usize;
    let dim_range = [
        max_value[0] - min_value[0],
        max_value[1] - min_value[1],
        max_value[2] - min_value[2],
    ];
    for i in 1..3 {
        if dim_range[i] > dim_range[div_axis] {
            div_axis = i;
        }
    }
    let slice = &mut storage[l..=r];
    let cmp = match div_axis {
        0 => cmp_x,
        1 => cmp_y,
        _ => cmp_z,
    };
    slice.select_nth_unstable_by(mid - l, cmp);
    let point = slice[mid - l];

    let mut node = Box::new(KdTreeNode::new());
    node.point = point;
    node.division_axis = div_axis;
    if mid > 0 {
        build_tree(&mut node.left_son, l, mid - 1, storage);
    }
    build_tree(&mut node.right_son, mid + 1, r, storage);
    update(&mut node);
    *root = Some(node);
}

fn rebuild(
    root: &mut Option<Box<KdTreeNode>>,
    delete_criterion: f32,
    balance_criterion: f32,
    scratch: &mut Scratch,
) {
    let _ = (delete_criterion, balance_criterion);
    scratch.pcl_storage.clear();
    flatten(root, &mut scratch.pcl_storage);
    *root = None;
    if !scratch.pcl_storage.is_empty() {
        let n = scratch.pcl_storage.len() - 1;
        build_tree(root, 0, n, &mut scratch.pcl_storage);
    }
}

// ---------------------------------------------------------------------------
// add / delete
// ---------------------------------------------------------------------------

fn add_by_point_impl(
    root: &mut Option<Box<KdTreeNode>>,
    point: &PointType,
    allow_rebuild: bool,
    father_axis: usize,
    params: (f32, f32),
    scratch: &mut Scratch,
) {
    if root.is_none() {
        let mut new_node = Box::new(KdTreeNode::new());
        new_node.point = *point;
        new_node.division_axis = (father_axis + 1) % 3;
        update(&mut new_node);
        *root = Some(new_node);
        return;
    }
    let (axis, px, py, pz) = {
        let n = root.as_ref().unwrap();
        (n.division_axis, n.point.x, n.point.y, n.point.z)
    };
    push_down(root);
    let node = root.as_mut().unwrap();
    let go_left = (axis == 0 && point.x < px) || (axis == 1 && point.y < py) || (axis == 2 && point.z < pz);
    if go_left {
        add_by_point_impl(&mut node.left_son, point, allow_rebuild, axis, params, scratch);
    } else {
        add_by_point_impl(&mut node.right_son, point, allow_rebuild, axis, params, scratch);
    }
    update(node);
    let (delete_criterion, balance_criterion) = params;
    let need = allow_rebuild && criterion_check(node, delete_criterion, balance_criterion);
    if need {
        rebuild(root, delete_criterion, balance_criterion, scratch);
    }
}

fn delete_by_range_impl(
    root: &mut Option<Box<KdTreeNode>>,
    boxpoint: &BoxPointType,
    allow_rebuild: bool,
    is_downsample: bool,
    params: (f32, f32),
    scratch: &mut Scratch,
) -> i32 {
    if root.as_ref().map(|n| n.tree_deleted).unwrap_or(true) {
        return 0;
    }
    push_down(root);
    let node = root.as_mut().unwrap();
    if boxpoint.vertex_max[0] <= node.node_range_x[0] || boxpoint.vertex_min[0] > node.node_range_x[1] {
        return 0;
    }
    if boxpoint.vertex_max[1] <= node.node_range_y[0] || boxpoint.vertex_min[1] > node.node_range_y[1] {
        return 0;
    }
    if boxpoint.vertex_max[2] <= node.node_range_z[0] || boxpoint.vertex_min[2] > node.node_range_z[1] {
        return 0;
    }
    if boxpoint.vertex_min[0] <= node.node_range_x[0]
        && boxpoint.vertex_max[0] > node.node_range_x[1]
        && boxpoint.vertex_min[1] <= node.node_range_y[0]
        && boxpoint.vertex_max[1] > node.node_range_y[1]
        && boxpoint.vertex_min[2] <= node.node_range_z[0]
        && boxpoint.vertex_max[2] > node.node_range_z[1]
    {
        node.tree_deleted = true;
        node.point_deleted = true;
        node.need_push_down_to_left = true;
        node.need_push_down_to_right = true;
        let tmp_counter = node.tree_size - node.invalid_point_num;
        node.invalid_point_num = node.tree_size;
        if is_downsample {
            node.tree_downsample_deleted = true;
            node.point_downsample_deleted = true;
            node.down_del_num = node.tree_size;
        }
        return tmp_counter;
    }
    let mut tmp_counter = 0i32;
    if !node.point_deleted
        && boxpoint.vertex_min[0] <= node.point.x
        && boxpoint.vertex_max[0] > node.point.x
        && boxpoint.vertex_min[1] <= node.point.y
        && boxpoint.vertex_max[1] > node.point.y
        && boxpoint.vertex_min[2] <= node.point.z
        && boxpoint.vertex_max[2] > node.point.z
    {
        node.point_deleted = true;
        tmp_counter += 1;
        if is_downsample {
            node.point_downsample_deleted = true;
        }
    }
    tmp_counter += delete_by_range_impl(&mut node.left_son, boxpoint, allow_rebuild, is_downsample, params, scratch);
    tmp_counter += delete_by_range_impl(&mut node.right_son, boxpoint, allow_rebuild, is_downsample, params, scratch);
    update(node);
    let (delete_criterion, balance_criterion) = params;
    let need = allow_rebuild && criterion_check(node, delete_criterion, balance_criterion);
    if need {
        rebuild(root, delete_criterion, balance_criterion, scratch);
    }
    tmp_counter
}

#[allow(dead_code)]
fn delete_by_point_impl(
    root: &mut Option<Box<KdTreeNode>>,
    point: &PointType,
    allow_rebuild: bool,
    params: (f32, f32),
    scratch: &mut Scratch,
) {
    if root.as_ref().map(|n| n.tree_deleted).unwrap_or(true) {
        return;
    }
    push_down(root);
    let node = root.as_mut().unwrap();
    if same_point(&node.point, point) && !node.point_deleted {
        node.point_deleted = true;
        node.invalid_point_num += 1;
        if node.invalid_point_num == node.tree_size {
            node.tree_deleted = true;
        }
        return;
    }
    let axis = node.division_axis;
    let go_left = (axis == 0 && point.x < node.point.x)
        || (axis == 1 && point.y < node.point.y)
        || (axis == 2 && point.z < node.point.z);
    if go_left {
        delete_by_point_impl(&mut node.left_son, point, allow_rebuild, params, scratch);
    } else {
        delete_by_point_impl(&mut node.right_son, point, allow_rebuild, params, scratch);
    }
    update(node);
    let (delete_criterion, balance_criterion) = params;
    let need = allow_rebuild && criterion_check(node, delete_criterion, balance_criterion);
    if need {
        rebuild(root, delete_criterion, balance_criterion, scratch);
    }
}

// ---------------------------------------------------------------------------
// search
// ---------------------------------------------------------------------------

fn search(
    root: &mut Option<Box<KdTreeNode>>,
    k_nearest: i32,
    point: &PointType,
    q: &mut BinaryHeap<PointCmp>,
    max_dist: f32,
) {
    if root.is_none() {
        return;
    }
    if root.as_ref().map(|n| n.tree_deleted).unwrap_or(true) {
        return;
    }
    let cur_dist = root.as_ref().map(|n| calc_box_dist(n, point)).unwrap_or(f32::INFINITY);
    let max_dist_sqr = max_dist * max_dist;
    if cur_dist > max_dist_sqr {
        return;
    }
    let need_push = root
        .as_ref()
        .map(|n| n.need_push_down_to_left || n.need_push_down_to_right)
        .unwrap_or(false);
    if need_push {
        push_down(root);
    }
    let node = root.as_mut().unwrap();
    if !node.point_deleted {
        let dist = calc_dist(point, &node.point);
        if dist <= max_dist_sqr && (q.len() < k_nearest as usize || dist < q.peek().map(|p| p.dist).unwrap_or(f32::INFINITY)) {
            if q.len() >= k_nearest as usize {
                q.pop();
            }
            q.push(PointCmp {
                point: node.point,
                dist,
            });
        }
    }
    let dist_left_node = node
        .left_son
        .as_ref()
        .map(|n| calc_box_dist(n, point))
        .unwrap_or(f32::INFINITY);
    let dist_right_node = node
        .right_son
        .as_ref()
        .map(|n| calc_box_dist(n, point))
        .unwrap_or(f32::INFINITY);
    let top_dist = q.peek().map(|p| p.dist).unwrap_or(f32::INFINITY);
    if q.len() < k_nearest as usize || (dist_left_node < top_dist && dist_right_node < top_dist) {
        if dist_left_node <= dist_right_node {
            search(&mut node.left_son, k_nearest, point, q, max_dist);
            let top_dist = q.peek().map(|p| p.dist).unwrap_or(f32::INFINITY);
            if q.len() < k_nearest as usize || dist_right_node < top_dist {
                search(&mut node.right_son, k_nearest, point, q, max_dist);
            }
        } else {
            search(&mut node.right_son, k_nearest, point, q, max_dist);
            let top_dist = q.peek().map(|p| p.dist).unwrap_or(f32::INFINITY);
            if q.len() < k_nearest as usize || dist_left_node < top_dist {
                search(&mut node.left_son, k_nearest, point, q, max_dist);
            }
        }
    } else {
        if dist_left_node < top_dist {
            search(&mut node.left_son, k_nearest, point, q, max_dist);
        }
        if dist_right_node < top_dist {
            search(&mut node.right_son, k_nearest, point, q, max_dist);
        }
    }
}

fn search_by_range(root: &mut Option<Box<KdTreeNode>>, boxpoint: &BoxPointType, storage: &mut Vec<PointType>) {
    if root.is_none() {
        return;
    }
    push_down(root);
    let node = root.as_mut().unwrap();
    if boxpoint.vertex_max[0] <= node.node_range_x[0] || boxpoint.vertex_min[0] > node.node_range_x[1] {
        return;
    }
    if boxpoint.vertex_max[1] <= node.node_range_y[0] || boxpoint.vertex_min[1] > node.node_range_y[1] {
        return;
    }
    if boxpoint.vertex_max[2] <= node.node_range_z[0] || boxpoint.vertex_min[2] > node.node_range_z[1] {
        return;
    }
    if boxpoint.vertex_min[0] <= node.node_range_x[0]
        && boxpoint.vertex_max[0] > node.node_range_x[1]
        && boxpoint.vertex_min[1] <= node.node_range_y[0]
        && boxpoint.vertex_max[1] > node.node_range_y[1]
        && boxpoint.vertex_min[2] <= node.node_range_z[0]
        && boxpoint.vertex_max[2] > node.node_range_z[1]
    {
        flatten(root, storage);
        return;
    }
    if boxpoint.vertex_min[0] <= node.point.x
        && boxpoint.vertex_max[0] > node.point.x
        && boxpoint.vertex_min[1] <= node.point.y
        && boxpoint.vertex_max[1] > node.point.y
        && boxpoint.vertex_min[2] <= node.point.z
        && boxpoint.vertex_max[2] > node.point.z
        && !node.point_deleted {
            storage.push(node.point);
        }
    search_by_range(&mut node.left_son, boxpoint, storage);
    search_by_range(&mut node.right_son, boxpoint, storage);
}

fn search_by_radius(root: &mut Option<Box<KdTreeNode>>, point: &PointType, radius: f32, storage: &mut Vec<PointType>) {
    if root.is_none() {
        return;
    }
    push_down(root);
    let node = root.as_mut().unwrap();
    let range_center = PointType::new(
        (node.node_range_x[0] + node.node_range_x[1]) * 0.5,
        (node.node_range_y[0] + node.node_range_y[1]) * 0.5,
        (node.node_range_z[0] + node.node_range_z[1]) * 0.5,
    );
    let dist = calc_dist(&range_center, point).sqrt();
    if dist > radius + node.radius_sq.sqrt() {
        return;
    }
    if dist <= radius - node.radius_sq.sqrt() {
        flatten(root, storage);
        return;
    }
    if !node.point_deleted && calc_dist(&node.point, point) <= radius * radius {
        storage.push(node.point);
    }
    search_by_radius(&mut node.left_son, point, radius, storage);
    search_by_radius(&mut node.right_son, point, radius, storage);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rand_points(n: usize) -> Vec<PointType> {
        // deterministic pseudo-random points
        let mut pts = Vec::with_capacity(n);
        for i in 0..n {
            let t = i as f32 * 0.7919;
            pts.push(PointType::new(
                (t * 13.0).sin() * 10.0,
                (t * 7.0).cos() * 10.0,
                (t * 3.0).sin() * 5.0,
            ));
        }
        pts
    }

    fn brute_force_knn(points: &[PointType], q: &PointType, k: usize) -> Vec<(PointType, f32)> {
        let mut all: Vec<(PointType, f32)> = points
            .iter()
            .map(|p| (*p, calc_dist(p, q)))
            .collect();
        all.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        all.truncate(k);
        all
    }

    #[test]
    fn build_and_knn_matches_bruteforce() {
        let pts = rand_points(300);
        let mut tree = KdTree::new();
        tree.build(pts.clone());
        assert_eq!(tree.size(), 300);
        assert_eq!(tree.validnum(), 300);

        for trial in 0..10 {
            let q = rand_points(500)[trial * 50];
            let mut np = Vec::new();
            let mut pd = Vec::new();
            tree.nearest_search(&q, 5, &mut np, &mut pd, f32::INFINITY);
            assert_eq!(np.len(), 5);
            let expected = brute_force_knn(&pts, &q, 5);
            for i in 0..5 {
                assert!((pd[i] - expected[i].1).abs() < 1e-3, "trial={} i={} got {} exp {}", trial, i, pd[i], expected[i].1);
            }
        }
    }

    #[test]
    fn add_points_are_found() {
        let mut tree = KdTree::new();
        tree.build(rand_points(50));
        let mut extra: Vec<PointType> = (0..100)
            .map(|i| {
                let t = i as f32 * 1.1;
                PointType::new(20.0 + t, -15.0 + (t * 0.3).sin() * 5.0, t * 0.5)
            })
            .collect();
        tree.add_points(&mut extra, false);
        let q = PointType::new(1.0, 2.0, 3.0);
        let mut np = Vec::new();
        let mut pd = Vec::new();
        tree.nearest_search(&q, 3, &mut np, &mut pd, f32::INFINITY);
        assert_eq!(np.len(), 3);
        assert_eq!(tree.size(), 150);
    }

    #[test]
    fn delete_box_removes_points() {
        let pts = rand_points(200);
        let mut tree = KdTree::new();
        tree.build(pts.clone());
        let boxpoint = BoxPointType {
            vertex_min: [-1.0, -1.0, -1.0],
            vertex_max: [1.0, 1.0, 1.0],
        };
        let deleted = tree.delete_point_boxes(&[boxpoint]);
        let inside = pts
            .iter()
            .filter(|p| {
                p.x > -1.0 && p.x <= 1.0 && p.y > -1.0 && p.y <= 1.0 && p.z > -1.0 && p.z <= 1.0
            })
            .count() as i32;
        assert_eq!(deleted, inside);
        assert_eq!(tree.validnum(), 200 - inside);
        // points inside the box should no longer be returned
        let mut np = Vec::new();
        let mut pd = Vec::new();
        tree.nearest_search(&PointType::new(0.0, 0.0, 0.0), 1, &mut np, &mut pd, f32::INFINITY);
        if let Some(nn) = np.first() {
            assert!(!(nn.x.abs() <= 1.0 && nn.y.abs() <= 1.0 && nn.z.abs() <= 1.0));
        }
    }

    #[test]
    fn downsample_add_keeps_single_point_per_cell() {
        let mut tree = KdTree::new();
        tree.set_downsample_param(0.5);
        // many points inside one 0.5x0.5x0.5 cell
        let mut pts = Vec::new();
        for i in 0..20 {
            pts.push(PointType::new(0.1 + i as f32 * 0.01, 0.1, 0.1));
        }
        tree.build(pts);
        let mut add = vec![PointType::new(0.15, 0.15, 0.15)];
        tree.add_points(&mut add, true);
        // after downsample add, the cell should hold ~1 point
        assert!(tree.validnum() <= 2, "validnum = {}", tree.validnum());
    }
}
