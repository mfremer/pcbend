#pragma once

// #include <global_parameters.hpp>
#include "FaceInfo.hpp"
#include "Settings.hpp"

#include <LibSL/LibSL.h>

#include <array>
#include <algorithm>
#include <map>
#include <string>
#include <vector>

typedef std::array<size_t, 2> v2sz;
typedef std::array<size_t, 4> v4sz;
typedef std::vector<size_t> vNsz;

struct Edge {
    int hinge_id;           // -1 if not a hinge, else hinge_id
    v2sz adjacent_faces;    // ids of adjacent faces
    double offset;          // offset size
    bool is_hinge;
    bool is_halfhinge;
    bool is_boundary;
};

enum class VertexType {
    face,
    hinge
};

// <face/hinge, hid, vid, fid>
typedef std::tuple<VertexType, size_t, size_t, size_t> ExtendedVertex;

class Sheet {
public:
    Sheet() = default;

    void use_settings(const std::string& file_settings);
    void read_sheet(const std::string& file_sheet);
    void init_sheet();

    /* Checks that faces and hinges are provided in CCW order,
     * and that hinges' first edges are parallel to the face edge.
     * Should also check that edges in FE fields follow vertex order:
     * CCW and edge[i] lies between vert[i] and vert[i+1]. */
    bool validate_sheet() const;

    // Faces written following sheet order
    void write_faces(const std::string& file_fcs, const std::vector<int>& connector_sfids, bool split) const;

    // Layout2SVG
    void compute_order(); // DFS for contour construction
    void compute_bfs_depth();
    std::pair<size_t, double> half_hinges_max_shrink() const;
    bool faces_in_same_patch(size_t sfid0, size_t sfid1) const;
    bool lt_depth(size_t sfid0, size_t sfid1) const;

    void compute_patch_bbox();
    AAB<2, double> get_patch_bbox(size_t pid) const;
    v2d get_patch_extent(size_t pid) const;
    size_t patch_count() const;
    size_t hinge_count() const;
    std::pair<size_t, size_t> sfid_to_pfid(size_t sfid) const;
    std::vector<ExtendedVertex> get_vertex_order(size_t pid) const;
    std::vector<size_t> get_patch_hinges(size_t pid) const;
    
    v2d get_vertex(size_t vid, bool translate = false) const;
    bool is_half_hinge(size_t hid) const;
    v2d get_hinge_parallel(size_t hid) const;
    v2d get_hinge_ortho(size_t hid) const;
    v2d get_hinge_center(size_t hid, bool translate = false) const;
    double get_hinge_length(size_t hid) const;
    double get_hinge_min_length(size_t hid) const;

    // <track_width, inner_diameter, outer_diameter>
    std::tuple<double, double, double> get_hinge_parameters() const;
    std::pair<double, double> shrink_half_hinge(size_t hid, size_t fid0, size_t fid1) const;
    double shrink_half_hinge_total(size_t hid) const;
    v2d shrink_half_hinge_center(size_t hid) const;

    std::vector<FaceInfo> get_faces(const std::vector<int>& connector_sfids) const;
    bool is_half_hinge(size_t fid, size_t i) const;
    bool is_wide(size_t fid, size_t i) const;
    int get_hid(size_t fid, size_t i) const;

    std::vector<size_t> get_edges(size_t face_id);

    int get_edge_id_between_faces(size_t face_id0, size_t face_id1) const;

    // std::vector<double> vorlayout_inset_values(size_t fid) const {
    //     size_t num_edges = m_face_to_adjacent_edges.at(fid).size();
    //     std::vector<double> insets(num_edges);
    //     for (size_t i = 0; i < insets.size(); ++i) {
    //         bool cond = is_half_hinge(fid, i) && is_wide(fid, i);
    //         double inset_normal = LED_INSET_DEPTH;
    //         double inset_wide = LED_INSET_DEPTH_WIDE;
    //         insets[i] = cond ? inset_wide : inset_normal;
    //     }
    //     return insets;
    // }

private:
    double get_hinge_width(size_t hid) const ;
    std::vector<v2d> get_face_vertices(size_t i) const;
    std::array<v2d, 4> get_hinge_vertices(size_t i) const;

    bool is_face_ccw(size_t i) const;
    bool is_hinge_ccw(size_t i) const;
    bool is_hinge_normal(size_t i) const;
    bool is_face_edge_order_consistent(size_t i) const;

    // Returns IDs of faces connected through a hinge to "face_id" in CCW order
    // [UNUSED] std::vector<size_t> get_adjacent_face_ids(size_t face_id) const;
    // Returns ID of edge between provided faces, and -1 if the faces are not by an edge

    // no connector info
    FaceInfo get_face_info(size_t face_id) const;
    std::vector<size_t> get_connector_sfids(const std::vector<int>& conn_sfids) const;

    // Needed for compute_order()
    size_t get_vid_in_fid_before_hid(size_t fid, size_t hid) const;
    size_t get_hinge_vid_after_face_vid(size_t hid, size_t vid) const;
    size_t get_next_vid_in_hid(size_t hid, size_t vid) const;
    size_t get_fid_across_hid_from_fid(size_t fid, size_t hid) const;
    size_t get_vid_in_fid_after_hid(size_t fid, size_t hid) const;
    // returns -1 if not before hinge and hid if before hinge
    int is_face_vid_before_hinge(size_t fid, size_t vid) const;
    size_t get_closest_hinge_vid_from_vid(size_t hid, size_t vid) const;
    size_t get_next_vid_in_fid(size_t fid, size_t vid) const;
    size_t get_prev_vid_in_fid(size_t fid, size_t vid) const;

    size_t vid_to_pid(size_t vid) const;

    // shrink half-hinges aux functions
    double shrink_vertex(v2d vt0, v2d vt1, v2d vt_prev, v2d ht, double inset, double offset) const;
    std::tuple<v2d, v2d, v2d, v2d, v2d, v2d, v2d, v2d, v2d, v2d, v2d, v2d>
    shrink_points(size_t fid0, size_t fid1, size_t hid) const;
    std::pair<double, double> shrink_compute(
        size_t fid0, size_t fid1, size_t hid,
        v2d f0_v0, v2d f0_v1, v2d f1_v0, v2d f1_v1,
        v2d f0_ht0, v2d f0_ht1, v2d f1_ht0, v2d f1_ht1,
        v2d f0_v0_prev, v2d f0_v1_next, v2d f1_v0_prev, v2d f1_v1_next) const;

    // Sheet info (from file)
    std::string m_file_sheet;
    std::vector<v2d> m_vertices;
    std::vector<vNsz> m_faces;
    std::vector<v4sz> m_hinges;
    std::vector<Edge> m_edges;
    std::map<size_t, v2sz> m_hinge_to_adjacent_faces; // hid -> fid
    std::map<size_t, vNsz> m_face_to_adjacent_edges; // fid -> eid, eid in order of vs
    std::map<size_t, size_t> m_hid_to_eid;

    // Unfolder parameters
    // double m_drill_bit_size;
    // double m_user_fabrication_margin;
    
    // Hinge parameters
    // double m_track_width;
    // double m_inner_diameter;
    // double m_outer_diameter;

    // Patch info
    size_t m_patch_count;
    std::vector<std::vector<size_t>> m_patch_faces;
    std::vector<std::vector<size_t>> m_patch_hinges;
    // m_sfid_to_pfid.at(sfid) -> <patch_id, face_id> s.t. sfid = m_patch_faces.at(patch_id).at(face_id)
    std::map<size_t, std::pair<size_t, size_t>> m_sfid_to_pfid;
    // std::map<std::pair<size_t, size_t>, size_t> m_pfid_to_sfid;
    std::map<size_t, size_t> m_vid_to_pid;
    std::vector<AAB<2, double>> m_patch_bbox;
    std::map<size_t, size_t> m_sfid_to_depth;

    std::vector<std::vector<ExtendedVertex>> m_vertex_order;

    Settings m_settings;
};