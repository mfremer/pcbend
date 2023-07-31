#include "Sheet.hpp"

// #include <global_parameters.hpp>
#include <parsing_helpers.hpp>
#include <FaceInfo.hpp>

#include <algorithm>
#include <queue>
#include <sstream>

#include <cassert>

void Sheet::use_settings(const std::string& file_settings) {
    m_settings = read_settings(file_settings);
}

void Sheet::read_sheet(const std::string& file_sheet) {
    m_file_sheet = file_sheet;

    std::ifstream file(m_file_sheet);
    if (!file.is_open()) {
        std::cerr << "[ERROR] Could not open file at " << m_file_sheet << '\n';
        exit(EXIT_FAILURE);
    }

    std::string line;
    size_t line_number = 0;
    size_t vertex_count = 0;
    size_t face_count = 0;
    size_t hinge_count = 0;
    while (std::getline(file, line)) {
        bool is_comment = line.at(0) == '#';
        ++line_number;
        size_t header_end = line.find_first_of(' ');
        std::string line_prefix = line.substr(0, header_end);
        line = line.erase(0, header_end + 1);
        /* #: comment or parameter
         * si: sheet info (#vertices, #tri faces, #hinge faces)
         * up: unfolder parameters (drill bit radius, user fab margin)
         * hp: hinge parameters (track width, inner diameter, outer diameter)
         * v: vertex (triangle or hinge) [3x double]
         * f: triangle face indices [3x uint]
         * h: hinge face indices (starting with parallel edge) [4x uint]
         * e: edge adj info [eid, adjtri_id0, adjtri_id1,
         *      offset, is_hinge, is_halfhinge, is_boundary]
         * fe: face adj info [fid, adjedge_id0, adjedge_id1]
         */
        if (is_comment) {
            continue;
        } else if (line_prefix == "si") {
            auto [vc, tc, hc] = string_to_tuple<size_t, size_t, size_t>(line);
            vertex_count = vc;
            face_count = tc;
            hinge_count = hc;
        } else if (line_prefix == "up") {
            auto [dbr, ufm] = string_to_tuple<double, double>(line);
            // m_drill_bit_size = dbr;
            // m_user_fabrication_margin = ufm;
        } else if (line_prefix == "hp") {
            auto [tw, id, od] = string_to_tuple<double, double, double>(line);
            // m_track_width = tw;
            // m_inner_diameter = id;
            // m_outer_diameter = od;
        } else if (line_prefix == "v") {
            auto [x, y, z] = string_to_tuple<double, double, double>(line);
            m_vertices.push_back(v2d(x, y));
        } else if (line_prefix == "f") {
            auto ids = string_to_vector<size_t>(line);
            m_faces.push_back(ids);
        } else if (line_prefix == "h") {
            auto [i0, i1, i2, i3] = string_to_tuple<size_t, size_t, size_t, size_t>(line);
            m_hinges.push_back(v4sz{ i0, i1, i2, i3 });
        } else if (line_prefix == "e") {
            auto vals = string_to_vector<double>(line);
            if (vals.size() == 7) {
                auto [hid, af0, af1, off, is_hinge, is_halfhinge, is_boundary] =
                    string_to_tuple<int, size_t, size_t, double, bool, bool, bool>(line);
                Edge e{ hid, af0, af1, off, is_hinge, is_halfhinge, is_boundary };
                if (hid != -1) {
                    m_hid_to_eid[hid] = m_edges.size();
                }
                m_edges.push_back(e);
                if (hid >= 0) { m_hinge_to_adjacent_faces[hid] = v2sz{ af0, af1 }; }
            } else if (vals.size() == 6) {
                auto [hid, af0, af1, off, is_hinge, is_boundary] =
                    string_to_tuple<int, size_t, size_t, double, bool, bool>(line);
                Edge e{ hid, af0, af1, off, is_hinge, false, is_boundary };
                if (hid != -1) {
                    m_hid_to_eid[hid] = m_edges.size();
                }
                m_edges.push_back(e);
                if (hid >= 0) { m_hinge_to_adjacent_faces[hid] = v2sz{ af0, af1 }; }
            } else { assert(false); }
        } else if (line_prefix == "fe") {
            auto vals = string_to_vector<size_t>(line);
            m_face_to_adjacent_edges[vals.at(0)] = vNsz(vals.begin() + 1, vals.end());
        } else {
            std::cerr << "[ERROR] Unhandled case in line " << line_number
                      << " of file " << m_file_sheet << '\n';
            std::cerr << "Case is " << line_prefix << '\n';
            exit(EXIT_FAILURE);
        }
    }

    if (!(vertex_count == m_vertices.size() &&
          face_count   == m_faces.size() &&
          hinge_count  == m_hinges.size())) {
        std::cerr << "[ERROR] Parsed vertex/face/hinge count does not match the one indicated in the file\n";
        exit(EXIT_FAILURE);
    }

    file.close();
}

void Sheet::init_sheet() {
    validate_sheet();
    compute_order();
    compute_bfs_depth();
    compute_patch_bbox();
}

bool Sheet::validate_sheet() const {
    bool valid = true;
    for (size_t i = 0; i < m_faces.size(); ++i) {
        if (!is_face_ccw(i)) {
            std::cerr << "[ERROR] Face " << i << " not CCW\n";
            valid = false;
        }
    }
    
    for (size_t i = 0; i < m_hinges.size(); ++i) {
        if (!is_hinge_ccw(i)) {
            std::cerr << "[ERROR] Hinge " << i << " not CCW\n";
            valid = false;
        }
        if (!is_hinge_normal(i)) {
            std::cerr << "[ERROR] Hinge " << i << " not normal\n";
            valid = false;
        }
    }
    
    // makes no sense to test if hinges are not normal
    for (size_t i = 0; i < m_faces.size(); ++i) {
        if (!is_face_edge_order_consistent(i)) {
            std::cerr << "[ERROR] Edge order of face " << i
                      << " is not consistent with vertex order\n";
            std::cerr << "This error is meaningless if any hinge fails the normality test\n";
            valid = false;
        }
    }

    return valid;
}

// if there is any negative SFID, assigns no connectors (e.g. {-1})
// if there is the wrong number of SFIDs, assigns default connectors (e.g. {})
// if there is the right number of SFIDs, assigns those as connectors
std::vector<size_t> Sheet::get_connector_sfids(const std::vector<int>& conn_sfids) const {
    bool no_connectors =
        std::count_if(conn_sfids.begin(), conn_sfids.end(),
        [](int i) { return i < 0; }) != 0;
    if (no_connectors) { return {}; }

    std::vector<size_t> conn_sfids_cast(conn_sfids.size());
    std::transform(conn_sfids.begin(), conn_sfids.end(),
                   conn_sfids_cast.begin(),
                   [](int i) { return static_cast<size_t>(i); });

    bool default_connectors = conn_sfids_cast.size() != m_patch_count;
    std::vector<bool> patch_has_connector(m_patch_count, false);
    std::vector<size_t> final_conn_sfids(m_patch_count);

    if (default_connectors) {
        // default -> first time a patch is encountered, face is given a connector
        for (size_t sfid = 0; sfid < m_faces.size(); ++sfid) {
            auto [pid, pfid] = sfid_to_pfid(sfid);
            if (!patch_has_connector.at(pid)) {
                patch_has_connector[pid] = true;
                final_conn_sfids[pid] = sfid;
            }
        }
    } else {
        // provided -> faces appearing in provided list get a connector
        for (size_t i = 0; i < conn_sfids_cast.size(); ++i) {
            size_t sfid = conn_sfids_cast[i];
            auto [pid, pfid] = sfid_to_pfid(sfid);
            patch_has_connector[pid] = true;
            final_conn_sfids[pid] = sfid;
        }
    }

    // check every patch has a connector
    assert(std::count(patch_has_connector.begin(), patch_has_connector.end(), false) == 0);
    return final_conn_sfids;
}

void Sheet::write_faces(const std::string& file_fcs, const std::vector<int>& connector_sfids, bool split) const {
    auto fis = get_faces(connector_sfids);
    
    if (!split) {
        // clear file if it already exists
        std::ofstream file(file_fcs, std::ios::out | std::ios::trunc);
        if (!file.is_open()) {
            std::cerr << "[ERROR] Could not open " << file_fcs << '\n';
            exit(EXIT_FAILURE);
        }
        file.close();

        write_face_info(file_fcs, fis);
        return;
    }

    std::string file_noext = file_fcs.substr(0, file_fcs.find_last_of('.'));
    std::string file_ext = file_fcs.substr(file_fcs.find_last_of('.'));
    size_t num_faces = fis.size();
    assert(num_faces > 0);
    size_t num_digits = std::lround(std::floor(std::log10(num_faces)) + 1.);

    for (size_t i = 0; i < num_faces; ++i) {
        std::string file_num = std::to_string(i);
        std::string file_num_padded = std::string(num_digits - std::min(num_digits, file_num.length()), '0') + file_num;
        std::string file_name = file_noext + "_" + file_num_padded + file_ext;

        // clear file if it already exists
        std::ofstream file(file_name, std::ios::out | std::ios::trunc);
        if (!file.is_open()) {
            std::cerr << "[ERROR] Could not open " << file_name << '\n';
            exit(EXIT_FAILURE);
        }
        file.close();

        write_face_info(file_name, fis[i]);
    }
}

void Sheet::compute_patch_bbox() {
    m_patch_bbox.resize(m_patch_count);

    std::vector<double> xmin(m_patch_count, std::numeric_limits<double>::max());
    std::vector<double> xmax(m_patch_count, std::numeric_limits<double>::lowest());
    std::vector<double> ymin(m_patch_count, std::numeric_limits<double>::max());
    std::vector<double> ymax(m_patch_count, std::numeric_limits<double>::lowest());

    for (size_t vid = 0; vid < m_vertices.size(); ++vid) {
        size_t pid = m_vid_to_pid.at(vid);
        v2d vert = m_vertices[vid];
        double nx = vert[0];
        double ny = vert[1];
        if (nx < xmin[pid]) xmin[pid] = nx;
        if (ny < ymin[pid]) ymin[pid] = ny;
        if (nx > xmax[pid]) xmax[pid] = nx;
        if (ny > ymax[pid]) ymax[pid] = ny;
    }

    for (size_t pid = 0; pid < m_patch_count; ++pid) {
        m_patch_bbox[pid].m_Mins = v2d(xmin[pid], ymin[pid]);
        m_patch_bbox[pid].m_Maxs = v2d(xmax[pid], ymax[pid]);
    }
}

AAB<2, double> Sheet::get_patch_bbox(size_t pid) const { return m_patch_bbox.at(pid); }
v2d Sheet::get_patch_extent(size_t pid) const { return m_patch_bbox.at(pid).extent(); }
size_t Sheet::patch_count() const { return m_patch_count; }
size_t Sheet::hinge_count() const { return m_hinges.size(); }
std::pair<size_t, size_t> Sheet::sfid_to_pfid(size_t sfid) const { return m_sfid_to_pfid.at(sfid); }
std::vector<ExtendedVertex> Sheet::get_vertex_order(size_t pid) const { return m_vertex_order.at(pid); }
std::vector<size_t> Sheet::get_patch_hinges(size_t pid) const { return m_patch_hinges.at(pid); }

v2d Sheet::get_vertex(size_t vid, bool translate) const { 
    if (translate) {
        assert(!m_patch_bbox.empty());
        size_t pid = m_vid_to_pid.at(vid);
        return m_vertices.at(vid) - get_patch_bbox(pid).minCorner();
    }
    return m_vertices.at(vid);
}
bool Sheet::is_half_hinge(size_t hid) const {
    return m_edges.at(m_hid_to_eid.at(hid)).is_halfhinge;
}
v2d Sheet::get_hinge_parallel(size_t hid) const {
    auto [h0, h1, h2, h3] = get_hinge_vertices(hid);
    return h1 - h0;
}
v2d Sheet::get_hinge_ortho(size_t hid) const {
    auto [h0, h1, h2, h3] = get_hinge_vertices(hid);
    return h2 - h1;
}
v2d Sheet::get_hinge_center(size_t hid, bool translate) const {
    auto [h0, h1, h2, h3] = get_hinge_vertices(hid);
    v2d c = 0.25 * (h0 + h1 + h2 + h3);
    if (translate) {
        assert(!m_patch_bbox.empty());
        size_t pid = m_vid_to_pid.at(m_hinges.at(hid)[0]);
        return c - get_patch_bbox(pid).minCorner();
    }
    return c;
}
double Sheet::get_hinge_length(size_t hid) const { return length(get_hinge_parallel(hid)); }
double Sheet::get_hinge_min_length(size_t hid) const { 
    auto eid = m_hid_to_eid.at(hid);
    const auto& [tw, id, od] = get_hinge_parameters();
    if (m_edges.at(eid).is_halfhinge) { return 4.0 * tw + 2.0 * od + id; }
    return 4.0 * tw + od + id;
}

std::tuple<double, double, double> Sheet::get_hinge_parameters() const {
    return std::make_tuple(m_settings.track_width_mm,
        m_settings.inner_diameter_mm, m_settings.outer_diameter_mm);
}

// sets m_vid_to_pid, m_sfid_to_pfid, m_patch_count, m_vertex_order, m_patch_faces, m_patch_hinges
void Sheet::compute_order() {
    auto next_false = [](const std::vector<bool>& bs) {
        return std::find(bs.begin(), bs.end(), false) - bs.begin();
    };
    std::vector<bool> visited_faces(m_faces.size(), false);
    std::vector<bool> visited_vertices(m_vertices.size(), false);

    size_t start_sfid = 0;
    m_patch_count = 0;
    
    // for each patch
    while (start_sfid < visited_faces.size()) {
        std::vector<ExtendedVertex> vo;
        std::vector<size_t> fo;
        std::vector<size_t> ho;

        const auto& start_edges = m_face_to_adjacent_edges.at(start_sfid);
        auto is_hinge = [this](size_t eid) { return m_edges.at(eid).is_hinge; };
        auto first_hinge = std::find_if(start_edges.begin(), start_edges.end(), is_hinge);
        // assert(first_hinge != start_edges.end());

        size_t start_svid, start_shid;
        size_t svid, shid, sfid;
        bool start_hinge, exit_hinge;

        if (first_hinge == start_edges.end()) {
            start_svid = m_faces.at(start_sfid)[0];

            m_vid_to_pid[start_svid] = m_patch_count;
            visited_vertices[start_svid] = true;

            vo.push_back({VertexType::face, 0, start_svid, start_sfid});
            fo.push_back(start_sfid);
            
            m_sfid_to_pfid[start_sfid] = std::make_pair(m_patch_count, 0);
            visited_faces[start_sfid] = true;

            svid = get_next_vid_in_fid(start_sfid, start_svid);
            sfid = start_sfid;
            shid = 0;

            start_hinge = false;
            exit_hinge = false;
        } else {
            start_shid = m_edges.at(start_edges.at(first_hinge - start_edges.begin())).hinge_id;
            ho.push_back(start_shid);
            start_svid = get_vid_in_fid_before_hid(start_sfid, start_shid);
            m_vid_to_pid[start_svid] = m_patch_count;
            visited_vertices[start_svid] = true;

            vo.push_back({VertexType::face, start_shid, start_svid, start_sfid});
            fo.push_back(start_sfid);
            visited_faces[start_sfid] = true;
            m_sfid_to_pfid[start_sfid] = std::make_pair(m_patch_count, 0);

            svid = get_closest_hinge_vid_from_vid(start_shid, start_svid);
            sfid = start_sfid;
            shid = start_shid;

            start_hinge = true;
            exit_hinge = false;
        }

        // complete tour of patch
        size_t pfid = 0;
        while (svid != start_svid) {
            m_vid_to_pid[svid] = m_patch_count;
            visited_vertices[svid] = true;
            // push vertex depending on type
            if (start_hinge) {
                vo.push_back({VertexType::hinge, shid, svid, sfid});
            } else if (exit_hinge) {
                vo.push_back({VertexType::hinge, shid, svid, get_fid_across_hid_from_fid(sfid, shid)});
            } else {
                vo.push_back({VertexType::face, shid, svid, sfid});
            }

            if (start_hinge) {
                // get next vertex in hinge
                svid = get_next_vid_in_hid(shid, svid);
                start_hinge = false; exit_hinge = true;
            } else if (exit_hinge) {
                // get first vertex in next face and set face to visited
                sfid = get_fid_across_hid_from_fid(sfid, shid);
                svid = get_vid_in_fid_after_hid(sfid, shid);
                exit_hinge = false;

                if (!visited_faces[sfid]) {
                    visited_faces[sfid] = true;
                    fo.push_back(sfid);
                    m_sfid_to_pfid[sfid] = std::make_pair(m_patch_count, ++pfid);
                }
            } else {
                // go through face vertices until before a hinge
                int test_hid = is_face_vid_before_hinge(sfid, svid);
                if (test_hid >= 0) { // is before hinge w/ id "test_hid"
                    shid = test_hid;
                    svid = get_closest_hinge_vid_from_vid(shid, svid);
                    start_hinge = true;
                    ho.push_back(shid);
                } else {             // not before hinge
                    svid = get_next_vid_in_fid(sfid, svid);
                }
            }
        }

        m_vertex_order.push_back(vo);
        m_patch_faces.push_back(fo);
        m_patch_hinges.push_back(ho);

        start_sfid = next_false(visited_faces);
        ++m_patch_count;
    }

    assert(std::count(visited_faces.begin(), visited_faces.end(), false) == 0);
    assert(std::count(visited_vertices.begin(), visited_vertices.end(), false) == 0);
}

void Sheet::compute_bfs_depth() {
    auto next_false = [](const std::vector<bool>& bs) {
        return std::find(bs.begin(), bs.end(), false) - bs.begin();
    };

    auto unvisited_children = [this](size_t fid, const std::vector<bool>& visited) {
        vNsz adj_fids;
        for (auto eid : m_face_to_adjacent_edges.at(fid)) {
            const auto& e = m_edges.at(eid);
            if (!e.is_hinge) { continue; }
            size_t adj_fid = get_fid_across_hid_from_fid(fid, e.hinge_id);
            if (!visited.at(adj_fid)) {
                adj_fids.push_back(adj_fid);
            }
        }
        return adj_fids;
    };

    auto next_depth = [this, &unvisited_children](const std::vector<size_t>& curr_depth, const std::vector<bool>& visited) {
        std::vector<size_t> next;
        for (auto fid : curr_depth) {
            auto children = unvisited_children(fid, visited);
            next.insert(next.end(), children.begin(), children.end());
        }
        return next;
    };

    size_t start_sfid = 0;
    m_patch_count = 0;
    std::vector<bool> visited_faces(m_faces.size(), false);
    
    // for each patch
    while (start_sfid < visited_faces.size()) {
        std::vector<size_t> curr_depth{ start_sfid };
        size_t depth = 0;
        while (!curr_depth.empty()) {
            for (auto fid : curr_depth) {
                m_sfid_to_depth[fid] = depth;
                visited_faces[fid] = true;
            }
            
            curr_depth = next_depth(curr_depth, visited_faces);
            ++depth;
        }
        start_sfid = next_false(visited_faces);
        ++m_patch_count;
    }
}

std::pair<size_t, double> Sheet::half_hinges_max_shrink() const {
    double max_shrink = 0.0;
    double max_arg = 0;
    for (size_t i = 0; i < m_hinges.size(); ++i) {
        double curr = shrink_half_hinge_total(i);
        if (max_shrink < curr) {
            max_shrink = curr;
            max_arg = i;
        }
    }
    return {max_arg, max_shrink};
}

bool Sheet::faces_in_same_patch(size_t sfid0, size_t sfid1) const {
    size_t pid0 = vid_to_pid(m_faces.at(sfid0).at(0));
    size_t pid1 = vid_to_pid(m_faces.at(sfid1).at(0));
    return pid0 == pid1;
}

bool Sheet::lt_depth(size_t sfid0, size_t sfid1) const {
    return m_sfid_to_depth.at(sfid0) < m_sfid_to_depth.at(sfid1);
}

double Sheet::get_hinge_width(size_t hid) const {
    auto eid = m_hid_to_eid.at(hid);
    const auto& [tw, id, od] = get_hinge_parameters();
    if (m_edges.at(eid).is_halfhinge) { return tw + od + id; }
    return 2.0 * (tw + od) + id;
}

std::vector<v2d> Sheet::get_face_vertices(size_t i) const {
    auto face_ids = m_faces.at(i);
    std::vector<v2d> vertices(face_ids.size());
    size_t k = 0;
    for (size_t id : face_ids) {
        vertices[k++] = m_vertices.at(id);
    }
    return vertices;
}

std::array<v2d, 4> Sheet::get_hinge_vertices(size_t i) const {
    auto hinge_ids = m_hinges.at(i);
    std::array<v2d, 4> vertices;
    size_t k = 0;
    for (size_t id : hinge_ids) {
        vertices[k++] = m_vertices.at(id);
    }
    return vertices;
}

// https://en.wikipedia.org/wiki/Curve_orientation
bool is_simple_polygon_ccw(const std::vector<v2d>& pts) {
    auto positive_modulo = [](int i, int n) { return (i % n + n) % n; };
    size_t i = std::min_element(pts.begin(), pts.end()) - pts.begin();
    size_t n = (i + 1) % pts.size();
    size_t p = positive_modulo(static_cast<int>(i) - 1, pts.size());

    auto det = [](v2d a, v2d b, v2d c) {
        return (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
    };
    return det(pts[p], pts[i], pts[n]) > 0;
}
template<size_t N>
bool is_simple_polygon_ccw(const std::array<v2d, N>& pts) {
    auto positive_modulo = [](int i, int n) { return (i % n + n) % n; };
    size_t i = std::min_element(pts.begin(), pts.end()) - pts.begin();
    size_t n = (i + 1) % pts.size();
    size_t p = positive_modulo(static_cast<int>(i) - 1, pts.size());

    auto det = [](v2d a, v2d b, v2d c) {
        return (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
    };
    return det(pts[p], pts[i], pts[n]) > 0;
}

bool Sheet::is_face_ccw(size_t i) const {
    auto face_vertices = get_face_vertices(i);
    return is_simple_polygon_ccw(face_vertices);
}

bool Sheet::is_hinge_ccw(size_t i) const {
    auto hinge_vertices = get_hinge_vertices(i);
    return is_simple_polygon_ccw(hinge_vertices);
}

bool Sheet::is_hinge_normal(size_t i) const {
    auto equality_double = [](double a, double b, double eps) {
        if (std::abs(a - b) < eps) { return true; }
        else { return std::abs(a - b) < eps * std::max(std::abs(a), std::abs(b)); }
    };
    const double eps = m_settings.float_eq_eps;

    auto hinge_vertices = get_hinge_vertices(i);
    double width1 = distance(hinge_vertices[1], hinge_vertices[2]);
    double width2 = distance(hinge_vertices[3], hinge_vertices[0]);
    return equality_double(width1, get_hinge_width(i), eps) &&
           equality_double(width2, get_hinge_width(i), eps);
}

bool Sheet::is_face_edge_order_consistent(size_t i) const {
    auto equality_double = [](double a, double b, double eps) {
        if (std::abs(a - b) < eps) { return true; }
        else { return std::abs(a - b) < eps * std::max(std::abs(a), std::abs(b)); }
    };
    const double eps = m_settings.float_eq_eps;

    auto face_vertices = get_face_vertices(i);

    size_t k = 0;
    for (size_t eid : m_face_to_adjacent_edges.at(i)) {
        size_t n = (k + 1) % face_vertices.size();
        v2d face_edge = normalize(face_vertices[n] - face_vertices[k]);

        Edge e = m_edges.at(eid);
        if (e.is_hinge) {
            auto [h0, h1, h2, h3] = get_hinge_vertices(e.hinge_id);
            v2d hinge_parallel = normalize(h1 - h0);
            
            if (!equality_double(std::abs(dot(face_edge, hinge_parallel)), 1.0, eps)) {
                return false;
            }
        }
        ++k;
    }
    return true;
}

double Sheet::shrink_vertex(v2d vt0, v2d vt1, v2d vt_prev, v2d ht, double inset, double offset) const {
    double theta = std::acos(dot(normalize(vt1 - vt0), normalize(vt_prev - vt0)));
    // positive if ht in [vt0, vt1], else negative 
    double sign = (dot(ht - vt0, vt1 - vt0) > 0.0) ? 1.0 : -1.0;
    double d_shrink = inset / std::tan(theta) - sign * distance(vt0, ht);
    
    return std::max(0.0, d_shrink - offset);
    // return std::max(0.0, d_shrink + offset);

    // v2d nht = ht + offset * normalize(vt1 - vt0);
    // double theta = std::acos(dot(normalize(vt1 - vt0), normalize(vt_prev - vt0)));
    // // positive if ht in [vt0, vt1], else negative 
    // double sign = (dot(nht - vt0, vt1 - vt0) > 0.0) ? 1.0 : -1.0;
    // double d_shrink = inset / std::tan(theta) - sign * distance(vt0, nht);

    return std::max(0.0, d_shrink);
}

std::tuple<v2d, v2d, v2d, v2d, v2d, v2d, v2d, v2d, v2d, v2d, v2d, v2d>
Sheet::shrink_points(size_t fid0, size_t fid1, size_t hid) const {
    auto [efid0, efid1] = m_hinge_to_adjacent_faces.at(hid);
    assert((fid0 == efid0 && fid1 == efid1) || (fid1 == efid0 && fid0 == efid1));

    // Face 0, ccw dir: f0_v0_prev -> f0_v0 -> [f0_ht0] -> f0_v1 (-> f0_v1_next)
    size_t f0_vid0 = get_vid_in_fid_before_hid(fid0, hid);
    v2d f0_v0 = get_vertex(f0_vid0);
    v2d f0_v0_prev = get_vertex(get_prev_vid_in_fid(fid0, f0_vid0));
    v2d f0_ht0 = get_vertex(get_closest_hinge_vid_from_vid(hid, f0_vid0));
    // Face 0, cw dir:  f0_v1_next -> f0_v1 -> [f0_ht1] -> f0_v0 (-> f0_v0_prev)
    size_t f0_vid1 = get_vid_in_fid_after_hid(fid0, hid);
    v2d f0_v1 = get_vertex(f0_vid1);
    v2d f0_v1_next = get_vertex(get_next_vid_in_fid(fid0, f0_vid1));
    v2d f0_ht1 = get_vertex(get_closest_hinge_vid_from_vid(hid, f0_vid1));
    // Face 1, ccw dir: f1_v0_prev -> f1_v0 -> [f1_ht0] -> f1_v1 (-> f1_v1_next)
    size_t f1_vid0 = get_vid_in_fid_before_hid(fid1, hid);
    v2d f1_v0 = get_vertex(f1_vid0);
    v2d f1_v0_prev = get_vertex(get_prev_vid_in_fid(fid1, f1_vid0));
    v2d f1_ht0 = get_vertex(get_closest_hinge_vid_from_vid(hid, f1_vid0));
    // Face 1, cw dir  f1_v1_next -> f1_v1 -> [f1_ht1] -> f1_v0 (-> f1_v0_prev)
    size_t f1_vid1 = get_vid_in_fid_after_hid(fid1, hid);
    v2d f1_v1 = get_vertex(f1_vid1);
    v2d f1_v1_next = get_vertex(get_next_vid_in_fid(fid1, f1_vid1));
    v2d f1_ht1 = get_vertex(get_closest_hinge_vid_from_vid(hid, f1_vid1));

    return {f0_v0, f0_v1, f1_v0, f1_v1,
            f0_ht0, f0_ht1, f1_ht0, f1_ht1,
            f0_v0_prev, f0_v1_next, f1_v0_prev, f1_v1_next};
}

std::pair<double, double> Sheet::shrink_compute(
    size_t fid0, size_t fid1, size_t hid,
    v2d f0_v0, v2d f0_v1, v2d f1_v0, v2d f1_v1,
    v2d f0_ht0, v2d f0_ht1, v2d f1_ht0, v2d f1_ht1,
    v2d f0_v0_prev, v2d f0_v1_next, v2d f1_v0_prev, v2d f1_v1_next) const
{  
    const auto& [tw, id, od] = get_hinge_parameters();
    double data_inset_depth_wide_mm = m_settings.data_inset_depth_wide_mm;
    double trace_width_mm = m_settings.trace_width_mm;
    double trace_clearance_mm = m_settings.trace_clearance_mm; 

    // double offset_data_wide = 0.5 * m_outer_diameter + EDGE_CLEARANCE + 0.5 * TRACE_WIDTH;
    double offset_data_wide = 0.5 * od;
    double inset = data_inset_depth_wide_mm + 0.5 * trace_width_mm + trace_clearance_mm;
    double sr = 0.0; double sl = 0.0;
    if (lt_depth(fid0, fid1)) {
        // fid0 NARROW, fid1 WIDE
        sr = shrink_vertex(f1_v1, f1_v0, f1_v1_next, f1_ht1, inset, offset_data_wide);
        sl = shrink_vertex(f1_v0, f1_v1, f1_v0_prev, f1_ht0, inset, offset_data_wide);
    } else {
        // fid0 WIDE, fid1 NARROW
        sr = shrink_vertex(f0_v0, f0_v1, f0_v0_prev, f0_ht0, inset, offset_data_wide);
        sl = shrink_vertex(f0_v1, f0_v0, f0_v1_next, f0_ht1, inset, offset_data_wide);
    }

    v2d f0_ht0_new = f0_ht0 + sr * normalize(f0_v1 - f0_v0);
    v2d f0_ht1_new = f0_ht1 + sl * normalize(f0_v0 - f0_v1);
    // v2d f1_ht0_new = f1_ht0 + shrink_left * normalize(f1_v1 - f1_v0);
    // v2d f1_ht1_new = f1_ht1 + shrink_right * normalize(f1_v0 - f1_v1);

    assert(sr + sl >= 0.0);
    assert(distance(f0_ht0_new, f0_ht1_new) >= get_hinge_min_length(hid)); 
    return {sl, sr};
}

v2d Sheet::shrink_half_hinge_center(size_t hid) const {
    size_t eid = m_hid_to_eid.at(hid);
    if (!m_edges.at(eid).is_halfhinge) { 
        return get_hinge_center(hid);
    }
    
    auto [fid0, fid1] = m_hinge_to_adjacent_faces.at(hid);
    auto [f0_v0, f0_v1, f1_v0, f1_v1,
          f0_ht0, f0_ht1, f1_ht0, f1_ht1,
          f0_v0_prev, f0_v1_next, f1_v0_prev, f1_v1_next] = shrink_points(fid0, fid1, hid);

    auto [sl, sr] = shrink_compute(fid0, fid1, hid,
        f0_v0, f0_v1, f1_v0, f1_v1,
        f0_ht0, f0_ht1, f1_ht0, f1_ht1,
        f0_v0_prev, f0_v1_next, f1_v0_prev, f1_v1_next);

    v2d f0_ht0_new = f0_ht0 + sr * normalize(f0_v1 - f0_v0);
    v2d f0_ht1_new = f0_ht1 + sl * normalize(f0_v0 - f0_v1);
    v2d f1_ht0_new = f1_ht0 + sl * normalize(f1_v1 - f1_v0);
    v2d f1_ht1_new = f1_ht1 + sr * normalize(f1_v0 - f1_v1);
    
    return 0.25 * (f0_ht0_new + f0_ht1_new + f1_ht0_new + f1_ht1_new);
}

// NEEDS TO TAKE HALF_HINGE ORIENTATION TO KNOW WHERE IS THE RIGHT AND LEFT
std::pair<double, double> Sheet::shrink_half_hinge(size_t hid, size_t fid0, size_t fid1) const {    
    size_t eid = m_hid_to_eid.at(hid);
    if (!m_edges.at(eid).is_halfhinge) { return {0.0, 0.0}; }
        
    auto [f0_v0, f0_v1, f1_v0, f1_v1,
          f0_ht0, f0_ht1, f1_ht0, f1_ht1,
          f0_v0_prev, f0_v1_next, f1_v0_prev, f1_v1_next] = shrink_points(fid0, fid1, hid);
    
    return shrink_compute(fid0, fid1, hid,
        f0_v0, f0_v1, f1_v0, f1_v1,
        f0_ht0, f0_ht1, f1_ht0, f1_ht1,
        f0_v0_prev, f0_v1_next, f1_v0_prev, f1_v1_next);
}

double Sheet::shrink_half_hinge_total(size_t hid) const {
    auto [efid0, efid1] = m_hinge_to_adjacent_faces.at(hid);
    auto [shrink_left, shrink_right] = shrink_half_hinge(hid, efid0, efid1);
    return shrink_left + shrink_right;
}


// [UNUSED]
// std::vector<size_t> Sheet::get_adjacent_face_ids(size_t face_id) const {
//     auto adj_edges = m_face_to_adjacent_edges.at(face_id);
//     std::vector<size_t> adj_faces;
//     for (auto eid : adj_edges) {
//         auto edge = m_edges.at(eid);
//         if (edge.is_hinge) {
//             adj_faces.push_back((face_id == edge.adjacent_faces[0]) ?
//                                     edge.adjacent_faces[1] :
//                                     edge.adjacent_faces[0]);
//         }
//     }
//     return adj_faces;
// }

int Sheet::get_edge_id_between_faces(size_t face_id0, size_t face_id1) const {
    auto adj_edges0 = m_face_to_adjacent_edges.at(face_id0);
    auto adj_edges1 = m_face_to_adjacent_edges.at(face_id1);
    std::sort(adj_edges0.begin(), adj_edges0.end());
    std::sort(adj_edges1.begin(), adj_edges1.end());
    std::vector<size_t> inter;
    std::set_intersection(adj_edges0.begin(), adj_edges0.end(),
                          adj_edges1.begin(), adj_edges1.end(),
                          std::back_inserter(inter));
    
    if (inter.size() == 0) { return -1; }
    else if (inter.size() == 1) { return inter[0]; }
    else {
        std::cerr << "[ERROR] Adjacent faces share multiple edges\n";
        exit(EXIT_FAILURE);
    }
}

// run after init_sheet
FaceInfo Sheet::get_face_info(size_t face_id) const {
    // std::cout << "F " << face_id << '\n';
    double tkw = m_settings.track_width_mm;
    double od = m_settings.outer_diameter_mm;
    double tcw = m_settings.trace_width_mm;
    double ec = m_settings.edge_clearance_mm;

    // position of IO relative to track_width, based on data_pos in layout2svg
    double data_pos = (tkw - (0.5 * tcw + ec)) / tkw;
    double offset_io = data_pos * tkw;
    
    // enough space to avoid issues with GND plane round caps
    double inset_io = m_settings.data_inset_depth_mm;
    double inset_io_wide = m_settings.data_inset_depth_wide_mm;

    FaceInfo fi;
    fi.fid = face_id;
    const auto& face = get_face_vertices(face_id);

    size_t pid = sfid_to_pfid(face_id).first;
    v2d min_corner = get_patch_bbox(pid).minCorner();
    double ext_y = get_patch_extent(pid)[1];

    size_t k = 0;
    bool once_wide = false;
    for (size_t eid : m_face_to_adjacent_edges.at(face_id)) {
        fi.vertices.push_back(face[k]);
        v2d ctrd = face[k] - min_corner;
        fi.vertices_svg_pos.push_back(v2d(ctrd[0], ext_y - ctrd[1]));
        // std::cout << "\tV " << face[k] << '\n';

        size_t n = (k + 1) % face.size();
        v2d face_edge = face[n] - face[k];
        v2d face_edge_ortho = v2d(-face_edge[1], face_edge[0]);

        Edge e = m_edges.at(eid);
        fi.is_hinge_edge.push_back(e.is_hinge);
        fi.is_half_hinge_edge.push_back(e.is_halfhinge);
        fi.is_boundary_edge.push_back(e.is_boundary);
        fi.hinge_offsets.push_back(e.offset);
        fi.is_detour.push_back(false);
        
        if (e.is_hinge) {
            auto hinge = get_hinge_vertices(e.hinge_id);
            auto [efid0, efid1] = m_hinge_to_adjacent_faces.at(e.hinge_id);
            size_t other_fid = (face_id != efid0) ? efid0 : efid1;
            auto [shrink_left, shrink_right] = shrink_half_hinge(e.hinge_id, face_id, other_fid);
            
            double hl = get_hinge_length(e.hinge_id) - (shrink_left + shrink_right);
            double offset_io_wide = 0.5 * hl - 0.5 * od - tkw + data_pos * tkw;

            // hinge orthogonal edge pointing towards the face
            v2d hinge_ortho = hinge[2] - hinge[1];
            if (dot(face_edge_ortho, hinge_ortho) < 0.0) { hinge_ortho = -hinge_ortho; }
            // hinge parallel edge pointing in same direction as face edge
            v2d hinge_parallel = hinge[1] - hinge[0];
            if (dot(face_edge, hinge_parallel) < 0.0) { hinge_parallel = -hinge_parallel; }

            v2d hinge_center = v2d(0.0);
            for (auto v : hinge) { hinge_center += v; }
            // double delta_shrink = shrink_right - shrink_left;
            // change input/output for wide half of half-hinges
            if (e.is_halfhinge) {
                // size_t other_fid = get_fid_across_hid_from_fid(face_id, e.hinge_id);
                assert(faces_in_same_patch(face_id, other_fid));
                if (!lt_depth(face_id, other_fid)) {
                    fi.is_deeper_than_neighbor.push_back(true);
                    assert(!once_wide);
                    once_wide = true;
                    // std::cout << "\tWIDE\n";
                    inset_io = m_settings.data_inset_depth_wide_mm;
                    fi.is_narrow.push_back({false, false});
                } else {
                    // std::cout << "\tNARROW\n";
                    inset_io = m_settings.data_inset_depth_mm;
                    fi.is_narrow.push_back({true, true});
                }
            } else {
                fi.is_narrow.push_back({true, true});
            }
            v2d hinge_edge_center = shrink_half_hinge_center(e.hinge_id)
                + 0.5 * get_hinge_width(e.hinge_id) * normalize(hinge_ortho);

            auto io = std::make_pair(
                hinge_edge_center - offset_io * normalize(hinge_parallel)
                                  + inset_io * normalize(hinge_ortho),
                hinge_edge_center + offset_io * normalize(hinge_parallel)
                                  + inset_io * normalize(hinge_ortho));
            fi.hinge_ios.push_back(io);
            auto io_wide = std::make_pair(
                hinge_edge_center - offset_io_wide * normalize(hinge_parallel)
                                  + inset_io_wide * normalize(hinge_ortho),
                hinge_edge_center + offset_io_wide * normalize(hinge_parallel)
                                  + inset_io_wide * normalize(hinge_ortho));
            fi.hinge_ios_wide.push_back(io_wide);
            // std::cout << "\t\tO " << io.first << '\n';
            // std::cout << "\t\tI " << io.second << '\n';
        } else {
            fi.hinge_ios.push_back(std::make_pair(v2d(0.0), v2d(0.0)));
            fi.hinge_ios_wide.push_back(std::make_pair(v2d(0.0), v2d(0.0)));
            fi.is_narrow.push_back({false, false});
        }
        ++k;
        if (k > fi.is_deeper_than_neighbor.size()) {
            fi.is_deeper_than_neighbor.push_back(false);
        }
    }

    auto [vmin, vmax] = face_bbox(fi.vertices);
    fi.min_corner = vmin;
    fi.max_corner = vmax;

    fi.run_pnr = true;
    fi.run_verif = false;
    fi.failed_pnr = false;
    fi.failed_verif = false;
    fi.pnr_runs_same_LEDs = 0;
    fi.stats_overall = std::vector<size_t>(3, 0);

    // std::cout << '\n';
    return fi;
}

std::vector<FaceInfo> Sheet::get_faces(const std::vector<int>& connector_sfids) const {
    std::vector<FaceInfo> fis(m_faces.size());
    for (size_t i = 0; i < fis.size(); ++i) {
        fis[i] = get_face_info(i);
    }

    std::vector<size_t> conn_sfids = get_connector_sfids(connector_sfids);
    for (auto sfid : conn_sfids) {
        fis[sfid].has_connector = true;
    }

    return fis;
}

bool Sheet::is_half_hinge(size_t fid, size_t i) const {
    auto eid = m_face_to_adjacent_edges.at(fid).at(i);
    return m_edges.at(eid).is_halfhinge;
}
bool Sheet::is_wide(size_t fid, size_t i) const {
    auto eid = m_face_to_adjacent_edges.at(fid).at(i);
    auto fids = m_hinge_to_adjacent_faces.at(m_edges.at(eid).hinge_id);
    size_t other_fid = (fid != fids[0]) ? fids[0] : fids[1];
    return !lt_depth(fid, other_fid);
}
int Sheet::get_hid(size_t fid, size_t i) const {
    return m_edges.at(m_face_to_adjacent_edges.at(fid).at(i)).hinge_id;
}

std::vector<size_t> Sheet::get_edges(size_t face_id) {
    return m_face_to_adjacent_edges.at(face_id);
}

size_t Sheet::get_vid_in_fid_before_hid(size_t fid, size_t hid) const {
    auto eids = m_face_to_adjacent_edges.at(fid);

    size_t i = 0;
    while (m_edges.at(eids[i]).hinge_id != static_cast<int>(hid)) { ++i; }
    return m_faces.at(fid)[i];
}
size_t Sheet::get_next_vid_in_hid(size_t hid, size_t vid) const {
    auto vids = m_hinges.at(hid);

    size_t i = 0;
    while (vids[i] != vid) { ++i; }
    return vids[(i + 1) % vids.size()];
}
size_t Sheet::get_fid_across_hid_from_fid(size_t fid, size_t hid) const {
    auto eids = m_face_to_adjacent_edges.at(fid);

    size_t i = 0;
    while (m_edges.at(eids[i]).hinge_id != static_cast<int>(hid)) { ++i; }

    auto fids = m_hinge_to_adjacent_faces.at(m_edges.at(eids[i]).hinge_id);
    if (fids[0] == fid) { return fids[1]; }
    return fids[0];
}
size_t Sheet::get_vid_in_fid_after_hid(size_t fid, size_t hid) const {
    auto eids = m_face_to_adjacent_edges.at(fid);

    size_t i = 0;
    while (m_edges.at(eids[i]).hinge_id != static_cast<int>(hid)) { ++i; }
    return m_faces.at(fid)[(i + 1) % eids.size()];
} 
int Sheet::is_face_vid_before_hinge(size_t fid, size_t vid) const {
    auto vids = m_faces.at(fid);

    size_t i = 0;
    while (vids[i] != vid) { ++i; }
    
    auto edge = m_edges.at(m_face_to_adjacent_edges.at(fid)[i]);
    if (edge.is_hinge) { return edge.hinge_id; }
    return -1;
}
size_t Sheet::get_closest_hinge_vid_from_vid(size_t hid, size_t vid) const {
    v2d vert = m_vertices.at(vid);
    auto hvids = m_hinges.at(hid);

    double dmin = std::numeric_limits<double>::max();
    size_t hmin = 0;
    for (size_t i = 0; i < 4; ++i) {
        v2d h = m_vertices.at(hvids[i]);
        double d = sqDistance(vert, h);
        if (d < dmin) {
            dmin = d;
            hmin = hvids[i];
        }
    }

    return hmin;
}
size_t Sheet::get_next_vid_in_fid(size_t fid, size_t vid) const {
    auto vids = m_faces.at(fid);

    size_t i = 0;
    while (vids[i] != vid) { ++i; }
    return vids[(i + 1) % m_faces.at(fid).size()];
}
size_t Sheet::get_prev_vid_in_fid(size_t fid, size_t vid) const {
    auto rem = [](int a, int b) {
        return (a % b + b) % b;
    };

    auto vids = m_faces.at(fid);

    size_t i = 0;
    while (vids[i] != vid) { ++i; }
    return vids[rem(static_cast<int>(i) - 1, m_faces.at(fid).size())];
}

size_t Sheet::vid_to_pid(size_t vid) const {
    size_t pid = 0;
    for (; pid < m_patch_count; ++pid) {
        if (std::find(m_patch_faces.at(pid).begin(), m_patch_faces.at(pid).end(), vid) != m_patch_faces.at(pid).end()) {
            break;
        }
        if (std::find(m_patch_hinges.at(pid).begin(), m_patch_hinges.at(pid).end(), vid) != m_patch_faces.at(pid).end()) {
            break;
        }
    }
    return pid;
}
