#pragma once

#include <svgwriter.hpp>

// #include <global_parameters.hpp>
#include <Settings.hpp>
#include <Sheet.hpp>
#include <ModuleInfo.hpp>

#include <algorithm>
#include <array>
#include <string>
#include <vector>

enum class Layer {
    EdgeCuts,
    FCu,
    FMask,
    FPaste,
    FSilkscreen,
    BCu,
    BMask
};
// hack to be able to iterate over layers
constexpr std::array AllLayers{ Layer::EdgeCuts, Layer::FCu, Layer::FMask, Layer::FPaste, Layer::FSilkscreen, Layer::BCu, Layer::BMask }; 
size_t get_layer_id(Layer layer);
Layer get_layer_from_id(size_t id);
std::string get_layer_name(Layer layer);

class Layout {
public:
    Layout() = delete;
    Layout(const std::string& file_stg, const std::string& file_sht, const std::string& file_fis) {
        m_settings = read_settings(file_stg);
        
        m_sheet.read_sheet(file_sht);
        m_sheet.init_sheet();

        m_module_info = parse_module_info(m_settings.file_module);
        m_connector_info = parse_module_info(m_settings.file_connector);
        
        parse_face_infos(file_fis);
        m_patch_layouts.resize(m_sheet.patch_count());
    }

    void parse_face_infos(const std::string& file_fis) {
        auto fis = parse_face_info(file_fis); 
        std::vector<std::vector<std::pair<size_t, size_t>>> permutation(m_sheet.patch_count(), std::vector<std::pair<size_t, size_t>>());        
        for (size_t fid = 0; fid < fis.size(); ++fid) {
            size_t sfid = fis.at(fid).fid;
            auto [pid, pfid] = m_sheet.sfid_to_pfid(sfid);
            // std::cout << sfid << "\t->\t" << pid << ", " << pfid << '\n';
            permutation[pid].push_back(std::make_pair(fid, pfid));
            // m_patch_fis[pid].push_back(fis.at(sfid));
        }

        auto cmp_pfid = [](const std::pair<size_t, size_t>& a, const std::pair<size_t, size_t>& b) {
            return a.second < b.second;
        };
        for (auto& p : permutation) { std::sort(p.begin(), p.end(), cmp_pfid); }

        m_patch_fis.resize(m_sheet.patch_count());
        for (size_t pid = 0; pid < m_sheet.patch_count(); ++pid) {
            // std::cout << "PID\t" << pid << '\n'; 
            for (auto [fid, pfid] : permutation[pid]) {
                // std::cout << "SFID, PFID\t" << sfid << ", " << pfid << '\n';
                m_patch_fis[pid].push_back(fis.at(fid));
            }
        }
    }

    void set_failed_fids(const std::vector<size_t>& ffids) {
        m_halfhinge_center_fids = ffids;
    }

    void write_layout(const std::string& name) const;
    void write_layout_patch(const std::string& name, size_t pid) const;
    void write_layer_patch(const std::string& name, size_t pid, Layer layer) const;

    void write_auxiliary_files(const std::string& name);
    void write_pos_file(const std::string& name, size_t pid) const;
    void write_drl_file(const std::string& name, size_t pid) const;
    void write_face_file(const std::string& name, size_t pid) const;
    void write_rects_file(const std::string& name, size_t pid) const;

    // compute patch layers
    void compute_layout();
    void compute_layout_patch(size_t pid);
    void compute_layer(size_t pid, Layer layer);

    void compute_EdgeCuts(size_t pid);
    void compute_FCu(size_t pid);
    void compute_FMask(size_t pid);
    void compute_FPaste(size_t pid);
    void compute_FSilkscreen(size_t pid);
    void compute_BCu(size_t pid);
    void compute_BMask(size_t pid);

    std::vector<Component> compute_pos(const std::string& name, size_t pid) const;
    std::vector<Via> compute_drl(const std::string& name, size_t pid) const;

    size_t get_num_modules(size_t sfid) const {
        auto [pid, pfid] = m_sheet.sfid_to_pfid(sfid);
        const auto& fi = m_patch_fis.at(pid).at(pfid);
        return fi.modules.size() + (fi.has_connector ? 1 : 0);
    }
    v2d to_svg_coords(size_t pid, v2d pt, bool already_centered = false) const {
        double ext_y = m_sheet.get_patch_extent(pid)[1];
        auto tr_pt = !already_centered ? center_vertex(pt, pid) : pt;
        return v2d(tr_pt[0], ext_y - tr_pt[1]);
    }

    void print_info() const {
        size_t num_leds_total = 0;
        size_t num_leds_patch = 0;
        for (size_t pid = 0; pid < m_patch_fis.size(); ++pid) {
            const auto& fis = m_patch_fis.at(pid);
            num_leds_patch = 0;
            for (const auto& fi : fis) {
                num_leds_patch += fi.modules.size();
            }
            num_leds_total += num_leds_patch;
            std::cout << "P" << pid << '\t' << "LEDs " << num_leds_patch << '\n';
        }
        std::cout << "Total LEDs " << num_leds_total << '\n';
    }

private:
    v2d center_vertex(v2d pt, size_t pid) const { return pt - m_sheet.get_patch_bbox(pid).minCorner(); }
    std::pair<double, double> center_vertex(std::pair<double, double> pt, size_t pid) const {
        auto [x, y] = pt;
        v2d v(x, y);
        v2d c = center_vertex(v, pid);
        return std::make_pair(c[0], c[1]);
    }

    std::vector<v2d> center_v2ds(const std::vector<v2d>& vs, size_t pid) const {
        std::vector<v2d> result(vs.size());
        auto center_vertex_unary = [&pid, this](v2d pt){ return center_vertex(pt, pid); };
        std::transform(vs.begin(), vs.end(), result.begin(), center_vertex_unary);
        return result;
    }
    std::vector<std::vector<v2d>> center_vv2ds(const std::vector<std::vector<v2d>>& vvs, size_t pid) const {
        std::vector<std::vector<v2d>> result(vvs.size());
        auto center_v2ds_unary = [&pid, this](std::vector<v2d> pts){ return center_v2ds(pts, pid); };
        std::transform(vvs.begin(), vvs.end(), result.begin(), center_v2ds_unary);
        return result;
    }

    Segment center_segment(const Segment& s, size_t pid) const {
        return std::make_pair(center_vertex(s.first, pid), center_vertex(s.second, pid));
    }

    std::tuple<v2d, v2d, v2d> center_module(const std::tuple<v2d, v2d, v2d>& mod, size_t pid) const {
        return std::make_tuple(center_vertex(std::get<0>(mod), pid),
                               center_vertex(std::get<1>(mod), pid),
                               center_vertex(std::get<2>(mod), pid));
    }

    // full hinge methods
    std::string half_fullhinge_to_svgpath_commands(const ExtendedVertex& ev0, const ExtendedVertex& ev1) const;
    SVGPath fullhinge_hole_to_svgpath(const ExtendedVertex& ev0, const ExtendedVertex& ev1) const;
    // half hinge methods
    std::string half_halfhinge_to_svgpath_commands(const ExtendedVertex& ev0, const ExtendedVertex& ev1) const;
    SVGPath halfhinge_hole_to_svgpath(const ExtendedVertex& ev0, const ExtendedVertex& ev1) const;

    SVGPath contour_to_svgpath(size_t pid) const;
    
    std::vector<v2d> half_fullhinge_trace_pts(const ExtendedVertex& ev0, const ExtendedVertex& ev1, double t, double inset) const;
    std::vector<std::vector<v2d>> half_fullhinge_trace_to_svgpath(const ExtendedVertex& ev0, const ExtendedVertex& ev1, double t, double inset, double width) const;
    std::vector<v2d> half_halfhinge_trace_pts(const ExtendedVertex& ev0, const ExtendedVertex& ev1, double t, double inset) const;
    std::vector<std::vector<v2d>> half_halfhinge_trace_to_svgpath(const ExtendedVertex& ev0, const ExtendedVertex& ev1, double t, double inset, double width) const;

    std::vector<std::vector<v2d>> vcc_plane(size_t pid) const;
    // std::vector<std::vector<v2d>> gnd_plane(size_t pid, const std::vector<std::vector<v2d>>& traces) const;

    // std::pair<std::vector<std::vector<v2d>>, std::vector<std::vector<v2d>>>
    // gnd_plane_fid(size_t fid, bool& is_valid) const;

    Segment get_hio(size_t fid, size_t i) const {
        auto [pid, pfid] = m_sheet.sfid_to_pfid(fid);
        // not half-hinge, or narrow part of half-hinge
        if (!m_sheet.is_half_hinge(fid, i) || !m_sheet.is_wide(fid, i)) {
            return m_patch_fis.at(pid).at(pfid).hinge_ios[i];
        }
        // wide part of half-hinge, but not failed
        if (std::find(m_halfhinge_center_fids.begin(), m_halfhinge_center_fids.end(), 
                      fid) == m_halfhinge_center_fids.end()) {
            return m_patch_fis.at(pid).at(pfid).hinge_ios_wide[i];
        }
        // wide part of half-hinge, failed so centered IOs
        return m_patch_fis.at(pid).at(pfid).hinge_ios[i];
    }

    // data members
    // std::string m_file_sheet;
    Sheet m_sheet;
    // std::string m_file_fis;
    std::vector<std::vector<FaceInfo>> m_patch_fis;
    
    // std::string m_file_settings;
    Settings m_settings;

    // std::string m_file_module;
    ModuleInfo m_module_info;
    // std::string m_file_connector;
    ModuleInfo m_connector_info;
    
    std::vector<size_t> m_halfhinge_center_fids;

    // for each patch, stores a vector of SVGPaths per layer
    std::vector<std::array<std::vector<SVGPath>, AllLayers.size()>> m_patch_layouts; 
};