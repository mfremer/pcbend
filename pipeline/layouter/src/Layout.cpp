#include "Layout.hpp"

// #include <global_parameters.hpp>
#include <layout_helpers.hpp>

#include <forward_list>
#include <iomanip>
#include <numeric>

#include <clipper.hpp>

#include <LibSL/LibSL.h>

size_t get_layer_id(Layer layer) { return std::find(AllLayers.begin(), AllLayers.end(), layer) - AllLayers.begin(); }
Layer get_layer_from_id(size_t id) { return AllLayers.at(id); }
std::string get_layer_name(Layer layer) {
    switch (layer) {
        case Layer::EdgeCuts:    return "EdgeCuts"; break;
        case Layer::FCu:         return "FCu"; break;
        case Layer::FMask:       return "FMask"; break;
        case Layer::FPaste:      return "FPaste"; break;
        case Layer::FSilkscreen: return "FSilkscreen"; break;
        case Layer::BCu:         return "BCu"; break;
        case Layer::BMask:       return "BMask"; break;
        default:                 return "ERROR"; break;
    }
}

std::vector<std::vector<v2d>> generate_pad_paths(
    const Settings& stgs,
    const Pad& pad, const std::tuple<double, double, double>& tr)
{
    using namespace ClipperLib;
    double offset = pad.r * std::min(pad.w, pad.h);

    std::vector<v2d> pts({
        apply_transform_mod(tr, v2d(-0.5 * (pad.w - 2.0 * offset), -0.5 * (pad.h - 2.0 * offset))),
        apply_transform_mod(tr, v2d(+0.5 * (pad.w - 2.0 * offset), -0.5 * (pad.h - 2.0 * offset))),
        apply_transform_mod(tr, v2d(+0.5 * (pad.w - 2.0 * offset), +0.5 * (pad.h - 2.0 * offset))),
        apply_transform_mod(tr, v2d(-0.5 * (pad.w - 2.0 * offset), +0.5 * (pad.h - 2.0 * offset)))
    });

    if (pad.r == 0.0) { return std::vector<std::vector<v2d>>({pts}); }
    
    ClipperOffset co_pad; Paths ps_pad;
    co_pad.ArcTolerance = stgs.clipper_arc_tolerance;
    co_pad.AddPath(v2ds_to_path(stgs, pts), JoinType::jtRound, EndType::etClosedPolygon);
    co_pad.Execute(ps_pad, stgs.clipper_scaling * offset);
    
    return paths_to_vv2ds(stgs, ps_pad);
}

void Layout::write_layout(const std::string& name) const {
    for (size_t pid = 0; pid < m_sheet.patch_count(); ++pid) {
        std::string patch_extension = "_p" + std::to_string(pid);
        write_layout_patch(name + patch_extension, pid);
    }
}

void Layout::write_layout_patch(const std::string& name, size_t pid) const {
    std::cout << "Dimensions (in mm) of patch " << pid << ":\t" << m_sheet.get_patch_extent(pid) << '\n';
    for (size_t lid = 0; lid < AllLayers.size(); ++lid) {
        Layer layer = get_layer_from_id(lid);
        write_layer_patch(name, pid, layer);
    }
}

void Layout::write_layer_patch(const std::string& name, size_t pid, Layer layer) const {
    const auto& patch_layer = m_patch_layouts.at(pid).at(get_layer_id(layer));
    v2d extent = m_sheet.get_patch_extent(pid);

    SVGFile svg(name + "_" + get_layer_name(layer) + ".svg", extent[0], extent[1], true);
    for (const auto& elem : patch_layer) {
        svg.add_geometry_element(elem);
    }
    svg.to_file();
}

void Layout::write_pos_file(const std::string& name, size_t pid) const {
// void write_pos_file(const std::string& filepath, std::vector<Component>& comps)
    auto comps = compute_pos(name, pid);

    auto cmp_pads = [](const Component& lhs, const Component& rhs) { return lhs.pads.size() < rhs.pads.size(); };
    std::sort(comps.begin(), comps.end(), cmp_pads);

    std::string file_name = name + "_p" + std::to_string(pid) + ".pos";
    std::ofstream ofile(file_name);
    if (!ofile.is_open()) {
        std::cerr << "[ERROR] Could not open file at " << file_name << '\n';
        exit(EXIT_FAILURE);
    }
    ofile << std::setprecision(4) << std::fixed;
    ofile << "### Module positions\n";
    ofile << "## Unit = mm, Angle = deg.\n";
    ofile << "## Side : All\n";
    ofile << "# Ref\tVal\tPackage\tPosX\tPosY\tRot\tSide\n";

    size_t cnum = 0;
    size_t dnum = 0;
    for (const auto& c : comps) {
        auto [cx, cy, cdeg] = c.transform;
        if (c.pads.size() == 2) {
            ofile << "C" << ++cnum << '\t' << "C_0603_1608Metric" << '\t' << "C_0603_1608Metric" << '\t';
        } else {
            if (std::abs(c.pads[0].x + c.pads[1].x) - 0.9 < 1e-3) {
                ofile << "D" << ++dnum << '\t' << "LED_SK6812_EC15_1.5x1.5mm" << '\t' << "LED_SK6812_EC15_1.5x1.5mm" << '\t';
            } else {
                ofile << "D" << ++dnum << '\t' << "LED_WS2812B_PLCC4_5.0x5.0mm_P3.2mm" << '\t' << "LED_WS2812B_PLCC4_5.0x5.0mm_P3.2mm" << '\t';
            }
        }
        ofile << cx << '\t' << cy << '\t' << (180.0 / M_PI) * cdeg << "\ttop\n";
    }
    ofile << "## End";
    ofile.close();

    std::cout << "[" << pid << "]\tGenerated\t\tPOS file\n";
}

void Layout::write_drl_file(const std::string& name, size_t pid) const {
// void write_drill_file(const std::string& filepath, std::vector<Via>& vias, double y_extent)
    auto vias = compute_drl(name, pid);
    v2d extent = m_sheet.get_patch_extent(pid);
    double y_extent = extent[1];
    
    assert(!vias.empty());
    auto comp_hd = [](const Via& lhs, const Via& rhs) { return lhs.hd < rhs.hd || (lhs.hd == rhs.hd && lhs.x < rhs.x); };
    std::sort(vias.begin(), vias.end(), comp_hd);

    auto pad_with_zeros = [](const std::string& str, size_t num_zeros) {
        return std::string(num_zeros - std::min(num_zeros, str.length()), '0') + str;
    };

    auto format_double = [](double val) {
        bool is_neg = val < 0.0;
        val = std::abs(val);
        // std::cout << "VAL\t" << val << '\n';
        val *= 1e4;
        std::forward_list<char> digits;
        int rounded_val = std::round(val);
        // std::cout << "RdVAL\t" << rounded_val << "\nDIGITs\t";
        while (rounded_val > 0) {
            int digit = rounded_val % 10;
            // std::cout << "dig " << digit << " -> ";
            // std::cout << static_cast<char>('0' + digit) << "\n";
            digits.push_front(static_cast<char>('0' + digit));
            rounded_val = (rounded_val - digit) / 10;
        }
        // std::cout << "\n";
        return (is_neg ? "-" : "") + std::string(digits.begin(), digits.end());
    };

    constexpr double MM_TO_IN = (1.0 / 25.4);

    std::vector<size_t> change_drills;
    double drill_size = 0.0;
    for (size_t i = 0; i < vias.size(); ++i) {
        // std::cout << "i" << i << "\tsz" << vias[i].hd << '\n';
        if (vias[i].hd != drill_size) {
            // std::cout << "NEW\n";
            change_drills.push_back(i);
            drill_size = vias[i].hd;
        }
    }

    std::string file_name = name + "_p" + std::to_string(pid) + "_PTH.drl";
    std::ofstream ofile(file_name);
    if (!ofile.is_open()) {
        std::cerr << "[ERROR] Could not open file at " << file_name << '\n';
        exit(EXIT_FAILURE);
    }
    ofile << std::setprecision(4) << std::fixed;

    // HEADER
    ofile << "M48" << '\n';         // start file
    ofile << "INCH,TZ" << '\n';   // inches, trailing zeros
    // define drills e.g. T1C0.0394 defines tool T1 with diameter 0.0394 in (1mm)
    ofile << "T01C" << MM_TO_IN * vias[0].hd << '\n';
    for (size_t i = 0; i < change_drills.size(); ++i) {
        ofile << "T" << pad_with_zeros(std::to_string(i + 2), 2) << "C" << MM_TO_IN * vias[change_drills[i]].hd << '\n';
    }
    ofile << "%\n";  // end of header info

    // BODY
    ofile << "G90" << '\n'; // absolute mode
    ofile << "G05" << '\n'; // drill mode
    ofile << "T01" << '\n'; // first drill
    size_t curr_drill = 0;
    for (size_t i = 0; i < vias.size(); ++i) {
        if (i == change_drills[curr_drill]) {   // change drills if necessary
            ++curr_drill;
            ofile << "T" << pad_with_zeros(std::to_string(curr_drill + 1), 2) << '\n';
        }

        const auto& v = vias[i];
        // std::cout << MM_TO_IN * v.x << "\t" << MM_TO_IN * v.y << "\t->" << "X" << format_double(MM_TO_IN * v.x) << "Y" << format_double(MM_TO_IN * v.y) << '\n';
        ofile << "X" << format_double(MM_TO_IN * v.x) << "Y" << format_double(MM_TO_IN * (v.y - y_extent)) << '\n';
    }

    // EOF
    ofile << "T0" << '\n';
    ofile << "M30";
    ofile.close();

    std::cout << "[" << pid << "]\tGenerated\t\tDRL file\n";
}

void Layout::write_face_file(
    const std::string& name, size_t pid) const
{
    SVGPath faces("none", "black", "stroke-width:" + std::to_string(m_settings.edgecuts_width_mm)
        + ";stroke-linecap:round");

    for (const auto& fc : m_patch_fis.at(pid)) {
        const auto& vs = fc.vertices;
        v2d v_st = center_vertex(vs[0], pid);
        faces << move_to(v_st[0], v_st[1], false) << SVG_SEP;
        for (size_t i = 1; i <= vs.size(); ++i) {
            v2d vi = center_vertex(vs[i % vs.size()], pid);
            faces << line_to(vi[0], vi[1], false) << SVG_SEP;
        }
    }

    v2d extent = m_sheet.get_patch_extent(pid);
    SVGFile svg_Faces(name + "_p" + std::to_string(pid) + "_faces.svg",
        extent[0], extent[1], true);
    svg_Faces.add_geometry_element(faces);
    svg_Faces.to_file();
    std::cout << "[" << pid << "]\tGenerated\t\tFACES file\n";
}

void Layout::write_rects_file(
    const std::string& name, size_t pid) const
{
    SVGPath rects("none", "black", "stroke-width:" + std::to_string(m_settings.edgecuts_width_mm)
        + ";stroke-linecap:round");
    v2d extent = m_sheet.get_patch_extent(pid);
    SVGFile svg_Rects(name + "_p" + std::to_string(pid) + "_rects.svg",
        extent[0], extent[1], true);
    // std::vector<SVGRect> rs;

    for (const auto& fi : m_patch_fis.at(pid)) {
        for (auto mod : fi.modules) {
            mod = center_module(mod, pid);
            auto [p0, p1, p2] = mod;
            v2d ctr = 0.5 * (p1 + p2);
            auto mod_tr = module_transform(mod);

            SVGRect rect("none", "black", "stroke-width:" + std::to_string(m_settings.edgecuts_width_mm)
                + ";stroke-linecap:round");
            rect.set_params(ctr[0], ctr[1], 180.0 / M_PI * std::get<2>(mod_tr), length(p1 - p0), length(p2 - p0), 0.0);
            svg_Rects.add_geometry_element(rect);
        }

        if (fi.has_connector) {
            auto mod = center_module(fi.connector, pid);
            auto [p0, p1, p2] = mod;
            v2d ctr = 0.5 * (p1 + p2);
            auto mod_tr = module_transform(mod);

            SVGRect rect("none", "black", "stroke-width:" + std::to_string(m_settings.edgecuts_width_mm)
                + ";stroke-linecap:round");
            rect.set_params(ctr[0], ctr[1], 180.0 / M_PI * std::get<2>(mod_tr), length(p1 - p0), length(p2 - p0), 0.0);
            svg_Rects.add_geometry_element(rect);
        }
    }

    // SVGCircle c1("black", "none", "");
    // c1.set_params(0.0, 0.0, 0.0, 0.4);
    // svg_Rects.add_geometry_element(c1);
    // SVGCircle c2("black", "none", "");
    // c2.set_params(extent[0], extent[1], 0.0, 0.4);
    // svg_Rects.add_geometry_element(c2);
    // SVGCircle c3("black", "none", "");
    // c3.set_params(0.0, extent[1], 0.0, 0.4);
    // svg_Rects.add_geometry_element(c3);
    // SVGCircle c4("black", "none", "");
    // c4.set_params(extent[0], 0.0, 0.0, 0.4);
    // svg_Rects.add_geometry_element(c4);

    svg_Rects.to_file();
    std::cout << "[" << pid << "]\tGenerated\t\tRECTS file\n";
}


void Layout::write_auxiliary_files(const std::string& name) {
    for (size_t pid = 0; pid < m_sheet.patch_count(); ++pid) {
        write_pos_file(name, pid);
        write_drl_file(name, pid);
//        write_face_file(name, pid);
//        write_rects_file(name, pid);
    }
}

void Layout::compute_layout() {
    for (size_t pid = 0; pid < m_sheet.patch_count(); ++pid) {
        compute_layout_patch(pid);
    }
}

void Layout::compute_layout_patch(size_t pid) {
    for (size_t lid = 0; lid < AllLayers.size(); ++lid) {
        Layer layer = get_layer_from_id(lid);
        compute_layer(pid, layer);
    }
}

void Layout::compute_layer(size_t pid, Layer layer) {
    switch (layer) {
        case Layer::EdgeCuts:    compute_EdgeCuts(pid); break;
        case Layer::FCu:         compute_FCu(pid); break;
        case Layer::FMask:       compute_FMask(pid); break;
        case Layer::FPaste:      compute_FPaste(pid); break;
        case Layer::FSilkscreen: compute_FSilkscreen(pid); break;
        case Layer::BCu:         compute_BCu(pid); break;
        case Layer::BMask:       compute_BMask(pid); break;
        default:
            std::cerr << "[ERROR] Unhandled type of layer\n";
            exit(EXIT_FAILURE);
            break;
    }
}

void Layout::compute_EdgeCuts(size_t pid) {
    auto& patch_layer = m_patch_layouts.at(pid).at(get_layer_id(Layer::EdgeCuts));

    patch_layer.push_back(contour_to_svgpath(pid));
    // for (auto hid : m_sheet.get_patch_hinges(pid)) {
    //     patch_layer.push_back(fullhinge_hole_to_svgpath(hid));
    // }

    const auto& mvo = m_sheet.get_vertex_order(pid);

    // auto[vt0, id0, vid0, fid0] = mvo[0];
    size_t i = 1;

    while (i < mvo.size()) {
        auto[vt, id, vid, fid] = mvo[i];
        if (vt == VertexType::hinge) {
            SVGPath hole;
            if (m_sheet.is_half_hinge(id)) {
                hole = halfhinge_hole_to_svgpath(mvo[i], mvo[i+1]);
            } else {
                hole = fullhinge_hole_to_svgpath(mvo[i], mvo[i+1]);
            }
            if (!hole.get_dvalue().empty()) { patch_layer.push_back(hole); }
            i += 2;
        } else {
            ++i;
        }
    }
    
    std::cout << "[" << pid << "]\tGenerated\t\tEdgeCuts\n";
}

void Layout::compute_FCu(size_t pid) {
    using namespace ClipperLib;
    
    auto& patch_layer = m_patch_layouts.at(pid).at(get_layer_id(Layer::FCu));

    SVGPath module_traces_path("none", "black", "stroke-width:" + std::to_string(m_settings.trace_width_mm) +
                                    ";stroke-linecap:round");
    SVGPath module_vias_path("black", "none", "");
    SVGPath module_pads_path("black", "none", "");
    SVGPath vcc_plane_path("red", "none", "stroke-width:" + std::to_string(m_settings.plane_width_mm));
    // vcc_plane_path.add_attribute(to_attribute("fill-rule", "nonzero"));

    Clipper cl; Paths ps;

    for (const auto& fi : m_patch_fis.at(pid)) {
        for (auto mod : fi.modules) {
            mod = center_module(mod, pid);
            auto mod_tr = module_transform(mod);
            for (const auto& trace : m_module_info.traces) {
                bool first = true;
                for (auto [x, y] : trace) {
                    // auto [x, y] = center_vertex(p, pid);
                    auto [xx, yy] = apply_transform_mod(mod_tr, x, y);
                    if (first) {
                        first = false;
                        module_traces_path << move_to(xx, yy, false) << SVG_SEP;
                    } else {
                        module_traces_path << line_to(xx, yy, false) << SVG_SEP;
                    }
                }
            }

            for (const auto& via : m_module_info.vias) {
                auto via_paths = generate_via_paths(m_settings, via, mod_tr);
                module_vias_path.from_v2ds(via_paths);
            }

            for (const auto& comp : m_module_info.comps) {
                for (const auto& pad : comp.pads) {
                    auto pad_tr = pad_transform(mod, comp, pad);
                    auto pad_paths = generate_pad_paths(m_settings, pad, pad_tr);
                    module_pads_path.from_v2ds(pad_paths);
                }
            }
        }

        if (fi.has_connector) {
            auto mod = center_module(fi.connector, pid);
            auto mod_tr = module_transform(mod);
            for (const auto& via : m_connector_info.vias) {
                auto via_paths = generate_via_paths(m_settings, via, mod_tr);
                cl.AddPaths(vv2ds_to_paths(m_settings, via_paths), PolyType::ptClip, true);
                // module_vias_path.from_v2ds(via_paths);
            }
        }
    }
    patch_layer.push_back(module_traces_path);
    patch_layer.push_back(module_vias_path);
    patch_layer.push_back(module_pads_path);

    auto [tw, id, od] = m_sheet.get_hinge_parameters();
    const auto& mvo = m_sheet.get_vertex_order(pid);
    size_t i = 0;

    while (i < mvo.size()) {
        auto[vt, id, vid, fid] = mvo[i];
        if (vt == VertexType::hinge) {
            SVGPath half_hinge("black", "none", "");
            if (m_sheet.is_half_hinge(id)) {
                half_hinge.from_v2ds(half_halfhinge_trace_to_svgpath(mvo[i], mvo[i + 1],
                    0.5, m_settings.plane_inset_depth_mm, tw - 2.0 * m_settings.edge_clearance_mm));
                // half_hinge.from_v2ds(half_halfhinge_trace_to_svgpath(mvo[i], mvo[i + 1],
                //     0.5, 0.0, tw - 2.0 * EDGE_CLEARANCE));
            } else {
                half_hinge.from_v2ds(half_fullhinge_trace_to_svgpath(mvo[i], mvo[i + 1],
                    0.5, m_settings.plane_inset_depth_mm, tw - 2.0 * m_settings.edge_clearance_mm));
                // half_hinge.from_v2ds(half_fullhinge_trace_to_svgpath(mvo[i], mvo[i + 1],
                //     0.5, 0.0, tw - 2.0 * EDGE_CLEARANCE));
            }
            patch_layer.push_back(half_hinge);
            i += 2;
        } else { ++i; }
    }

    auto vccp = vcc_plane(pid);
    // Paths ps_tmp;
    cl.AddPaths(vv2ds_to_paths(m_settings, vccp), PolyType::ptSubject, true);
    cl.Execute(ClipType::ctUnion, ps, PolyFillType::pftEvenOdd);

    vcc_plane_path.from_v2ds(paths_to_vv2ds(m_settings, ps));
    // vcc_plane_path.from_v2ds(vccp);
#ifndef DISABLE_PLANES
    patch_layer.push_back(vcc_plane_path);
#endif
    std::cout << "[" << pid << "]\tGenerated\t\tFCu\n";
}

void Layout::compute_FMask(size_t pid) {
    auto& patch_layer = m_patch_layouts.at(pid).at(get_layer_id(Layer::FMask));

    SVGPath module_vias_path("black", "none", "");
    SVGPath module_pads_path("black", "none", "stroke-width:" + std::to_string(m_settings.plane_width_mm));

    for (const auto& ti : m_patch_fis.at(pid)) {
        for (auto mod : ti.modules) {
            mod = center_module(mod, pid);
            for (const auto& comp : m_module_info.comps) {
                for (const auto& pad : comp.pads) {
                    auto pad_tr = pad_transform(mod, comp, pad);
                    auto pad_paths = generate_pad_paths(m_settings, pad, pad_tr);
                    module_pads_path.from_v2ds(pad_paths);
                }
            }
        }

        if (ti.has_connector) {
            auto mod = center_module(ti.connector, pid);
            auto mod_tr = module_transform(mod);
            for (const auto& via : m_connector_info.vias) {
                auto via_paths = generate_via_paths(m_settings, via, mod_tr);
                module_vias_path.from_v2ds(via_paths);
            }
        }
    }
    patch_layer.push_back(module_vias_path);
    patch_layer.push_back(module_pads_path);

    std::cout << "[" << pid << "]\tGenerated\t\tFMask\n";
}

void Layout::compute_FPaste(size_t pid) {
    auto& patch_layer = m_patch_layouts.at(pid).at(get_layer_id(Layer::FPaste));

    SVGPath module_pads_path("black", "none", "stroke-width:" + std::to_string(m_settings.plane_width_mm));

    for (const auto& ti : m_patch_fis.at(pid)) {
        for (auto mod : ti.modules) {
            mod = center_module(mod, pid);
            for (const auto& comp : m_module_info.comps) {
                for (const auto& pad : comp.pads) {
                    auto pad_tr = pad_transform(mod, comp, pad);
                    auto pad_paths = generate_pad_paths(m_settings, pad, pad_tr);
                    module_pads_path.from_v2ds(pad_paths);
                }
            }
        }
    }
    patch_layer.push_back(module_pads_path);

    std::cout << "[" << pid << "]\tGenerated\t\tFPaste\n";
}

void Layout::compute_FSilkscreen(size_t pid) {
    auto& patch_layer = m_patch_layouts.at(pid).at(get_layer_id(Layer::FSilkscreen));

    SVGPath ss_path("none", "black", "stroke-width:" + std::to_string(m_settings.silkscreen_width_mm) +
                                    ";stroke-linecap:round");
    for (const auto& ti : m_patch_fis.at(pid)) {
        for (auto mod : ti.modules) {
            mod = center_module(mod, pid);
            for (const auto& ss_geom : m_module_info.ss_marks) {
                // module coordinates
                auto [p1, p2] = ss_geom;
                // auto [p1x, p1y] = center_vertex(p1, pid);
                // auto [p2x, p2y] = center_vertex(p2, pid);
                auto [p1x, p1y] = p1;
                auto [p2x, p2y] = p2;
                // global coordinates
                auto [tp1x, tp1y] = apply_transform_mod(module_transform(mod), p1x, p1y);
                auto [tp2x, tp2y] = apply_transform_mod(module_transform(mod), p2x, p2y);

                ss_path << move_to(tp1x, tp1y, false) << SVG_SEP;
                ss_path << line_to(tp2x, tp2y, false) << SVG_SEP;
            }
        }
    }
    patch_layer.push_back(ss_path);
    std::cout << "[" << pid << "]\tGenerated\t\tFSilkscreen\n";
}

void Layout::compute_BCu(size_t pid) {
    using namespace ClipperLib;

    // auto ortho_proj = [](v2d p0, v2d p1, v2d q) {
    //     return p0 + dot(q - p0, p1 - p0) / sqLength(p1 - p0) * (p1 - p0);
    // };

    auto& patch_layer = m_patch_layouts.at(pid).at(get_layer_id(Layer::BCu));

    // Traces between modules and IO
    SVGPath face_traces_path("none", "black", "stroke-width:" + std::to_string(m_settings.trace_width_mm) +
                                                 ";stroke-linecap:round;stroke-linejoin:round");
    // Filled vias paths
    SVGPath module_vias_path("black", "none", "");
    SVGPath gnd_plane_path("green", "green", "stroke-width:" + std::to_string(m_settings.plane_width_mm));

    // std::vector<std::vector<v2d>> traces;
    std::vector<Via> vias;

    for (const auto& fi : m_patch_fis.at(pid)) {
        // Generate via paths and gather info for trace avoidance
        for (auto mod : fi.modules) {
            mod = center_module(mod, pid);
            auto mod_tr = module_transform(mod);
            for (size_t k = 0; k < m_module_info.vias.size(); ++k) {
                const auto& via = m_module_info.vias[k];
                // ignore last via (gnd via)
                if (k < m_module_info.vias.size() - 1) {
                    auto via_paths = generate_via_paths(m_settings, via, mod_tr);
                    module_vias_path.from_v2ds(via_paths);
                }

                v2d via_tr = apply_transform_mod(mod_tr, v2d(via.x, via.y));
                Via v; v.vd = via.vd; v.hd = via.hd; v.x = via_tr[0]; v.y = via_tr[1];
                vias.push_back(v);
            }
        }

        if (fi.has_connector) {
            auto mod = center_module(fi.connector, pid);
            auto mod_tr = module_transform(mod);
            for (size_t k = 0; k < m_connector_info.vias.size() - 1; ++k) {
                const auto& via = m_connector_info.vias[k];
                // ignore last via (gnd via)
                if (k < m_connector_info.vias.size() - 1) {
                    auto via_paths = generate_via_paths(m_settings, via, mod_tr);
                    module_vias_path.from_v2ds(via_paths);
                }

                v2d via_tr = apply_transform_mod(mod_tr, v2d(via.x, via.y));
                Via v; v.vd = via.vd; v.hd = via.hd; v.x = via_tr[0]; v.y = via_tr[1];
                vias.push_back(v);
            }

        }

        auto traces = center_vv2ds(generate_traces(m_settings, fi), pid);
        face_traces_path.from_v2ds(traces);
    }
    patch_layer.push_back(face_traces_path);
    patch_layer.push_back(module_vias_path);

    auto [tkw, id, od] = m_sheet.get_hinge_parameters();
    const auto& mvo = m_sheet.get_vertex_order(pid);
    size_t i = 0;

    // Generate hinge traces
    while (i < mvo.size()) {
        auto[vt, id, vid, fid] = mvo[i];
        if (vt == VertexType::hinge) {
            SVGPath half_hinge_data("black", "none", "");
            SVGPath half_hinge_gnd("black", "none", "");
            double gnd_width = tkw - (2.0 * m_settings.edge_clearance_mm + m_settings.trace_width_mm + m_settings.trace_clearance_mm);
            
            double data_pos = (tkw - (0.5 * m_settings.trace_width_mm + m_settings.edge_clearance_mm)) / tkw;
            double gnd_pos = (m_settings.edge_clearance_mm + 0.5 * gnd_width) / tkw;

            if (m_sheet.is_half_hinge(id)) {
                half_hinge_data.from_v2ds(half_halfhinge_trace_to_svgpath(mvo[i], mvo[i + 1],
                    data_pos,
                    0.0,
                    // DATA_INSET_DEPTH,
                    m_settings.trace_width_mm));
                half_hinge_gnd.from_v2ds(half_halfhinge_trace_to_svgpath(mvo[i], mvo[i + 1],
                    gnd_pos,
                    m_settings.plane_inset_depth_mm,
                    gnd_width));
            } else {
                half_hinge_data.from_v2ds(half_fullhinge_trace_to_svgpath(mvo[i], mvo[i + 1],
                    data_pos,
                    0.0,
                    // DATA_INSET_DEPTH,
                    m_settings.trace_width_mm));
                half_hinge_gnd.from_v2ds(half_fullhinge_trace_to_svgpath(mvo[i], mvo[i + 1],
                    gnd_pos,
                    m_settings.plane_inset_depth_mm,
                    gnd_width));
            }

            patch_layer.push_back(half_hinge_data);
            patch_layer.push_back(half_hinge_gnd);
            i += 2;
        } else { ++i; }
    }
    
    // Generate GND plane per triangle
    for (size_t pfid = 0; pfid < m_patch_fis.at(pid).size(); ++pfid) {
        auto plane = center_vv2ds(paths_to_vv2ds(m_settings, generate_gnd_plane(m_settings, m_patch_fis.at(pid).at(pfid))), pid);
        gnd_plane_path.from_v2ds(plane);
    }

#ifndef DISABLE_PLANES
    patch_layer.push_back(gnd_plane_path);
#endif
    std::cout << "[" << pid << "]\tGenerated\t\tBCu\n";
}

void Layout::compute_BMask(size_t pid) {
    auto& patch_layer = m_patch_layouts.at(pid).at(get_layer_id(Layer::BMask));

    SVGPath module_vias_path("black", "none", "stroke-width:" + std::to_string(m_settings.plane_width_mm));

    for (const auto& ti : m_patch_fis.at(pid)) {
        if (ti.has_connector) {
            auto mod = center_module(ti.connector, pid);
            auto mod_tr = module_transform(mod);
            for (const auto& via : m_connector_info.vias) {
                auto via_paths = generate_via_paths(m_settings, via, mod_tr);
                module_vias_path.from_v2ds(via_paths);
            }
        }
    }
    patch_layer.push_back(module_vias_path);

    std::cout << "[" << pid << "]\tGenerated\t\tBMask\n";
}

std::vector<Component> Layout::compute_pos(const std::string& name, size_t pid) const {
    v2d extent = m_sheet.get_patch_extent(pid);
    SVGFile svg_Comps(name + "_p" + std::to_string(pid) + "_pos.svg",
        extent[0], extent[1], true);
    // auto lift_circ = [](const SVGCircle& circ) -> GeometryElement{ return circ; };
    auto lift_rect = [](const SVGRect& rect) -> GeometryElement{ return rect; };

    std::vector<Component> patch_comps;

    for (const auto& fi : m_patch_fis.at(pid)) {
        for (auto mod : fi.modules) {
            mod = center_module(mod, pid);
            // auto mod_tr = module_transform(mod);

            for (const auto& comp : m_module_info.comps) {
                auto comp_tr = component_transform(mod, comp);
                patch_comps.push_back({comp_tr, comp.pads});
                // debug
                SVGCircle comp_circ("blue", "blue", "");
                SVGRect comp_rect("blue", "blue", "");
                comp_rect.set_params(std::get<0>(comp_tr), std::get<1>(comp_tr), 180.0 / M_PI * std::get<2>(comp_tr), 1.0, 0.5, 0.0);
                // svg_Comps.add_geometry_element(lift_circ(comp_circ));
                comp_circ.set_params(std::get<0>(comp_tr), std::get<1>(comp_tr), 0, 1.0);
                svg_Comps.add_geometry_element(lift_rect(comp_rect));
            }
        }
    }

//    svg_Comps.to_file();
    return patch_comps;
}

std::vector<Via> Layout::compute_drl(const std::string& name, size_t pid) const {
    v2d extent = m_sheet.get_patch_extent(pid);
    SVGFile svg_Drill(name + "_p" + std::to_string(pid) + "_drl.svg",
        extent[0], extent[1], true);
    auto lift_circ = [](const SVGCircle& circ) -> GeometryElement{ return circ; };

    std::vector<Via> patch_vias;

    for (const auto& fi : m_patch_fis.at(pid)) {
        for (auto mod : fi.modules) {
            mod = center_module(mod, pid);
            auto mod_tr = module_transform(mod);

            for (const auto& via : m_module_info.vias) {
                auto [vx, vy] = apply_transform_mod(mod_tr, via.x, via.y);
                patch_vias.push_back({via.vd, via.hd, vx, vy});
                // debug
                SVGCircle via_circ("green", "green", "");
                via_circ.set_params(vx, vy, via.hd, 0.5 * (via.hd - via.vd));
                svg_Drill.add_geometry_element(lift_circ(via_circ));
            }
        }

        if (fi.has_connector) {
            auto mod = center_module(fi.connector, pid);
            auto mod_tr = module_transform(mod);
            for (const auto& via : m_connector_info.vias) {
                auto [vx, vy] = apply_transform_mod(mod_tr, via.x, via.y);
                patch_vias.push_back({via.vd, via.hd, vx, vy});
                // debug
                SVGCircle via_circ("green", "green", "");
                via_circ.set_params(vx, vy, via.hd, 0.5 * (via.hd - via.vd));
                svg_Drill.add_geometry_element(lift_circ(via_circ));
            }
        }
    }

    svg_Drill.to_file();

    return patch_vias;
}

std::string Layout::half_fullhinge_to_svgpath_commands(const ExtendedVertex& ev0, const ExtendedVertex& ev1) const {
    auto[vt0, hid0, vid0, fid0] = ev0;
    auto[vt1, hid1, vid1, fid1] = ev1;
    assert(vt0 == VertexType::hinge && vt1 == VertexType::hinge);
    assert(hid0 == hid1);

    v2d v0 = m_sheet.get_vertex(vid0, true);
    v2d v1 = m_sheet.get_vertex(vid1, true);

    v2d dir = normalize(v1 - v0);     // in the direction and sense of the hinge
    v2d ortho = v2d(dir[1], -dir[0]); // (ortho, dir) is direct
    
    auto [tw, id, od] = m_sheet.get_hinge_parameters();
    const double ml = m_sheet.get_hinge_min_length(hid0);
    double hl = m_sheet.get_hinge_length(hid0);

    SVGPath half_hinge("none", "black", "stroke-width:" + std::to_string(m_settings.edgecuts_width_mm));
    v2d pt = v0 - (0.5 * hl - (tw + 0.5 * od)) * ortho;
    half_hinge << line_to(pt[0], pt[1], false) << SVG_SEP;
    pt = od * dir;
    half_hinge << arc_to(0.5 * length(pt), true, false, pt[0], pt[1]) << SVG_SEP;
    pt = (0.5 * (hl - ml)) * ortho;
    half_hinge << line_to(pt[0], pt[1]) << SVG_SEP;
    pt = (2.0 * tw + id) * dir;
    half_hinge << arc_to(0.5 * length(pt), true, true, pt[0], pt[1]) << SVG_SEP;
    pt = -(0.5 * (hl - ml)) * ortho;
    half_hinge << line_to(pt[0], pt[1]) << SVG_SEP;
    pt = od * dir;
    half_hinge << arc_to(0.5 * length(pt), true, false, pt[0], pt[1]);

    return half_hinge.get_dvalue();
}

std::string Layout::half_halfhinge_to_svgpath_commands(const ExtendedVertex& ev0, const ExtendedVertex& ev1) const {
    auto[vt0, hid0, vid0, fid0] = ev0;
    auto[vt1, hid1, vid1, fid1] = ev1;
    assert(vt0 == VertexType::hinge && vt1 == VertexType::hinge);
    assert(hid0 == hid1);

    v2d v0 = m_sheet.get_vertex(vid0, true);
    v2d v1 = m_sheet.get_vertex(vid1, true);
    auto [shrink_left, shrink_right] = m_sheet.shrink_half_hinge(hid0, fid0, fid1);

    v2d dir = normalize(v1 - v0);     // in the direction and sense of the hinge
    v2d ortho = v2d(dir[1], -dir[0]); // (ortho, dir) is direct: ortho towards inside
    
    v2d v0_s = v0 - shrink_right * ortho;
    // v2d v1_s = v1 - shrink_right * ortho;

    auto [tw, id, od] = m_sheet.get_hinge_parameters();
    const double ml = m_sheet.get_hinge_min_length(hid0);
    double hl = m_sheet.get_hinge_length(hid0) - (shrink_left + shrink_right);
    v2d v0_m = v0_s - 0.5 * hl * ortho;

    SVGPath half_hinge("none", "black", "stroke-width:" + std::to_string(m_settings.edgecuts_width_mm));
    
    assert(m_sheet.faces_in_same_patch(fid0, fid1));
    if (m_sheet.lt_depth(fid0, fid1)) { // NARROW -> WIDE
        // v2d pt = v0_s - (0.5 * hl - (tw + 0.5 * od)) * ortho;
        v2d pt = v0_m + (tw + 0.5 * od) * ortho;
        half_hinge << line_to(pt[0], pt[1], false) << SVG_SEP;
        pt = od * dir;
        half_hinge << arc_to(0.5 * length(pt), true, false, pt[0], pt[1]) << SVG_SEP;
        pt = (0.5 * (hl - ml)) * ortho;
        half_hinge << line_to(pt[0], pt[1]) << SVG_SEP;
        pt = (tw + 0.5 * id) * dir + (tw + 0.5 * id) * ortho;
        half_hinge << arc_to(tw + 0.5 * id, false, true, pt[0], pt[1]) << SVG_SEP;
        if (id > od) {
            pt = 0.5 * (id - od) * dir;
            half_hinge << line_to(pt[0], pt[1]) << SVG_SEP;
        }
        pt = 0.5 * od * dir + 0.5 * od * ortho;
        half_hinge << arc_to(0.5 * od, false, false, pt[0], pt[1]);
    } else { // WIDE -> NARROW
        v2d pt = v0_s;
        half_hinge << line_to(pt[0], pt[1], false) << SVG_SEP;
        pt = 0.5 * od * dir - 0.5 * od * ortho;;
        half_hinge << arc_to(0.5 * od, false, false, pt[0], pt[1]) << SVG_SEP;
        if (id > od) {
            pt = 0.5 * (id - od) * dir;
            half_hinge << line_to(pt[0], pt[1]) << SVG_SEP;
        }
        pt = (tw + 0.5 * id) * dir - (tw + 0.5 * id) * ortho;
        half_hinge << arc_to(tw + 0.5 * id, false, true, pt[0], pt[1]) << SVG_SEP;
        pt = - (0.5 * (hl - ml)) * ortho;
        half_hinge << line_to(pt[0], pt[1]) << SVG_SEP;
        pt = od * dir;
        half_hinge << arc_to(0.5 * od, true, false, pt[0], pt[1]);
    }

    return half_hinge.get_dvalue();
}

std::vector<v2d> Layout::half_fullhinge_trace_pts(const ExtendedVertex& ev0, const ExtendedVertex& ev1, double t, double inset) const {
    constexpr size_t samples_per_tour = 64;
    
    auto[vt0, hid0, vid0, fid0] = ev0;
    auto[vt1, hid1, vid1, fid1] = ev1;
    assert(vt0 == VertexType::hinge && vt1 == VertexType::hinge);
    assert(hid0 == hid1);

    v2d v0 = m_sheet.get_vertex(vid0, true);
    v2d v1 = m_sheet.get_vertex(vid1, true);

    // (dir, ortho) direct basis
    v2d dir = normalize(m_sheet.get_hinge_ortho(hid0));
    v2d ortho = normalize(m_sheet.get_hinge_parallel(hid0));
    if (dot(dir, v1 - v0) < 0.0) { dir = -dir; }
    if (dot(v2d(dir[1], -dir[0]), ortho) < 0.0) { ortho = -ortho; }
    
    auto [tw, id, od] = m_sheet.get_hinge_parameters();
    double hl = m_sheet.get_hinge_length(hid0);

    std::vector<v2d> pts;
    v2d pt = v0 - (0.5 * hl - t * tw) * ortho - inset * dir;
    pts.push_back(pt);
    
    v2d off0 = (0.5 * od + inset) * dir;
    v2d off1 = (0.5 * od + (1.0 - t) * tw) * ortho + (0.5 * od + (1.0 - t) * tw) * dir;
    pt = pt + off0;
    pts.push_back(pt);
    
    double angle = 0.5 * M_PI;
    auto arc_pts = generate_arc(pt, pt + off1, angle, true, samples_per_tour);
    pts.insert(pts.end(), arc_pts.begin(), arc_pts.end());
    pt = pt + off1;
    pts.push_back(pt);

    off0 = (0.5 * hl - (2.0 * tw + 0.5 * id + 0.5 * od)) * ortho;
    pt = pt + off0;
    pts.push_back(pt);

    off1 = 2.0 * (0.5 * id + t * tw) * dir;
    angle = M_PI;
    arc_pts = generate_arc(pt, pt + off1, angle, false, samples_per_tour);
    pts.insert(pts.end(), arc_pts.begin(), arc_pts.end());
    pt = pt + off1;
    pts.push_back(pt);

    off0 = -(0.5 * hl - (2.0 * tw + 0.5 * id + 0.5 * od)) * ortho;
    pt = pt + off0;
    pts.push_back(pt);

    off1 = -(0.5 * od + (1.0 - t) * tw) * ortho + (0.5 * od + (1.0 - t) * tw) * dir;
    angle = 0.5 * M_PI;
    arc_pts = generate_arc(pt, pt + off1, angle, true, samples_per_tour);
    pts.insert(pts.end(), arc_pts.begin(), arc_pts.end());
    pt = pt + off1;
    pts.push_back(pt);

    pts.push_back(pt + (0.5 * od + inset) * dir);

    return pts;
}

std::vector<std::vector<v2d>> Layout::half_fullhinge_trace_to_svgpath(const ExtendedVertex& ev0, const ExtendedVertex& ev1, double t, double inset, double width) const {
    using namespace ClipperLib;
    auto pts = half_fullhinge_trace_pts(ev0, ev1, t, inset);
    
    ClipperOffset co; Paths ps;
    co.AddPath(v2ds_to_path(m_settings, pts), JoinType::jtRound, EndType::etOpenButt);
    co.Execute(ps, 0.5 * m_settings.clipper_scaling * width);

    return paths_to_vv2ds(m_settings, ps);
}

std::vector<v2d> Layout::half_halfhinge_trace_pts(const ExtendedVertex& ev0, const ExtendedVertex& ev1, double t, double inset) const {
    constexpr size_t samples_per_tour = 64;
    
    auto[vt0, hid0, vid0, fid0] = ev0;
    auto[vt1, hid1, vid1, fid1] = ev1;
    assert(vt0 == VertexType::hinge && vt1 == VertexType::hinge);
    assert(hid0 == hid1);
    size_t hid = hid0;

    v2d v0 = m_sheet.get_vertex(vid0, true);
    v2d v1 = m_sheet.get_vertex(vid1, true);
    auto [shrink_left, shrink_right] = m_sheet.shrink_half_hinge(hid0, fid0, fid1);

    // (dir, ortho) direct basis
    v2d dir = normalize(m_sheet.get_hinge_ortho(hid0));
    v2d ortho = normalize(m_sheet.get_hinge_parallel(hid0));
    if (dot(dir, v1 - v0) < 0.0) { dir = -dir; }
    if (dot(v2d(dir[1], -dir[0]), ortho) < 0.0) { ortho = -ortho; }
    
    v2d v0_s = v0 - shrink_right * ortho;
    // v2d v1_s = v1 - shrink_right * ortho;

    auto [tw, id, od] = m_sheet.get_hinge_parameters();
    // double true_hl = m_sheet.get_hinge_length(hid0);
    const double ml = m_sheet.get_hinge_min_length(hid0);
    double hl = m_sheet.get_hinge_length(hid) - (shrink_left + shrink_right);

    std::vector<v2d> pts;
    assert(m_sheet.faces_in_same_patch(fid0, fid1));
    if (m_sheet.lt_depth(fid0, fid1)) {
        v2d pt = v0_s - (0.5 * hl - t * tw) * ortho - inset * dir;
        pts.push_back(pt);
        
        v2d off0 = (0.5 * od + inset) * dir;
        pt = pt + off0;
        pts.push_back(pt);
        
        v2d off1 = (0.5 * od + (1.0 - t) * tw) * ortho + (0.5 * od + (1.0 - t) * tw) * dir;
        double angle = 0.5 * M_PI;
        auto arc_pts = generate_arc(pt, pt + off1, angle, true, samples_per_tour);
        pts.insert(pts.end(), arc_pts.begin(), arc_pts.end());
        pt = pt + off1;
        pts.push_back(pt);

        off0 = (0.5 * (hl - ml)) * ortho;
        pt = pt + off0;
        pts.push_back(pt);

        off1 = (t * tw + 0.5 * id) * ortho + (0.5 * id + t * tw) * dir;
        angle = 0.5 * M_PI;
        arc_pts = generate_arc(pt, pt + off1, angle, false, samples_per_tour);
        pts.insert(pts.end(), arc_pts.begin(), arc_pts.end());
        pt = pt + off1;
        pts.push_back(pt);

        off0 = (0.5 * id + inset) * dir;
        pt = pt  + off0;
        pts.push_back(pt);
    } else {
        v2d pt = v0_s - ((1.0 - t) * tw + 0.5 * od) * ortho - inset * dir;
        pts.push_back(pt);

        v2d off0 = (0.5 * id + inset) * dir;
        pt = pt + off0;
        pts.push_back(pt);

        v2d off1 = -(t * tw + 0.5 * id) * ortho + (0.5 * id + t * tw) * dir;
        double angle = 0.5 * M_PI;
        auto arc_pts = generate_arc(pt, pt + off1, angle, false, samples_per_tour);
        pts.insert(pts.end(), arc_pts.begin(), arc_pts.end());
        pt = pt + off1;
        pts.push_back(pt);

        off0 = -(0.5 * (hl - ml)) * ortho;
        pt = pt + off0;
        pts.push_back(pt);

        off1 = -(0.5 * od + (1.0 - t) * tw) * ortho + (0.5 * od + (1.0 - t) * tw) * dir;
        angle = 0.5 * M_PI;
        arc_pts = generate_arc(pt, pt + off1, angle, true, samples_per_tour);
        pts.insert(pts.end(), arc_pts.begin(), arc_pts.end());
        pt = pt + off1;
        pts.push_back(pt);

        off0 = (0.5 * od + inset) * dir;
        pt = pt + off0;
        pts.push_back(pt);
    }

    // off0 = -(0.5 * hl - (2.0 * tw + 0.5 * id + 0.5 * od)) * ortho;
    // pt = pt + off0;
    // pts.push_back(pt);

    // off1 = -(0.5 * od + (1.0 - t) * tw) * ortho + (0.5 * od + (1.0 - t) * tw) * dir;
    // angle = 0.5 * M_PI;
    // arc_pts = generate_arc(pt, pt + off1, angle, true, samples_per_tour);
    // pts.insert(pts.end(), arc_pts.begin(), arc_pts.end());
    // pt = pt + off1;
    // pts.push_back(pt);

    // pts.push_back(pt + (0.5 * od + inset) * dir);

    return pts;
}

std::vector<std::vector<v2d>> Layout::half_halfhinge_trace_to_svgpath(const ExtendedVertex& ev0, const ExtendedVertex& ev1, double t, double inset, double width) const {
    using namespace ClipperLib;
    auto pts = half_halfhinge_trace_pts(ev0, ev1, t, inset);
    
    ClipperOffset co; Paths ps;
    co.AddPath(v2ds_to_path(m_settings, pts), JoinType::jtRound, EndType::etOpenButt);
    co.Execute(ps, 0.5 * m_settings.clipper_scaling * width);

    return paths_to_vv2ds(m_settings, ps);
}

SVGPath Layout::fullhinge_hole_to_svgpath(const ExtendedVertex& ev0, const ExtendedVertex& ev1) const {
    auto[vt0, hid0, vid0, fid0] = ev0;
    auto[vt1, hid1, vid1, fid1] = ev1;
    assert(vt0 == VertexType::hinge && vt1 == VertexType::hinge);
    assert(hid0 == hid1);
    size_t hid = hid0;
    
    v2d center = m_sheet.get_hinge_center(hid, true);
    v2d dir = normalize(m_sheet.get_hinge_ortho(hid));
    v2d ortho = normalize(m_sheet.get_hinge_parallel(hid));

    auto [tw, id, od] = m_sheet.get_hinge_parameters();
    double hl = m_sheet.get_hinge_length(hid);

    SVGPath hole("none", "black", "stroke-width:" + std::to_string(m_settings.edgecuts_width_mm));
    if (fid0 < fid1) {
        v2d pt = center - (0.5 * id) * dir;
        hole << move_to(pt[0], pt[1], false) << SVG_SEP;
        pt = (0.5 * hl - tw - 0.5 * id) * ortho;
        hole << line_to(pt[0], pt[1]) << SVG_SEP;
        pt = id * dir;
        hole << arc_to(0.5 * length(pt), true, true, pt[0], pt[1]) << SVG_SEP;
        pt = -(hl - 2.0 * tw - id) * ortho;
        hole << line_to(pt[0], pt[1]) << SVG_SEP;
        pt = -id * dir;
        hole << arc_to(0.5 * length(pt), true, true, pt[0], pt[1]) << SVG_SEP;
        hole << close_path();
    }

    return hole;
}

SVGPath Layout::halfhinge_hole_to_svgpath(const ExtendedVertex& ev0, const ExtendedVertex& ev1) const {
    auto[vt0, hid0, vid0, fid0] = ev0;
    auto[vt1, hid1, vid1, fid1] = ev1;
    assert(vt0 == VertexType::hinge && vt1 == VertexType::hinge);
    assert(hid0 == hid1);
    size_t hid = hid0;
    
    v2d v0 = m_sheet.get_vertex(vid0, true);
    v2d v1 = m_sheet.get_vertex(vid1, true);
    auto [shrink_left, shrink_right] = m_sheet.shrink_half_hinge(hid0, fid0, fid1);

    v2d dir = normalize(v1 - v0);     // in the direction and sense of the hinge
    v2d ortho = v2d(dir[1], -dir[0]); // (ortho, dir) is direct: ortho towards inside

    v2d v0_s = v0 - shrink_right * ortho;
    // v2d v1_s = v1 - shrink_right * ortho;

    auto [tw, id, od] = m_sheet.get_hinge_parameters();
    // double true_hl = m_sheet.get_hinge_length(hid);
    const double ml = m_sheet.get_hinge_min_length(hid0);
    double hl = m_sheet.get_hinge_length(hid) - (shrink_left + shrink_right);
    v2d v0_m = v0_s - 0.5 * hl * ortho; 
    // hl = true_hl;
    // v0_s = v0;
    // v1_s = v1;
    SVGPath hole("none", "black", "stroke-width:" + std::to_string(m_settings.edgecuts_width_mm));
    assert(m_sheet.faces_in_same_patch(fid0, fid1));

    if (m_sheet.lt_depth(fid0, fid1)) {
        v2d pt = v0_m + (od + tw) * dir;
        hole << move_to(pt[0], pt[1], false) << SVG_SEP;
        pt = (tw + 0.5 * od + 0.5 * (hl - ml)) * ortho;
        hole << line_to(pt[0], pt[1], true) << SVG_SEP;
        pt = id * dir;
        hole << arc_to(0.5 * id, true, true, pt[0], pt[1]) << SVG_SEP;
        pt = -(2.0 * tw + od + (hl - ml)) * ortho;
        hole << line_to(pt[0], pt[1], true) << SVG_SEP;
        pt = -id * dir;
        hole << arc_to(0.5 * id, true, true, pt[0], pt[1]) << SVG_SEP;
        hole << close_path();
    }

    return hole;
}

SVGPath Layout::contour_to_svgpath(size_t pid) const {
    const auto& mvo = m_sheet.get_vertex_order(pid);

    SVGPath contour("none", "black", "stroke-width:" + std::to_string(m_settings.edgecuts_width_mm));

    auto[vt0, id0, vid0, fid0] = mvo[0];
    v2d pt0 = m_sheet.get_vertex(vid0, true);
    contour << move_to(pt0[0], pt0[1], false);
    
    size_t i = 1;

    while (i < mvo.size()) {
        auto[vt, id, vid, fid] = mvo[i];
        if (vt == VertexType::hinge) {
            if (m_sheet.is_half_hinge(id)) {
                // std::cout << "HALF\n";
                contour << half_halfhinge_to_svgpath_commands(mvo[i], mvo[i+1]) << SVG_SEP;
            } else {
                // std::cout << "FULL\n";
                contour << half_fullhinge_to_svgpath_commands(mvo[i], mvo[i+1]) << SVG_SEP;
            }
            i += 2;
        } else {
            v2d pt = m_sheet.get_vertex(vid, true);
            contour << line_to(pt[0], pt[1], false) << SVG_SEP;
            ++i;
        }
    }

    contour << close_path();

    return contour;
}

std::vector<std::vector<v2d>> Layout::vcc_plane(size_t pid) const {
    using namespace ClipperLib;

    Clipper vcc; Paths ps;
    for (const auto& fi : m_patch_fis.at(pid)) {
        std::vector<v2d> ivs = inset_verts(fi.vertices, m_settings.plane_inset_depth_mm);
        std::vector<v2d> centered_verts;
        auto center_vertex_pid = [&pid, this](const v2d& pt) { return center_vertex(pt, pid); };
        std::transform(ivs.begin(), ivs.end(), std::back_inserter(centered_verts), center_vertex_pid);
        vcc.AddPath(v2ds_to_path(m_settings, centered_verts), PolyType::ptSubject, true);
    }

    for (const auto& fi : m_patch_fis.at(pid)) {
        for (auto mod : fi.modules) {
            mod = center_module(mod, pid);
            auto mod_tr = module_transform(mod);

            for (auto koz : m_module_info.ko_zones) {
                if (std::holds_alternative<KeepOutPolygon>(koz)) {
                    auto ko = std::get<KeepOutPolygon>(koz);
                    std::vector<v2d> transformed_polygon;
                    for (auto pt : ko.polygon) {
                        auto [x, y] = apply_transform_mod(mod_tr, pt);
                        transformed_polygon.push_back(v2d(x, y));
                    }
                    vcc.AddPath(v2ds_to_path(m_settings, transformed_polygon), PolyType::ptClip, true);
                } else if (std::holds_alternative<KeepOutCircle>(koz)) {
                    // auto ko = std::get<KeepOutCircle>(koz);
                    // ClipperOffset co_ko; Paths ps_ko;
                    // co_ko.ArcTolerance = CLIPPER_ARC_TOLERANCE;
                    // co_ko.AddPath(Path({pd_to_ip(apply_transform(mod_tr, ko.position))}), JoinType::jtRound, EndType::etOpenRound);
                    // co_ko.Execute(ps_ko, CLIPPER_SCALING * 0.5 * ko.diameter);
                    // vcc.AddPaths(ps_ko, PolyType::ptClip, true);
                } else {
                    assert(false);
                }
            }
        }

        if (fi.has_connector) {
            auto mod = center_module(fi.connector, pid);
            auto mod_tr = module_transform(mod);
            // auto comp_tr = component_transform(mod, m_connector_info.comps.at(0));

            for (auto koz : m_connector_info.ko_zones) {
                if (std::holds_alternative<KeepOutPolygon>(koz)) {
                    auto ko = std::get<KeepOutPolygon>(koz);
                    std::vector<v2d> transformed_polygon;
                    for (auto pt : ko.polygon) {
                        auto [x, y] = apply_transform_mod(mod_tr, pt);
                        transformed_polygon.push_back(v2d(x, y));
                    }
                    vcc.AddPath(v2ds_to_path(m_settings, transformed_polygon), PolyType::ptClip, true);
                } else if (std::holds_alternative<KeepOutCircle>(koz)) {
                    // auto ko = std::get<KeepOutCircle>(koz);
                    // ClipperOffset co_ko; Paths ps_ko;
                    // co_ko.ArcTolerance = CLIPPER_ARC_TOLERANCE;
                    // co_ko.AddPath(Path({pd_to_ip(apply_transform(mod_tr, ko.position))}), JoinType::jtRound, EndType::etOpenRound);
                    // co_ko.Execute(ps_ko, CLIPPER_SCALING * 0.5 * ko.diameter);
                    // vcc.AddPaths(ps_ko, PolyType::ptClip, true);
                } else {
                    assert(false);
                }
            }
        }
    }
    
    vcc.Execute(ClipType::ctDifference, ps, PolyFillType::pftNonZero);    

    return paths_to_vv2ds(m_settings, ps);
}

// std::vector<std::vector<v2d>> Layout::gnd_plane(size_t pid, const std::vector<std::vector<v2d>>& traces) const {
//     using namespace ClipperLib;

//     auto ortho_proj = [](v2d p0, v2d p1, v2d q) {
//         return p0 + dot(q - p0, p1 - p0) / sqLength(p1 - p0) * (p1 - p0);
//     };

//     Clipper gnd; Paths ps;
//     for (const auto& fi : m_patch_fis.at(pid)) {
//         std::vector<v2d> ivs = inset_verts(fi.vertices, PLANE_INSET_DEPTH);
//         std::vector<v2d> centered_verts;
//         auto center_vertex_pid = [&pid, this](const v2d& pt) { return center_vertex(pt, pid); };
//         std::transform(ivs.begin(), ivs.end(), std::back_inserter(centered_verts), center_vertex_pid);
//         gnd.AddPath(v2ds_to_path(centered_verts), PolyType::ptSubject, true);
//     }

//     std::vector<std::vector<v2d>> io_traces;
//     for (const auto& fi : m_patch_fis.at(pid)) {
//         for (size_t i = 0; i < fi.vertices.size(); ++i) {
//             if (!fi.is_hinge_edge[i]) { continue; }
//             size_t n = (i + 1) % fi.vertices.size();
//             v2d p0 = center_vertex(fi.vertices[i], pid);
//             v2d p1 = center_vertex(fi.vertices[n], pid);
            
//             auto hio = get_hio(fi.fid, i);
//             auto [out, inp] = center_segment(hio, pid);
//             v2d out_proj = ortho_proj(p0, p1, out);
//             v2d inp_proj = ortho_proj(p0, p1, inp);
//             io_traces.push_back({out_proj, out});
//             io_traces.push_back({inp_proj, inp});
//         }

//         for (auto mod : fi.modules) {
//             mod = center_module(mod, pid);
//             auto mod_tr = module_transform(mod);

//             for (auto koz : m_module_info.ko_zones) {
//                 if (std::holds_alternative<KeepOutPolygon>(koz)) {
//                     // auto ko = std::get<KeepOutPolygon>(koz);
//                     // gnd.AddPath(pds_to_path(ko.polygon), PolyType::ptClip, true);
//                 } else if (std::holds_alternative<KeepOutCircle>(koz)) {
//                     auto ko = std::get<KeepOutCircle>(koz);
//                     ClipperOffset co_ko; Paths ps_ko;
//                     co_ko.ArcTolerance = CLIPPER_ARC_TOLERANCE;
//                     co_ko.AddPath(Path({pd_to_ip(apply_transform_mod(mod_tr, ko.position))}), JoinType::jtRound, EndType::etOpenRound);
//                     co_ko.Execute(ps_ko, CLIPPER_SCALING * 0.5 * ko.diameter);
//                     gnd.AddPaths(ps_ko, PolyType::ptClip, true);
//                 } else {
//                     assert(false);
//                 }
//             }
//         }

//         if (fi.has_connector) {
//             auto mod = center_module(fi.connector, pid);
//             auto mod_tr = module_transform(mod);

//             for (auto koz : m_connector_info.ko_zones) {
//                 if (std::holds_alternative<KeepOutPolygon>(koz)) {
//                     // auto ko = std::get<KeepOutPolygon>(koz);
//                     // gnd.AddPath(pds_to_path(ko.polygon), PolyType::ptClip, true);
//                 } else if (std::holds_alternative<KeepOutCircle>(koz)) {
//                     auto ko = std::get<KeepOutCircle>(koz);
//                     ClipperOffset co_ko; Paths ps_ko;
//                     co_ko.ArcTolerance = CLIPPER_ARC_TOLERANCE;
//                     co_ko.AddPath(Path({pd_to_ip(apply_transform_mod(mod_tr, ko.position))}), JoinType::jtRound, EndType::etOpenRound);
//                     co_ko.Execute(ps_ko, CLIPPER_SCALING * 0.5 * ko.diameter);
//                     gnd.AddPaths(ps_ko, PolyType::ptClip, true);
//                 } else {
//                     assert(false);
//                 }
//             }
//         }
//     }
    
//     // size_t tid = 0;
//     ClipperOffset co_trace; Paths ps_trace;
//     co_trace.ArcTolerance = CLIPPER_ARC_TOLERANCE;
//     for (auto trace : traces) {
//         co_trace.AddPath(v2ds_to_path(trace), JoinType::jtRound, EndType::etOpenRound);
//     }
//     co_trace.Execute(ps_trace, CLIPPER_SCALING * (0.5 * TRACE_WIDTH + TRACE_CLEARANCE));
//     gnd.AddPaths(ps_trace, PolyType::ptClip, true);

//     gnd.Execute(ClipType::ctDifference, ps, PolyFillType::pftNonZero);    

//     return paths_to_vv2ds(ps);
// }

// std::pair<std::vector<std::vector<v2d>>, std::vector<std::vector<v2d>>>
// Layout::gnd_plane_fid(size_t fid, bool& is_valid) const {
//     using namespace ClipperLib;
//     is_valid = true;

//     auto ortho_proj = [](v2d p0, v2d p1, v2d q) {
//         return p0 + dot(q - p0, p1 - p0) / sqLength(p1 - p0) * (p1 - p0);
//     };

//     auto ids = m_sheet.sfid_to_pfid(fid);
//     auto pid = ids.first; auto pfid = ids.second;
//     const auto& fi = m_patch_fis.at(pid).at(pfid);

//     double inset_rmv = TRACE_CONFLICT_FACTOR * TRACE_CLEARANCE;

//     Clipper gnd; Paths ps;
//     Clipper gnd_rmv; Paths ps_rmv;

//     std::vector<v2d> ivs = inset_verts(fi.vertices, PLANE_INSET_DEPTH);
//     std::vector<v2d> centered_verts;
//     auto center_vertex_pid = [&pid, this](const v2d& pt) { return center_vertex(pt, pid); };
//     std::transform(ivs.begin(), ivs.end(), std::back_inserter(centered_verts), center_vertex_pid);
//     gnd.AddPath(v2ds_to_path(centered_verts), PolyType::ptSubject, true);

//     std::vector<Via> vias;
//     std::vector<v2d> gnd_vias;
//     for (auto mod : fi.modules) {
//         mod = center_module(mod, pid);
//         auto mod_tr = module_transform(mod);
//         for (const auto& via : m_module_info.vias) {
//             v2d via_tr = apply_transform_mod(mod_tr, v2d(via.x, via.y));
//             Via v;
//             v.x = via_tr[0]; v.y = via_tr[1];
//             v.hd = via.hd; v.vd = via.vd;
//             vias.push_back(v);
//         }
//         // HUGE HACK, GND VIA IS LAST IN MODULE DESCRIPTION
//         Via gvia = m_module_info.vias.at(m_module_info.vias.size() - 1);
//         gnd_vias.push_back(apply_transform_mod(mod_tr, v2d(gvia.x, gvia.y)));

//         for (auto koz : m_module_info.ko_zones) {
//             if (std::holds_alternative<KeepOutPolygon>(koz)) {
//                 // auto ko = std::get<KeepOutPolygon>(koz);
//                 // gnd.AddPath(pds_to_path(ko.polygon), PolyType::ptClip, true);
//             } else if (std::holds_alternative<KeepOutCircle>(koz)) {
//                 auto ko = std::get<KeepOutCircle>(koz);
//                 ClipperOffset co_ko; Paths ps_ko; Paths ps_ko_rmv;
//                 co_ko.ArcTolerance = CLIPPER_ARC_TOLERANCE;
//                 co_ko.AddPath(Path({pd_to_ip(apply_transform_mod(mod_tr, ko.position))}), JoinType::jtRound, EndType::etOpenRound);
//                 co_ko.Execute(ps_ko, CLIPPER_SCALING * 0.5 * ko.diameter);
//                 gnd.AddPaths(ps_ko, PolyType::ptClip, true);
                
//                 co_ko.Execute(ps_ko_rmv, CLIPPER_SCALING * (0.5 * ko.diameter - inset_rmv));
//                 gnd_rmv.AddPaths(ps_ko_rmv, PolyType::ptSubject, true);
//             } else {
//                 assert(false);
//             }
//         }
//     }

//     if (fi.has_connector) {
//         auto mod = center_module(fi.connector, pid);
//         auto mod_tr = module_transform(mod);
//         for (const auto& via : m_connector_info.vias) {
//             v2d via_tr = apply_transform_mod(mod_tr, v2d(via.x, via.y));
//             Via v;
//             v.x = via_tr[0]; v.y = via_tr[1];
//             v.hd = via.hd; v.vd = via.vd;
//             vias.push_back(v);
//         }
//         // HUGE HACK, GND VIA IS LAST IN MODULE DESCRIPTION
//         Via gvia = m_connector_info.vias.at(m_connector_info.vias.size() - 1);
//         gnd_vias.push_back(apply_transform_mod(mod_tr, v2d(gvia.x, gvia.y)));

//         for (auto koz : m_connector_info.ko_zones) {
//             if (std::holds_alternative<KeepOutPolygon>(koz)) {
//                 // auto ko = std::get<KeepOutPolygon>(koz);
//                 // gnd.AddPath(pds_to_path(ko.polygon), PolyType::ptClip, true);
//             } else if (std::holds_alternative<KeepOutCircle>(koz)) {
//                 auto ko = std::get<KeepOutCircle>(koz);
//                 ClipperOffset co_ko; Paths ps_ko; Paths ps_ko_rmv;
//                 co_ko.ArcTolerance = CLIPPER_ARC_TOLERANCE;
//                 co_ko.AddPath(Path({pd_to_ip(apply_transform_mod(mod_tr, ko.position))}), JoinType::jtRound, EndType::etOpenRound);
//                 co_ko.Execute(ps_ko, CLIPPER_SCALING * 0.5 * ko.diameter);
//                 gnd.AddPaths(ps_ko, PolyType::ptClip, true);
                
//                 co_ko.Execute(ps_ko_rmv, CLIPPER_SCALING * (0.5 * ko.diameter - inset_rmv));
//                 gnd_rmv.AddPaths(ps_ko_rmv, PolyType::ptSubject, true);
//             } else {
//                 assert(false);
//             }
//         }
//     }

//     // GENERATE TRACE OFFSET
//     ClipperOffset co_trace; Paths ps_trace; Paths ps_trace_rmv;
//     co_trace.ArcTolerance = CLIPPER_ARC_TOLERANCE;
//     std::vector<std::vector<v2d>> traces;
//     for (auto s : fi.traces) {
//         s = center_segment(s, pid);
//         traces.push_back(generate_avoiding_trace_path(s, vias));
//     }
//     // WIDE HINGE TO CENTER
//     for (size_t i = 0; i < fi.vertices.size(); ++i) {
//         int hid = m_sheet.get_hid(fi.fid, i);
//         if (hid < 0) { continue; }
//         size_t n = (i + 1) % fi.vertices.size();
        
//         // bridge the gap between hinge data trace and hinge IO 
//         bool cond = m_sheet.is_half_hinge(fi.fid, i) && m_sheet.is_wide(fi.fid, i);
//         auto hio = cond ? fi.hinge_ios_wide[i] : fi.hinge_ios[i];
//         auto [out, inp] = center_segment(hio, pid);
//         v2d p0 = center_vertex(fi.vertices[i], pid);
//         v2d p1 = center_vertex(fi.vertices[n], pid);
//         v2d out_proj = ortho_proj(p0, p1, out);
//         v2d inp_proj = ortho_proj(p0, p1, inp);
//         traces.push_back({out_proj, out});
//         traces.push_back({inp_proj, inp});

//         // if wide half-hinge has centered IOs, connect wide IO with narrow IO
//         if (cond &&
//             std::find(m_halfhinge_center_fids.begin(),
//                         m_halfhinge_center_fids.end(), 
//                         fi.fid) != m_halfhinge_center_fids.end()) {
//             auto [out_n, inp_n] = center_segment(fi.hinge_ios[i], pid);
//             auto [out_w, inp_w] = center_segment(fi.hinge_ios_wide[i], pid);
//             traces.push_back({out_n, out_w});
//             traces.push_back({inp_n, inp_w});
//         }
//     }

//     for (auto trace : traces) {
//         co_trace.AddPath(v2ds_to_path(trace), JoinType::jtRound, EndType::etOpenRound);
//     }
//     co_trace.Execute(ps_trace, CLIPPER_SCALING * (0.5 * TRACE_WIDTH + TRACE_CLEARANCE));
//     gnd.AddPaths(ps_trace, PolyType::ptClip, true);
//     gnd.Execute(ClipType::ctDifference, ps, PolyFillType::pftNonZero); 

//     // TESTING FOR TRACE CONFLICTS
//     auto count_detours = [](const std::vector<Segment>& ss) {
//         size_t count = 0;
//         for (size_t i = 0; i < ss.size(); ++i) {
//             auto [i0, i1] = ss[i];
//             for (size_t j = i + 1; j < ss.size(); ++j) {
//                 auto [j0, j1] = ss[j];
//                 if (i0 == j0 || i0 == j1 || i1 == j0 || i1 == j1) { ++count; }
//             }
//         }
//         // number of segments sharing an endpoint
//         return count;
//     };
//     auto count_trace_conflicts = [](ClipperOffset& co, int num_traces, double inset) {
//         Clipper cl_count; Paths ps_count;
//         co.Execute(ps_count, CLIPPER_SCALING * (0.5 * TRACE_WIDTH + TRACE_CLEARANCE - inset));
//         cl_count.AddPaths(ps_count, PolyType::ptSubject, true);
//         cl_count.Execute(ClipType::ctUnion, ps_count, PolyFillType::pftNonZero);
//         SimplifyPolygons(ps_count, PolyFillType::pftNonZero);
//         return num_traces - static_cast<int>(ps_count.size());
//     };
//     auto trace_conflicts_inventory = [&count_trace_conflicts](ClipperOffset& co, int num_traces, const std::vector<double>& insets) {
//         std::vector<std::pair<double, int>> vals;
//         for (double inst : insets) {
//             vals.push_back({inst, count_trace_conflicts(co, num_traces, inst)});
//         }
//         return vals;
//     };


//     int total_separate_traces = fi.traces.size() - count_detours(fi.traces);
//     int num_trace_conflicts =
//         count_trace_conflicts(co_trace, total_separate_traces, 0.51 * TRACE_CLEARANCE);

//     size_t num_samples = 100;
//     double min_val = 0.51 * TRACE_CLEARANCE;
//     double max_val = TRACE_CLEARANCE; //+ 0.5 * TRACE_WIDTH;
//     std::vector<double> test_insets(num_samples);
//     double step = (max_val - min_val) / (num_samples - 1.0);
//     for (size_t k = 0; k < num_samples; ++k) {
//         test_insets[k] = min_val + k * step;
//     }
//     auto vals = trace_conflicts_inventory(co_trace, total_separate_traces, test_insets);
//     for (size_t k = 0; k < vals.size() - 1; ++k) {
//         size_t n = (k + 1) % vals.size();
//         // ci < ni
//         auto [ci, cc] = vals[k];
//         auto [ni, nc] = vals[n];
//         int diff = std::max(nc - cc, cc - nc);
//         // std::cout << cc << '\t' << ci << '\n';
//         // assert(diff <= 1);
//         if (diff > 0) {
//             is_valid = false;
//             std::cout << "[" << fid << "]\t";
//             std::cout << ci << '\t' << ni << '\n';
//             std::cout << diff << " pair(s) of traces w/ at most " << 2.0 * (TRACE_CLEARANCE - ni) << "mm in between\n";  
//             for (const auto& pt : fi.vertices) {
//                 std::cout << to_svg_coords(pid, pt) << '\t';
//             } std::cout << '\n';
//         }
//     }

//     if (num_trace_conflicts > 0) {
//         is_valid = false;
//         std::cout << "[" << fid << "]\t" << num_trace_conflicts << " TRACE CONFLICT(s)\t";
//         for (const auto& pt : fi.vertices) {
//             std::cout << to_svg_coords(pid, pt) << '\t';
//         } std::cout << '\n';
//     }

//     // TESTING FOR DISCONNECTED VIAS
//     // auto connected_components = [](const Paths& paths) {
//     //     size_t comps = 0;
//     //     for (const auto& p : paths) {
//     //         if (Orientation(p)) { ++comps; }
//     //     }
//     //     return comps;
//     // };

//     auto id_largest_conn_comp = [](const Paths& paths) {
//         double max_area = 0.0;
//         size_t arg_max = 0;
//         for (size_t id = 0; id < paths.size(); ++id) {
//             const auto& p = paths[id];
//             if (!Orientation(p)) { continue; }
//             double area = Area(p);
//             if (area > max_area) {
//                 max_area = area;
//                 arg_max = id;
//             }
//         }
//         return arg_max;
//     };

//     auto is_v2d_in_largest_cc = [id_largest_conn_comp](const Paths& paths, v2d pt) {
//         return PointInPolygon(v2d_to_ip(pt), paths[id_largest_conn_comp(paths)]);
//     };
    
//     // TESTS for disconnected GND VIAS
//     double test_value = PLANE_EROSION_FACTOR;
//     SimplifyPolygons(ps, PolyFillType::pftNonZero);
//     // size_t ccs = connected_components(ps);
//     ClipperOffset test_erode; Paths ps_erode;
//     test_erode.AddPaths(ps, JoinType::jtRound, EndType::etClosedPolygon);
//     test_erode.Execute(ps_erode, -0.5 * CLIPPER_SCALING * test_value);
//     ClipperOffset test_dilate; Paths ps_dilate;
//     test_dilate.AddPaths(ps_erode, JoinType::jtRound, EndType::etClosedPolygon);
//     test_dilate.Execute(ps_dilate, 0.5 * CLIPPER_SCALING * test_value);
//     // int delta_ccs = connected_components(ps_dilate) - ccs;

//     for (const auto& pt : gnd_vias) {
//         // std::cout << "VIA\t" << to_svg_coords(pid, pt, true) << '\n';
//         if (!is_v2d_in_largest_cc(ps_dilate, pt)) {
//             std::cout << "[" << fid << "]\tDISCONNECTED GND VIA\t";
//             std::cout << to_svg_coords(pid, pt, true);
//             std::cout << '\n';
//             is_valid = false;
//         }
//     }

//     return {paths_to_vv2ds(ps), paths_to_vv2ds(ps_dilate)};
// }
