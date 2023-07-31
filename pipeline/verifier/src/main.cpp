// #include <global_parameters.hpp>
#include <layout_helpers.hpp>

#include <FaceInfo.hpp>
#include <ModuleInfo.hpp>
// #include <Sheet.hpp>
#include <svgwriter.hpp>

#include <clipper.hpp>
#include <LibSL/LibSL.h>

#include <iostream>
#include <numeric>

#include <cstdlib>
#include <ctime>

/* Computes the morphological opening of a plane (closed polygon with holes)
 * and a disk of radius 0.5 * narrowest gap, and performs the following checks:
 * - compute plane's connected component     (CC) ID containing all IOs
 * - ensure that all vias are in that same CC
 * Fails if:
 * - any IO or via is not in any CC
 * - not all IOs are in the same CC
 * - any via is not in the same CC as the IOs
 * Else returns true.
*/
bool is_plane_valid(
    const Settings& stgs, const FaceInfo& fi,
    const ClipperLib::Paths& plane, const std::vector<v2d>& vias,
    double narrowest_gap, double track_width,
    std::vector<v2d>& io_pts, ClipperLib::Paths& ps_op)
{
    using namespace ClipperLib;
    auto ortho_proj = [](v2d p0, v2d p1, v2d q) {
        return p0 + dot(q - p0, p1 - p0) / sqLength(p1 - p0) * (p1 - p0);
    };
    auto get_plane_ios = [&stgs, &ortho_proj](const std::pair<v2d, v2d>& edge, const std::pair<v2d, v2d>& hio, double tkw) {
        auto [p0, p1] = edge;
        auto [out, inp] = hio;
        v2d evec = normalize(p1 - p0);
        v2d evec_ortho = v2d(-evec[1], evec[0]); // towards inside
        v2d pout = ortho_proj(p0, p1, out);
        v2d pinp = ortho_proj(p0, p1, inp);

        // CAUTION: depends on the dimensions and position of data / GND / VCC
        double dist_data_plane = 0.5 * (tkw + stgs.trace_clearance_mm) + stgs.edge_clearance_mm;
        v2d pinp_plane = pinp - 0.5 * dist_data_plane * evec + 1.10 * stgs.plane_inset_depth_mm * evec_ortho;
        v2d pout_plane = pout + 0.5 * dist_data_plane * evec + 1.10 * stgs.plane_inset_depth_mm * evec_ortho;
        return std::make_pair(pout_plane, pinp_plane);
    };
    auto get_io_pts = [&get_plane_ios](const FaceInfo& fi, double tkw) {
        std::vector<v2d> pts;
        for (size_t i = 0; i < fi.vertices.size(); ++i) {
            if (!fi.is_hinge_edge[i]) { continue; }
            size_t n = (i + 1) % fi.vertices.size();

            auto hios = fi.hinge_ios[i];
            if (fi.is_half_hinge_edge[i] && fi.is_deeper_than_neighbor[i]) {
                hios = fi.hinge_ios_wide[i];
            }
            auto [out, inp] = get_plane_ios({fi.vertices[i], fi.vertices[n]}, hios, tkw);
            pts.push_back(out); pts.push_back(inp);
        }
        return pts;
    };
    auto which_cc_is_pt_in = [&stgs](const ClipperLib::Paths& ps, v2d pt) {
        for (size_t i = 0; i < ps.size(); ++i) {
            if (PointInPolygon(v2d_to_ip(stgs, pt), ps[i])) {
                return static_cast<int>(i);
            }
        }
        return -1;
    };

    // Morphological opening with Clipper
    Paths plane_test = plane;
    SimplifyPolygons(plane_test, PolyFillType::pftNonZero);
    // Erode
    ClipperOffset co_erode; Paths ps_erode;
    co_erode.ArcTolerance = stgs.clipper_arc_tolerance;
    co_erode.AddPaths(plane_test, JoinType::jtRound, EndType::etClosedPolygon);
    co_erode.Execute(ps_erode, -0.5 * stgs.clipper_scaling * narrowest_gap);
    // Dilate
    ClipperOffset co_dilate; Paths ps_dilate;
    co_dilate.ArcTolerance = stgs.clipper_arc_tolerance;
    co_dilate.AddPaths(ps_erode, JoinType::jtRound, EndType::etClosedPolygon);
    co_dilate.Execute(ps_dilate, 0.5 * stgs.clipper_scaling * narrowest_gap);
    SimplifyPolygons(ps_dilate, PolyFillType::pftNonZero);
    ps_op = ps_dilate;
    // std::cout << "Morphological opening #CCs " << ps_dilate.size() << '\n';

    // compute CC id of IOs
    // (should not use IOs but GND/VCC plane entries)
    io_pts = get_io_pts(fi, track_width);
    int prev_cc_id = -1;
    int cc_id = -1;
    for (auto io_pt : io_pts) {
        cc_id = which_cc_is_pt_in(ps_dilate, io_pt);
        // std::cout << io_pt << '\t' << cc_id << '\n';
        if (cc_id < 0) { 
            std::cout << "IO point not in plane\t" << io_pt << '\n';
            return false; 
        } // io not in ps 
        
        if (prev_cc_id < 0) { prev_cc_id = cc_id; } // init prev
        else if (cc_id != prev_cc_id) { 
            std::cout << "IO points in different CCs\t" << io_pt << '\n';
            return false;
        } // different ccs, issue
        else { prev_cc_id = cc_id; } // same ccs, OK        
    }

    // check that all vias are in the same CC as IOs
    for (auto pt : vias) {
        if (which_cc_is_pt_in(ps_dilate, pt) != cc_id) {
            std::cout << "Via not in same CC as IOs\t" << pt << '\n';
            return false;
        }
    }
    return true;
}

bool are_traces_valid(
    const Settings& stgs, const FaceInfo& fi,
    const std::vector<std::vector<v2d>>& traces)
{
    using namespace ClipperLib;
    // number of segments sharing an endpoint
    auto count_detours = [](const std::vector<Segment>& ss) {
        size_t count = 0;
        for (size_t i = 0; i < ss.size(); ++i) {
            auto [i0, i1] = ss[i];
            for (size_t j = i + 1; j < ss.size(); ++j) {
                auto [j0, j1] = ss[j];
                if (i0 == j0 || i0 == j1 || i1 == j0 || i1 == j1) { ++count; }
            }
        }
        return count;
    };
    auto dilate_traces = [&stgs](const std::vector<std::vector<v2d>>& traces, double amount) {
        ClipperOffset co_traces; Paths ps_traces;
        co_traces.ArcTolerance = stgs.clipper_arc_tolerance;
        for (const auto& trace : traces) {
            co_traces.AddPath(v2ds_to_path(stgs, trace), JoinType::jtRound, EndType::etOpenRound);
        }
        co_traces.Execute(ps_traces, stgs.clipper_scaling * amount);
        SimplifyPolygons(ps_traces, PolyFillType::pftNonZero);
        return ps_traces;
    };
    auto count_conflicts = [&dilate_traces](const std::vector<std::vector<v2d>>& traces, double dilation_mm, int num_traces) {
        return num_traces - static_cast<int>(dilate_traces(traces, dilation_mm).size());
    };
    int num_traces = fi.traces.size() - count_detours(fi.traces);
    auto count_conflicts_unary = [&count_conflicts, &traces, &num_traces](double factor) {
        return count_conflicts(traces, factor, num_traces);
    };
    auto gen_samples = [](double min, double max, double step) {
        size_t num_samples = std::round((max - min) / step) + 1;
        std::vector<double> factor_samples(num_samples);
        for (size_t i = 0; i < num_samples; ++i) {
            factor_samples[i] = min + i * step;
        }
        return factor_samples;
    };
    
    std::vector<double> factor_samples = gen_samples(0.01, 0.5 * stgs.trace_width_mm + 0.5 * stgs.trace_clearance_mm, 0.01);
    std::vector<int> conflicts(factor_samples.size());
    std::transform(factor_samples.begin(), factor_samples.end(), conflicts.begin(), count_conflicts_unary);
    // int sum_conflicts = std::accumulate(conflicts.begin(), conflicts.end(), 0);

    int first_conflict = -1;
    for (size_t i = 0; i < conflicts.size(); ++i) {
        if (conflicts[i] != 0) { first_conflict = i; break; }
    }

    if (first_conflict >= 0) { 
        // std::cout << "F "; for (auto fs : factor_samples) { std::cout << fs << '\t'; } std::cout << '\n';
        // std::cout << "C "; for (auto cs : conflicts) { std::cout << cs << '\t'; } std::cout << '\n';
        // std::cout << "At most " << TRACE_WIDTH - factor_samples[first_conflict] << " mm between traces.\n";
        return false;
    }

    return true;
}

void process_face_info(
    const Settings& stgs, FaceInfo& fi,
    size_t& num_plane_conflicts, size_t& num_trace_conflicts, size_t& num_faces_conflicts,
    const std::string& dir_debug)
{
    ModuleInfo mi = parse_module_info(stgs.file_module);
    ModuleInfo ci = parse_module_info(stgs.file_connector);

    if (!fi.run_verif) { return; }
    Timer tm("[verif_fi]");
    debug_info(fi);

    double tkw = stgs.track_width_mm;
    double id = stgs.inner_diameter_mm;
    double od = stgs.outer_diameter_mm;

    // auto vias = generate_vias(fi, mi, ci);
    auto gnd_vias = generate_gnd_vias(fi, mi, ci);
    auto traces = generate_traces(stgs, fi);
    auto plane = generate_gnd_plane(stgs, fi);
    
    auto ctr_v = [](const FaceInfo& fi, v2d v) {
        return v - fi.bbox().minCorner();
    };
    auto ctr_vv = [&ctr_v](const FaceInfo&fi, const std::vector<v2d>& vv) {
        std::vector<v2d> res(vv.size());
        std::transform(vv.begin(), vv.end(), res.begin(), [&](v2d v){ return ctr_v(fi, v); });
        return res;
    };
    auto ctr_vvv = [&ctr_vv](const FaceInfo&fi, const std::vector<std::vector<v2d>>& vvv) {
        std::vector<std::vector<v2d>> res(vvv.size());
        std::transform(vvv.begin(), vvv.end(), res.begin(), [&](const std::vector<v2d>& vv){ return ctr_vv(fi, vv); });
        return res;
    };
    auto pad_with_zeros = [](const std::string& str, size_t num_zeros) {
        return std::string(num_zeros - std::min(num_zeros, str.length()), '0') + str;
    };

    std::vector<v2d> io_pts; ClipperLib::Paths ps_op;
    bool ipv = is_plane_valid(stgs, fi, plane, gnd_vias, 0.90 * stgs.trace_width_mm, tkw, io_pts, ps_op);
    bool atv = are_traces_valid(stgs, fi, traces);

    // // START DEBUG
    // int rnd = rand();
    // std::cout << "DEBUG CODE: " << rnd << '\n';
    // v2d extent = fi.bbox().extent();
    // SVGFile failed_svg("test/" + pad_with_zeros(std::to_string(fi.fid), 3) + "_" + std::to_string(rnd) + ".svg", extent[0], extent[1], true);
    
    // // FACE CONTOUR
    // size_t numv = fi.vertices.size();
    // v2d ctr_last = ctr_v(fi, fi.vertices[numv-1]);
    // SVGPath contour("none", "black", "stroke-width:" + std::to_string(EDGECUTS_WIDTH)
    //     + ";stroke-linecap:round");
    // contour << move_to(ctr_last[0], ctr_last[1], false) << SVG_SEP;
    // for (size_t i = 0; i < fi.vertices.size(); ++i) {
    //     v2d ctr_vert = ctr_v(fi, fi.vertices[i]);
    //     contour << line_to(ctr_vert[0], ctr_vert[1], false) << SVG_SEP;
    // }
    // contour << close_path();
    // failed_svg.add_geometry_element(contour);

    // SVGPath plane_path("green", "green", "stroke-width:" + std::to_string(PLANE_WIDTH));
    // plane_path.from_v2ds(ctr_vvv(fi, paths_to_vv2ds(plane)));
    // failed_svg.add_geometry_element(plane_path);
    
    // SVGPath op_path("blue", "blue", "stroke-width:" + std::to_string(PLANE_WIDTH) + ";opacity:0.3");
    // op_path.from_v2ds(ctr_vvv(fi, paths_to_vv2ds(ps_op)));
    // failed_svg.add_geometry_element(op_path);

    // for (size_t i = 0; i < gnd_vias.size(); ++i) {
    //     SVGCircle circ("none", "red", "stroke-width:" + std::to_string(TRACE_WIDTH));
    //     auto gv = ctr_v(fi, gnd_vias[i]);
    //     circ.set_params(gv[0], gv[1], 0.0, 0.2);
    //     failed_svg.add_geometry_element(circ);
    // }
    // for (size_t i = 0; i < io_pts.size(); ++i) {
    //     SVGCircle circ("none", "red", "stroke-width:" + std::to_string(TRACE_WIDTH));
    //     auto gv = ctr_v(fi, io_pts[i]);
    //     circ.set_params(gv[0], gv[1], 0.0, 0.2);
    //     failed_svg.add_geometry_element(circ);
    // }

    // SVGPath trs("none", "black", "stroke-width:" + std::to_string(TRACE_WIDTH) +
    //                                             ";stroke-linecap:round;stroke-linejoin:round");
    // trs.from_v2ds(ctr_vvv(fi, traces));
    // failed_svg.add_geometry_element(trs);
    // failed_svg.to_file();
    // // END DEBUG

    fi.failed_pnr = false;
    fi.run_verif = false;
    char code = 'V';
    if (ipv && atv) {
        fi.failed_verif = false;
        std::cout << "SFID " << fi.fid << '\t' << "PASSED\n";
        code = 'o';
    } else {
        if (!dir_debug.empty()) {
            // START DEBUG
            int rnd = rand();
            std::cout << "DEBUG CODE: " << rnd << '\n';
            v2d extent = fi.bbox().extent();
            SVGFile failed_svg(dir_debug + pad_with_zeros(std::to_string(fi.fid), 3) + "_" + std::to_string(rnd) + ".svg", extent[0], extent[1], true);
            
            // FACE CONTOUR
            size_t numv = fi.vertices.size();
            v2d ctr_last = ctr_v(fi, fi.vertices[numv-1]);
            SVGPath contour("none", "black", "stroke-width:" + std::to_string(stgs.edgecuts_width_mm)
                + ";stroke-linecap:round");
            contour << move_to(ctr_last[0], ctr_last[1], false) << SVG_SEP;
            for (size_t i = 0; i < fi.vertices.size(); ++i) {
                v2d ctr_vert = ctr_v(fi, fi.vertices[i]);
                contour << line_to(ctr_vert[0], ctr_vert[1], false) << SVG_SEP;
            }
            contour << close_path();
            failed_svg.add_geometry_element(contour);

            SVGPath plane_path("green", "green", "stroke-width:" + std::to_string(stgs.plane_width_mm));
            plane_path.from_v2ds(ctr_vvv(fi, paths_to_vv2ds(stgs, plane)));
            failed_svg.add_geometry_element(plane_path);
            
            SVGPath op_path("blue", "blue", "stroke-width:" + std::to_string(stgs.plane_width_mm) + ";opacity:0.3");
            op_path.from_v2ds(ctr_vvv(fi, paths_to_vv2ds(stgs, ps_op)));
            failed_svg.add_geometry_element(op_path);

            for (size_t i = 0; i < gnd_vias.size(); ++i) {
                SVGCircle circ("none", "red", "stroke-width:" + std::to_string(stgs.trace_width_mm));
                auto gv = ctr_v(fi, gnd_vias[i]);
                circ.set_params(gv[0], gv[1], 0.0, 0.2);
                failed_svg.add_geometry_element(circ);
            }
            for (size_t i = 0; i < io_pts.size(); ++i) {
                SVGCircle circ("none", "red", "stroke-width:" + std::to_string(stgs.trace_width_mm));
                auto gv = ctr_v(fi, io_pts[i]);
                circ.set_params(gv[0], gv[1], 0.0, 0.2);
                failed_svg.add_geometry_element(circ);
            }

            SVGPath trs("none", "black", "stroke-width:" + std::to_string(stgs.trace_width_mm) +
                                                        ";stroke-linecap:round;stroke-linejoin:round");
            trs.from_v2ds(ctr_vvv(fi, traces));
            failed_svg.add_geometry_element(trs);
            failed_svg.to_file();
            // END DEBUG
        }

        fi.failed_verif = true;
        fi.run_pnr = true;
        ++num_faces_conflicts;
        if (!ipv) {
            ++num_plane_conflicts;
            std::cout << "SFID " << fi.fid << '\t' << "FAILED (plane not valid)\n";
            code = 'p';
        }
        if (!atv) { 
            ++num_trace_conflicts;
            std::cout << "SFID " << fi.fid << '\t' << "FAILED (traces not valid)\n";
            code = 't';
        }
        if (!ipv && !atv) { code = 'b'; }
        // std::cout << "Face INS";
        // for (auto pt : fi.vertices) { std::cout << pt << ' '; } std::cout << '\n';
        std::cout << "Face SVG\t";
        for (auto pt : fi.vertices_svg_pos) { std::cout << pt << ' '; } std::cout << '\n';
    }

    ++fi.stats_overall[2];
    VerifierRecord rec = {code, tm.elapsed()};
    fi.stats_detailed.push_back(rec);
}

std::string interface() {
    return "./Verifier inp:<*.cfg> inp:<*.fis> (out:<*.fis>)\n";
}

void print_info(
	const std::string& file_stg,
	const std::string& file_fis,
	const std::string& file_out)
{
    std::cout << "VERIFIER\n";
    std::cout << "file_stg\t" << file_stg << '\n';
    std::cout << "file_fis\t" << file_fis << '\n';
	std::cout << "file_out\t" << file_out << '\n';
}

int main(int argc, char* argv[]) {
    srand(time(NULL));

    std::string file_stg{};
	std::string file_fis{};
	std::string file_out{};
    std::string dir_debug{};

    // Input handling
	if (argc == 3) {
		file_stg = argv[1];
		file_fis = argv[2];
		file_out = file_fis;
	} else if (argc == 4) {
		file_stg = argv[1];
		file_fis = argv[2];
		file_out = argv[3];
	} else {
		std::cerr << "[ERROR] Wrong number of arguments\n";
		std::cerr << interface();
		exit(EXIT_FAILURE);
	}
	print_info(file_stg, file_fis, file_out);

    Settings stgs = read_settings(file_stg);

    auto fis = parse_face_info(file_fis);
    size_t num_trace_conflicts = 0;
    size_t num_plane_conflicts = 0;
    size_t num_faces_conflicts = 0;
    for (auto& fi : fis) {
        process_face_info(stgs, fi, num_plane_conflicts, num_trace_conflicts, num_faces_conflicts, dir_debug);   
    }

    std::cout << "VERIFIER SUMMARY: " << num_trace_conflicts << " trace conflicts, ";
    std::cout << num_plane_conflicts << " plane conflicts, faces w/ conflicts: " << num_faces_conflicts << '\n';
    write_face_info(file_fis, fis);

    return EXIT_SUCCESS;
}