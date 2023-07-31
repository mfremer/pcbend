#include "layout_helpers.hpp"

std::pair<double, double> rotate(double x, double y, double rad) {
    return std::make_pair(cos(rad) * x - sin(rad) * y, sin(rad) * x + cos(rad) * y);
}

// rotates then translates
std::pair<double, double> apply_transform_mod(const std::tuple<double, double, double>& tr, double x, double y) {
    auto [tx, ty, trad] = tr;
    auto [xx, yy] = rotate(x, y, trad);
    return std::make_pair(tx + xx, ty + yy);
}
std::pair<double, double> apply_transform_mod(const std::tuple<double, double, double>& tr, std::pair<double, double> pos) {
    auto [x, y] = pos;
    auto [tx, ty, trad] = tr;
    auto [xx, yy] = rotate(x, y, trad);
    return std::make_pair(tx + xx, ty + yy);
}
v2d apply_transform_mod(const std::tuple<double, double, double>& tr, v2d v) {
    auto [x, y] = apply_transform_mod(tr, v[0], v[1]);
    return v2d(x, y);
}

// x, y, rotation (rad)
std::tuple<double, double, double> module_transform(const std::tuple<v2d, v2d, v2d>& mod) {
    auto [p0, p1, p2] = mod;    // absolute
    double x = p0[0];
    double y = p0[1];
    v2d v = p1 - p0;
    double rot = std::atan2(v[1], v[0]);
    return std::make_tuple(x, y, rot);
}

std::tuple<double, double, double> component_transform(const std::tuple<v2d, v2d, v2d>& mod, const Component& comp) {
    auto [mx, my, mrad] = module_transform(mod);
    auto [cx, cy, cdeg] = comp.transform;   // relative to module origin
    auto [ccx, ccy] = rotate(cx, cy, mrad);
    return std::make_tuple(mx + ccx, my + ccy, mrad + M_PI / 180.0 * cdeg);
}

std::tuple<double, double, double> pad_transform(const std::tuple<v2d, v2d, v2d>& mod, const Component& comp, const Pad& pad) {
    // auto [mx, my, mrad] = module_transform(mod);
    auto [cx, cy, crad] = component_transform(mod, comp);
    auto [px, py] = rotate(pad.x, pad.y, crad);
    return std::make_tuple(cx + px, cy + py, crad);
}

ClipperLib::IntPoint v2d_to_ip(const Settings& stgs, v2d v) {
    return ClipperLib::IntPoint(stgs.clipper_scaling * v[0], stgs.clipper_scaling * v[1]);
}
ClipperLib::IntPoint pd_to_ip(const Settings& stgs, std::pair<double, double> v) {
    return ClipperLib::IntPoint(stgs.clipper_scaling * v.first, stgs.clipper_scaling * v.second);
}
v2d ip_to_v2d(const Settings& stgs, ClipperLib::IntPoint p) {
    return v2d(p.X / stgs.clipper_scaling, p.Y / stgs.clipper_scaling);
}
std::pair<double, double> ip_to_pd(const Settings& stgs, ClipperLib::IntPoint p) {
    return std::make_pair(p.X / stgs.clipper_scaling, p.Y / stgs.clipper_scaling);
}
std::vector<v2d> path_to_v2ds(const Settings& stgs, const ClipperLib::Path& p) {
    std::vector<v2d> vs(p.size());
    for (size_t i = 0; i < p.size(); ++i) {
        vs[i] = ip_to_v2d(stgs, p[i]);
    }
    return vs;
}
ClipperLib::Path v2ds_to_path(const Settings& stgs, const std::vector<v2d>& vs) {
    ClipperLib::Path p(vs.size());
    for (size_t i = 0; i < vs.size(); ++i) {
        p[i] = v2d_to_ip(stgs, vs[i]);
    }
    return p;
}
ClipperLib::Path pds_to_path(const Settings& stgs, const std::vector<std::pair<double, double>>& vs) {
    ClipperLib::Path p(vs.size());
    for (size_t i = 0; i < vs.size(); ++i) {
        p[i] = pd_to_ip(stgs, vs[i]);
    }
    return p;
}
ClipperLib::Paths vv2ds_to_paths(const Settings& stgs, const std::vector<std::vector<v2d>>& vvs) {
    ClipperLib::Paths ps(vvs.size());
    for (size_t i = 0; i < vvs.size(); ++i) {
        ps[i] = v2ds_to_path(stgs, vvs[i]);
    }
    return ps;
}
std::vector<std::vector<v2d>> paths_to_vv2ds(const Settings& stgs, const ClipperLib::Paths& ps) {
    std::vector<std::vector<v2d>> vvs(ps.size());
    for (size_t i = 0; i < ps.size(); ++i) {
        vvs[i] = path_to_v2ds(stgs, ps[i]);
    }
    return vvs;
}

std::vector<v2d> generate_arc(v2d p0, v2d p1, double arc_angle_rad, bool left_arc, size_t samples_per_tour) {
    double factor = left_arc ? -1.0 : 1.0;
    double chord_len = distance(p0, p1);
    double radius = 0.5 * chord_len / std::sin(0.5 * arc_angle_rad);

    v2d e = normalize(p1 - p0);
    auto [rx, ry] = rotate(e[0], e[1], factor * 0.5 * (M_PI - arc_angle_rad));
    v2d origin = p0 + radius * v2d(rx, ry);

    v2d v0 = p0 - origin;
    // v2d v1 = p1 - origin;
    double theta0 = std::atan2(v0[1], v0[0]);
    // double theta1 = std::atan2(v1[1], v1[0]);

    std::vector<v2d> pts;
    size_t samples = std::round(arc_angle_rad / (2.0 * M_PI) * samples_per_tour);
    for (size_t i = 0; i < samples; ++i) {
        double t = static_cast<double>(i + 1) / static_cast<double>(samples + 1);
        double theta = theta0 + factor * t * arc_angle_rad;
        pts.push_back(origin + radius * v2d(std::cos(theta), std::sin(theta)));
    }

    return pts;
}

std::vector<v2d> generate_arc(v2d p0, v2d p1, v2d ctr, bool short_arc, size_t samples_per_tour) {
    auto positive_modulo = [](double i, double n) { return fmod(fmod(i, n) + n, n); };
    auto are_direct = [](v2d v0, v2d v1) {
        v3d vv0 = v3d(v0, 0.0);
        v3d vv1 = v3d(v1, 0.0);
        return dot(v3d(0.0, 0.0, 1.0), cross(vv0, vv1)) > 0.0;
    };

    double radius = distance(p0, ctr);

    v2d v0 = p0 - ctr;
    v2d v1 = p1 - ctr;
    double theta0 = std::atan2(v0[1], v0[0]);
    double theta1 = std::atan2(v1[1], v1[0]);
    double diff = positive_modulo(theta1 - theta0, 2.0 * M_PI);
    double delta = short_arc ?
        std::min(diff, 2 * M_PI - diff) :
        std::max(diff, 2 * M_PI - diff);
    double factor = are_direct(v0, v1) ? 1.0 : -1.0;

    std::vector<v2d> pts;
    size_t samples = std::round(delta / (2.0 * M_PI) * samples_per_tour);
    for (size_t i = 0; i < samples; ++i) {
        double t = static_cast<double>(i + 1) / static_cast<double>(samples + 1);
        double theta = theta0 + factor * t * delta;
        pts.push_back(ctr + radius * v2d(std::cos(theta), std::sin(theta)));
    }

    return pts;
}

std::vector<std::vector<v2d>> generate_via_paths(
    const Settings& stgs,
    const Via& v, const std::tuple<double, double, double>& tr)
{
    using namespace ClipperLib;
    // outer ring (via trace)
    ClipperOffset co_out; Paths ps_out;
    co_out.ArcTolerance = stgs.clipper_arc_tolerance;
    co_out.AddPath(Path({v2d_to_ip(stgs, apply_transform_mod(tr, v2d(v.x, v.y)))}), JoinType::jtRound, EndType::etOpenRound);
    co_out.Execute(ps_out, stgs.clipper_scaling * 0.5 * v.vd);
    // inner ring (hole)
    ClipperOffset co_in; Paths ps_in;
    co_in.ArcTolerance = stgs.clipper_arc_tolerance;
    co_in.AddPath(Path({v2d_to_ip(stgs, apply_transform_mod(tr, v2d(v.x, v.y)))}), JoinType::jtRound, EndType::etOpenRound);
    co_in.Execute(ps_in, stgs.clipper_scaling * 0.5 * v.hd);
    // difference
    Clipper clip; Paths ps_via;
    clip.AddPaths(ps_out, PolyType::ptSubject, true);
    // clip.AddPaths(ps_in, PolyType::ptClip, true);
    clip.Execute(ClipType::ctDifference, ps_via, PolyFillType::pftNonZero);

    return paths_to_vv2ds(stgs, ps_via);
}

// ax^2 + bx + c = 0
// return.first == true iff both roots in [0, 1]
std::tuple<bool, double, double> solve_quadratic(double a, double b, double c) {
    double delta = b * b - 4.0 * a * c;
    if (delta <= 0) { return std::make_tuple(false, 0.0, 0.0); }
    double t1 = (-b - sqrt(delta)) / (2.0 * a);
    double t2 = (-b + sqrt(delta)) / (2.0 * a);

    if (t1 < 0.0 || t1 > 1.0 || t2 < 0.0 || t2 > 1.0) {
        return std::make_tuple(false, std::min(t1, t2), std::max(t1, t2));
    }

    return std::make_tuple(true, std::min(t1, t2), std::max(t1, t2));
}

std::tuple<bool, double, double> segment_circle_intersection(const Segment& s, v2d cpos, double crad) {
    auto [p0, p1] = s;
    v2d u = p1 - p0;
    return solve_quadratic(sqLength(u), 2.0 * dot(u, p0 - cpos), sqDistance(p0, cpos) - crad * crad);
}

std::vector<v2d> generate_avoiding_trace_path(
    const Settings& stgs,
    const Segment& s, const std::vector<Via>& vias)
{
    constexpr size_t samples_per_tour = 64;
    auto get_via_excl = [&stgs](const Via& v) {
        return 0.5 * v.vd + 0.5 * stgs.trace_width_mm + stgs.via_clearance_mm;
    };

    v2d p0 = s.first; v2d p1 = s.second;

    // t1, t2, via, is_via_in_left_half_plane
    std::vector<std::tuple<double, double, Via>> intersections;
    for (const auto& v : vias) {
        double via_excl = get_via_excl(v);
        v2d via_pos = v2d(v.x, v.y);
        auto [intersects, t1, t2] = segment_circle_intersection(s, via_pos, via_excl);
        if (intersects) { 
            // std::cout << "\tV " << via_pos << '\t' << intersects << ' ' << '\t' << t1 << '\t' << t2 << '\n'; 
            intersections.push_back(std::make_tuple(t1, t2, v)); 
        }
    }

    if (intersections.empty()) { return std::vector<v2d>({p0, p1}); }

    // sort them in order of occurrence along the trace segment
    auto compare_first = [](const std::tuple<double, double, Via>& a, const std::tuple<double, double, Via>& b) {
        return std::get<0>(a) < std::get<0>(b);
    };
    std::sort(intersections.begin(), intersections.end(), compare_first);

    std::vector<v2d> trace; trace.push_back(p0);
    // go through all intersections in order
    for (auto t : intersections) {
        auto [t1, t2, via] = t;
        v2d via_pos = v2d(via.x, via.y);
        // double via_excl = get_via_excl(via);

        // build svg path
        v2d c1 = p0 + t1 * (p1 - p0);
        v2d c2 = p0 + t2 * (p1 - p0);

        trace.push_back(c1);
        auto arc_pts = generate_arc(c1, c2, via_pos, true, samples_per_tour);
        trace.insert(trace.end(), arc_pts.begin(), arc_pts.end());
        trace.push_back(c2);
    }
    trace.push_back(p1);

    return trace;
}

std::vector<Via> generate_vias(const FaceInfo& fi, const ModuleInfo& mi, const ModuleInfo& ci) {
    std::vector<Via> vias;
    for (auto mod : fi.modules) {
        auto mod_tr = module_transform(mod);
        for (const auto& via : mi.vias) {
            v2d via_tr = apply_transform_mod(mod_tr, v2d(via.x, via.y));
            Via v;
            v.x = via_tr[0]; v.y = via_tr[1];
            v.hd = via.hd; v.vd = via.vd;
            vias.push_back(v);
        }
    }

    if (fi.has_connector) {
        auto mod_tr = module_transform(fi.connector);
        for (const auto& via : ci.vias) {
            v2d via_tr = apply_transform_mod(mod_tr, v2d(via.x, via.y));
            Via v;
            v.x = via_tr[0]; v.y = via_tr[1];
            v.hd = via.hd; v.vd = via.vd;
            vias.push_back(v);
        }
    }

    return vias;
}

std::vector<v2d> generate_gnd_vias(const FaceInfo& fi, const ModuleInfo& mi, const ModuleInfo& ci) {
    std::vector<v2d> gnd_vias;
    for (auto mod : fi.modules) {
        auto mod_tr = module_transform(mod);
        // HUGE HACK, GND VIA IS LAST IN MODULE DESCRIPTION
        Via gvia = mi.vias.at(mi.vias.size() - 1);
        gnd_vias.push_back(apply_transform_mod(mod_tr, v2d(gvia.x, gvia.y)));
    }

    if (fi.has_connector) {
        auto mod_tr = module_transform(fi.connector);
        // HUGE HACK, GND VIA IS LAST IN MODULE DESCRIPTION
        Via gvia = ci.vias.at(ci.vias.size() - 1);
        gnd_vias.push_back(apply_transform_mod(mod_tr, v2d(gvia.x, gvia.y)));
    }

    return gnd_vias;
}

// missing IO to hinge traces (+ wide traces)
std::vector<std::vector<v2d>> generate_traces(
    const Settings& stgs, const FaceInfo& fi)
{
    auto ortho_proj = [](v2d p0, v2d p1, v2d q) {
        return p0 + dot(q - p0, p1 - p0) / sqLength(p1 - p0) * (p1 - p0);
    };
    auto rem = [](int a, int b) {
        return (a % b + b) % b;
    };
    auto select_hio = [&rem, &fi](size_t i) {
        size_t p = static_cast<size_t>(rem(i - 1, fi.vertices.size()));
        v2d out = (fi.is_narrow[i].first && !fi.is_detour[p]) ? fi.hinge_ios[i].first : fi.hinge_ios_wide[i].first;
        v2d inp = (fi.is_narrow[i].second && !fi.is_detour[i]) ? fi.hinge_ios[i].second : fi.hinge_ios_wide[i].second; 
        return std::make_pair(out, inp);
    };

    ModuleInfo mi = parse_module_info(stgs.file_module);
    ModuleInfo ci = parse_module_info(stgs.file_connector);

    // Inter LED traces (including detours)
    std::vector<Via> vias = generate_vias(fi, mi, ci);
    std::vector<std::vector<v2d>> traces;
    for (auto s : fi.traces) {
        traces.push_back(generate_avoiding_trace_path(stgs, s, vias));
    }

    // Connect hinge data trace to LED data trace
    for (size_t i = 0; i < fi.vertices.size(); ++i) {
        if (!fi.is_hinge_edge[i]) { continue; }
        size_t n = (i + 1) % fi.vertices.size();
        
        // bridge the gap between hinge data trace and hinge IO 
        bool cond = fi.is_half_hinge_edge[i] && fi.is_deeper_than_neighbor[i];
        auto hio = fi.hinge_ios[i];
        if (cond) { hio = fi.hinge_ios_wide[i]; }
        auto [out, inp] = hio;
        // auto [out, inp] = cond ? fi.hinge_ios_wide[i] : fi.hinge_ios[i];
        v2d p0 = fi.vertices[i];
        v2d p1 = fi.vertices[n];
        v2d out_proj = ortho_proj(p0, p1, out);
        v2d inp_proj = ortho_proj(p0, p1, inp);
        traces.push_back({out_proj, out});
        traces.push_back({inp_proj, inp});
        
        // if wide half-hinge has centered IOs, connect wide IO with narrow IO
        size_t p = static_cast<size_t>(rem(i - 1, fi.vertices.size()));
        auto [is_narrow_out, is_narrow_inp] = fi.is_narrow[i];
        auto [is_detour_out, is_detour_inp] = std::make_pair(fi.is_detour[p], fi.is_detour[i]);
        if (cond) {
            // std::cout << "FID " << fi.fid << ' ' << i << '\t';
            // std::cout << "YES COND ";
            // std::cout << is_narrow_out << ' ' << is_narrow_inp << ' ' << is_detour_out << ' ' << is_detour_inp << '\n';
            auto [out_n, inp_n] = fi.hinge_ios[i];
            auto [out_w, inp_w] = fi.hinge_ios_wide[i];
            if (is_narrow_out && !fi.is_detour[p]) { 
                // std::cout << "NO ";
                traces.push_back({out_n, out_w});
            }
            if (is_narrow_inp && !fi.is_detour[i]) { 
                // std::cout << "NI ";
                traces.push_back({inp_n, inp_w});
            }
            // std::cout << '\n';
        } else {
            // std::cout << "NOT COND ";
            // std::cout << is_narrow_out << ' ' << is_narrow_inp << ' ' << is_detour_out << ' ' << is_detour_inp << '\n';
        }
    }

    return traces;
}

ClipperLib::Paths generate_gnd_plane(
    const Settings& stgs, const FaceInfo& fi)
{
    using namespace ClipperLib;
    ModuleInfo mi = parse_module_info(stgs.file_module);
    ModuleInfo ci = parse_module_info(stgs.file_connector);

    Clipper gnd; Paths ps;
    // Clipper gnd_rmv; Paths ps_rmv;

    std::vector<v2d> ivs = inset_verts(fi.vertices, stgs.plane_inset_depth_mm);
    gnd.AddPath(v2ds_to_path(stgs, ivs), PolyType::ptSubject, true);

    for (auto mod : fi.modules) {
        auto mod_tr = module_transform(mod);
        for (auto koz : mi.ko_zones) {
            if (std::holds_alternative<KeepOutPolygon>(koz)) { continue; }
            else if (std::holds_alternative<KeepOutCircle>(koz)) {
                auto ko = std::get<KeepOutCircle>(koz);
                ClipperOffset co_ko; Paths ps_ko; Paths ps_ko_rmv;
                co_ko.ArcTolerance = stgs.clipper_arc_tolerance;
                co_ko.AddPath(Path({pd_to_ip(stgs, apply_transform_mod(mod_tr, ko.position))}), JoinType::jtRound, EndType::etOpenRound);
                co_ko.Execute(ps_ko, stgs.clipper_scaling * 0.5 * ko.diameter);
                gnd.AddPaths(ps_ko, PolyType::ptClip, true);
            } else {
                assert(false);
            }
        }
    }

    if (fi.has_connector) {
        auto mod_tr = module_transform(fi.connector);
        for (auto koz : ci.ko_zones) {
            if (std::holds_alternative<KeepOutPolygon>(koz)) { continue; }
            else if (std::holds_alternative<KeepOutCircle>(koz)) {
                auto ko = std::get<KeepOutCircle>(koz);
                ClipperOffset co_ko; Paths ps_ko; Paths ps_ko_rmv;
                co_ko.ArcTolerance = stgs.clipper_arc_tolerance;
                co_ko.AddPath(Path({pd_to_ip(stgs, apply_transform_mod(mod_tr, ko.position))}), JoinType::jtRound, EndType::etOpenRound);
                co_ko.Execute(ps_ko, stgs.clipper_scaling * 0.5 * ko.diameter);
                gnd.AddPaths(ps_ko, PolyType::ptClip, true);
            } else {
                assert(false);
            }
        }
    }

    auto traces = generate_traces(stgs, fi);
    ClipperOffset co_traces; Paths ps_traces;
    co_traces.ArcTolerance = stgs.clipper_arc_tolerance;
    for (auto trace : traces) {
        co_traces.AddPath(v2ds_to_path(stgs, trace), JoinType::jtRound, EndType::etOpenRound);
    }
    co_traces.Execute(ps_traces, stgs.clipper_scaling * (0.5 * stgs.trace_width_mm + stgs.trace_clearance_mm));
    gnd.AddPaths(ps_traces, PolyType::ptClip, true);
    gnd.Execute(ClipType::ctDifference, ps, PolyFillType::pftNonZero); 

    return ps;
}