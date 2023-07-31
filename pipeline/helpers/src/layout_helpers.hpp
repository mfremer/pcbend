#pragma once

// #include <global_parameters.hpp>
#include <FaceInfo.hpp>
#include <ModuleInfo.hpp>

#include <utility>

#include <cmath>

#include <clipper.hpp>
#include <LibSL/LibSL.h>

// x, y, rotation (rad)
std::pair<double, double> rotate(double x, double y, double rad);

// rotates then translates
std::pair<double, double> apply_transform_mod(const std::tuple<double, double, double>& tr, double x, double y);
std::pair<double, double> apply_transform_mod(const std::tuple<double, double, double>& tr, std::pair<double, double> pos);
v2d apply_transform_mod(const std::tuple<double, double, double>& tr, v2d v);

std::tuple<double, double, double> module_transform(const std::tuple<v2d, v2d, v2d>& mod);
std::tuple<double, double, double> component_transform(const std::tuple<v2d, v2d, v2d>& mod, const Component& comp);
std::tuple<double, double, double> pad_transform(const std::tuple<v2d, v2d, v2d>& mod, const Component& comp, const Pad& pad);

// Clipper <-> LibSL conversion functions
ClipperLib::IntPoint v2d_to_ip(const Settings& stgs, v2d v);
ClipperLib::IntPoint pd_to_ip(const Settings& stgs, std::pair<double, double> v);
v2d ip_to_v2d(const Settings& stgs, ClipperLib::IntPoint p);
std::pair<double, double> ip_to_pd(const Settings& stgs, ClipperLib::IntPoint p);
std::vector<v2d> path_to_v2ds(const Settings& stgs, const ClipperLib::Path& p);
ClipperLib::Path v2ds_to_path(const Settings& stgs, const std::vector<v2d>& vs);
ClipperLib::Path pds_to_path(const Settings& stgs, const std::vector<std::pair<double, double>>& vs);
ClipperLib::Paths vv2ds_to_paths(const Settings& stgs, const std::vector<std::vector<v2d>>& vvs);
std::vector<std::vector<v2d>> paths_to_vv2ds(const Settings& stgs, const ClipperLib::Paths& ps);

// real layout functions
std::vector<v2d> generate_arc(v2d p0, v2d p1, double arc_angle_rad, bool left_arc, size_t samples_per_tour);
std::vector<v2d> generate_arc(v2d p0, v2d p1, v2d ctr, bool short_arc, size_t samples_per_tour);
std::vector<std::vector<v2d>> generate_via_paths(const Settings& stgs, const Via& v, const std::tuple<double, double, double>& tr);

// ax^2 + bx + c = 0
// return.first == true iff both roots in [0, 1]
std::tuple<bool, double, double> solve_quadratic(double a, double b, double c);
std::tuple<bool, double, double> segment_circle_intersection(const Segment& s, v2d cpos, double crad);

std::vector<v2d> generate_avoiding_trace_path(const Settings& stgs, const Segment& s, const std::vector<Via>& vias);
std::vector<Via> generate_vias(const FaceInfo& fi, const ModuleInfo& mi, const ModuleInfo& ci);
std::vector<v2d> generate_gnd_vias(const FaceInfo& fi, const ModuleInfo& mi, const ModuleInfo& ci);
// missing IO to hinge traces (+ wide traces)
std::vector<std::vector<v2d>> generate_traces(const Settings& stgs, const FaceInfo& fi);
ClipperLib::Paths generate_gnd_plane(const Settings& stgs, const FaceInfo& fi);