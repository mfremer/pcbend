#pragma once

#include <string>

typedef struct {
    // Pipeline settings
    std::string file_module;
    std::string file_connector;
    size_t placer_iters = 2;
    size_t placer_tries = 1;
    double router_min_trace_spacing_mm = 0.2;
    // Hinge parameters
    double track_width_mm = 1.7;
    double inner_diameter_mm = 1.0;
    double outer_diameter_mm = 1.0;
    // UNFOLDER SETTINGS
    std::string heuristic;
    double fab_margin = 0.5;
    size_t num_patches_desired = 1;
    bool use_cli_interface = true;
    bool use_half_hinges = true;
    // LEDIFIER SETTINGS
    bool use_cli_interface_ledifier = true;
    // CHAMFERER SETTINGS
    bool use_cli_interface_chamfer = true;
    size_t chamfer_mode = 0;
    // Fabrication settings
    double trace_width_mm = 0.2;
    double trace_clearance_mm = 0.2;
    double plane_clearance_mm = 0.2;
    double edge_clearance_mm = 0.25;
    double via_clearance_mm = 0.2;
    double silkscreen_width_mm = 0.15;
    double edgecuts_width_mm = 0.1;
    double plane_width_mm = 1.0e-6;
    // Internal settings
    double float_eq_eps = 1.0e-2;
    double concorde_scaling = 1.0e3;
    size_t concorde_led_threshold = 7;
    double clipper_scaling = 1.0e6;
    double clipper_arc_tolerance = 1.0e3; // clipper_scaling * 1.0e-3
    double plane_inset_depth_mm = 0.25;
    double min_plane_io_clearance_mm = 0.2;
    double led_clearance_mm = 0.4;
    double data_inset_depth_mm = 0.55; // plane_inset_depth + min_plane_io_clearance + 0.5 * trace_width
    double data_inset_depth_wide_mm = 0.75; // data_inset_depth + min_plane)io_clearance
    double led_inset_depth_mm = 0.95; // data_inset_depth + led_clearance
    double led_inset_depth_wide_mm = 1.15; // data_inset_depth_wide + led_clearance 
    double shrink_half_hinges_margin_mm = 0.35; // 0.5 * trace_width + edge_clearance
    double detour_inset_depth_mm = 0.35; // edge_clearance + 0.5 * trace_width
    double trace_conflict_factor = 0.6;
    double plane_erosion_factor = 0.2;
    double led_offset_x_mm = 0.2;
    double led_offset_y_mm = 0.2;
    double min_io_spacing_mm = 1.2;
} Settings;

Settings read_settings(const std::string& file_settings);
void dump_settings(const Settings& stgs);
