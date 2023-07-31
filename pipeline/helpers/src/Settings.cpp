#include "Settings.hpp"

#include "parsing_helpers.hpp"

#include <iostream>
#include <fstream>

#include <cstdlib>

Settings read_settings(const std::string& file_settings) {
    Settings stgs;

    std::ifstream ifs(file_settings);
    if (!ifs.is_open()) {
        std::cerr << "[ERROR] Cannot open " << file_settings << '\n';
        exit(EXIT_FAILURE);
    }

    std::string line;
    size_t line_number = 0;
    while (std::getline(ifs, line)) {
        ++line_number;
        bool is_comment = line.at(0) == '#';
        if (is_comment) { continue; }
        
        size_t del_pos = line.find(' ');
        if (del_pos == std::string::npos) {
            std::cerr << "[ERROR] No \' \' delimiter in file " << file_settings << " on line " << line_number << '\n';
        }
        std::string field_name = line.substr(0, del_pos);
        line.erase(0, del_pos + 1);

        // please don't look
        if (field_name == "file_module") { // BEGIN STRING VALS
            stgs.file_module = line;
        } else if (field_name == "file_connector") { // END
            stgs.file_connector = line;
        } else if (field_name == "placer_iters") { // BEGIN SIZE_T VALS
            stgs.placer_iters = read<size_t>(line);
        } else if (field_name == "placer_tries") {
            stgs.placer_tries = read<size_t>(line);
        } else if (field_name == "concorde_led_threshold") { // END
            stgs.concorde_led_threshold = read<size_t>(line);
        } else if (field_name == "router_min_trace_spacing_mm") { // BEGIN DOUBLE VALS
            stgs.router_min_trace_spacing_mm = read<double>(line);
        } else if (field_name == "track_width_mm") {
            stgs.track_width_mm = read<double>(line);
        } else if (field_name == "inner_diameter_mm") {
            stgs.inner_diameter_mm = read<double>(line);
        } else if (field_name == "outer_diameter_mm") {
            stgs.outer_diameter_mm = read<double>(line);
        } else if (field_name == "heuristic") { 
            stgs.heuristic = line;
        } else if (field_name == "fab_margin") {
            stgs.fab_margin = read<double>(line);
        } else if (field_name == "num_patches_desired") {
            stgs.num_patches_desired = read<int>(line);
        } else if (field_name == "use_cli_interface") {
            stgs.use_cli_interface = read<bool>(line);
        } else if (field_name == "use_half_hinges") {
            stgs.use_half_hinges = read<bool>(line);
        } else if (field_name == "use_cli_interface_ledifier") {
            stgs.use_cli_interface_ledifier = read<bool>(line);
        } else if (field_name == "use_cli_interface_chamfer") {
            stgs.use_cli_interface_chamfer = read<bool>(line);
        } else if (field_name == "chamfer_mode") {
            stgs.chamfer_mode = read<size_t>(line);
        } else if (field_name == "trace_width_mm") {
            stgs.trace_width_mm = read<double>(line);
        } else if (field_name == "trace_clearance_mm") {
            stgs.trace_clearance_mm = read<double>(line);
        } else if (field_name == "plane_clearance_mm") {
            stgs.plane_clearance_mm = read<double>(line);
        } else if (field_name == "edge_clearance_mm") {
            stgs.edge_clearance_mm = read<double>(line);
        } else if (field_name == "via_clearance_mm") {
            stgs.via_clearance_mm = read<double>(line);
        } else if (field_name == "silkscreen_width_mm") {
            stgs.silkscreen_width_mm = read<double>(line);
        } else if (field_name == "edgecuts_width_mm") {
            stgs.edgecuts_width_mm = read<double>(line);
        } else if (field_name == "plane_width_mm") {
            stgs.plane_width_mm = read<double>(line);
        } else if (field_name == "float_eq_eps") {
            stgs.float_eq_eps = read<double>(line);
        } else if (field_name == "concorde_scaling") {
            stgs.concorde_scaling = read<double>(line);
        } else if (field_name == "clipper_scaling") {
            stgs.clipper_scaling = read<double>(line);
        } else if (field_name == "clipper_arc_tolerance") {
            stgs.clipper_arc_tolerance = read<double>(line);
        } else if (field_name == "plane_inset_depth_mm") {
            stgs.plane_inset_depth_mm = read<double>(line);
        } else if (field_name == "min_plane_io_clearance_mm") {
            stgs.min_plane_io_clearance_mm = read<double>(line);
        } else if (field_name == "led_clearance_mm") {
            stgs.led_clearance_mm = read<double>(line);
        } else if (field_name == "data_inset_depth_mm") {
            stgs.data_inset_depth_mm = read<double>(line);
        } else if (field_name == "data_inset_depth_wide_mm") {
            stgs.data_inset_depth_wide_mm = read<double>(line);
        } else if (field_name == "led_inset_depth_mm") {
            stgs.led_inset_depth_mm = read<double>(line);
        } else if (field_name == "led_inset_depth_wide_mm") {
            stgs.led_inset_depth_wide_mm = read<double>(line);
        } else if (field_name == "shrink_half_hinges_margin_mm") {
            stgs.shrink_half_hinges_margin_mm = read<double>(line);
        } else if (field_name == "detour_inset_depth_mm") {
            stgs.detour_inset_depth_mm = read<double>(line);
        } else if (field_name == "trace_conflict_factor") {
            stgs.trace_conflict_factor = read<double>(line);
        } else if (field_name == "plane_erosion_factor") {
            stgs.plane_erosion_factor = read<double>(line);
        } else if (field_name == "led_offset_x_mm") {
            stgs.led_offset_x_mm = read<double>(line);
        } else if (field_name == "led_offset_y_mm") {
            stgs.led_offset_y_mm = read<double>(line);
        } else if (field_name == "min_io_spacing_mm") {
            stgs.min_io_spacing_mm = read<double>(line);
        } else { // END
            std::cerr << "[ERROR] When reading file " << file_settings << " encountered unknown field " << field_name << " at line " << line_number << '\n';
            exit(EXIT_FAILURE);
        }
    }

    ifs.close();
    return stgs;
}

void dump_settings(const Settings& stgs) {
    char delim = ' ';
    std::cout << "file_module" << delim << stgs.file_module << '\n';
    std::cout << "file_connector" << delim << stgs.file_connector << '\n';
    std::cout << "placer_iters" << delim << stgs.placer_iters << '\n';
    std::cout << "placer_tries" << delim << stgs.placer_tries << '\n';
    std::cout << "router_min_trace_spacing_mm" << delim << stgs.router_min_trace_spacing_mm << '\n';
    std::cout << "track_width_mm" << delim << stgs.track_width_mm << '\n';
    std::cout << "inner_diameter_mm" << delim << stgs.inner_diameter_mm << '\n';
    std::cout << "outer_diameter_mm" << delim << stgs.outer_diameter_mm << '\n';
    
    std::cout << "fab_margin" << delim << stgs.fab_margin << '\n';
    std::cout << "num_patches_desired" << delim << stgs.num_patches_desired << '\n';
    std::cout << "use_cli_interface" << delim << stgs.use_cli_interface << '\n';
    std::cout << "use_half_hinges" << delim << stgs.use_half_hinges << '\n';

    std::cout << "use_cli_interface_chamfer" << delim << stgs.use_cli_interface_chamfer << '\n';
    std::cout << "chamfer_mode" << delim << stgs.chamfer_mode << '\n';

    std::cout << "trace_width_mm" << delim << stgs.trace_width_mm << '\n';
    std::cout << "trace_clearance_mm" << delim << stgs.trace_clearance_mm << '\n';
    std::cout << "plane_clearance_mm" << delim << stgs.plane_clearance_mm << '\n';
    std::cout << "edge_clearance_mm" << delim << stgs.edge_clearance_mm << '\n';
    std::cout << "via_clearance_mm" << delim << stgs.via_clearance_mm << '\n';
    std::cout << "silkscreen_width_mm" << delim << stgs.silkscreen_width_mm << '\n';
    std::cout << "edgecuts_width_mm" << delim << stgs.edgecuts_width_mm << '\n';
    std::cout << "plane_width_mm" << delim << stgs.plane_width_mm << '\n';
    std::cout << "float_eq_eps" << delim << stgs.float_eq_eps << '\n';
    std::cout << "concorde_scaling" << delim << stgs.concorde_scaling << '\n';
    std::cout << "concorde_led_threshold" << delim << stgs.concorde_led_threshold << '\n';
    std::cout << "clipper_scaling" << delim << stgs.clipper_scaling << '\n';
    std::cout << "clipper_arc_tolerance" << delim << stgs.clipper_arc_tolerance << '\n';
    std::cout << "plane_inset_depth_mm" << delim << stgs.plane_inset_depth_mm << '\n';
    std::cout << "min_plane_io_clearance_mm" << delim << stgs.min_plane_io_clearance_mm << '\n';
    std::cout << "led_clearance_mm" << delim << stgs.led_clearance_mm << '\n';
    std::cout << "data_inset_depth_mm" << delim << stgs.data_inset_depth_mm << '\n';
    std::cout << "data_inset_depth_wide_mm" << delim << stgs.data_inset_depth_wide_mm << '\n';
    std::cout << "led_inset_depth_mm" << delim << stgs.led_inset_depth_mm << '\n';
    std::cout << "led_inset_depth_wide_mm" << delim << stgs.led_inset_depth_wide_mm << '\n';
    std::cout << "shrink_half_hinges_margin_mm" << delim << stgs.shrink_half_hinges_margin_mm << '\n';
    std::cout << "detour_inset_depth_mm" << delim << stgs.detour_inset_depth_mm << '\n';
    std::cout << "trace_conflict_factor" << delim << stgs.trace_conflict_factor << '\n';
    std::cout << "plane_erosion_factor" << delim << stgs.plane_erosion_factor << '\n';
    std::cout << "led_offset_x_mm" << delim << stgs.led_offset_x_mm << '\n';
    std::cout << "led_offset_y_mm" << delim << stgs.led_offset_y_mm << '\n';
    std::cout << "min_io_spacing_mm" << delim << stgs.min_io_spacing_mm;// << '\n';
}