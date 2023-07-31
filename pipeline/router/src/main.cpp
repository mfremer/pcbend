#include "globals.hpp"
#include "extended_tour.hpp"

#include <FaceInfo.hpp>
#include <Sheet.hpp>

// #include <global_parameters.hpp>
#include <LibSL/LibSL.h>
#include <discorde_cpp.h>
#include <clipper.hpp>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

void process_face_info(const Settings& stgs, FaceInfo& fi,
                       double& min_max_space_with_no_inters,
                       double& min_max_space_with_no_inters_passing)
{
    // std::cout << "RPNR RVER FPNR FVER\n";
    // std::cout << "SFID " << fi.fid << '\t' << fi.run_pnr << ' ' << fi.run_verif << ' ' << fi.failed_pnr << ' ' << fi.failed_verif << '\n';
    double inset_mm = stgs.detour_inset_depth_mm;
    double trace_spacing_mm = stgs.router_min_trace_spacing_mm;
    
    if (!fi.run_pnr) { return; }
    Timer tm("[route_fi]");
    debug_info(fi);

    // router
    ExtendedTour extt(stgs, fi);
    double max_space_with_no_inters = 0.0;
    std::tuple<size_t, double, double> cost;
    size_t num_rects = fi.modules.size() + (fi.has_connector ? 1 : 0);
    if (num_rects < stgs.concorde_led_threshold) {
        std::cout << "EXHAUSTIVE APPROACH\n";
        cost = extt.solve_orientation_permutation_cuts_all(inset_mm,
                trace_spacing_mm, max_space_with_no_inters);  
    } else {
        std::cout << "TSP APPROACH\n";
        cost = extt.solve_orientation_permutation_cuts(inset_mm,
                trace_spacing_mm, max_space_with_no_inters);
    }
    auto [sol_inter, sol_len, sol_space] = cost;
    fi = extt.m_fi;

    char code = 'o';
    // if intersections or bad spacing, add to filter + debug info
    
    if (!extt.m_found_spacing_io || sol_inter != 0 || sol_space < trace_spacing_mm) {
        if (!extt.m_found_spacing_io) {
            code = 'h';
        } else if (sol_inter != 0) {
            code = 'i';
        } else {
            code = 's';
        }
        fi.run_pnr = true;
        fi.run_verif = false;
        fi.failed_pnr = true;
        fi.failed_verif = false;
    } else { // PASSED!
        fi.run_pnr = false;
        fi.run_verif = true;
        fi.failed_pnr = false;
        fi.failed_verif = false;
        // fi.pnr_runs_same_LEDs++;
        if (max_space_with_no_inters < min_max_space_with_no_inters_passing) {
            min_max_space_with_no_inters_passing = max_space_with_no_inters;
        }
    }
    if (max_space_with_no_inters < min_max_space_with_no_inters) {
        min_max_space_with_no_inters = max_space_with_no_inters;
    }

    ++fi.stats_overall[1];
    RouterRecord rec = {code, sol_inter, sol_len,
                        extt.m_io_spacing, sol_space,
                        extt.m_best_io_spacing,
                        max_space_with_no_inters,
                        extt.m_longest_unstable_subseq, tm.elapsed()};
    fi.stats_detailed.push_back(rec);
}

void process_file(
    const std::string& file_stg,
    const std::string& file_inp,
    const std::string& file_out)
{
    Settings stgs = read_settings(file_stg);
    std::vector<FaceInfo> fis = parse_face_info(file_inp, {});

    // min over all (filtered) faces of max_space_with_no_inters
    double min_max_space_with_no_inters = std::numeric_limits<double>::max();
    // min over passing (filtered) faces of max_space_with_no_inters
    double min_max_space_with_no_inters_passing = std::numeric_limits<double>::max();

    for (auto& fi : fis) {
        process_face_info(stgs, fi, min_max_space_with_no_inters, min_max_space_with_no_inters_passing);
    }

    write_face_info(file_out, fis);

    // debug info
    std::cout << "Failed FIDs\t";
    for (const auto& fi : fis) {
        if (fi.failed_pnr) { std::cout << fi.fid << ' '; }
    }
    std::cout << "Max spacing of all faces:\t" << min_max_space_with_no_inters << '\n';
    std::cout << "Max spacing of passing faces:\t" << min_max_space_with_no_inters_passing << '\n';
}

std::string interface() {
    return "./Router inp:<*.cfg> inp:<*.fis> (out:<*.fis>)\n";
}

void print_info(
	const std::string& file_stg,
	const std::string& file_fis,
	const std::string& file_out)
{
    std::cout << "ROUTER\n";
    std::cout << "file_stg\t" << file_stg << '\n';
    std::cout << "file_fis\t" << file_fis << '\n';
	std::cout << "file_out\t" << file_out << '\n';
}
                
int main(int argc, char* argv[]) {
    std::string file_stg{};
	std::string file_fis{};
	std::string file_out{};

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

    process_file(file_stg, file_fis, file_out);

    return EXIT_SUCCESS;
}
