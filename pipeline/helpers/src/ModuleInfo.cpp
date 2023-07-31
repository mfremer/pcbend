#include "ModuleInfo.hpp"

#include "FaceInfo.hpp"

#include <fstream>
#include <iostream>

#include <cassert>

namespace {
// line should contain a space-separated list of doubles (nothing at the beginning or end)
std::vector<double> parse_doubles(const std::string& line) {
    std::vector<double> vals;
    char sep = ' ';

    size_t curr_id = 0;
    size_t sep_id = 0;
    while (sep_id != std::string::npos) {
        sep_id = line.find(sep, curr_id);
        if (sep_id == std::string::npos) {
            vals.push_back(std::stod(line.substr(curr_id)));
        } else {
            vals.push_back(std::stod(line.substr(curr_id, sep_id - curr_id)));
            curr_id = sep_id + 1;
        }
    }

    return vals;
}
}

ModuleInfo parse_module_info(const std::string& file_name) {
    std::ifstream file_in(file_name);
    if (!file_in.is_open()) {
        std::cerr << "[ERROR] Could not open file " << file_name << '\n';
        exit(EXIT_FAILURE);
    }

    ModuleInfo mi;

    int num_line = 0;
    // int num_tri = 0;
    std::string line;
    bool has_component = false;

    std::tuple<double, double, double> trans;
    std::vector<Pad> pads;
    std::pair<double, double> input;

    while (std::getline(file_in, line)) {
        ++num_line;
        has_component = false;

        std::vector<double> vals;
        char init_char = line.at(0);
        // std::cout << line << '\n';

        if (init_char == 'd') {
            std::vector<double> vals = ::parse_doubles(line.substr(2));
            assert(vals.size() == 2);
            mi.dims = std::make_pair(vals[0], vals[1]);
        } else if (init_char == 'i') {
            std::vector<double> vals = ::parse_doubles(line.substr(2));
            assert(vals.size() == 2);
            input = std::make_pair(vals[0], vals[1]);
        } else if (init_char == 'o') {
            std::vector<double> vals = ::parse_doubles(line.substr(2));
            assert(vals.size() == 2);
            mi.io = std::make_pair(input, std::make_pair(vals[0], vals[1]));
        } else if (init_char == 'v') {
            std::vector<double> vals = ::parse_doubles(line.substr(2));
            assert(vals.size() == 4);
            Via v = { vals[0], vals[1], vals[2], vals[3] };
            mi.vias.push_back(v);
        } else if (init_char == 'x') {
            std::vector<double> vals = ::parse_doubles(line.substr(2));
            assert(vals.size() == 3);
            trans = std::make_tuple(vals[0], vals[1], vals[2]);
        } else if (init_char == 'p') {
            std::vector<double> vals = ::parse_doubles(line.substr(2));
            Pad p = { vals[0], vals[1], vals[2], vals[3], 0.0 };
            if (vals.size() == 5) { p.r = vals[4]; }
            pads.push_back(p);
        } else if (init_char == 'l') {
            std::vector<double> vals = ::parse_doubles(line.substr(2));
            assert(vals.size() % 2 == 1);
            std::vector<std::pair<double, double>> pairs;
            for (size_t i = 1; i < vals.size() - 1; i += 2) {
                pairs.push_back(std::make_pair(vals[i], vals[i+1]));
            }
            mi.traces.push_back(pairs);
        } else if (init_char == 's') {
            std::vector<double> vals = ::parse_doubles(line.substr(2));
            assert(vals.size() == 4);
            mi.ss_marks.push_back(std::make_pair(std::make_pair(vals[0], vals[1]),
                                                 std::make_pair(vals[2], vals[3])));
        } else if (init_char == 'k') {
            std::vector<double> vals = ::parse_doubles(line.substr(2));
            assert(vals.size() % 2 == 0);
            auto polygon_from_doubles = [](const std::vector<double>& vs) {
                std::vector<std::pair<double, double>> ps(vs.size() / 2);
                for (size_t i = 0; i < ps.size(); ++i) {
                    ps[i] = std::make_pair(vs[2 * i], vs[2 * i + 1]);
                }
                KeepOutPolygon kop{ ps };
                return kop;
            };
            mi.ko_zones.push_back(polygon_from_doubles(vals));
        } else if (init_char == 'q') {
            std::vector<double> vals = ::parse_doubles(line.substr(2));
            assert(vals.size() == 3);
            KeepOutCircle koc{ std::make_pair(vals[1], vals[2]), vals[0] };
            mi.ko_zones.push_back(koc);
        } else if (init_char == 'e') {
            has_component = true;
        } else if (init_char == '#') {
            continue;
        } else {
            continue;
            // std::cerr << "File " << file_name << " contains badly formatted line:\n";
            // std::cerr << num_line << " >" << line << '\n';
            // exit(EXIT_FAILURE);
        }

        if (has_component) {
            Component c = { trans, pads };
            pads.clear();
            mi.comps.push_back(c);
        }
    }

    return mi;
}
