#include "FaceInfo.hpp"

#include "parsing_helpers.hpp"
// #include "global_parameters.hpp"
// #include "Settings.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numeric>
#include <string>

// namespace {
// // line should contain a space-separated list of doubles (nothing at the beginning or end)
// std::vector<double> parse_doubles(const std::string& line) {
//     std::vector<double> vals;
//     char sep = ' ';

//     size_t curr_id = 0;
//     size_t sep_id = 0;
//     while (sep_id != std::string::npos) {
//         sep_id = line.find(sep, curr_id);
//         if (sep_id == std::string::npos) {
//             vals.push_back(std::stod(line.substr(curr_id)));
//         } else {
//             vals.push_back(std::stod(line.substr(curr_id, sep_id - curr_id)));
//             curr_id = sep_id + 1;
//         }
//     }

//     return vals;
// }
// }

HistoryRecord parse_record(std::string str) {
	char init_char = str[0];
	str = str.substr(2);
	HistoryRecord rec;
	switch (init_char) {
		case PLACER_CODE:
			rec = string_to_tuple<size_t, double>(str);
			break;
		case ROUTER_CODE:
			rec = string_to_tuple<char, size_t, double, double, double, double, double, int, double>(str);
			break;
		case VERIFIER_CODE:
			rec = string_to_tuple<char, double>(str);
			break;
		default:
			std::cerr << "[ERROR] parse_record: unhandled record code: " << init_char << "\n";
			exit(EXIT_FAILURE);
	}
	return rec;
}

std::string write_record(const HistoryRecord& rec) {
	std::ostringstream rec_os;

	if (std::holds_alternative<PlacerRecord>(rec)) {
		auto [v1, v2] = std::get<PlacerRecord>(rec);
		rec_os << PLACER_CODE << ' ' << v1 << ' ' << v2;
	} else if (std::holds_alternative<RouterRecord>(rec)) {
		auto [v1, v2, v3, v4, v5, v6, v7, v8, v9] = std::get<RouterRecord>(rec);
		rec_os << ROUTER_CODE << ' '  << v1 << ' ' << v2 << ' ' << v3 << ' ' << v4 << ' ' << v5 << ' ' << v6 << ' ' << v7 << ' ' << v8 << ' ' << v9;
	} else if (std::holds_alternative<VerifierRecord>(rec)) {
		auto [v1, v2] = std::get<VerifierRecord>(rec);
		rec_os << VERIFIER_CODE << ' '  << v1 << ' ' << v2;
	} else {
		std::cerr << "[ERROR] parse_record: unhandled record type\n";
		exit(EXIT_FAILURE);
	}

	return rec_os.str();
}

std::vector<HistoryRecord> parse_records(std::string line) {
	std::string codes{ PLACER_CODE, ROUTER_CODE, VERIFIER_CODE };
	auto next_record = [&codes](std::string& line) {
		size_t cid = line.find_first_of(codes);
		if (cid <= line.size()) { line = line.substr(cid); }
		return cid;
	};
	
	std::vector<HistoryRecord> history;
	while (next_record(line) != std::string::npos) { // find first record
		history.push_back(parse_record(line));
		line = line.substr(1);
	}

	return history;
}

std::string write_records(const std::vector<HistoryRecord>& history) {
	std::ostringstream os;
	bool first = true;
	for (const auto& rec : history) {
		if (first) { first = false; os << write_record(rec); }
		else { os << ' ' << write_record(rec); }
	}
	return os.str();
}

std::pair<v2d, v2d> face_bbox(const std::vector<v2d>& vertices) {
    double xmin = std::numeric_limits<double>::max();
    double ymin = xmin;
    double xmax = std::numeric_limits<double>::lowest();
    double ymax = xmax;

    for (auto pt : vertices) {
        // std::cout << "pt " << pt << '\t';
        if (pt[0] < xmin) { xmin = pt[0]; }
        if (pt[1] < ymin) { ymin = pt[1]; }
        if (pt[0] > xmax) { xmax = pt[0]; }
        if (pt[1] > ymax) { ymax = pt[1]; }
        // std::cout << xmin << ' ' << ymin << ' ' << xmax << ' ' << ymax << '\n';
    }
    // std::cout << '\n';
    return std::make_pair(v2d(xmin, ymin), v2d(xmax, ymax));   
}

// std::vector<FaceInfo> parse_sheet_info(const std::string &file, const std::vector<size_t> &excluded_fids) {
//     std::ifstream file_in_sht(file);
//     std::vector<v3f> sheet_vertices;
//     std::vector<std::vector<int>> sheet_faces;
//     std::map<int, std::vector<int>> face2Edge;

//     if (!file_in_sht.is_open()) {
//         std::cerr << "[ERROR] Could not open file " << file << '\n';
//         exit(1);
//     }
//     // create the flatface2worldface mapping
//     std::string line;
//     std::map<int, double> edge2offset;
//     std::map<int, bool> edge2isHinge, edge2isHalfHinge, edge2isBoundary;
//     int edge_index_count = 0;
//     while (std::getline(file_in_sht, line)) {
//         // cout << line << endl;
//         if (line[0] == 'v')
//         {
//             // cout << line << endl;
//             std::vector<double> vertex = parse_doubles(line.substr(2));
//             sheet_vertices.push_back(v3f(vertex[0], vertex[1], vertex[2]));
//             // cout << vertex[0] << " " << vertex[1] << " " << vertex[2] << endl;
//         }
//         else if (line[0] == 'f' and not(line.substr(0, 2) == "fe"))
//         {
//             std::vector<int> face = parseLine2int(line.substr(2), " ");
//             sheet_faces.push_back(std::vector<int>{face});
//             // cout << "f" << face[0] << " " << face[1] << " " << face[2] << endl;
//         }
//         else if (line.substr(0, 2) == "fe")
//         { // the line is off the format
//             std::vector<int> info = parseLine2int(line.substr(3), " ");
//             face2Edge[info[0]] = std::vector<int>{info[1], info[2], info[3]};
//             // cout << info[0] << " " << info[1] << " " << info[2] << " " << info[3] << endl;
//         }
//         else if (line[0] == 'e')  {
//             // else if(line.substr(0,11) == "# Map Edges") { // the line is off the format
//             // cout << line << endl;
//             std::vector<double> info = parse_doubles(line.substr(2));
//             // cout << info[0] << " " << info[1] << " " << info[2] << " " << info[3] << " " << info[4] << " " << info[5] << endl;
//             int edge_index = edge_index_count;
//             int f0 = int(info[1]), f1 = int(info[2]);
//             double offset = info[3];
//             bool isHinge = int(info[4]), isHalfHinge = int(info[5]), isBoundary = int(info[6]);
//             edge2offset[edge_index] = offset;
//             edge2isHinge[edge_index] = isHinge;
//             edge2isHalfHinge[edge_index] = isHalfHinge;
//             edge2isBoundary[edge_index] = isBoundary;
//             edge_index_count++;
//         }
//         else  {
//             // nothing here for now..
//         }
//     }

//     std::vector<std::vector<double>> sheet_offsets;
//     std::vector<std::vector<bool>> sheet_isHinge, sheet_isHalfHinge, sheet_isBoundary;
//     sheet_offsets.resize(sheet_faces.size(), std::vector<double>{0, 0, 0});
//     sheet_isHinge.resize(sheet_faces.size(), std::vector<bool>{0, 0, 0});
//     sheet_isHalfHinge.resize(sheet_faces.size(), std::vector<bool>{0, 0, 0});
//     sheet_isBoundary.resize(sheet_faces.size(), std::vector<bool>{0, 0, 0});
//     ForIndex(fi, sheet_offsets.size()) {
//         std::vector<int> edges = face2Edge[fi];

//         double offset0 = edge2offset[edges[0]], offset1 = edge2offset[edges[1]],  offset2 = edge2offset[edges[2]];
//         bool isHinge0 = edge2isHinge[edges[0]],  isHinge1 = edge2isHinge[edges[1]], isHinge2 = edge2isHinge[edges[2]];
//         bool isHHinge0 = edge2isHalfHinge[edges[0]], isHHinge1 = edge2isHalfHinge[edges[1]], isHHinge2 = edge2isHalfHinge[edges[2]];
//         bool isBndry0 = edge2isBoundary[edges[0]], isBndry1 = edge2isBoundary[edges[1]], isBndry2 = edge2isBoundary[edges[2]];
//         sheet_offsets[fi] = {offset0, offset1, offset2}; 
//         sheet_isHinge[fi] = {isHinge0, isHinge1, isHinge2}; 
//         sheet_isHalfHinge[fi] = {isHHinge0, isHHinge1, isHHinge2}; 
//         sheet_isBoundary[fi] = {isBndry0, isBndry1, isBndry2};

//         // because edge0 is between 0 and 1. ids are mapped from opposite vertex
//     }

//     std::vector<FaceInfo> fis;

//     auto v3fTov2d = [](v3f v) { return v2d(v[0], v[1]);};

//     for (int fid = 0; fid < sheet_faces.size(); fid++)
//     {
//         FaceInfo fi;
//         int vi0 = sheet_faces[fid][0], vi1 = sheet_faces[fid][1], vi2 = sheet_faces[fid][2];
//         v2d v0 = v3fTov2d(sheet_vertices[vi0]), v1 = v3fTov2d(sheet_vertices[vi1]), v2 = v3fTov2d(sheet_vertices[vi2]);

//         fi.vertices = {v0, v1, v2};
//         fi.is_hinge_edge = sheet_isHinge[fid];
//         fi.is_half_hinge_edge = sheet_isHalfHinge[fid];
//         fi.is_boundary_edge = sheet_isBoundary[fid];
//         fi.hinge_offsets = sheet_offsets[fid];

//         auto [vmin, vmax] = face_bbox(fi.vertices);
//         fi.min_corner = vmin;
//         fi.max_corner = vmax;

//         fi.fid = fid;
//         fis.push_back(fi);
//     }
//     file_in_sht.close();

//     return fis;
// }

std::vector<FaceInfo> parse_face_info(const std::string& file, const std::vector<size_t>& excluded_fids) {   
    // std::cout << "[parse_face_info()]\t" << file << "\texcl_fids\t";
    // if (excluded_fids.empty()) { std::cout << "NONE"; }
    // for (auto fid : excluded_fids) {
    //     std::cout << fid << ' ';
    // }
    // std::cout << '\n';

    std::ifstream file_in(file);
    if (!file_in.is_open()) {
        std::cerr << "[ERROR] Could not open file " << file << '\n';
        // exit(EXIT_FAILURE);
        return {};
    } 

    std::vector<FaceInfo> fis;
    FaceInfo fi;

    int line_number = 0;
    int num_faces = 0;
    std::string line;
    bool has_face = false;
    while (std::getline(file_in, line)) {
        // std::cout << line_number << '\t' << line << '\n';
        ++line_number;
        size_t header_end = line.find_first_of(' ');
        std::string line_prefix = line.substr(0, header_end);
        line = line.erase(0, header_end + 1);
        if (line_prefix == "#") {
            continue;
        } else if (line_prefix == "t") {
            auto vals = string_to_vector<double>(line);
            assert(vals.size() % 2 == 0);
            for (size_t i = 0; i < vals.size(); i += 2) {
                fi.vertices.push_back(v2d(vals[i], vals[i + 1]));
            }
        } else if (line_prefix == "l") {
            auto vals = string_to_vector<double>(line);
            assert(vals.size() % 2 == 0);
            for (size_t i = 0; i < vals.size(); i += 2) {
                fi.vertices_svg_pos.push_back(v2d(vals[i], vals[i + 1]));
            }
        } else if (line_prefix == "n") {
            fi.is_hinge_edge = string_to_vector<bool>(line);
            assert(fi.vertices.size() == fi.is_hinge_edge.size());
        } else if (line_prefix == "h") {
            fi.is_half_hinge_edge = string_to_vector<bool>(line);
            assert(fi.vertices.size() == fi.is_half_hinge_edge.size());
        } else if (line_prefix == "d") {
            fi.is_deeper_than_neighbor = string_to_vector<bool>(line);
            assert(fi.vertices.size() == fi.is_deeper_than_neighbor.size());
        } else if (line_prefix == "b") {
            fi.is_boundary_edge = string_to_vector<bool>(line);
            assert(fi.vertices.size() == fi.is_boundary_edge.size());
        } else if (line_prefix == "u") {
            fi.is_detour = string_to_vector<bool>(line);
            assert(fi.vertices.size() == fi.is_detour.size());
        } else if (line_prefix == "q") {
            auto vals = string_to_vector<bool>(line);
            assert(2 * fi.vertices.size() == vals.size());
            for (size_t i = 0; i < vals.size(); i +=2) {
                fi.is_narrow.push_back({vals[i], vals[i+1]});
            }
        } else if (line_prefix == "o") {
            fi.hinge_offsets = string_to_vector<double>(line);
            assert(fi.vertices.size() == fi.hinge_offsets.size());
        } else if (line_prefix == "i") {
            auto [r_pnr, r_ver, f_pnr, f_ver, n_same] =
                string_to_tuple<bool, bool, bool, bool, size_t>(line);
            fi.run_pnr = r_pnr;
            fi.run_verif = r_ver;
            fi.failed_pnr = f_pnr;
            fi.failed_verif = f_ver;
            fi.pnr_runs_same_LEDs = n_same;
        } else if (line_prefix == "c") {
            fi.hinge_ios.clear();
            auto vals = string_to_vector<double>(line);
            assert(vals.size() % 4 == 0);
            for (size_t i = 0; i < vals.size(); i += 4) {
                fi.hinge_ios.push_back(std::make_pair(v2d(vals[i    ], vals[i + 1]),
                                                      v2d(vals[i + 2], vals[i + 3])));
            }
            assert(fi.vertices.size() == fi.hinge_ios.size());
        } else if (line_prefix == "w") {
            fi.hinge_ios_wide.clear();
            auto vals = string_to_vector<double>(line);
            assert(vals.size() % 4 == 0);
            for (size_t i = 0; i < vals.size(); i += 4) {
                fi.hinge_ios_wide.push_back(std::make_pair(v2d(vals[i    ], vals[i + 1]),
                                                           v2d(vals[i + 2], vals[i + 3])));
            }
            assert(fi.vertices.size() == fi.hinge_ios_wide.size());
        } else if (line_prefix == "m") {
            auto [p0x, p0y, p1x, p1y, p2x, p2y] =
                string_to_tuple<double, double, double, double, double, double>(line);
            fi.modules.push_back(std::make_tuple(v2d(p0x, p0y), v2d(p1x, p1y), v2d(p2x, p2y)));
        } else if (line_prefix == "s") {
            auto [p0x, p0y, p1x, p1y] =
                string_to_tuple<double, double, double, double>(line);
            fi.traces.push_back(std::make_pair(v2d(p0x, p0y), v2d(p1x, p1y)));
        } else if (line_prefix == "p") {
            fi.has_connector = true;
            if (line.size() > 2) {  // connector module has been placed
                auto [p0x, p0y, p1x, p1y, p2x, p2y] =
                    string_to_tuple<double, double, double, double, double, double>(line);
                fi.connector = std::make_tuple(v2d(p0x, p0y), v2d(p1x, p1y), v2d(p2x, p2y)); 
            }
        } else if (line_prefix == "z") {
            fi.stats_overall = string_to_vector<size_t>(line);
        } else if (line_prefix == "y") {
            // size_t fields = std::accumulate(fi.stats_overall.begin(), fi.stats_overall.end(), 0);
            // size_t k = 0;
            // // std::cout << "k < " << fields << '\n'; 
            // while (k++ < fields) {
            //     auto log = string_to_tuple<char, size_t, double>(line);
            //     fi.stats_detailed.push_back(log);
            //     // std::cout << "k = " << k - 1 << '\n';
            //     // std::cout << line << '\n';
            //     // std::cout << "LOG\t" << std::get<0>(log) << ' ' << std::get<1>(log) << ' ' << std::get<2>(log) << '\n';
            //     size_t space = line.find(' ', 0);
            //     space = line.find(' ', space + 1);
            //     space = line.find(' ', space + 1);
            //     line = line.substr(space + 1);
            //     // std::cout << line << '\n';
            // }
            fi.stats_detailed = parse_records(line);
        } else if (line_prefix == "e") {
            auto fid = string_to_tuple<size_t>(line);
            fi.fid = std::get<0>(fid);
            has_face = true;
        } else {
            std::cerr << "[ERROR] Unhandled case in line " << line_number
                      << " of file " << file << '\n';
            std::cerr << "Case is " << line_prefix << '\n';
            file_in.close();
            exit(EXIT_FAILURE);
        }

        if (has_face) {
            // std::cout << "has face\n";
            has_face = false;
            // if not excluded
            if (std::find(excluded_fids.begin(), excluded_fids.end(), fi.fid) == excluded_fids.end()) {
                // std::cout << fi.fid << " is not excluded\n";
                ++num_faces;
                auto [vmin, vmax] = face_bbox(fi.vertices);
                fi.min_corner = vmin;
                fi.max_corner = vmax;
                fis.push_back(fi);
            } else {
                // std::cout << fi.fid << " is excluded\n";
            }
            fi.vertices.clear();
            fi.vertices_svg_pos.clear();
            fi.modules.clear();
            fi.traces.clear();
            fi.is_narrow.clear();
            fi.stats_detailed.clear();
            fi.has_connector = false;
        }
    }
    file_in.close();

    return fis;
}

void write_face_info(const std::string& file, const FaceInfo& fi) {
    std::ofstream file_out(file, std::ios::out | std::ios::app);
    if (!file_out.is_open()) {
        std::cerr << "[ERROR] Cannot open file " << file << '\n';
        exit(EXIT_FAILURE);
    }

    // vertices (t -> triangle)
    file_out << "t ";
    for (size_t i = 0; i < fi.vertices.size(); ++i) {
        std::string sep = (i == 2) ? "\n" : " ";
        file_out << fi.vertices[i][0] << ' ' << fi.vertices[i][1] << sep;
    }

    // vertices pos in final layout (l -> layout)
    file_out << "l ";
    for (size_t i = 0; i < fi.vertices_svg_pos.size(); ++i) {
        std::string sep = (i == 2) ? "\n" : " ";
        file_out << fi.vertices_svg_pos[i][0] << ' ' << fi.vertices_svg_pos[i][1] << sep;
    }

    // is_hinge_edge (n -> notch)
    file_out << "n ";
    for (auto n : fi.is_hinge_edge) {
        file_out << n << ' ';
    }
    file_out << '\n';

    // is_half_hinge_edge (h -> half)
    file_out << "h ";
    for (auto h : fi.is_half_hinge_edge) {
        file_out << h << ' ';
    }
    file_out << '\n';

    // is_deeper_than_neighbor (d -> deeper)
    file_out << "d ";
    for (auto d : fi.is_deeper_than_neighbor) {
        file_out << d << ' ';
    }
    file_out << '\n';
    
    // is_boundary_edge (b -> boundary)
    file_out << "b ";
    for (auto b : fi.is_boundary_edge) {
        file_out << b << ' ';
    }
    file_out << '\n';

    // is_detour (u -> looks like a detour)
    file_out << "u ";
    for (auto b : fi.is_detour) {
        file_out << b << ' ';
    }
    file_out << '\n';

    // is_narrow (q -> sQueezed)
    file_out << "q ";
    for (auto [b1, b2] : fi.is_narrow) {
        file_out << b1 << ' ' << b2 << ' ';
    }
    file_out << '\n';

    file_out << "o ";
    for(auto o : fi.hinge_offsets) {
        file_out << o << ' ';
    }
    file_out << '\n';

    // status (i -> info)
    file_out << "i " << fi.run_pnr << ' ' << fi.run_verif << ' '
             << fi.failed_pnr << ' ' << fi.failed_verif << ' ' << fi.pnr_runs_same_LEDs;
    file_out << '\n';

    // hinge IOs (connections)
    file_out << 'c';
    for (const auto& c : fi.hinge_ios) {
        file_out << ' ' << c.first[0] << ' ' << c.first[1] << ' '
                 << c.second[0] << ' ' << c.second[1];
    }
    file_out << '\n';

    // hinge IOs (connections)
    file_out << 'w';
    for (const auto& c : fi.hinge_ios_wide) {
        file_out << ' ' << c.first[0] << ' ' << c.first[1] << ' '
                 << c.second[0] << ' ' << c.second[1];
    }
    file_out << '\n';

    // modules
    for (const auto& m : fi.modules) {
        auto [a, b, c] = m;
        file_out << "m " << a[0] << ' ' << a[1] << ' '
                 << b[0] << ' ' << b[1] << ' '
                 << c[0] << ' ' << c[1] << '\n';
    }
    // connector
    if (fi.has_connector) {
        auto [a, b, c] = fi.connector;
        file_out << "p " << a[0] << ' ' << a[1] << ' '
                 << b[0] << ' ' << b[1] << ' '
                 << c[0] << ' ' << c[1] << '\n';
    }

    // traces
    for (const auto& s : fi.traces) {
        file_out << "s " << s.first[0] << ' ' << s.first[1] << ' '
                 << s.second[0] << ' ' << s.second[1] << '\n';
    }

    // STATS
    file_out << "z ";
    for(auto s : fi.stats_overall) {
        file_out << s << ' ';
    }
    file_out << '\n';

    file_out << "y ";
    file_out << write_records(fi.stats_detailed);
    file_out << '\n';

    file_out << "e " << fi.fid << '\n';

    file_out.close();
}

void write_face_info(const std::string& file, const std::vector<FaceInfo>& fis) {
    // std::cout << "[write_face_info()]\t" << file << '\t';
    std::ofstream file_out(file, std::ios::out | std::ios::trunc);
    if (!file_out.is_open()) {
        std::cerr << "[ERROR] Cannot open file " << file << '\n';
        exit(EXIT_FAILURE);
    }   
    file_out.close();

    for (const auto& fi : fis) {
        // std::cout << fi.fid << ' ';
        write_face_info(file, fi);
    }
    // std::cout << '\n';
}

void cat_face_info(const std::string& file) {
    std::cout << "[cat_face_info()]\t" << file << '\n';
    std::ifstream file_in(file, std::ios::in);
    if (!file_in.is_open()) {
        std::cerr << "[ERROR] Could not open file " << file << '\n';
        exit(EXIT_FAILURE);
    } 

    size_t line_num = 0;
    std::string line{};
    while(std::getline(file_in, line)) {
        std::cout << "[" << line_num++ << "]\t" << line << '\n';
    }
    file_in.close();
}

void update_face_info(const std::string& file, const std::vector<FaceInfo>& fis) {
    // std::cout << "[update_face_info()]\t" << file;
    // std::cout << "\texcl_fids\t";
    std::vector<size_t> excl_fids;
    for (size_t i = 0; i < fis.size(); ++i) {
        excl_fids.push_back(fis[i].fid);
        // std::cout << fis[i].fid << ' ';
    }
    // std::cout << '\n';

    std::vector<FaceInfo> updated_fis;
    updated_fis = parse_face_info(file, excl_fids); // non-modified fids
    // std::cout << "existing_fids\t";
    // for (const auto& fi : updated_fis) {
    //     std::cout << fi.fid << ' ';
    // }
    // std::cout << '\n';
    // bool file_exists = std::filesystem::exists(file);
    // if (file_exists) {
    //     std::cout << "file " << file << " exists\n";
    //     updated_fis = parse_face_info(file, excl_fids); // non-modified fids
    //     std::cout << "existing_fids\t";
    //     for (const auto& fi : updated_fis) {
    //         std::cout << fi.fid << ' ';
    //     }
    //     std::cout << '\n';
    // } else {
    //     std::cout << "file " << file << " doesn't exist\n";
    // }

    updated_fis.insert(updated_fis.end(), fis.begin(), fis.end());
    write_face_info(file, updated_fis);
}

// preserves aspect ratio, cannot be inverted by switching source and target
std::pair<double, v2d> transform_AAB(const AAB<2, double>& source, const AAB<2, double>& target) {
    v2d src_extent = source.extent();
    v2d tgt_extent = target.extent();

    double scale = (tgt_extent[0] * src_extent[1] <= tgt_extent[1] * src_extent[0]) ? tgt_extent[0] / src_extent[0] : tgt_extent[1] / src_extent[1];
    v2d translation = target.center() - scale * source.center();

    return std::make_pair(scale, translation);
}

// transform(target, source) != inv(transform(source, target))
// used to recover the point before applying the transformation
std::pair<double, v2d> inv_transform_AAB(const std::pair<double, v2d>& tr) {
    auto [scale, translation] = tr;
    double inv_scale = 1.0 / scale;
    return std::make_pair(inv_scale, -inv_scale * translation);
}

// pt = apply(inv(transform(source, target)), apply(transform(source, target), pt))
v2d apply_transform_AAB(const std::pair<double, v2d>& tr, v2d pt) {
    auto [scale, translation] = tr;
    return scale * pt + translation;
}

Segment apply_transform_AAB(const std::pair<double, v2d>& tr, const std::pair<v2d, v2d>& segment) {
    return std::make_pair(apply_transform_AAB(tr, segment.first), apply_transform_AAB(tr, segment.second));
}

std::vector<v2d> inset_verts(const std::vector<v2d>& vs, double mm_from_edge) {
    auto positive_modulo = [](int a, int b) { return (a % b + b) % b; };
    size_t num_verts = vs.size();
    std::vector<v2d> inset_verts(num_verts);
    for (size_t i = 0; i < num_verts; ++i) {
        int prev = positive_modulo(static_cast<int>(i) - 1, num_verts);
        int next = (i + 1) % num_verts;
        
        v2d e1 = vs[next] - vs[i];
        v2d e2 = vs[prev] - vs[i];
        
        double cos_theta = dot(normalize(e1), normalize(e2));
        double diag = mm_from_edge * std::sqrt(2.0 / (1.0 - cos_theta));  // distance from corner
        inset_verts[i] = vs[i] + diag * normalize(normalize(e1) + normalize(e2));
    }

    return inset_verts;
}

std::pair<v2d, v2d> inset_edge(const std::vector<v2d>& vs, double mm_from_edge, size_t i) {
    auto positive_modulo = [](int a, int b) { return (a % b + b) % b; };
    auto ortho = [](v2d v) { return v2d(-v[1], v[0]); };
    size_t prev = positive_modulo(static_cast<int>(i) - 1, vs.size());
    size_t n = (i + 1) % vs.size();
    size_t next = (n + 1) % vs.size();

    // pointing towards inside
    v2d u = ortho(normalize(vs[n] - vs[i]));
    v2d e_prev = normalize(vs[prev] - vs[i]);
    v2d e_next = normalize(vs[next] - vs[n]);

    return { vs[i] + (mm_from_edge / dot(u, e_prev)) * e_prev,
             vs[n] + (mm_from_edge / dot(u, e_next)) * e_next };
}

// mm[i] is the inset for edge vs[i] -> vs[i+1]
std::vector<v2d> inset_verts(const std::vector<v2d>& vs, const std::vector<double>& mm_from_edge) {
    std::vector<v2d> inset_vs{ vs };
    for (size_t i = 0; i < inset_vs.size(); ++i) {
        auto [vi_new, vn_new] = inset_edge(inset_vs, mm_from_edge[i], i);
        inset_vs[i] = vi_new;
        inset_vs[(i + 1) % inset_vs.size()] = vn_new;
    }
    return inset_vs;
}

std::vector<std::pair<size_t, std::optional<size_t>>>
parse_filter(const std::string& file_filter) {
    std::ifstream file_in(file_filter);
    if (!file_in.is_open()) {
        std::cerr << "[ERROR] Could not open file " << file_filter << '\n';
        exit(EXIT_FAILURE);
    }

    std::vector<std::pair<size_t, std::optional<size_t>>> data{};
    
    int line_number = 0;
    std::string line;

    while (std::getline(file_in, line)) {
        if (line.at(0) == '#') { continue; }
        auto vals = string_to_vector<size_t>(line);
        if (vals.size() == 1) {
            auto val = std::make_pair(vals[0], std::optional<size_t>());
            data.push_back(val);
        } else if (vals.size() == 2) {
            auto val = std::make_pair(vals[0], std::optional<size_t>(vals[1]));
            data.push_back(val);
        } else {
            std::cerr << "[ERROR] Line containing " << vals.size() << " entries in file " << file_filter << ".\n";
            exit(EXIT_FAILURE);
        }
        ++line_number;
    }    

    file_in.close();
    return data;
}

void write_filter(const std::string& file_filter, const std::vector<std::pair<size_t, std::optional<size_t>>>& data) {
    std::ofstream file_out(file_filter, std::ios::out | std::ios::trunc);
    if (!file_out.is_open()) {
        std::cerr << "[ERROR] Could not open file " << file_filter << '\n';
        exit(EXIT_FAILURE);
    }

    for (const auto& [fid, opt_num] : data) {
        file_out << fid;
        if (opt_num.has_value()) {
            file_out << ' ' << opt_num.value();
        }
        file_out << '\n';
    }

    file_out.close();
}

// std::vector<int> parseLine2int(std::string data, std::string delim) {
//     std::vector<int> vals;
//     size_t pos = 0;
//     std::string token;
//     while ((pos = data.find(delim)) != std::string::npos)
//     {
//         token = data.substr(0, pos);
//         // std::cout << token << std::endl;
//         vals.push_back(stoi(token));
//         data.erase(0, pos + delim.length());

//         if ((pos = data.find(delim)) == std::string::npos)
//         {
//             vals.push_back(stoi(data));
//             // cout << data << endl;
//         }
//     }
//     // std::cout << pos << std::endl;
//     return vals;
// }

std::vector<double> led_insets(const Settings& stgs, const FaceInfo& fi) {
    std::vector<double> insets(fi.vertices.size()); // size = #edges
    for (size_t i = 0; i < fi.vertices.size(); ++i) {
        bool cond = fi.is_half_hinge_edge[i] && fi.is_deeper_than_neighbor[i];
        double inset_normal = stgs.led_inset_depth_mm; //LED_INSET_DEPTH;
        double inset_wide = stgs.led_inset_depth_wide_mm; //LED_INSET_DEPTH_WIDE;
        insets[i] = cond ? inset_wide : inset_normal;
    }
    return insets;
}

void debug_info(const FaceInfo& fi, std::ostream& os) {
    auto separate_records = [](std::string& recs) {
        size_t cid = recs.find_first_of(PLACER_CODE, 0);
        while (cid <= recs.size()) {
            if (cid > 0) { 
                if (recs[cid - 1] == ' ') { recs[cid - 1] = '\n'; }
            }
            cid = recs.find_first_of(PLACER_CODE, cid + 1);
        }
    };

    auto polygon_area = [](const std::vector<v2d>& pts) {
        auto cross = [](v2d a, v2d b) { return a[0] * b[1] - a[1] * b[0]; };
        double area = 0.0;
        for (size_t i = 0; i < pts.size(); ++i) {
            area += cross(pts[i], pts[(i + 1) % pts.size()]);
        }
        return 0.5 * std::abs(area);
    };

    auto tr01 = transform_AAB(fi.bbox(), AAB<2, double>(v2d(0), v2d(1)));

    os << "\n################################################################################\n";
    os << "SHEET FACE ID\t" << fi.fid << '\t';
    if (fi.has_connector) { os << "(HAS CONNECTOR)"; } else { os << "(NO CONNECTOR)"; }
    os << '\n';
    os << "AREA " << polygon_area(fi.vertices) << " mm^2\n";
    os << "CURRENT #RECTS " << fi.num_rects() << '\n'; 
    os << "SCALE FACTOR TO 01 " << tr01.first << '\n';

    os << "HISTORY\n";
    os << "P num_leds time\n";
    os << "R exit_status[(o)k, (i)nter, (s)pace] #inter len iosp sp max_iosp max_sp_no_inters max_len_unst_sseq time\n";
    os << "V exit_status[(o)k, (p)lane, (t)race, (b)oth] time\n";
    auto recs = write_records(fi.stats_detailed); separate_records(recs);
    os << recs;
    os << "\n\n";
    
    os << "STATE (run_pnr, run_verif, failed_pnr, failed_verif, pnr_runs_same_LEDs)\n";
    os << "I " << fi.run_pnr << ' ' << fi.run_verif << ' ' << fi.failed_pnr << ' ' << fi.failed_verif << ' ' << fi.pnr_runs_same_LEDs << "\n\n";
    
    os << "BBOX\t" << fi.min_corner << '\t' << fi.max_corner << '\n';
    
    os << "FACE VERTICES\n";
    for (auto v : fi.vertices) {
        os << "V" << v << '\t';
    } os << '\n';
    for (auto v : fi.vertices) {
        os << "V" << apply_transform_AAB(tr01, v) << '\t';
    } os << '\n';

    os << "HINGE IOs\n";
    for (size_t i = 0; i < fi.hinge_ios.size(); ++i) {
        os << "H ";
        if (fi.is_hinge_edge[i]) {
            auto [out, inp] = fi.hinge_ios[i];
            os << "O" << out << "\tI" << inp << '\n';
            auto [sout, sinp] = apply_transform_AAB(tr01, fi.hinge_ios[i]);
            os << "sO" << sout << "\tsI" << sinp << '\n';
        } else {
            os << "BOUNDARY EDGE\n";
        }
    } os << '\n';

    os << "HINGE IOs (WIDE)\n";
    for (size_t i = 0; i < fi.hinge_ios_wide.size(); ++i) {
        os << "W ";
        if (fi.is_hinge_edge[i]) {
            auto [out, inp] = fi.hinge_ios_wide[i];
            os << "O" << out << "\tI" << inp << '\n';
            auto [sout, sinp] = apply_transform_AAB(tr01, fi.hinge_ios_wide[i]);
            os << "sO" << sout << "\tsI" << sinp << '\n';
        } else {
            os << "BOUNDARY EDGE\n";
        }
    } os << '\n';
    
    os << "EDGE INFO (is_hinge, is_half, is_boundary, is_deeper, dihedral_offset)\n";
    for (size_t i = 0; i < fi.vertices.size(); ++i) {
        os << "E " << fi.is_hinge_edge[i] << ' ' << fi.is_half_hinge_edge[i] << ' '
           << fi.is_boundary_edge[i] << ' ' << fi.is_deeper_than_neighbor[i] << ' '
           << fi.hinge_offsets[i] << '\n';
    } os << '\n';

}