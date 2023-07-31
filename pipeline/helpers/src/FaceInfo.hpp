#pragma once

#include "parsing_helpers.hpp"
#include "Settings.hpp"
#include <LibSL/LibSL.h>

#include <cstdlib>
#include <optional>
#include <sstream>
#include <variant>
#include <vector>

#include <cassert>

#define PLACER_CODE 'P'
#define ROUTER_CODE 'R'
#define VERIFIER_CODE 'V'

typedef std::pair<v2d, v2d> Segment;

// number of LEDs, time
typedef std::tuple<size_t, double> PlacerRecord;
// exit status, #intersections, length, io_spacing, spacing, best_io_spacing, max_spacing_no_inters, longest_subseq, time
typedef std::tuple<char, size_t, double, double, double, double, double, int, double> RouterRecord;
// exit status, dist btw traces (if applicable), time
typedef std::tuple<char, double> VerifierRecord;

typedef std::variant<PlacerRecord, RouterRecord, VerifierRecord> HistoryRecord;

HistoryRecord parse_record(std::string str);
std::string write_record(const HistoryRecord& rec);
std::vector<HistoryRecord> parse_records(std::string line);
std::string write_records(const std::vector<HistoryRecord>& history);

// Unused chars: a f g j k r v x
// Used chars: # t l n h d b o i c w m s p e u q z y
typedef struct {
	size_t fid;											// e
	std::vector<v2d> vertices;							// t
	std::vector<v2d> vertices_svg_pos;					// l
	std::vector<std::pair<v2d, v2d>> hinge_ios;			// c
	std::vector<std::pair<v2d, v2d>> hinge_ios_wide;	// w
	std::vector<bool> is_hinge_edge;					// n
	std::vector<bool> is_half_hinge_edge;				// h
	std::vector<bool> is_boundary_edge;					// b
	std::vector<bool> is_deeper_than_neighbor;			// d
	std::vector<double> hinge_offsets;					// o
	std::vector<bool> is_detour;						// u
	std::vector<std::pair<bool, bool>> is_narrow;		// q
	std::vector<std::tuple<v2d, v2d, v2d>> modules;		// m
	std::vector<std::pair<v2d, v2d>> traces;			// s

	std::vector<size_t> stats_overall{};
	std::vector<HistoryRecord> stats_detailed{}; 

	v2d min_corner = v2d(0.0);
	v2d max_corner = v2d(0.0);
	bool has_connector = false;
	std::tuple<v2d, v2d, v2d> connector =				// p
		std::make_tuple(v2d(0.0), v2d(0.0), v2d(0.0));
	
	// status											// i
	bool run_pnr = true; // vorlayout + routing
	bool run_verif = false;
	bool failed_pnr = false;	
	bool failed_verif = false;
	size_t pnr_runs_same_LEDs = 0;

	AAB<2, double> bbox() const { return AAB<2, double>(min_corner, max_corner); };
	int num_rects() const { return modules.size() + (has_connector ? 1 : 0); }
} FaceInfo;

// INPUT/OUTPUT
std::vector<FaceInfo> parse_face_info(const std::string& file, const std::vector<size_t>& excluded_fids = {});
// std::vector<FaceInfo> parse_sheet_info(const std::string& file, const std::vector<size_t>& excluded_fids = {});
void write_face_info(const std::string& file, const FaceInfo& fi);
void write_face_info(const std::string& file, const std::vector<FaceInfo>& fis);
void update_face_info(const std::string& file, const std::vector<FaceInfo>& fis);
void cat_face_info(const std::string& file);

// TRANSFORMATIONS
// used to recover the point before applying the transformation
std::pair<double, v2d> transform_AAB(const AAB<2, double>& source, const AAB<2, double>& target);
// transform(target, source) != inv(transform(source, target))
// used to recover the point before applying the transformation
std::pair<double, v2d> inv_transform_AAB(const std::pair<double, v2d>& tr);
// pt = apply(inv(transform(source, target)), apply(transform(source, target), pt))
v2d apply_transform_AAB(const std::pair<double, v2d>& tr, v2d pt);
Segment apply_transform_AAB(const std::pair<double, v2d>& tr, const Segment& segment);
std::pair<v2d, v2d> face_bbox(const std::vector<v2d>& vertices);

// mm from edges of the triangle
std::vector<v2d> inset_verts(const std::vector<v2d>& vs, double mm_from_edge);
std::pair<v2d, v2d> inset_edge(const std::vector<v2d>& vs, double mm_from_edge, size_t i);
std::vector<v2d> inset_verts(const std::vector<v2d>& vs, const std::vector<double>& mm_from_edge);

std::vector<std::pair<size_t, std::optional<size_t>>> parse_filter(const std::string& file_filter);
void write_filter(const std::string& file_filter, const std::vector<std::pair<size_t, std::optional<size_t>>>& data);

// std::vector<int> parseLine2int(std::string data, std::string delim);

std::vector<double> led_insets(const Settings& stgs, const FaceInfo& fi);

void debug_info(const FaceInfo& fi, std::ostream& os = std::cout);