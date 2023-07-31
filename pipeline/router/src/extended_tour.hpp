#pragma once

#include "globals.hpp"

#include <Sheet.hpp>
#include <Settings.hpp>
#include <FaceInfo.hpp>
#include <ModuleInfo.hpp>

#include <algorithm>
#include <optional>
#include <utility>
#include <vector>

#include <cassert>

struct IOPair {
    v2d input;
    v2d output;
    std::vector<v2d> detour;
};

struct CutsInfo {
    std::vector<size_t> cuts;
    size_t offset;
    std::vector<IOPair> iops;
};

struct CutSegment {
    v2d start;
    v2d end;
    bool is_LED;
    bool is_detour;
};

class ExtendedTour {
public:
    ExtendedTour() = delete;
    // initializes m_leds, m_leds[0] corresponds to the connector if there is one
    ExtendedTour(const Settings& stgs, const FaceInfo& fi)
        : m_fi(fi), m_leds(), m_tour(), m_orientation(), m_traces() {
        auto transform_led = [&t = std::as_const(fi)](const Led& l) {
            const auto& [inp, out, ctr] = l;
            auto tr = transform_AAB(t.bbox(), AAB<2, double>(v2d(0), v2d(1)));
            
            auto [new_inp, new_out] = apply_transform_AAB(tr, std::make_pair(inp, out));
            auto new_ctr = apply_transform_AAB(tr, ctr);
            return std::make_tuple(new_inp, new_out, new_ctr);
        };
        
        ModuleInfo mi = parse_module_info(stgs.file_module);
        ModuleInfo conn = parse_module_info(stgs.file_connector);
        
        auto get_inp_out = [&t = std::as_const(fi)](const std::tuple<v2d, v2d, v2d>& mod, const ModuleInfo& modinfo) {
            auto [p0, p1, p2] = mod;
            v2d e1 = p1 - p0;
            double rad = std::atan2(e1[1], e1[0]);

            auto rotate = [](double x, double y, double rad) {
                return std::make_pair(cos(rad) * x - sin(rad) * y,
                                        sin(rad) * x + cos(rad) * y);
            };

            auto [inp, out] = modinfo.io;
            auto [inpx, inpy] = inp;
            auto [rinpx, rinpy] = rotate(inpx, inpy, rad);
            auto [outx, outy] = out;
            auto [routx, routy] = rotate(outx, outy, rad);

            // world (PCB) coordinates
            v2d new_inp = p0 + v2d(rinpx, rinpy);
            v2d new_out = p0 + v2d(routx, routy);

            return std::make_tuple(new_inp, new_out, 0.5 * (p1 + p2));
        };
        auto compfunc_mi = [&transform_led, &get_inp_out, &mi](const std::tuple<v2d, v2d, v2d>& mod) {
            return transform_led(get_inp_out(mod, mi));
        };
        auto compfunc_conn = [&transform_led, &get_inp_out, &conn](const std::tuple<v2d, v2d, v2d>& mod) {
            return transform_led(get_inp_out(mod, conn));
        };
        if (fi.has_connector) { m_leds.push_back(compfunc_conn(fi.connector)); }
        std::transform(fi.modules.begin(), fi.modules.end(), std::back_inserter(m_leds), compfunc_mi);
    };

    std::tuple<size_t, double, double> solve_orientation_permutation_cuts(double mm_from_edge, double min_trace_spacing, double& best_spacing);
    std::tuple<size_t, double, double> solve_orientation_permutation_cuts_all(double mm_from_edge, double min_trace_spacing, double& best_spacing);

    // IO functions
    void output_dat(const std::string& filepath) const;
    void print_leds() const {
        for (size_t tid = 0; tid < m_tour.size(); ++tid) {
            auto led = flip_led(tid_to_led(tid), tid_to_flip(tid));
            std::cout << "TID " << tid << "\tM " << std::get<2>(tid_to_led(tid)) << "\tI " << led.first << "\tO " << led.second << '\n';
            led = flip_led(tid_to_led(tid), !tid_to_flip(tid));
            std::cout << "\tAlt. flip\t" << "\tI " << led.first << "\tO " << led.second << '\n';
        }
    }
    friend std::ostream& operator<<(std::ostream& os, const ExtendedTour& extt) {
        os << "Tour:   ";
        for (size_t i = 0; i < extt.m_tour.size(); ++i) {
            os << extt.m_tour[i] << " ";
        }   os << "\nOrient: ";
        for (size_t i = 0; i < extt.m_tour.size(); ++i) {
            os << extt.m_orientation[i] << " ";
        }   os << "\nLength: " << extt.subseq_length() << "\n";
        extt.print_leds();
        return os;
    }

    FaceInfo m_fi;
    bool m_found_spacing_io = false;
    double m_io_spacing = 0.0;
    double m_best_io_spacing = 0.0;
    int m_longest_unstable_subseq = -1;
    Settings m_settings;

private:    
    Led tid_to_led(size_t tid) const { return m_leds.at(m_tour.at(tid)); }
    bool tid_to_flip(size_t tid) const { return m_orientation.at(tid); }
    void set_tid_orientation(size_t tid, bool val) { m_orientation[tid] = val; }

    // returns number of LEDs in a subsequence
    size_t subseq_count(size_t start_tid, size_t end_tid, size_t tour_sz) const;
    // counts self-intersections within a subsequence given an orientation
    size_t subseq_selfintersects(const std::vector<bool>& orient, const std::vector<size_t>& tour, size_t start_tid, size_t end_tid, const std::optional<std::pair<bool, bool>>& stable_orientation = std::optional<std::pair<bool, bool>>()) const;
    // general function
    double subseq_length(const std::vector<bool>& orient, const std::vector<size_t>& tour, size_t start_tid, size_t end_tid, const std::optional<std::pair<bool, bool>>& stable_orientation = std::optional<std::pair<bool, bool>>()) const;
    double subseq_length() const;
    // provided tour with provided orientation
    std::pair<size_t, double> subseq_cost(const std::vector<bool>& orient, const std::vector<size_t>& tour) const;
    // cost of the subsequence of the provided tour with provided orientation
    std::pair<size_t, double> subseq_cost(const std::vector<bool>& orient, size_t start_tid, size_t end_tid) const;
    // general function
    std::pair<size_t, double> subseq_cost(const std::vector<bool>& orient, const std::vector<size_t>& tour, size_t start_tid, size_t end_tid, const std::optional<std::pair<bool, bool>>& stable_orientation = std::optional<std::pair<bool, bool>>()) const;
    
    bool is_led_stable(size_t i, bool& orient) const;
    std::vector<bool> compute_stable_leds() const;

    std::vector<CutSegment> detour_io_segment(const std::vector<IOPair>& iocs, const Segment& s) const; 
    std::vector<IOPair> compute_io_pairs(double detour_mm_from_edge, const std::vector<std::pair<bool, bool>>& is_narrow, const std::vector<bool>& is_detour) const; 

    std::vector<Segment> sol_to_segments(
    const std::vector<size_t>& tour,
    const std::vector<bool>& orient) const;
    std::vector<CutSegment> subseq_to_segments(const std::vector<size_t>& subseq, const IOPair& iop) const;
    // std::vector<std::vector<CutSegment>> cuts_to_segments(const CutsInfo& ci) const;
    // double min_all_trace_spacing(const CutsInfo& ci) const;
    double min_within_path_spacing(const std::vector<Segment>& ss) const;
    // std::tuple<size_t, double, double> cuts_cost(const CutsInfo& ci) const;

    // returns false if ctr tour is self-intersecting
    bool flip_tour_if_not_ccw(std::vector<size_t>& tour);
    std::tuple<size_t, double, double> generate_traces(double inset_verts_mm, double min_trace_spacing, double& best_spacing);

    std::vector<Led> m_leds;            // lid --> led
    std::vector<size_t> m_tour;         // tid --> lid
    std::vector<bool> m_orientation;    // tid --> flip
    std::vector<std::vector<std::pair<v2d, v2d>>> m_traces;

    std::vector<size_t> solve_permutation_concorde(const std::vector<Led>& leds) const;
    bool reverse_tour_if_not_ccw(const std::vector<Led>& leds, std::vector<size_t>& tour, std::vector<bool>& orientation, bool is_concorde) const;
    bool is_led_stable(const std::vector<Led>& leds, const std::vector<size_t>& tour, size_t tid, bool& orient) const;
    std::vector<std::pair<size_t, size_t>> get_unstable_subsequences(const std::vector<bool>& stable_leds) const;
    std::vector<bool> solve_orientation_stability(const std::vector<Led>& leds, const std::vector<size_t>& tour);// const;
    std::pair<std::vector<size_t>, std::vector<bool>> solve_orientation_permutation_exhaustive(const std::vector<Led>& leds) const;
    std::vector<std::tuple<std::vector<size_t>, std::vector<bool>, std::pair<size_t, double>>> solve_orientation_permutation_exhaustive_all(const std::vector<Led>& leds) const;
    
    std::pair<CutSegment, CutSegment>
    subseq_to_io_segments(
        const std::vector<size_t>& tour,
        const std::vector<bool>& orient,
        const std::vector<size_t>& subseq,
        const IOPair& iop) const;
    double min_iocs_spacing(
        const std::vector<size_t>& tour,
        const std::vector<bool>& orient,
        const CutsInfo& ci) const;
    std::vector<std::pair<CutSegment, CutSegment>>
    compute_io_cutsegments(
        const std::vector<size_t>& tour,
        const std::vector<bool>& orient,
        const CutsInfo& ci) const;
    std::vector<CutSegment> subseq_to_segments(const std::vector<size_t>& tour, const std::vector<bool>& orient, const std::vector<size_t>& subseq, const IOPair& iop) const;
    std::vector<std::vector<CutSegment>> cuts_to_segments(const std::vector<size_t>& tour, const std::vector<bool>& orient, const CutsInfo& ci) const;
    std::vector<CutSegment> tour_to_segments(const std::vector<size_t>& tour, const std::vector<bool>& orient) const;
    // double min_all_trace_spacing(const std::vector<size_t>& tour, const std::vector<bool>& orient, const CutsInfo& ci) const;
    double min_all_trace_spacing_v2(const std::vector<size_t>& tour, const std::vector<bool>& orient, const CutsInfo& ci) const;
    double no_hinges_spacing(const std::vector<CutSegment>& css) const;
    std::tuple<size_t, double, double> no_hinges_cost(const std::vector<size_t>& tour, const std::vector<bool>& orient) const;
    std::tuple<size_t, double, double> cuts_cost(const std::vector<size_t>& tour, const std::vector<bool>& orient, const CutsInfo& ci) const;
    CutsInfo solve_cuts(const std::vector<size_t>& tour, std::vector<bool>& orient, double mm_from_edge, double min_trace_spacing, std::tuple<size_t, double, double>& min_cost, double& max_spacing, bool& found_ok_iosp, double min_io_spacing = std::numeric_limits<double>::max());
    void fix_intersections(std::vector<std::vector<CutSegment>>& css, const std::vector<IOPair>& iops) const;
    void flip_modules(const std::vector<size_t>& tour, const std::vector<bool>& orient);
    
    std::tuple<std::vector<size_t>, std::vector<bool>, std::optional<CutsInfo>> exhaustive_pipeline(const std::vector<Led>& leds, double mm_from_edge, double min_trace_spacing, std::tuple<size_t, double, double>& cuts_cost);
};
