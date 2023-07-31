#pragma once

#include <FaceInfo.hpp>

#include <LibSL/LibSL.h>

#include <algorithm>
#include <functional>

typedef std::tuple<v2d, v2d, v2d> Led;  // module inp, module out, module center
Segment flip_led(const Led& ll, bool flipped);

enum class IntersectionType {
    None,
    Point,
    Segment,
};

struct TourPermutation {
    std::vector<std::pair<size_t, bool>> vals;

    void init(size_t n) {
        vals.resize(n);
        for (size_t i = 0; i < n; ++i) {
            vals[i] = std::make_pair(i + 1, true);
        }
    }
    std::vector<size_t> get() const {
        std::vector<size_t> tour(vals.size() + 1); tour[0] = 0;
        size_t k = 1;
        for (auto [i, b] : vals) { tour[k++] = i; }
        return tour;
    }
    bool is_halfway() const { // if starting at (1234) return true if equal to (4321)
        for (size_t i = 0; i < vals.size() - 1; ++i) {
            auto [val, dir] = vals[i];
            if (val != vals.size() - i) { return false; }
        }
        return true;
    }
    bool next() { return std::next_permutation(vals.begin(), vals.end()); }
};

double det(v2d v1, v2d v2);
double point_to_segment_distance(v2d pt, const Segment& s);
double segment_to_segment_distance(const Segment& s1, const Segment& s2);
double min_distance_between_segment_sets(const std::vector<Segment>& ss1, const std::vector<Segment>& ss2);
IntersectionType segments_intersect(const Segment& s1, const Segment& s2, double& tval);
IntersectionType line_segment_intersection(const Segment& line, const Segment& segment, double& tval);
// returns vector of pairs of indices of intersecting segments in input Segment vector
std::vector<std::pair<size_t, size_t>> list_segment_intersections(const std::vector<Segment>& segs);
size_t count_segment_intersections(const std::vector<Segment>& segs);
double segments_length(const std::vector<Segment>& segs);

bool check_led_intersections(const std::vector<Segment>& leds, const Segment& new_led);
// Segment flip_led(const Segment& ll, bool flipped);
double led_asym_cost(const Segment& l1, const Segment& l2);
double led_sym_cost(const Segment& l1, const Segment& l2);
void led_print(const Segment& led);

// v2d led_input(const Segment& ll, bool flipped);
// v2d led_output(const Segment& ll, bool flipped);
// v2d led_middle(const Segment& ll);

v2d led_input(const Led& ll, bool flipped);
v2d led_output(const Led& ll, bool flipped);
v2d led_middle(const Led& ll);

bool both_same_halfplane(v2d hp_tangent, v2d v1, v2d v2);
int rem(int a, int b);

// return true if not done
bool next_permutation(std::vector<std::pair<size_t, bool>>& perm);
bool next_orientation(std::vector<bool>& orient);
bool next_cuts(std::vector<size_t>& cuts, size_t max);
