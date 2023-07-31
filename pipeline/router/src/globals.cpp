#include "globals.hpp"

#include <LibSL/LibSL.h>

#include <algorithm>
#include <set>

#include <cassert>

double det(v2d v1, v2d v2) {
    return v1[0] * v2[1] - v1[1] * v2[0];
}

double point_to_segment_distance(v2d pt, const Segment& s) {
    auto [p0, p1] = s;    
    double t = dot(pt - p0, p1 - p0) / sqDistance(p0, p1);
    t = std::clamp(t, 0.0, 1.0);
    v2d h = p0 + t * (p1 - p0);
    return distance(pt, h);
}

double segment_to_segment_distance(const Segment& s1, const Segment& s2) {
    double tval = 0.0;
    if (segments_intersect(s1, s2, tval) != IntersectionType::None) { return 0.0; }

    auto [p0, p1] = s1;
    auto [q0, q1] = s2;
    return std::min({point_to_segment_distance(p0, s2), point_to_segment_distance(p1, s2),
                     point_to_segment_distance(q0, s1), point_to_segment_distance(q1, s1)}); 
}

double min_distance_between_segment_sets(const std::vector<Segment>& ss1, const std::vector<Segment>& ss2) {
    double min = std::numeric_limits<double>::max();
    for (const auto& s1 : ss1) {
        for (const auto& s2 : ss2) {
            double d = segment_to_segment_distance(s1, s2);
            if (d < min) { min = d; }
        }
    }
    return min;
}

IntersectionType segments_intersect(const Segment& s1, const Segment& s2, double& tval) {
    constexpr double eps = 1e-9;

    v2d p1 = s1.first;
    v2d v1 = s1.second - s1.first;
    v2d p2 = s2.first;
    v2d v2 = s2.second - s2.first;

    double val1 = abs(det(v1, v2));
    double val2 = abs(det(p2 - p1, v1));

    if (val1 < eps && val2 < eps) { // collinear
        double t0 = dot(p2 - p1, v1) / sqLength(v1);
        double t1 = t0 + dot(v1, v2) / sqLength(v1);

        // ensure t0 <= t1
        if (dot(v1, v2) < 0) {
            double tmp = t1;
            t1 = t0;
            t0 = tmp;
        }

        // true if [t0, t1] and [0, 1] are disjoint
        if ((t0 < 0 && t1 < 0) || (t0 > 1 && t1 > 1)) return IntersectionType::None;
        return IntersectionType::Segment;
    } else if (val1 < eps && !(val2 < eps)) {   // parallel non-intersecting
        return IntersectionType::None;
    } else if (!(val1 < eps)) {
        double t = det(p2 - p1, v2) / det(v1, v2);
        double u = det(p2 - p1, v1) / det(v1, v2);
        if (0.0 <= t && t <= 1.0 && 0.0 <= u && u <= 1.0) { // segments intersecting
        // if (0.0 < t && t < 1.0 && 0.0 < u && u < 1.0) { // segments intersecting
            tval = t;
            return IntersectionType::Point;
        }
        return IntersectionType::None;   // lines intersecting
    } else {
        assert(false);
    }
}

IntersectionType line_segment_intersection(const Segment& line, const Segment& segment, double& tval) {
    constexpr double eps = 1e-9;

    v2d p1 = line.first;
    v2d v1 = line.second - line.first;
    v2d p2 = segment.first;
    v2d v2 = segment.second - segment.first;

    double val1 = abs(det(v1, v2));
    double val2 = abs(det(p2 - p1, v1));

    if (val1 < eps && val2 < eps) { // collinear
        double t0 = dot(p2 - p1, v1) / sqLength(v1);
        double t1 = t0 + dot(v1, v2) / sqLength(v1);

        // ensure t0 <= t1
        if (dot(v1, v2) < 0) {
            double tmp = t1;
            t1 = t0;
            t0 = tmp;
        }

        // true if [t0, t1] and [0, 1] are disjoint
        if ((t0 < 0 && t1 < 0) || (t0 > 1 && t1 > 1)) return IntersectionType::None;
        return IntersectionType::Segment;
    } else if (val1 < eps && !(val2 < eps)) {   // parallel non-intersecting
        return IntersectionType::None;
    } else if (!(val1 < eps)) {
        double t = det(p2 - p1, v2) / det(v1, v2);
        double u = det(p2 - p1, v1) / det(v1, v2);
        if (0.0 <= u && u <= 1.0) { // segments intersecting
        // if (0.0 < t && t < 1.0 && 0.0 < u && u < 1.0) { // segments intersecting
            tval = t;
            return IntersectionType::Point;
        }
        return IntersectionType::None;   // lines intersecting
    } else {
        assert(false);
    }
}

bool check_led_intersections(const std::vector<Segment>& leds, const Segment& new_led) {
    for (const auto& led : leds) {
        double t = 0.0;
        if (segments_intersect(led, new_led, t) != IntersectionType::None) return true;
    }
    return false;
}

std::vector<std::pair<size_t, size_t>> list_segment_intersections(const std::vector<Segment>& segs) {
    std::vector<std::pair<size_t, size_t>> is;
    for (size_t i = 0; i < segs.size(); ++i) {
        for (size_t j = i + 1; j < segs.size(); ++j) {
            double t = 0.0;
            if (segments_intersect(segs[i], segs[j], t) != IntersectionType::None) {
                // std::cout << "\ninter\t" << segs[i].first << ' ' << segs[i].second << '\t' << segs[j].first << ' ' << segs[j].second << '\n';
                is.push_back(std::make_pair(i, j));
            }
        }
    }
    return is;
}

size_t count_segment_intersections(const std::vector<Segment>& segs) {
    return list_segment_intersections(segs).size();
}

double segments_length(const std::vector<Segment>& segs) {
    double len = 0.0;
    for (auto& s : segs) {
        len += distance(s.first, s.second);
        // std::cout << s.first << '\t' << s.second << '\n';
    }
    return len;
}

Segment flip_led(const Segment& ll, bool flipped) {
    if (flipped) return std::make_pair(ll.second, ll.first);
    return ll;
}

double led_asym_cost(const Segment& l1, const Segment& l2) {
    return distance(l1.second, l2.first);
}

double led_sym_cost(const Segment& l1, const Segment& l2) {
    v2d mid1 = 0.5 * (l1.first + l1.second);
    v2d mid2 = 0.5 * (l2.first + l2.second);
    return distance(mid1, mid2);
}

void led_print(const Segment& led) {
    std::cout << led.first;
    std::cout << "\t";
    std::cout << led.second;
}

v2d led_input(const Segment& ll, bool flipped) {
    return flipped ? ll.second : ll.first;
}

v2d led_output(const Segment& ll, bool flipped) {
    return flipped ? ll.first : ll.second;
}

v2d led_middle(const Segment& ll) {
    return 0.5 * (ll.first + ll.second);
}

Segment flip_led(const Led& ll, bool flipped) {
    const auto& [inp, out, ctr] = ll;
    if (!flipped) { return std::make_pair(inp, out); }

    v2d inp180 = 2.0 * ctr - inp;
    v2d out180 = 2.0 * ctr - out;
    return std::make_pair(inp180, out180);
}

v2d led_input(const Led& ll, bool flipped) {
    return std::get<0>(flip_led(ll, flipped));
}
v2d led_output(const Led& ll, bool flipped) {
    return std::get<1>(flip_led(ll, flipped));
}
v2d led_middle(const Led& ll) {
    return std::get<2>(ll);
}


bool both_same_halfplane(v2d hp_tangent, v2d v1, v2d v2) {
    v2d hp_ortho = v2d(-hp_tangent[1], hp_tangent[0]);
    return dot(v1, hp_ortho) * dot(v2, hp_ortho) > 0;
}

// always returns positive value
int rem(int a, int b) {
    return (a % b + b) % b;
}

// https://www.cut-the-knot.org/Curriculum/Combinatorics/JohnsonTrotter.shtml
bool next_permutation(std::vector<std::pair<size_t, bool>>& perm) {
    // an element of the permutation consists of a value and an orientation
    // an element el of perm points left if el.second is true, right if it's false

    // swap element of index i with 
    auto swap = [](std::vector<std::pair<size_t, bool>>& pm, size_t i) {
        auto tmp = pm[i];
        int j = i;
        if (tmp.second) --j;    // true  = left
        else            ++j;    // false = right
        pm[i] = pm[j];
        pm[j] = tmp;
        return j;
    };

    // find index of largest mobile value
    auto argmax_mobile = [](const std::vector<std::pair<size_t, bool>>& pm) {
        // value is mobile if larger than value it points to
        auto is_mobile = [](const std::vector<std::pair<size_t, bool>>& ppm, size_t i) {
            auto tmp = ppm[i];
            int j = i;
            if (tmp.second) --j;
            else            ++j;
            return j >= 0 && j < (int)ppm.size() && tmp.first > ppm[j].first;
        };

        size_t max = 0;
        int arg_max = -1;
        for (size_t i = 0; i < pm.size(); ++i) {
            if (pm[i].first > max && is_mobile(pm, i)) {
                max = pm[i].first;
                arg_max = i;
            }
        }
        return arg_max;
    };

    // change orientation of all elements with value larger than the one with index i
    auto rev_gt = [](std::vector<std::pair<size_t, bool>>& pm, size_t i) {
        for (size_t j = 0; j < pm.size(); ++j) {
            if (pm[j].first > pm[i].first) pm[j].second = !pm[j].second;
        }
    };

    int argmax = argmax_mobile(perm);
    if (argmax < 0) return false;    // done
    int new_argmax = swap(perm, argmax);
    rev_gt(perm, new_argmax);
    return true;
}

// when applied to all 1s, returns all 0s and returns false
bool next_orientation(std::vector<bool>& orient) {
    size_t k = 0;
    bool done = false;
    while (k < orient.size() && !done) {
        if (!orient[k]) done = true;    // if 0, flip to 1 and stop
        orient[k] = !orient[k];
        ++k;
    }
    return done;   // false only if orient was all 1s (no next orientation)
}

// a cut set is a set of indices in a vector of size sz
bool next_cuts(std::vector<size_t>& cuts, size_t size) {
    // advances rightmost mobile cut and sets all cuts right of it to its new value
    std::function<bool(std::vector<size_t>&, int, size_t)> aux;
    aux = [&aux](std::vector<size_t>& cs, int i, size_t sz)->bool {
        if (i < 0 || sz <= 0) return false;
        if (cs[i] < sz - 1) {   // if i-th cut is mobile
            cs[i] += 1;         // then move it
            for (size_t j = i + 1; j < cs.size(); ++j) {
                cs[j] = cs[i];  // set all cuts to the right to new value
            }
            return true;
        }
        return aux(cs, i - 1, sz);
    };
    return aux(cuts, cuts.size() - 1, size);
}