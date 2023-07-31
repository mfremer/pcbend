#include "extended_tour.hpp"

#include "globals.hpp"

// #include <global_parameters.hpp>

#include <discorde_cpp.h>
#include <clipper.hpp>

#include <algorithm>
#include <tuple>

double ExtendedTour::subseq_length() const {
    return subseq_length(m_orientation, m_tour, 0, m_tour.size());
}

// if start_tid == end_tid --> subseq of length 1
// if start_tid == 0 && end_tid == tour.size() --> tour
double ExtendedTour::subseq_length(const std::vector<bool>& orient, const std::vector<size_t>& tour, size_t start_tid, size_t end_tid, const std::optional<std::pair<bool, bool>>& stable_orientation) const {
    if (orient.empty() && tour.empty()) { return 0; }

    bool is_tour = (start_tid == 0) && (end_tid == tour.size());
    if (is_tour) { end_tid = 0; }
    
    size_t curr_tid = start_tid;
    size_t ss_id = 0;   // in [0, orient.size()[
    double len = 0.0;

    // length from output of start_led to input of end_led
    bool enter = is_tour;
    while (enter || (curr_tid != end_tid)) { // enter only if it's a tour or a subseq of length > 1
        enter = false;
        size_t next_tid = (curr_tid + 1) % tour.size();
        Segment curr_led = flip_led(m_leds.at(tour.at(curr_tid)), orient.at(ss_id));
        ss_id = (ss_id + 1) % orient.size();
        Segment next_led = flip_led(m_leds.at(tour.at(next_tid)), orient.at(ss_id));
        double cost = distance(curr_led.second, next_led.first);
        len += cost;
        
        curr_tid = next_tid;
    }

    if (!is_tour) { // if not tour
        assert(stable_orientation.has_value());
        auto [so_before, so_after] = stable_orientation.value();
        // connect to stable leds (current orientation, not provided)
        size_t before_tid = rem(static_cast<int>(start_tid) - 1, tour.size());
        Segment before_led = flip_led(m_leds.at(tour.at(before_tid)), so_before);
        Segment start_led  = flip_led(m_leds.at(tour.at(start_tid)), orient.at(0));
        len += distance(before_led.second, start_led.first);

        size_t after_tid = (end_tid + 1) % tour.size();
        Segment after_led = flip_led(m_leds.at(tour.at(after_tid)), so_after);
        Segment end_led   = flip_led(m_leds.at(tour.at(end_tid)), orient.at(orient.size() - 1));
        len += distance(end_led.second, after_led.first);
    }

    return len;
}

std::pair<size_t, double> ExtendedTour::subseq_cost(const std::vector<bool>& orient, const std::vector<size_t>& tour) const {
    return std::make_pair(subseq_selfintersects(orient, tour, 0, tour.size()),
                          subseq_length(orient, tour, 0, tour.size()));
}
std::pair<size_t, double> ExtendedTour::subseq_cost(const std::vector<bool>& orient, size_t start_tid, size_t end_tid) const {
    return std::make_pair(subseq_selfintersects(orient, m_tour, start_tid, end_tid),
                          subseq_length(orient, m_tour, start_tid, end_tid));
}
std::pair<size_t, double> ExtendedTour::subseq_cost(const std::vector<bool>& orient, const std::vector<size_t>& tour, size_t start_tid, size_t end_tid, const std::optional<std::pair<bool, bool>>& stable_orientation) const {
    return std::make_pair(subseq_selfintersects(orient, tour, start_tid, end_tid, stable_orientation),
                          subseq_length(orient, tour, start_tid, end_tid, stable_orientation));
}

void ExtendedTour::output_dat(const std::string& filepath) const {
    std::vector<bool> stable_leds = compute_stable_leds();

    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "ERROR: Cannot open " << filepath << "\n";
        exit(EXIT_FAILURE);
    }
    std::cout << "Writing plotting data to " << filepath << '\n';

    std::string sep = "\t";

    file << "# " << filepath << "\n";

    file << "# Triangle (index 0)\n";
    file << -1.0 << sep << -1.0 << "\n" << 1.0 << sep << -1.0 << "\n\n";
    
    auto tr01 = transform_AAB(m_fi.bbox(), AAB<2, double>(v2d(0), v2d(1)));

    for (size_t i = 0; i < m_fi.vertices.size(); ++i) {
        size_t n = (i + 1) % m_fi.vertices.size();
        v2d pi = apply_transform_AAB(tr01, m_fi.vertices[i]);
        v2d pn = apply_transform_AAB(tr01, m_fi.vertices[n]);
        file << pi[0] << sep << pi[1] << "\n";
        file << pn[0] << sep << pn[1] << "\n\n";    
    }
    file << "\n";

    file << "# LED input points (index 1)\n";
    file << "# X\tY\n";
    for (size_t tid = 0; tid < m_tour.size(); ++tid) {
        v2d input = flip_led(tid_to_led(tid), m_orientation[tid]).first;
        file << input[0] << sep << input[1] << "\n";
    }
    file << "\n\n";

    file << "# LED output points (index 2)\n";
    file << "# X\tY\n";
    for (size_t tid = 0; tid < m_tour.size(); ++tid) {
        v2d output = flip_led(tid_to_led(tid), m_orientation[tid]).second;
        file << output[0] << sep << output[1] << "\n";
    }
    file << "\n\n";

    file << "# LED middle points (index 3)\n";
    file << "# X\tY\n";
    for (size_t tid = 0; tid < m_tour.size(); ++tid) {
        v2d mid = std::get<2>(tid_to_led(tid));
        file << mid[0] << sep << mid[1] << "\n";
    }
    file << "\n\n";

    file << "# LED connections (index 4)\n";
    file << "# X\tY\n";
    for (size_t i = 0; i < m_tour.size(); ++i) {
        size_t next_tid = (i + 1) % m_tour.size();
        // file << "# LED" << led1_id << " --> LED" << led2_id << "\n";
        v2d output = flip_led(tid_to_led(i), tid_to_flip(i)).second; // leds[led1_id].second;
        v2d input = flip_led(tid_to_led(next_tid), tid_to_flip(next_tid)).first; // leds[led2_id].first;
        file << output[0] << sep << output[1] << "\n";
        file << input[0] << sep << input[1] << "\n\n";
    }
    file << "\n\n";

    file << "# LED middle tour (index 5)\n";
    file << "# X\tY\n";
    for (size_t i = 0; i < m_tour.size(); ++i) {
        size_t next_tid = (i + 1) % m_tour.size();
        // file << "# LED" << led1_id << " --> LED" << led2_id << "\n";
        v2d mid1 = std::get<2>(tid_to_led(i));
        v2d mid2 = std::get<2>(tid_to_led(next_tid));
        file << mid1[0] << sep << mid1[1] << "\n";
        file << mid2[0] << sep << mid2[1] << "\n\n";
    }
    file << "\n\n";

    file << "# Stable LEDs (index 6)\n";
    file << "# X\tY\n";
    file << -1.0 << sep << -1.0 << "\n" << 1.0 << sep << -1.0 << "\n\n";
    for (size_t i = 0; i < m_tour.size(); ++i) {
        if (stable_leds[i]) {
            Segment led = flip_led(tid_to_led(i), tid_to_flip(i));
            file << led.first[0] << sep << led.first[1] << "\n";
            file << led.second[0] << sep << led.second[1] << "\n\n";
        }
    }
    file << "\n\n";

    file << "# Unstable LEDs (index 7)\n";
    file << "# X\tY\n";
    file << -1.0 << sep << -1.0 << "\n" << 1.0 << sep << -1.0 << "\n\n";
    for (size_t i = 0; i < m_tour.size(); ++i) {
        if (!stable_leds[i]) {
            Segment led = flip_led(tid_to_led(i), tid_to_flip(i));
            file << led.first[0] << sep << led.first[1] << "\n";
            file << led.second[0] << sep << led.second[1] << "\n\n";
        }
    }
    file << "\n\n";

    file << "# LED connections (index 8)\n";
    file << "# X\tY\n";
    file << -1.0 << sep << -1.0 << "\n" << 1.0 << sep << -1.0 << "\n\n";
    for (auto s : m_fi.traces) {
        s = apply_transform_AAB(tr01, s);
        file << s.first[0] << sep << s.first[1] << "\n";
        file << s.second[0] << sep << s.second[1] << "\n\n";
    }
    file << "\n\n";

    file << "# Flipped LEDs (index 9)\n";
    file << "# X\tY\n";
    file << -1.0 << sep << -1.0 << "\n" << 1.0 << sep << -1.0 << "\n\n";
    for (size_t i = 0; i < m_tour.size(); ++i) {
        if (tid_to_flip(i)) {
            Segment led = flip_led(tid_to_led(i), tid_to_flip(i));
            file << led.first[0] << sep << led.first[1] << "\n";
            file << led.second[0] << sep << led.second[1] << "\n\n";
        }
    }
    file << "\n\n";
    
    file.close();
}

size_t ExtendedTour::subseq_count(size_t start_tid, size_t end_tid, size_t tour_sz) const { 
    // proper subseq
    if (start_tid <= end_tid) { return end_tid - start_tid + 1; }
    end_tid += tour_sz;
    return end_tid - start_tid + 1;
}

// tour if start_tid == 0 && end_tid == tour.size()
// else subseq 
size_t ExtendedTour::subseq_selfintersects(const std::vector<bool>& orient, const std::vector<size_t>& tour, size_t start_tid, size_t end_tid, const std::optional<std::pair<bool, bool>>& stable_orientation) const {
    if (orient.empty() && tour.empty()) { return 0; }

    bool is_ss1 = start_tid == end_tid;
    bool is_tour = (start_tid == 0) && (end_tid == tour.size());
    if (is_tour) { end_tid = 0; }

    std::vector<v2d> pts;

    // add output of start stable LED if not tour
    if (!is_tour) {
        assert(stable_orientation.has_value());
        bool so_before = stable_orientation.value().first;
        size_t before_tid = rem(static_cast<int>(start_tid) - 1, tour.size());
        Segment before_led = flip_led(m_leds.at(tour.at(before_tid)), so_before);
        pts.push_back(before_led.first);
        pts.push_back(before_led.second);
    }

    // add inp&out of LEDs in subseq
    size_t curr_tid = start_tid;
    size_t ss_id = 0;
    do {
        Segment curr_led = flip_led(m_leds.at(tour.at(curr_tid)), orient.at(ss_id));
        pts.push_back(curr_led.first);
        pts.push_back(curr_led.second);

        curr_tid = (curr_tid + 1) % tour.size();
        ss_id = (ss_id + 1) % orient.size();
    } while (!is_ss1 && (curr_tid != end_tid));  // only once if subseq of length 1

    if (!is_tour && !is_ss1) {
        Segment curr_led = flip_led(m_leds.at(tour.at(curr_tid)), orient.at(ss_id));
        pts.push_back(curr_led.first);
        pts.push_back(curr_led.second);
    }

    // add input of end stable LED if not tour
    if (!is_tour) {
        assert(stable_orientation.has_value());
        bool so_after = stable_orientation.value().second;
        size_t after_tid = (end_tid + 1) % tour.size();
        Segment after_led = flip_led(m_leds.at(tour.at(after_tid)), so_after);
        pts.push_back(after_led.first);
        pts.push_back(after_led.second);
    }

    // compute segments connecting the LEDs to check for intersections
    std::vector<Segment> segs;
    for (size_t i = 1; i < pts.size() - 1; i += 2) {
        segs.push_back(std::make_pair(pts[i], pts[i+1]));
    }
    if (is_tour) { segs.push_back(std::make_pair(pts[pts.size() - 1], pts[0])); }
    return count_segment_intersections(segs);
}

// CAUTION: only examines stability in the order prev -> curr -> next
// Proper stability should also take next -> curr -> prev into account
// Should work most of the time now since tour is set to CCW before computing stability
bool ExtendedTour::is_led_stable(size_t tid, bool& orient) const {
    Led prev_led = tid_to_led(rem(static_cast<int>(tid) - 1, m_tour.size()));
    Led curr_led = tid_to_led(tid);
    Led next_led = tid_to_led((tid + 1) % m_tour.size());

    v2d curr_inp = led_input(curr_led, false);
    v2d curr_out = led_output(curr_led, false);

    double dA_min = std::min(distance(led_output(prev_led, false), curr_inp),
                             distance(led_output(prev_led, true) , curr_inp)) +
                    std::min(distance(curr_out, led_input(next_led, false)),
                             distance(curr_out, led_input(next_led, true)));
    double dA_max = std::max(distance(led_output(prev_led, false), curr_inp),
                             distance(led_output(prev_led, true) , curr_inp)) +
                    std::max(distance(curr_out, led_input(next_led, false)),
                             distance(curr_out, led_input(next_led, true)));
    
    curr_inp = led_input(curr_led, true);
    curr_out = led_output(curr_led, true);

    double dB_min = std::min(distance(led_output(prev_led, false), curr_inp),
                             distance(led_output(prev_led, true) , curr_inp)) +
                    std::min(distance(curr_out, led_input(next_led, false)),
                             distance(curr_out, led_input(next_led, true)));
    double dB_max = std::max(distance(led_output(prev_led, false), curr_inp),
                             distance(led_output(prev_led, true) , curr_inp)) +
                    std::max(distance(curr_out, led_input(next_led, false)),
                             distance(curr_out, led_input(next_led, true)));

    bool good_A = dA_max < dB_min;
    bool good_B = dB_max < dA_min;
    bool is_stable = good_A || good_B;

    if (is_stable) { orient = good_B; }

    // std::cout << "Stable?\tTID " << tid << "\tM " << led_middle(curr_led) << '\n';
    // std::cout << '\t' << "A (min, max)\t" << dA_min << '\t' << dA_max << '\n';
    // std::cout << '\t' << "B (min, max)\t" << dB_min << '\t' << dB_max << '\n';
    // std::cout << "\t\tdA_max < dB_min: " << good_A << '\n';
    // std::cout << "\t\tdB_max < dA_min: " << good_B << '\n';
    // std::cout << "\t\tstable?: " << is_stable << '\n';

    return is_stable;
}

std::vector<bool> ExtendedTour::compute_stable_leds() const {
    std::vector<bool> mask(m_leds.size(), false);
    for (size_t tid = 0; tid < m_tour.size(); ++tid) {
        bool orient;
        if (is_led_stable(tid, orient)) mask[tid] = true;
    }
    return mask;
}

// TODO FIX
std::vector<CutSegment> ExtendedTour::detour_io_segment(const std::vector<IOPair>& iops, const Segment& s) const {
    auto [fst, snd] = s;
    // triangle w/ single hinge
    assert(std::count(m_fi.is_hinge_edge.begin(), m_fi.is_hinge_edge.end(), true) == 1);
    size_t hinge_id = std::find(m_fi.is_hinge_edge.begin(), m_fi.is_hinge_edge.end(), true) - m_fi.is_hinge_edge.begin();
    auto [inp, out, ins] = iops[hinge_id];

    bool is_first_segment = fst == inp;
    bool is_last_segment = snd == out;
    assert(is_first_segment || is_last_segment);    // segment is connected to I/O
    assert(ins.size() == m_fi.vertices.size());

    Segment ins_s1 = std::make_pair(ins[0], ins[1]);
    Segment ins_s2 = std::make_pair(ins[1], ins[2]);

    double t1 = 0.0;
    double t2 = 0.0;
    v2d i = v2d(0.0);
    bool intersect_first = false;
    bool intersect_second = false;

    std::cout << "Segment\t" << s.first << '\t' << s.second << '\n';
    std::cout << "ins_s1\t" << ins_s1.first << '\t' << ins_s1.second << '\n';
    std::cout << "ins_s2\t" << ins_s2.first << '\t' << ins_s2.second << '\n';

    if (line_segment_intersection(s, ins_s1, t1) == IntersectionType::Point) {
        i = fst + t1 * (snd - fst);
        intersect_first = true;
    } 
    if (line_segment_intersection(s, ins_s2, t2) == IntersectionType::Point) {
        i = fst + t2 * (snd - fst);
        intersect_second = true;
    }
    std::cout << "(t1, t2)\t" << t1 << '\t' << t2 << '\n';

    // should only intersect one of the segments if [is_first_segment || is_last_segment]
    assert(!intersect_first || !intersect_second);
    assert(intersect_first || intersect_second);

    std::vector<CutSegment> detour;
    if (is_first_segment) {
        detour.push_back({fst, ins[0], false, true});
        if (intersect_first) {
            detour.push_back({ins[0], i, false, true});
        } else if (intersect_second) {
            detour.push_back({ins[0], ins[1], false, true});
            detour.push_back({ins[1], i, false, true});
        } else { assert(false); }
        detour.push_back({i, snd, false, true});
    } else if (is_last_segment) {
        detour.push_back({fst, i, false, true});
        if (intersect_first) {
            detour.push_back({i, ins[1], false, true});
            detour.push_back({ins[1], ins[2], false, true});
        } else if (intersect_second) {
            detour.push_back({i, ins[2], false, true});
        } else { assert(false); }
        detour.push_back({ins[2], snd, false, true});
    } else { assert(false); }

    return detour;
}

std::vector<IOPair> ExtendedTour::compute_io_pairs(double detour_mm_from_edge, const std::vector<std::pair<bool, bool>>& is_narrow, const std::vector<bool>& is_detour) const {
    auto select_hio = [this](size_t i, const std::vector<std::pair<bool, bool>>& narrow, const std::vector<bool>& detour) {
        size_t p = static_cast<size_t>(rem(i - 1, detour.size()));
        v2d out = (narrow[i].first && !detour[p]) ? m_fi.hinge_ios[i].first : m_fi.hinge_ios_wide[i].first;
        v2d inp = (narrow[i].second && !detour[i]) ? m_fi.hinge_ios[i].second : m_fi.hinge_ios_wide[i].second; 
        return std::make_pair(out, inp);
    };
    
    auto tr01 = transform_AAB(m_fi.bbox(), AAB<2, double>(v2d(0.0), v2d(1.0)));

    // std::cout << "V\t";
    // for (auto v : m_fi.vertices) {
    //     std::cout << v << ' ';
    // }   std::cout << '\n';

    // std::cout << "AAB\t" << m_fi.min_corner << ' ' << m_fi.max_corner << '\n';

    // std::cout << "sV\t";
    // for (auto v : m_fi.vertices) {
    //     std::cout << apply_transform_AAB(tr01, v) << ' ';
    // }   std::cout << '\n';

    // std::cout << "H\t";
    // for (auto [p0, p1] : m_fi.hinge_ios) {
    //     std::cout << p0 << ' ' << p1 << ' ';
    // }   std::cout << '\n';

    // std::cout << "sH\t";
    // for (auto [p0, p1] : m_fi.hinge_ios) {
    //     std::cout << apply_transform_AAB(tr01, p0) << ' ' << apply_transform_AAB(tr01, p1) << ' ';
    // }   std::cout << '\n';

    // std::cout << "N\t";
    // for (auto v : m_fi.is_hinge_edge) {
    //     std::cout << v << ' ';
    // }   std::cout << '\n';

    auto ivs = inset_verts(m_fi.vertices, detour_mm_from_edge);
    for (size_t i = 0; i < ivs.size(); ++i) {
        ivs[i] = apply_transform_AAB(tr01, ivs[i]);
    }   
    
    auto proj = [](v2d p0, v2d p1, v2d q) {
        return p0 + dot(q - p0, p1 - p0) / sqLength(p1 - p0) * (p1 - p0);
    };

    size_t num_vertices = m_fi.vertices.size();
    size_t num_hinges = std::count(m_fi.is_hinge_edge.begin(), m_fi.is_hinge_edge.end(), true);
    // input, output (in different sides), no led cost (through inset pts)
    size_t ins_total = 0;

    std::vector<IOPair> iops;

    for (size_t i = 0; i < num_vertices; ++i) {
        if (m_fi.is_hinge_edge[i]) {
            // NOT NEXT, next with a hinge edge
            size_t n = (i + 1) % num_vertices;
            while (!m_fi.is_hinge_edge[n]) { n = (n + 1) % num_vertices; }

            // if deeper, then wide
            // bool is_wide_hinge_i = m_fi.is_half_hinge_edge[i] && m_fi.is_deeper_than_neighbor[i];
            // Segment hio_i = m_fi.hinge_ios[i];
            // if (is_wide_hinge_i && !m_fi.failed_verif) {
            //     hio_i = m_fi.hinge_ios_wide[i];
            // }
            // bool is_wide_hinge_n = m_fi.is_half_hinge_edge[n] && m_fi.is_deeper_than_neighbor[n];
            // Segment hio_n = m_fi.hinge_ios[n];
            // if (is_wide_hinge_n && !m_fi.failed_verif) {
            //     hio_n = m_fi.hinge_ios_wide[n];
            // }

            bool is_wide_hinge_i = m_fi.is_half_hinge_edge[i] && m_fi.is_deeper_than_neighbor[i];
            Segment hio_i = m_fi.hinge_ios[i];
            if (is_wide_hinge_i) {
                hio_i = select_hio(i, is_narrow, is_detour);
            }
            bool is_wide_hinge_n = m_fi.is_half_hinge_edge[n] && m_fi.is_deeper_than_neighbor[n];
            Segment hio_n = m_fi.hinge_ios[n];
            if (is_wide_hinge_n) {
                hio_n = select_hio(n, is_narrow, is_detour);
            }

            // hinge input to triangle is hinge_io's right component
            // hinge IOs are inset from the border of the triangle
            // while notches are on the triangle edge
            v2d inp = apply_transform_AAB(tr01, hio_i.second);
            v2d inp_p0 = apply_transform_AAB(tr01, m_fi.vertices[i]);
            v2d inp_p1 = apply_transform_AAB(tr01, m_fi.vertices[(i + 1) % num_vertices]);
            v2d inp_proj = proj(inp_p0, inp_p1, inp);

            v2d out = apply_transform_AAB(tr01, hio_n.first);
            v2d out_p0 = apply_transform_AAB(tr01, m_fi.vertices[n]);
            v2d out_p1 = apply_transform_AAB(tr01, m_fi.vertices[(n + 1) % num_vertices]);
            v2d out_proj = proj(out_p0, out_p1, out);

            v2d io_dir = normalize(out_proj - inp_proj);
            v2d io_nrm = v2d(-io_dir[1], io_dir[0]); // CCW ortho
            
            // std::cout << "i " << i << ' ';
            // std::cout << "n " << n << '\n';
            // std::cout << "inp " << inp << '\t' << "proj " << inp_proj << '\n';
            // std::cout << "out " << out << '\t' << "proj " << out_proj << '\n';

            // projecting IO on the triangle edge (notch) makes ins detection robust
            std::vector<v2d> ins;
            for (size_t k = 0; k < ivs.size(); ++k) {
                // std::cout << "ivs " << ivs[k] << '\t';
                if (dot(ivs[k] - inp_proj, io_nrm) < 0.0) {   // iv right of vector from inp to out
                    // std::cout << "push";
                    ins.push_back(ivs[k]);
                    ++ins_total;
                }
                // std::cout << '\n';
            }  

            auto ccw_wrt_inp =
            [&dr = std::as_const(io_dir), &in = std::as_const(inp_proj)]
            (const v2d& a, const v2d& b) {
                return dot(a - in, dr) < dot(b - in, dr);
            };
            // sort such that full detour is [inp + ins + out]
            std::sort(ins.begin(), ins.end(), ccw_wrt_inp);
            // std::cout << inp << '\t' << out << "\t\t";
            // for (auto v : ins) { std::cout << v << ' '; } std::cout << "\n";
            // std::cout << "push\n";
            iops.push_back({inp, out, ins});
        }
    }
    assert(iops.size() == num_hinges);
    assert(ins_total == num_vertices);

    return iops;
}

std::vector<CutSegment> ExtendedTour::subseq_to_segments(const std::vector<size_t>& subseq, const IOPair& iop) const {
    std::vector<CutSegment> segments;
    // v2d start; v2d end; bool is_LED; bool is_detour;
    
    auto[inp, out, ins] = iop;

    if (subseq.empty()) { // no LEDs in subsequence, connect to inset face vertices
        v2d v = ins.at(0);  // should always exist
        segments.push_back({inp, ins[0], false, true});
        for (size_t i = 1; i < ins.size(); ++i) {
            segments.push_back({v, ins[i], false, true});
            v = ins[i];
        }
        segments.push_back({v, out, false, true});
    } else {
        Segment fst = flip_led(tid_to_led(subseq[0]), tid_to_flip(subseq[0]));
        segments.push_back({inp, fst.first, false, false});
        
        for (size_t tid = 1; tid < subseq.size(); ++tid) {
            Segment prev = flip_led(tid_to_led(subseq[tid - 1]), tid_to_flip(subseq[tid - 1]));
            Segment curr = flip_led(tid_to_led(subseq[tid]), tid_to_flip(subseq[tid]));

            segments.push_back({prev.first, prev.second, true, false});
            segments.push_back({prev.second, curr.first, false, false});
        }

        Segment lst = flip_led(tid_to_led(subseq[subseq.size() - 1]), tid_to_flip(subseq[subseq.size() - 1]));
        segments.push_back({lst.first, lst.second, true, false});
        segments.push_back({lst.second, out, false, false});
    }

    return segments;
}

// std::vector<std::vector<CutSegment>> ExtendedTour::cuts_to_segments(const CutsInfo& ci) const {
//     const auto& [cuts, offset, iops] = ci;

//     // SubSequenceS
//     std::vector<std::vector<size_t>> sss;
//     for (size_t i = 0; i < cuts.size(); ++i) {
//         std::vector<size_t> ss;
//         if (m_tour.size() > 0) {
//             size_t curr = cuts[i];
//             size_t next = cuts[(i+1) % cuts.size()];
//             if (i == cuts.size() - 1 && next <= curr) next += m_tour.size();

//             // subseq = [curr, next[ with cn maybe looping 
//             for (size_t j = curr; j < next; ++j) {
//                 size_t k = j % m_tour.size();
//                 ss.push_back(k);
//             }
//         }
//         sss.push_back(ss);
//     }

//     // SubSequence SegmentS (look, one has to make their own fun)
//     std::vector<std::vector<CutSegment>> ssss(sss.size());
//     // connect subseq to corresponding IO pair
//     for (size_t i = 0; i < sss.size(); ++i) {
//         auto iop = iops[(offset + i) % iops.size()];
//         ssss[i] = subseq_to_segments(sss[i], iop);
//     }

//     return ssss;
// }

// double ExtendedTour::min_all_trace_spacing(const CutsInfo& ci) const {
//     auto ssss = cuts_to_segments(ci);
//     auto cs_to_s = [](const CutSegment& cs) { return std::make_pair(cs.start, cs.end); };
    
//     auto tr01 = transform_AAB(m_fi.bbox(), AAB<2, double>(v2d(0), v2d(1)));
//     // auto inv_tr01 = inv_transform_AAB(tr01);

//     if (ssss.size() == 1) {
//         return segment_to_segment_distance(
//                 cs_to_s(ssss[0].back()),
//                 cs_to_s(ssss[0].front())) / tr01.first;
//     }

//     double min = std::numeric_limits<double>::max();
//     for (size_t i = 0; i < ssss.size(); ++i) {
//         for (size_t j = i + 1; j < ssss.size(); ++j) {
//             std::vector<CutSegment>& vcs1 = ssss[i];
//             std::vector<CutSegment>& vcs2 = ssss[j];
//             auto cs_is_LED = [](const CutSegment& cs) { return cs.is_LED; };
//             (void) std::remove_if(vcs1.begin(), vcs1.end(), cs_is_LED);
//             (void) std::remove_if(vcs2.begin(), vcs2.end(), cs_is_LED); 
            
//             std::vector<Segment> vs1(vcs1.size());
//             std::vector<Segment> vs2(vcs2.size());
//             std::transform(vcs1.begin(), vcs1.end(), vs1.begin(), cs_to_s);
//             std::transform(vcs2.begin(), vcs2.end(), vs2.begin(), cs_to_s);

//             // double d = min_distance_between_segment_sets(vs1, vs2);

//             // std::cout << "i" << i << " j" << j << '\n';
//             for (const auto& s1 : vs1) {
//                 for (const auto& s2 : vs2) {
//                     // auto [p1, p2] = s1;
//                     // auto [q1, q2] = s2;
//                     double d = segment_to_segment_distance(s1, s2);
//                     // std::cout << "\t[" << p1 << "," << p2 << "]\t";
//                     // std::cout << "\t[" << q1 << "," << q2 << "]\n\t\t->\t" << d / tr01.first;
//                     if (d < min) {
//                         min = d;
//                         // std::cout << "\t[MIN]";
//                     } 
//                     // std::cout << '\n';
//                 }
//             }
//             // if (d < min) { min = d; }
//         }
//     }

//     return min / tr01.first;
// }

std::vector<size_t>
ExtendedTour::solve_permutation_concorde(const std::vector<Led>& leds) const {
    // Concorde symmetric distance matrix using distance between module centers
    auto led_sym_cost_matrix = [this](const std::vector<Led>& leds) {
        size_t num_leds = leds.size();
        // initialize matrix
        int** cmat = new int*[num_leds];
        for (size_t i = 0; i < num_leds; ++i) { cmat[i] = new int[num_leds]; }

        for (size_t i = 0; i < num_leds; ++i) {
            for (size_t j = 0; j < num_leds; ++j) {
                if (i < j) {    // symmetric distance, only fill strictly triangular
                    double d_ctr = distance(std::get<2>(leds[i]), std::get<2>(leds[j]));
                    cmat[i][j] = std::round(m_settings.concorde_scaling * d_ctr);
                } else {
                    cmat[i][j] = 0;
                }
            }
        }

        return cmat;
    };
    auto free_cost_matrix = [](int** cmat, size_t num_leds) {
        for (size_t i = 0; i < num_leds; ++i) { delete[] cmat[i]; }
        delete[] cmat;
    };

    // CONCORDE TSP CALL
    size_t num_leds = leds.size();
    int** cmat = led_sym_cost_matrix(leds);
    int* out_tour = new int[num_leds];
    double out_cost;
    int out_status;

    std::cout << "\nCONCORDE TSP\n";
    int ret_val = discorde::concorde_full(num_leds, cmat, out_tour, &out_cost, &out_status);
    if (ret_val == DISCORDE_RETURN_OK) std::cout << "Solver succeeded\n";
    else {
        std::cout << "Solver failed\n";
        exit(EXIT_FAILURE);
    }
    if (out_status == DISCORDE_STATUS_OPTIMAL) std::cout << "Optimal tour found\n";
    std::cout << '\n';

    free_cost_matrix(cmat, num_leds);
    std::vector<size_t> permutation(out_tour, out_tour + num_leds);
    // m_orientation.resize(m_tour.size(), false);
    delete[] out_tour;
    // END CONCORDE

    return permutation;
}

// reverses tour if the polygon passing through the centers of the LEDs is not CCW
// if is_concorde, exits if self-intersecting, if use_center_polygon, returns false
// if use_real_polygon (orientation non empty), continues and returns false if flipped
bool ExtendedTour::reverse_tour_if_not_ccw(const std::vector<Led>& leds, std::vector<size_t>& tour, std::vector<bool>& orientation, bool is_concorde) const {
    using namespace ClipperLib;

    /* Sub has at most one point x not in Sup, Sub - {x} \subset Sup.
     * Sub should have at least three points (two if Sub \subset Sup).
     * Sup is in the same orientation as Sub if there is a pair of consecutive
     * points in Sub ocurring consecutively in that same order in Sup.
     */
    auto same_orientation = [](const Path& sub, const Path& sup) {
        for (size_t sub_i = 0; sub_i < sub.size(); ++sub_i) {
            size_t sub_n = (sub_i + 1) % sub.size();
            for (size_t sup_i = 0; sup_i < sup.size(); ++sup_i) {
                size_t sup_n = (sup_i + 1) % sup.size();
                if (sub[sub_i] == sup[sup_i] &&
                    sub[sub_n] == sup[sup_n]) { return true; }
            }
        }
        return false;
    };

    bool use_real_polygon = !orientation.empty();
    // std::cout << "use_real_polygon\t" << use_real_polygon << '\n';

    Path p; // polygon with vertices the LED centers
    for (size_t i = 0; i < tour.size(); ++i) {
        auto [in, out, ctr] = leds.at(tour.at(i));
        if (use_real_polygon) {
            auto [in_flip, out_flip] = flip_led(Led{in, out, ctr}, orientation.at(i));
            p.push_back(IntPoint(m_settings.clipper_scaling * in_flip[0] , m_settings.clipper_scaling * in_flip[1]));
            p.push_back(IntPoint(m_settings.clipper_scaling * out_flip[0], m_settings.clipper_scaling * out_flip[1]));
        } else {
            p.push_back(IntPoint(m_settings.clipper_scaling * ctr[0], m_settings.clipper_scaling * ctr[1]));
        }
    }

    // http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/SimplifyPolygon.htm
    Paths ps; SimplifyPolygon(p, ps, PolyFillType::pftNonZero);
    if (ps.size() > 1) {
        if (is_concorde) {
            std::cerr << "[ERROR] Concorde tour polygon passing through LED centers"
                "is self-intersecting. The TSP solution is not optimal.\n";
            exit(EXIT_FAILURE);
        } else if (!use_real_polygon) {
            return false;
        } else if (use_real_polygon) {
            // std::cerr << "[WARNING] LED polygon is self-intersecting.\n"; 
        }
    }

    // find largest simple polygon (only for real polygon)
    size_t i_max = 0;
    size_t sz_max = 0;
    for (size_t i = 0; i < ps.size(); ++i) {
        if (ps[i].size() > sz_max) {
            sz_max = ps[i].size();
            i_max = i;
        }
    }
    size_t ref = use_real_polygon ? i_max : 0;

    // std::cout << "Before simplify\n" << p << '\n';
    // std::cout << "After simplify\n" << ps;
    // std::cout << "ref " << ref << '\n';

    // TODO: assert the following statement (just in case)
    // SimplifyPolygon orders every ps[k] CCWise
    if (!same_orientation(ps.at(ref), p)) { // if p is not CCW flip tour
        // std::cout << "Reverse tour\n";
        std::reverse(tour.begin() + 1, tour.end());
        if (use_real_polygon) {
            // std::cout << "Reverse orientation\n";
            std::reverse(orientation.begin() + 1, orientation.end());
            return false;
        }
    }
    return true;
}

std::vector<std::pair<size_t, size_t>>
ExtendedTour::get_unstable_subsequences(const std::vector<bool>& stable_leds) const {
    // returns vb[i-1] && !vb[i] (with correct indices)
    auto true_then_false = [](const std::vector<bool>& vb, size_t i) {
        size_t p = rem(static_cast<int>(i) - 1, vb.size());
        return vb[p] && !vb[i];
    };
    // returns !vb[i] && vb[i+1] (with correct indices)
    auto false_then_true = [](const std::vector<bool>& vb, size_t i) {
        size_t n = (i + 1) % vb.size();
        return !vb[i] && vb[n];
    };

    // std::cout << "stable_leds ";
    // for (auto b : stable_leds) { 
    //     std::cout << b << ' ';
    // }   std::cout << '\n';

    std::vector<std::pair<size_t, size_t>> subseqs;
    size_t num_leds = stable_leds.size();
    
    // find beginning of first unstable subsequence
    size_t fst = 0;
    while (fst < num_leds && !true_then_false(stable_leds, fst)) { ++fst; }
    if (fst >= num_leds) { // full tour is unstable
        subseqs.push_back(std::make_pair(0, num_leds));
        return subseqs;
    }
    
    size_t curr = fst;
    size_t pair_fst = fst; // buffer for start of current subseq
    bool in_pair = true;   // are we inside an unstable subseq?
    do {
        // std::cout << curr << ' ';
        if (false_then_true(stable_leds, curr)) { // end of subseq
            // std::cout << "ftt ";
            if (in_pair) {
                in_pair = !in_pair;
                subseqs.push_back(std::make_pair(pair_fst, curr));
            } else { assert(false); }
        }
        curr = (curr + 1) % num_leds;
        // std::cout << curr << ' ';
        if (true_then_false(stable_leds, curr)) { // start of subseq
            // std::cout << "ttf ";
            if (!in_pair) {
                in_pair = !in_pair;
                pair_fst = curr;
            } else { assert(false); }
        }
    } while (curr != fst);

    return subseqs;
}

// CAUTION: only examines stability in the order prev -> curr -> next
// Proper stability should also take next -> curr -> prev into account
// Should work most of the time now since tour is set to CCW before computing stability
bool ExtendedTour::is_led_stable(const std::vector<Led>& leds, const std::vector<size_t>& tour, size_t tid, bool& orient) const {
    auto tid_to_led = [](const std::vector<Led>& ls, const std::vector<size_t>& t, size_t tid) {
        return ls.at(t.at(tid));
    };

    Led prev_led = tid_to_led(leds, tour, rem(static_cast<int>(tid) - 1, tour.size()));
    Led curr_led = tid_to_led(leds, tour, tid);
    Led next_led = tid_to_led(leds, tour, (tid + 1) % tour.size());

    v2d curr_inp = led_input(curr_led, false);
    v2d curr_out = led_output(curr_led, false);

    double dA_min = std::min(distance(led_output(prev_led, false), curr_inp),
                             distance(led_output(prev_led, true) , curr_inp)) +
                    std::min(distance(curr_out, led_input(next_led, false)),
                             distance(curr_out, led_input(next_led, true)));
    double dA_max = std::max(distance(led_output(prev_led, false), curr_inp),
                             distance(led_output(prev_led, true) , curr_inp)) +
                    std::max(distance(curr_out, led_input(next_led, false)),
                             distance(curr_out, led_input(next_led, true)));
    
    curr_inp = led_input(curr_led, true);
    curr_out = led_output(curr_led, true);

    double dB_min = std::min(distance(led_output(prev_led, false), curr_inp),
                             distance(led_output(prev_led, true) , curr_inp)) +
                    std::min(distance(curr_out, led_input(next_led, false)),
                             distance(curr_out, led_input(next_led, true)));
    double dB_max = std::max(distance(led_output(prev_led, false), curr_inp),
                             distance(led_output(prev_led, true) , curr_inp)) +
                    std::max(distance(curr_out, led_input(next_led, false)),
                             distance(curr_out, led_input(next_led, true)));

    bool good_A = dA_max < dB_min;
    bool good_B = dB_max < dA_min;
    bool is_stable = good_A || good_B;

    if (is_stable) { orient = good_B; }
    return is_stable;
}

std::vector<bool>
ExtendedTour::solve_orientation_stability(const std::vector<Led>& leds, const std::vector<size_t>& tour) {   
    auto replace = [](std::vector<bool>& dst, const std::vector<bool>& src, size_t first) {
        for (size_t i = first; i < first + src.size(); ++i) {
            dst[i % dst.size()] = src[i - first];
        }
    };

    std::vector<bool> orientation(tour.size(), false);
    
    // find stable LEDs and set best orientation
    std::vector<bool> stable_leds(tour.size(), false);
    for (size_t i = 0; i < tour.size(); ++i) {
        bool orient = false;
        stable_leds[i] = is_led_stable(leds, tour, i, orient);
        orientation[i] = orient;
    }

    // std::cout << "LED STABILITY\n";
    // for (auto b : stable_leds) { std::cout << b << ' ';}
    // std::cout << '\n';

    std::vector<std::pair<size_t, size_t>> unstable_subsequences =
        get_unstable_subsequences(stable_leds);

    size_t max_subseq_len = 0;

    // std::cout << "SOLVE UNSTABLE SUBSEQUENCES\n";
    // brute force orientation for every unstable subsequence
    std::cout << "US\t";
    for (auto [start_tid, end_tid] : unstable_subsequences) {
        std::cout << "[" << start_tid << ", " << end_tid << "]"; 
        size_t subseq_len = (end_tid == tour.size()) ? tour.size()
                                                     : subseq_count(start_tid, end_tid, tour.size());
        std::cout << "(#" << subseq_len << ") ";
        std::vector<bool> subseq_orientation(subseq_len, false);
        if (subseq_len > max_subseq_len) { max_subseq_len = subseq_len; }
        
        std::vector<bool> min_orientation(subseq_orientation);
        auto min_cost = std::make_pair(std::numeric_limits<size_t>::max(),
                                       std::numeric_limits<double>::max());
        do {
            // std::cout << '\t';
            // for (auto b : subseq_orientation) {
            //     std::cout << b << ' ';
            // }
            // std::cout << '\t';
            auto stable_orientation =
                std::make_pair(orientation[rem(start_tid - 1, orientation.size())],
                               orientation[(start_tid + 1) % orientation.size()]);
            auto cost = subseq_cost(subseq_orientation, tour, start_tid, end_tid, stable_orientation);
            auto [inters, len] = cost;
            // std::cout << inters << " " << len;
            if (cost < min_cost) { 
                // std::cout << "\tBEST";
                min_cost = cost;
                min_orientation = subseq_orientation;
            }
            // std::cout << "\n";
        } while (next_orientation(subseq_orientation));
        // std::cout << "\norientation\t";
        // for (auto b : orientation) { std::cout << b << ' '; } std::cout << '\n';
        // std::cout << "min_orientation\t";
        // for (auto b : min_orientation) { std::cout << b << ' '; } std::cout << '\n';
        replace(orientation, min_orientation, start_tid);
    }

    std::cout << "\nLength of longest unstable subsequence: " << max_subseq_len << '\n';
    m_longest_unstable_subseq = max_subseq_len;

    return orientation;
}

std::pair<std::vector<size_t>, std::vector<bool>>
ExtendedTour::solve_orientation_permutation_exhaustive(const std::vector<Led>& leds) const {
    TourPermutation perm; perm.init(leds.size() - 1);

    std::vector<size_t> min_tour(leds.size(), 0);
    std::vector<bool> min_orientation(leds.size(), false);
    auto min_cost = std::make_pair(std::numeric_limits<size_t>::max(),
                                   std::numeric_limits<double>::max());

    do {
        std::vector<size_t> tour = perm.get();
        std::vector<bool> orientation(leds.size(), false);
        
        std::vector<bool> empty_dummy{};

        // for (auto k : tour) {
        //     std::cout << k << ' ';
        // }   std::cout << '\n';

        // skips self-intersecting tours, orients the others CCWise
        if (tour.size() >= 3 &&
            !reverse_tour_if_not_ccw(leds, tour, empty_dummy, false)) { continue; }
        
        do {
            // std::cout << '\t';
            // for (auto k : orientation) {
            //     std::cout << k << ' ';
            // }   std::cout << '\t';

            auto cost = subseq_cost(orientation, tour);
            // std::cout << cost.first << ' ' << cost.second << '\t';
            if (cost < min_cost) {
                // std::cout << "BEST";
                min_cost = cost;
                min_tour = tour;
                min_orientation = orientation;
            }
            // std::cout << '\n';
        } while (next_orientation(orientation));
        // std::cout << '\n';
    } while (perm.next() && !perm.is_halfway());

    return std::make_pair(min_tour, min_orientation);
}

std::vector<std::tuple<std::vector<size_t>, std::vector<bool>, std::pair<size_t, double>>>
ExtendedTour::solve_orientation_permutation_exhaustive_all(const std::vector<Led>& leds) const {
    if (leds.size() == 0) {
        std::vector<size_t> tour{};
        std::vector<bool> orient{}; 
        return {std::make_tuple(tour, orient, subseq_cost(orient, tour))};
    }
    
    TourPermutation perm; perm.init(leds.size() - 1);

    std::vector<std::tuple<std::vector<size_t>, std::vector<bool>, std::pair<size_t, double>>> sols;

    std::vector<size_t> min_tour(leds.size(), 0);
    std::vector<bool> min_orientation(leds.size(), false);

    do {
        std::vector<size_t> tour = perm.get();
        std::vector<bool> orientation(leds.size(), false);
        
        std::vector<bool> empty_dummy;

        // skips self-intersecting tours, orients the others CCWise
        if (tour.size() >= 3 &&
            !reverse_tour_if_not_ccw(leds, tour, empty_dummy, false)) { continue; }
        
        do {
            auto cost = subseq_cost(orientation, tour);
            auto [inter, len] = cost;
            if (inter == 0) {
                sols.push_back(std::make_tuple(tour, orientation, cost));
            }
            
        } while (next_orientation(orientation));
    } while (perm.next() && !perm.is_halfway());

    auto cmp_cost = [] (auto lhs, auto rhs) { return std::get<2>(lhs) < std::get<2>(rhs); };
    std::sort(sols.begin(), sols.end(), cmp_cost);
    return sols;
}

std::pair<CutSegment, CutSegment>
ExtendedTour::subseq_to_io_segments(
    const std::vector<size_t>& tour,
    const std::vector<bool>& orient,
    const std::vector<size_t>& subseq,
    const IOPair& iop) const
{
    // override normal accessors by providing custom tour/orient
    auto tid_to_led = [this](const std::vector<size_t>& perm, size_t tid) {
        return m_leds.at(perm.at(tid));
    };
    auto tid_to_flip = [](const std::vector<bool>& orient, size_t tid) {
        return orient.at(tid);
    };
    CutSegment cs_fst; CutSegment cs_snd;
    auto[inp, out, ins] = iop;

    if (subseq.empty()) {
        v2d v = ins.at(0);  // should always exist
        cs_fst = {inp, ins[0], false, true};
        for (size_t i = 1; i < ins.size(); ++i) {
            // segments.push_back({v, ins[i], false, true});
            v = ins[i];
        }
        // cs_snd = {v, out, false, true};
        cs_snd = {ins[ins.size()-1], out, false, true};
    } else {
        Segment fst = flip_led(tid_to_led(tour, subseq[0]), tid_to_flip(orient, subseq[0]));
        cs_fst = {inp, fst.first, false, false};
        
        // for (size_t tid = 1; tid < subseq.size(); ++tid) {
        //     Segment prev = flip_led(tid_to_led(tour, subseq[tid - 1]), tid_to_flip(orient, subseq[tid - 1]));
        //     Segment curr = flip_led(tid_to_led(tour, subseq[tid]), tid_to_flip(orient, subseq[tid]));

        //     segments.push_back({prev.first, prev.second, true, false});
        //     segments.push_back({prev.second, curr.first, false, false});
        // }

        Segment lst = flip_led(tid_to_led(tour, subseq[subseq.size() - 1]), tid_to_flip(orient, subseq[subseq.size() - 1]));
        // segments.push_back({lst.first, lst.second, true, false});
        cs_snd = {lst.second, out, false, false};
    }

    return {cs_fst, cs_snd};
}

std::vector<std::pair<CutSegment, CutSegment>>
ExtendedTour::compute_io_cutsegments(
    const std::vector<size_t>& tour,
    const std::vector<bool>& orient,
    const CutsInfo& ci) const
{
    const auto& [cuts, offset, iops] = ci;

    // SubSequenceS
    std::vector<std::vector<size_t>> sss;
    for (size_t i = 0; i < cuts.size(); ++i) {  // one subsequence per hinge
        std::vector<size_t> ss;
        if (tour.size() > 0) {
            size_t curr = cuts[i];
            size_t next = cuts[(i+1) % cuts.size()];
            if (i == cuts.size() - 1 && next <= curr) next += tour.size();

            for (size_t j = curr; j < next; ++j) {
                size_t k = j % tour.size();
                ss.push_back(k);
            }
        }
        sss.push_back(ss);
    }

    // SubSequence SegmentS (look, one has to make their own fun)
    std::vector<std::pair<CutSegment, CutSegment>> ssss(sss.size());
    for (size_t i = 0; i < sss.size(); ++i) {
        auto iop = iops[(offset + i) % iops.size()];
        ssss[i] = subseq_to_io_segments(tour, orient, sss[i], iop);
    }
    return ssss;
}

double ExtendedTour::min_iocs_spacing(
    const std::vector<size_t>& tour,
    const std::vector<bool>& orient,
    const CutsInfo& ci) const
{
    auto cs_to_s = [](const CutSegment& cs) { return std::make_pair(cs.start, cs.end); };

    auto iocs = compute_io_cutsegments(tour, orient, ci);
    size_t num_chains = iocs.size();
    assert(num_chains > 0);

    // if (num_chains == 1) {
    //     auto [uc_inp, uc_out] = iocs[0]; // unique chain
    //     double d = segment_to_segment_distance(cs_to_s(uc_inp), cs_to_s(uc_out)); 
    //     std::cout << cs_to_s(uc_inp).first << ' ' << cs_to_s(uc_inp).second << '\t';
    //     std::cout << cs_to_s(uc_out).first << ' ' << cs_to_s(uc_out).second << '\t' << d << '\n';
    //     return d;
    // }

    v2d min_i1, min_i2, min_j1, min_j2;
    double min = std::numeric_limits<double>::max();
    for (size_t i = 0; i < num_chains; ++i) {
        int p = rem(static_cast<int>(i) - 1, num_chains);
        auto [pc_inp, pc_out] = iocs[p]; // prev chain inp/out
        auto [cc_inp, cc_out] = iocs[i]; // curr chain inp/out
        double d = segment_to_segment_distance(cs_to_s(pc_out), cs_to_s(cc_inp));
        if (d < min) { 
            min_i1 = cs_to_s(pc_out).first;
            min_i2 = cs_to_s(pc_out).second;
            min_j1 = cs_to_s(cc_inp).first;
            min_j2 = cs_to_s(cc_inp).second;
            min = d; 
        }
    }

    auto tr01 = transform_AAB(m_fi.bbox(), AAB<2, double>(v2d(0), v2d(1)));

    // std::cout << "\nBEST ";
    // std::cout << min_i1 << ' ' << min_i2 << '\t';
    // std::cout << min_j1 << ' ' << min_j2 << '\t';
    // std::cout << min << ' ' << min / tr01.first << '\n';

    return min / tr01.first;
}

std::vector<Segment> ExtendedTour::sol_to_segments(
    const std::vector<size_t>& tour,
    const std::vector<bool>& orient) const
{
    // override normal accessors by providing custom tour/orient
    auto tid_to_led = [this](const std::vector<size_t>& perm, size_t tid) {
        return m_leds.at(perm.at(tid));
    };
    auto tid_to_flip = [](const std::vector<bool>& orient, size_t tid) {
        return orient.at(tid);
    };
    std::vector<Segment> segments;
    Segment fst = flip_led(tid_to_led(tour, 0), tid_to_flip(orient, 0));
    // segments.push_back({inp, fst.first});
    
    for (size_t tid = 1; tid < tour.size(); ++tid) {
        Segment prev = flip_led(tid_to_led(tour, tid - 1), tid_to_flip(orient, tid - 1));
        Segment curr = flip_led(tid_to_led(tour, tid), tid_to_flip(orient, tid));

        // segments.push_back({prev.first, prev.second}); // LED
        segments.push_back({prev.second, curr.first});
    }

    Segment lst = flip_led(tid_to_led(tour, tour.size() - 1), tid_to_flip(orient, tour.size() - 1));
    // segments.push_back({lst.first, lst.second}); // LED
    segments.push_back({lst.second, fst.first});

    return segments;
}

// Takes a subsequence of LEDs gen from cuts and connects it to corresponding IOPair
std::vector<CutSegment>
ExtendedTour::subseq_to_segments(const std::vector<size_t>& tour,
                                 const std::vector<bool>& orient,
                                 const std::vector<size_t>& subseq,
                                 const IOPair& iop) const
{
    // override normal accessors by providing custom tour/orient
    auto tid_to_led = [this](const std::vector<size_t>& perm, size_t tid) {
        return m_leds.at(perm.at(tid));
    };
    auto tid_to_flip = [](const std::vector<bool>& orient, size_t tid) {
        return orient.at(tid);
    };
    std::vector<CutSegment> segments;
    auto[inp, out, ins] = iop;

    if (subseq.empty()) {
        v2d v = ins.at(0);  // should always exist
        segments.push_back({inp, ins[0], false, true});
        for (size_t i = 1; i < ins.size(); ++i) {
            segments.push_back({v, ins[i], false, true});
            v = ins[i];
        }
        segments.push_back({v, out, false, true});
    } else {
        Segment fst = flip_led(tid_to_led(tour, subseq[0]), tid_to_flip(orient, subseq[0]));
        segments.push_back({inp, fst.first, false, false});
        
        for (size_t tid = 1; tid < subseq.size(); ++tid) {
            Segment prev = flip_led(tid_to_led(tour, subseq[tid - 1]), tid_to_flip(orient, subseq[tid - 1]));
            Segment curr = flip_led(tid_to_led(tour, subseq[tid]), tid_to_flip(orient, subseq[tid]));

            segments.push_back({prev.first, prev.second, true, false});
            segments.push_back({prev.second, curr.first, false, false});
        }

        Segment lst = flip_led(tid_to_led(tour, subseq[subseq.size() - 1]), tid_to_flip(orient, subseq[subseq.size() - 1]));
        segments.push_back({lst.first, lst.second, true, false});
        segments.push_back({lst.second, out, false, false});
    }

    return segments;
}

std::vector<CutSegment>
ExtendedTour::tour_to_segments(const std::vector<size_t>& tour, const std::vector<bool>& orient) const {
    // override normal accessors by providing custom tour/orient
    auto tid_to_led = [this](const std::vector<size_t>& perm, size_t tid) {
        return m_leds.at(perm.at(tid));
    };
    auto tid_to_flip = [](const std::vector<bool>& orient, size_t tid) {
        return orient.at(tid);
    };
    
    std::vector<CutSegment> segments;

    // Segment fst = flip_led(tid_to_led(tour, tour[0]), tid_to_flip(orient, tour[0]));
    // segments.push_back({inp, fst.first, false, false});
    
    for (size_t tid = 0; tid < tour.size(); ++tid) {
        // Segment prev = flip_led(tid_to_led(tour, tour[tid - 1]), tid_to_flip(orient, tour[tid - 1]));
        Segment curr = flip_led(tid_to_led(tour, tour[tid]), tid_to_flip(orient, tour[tid]));
        Segment next = flip_led(tid_to_led(tour, tour[(tid + 1) % tour.size()]), tid_to_flip(orient, tour[(tid + 1) % tour.size()]));

        segments.push_back({curr.first, curr.second, true, false});
        segments.push_back({curr.second, next.first, false, false});
    }

    // Segment lst = flip_led(tid_to_led(tour, tour[tour.size() - 1]), tid_to_flip(orient, tour[tour.size() - 1]));
    // segments.push_back({lst.first, lst.second, true, false});
    // segments.push_back({lst.second, out, false, false});
    
    return segments;
}

// TODO
// From cuts generates subsequences of LEDs for each IOPair and gens segments
std::vector<std::vector<CutSegment>>
ExtendedTour::cuts_to_segments(const std::vector<size_t>& tour,
                               const std::vector<bool>& orient,
                               const CutsInfo& ci) const
{
    const auto& [cuts, offset, iops] = ci;

    // SubSequenceS
    std::vector<std::vector<size_t>> sss;
    for (size_t i = 0; i < cuts.size(); ++i) {  // one subsequence per hinge
        std::vector<size_t> ss;
        if (tour.size() > 0) {
            size_t curr = cuts[i];
            size_t next = cuts[(i+1) % cuts.size()];
            if (i == cuts.size() - 1 && next <= curr) next += tour.size();

            for (size_t j = curr; j < next; ++j) {
                size_t k = j % tour.size();
                ss.push_back(k);
            }
        }
        sss.push_back(ss);
    }

    // SubSequence SegmentS (look, one has to make their own fun)
    std::vector<std::vector<CutSegment>> ssss(sss.size());
    for (size_t i = 0; i < sss.size(); ++i) {
        auto iop = iops[(offset + i) % iops.size()];
        ssss[i] = subseq_to_segments(tour, orient, sss[i], iop);
    }

    return ssss;
}

double ExtendedTour::min_within_path_spacing(const std::vector<Segment>& ss) const {
    // auto cs_to_s = [](const CutSegment& cs) { return std::make_pair(cs.start, cs.end); };
    // auto cs_is_LED = [](const CutSegment& cs) { return cs.is_LED; };
    auto share_endpoints = [](const Segment& s1, const Segment& s2) {
        auto [s11, s12] = s1;
        auto [s21, s22] = s2;
        return (s11 == s21) || (s11 == s22) || (s12 == s21) || (s12 == s22);
    };    

    auto tr01 = transform_AAB(m_fi.bbox(), AAB<2, double>(v2d(0), v2d(1)));

    double min = std::numeric_limits<double>::max();
    v2d min_i1; v2d min_i2;
    v2d min_j1; v2d min_j2;
    // std::cout << "\nmin_within_path_spacing ";
    // for (auto [p1, p2] : ss) {
    //     std::cout << p1 << ' ' << p2 << '\n';
    // }
    // std::cout << ss.size() << '\n';
    for (size_t i = 0; i < ss.size(); ++i) {
        for (size_t j = i + 1; j < ss.size(); ++j) {
            if (!share_endpoints(ss[i], ss[j])) {
                double d = segment_to_segment_distance(ss[i], ss[j]);
                // if (d == 0) { std::cout << "\nwsp " << d / tr01.first << '\t' << ss[i].first << ' ' << ss[i].second << "; " << ss[j].first << ' ' << ss[j].second << '\n'; }
                if (d < min) { min = d; 
                    min_i1 = ss[i].first; min_i2 = ss[i].second;
                    min_j1 = ss[j].first; min_j2 = ss[j].second;
                }
            }
        }
    }
    // std::cout << "\nWithin chains: " << min / tr01.first << '\t'
    //     << min_i1 << ' ' << min_i2 << "; "
    //     << min_j1 << ' ' << min_j2 << '\n';
    return min;
}

// double ExtendedTour::min_all_trace_spacing(const std::vector<size_t>& tour,
//                                            const std::vector<bool>& orient,
//                                            const CutsInfo& ci) const
// {
//     auto ssss = cuts_to_segments(tour, orient, ci);
//     auto cs_to_s = [](const CutSegment& cs) { return std::make_pair(cs.start, cs.end); };
    
//     auto share_endpoints = [](const Segment& s1, const Segment& s2) {
//         auto [s11, s12] = s1;
//         auto [s21, s22] = s2;
//         return (s11 == s21) || (s11 == s22) || (s12 == s21) || (s12 == s22);
//     };

//     auto tr01 = transform_AAB(m_fi.bbox(), AAB<2, double>(v2d(0), v2d(1)));
//     // auto inv_tr01 = inv_transform_AAB(tr01);

//     // TODO: measure regular segment to segment distance too just for kicks
//     if (ssss.size() == 1) { // single hinge
//         double min = segment_to_segment_distance(
//                 cs_to_s(ssss[0].back()),
//                 cs_to_s(ssss[0].front())) / tr01.first;
//         // std::cout << "\nSINGLE PATH (BEF/AFT)\t" << min << '\t'
//         //     << min_within_path_spacing(ssss[0]) << '\n';
//         // min = min_within_path_spacing(ssss[0]);
//         return min;
//     }

//     double min = std::numeric_limits<double>::max();
//     // std::cout << "HEY\n";
//     for (size_t i = 0; i < ssss.size(); ++i) {
//         // for (size_t j = i; j < ssss.size(); ++j) {
//         for (size_t j = i + 1; j < ssss.size(); ++j) {
//             auto cs_is_LED = [](const CutSegment& cs) { return cs.is_LED; };
            
//             std::vector<CutSegment>& vcs1 = ssss[i];
//             (void) std::remove_if(vcs1.begin(), vcs1.end(), cs_is_LED);
//             std::vector<Segment> vs1(vcs1.size());
//             std::transform(vcs1.begin(), vcs1.end(), vs1.begin(), cs_to_s);
            
//             std::vector<CutSegment>& vcs2 = ssss[j];
//             (void) std::remove_if(vcs2.begin(), vcs2.end(), cs_is_LED); 
//             std::vector<Segment> vs2(vcs2.size());
//             std::transform(vcs2.begin(), vcs2.end(), vs2.begin(), cs_to_s);

//             for (size_t ii = 0; ii < vs1.size(); ++ii) {
//                 for (size_t jj = 0; jj < vs2.size(); ++jj) {
//                     if (!share_endpoints(vs1[ii], vs2[jj])) {
//                         double d = segment_to_segment_distance(vs1[ii], vs2[jj]);
//                         if (d < min) { min = d; }
//                     }
//                 }
//             }

//             // std::cout << vs1.size() << '\t' << vs2.size() << '\n';
//             // for (const auto& s1 : vs1) {
//             //     for (const auto& s2 : vs2) {
//             //         double d = segment_to_segment_distance(s1, s2);
//             //         if (d < min) { min = d; } 
//             //         // std::cout << d << '\t';
//             //         // std::cout << s1.first << ' ' << s1.second << '\t';
//             //         // std::cout << s2.first << ' ' << s2.second << '\n';
//             //     }
//             // }
//         }
//     }

//     return min / tr01.first;
// }

double ExtendedTour::min_all_trace_spacing_v2(const std::vector<size_t>& tour,
                                              const std::vector<bool>& orient,
                                              const CutsInfo& ci) const
{
    
    auto cs_to_s = [](const CutSegment& cs) { return std::make_pair(cs.start, cs.end); };
    auto cs_is_LED = [](const CutSegment& cs) { return cs.is_LED; };
    auto share_endpoints = [](const Segment& s1, const Segment& s2) {
        auto [s11, s12] = s1;
        auto [s21, s22] = s2;
        return (s11 == s21) || (s11 == s22) || (s12 == s21) || (s12 == s22);
    };
    
    auto ssss = cuts_to_segments(tour, orient, ci);
    std::vector<std::vector<Segment>> ssss_no_led;
    for (size_t i = 0; i < ssss.size(); ++i) {
        std::vector<Segment> sss_no_led;
        for (size_t j = 0; j < ssss[i].size(); ++j) {
            if (!cs_is_LED(ssss[i][j])) { sss_no_led.push_back(cs_to_s(ssss[i][j])); }
        }
        ssss_no_led.push_back(sss_no_led);
    } 
    // std::cout << "ssss\n";
    // for (size_t i = 0; i < ssss.size(); ++i) {
    //     std::cout << i << ' ' << ssss[i].size() << '\n';
    // }
    // std::cout << "ssss_no_led\n";
    // for (size_t i = 0; i < ssss_no_led.size(); ++i) {
    //     std::cout << i << ' ' << ssss_no_led[i].size() << '\n';
    // }
    auto tr01 = transform_AAB(m_fi.bbox(), AAB<2, double>(v2d(0), v2d(1)));

    std::pair<Segment, Segment> min_pair;

    double min = std::numeric_limits<double>::max();
    for (size_t i = 0; i < ssss_no_led.size(); ++i) {
        for (size_t j = i + 1; j < ssss_no_led.size(); ++j) {
            const auto& vs1 = ssss_no_led[i];
            const auto& vs2 = ssss_no_led[j];

            for (size_t ii = 0; ii < vs1.size(); ++ii) {
                for (size_t jj = 0; jj < vs2.size(); ++jj) {
                    if (!share_endpoints(vs1[ii], vs2[jj])) {
                        double d = segment_to_segment_distance(vs1[ii], vs2[jj]);
                        
                        // if (d == 0) { std::cout << "isp " << d / tr01.first << '\t' << vs1[ii].first << ' ' << vs1[ii].second << "; " << vs2[jj].first << ' ' << vs2[jj].second; }
                        if (d < min) { 
                            min = d;
                            min_pair = {vs1[ii], vs2[jj]};
                        }
                    }
                }
            }
        }
    }

    // std::cout << "Min pairs:\n";
    // std::cout << "Between chains: " << min / tr01.first << '\t'
    //     << min_pair.first.first << ' ' << min_pair.first.second << "; "
    //     << min_pair.second.first << ' ' << min_pair.second.second << '\n';

    for (size_t i = 0; i < ssss_no_led.size(); ++i) {
        double d = min_within_path_spacing(ssss_no_led[i]);
        if (d < min) { min = d; }
    }

    return min / tr01.first;
}

std::tuple<size_t, double, double>
ExtendedTour::no_hinges_cost(const std::vector<size_t>& tour,
                             const std::vector<bool>& orient) const
{
    std::vector<CutSegment> all_segments = tour_to_segments(tour, orient);
    // std::cout << '\n';
    // for (auto [st, nd, a, b] : all_segments) {
    //     std::cout << st << ' ' << nd << '\n';
    // }
    auto tr01 = transform_AAB(m_fi.bbox(), AAB<2, double>(v2d(0), v2d(1)));

    std::vector<Segment> segs_inter;
    std::vector<Segment> segs_len;
    for (const auto& [start, end, is_LED, is_detour] : all_segments) {
        if (!is_LED) {
            Segment s = std::make_pair(start, end);
            segs_len.push_back(s);
            if (!is_detour) {
                segs_inter.push_back(s);
            }
        }
    }

    return std::make_tuple(count_segment_intersections(segs_inter), segments_length(segs_len), min_within_path_spacing(segs_len) / tr01.first);
}


std::tuple<size_t, double, double>
ExtendedTour::cuts_cost(const std::vector<size_t>& tour,
                        const std::vector<bool>& orient, const CutsInfo& ci) const
{
    std::vector<std::vector<CutSegment>> all_segments = cuts_to_segments(tour, orient, ci);
   
    std::vector<Segment> segs_inter;
    std::vector<Segment> segs_len;
    for (const auto& cut_segments : all_segments) {
        for (const auto& [start, end, is_LED, is_detour] : cut_segments) {
            if (!is_LED) {
                Segment s = std::make_pair(start, end);
                segs_len.push_back(s);
                if (!is_detour) {
                    segs_inter.push_back(s);
                }
            }
        }
    }

    return std::make_tuple(count_segment_intersections(segs_inter), segments_length(segs_len), min_all_trace_spacing_v2(tour, orient, ci));
}

CutsInfo ExtendedTour::solve_cuts(const std::vector<size_t>& tour, std::vector<bool>& orient,
        double mm_from_edge, double min_trace_spacing,
        std::tuple<size_t, double, double>& min_cost, double& max_spacing,
        bool& found_ok_iosp, double min_io_spacing)
{
    // convert i \in [0, 3] to pair of booleans representing binary combinations
    auto id_to_bool_pair = [](size_t i) {
        assert(i < 4);
        if (i == 0) { return std::make_pair(false, false); }
        else if (i == 1) { return std::make_pair(false, true); }
        else if (i == 2) { return std::make_pair(true, false); }
        else if (i == 3) { return std::make_pair(true, true); }
        return std::make_pair(false, false);
    };
    // changes
    auto bool_pair_to_is_narrow = [this](std::pair<bool, bool> bp, size_t hid) {
        std::vector<std::pair<bool, bool>> result(m_fi.is_narrow);
        assert(hid < result.size());
        result[hid] = bp;
        return result;
    };
    // cid -> position of cut in cuts vector
    // offset -> wrt the first hinge edge
    auto cut_id_to_eid = [this](size_t cid, size_t offset, size_t num_cuts) {
        size_t num_hinge_edges = 0;
        size_t target = (cid + offset) % num_cuts;
        for (size_t i = 0; i < m_fi.vertices.size(); ++i) {
            if (m_fi.is_hinge_edge[i]) {
                if (num_hinge_edges == target) { return i; }
                ++num_hinge_edges;
            }
        }
        assert(false);
        return m_fi.vertices.size();
    };
    auto cuts_to_is_detour = [&cut_id_to_eid, this](const std::vector<size_t>& cuts, size_t offset) {
        std::vector<bool> result(m_fi.vertices.size(), false);

        for (size_t i = 0; i < cuts.size(); ++i) {
            size_t n = (i + 1) % cuts.size();
            size_t eid_i = cut_id_to_eid(i, offset, cuts.size());
            // size_t eid_n = cut_id_to_eid(n, offset, cuts.size());
            
            if (cuts[i] == cuts[n]) {
                result[eid_i] = true;
            }
        }

        // std::vector<bool> result(cuts.size());
        // for (size_t i = 0; i < cuts.size(); ++i) {
        //     size_t offset_i = (offset + i) % cuts.size();
        //     size_t offset_n = (offset_i + 1) % cuts.size();
        //     result[i] = cuts[offset_i] == cuts[offset_n];
        // }
        return result;
    };

    auto update_best = [](const std::tuple<size_t, double, double>& cost, 
                          std::tuple<size_t, double, double>& best,
                          double min_spacing, double& best_spacing) {
        auto [ci, cl, cs] = cost; // c -> current
        auto [bi, bl, bs] = best; // b -> best

        // updates max spacing with no intersections
        if (ci == 0 && cs > best_spacing) { best_spacing = cs; }

        if (bi < ci) { return false; }
        // here bi >= ci

        if (cs >= min_spacing) {
            if (bs >= min_spacing) { // both over, minimize (inter, length)
                if (std::make_pair(ci, cl) < std::make_pair(bi, bl)) {
                    best = cost;
                    return true;
                } else { return false; }
            } else {                 // curr over, curr becomes best
                if (ci <= bi) {
                    best = cost;
                    return true;
                } else { return false; }
            }
        } else {
            if (bs >= min_spacing) { // best over, best stays best
                if (ci < bi) {
                    best = cost;
                    return true;
                } else { return false; }
            } else {                 // none over, minimize (inter, -spacing)
                if (std::make_pair(ci, -cs) < std::make_pair(bi, -bs)) {
                    best = cost;
                    return true;
                } else { return false; }
            }
        }

        return false;
    };

    auto update_best_v2 = [](const std::tuple<size_t, double, double>& cost, 
                          std::tuple<size_t, double, double>& best,
                          double min_spacing, double& best_spacing) {
        auto [ci, cl, cs] = cost; // c -> current
        auto [bi, bl, bs] = best; // b -> best

        // updates max spacing with no intersections
        if (ci == 0 && cs > best_spacing) { best_spacing = cs; }

        if (ci < bi || (ci == bi && cs >= bs)) {
            best = cost;
            // best_spacing = cs;
            return true;
        }

        return false;
    };

    auto is_ok_solution = [](const std::tuple<size_t, double, double>& cost, double min_spacing) {
        auto [inter, len, space] = cost;
        return inter == 0 && space >= min_spacing;
    };
    
    size_t num_hinges = std::count(m_fi.is_hinge_edge.begin(), m_fi.is_hinge_edge.end(), true);
    // std::vector<IOPair> iops = compute_io_pairs(mm_from_edge);
    min_cost = std::make_tuple(std::numeric_limits<size_t>::max(),
                               std::numeric_limits<double>::max(), 0.0);
    
    if (m_leds.size() == 0) {
        std::vector<size_t> cuts(num_hinges, 0);
        size_t offset = 0;
        auto iops = compute_io_pairs(mm_from_edge,
            std::vector<std::pair<bool, bool>>(num_hinges, {false, false}),
            std::vector<bool>(num_hinges, true));
        auto cost = cuts_cost(tour, orient, {cuts, offset, iops});
        
        // update min_cost, max_spacing w/ update_best
        bool is_best = update_best_v2(cost, min_cost, min_trace_spacing, max_spacing);
        return {cuts, offset, iops};
    }


    // initialize variables
    std::vector<size_t> cuts(num_hinges, 0);
    std::vector<size_t> min_cuts = cuts;
    size_t min_offset = 0;
    // min is_narrow, is_detour
    std::vector<std::pair<bool, bool>> min_n;
    std::vector<bool> min_d;

    size_t wide_id = 0;
    while (wide_id < m_fi.vertices.size()) {
        if (m_fi.is_hinge_edge[wide_id] && m_fi.is_half_hinge_edge[wide_id] && m_fi.is_deeper_than_neighbor[wide_id]) {
            break;
        }
        ++wide_id;
    }

    // std::cout << "\nSOLVE CUTS (" << num_hinges << " hinge(s))\n"; 

    // find best cut over spacing threshold
    found_ok_iosp = false;
    double best_io_spacing = 0.0;
    CutsInfo best_ios_ci;
    std::vector<std::pair<bool, bool>> min_ios_n;
    std::vector<bool> min_ios_d;
    double final_io_spacing = 0.0;

    auto segs = sol_to_segments(tour, orient);
    // std::cout << "SEGMENTS\n";
    // for (const auto& [p1, p2] : segs) {
    //     std::cout << p1 << '\t' << p2 << '\n';
    // }
    // std::cout << '\n';

    do {
        // std::cout << "CUTS\t";
        // for (auto c : cuts) { std::cout << c << ' '; } std::cout << '\n';
        size_t offset = 0;
        do {
            // std::cout << "\tOFF " << offset << '\n';

            std::vector<std::pair<std::vector<std::pair<bool, bool>>, std::vector<bool>>> narrow_and_detour;
            auto detour = cuts_to_is_detour(cuts, offset);
            if (wide_id == m_fi.vertices.size()) { // no wide hinge to this triangle
                narrow_and_detour.push_back({m_fi.is_narrow, detour});
            } else { // wide hinge at ID
                for (size_t i = 0; i < 4; ++i) {
                    // auto bp = id_to_bool_pair(i);
                    // std::cout << "\nBP " << i << " " << bp.first << ' ' << bp.second;
                    // auto narrow_pair = bool_pair_to_is_narrow(id_to_bool_pair(i), wide_id);
                    // std::cout << "\nNARROW " << i << ' ' << wide_id << '\t'; 
                    // for (auto [b1, b2] : narrow_pair) {
                    //     std::cout << b1 << ' ' << b2 << ' ';
                    // }
                    // std::cout << '\n';
                    narrow_and_detour.push_back({bool_pair_to_is_narrow(id_to_bool_pair(i), wide_id), detour});
                }
            }

            // std::cout << '\n';
            // std::cout << "narrow_and_detour.size() " << narrow_and_detour.size() << '\n';
            for (const auto& [n, d] : narrow_and_detour) {
                // std::cout << "\tN ";
                // for (auto [vn1, vn2] : n) { std::cout << "(" << vn1 << ", " << vn2 << ") "; }
                // std::cout << "\tD ";
                // for (auto vd : d) { std::cout << vd << ' '; }

                std::vector<IOPair> iops = compute_io_pairs(mm_from_edge, n, d);

                // BEGIN NEW
                double io_spacing = min_iocs_spacing(tour, orient, {cuts, offset, iops});
                // std::cout << "iosp = " << io_spacing << '\t';
                if (io_spacing > best_io_spacing) {
                    best_io_spacing = io_spacing;
                    min_ios_n = n;
                    min_ios_d = d;
                    best_ios_ci = {cuts, offset, iops};
                }
                if (io_spacing < min_io_spacing) { 
                    // std::cout << "bad\n";
                    continue;
                }
                // std::cout << "good";
                found_ok_iosp = true;
                // END NEW

                // std::cout << "cuts\t\t";
                // for (auto c : best_ios_ci.cuts) { std::cout << c << ' '; } std::cout << '\n';

                auto cost = cuts_cost(tour, orient, {cuts, offset, iops});
                const auto& [inter, len, space] = cost;
                // std::cout << '\t' << inter << ' ' << len << ' ' << space;

                bool is_best = update_best_v2(cost, min_cost, min_trace_spacing, max_spacing);
                if (is_best) {
                    // std::cout << "\tBEST";
                    final_io_spacing = io_spacing;
                    min_cuts = cuts;
                    min_offset = offset;
                    min_n = n;
                    min_d = d;
                }
                // if (is_ok_solution(cost, min_trace_spacing)) {
                //     goto exit;
                // }
                // std::cout << '\n';
            }
        } while (m_leds.size() > 0 && ++offset < cuts.size());  // why m_leds.size() > 0?
    } while (next_cuts(cuts, m_leds.size()));
    // std::cout << '\n';

    // exit:

    // impossible to optimize LED orientation before cuts for a single LED
    if (m_leds.size() == 1) {   // cuts is 0 repeated #hinges times
        // flip single LED orientation and try to find a better cut
        bool found_better = false;
        orient[0] = !orient[0];

        size_t offset = 0;
        do {    // see what hinge should the LED be connected to
            // auto cost = cuts_cost(tour, orient, {min_cuts, offset, iops});
            // const auto& [inter, len, space] = cost;
            // const auto& [min_inter, min_len, min_space] = min_cost;

            // if (inter < min_inter && len < min_len && space >= min_trace_spacing) {
            //     found_better = true;
            //     min_cost = cost;
            //     min_offset = offset;
            // }

            std::vector<std::pair<std::vector<std::pair<bool, bool>>, std::vector<bool>>> narrow_and_detour;
            auto detour = cuts_to_is_detour(cuts, offset);
            if (wide_id == m_fi.vertices.size()) { // no wide hinge to this triangle
                narrow_and_detour.push_back({m_fi.is_narrow, detour});
            } else { // wide hinge at ID
                for (size_t i = 0; i < 4; ++i) {
                    narrow_and_detour.push_back({bool_pair_to_is_narrow(id_to_bool_pair(i), wide_id), detour});
                }
            }

            for (const auto& [n, d] : narrow_and_detour) {
                std::vector<IOPair> iops = compute_io_pairs(mm_from_edge, n, d);

                // BEGIN NEW
                double io_spacing = min_iocs_spacing(tour, orient, {cuts, offset, iops});
                // std::cout << "iosp = " << io_spacing << '\n';
                if (io_spacing > best_io_spacing) {
                    best_io_spacing = io_spacing;
                    min_ios_n = n;
                    min_ios_d = d;
                    best_ios_ci = {cuts, offset, iops};
                }
                if (io_spacing < min_io_spacing) { continue; }
                found_ok_iosp = true;
                // END NEW

                auto cost = cuts_cost(tour, orient, {cuts, offset, iops});
                const auto& [inter, len, space] = cost;
                // std::cout << '\t' << inter << ' ' << len << ' ' << space;

                bool is_best = update_best_v2(cost, min_cost, min_trace_spacing, max_spacing);
                if (is_best) {
                    // std::cout << "\tBEST";
                    final_io_spacing = io_spacing;
                    min_cuts = cuts;
                    min_offset = offset;
                    min_n = n;
                    min_d = d;
                }
                // std::cout << '\n';
            }

        } while (offset++ < cuts.size());

        if (!found_better) { orient[0] = !orient[0];; } // undo if nothing better
    }

    // auto css = cuts_to_segments(tour, orient, {min_cuts, min_offset, compute_io_pairs(mm_from_edge, min_n, min_d)});
    // auto cs_to_s = [](const CutSegment& cs) { return std::make_pair(cs.start, cs.end); };
    // auto cs_is_LED = [](const CutSegment& cs) { return cs.is_LED; };
    
    // std::cout << "Final segments\n";
    // for (size_t i = 0; i < css.size(); ++i) {
    //     std::vector<CutSegment>& vcs1 = css[i];
    //     (void) std::remove_if(vcs1.begin(), vcs1.end(), cs_is_LED);
    //     std::vector<Segment> vs1(vcs1.size());
    //     std::transform(vcs1.begin(), vcs1.end(), vs1.begin(), cs_to_s);
    //     for (size_t j = 0; j < vs1.size(); ++j) {
    //         auto [v1, v2] = vs1[j];
    //         std::cout << v1 << '\t' << v2 << '\n';
    //     }
    // }

    std::cout << "FOUND SOL: " << found_ok_iosp << '\n';
    std::cout << "BEST IOSp: " << best_io_spacing << '\n';

    if (!found_ok_iosp) {
        m_fi.is_narrow = min_ios_n;
        m_fi.is_detour = min_ios_d;
        auto cost = cuts_cost(tour, orient, best_ios_ci);
        update_best_v2(cost, min_cost, min_trace_spacing, max_spacing);
        final_io_spacing = best_io_spacing;
        m_io_spacing = final_io_spacing;
        m_best_io_spacing = best_io_spacing;
        return best_ios_ci;
    }

    m_fi.is_narrow = min_n;
    m_fi.is_detour = min_d;
    m_io_spacing = final_io_spacing;
    m_best_io_spacing = best_io_spacing;
    return {min_cuts, min_offset, compute_io_pairs(mm_from_edge, min_n, min_d)};
}

void ExtendedTour::fix_intersections(std::vector<std::vector<CutSegment>>& css, const std::vector<IOPair>& iops) const {
    assert(css.size() == 1);
    auto& cs = css[0];
    std::vector<Segment> intersectable_segments;
    for (const CutSegment& s : cs) {
        if (!s.is_LED && !s.is_detour) {
            intersectable_segments.push_back(std::make_pair(s.start, s.end)); 
        }
    }
    // get pairs of intersecting segments
    auto intersections = list_segment_intersections(intersectable_segments);

    if (intersections.size() == 1) {
        auto [i, j] = intersections.front();
        if (i == 0) {
            auto detour = detour_io_segment(iops, intersectable_segments[i]);
            cs.erase(cs.begin());
            cs.insert(cs.begin(), detour.begin(), detour.end());
        } else if (j == intersections.size() - 1) {
            auto detour = detour_io_segment(iops, intersectable_segments[i]);
            cs.pop_back();
            cs.insert(cs.end(), detour.begin(), detour.end());
        } else {
            std::cerr << "[ERROR] Intersections between segments other than the first and the last are not handled.\n";
            // output_dat("/home/marco/routing_dump.dat");
            exit(EXIT_FAILURE);
        }
    } else { assert(false); }
}

// flip orientation of modules in triangle info
void ExtendedTour::flip_modules(const std::vector<size_t>& tour, const std::vector<bool>& orient) {
    for (size_t tid = 0; tid < tour.size(); ++tid) {
        // CAUTION: HACK to get connector if there is one
        if (m_fi.has_connector) {
            auto& mod = tour.at(tid) == 0 ? m_fi.connector :
                                            m_fi.modules[tour.at(tid) - 1];
            if (orient.at(tid)) {
                auto [p0, p1, p2] = mod;
                mod = std::make_tuple(p1 + p2 - p0, p2, p1);
            }
        } else {
            auto& mod = m_fi.modules[tour.at(tid)];
            if (orient.at(tid)) {
                auto [p0, p1, p2] = mod;
                mod = std::make_tuple(p1 + p2 - p0, p2, p1);
            }
        }
    }
}

std::tuple<size_t, double, double>
ExtendedTour::solve_orientation_permutation_cuts(double mm_from_edge, double min_trace_spacing, double& best_spacing) {
    if (m_leds.size() == 0) {
        std::cerr << "[WARNING] 0 LEDs in the triangle.\n";
        // exit(EXIT_FAILURE);
    }

    std::vector<size_t> tour;
    std::vector<bool> orientation;

    if (m_leds.size() >= 1) {
        // SOLVE PERMUTATION AND ORIENTATION
        bool use_concorde = m_leds.size() >= m_settings.concorde_led_threshold;
        if (use_concorde) { // CONCORDE
            tour = solve_permutation_concorde(m_leds);
            reverse_tour_if_not_ccw(m_leds, tour, orientation, true);
            orientation = solve_orientation_stability(m_leds, tour);
        } else {                                       // BRUTE FORCE (EXHAUSTIVE)
            assert(false);
            auto sol = solve_orientation_permutation_exhaustive(m_leds);
            tour = sol.first; orientation = sol.second;
        }
        // reverse tour if not CCW
        if (m_leds.size() != 1) {
            // assert(use_concorde);
            bool flipped = !reverse_tour_if_not_ccw(m_leds, tour, orientation, false);
            /* If concorde has been used, the (center) polygon should already be
            * in CCW order. If the assert fails, it means that the center and real
            * polygons have different orientations.
            */
            assert(!(use_concorde && flipped));
        }
    }

    // std::cout << "Tour:\t\t\t\t\t";
    // for (auto t : tour) { std::cout << t << ' '; }
    // std::cout << "\nOrientation:\t\t\t\t";
    // for (auto o : orientation) { std::cout << o << ' ';}
    // std::cout << '\n';

    // m_tour = tour;
    // m_orientation = orientation;

    auto tr01 = transform_AAB(m_fi.bbox(), AAB<2, double>(v2d(0), v2d(1)));
    auto tr01_inv = inv_transform_AAB(tr01);
    std::tuple<size_t, double, double> min_cost;
    size_t num_hinges = std::count(m_fi.is_hinge_edge.begin(), m_fi.is_hinge_edge.end(), true);
    std::cout << "num_hinges == " << num_hinges << '\n';
    if (num_hinges == 0) {
        auto segs = tour_to_segments(tour, orientation);
        min_cost = no_hinges_cost(tour, orientation);
        const auto& [inter, len, space] = min_cost;
        best_spacing = space;

        // Routing info
        std::cout << "Tour:\t\t\t\t\t";
        for (auto t : tour) { std::cout << t << ' '; }
        std::cout << "\nOrientation:\t\t\t\t";
        for (auto o : orientation) { std::cout << o << ' ';}
        std::cout << '\n';
        // std::cout << "\n(Offset; cuts)\t\t\t\t" << offset << "; ";
        // for (auto c : cuts) { std::cout << c << ' ';} std::cout << "\n";
        std::cout << "(Intersections, Length, Spacing)\t" << inter << ", " << len << ", " << space << "\n";
        // std::cout << "Best spacing at 0 intersections:\t" << best_spacing << '\n';
        if (space < min_trace_spacing) {
            std::cerr << "[WARNING] All cuts are below the trace spacing threshold.\n";
            m_found_spacing_io = false;
            // exit(EXIT_FAILURE);
        } else {
            m_found_spacing_io = true;
        }

        for (const auto& [start, end, is_LED, is_detour] : segs) {
            if (!is_LED) {
                m_fi.traces.push_back(apply_transform_AAB(tr01_inv, std::make_pair(start, end)));
            }
        }
    } else {
        // SOLVE CUTS
        min_cost = std::make_tuple(std::numeric_limits<size_t>::max(),
                                   std::numeric_limits<double>::max(), 0.0);
        bool found_ok_iosp = false;
        auto ci = solve_cuts(tour, orientation, mm_from_edge, min_trace_spacing, min_cost, best_spacing, found_ok_iosp, m_settings.min_io_spacing_mm);
        m_found_spacing_io = found_ok_iosp;

        const auto& [cuts, offset, iops] = ci;
        const auto& [inter, len, space] = min_cost;

        // Routing info
        std::cout << "Tour:\t\t\t\t\t";
        for (auto t : tour) { std::cout << t << ' '; }
        std::cout << "\nOrientation:\t\t\t\t";
        for (auto o : orientation) { std::cout << o << ' ';}
        std::cout << "\n(Offset; cuts)\t\t\t\t" << offset << "; ";
        for (auto c : cuts) { std::cout << c << ' ';} std::cout << "\n";
        std::cout << "(Intersections, Length, Spacing)\t" << inter << ", " << len << ", " << space << "\n";
        std::cout << "Best spacing at 0 intersections:\t" << best_spacing << '\n';
        if (best_spacing < min_trace_spacing) {
            std::cerr << "[WARNING] All cuts are below the trace spacing threshold.\n";
            // exit(EXIT_FAILURE);
        }

        // MODIFY TRIANGLE INFO (traces, modules)
        std::vector<std::vector<CutSegment>> all_segments = cuts_to_segments(tour, orientation, ci);
        // if there is a single intersection and a single hinge, try to fix it
        // if (std::get<0>(min_cost) == 1 && all_segments.size() == 1) {
        //     fix_intersections(all_segments, ci.iops);
        // }

        // traces
        m_fi.traces.clear();
        for (const auto& cs : all_segments) {
            for (const auto& [start, end, is_LED, is_detour] : cs) {
                if (!is_LED) {
                    m_fi.traces.push_back(apply_transform_AAB(tr01_inv, std::make_pair(start, end)));
                }
            }
        }
    }

    // modules
    flip_modules(tour, orientation);

    m_tour = tour;
    m_orientation = orientation;

    return min_cost;
}

std::tuple<std::vector<size_t>, std::vector<bool>, std::optional<CutsInfo>>
ExtendedTour::exhaustive_pipeline(const std::vector<Led>& leds, double mm_from_edge, double min_trace_spacing, std::tuple<size_t, double, double>& cuts_cost) {
    auto sols = solve_orientation_permutation_exhaustive_all(leds);
    std::cout << "#sols without intersections: " << sols.size() << '\n';
    double spacing = 0.0;
    double best_spacing = 0.0;
    cuts_cost = std::make_tuple(std::numeric_limits<size_t>::max(),
                                std::numeric_limits<double>::max(), 0.0);
    
    auto tr01 = transform_AAB(m_fi.bbox(), AAB<2, double>(v2d(0), v2d(1)));
    // auto tr01_inv = inv_transform_AAB(tr01);
    size_t num_hinges = std::count(m_fi.is_hinge_edge.begin(), m_fi.is_hinge_edge.end(), true);
    
    CutsInfo ci;
    size_t k = 0;

    bool found_any_ok_iosp = false;

    do {
        // std::cout << "Hey\n";
        auto& [tour, orient, cost] = sols[k++];
        bool found_ok_iosp = false;
        
        if (num_hinges == 0) {
            auto segs = tour_to_segments(tour, orient);
            cuts_cost = no_hinges_cost(tour, orient);
            // const auto& [inter, len, space] = cuts_cost;
            spacing = std::get<2>(cuts_cost);
            found_ok_iosp = spacing > m_settings.min_io_spacing_mm;
        } else {
            ci = solve_cuts(tour, orient, mm_from_edge, min_trace_spacing, cuts_cost, spacing, found_ok_iosp, m_settings.min_io_spacing_mm);
        }
        found_any_ok_iosp = found_any_ok_iosp || found_ok_iosp;

        // auto [inter, len, space] = cuts_cost;
        if (spacing > best_spacing) {
            best_spacing = spacing;
        }
        // if (inter == 0 && space >= min_trace_spacing) {
        //     std::cout << "Tour:\t\t\t\t\t";
        //     for (auto t : tour) { std::cout << t << ' '; }
        //     std::cout << "\nOrientation:\t\t\t\t";
        //     for (auto o : orient) { std::cout << o << ' ';}
        //     std::cout << "\n(Offset; cuts)\t\t\t\t" << ci.offset << "; ";
        //     for (auto c : ci.cuts) { std::cout << c << ' ';}
        //     std::cout << "\n(Intersections, Length, Spacing)\t" << inter << ", " << len << ", " << space << '\n';
        // }

    } while (k < sols.size() &&
            (std::get<0>(cuts_cost) != 0 || spacing < min_trace_spacing));

    if (k == sols.size() && (std::get<0>(cuts_cost) != 0 || best_spacing < min_trace_spacing)) {
        std::cerr << "[WARNING] No 0 intersection solution for tour and orientation "
            "provides cuts with trace spacing over the minimum threshold.\n";
        std::cerr << "Best spacing found:\t" << best_spacing << '\n';
        // exit(EXIT_FAILURE);
    }

    m_found_spacing_io = found_any_ok_iosp;

    const auto& [tour, orient, cost] = sols[--k];
    
    std::optional<CutsInfo> ci_opt{};
    if (num_hinges > 0) { ci_opt = ci; }

    return std::make_tuple(tour, orient, ci_opt);

    // if (m_leds.size() != 1) {
    //     bool flipped = !reverse_tour_if_not_ccw(m_leds, tour, orient, false);
    //     /* If concorde has been used, the (center) polygon should already be
    //      * in CCW order. If the assert fails, it means that the center and real
    //      * polygons have different orientations.
    //      */
    //     assert(!(use_concorde && flipped));
    // }
}

std::tuple<size_t, double, double> ExtendedTour::solve_orientation_permutation_cuts_all(double mm_from_edge, double min_trace_spacing, double& best_spacing) {
    if (m_leds.size() == 0) {
        std::cerr << "[WARNING] 0 LEDs in the triangle.\n";
        // exit(EXIT_FAILURE);
    }
    
    auto min_cost = std::make_tuple(std::numeric_limits<size_t>::max(), std::numeric_limits<double>::max(), 0.0);

    const auto& [tour, orientation, ci] = exhaustive_pipeline(m_leds, mm_from_edge, min_trace_spacing, min_cost);
    
    auto tr01 = transform_AAB(m_fi.bbox(), AAB<2, double>(v2d(0), v2d(1)));
    auto tr01_inv = inv_transform_AAB(tr01);

    if (ci.has_value()) {
        const auto& [cuts, offset, iops] = ci.value();
        const auto& [inter, len, space] = min_cost;

        // Routing info
        std::cout << "Tour:\t\t\t\t\t";
        for (auto t : tour) { std::cout << t << ' '; }
        std::cout << "\nOrientation:\t\t\t\t";
        for (auto o : orientation) { std::cout << o << ' ';}
        std::cout << "\n(Offset; cuts)\t\t\t\t" << offset << "; ";
        for (auto c : cuts) { std::cout << c << ' ';} std::cout << "\n";
        std::cout << "(Intersections, Length, Spacing)\t" << inter << ", " << len << ", " << space << "\n";
        std::cout << "Best spacing at 0 intersections:\t" << space << '\n';
        if (space < min_trace_spacing) {
            std::cerr << "[WARNING] All cuts are below the trace spacing threshold.\n";
            m_found_spacing_io = false;
            // exit(EXIT_FAILURE);
        } else {
            m_found_spacing_io = true;
        }

        // MODIFY TRIANGLE INFO (traces, modules)
        std::vector<std::vector<CutSegment>> all_segments = cuts_to_segments(tour, orientation, ci.value());
        // if there is a single intersection and a single hinge, try to fix it
        // if (std::get<0>(min_cost) == 1 && all_segments.size() == 1) {
        //     fix_intersections(all_segments, ci.iops);
        // }

        // traces
        m_fi.traces.clear();
        for (const auto& cs : all_segments) {
            for (const auto& [start, end, is_LED, is_detour] : cs) {
                if (!is_LED) {
                    m_fi.traces.push_back(apply_transform_AAB(tr01_inv, std::make_pair(start, end)));
                }
            }
        }
    } else {
        auto segs = tour_to_segments(tour, orientation);
        const auto& [inter, len, space] = min_cost;

        // Routing info
        std::cout << "Tour:\t\t\t\t\t";
        for (auto t : tour) { std::cout << t << ' '; }
        std::cout << "\nOrientation:\t\t\t\t";
        for (auto o : orientation) { std::cout << o << ' ';}
        // std::cout << "\n(Offset; cuts)\t\t\t\t" << offset << "; ";
        // for (auto c : cuts) { std::cout << c << ' ';} std::cout << "\n";
        std::cout << "(Intersections, Length, Spacing)\t" << inter << ", " << len << ", " << space << "\n";
        // std::cout << "Best spacing at 0 intersections:\t" << best_spacing << '\n';
        if (space < min_trace_spacing) {
            std::cerr << "[WARNING] All cuts are below the trace spacing threshold.\n";
            m_found_spacing_io = false;
            // exit(EXIT_FAILURE);
        } else {
            m_found_spacing_io = true;
        }

        for (const auto& [start, end, is_LED, is_detour] : segs) {
            if (!is_LED) {
                m_fi.traces.push_back(apply_transform_AAB(tr01_inv, std::make_pair(start, end)));
            }
        }
    }
    
    // const auto& [inter, len, space] = min_cost;
    // best_spacing = space;

    // // Routing infos
    // std::cout << "FINAL\nTour:\t\t\t\t\t";
    // for (auto t : tour) { std::cout << t << ' '; }
    // std::cout << "\nOrientation:\t\t\t\t";
    // for (auto o : orientation) { std::cout << o << ' ';}
    // std::cout << "\n(Offset; cuts)\t\t\t\t" << offset << "; ";
    // for (auto c : cuts) { std::cout << c << ' ';} std::cout << "\n";
    // std::cout << "(Intersections, Length, Spacing)\t" << inter << ", " << len << ", " << space << "\n";
    // // std::cout << "Best spacing at 0 intersections:\t" << best_spacing << '\n';
    // if (best_spacing < min_trace_spacing) {
    //     std::cerr << "[WARNING] All cuts are below the trace spacing threshold.\n";
    //     // exit(EXIT_FAILURE);
    // }

    // // MODIFY TRIANGLE INFO (traces, modules)
    // std::vector<std::vector<CutSegment>> all_segments = cuts_to_segments(tour, orientation, ci);
    // // if there is a single intersection and a single hinge, try to fix it
    // // if (std::get<0>(min_cost) == 1 && all_segments.size() == 1) {
    // //     fix_intersections(all_segments, ci.iops);
    // // }

    // // traces
    // auto tr01 = transform_AAB(m_fi.bbox(), AAB<2, double>(v2d(0), v2d(1)));
    // auto tr01_inv = inv_transform_AAB(tr01);
    // m_fi.traces.clear();
    // for (const auto& cs : all_segments) {
    //     for (const auto& [start, end, is_LED, is_detour] : cs) {
    //         if (!is_LED) {
    //             m_fi.traces.push_back(apply_transform_AAB(tr01_inv, std::make_pair(start, end)));
    //         }
    //     }
    // }

    // // for OUTPUT_DAT display
    // // generates vector of segments from vector of non-LED CutSegments
    // auto proj = [](const std::vector<CutSegment>& cs) { 
    //     std::vector<Segment> segs;
    //     for (const auto& [start, end, is_LED, is_detour] : cs) {
    //         if (!is_LED) {
    //             segs.push_back(std::make_pair(start, end));
    //         }
    //     }
    //     return segs;
    // };
    // m_traces.resize(all_segments.size());
    // std::transform(all_segments.begin(), all_segments.end(), m_traces.begin(), proj);

    // modules
    flip_modules(tour, orientation);

    m_tour = tour;
    m_orientation = orientation;

    return min_cost;
}