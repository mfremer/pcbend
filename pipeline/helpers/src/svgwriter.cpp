#include "svgwriter.hpp"

std::string move_to(double x, double y, bool is_relative) {
    std::stringstream ss;
    ss << (is_relative ? "m " : "M ") << x << ' ' << y;
    return ss.str();
}
std::string line_to(double x, double y, bool is_relative) {
    std::stringstream ss;
    ss << (is_relative ? "l " : "L ") << x << ' ' << y;
    return ss.str();
}
std::string arc_to(double rx, double ry, double x_axis_rotation,
                   bool large_arc_flag, bool sweep_flag, double x, double y,
                   bool is_relative) {
    std::stringstream ss;
    ss << (is_relative ? "a " : "A ") << rx << ' ' << ry << ' ';
    ss << x_axis_rotation << ' ' << large_arc_flag << ' ' << sweep_flag << ' ';
    ss << x << ' ' << y; 
    return ss.str();
}
std::string arc_to(double r, bool large_arc_flag, bool sweep_flag, double x, double y) {
    return arc_to(r, r, 0.0, large_arc_flag, sweep_flag, x, y, true);
}
std::string close_path() {
    return "z";
}

Attribute to_attribute(const std::string& name, const std::string& value) {
    return std::make_pair(name, value);
}

// HELPER XML FUNCTIONS
std::string begin_element(const std::string& name) {
    return "<" + name;
}
std::string end_values() {
    return ">";
}
std::string end_element(const std::string& name = "") {
    if (name.empty()) { return "/>"; }
    return "</" + name + ">"; 
}
std::string attribute(const std::string& name, const std::string& value, const std::string& unit = "") {
    return name + "=\"" + value + unit + "\"";
}
std::string attribute(const Attribute& attr, const std::string& unit = "") {
    const auto& [name, value] = attr;
    return attribute(name, value, unit);
}

std::string SVGPath::to_string() const {
    std::stringstream ss;

    ss << begin_element("path") << SVG_SEP;
    for (const auto& attr : m_attributes) {
        ss << attribute(attr) << ' ';
    }
    ss << attribute("d", m_dvalue);
    ss << end_element();

    return ss.str();
}

void SVGPath::from_v2ds(const std::vector<std::vector<v2d>>& vs) {
    for (const auto& path : vs) {
        bool first = true;
        for (const auto& pt : path) {
            if (first) {
                first = false;
                *this << move_to(pt[0], pt[1], false) << SVG_SEP;
            } else {
                *this << line_to(pt[0], pt[1], false) << SVG_SEP;
            }
        }
    } 
}

SVGPath& SVGPath::operator<<(const std::string& cmd) {
    m_dvalue.append(cmd);
    return *this;
}

std::string SVGRect::to_string() const {
    std::stringstream ss;

    ss << begin_element("rect") << SVG_SEP;
    for (const auto& attr : m_attributes) {
        ss << attribute(attr) << ' ';
    }
    ss << attribute("width", std::to_string(m_w)) << SVG_SEP;
    ss << attribute("height", std::to_string(m_h)) << SVG_SEP;
    ss << attribute("rx", std::to_string(m_r)) << SVG_SEP;
    ss << attribute("transform",
                    "translate(" + std::to_string(m_x - 0.5 * m_w) + "," + std::to_string(m_y - 0.5 * m_h) + ") " +
                    "rotate(" + std::to_string(m_theta) + " " + std::to_string(0.5 * m_w) + " " + std::to_string(0.5 * m_h) + ")")
       << SVG_SEP;
    ss << end_element();
    
    return ss.str();
}

std::string SVGCircle::to_string() const {
    std::stringstream ss;

    ss << begin_element("circle") << SVG_SEP;
    for (const auto& attr : m_attributes) {
        ss << attribute(attr) << ' ';
    }
    ss << attribute("cx", std::to_string(m_x)) << SVG_SEP;
    ss << attribute("cy", std::to_string(m_y)) << SVG_SEP;
    ss << attribute("r",  std::to_string(0.5 * (m_hd + m_tw))) << SVG_SEP;
    ss << end_element();
    
    return ss.str();
}

std::string to_string(const GeometryElement& gel) {
    if (std::holds_alternative<SVGPath>(gel)) {
        const SVGPath& path = std::get<SVGPath>(gel);
        return path.to_string();
    } else if (std::holds_alternative<SVGRect>(gel)) {
        const SVGRect& rect = std::get<SVGRect>(gel);
        return rect.to_string();
    } else if (std::holds_alternative<SVGCircle>(gel)) {
        const SVGCircle& circ = std::get<SVGCircle>(gel);
        return circ.to_string();
    } else {
        std::cerr << "[ERROR] GeometryElement does not hold any alternative\n";
        exit(EXIT_FAILURE);
    }
}

void SVGFile::add_geometry_element(const GeometryElement& gel) {
    m_geometry_elements.push_back(gel);
}

void SVGFile::add_geometry_elements(const std::vector<GeometryElement>& gels) {
    m_geometry_elements.insert(m_geometry_elements.end(), gels.begin(), gels.end());
}

void SVGFile::to_file() const {
    std::ofstream file(m_file_name);
    if (!file.is_open()) {
        std::cerr << "[ERROR] Could not open file " << m_file_name << '\n';
        exit(EXIT_FAILURE);
    }

    // SVG ELEMENT
    file << begin_element("svg") << ' ';
    file << attribute("width", std::to_string(m_dimensions.first), "mm") << ' ';
    file << attribute("height", std::to_string(m_dimensions.second), "mm") << ' ';
    file << attribute("viewBox", std::to_string(0) + " " + std::to_string(0) + " " +
                                std::to_string(m_dimensions.first) + " " +
                                std::to_string(m_dimensions.second)) << ' ';
    file << attribute("xmlns", "http://www.w3.org/2000/svg");
    file << end_values() << '\n';

    if (m_flip_y_axis) {
        file << begin_element("g") << ' ';
        file << "transform=\"translate(0 " << m_dimensions.second << ") scale(1 -1)\"";
        file << end_values() << '\n'; 
    }
    // GEOMETRY
    for (const auto& gel : m_geometry_elements) {
        file << to_string(gel) << '\n';
    }

    if (m_flip_y_axis) {
        file << end_element("g") << '\n';
    }
    file << end_element("svg");
    file.close();
}