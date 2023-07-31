#pragma once

#include <LibSL/LibSL.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#define SVG_SEP " "

typedef std::pair<std::string, std::string> Attribute;

Attribute to_attribute(const std::string& name, const std::string& value);

// PATH COMMAND FUNCTIONS
std::string move_to(double x, double y, bool is_relative = true);
std::string line_to(double x, double y, bool is_relative = true);
std::string arc_to(double rx, double ry, double x_axis_rotation,
                   bool large_arc_flag, bool sweep_flag, double x, double y,
                   bool is_relative = true);
std::string arc_to(double r, bool large_arc_flag, bool sweep_flag, double x, double y);
std::string close_path();

// DIFFERENT GEOMETRY ELEMENTS
class SVGPath {
public:
    SVGPath() = default;
    SVGPath(const std::vector<Attribute>& attrs) : m_attributes(attrs), m_dvalue() {};
    SVGPath(const std::string& fill, const std::string& stroke, const std::string& style)
        : m_dvalue() {
        m_attributes = std::vector<Attribute>{
            std::make_pair("fill", fill),
            std::make_pair("stroke", stroke),
            std::make_pair("style", style)
        };
    }
    std::string to_string() const;
    std::string get_dvalue() const { return m_dvalue; }
    void from_v2ds(const std::vector<std::vector<v2d>>& vs);
    void add_attribute(const Attribute& attr) { m_attributes.push_back(attr); }
    SVGPath& operator<<(const std::string& cmd);
private:
    std::vector<Attribute> m_attributes;
    std::string m_dvalue;
};

class SVGRect {
public:
    SVGRect() = default;
    SVGRect(const std::vector<Attribute>& attrs)
        : m_attributes(attrs) {};
    SVGRect(const std::string& fill, const std::string& stroke, const std::string& style) {
        m_attributes = std::vector<Attribute>{
            std::make_pair("fill", fill),
            std::make_pair("stroke", stroke),
            std::make_pair("style", style)
        };
    }

    std::string to_string() const;
    void add_attribute(const Attribute& attr) { m_attributes.push_back(attr); }
    void set_params(double x, double y, double theta, double w, double h, double r) {
        m_w = w;
        m_h = h;
        m_r = r;
        m_x = x;
        m_y = y;
        m_theta = theta;
    }

private:
    std::vector<Attribute> m_attributes;
    // geometry
    double m_w;
    double m_h;
    double m_r;
    // transform
    double m_x;
    double m_y;
    double m_theta;
};

class SVGCircle {
public:
    SVGCircle() = default;
    SVGCircle(const std::vector<Attribute>& attrs)
        : m_attributes(attrs) {};
    SVGCircle(const std::string& fill, const std::string& stroke, const std::string& style) {
        m_attributes = std::vector<Attribute>{
            std::make_pair("fill", fill),
            std::make_pair("stroke", stroke),
            std::make_pair("style", style)
        };
    };

    std::string to_string() const;
    void add_attribute(const Attribute& attr) { m_attributes.push_back(attr); }
    void set_params(double pos_x, double pos_y, double hole_diameter, double trace_width) {
        m_x = pos_x;
        m_y = pos_y;
        m_hd = hole_diameter;
        m_tw = trace_width;
    }

private:
    std::vector<Attribute> m_attributes;
    double m_x;
    double m_y;
    double m_hd;    // hole diameter
    double m_tw;    // trace width
};

typedef std::variant<SVGPath, SVGRect, SVGCircle> GeometryElement;

// GEOMETRY ELEMENT VISITOR
std::string to_string(const GeometryElement& gel);

class SVGFile {
public:
    SVGFile(const std::string& file_name, double width, double height, bool flip_y_axis = true)
        : m_file_name(file_name), m_dimensions(std::make_pair(width, height)), m_flip_y_axis(flip_y_axis), m_geometry_elements() {};

    void add_geometry_element(const GeometryElement& gel);
    void add_geometry_elements(const std::vector<GeometryElement>& gels);
    void to_file() const;
private:
    std::string m_file_name;
    std::pair<double, double> m_dimensions;
    bool m_flip_y_axis = true;
    std::vector<GeometryElement> m_geometry_elements;
};