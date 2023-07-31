#pragma once

#include <algorithm>
#include <string>
#include <variant>
#include <vector>
#include <tuple>

typedef struct {
    double vd;  // via diameter
    double hd;  // hole diameter
    double x;
    double y;
} Via;

typedef struct {
    double x;
    double y;
    double w;
    double h;
    double r;
} Pad;

typedef struct {
    std::vector<std::pair<double, double>> polygon;
} KeepOutPolygon;
typedef struct {
    std::pair<double, double> position;
    double diameter;
} KeepOutCircle;

typedef std::variant<KeepOutPolygon, KeepOutCircle> KeepOutZone;

typedef struct {
    std::tuple<double, double, double> transform;   // posx, posy, rot
    std::vector<Pad> pads;
} Component;

typedef struct {
    std::pair<double, double> dims;
    std::pair<std::pair<double, double>, std::pair<double, double>> io;    
    std::vector<Component> comps;
    std::vector<std::vector<std::pair<double, double>>> traces;
    std::vector<Via> vias;
    std::vector<KeepOutZone> ko_zones;
    std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> ss_marks;
} ModuleInfo;

ModuleInfo parse_module_info(const std::string& file_name);
