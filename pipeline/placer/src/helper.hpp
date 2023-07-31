#pragma once

#include <FaceInfo.hpp>
#include <ModuleInfo.hpp>

// #include <global_parameters.hpp>

#include <LibSL/LibSL.h>
#include <LibSL/LibSL_gl.h>
#include <LibSL/LibSL_linalg.h>

#include <LibSL/Mesh/Mesh.h>
#include <LibSL/Mesh/MeshRenderer.h>

#include "HLBFGS.h"
#include "Lite_Sparse_Matrix.h"

#include "clipper.hpp"
#include <box2d.h>

#include "parsing_helpers.hpp"

#include <array>
#include <iostream>
#include <variant>
#include <CDT.h>

using namespace std;
using namespace LibSL;
using namespace LibSL::Mesh;

typedef Tuple<double, 5> v5d;

// declare vertex data
typedef struct
{
    v3f pos;
    v3f nrm;
    v2f uvs;
} t_VertexData;


enum class Validity {
	success,
	overlap,
	outOfBounds,
	tooSmall,
	notInsideWall,
    edgeLengthConstrainViolated
};

enum class SimState {
	Point,
	Rectangle,
	RigidBody,
	PointRigidBody
};

typedef std::array<v2d, 4> t_rect_geometry;

// sites = voronoi diagram seeds
typedef struct {
} t_indepedent_site;

typedef struct {
	double u;
	double v;
	int rid;
} t_rect_site;

typedef struct {
	int first_var_id;
	bool is_connector = false;
} t_rect;

typedef struct {
	int first_var_id;
	b2Body *body;
	float sx, sy; // length and breadth
	bool is_connector = false;
} t_rigid;

typedef struct {
	int site_id;
	int first_var_id;
	variant<t_indepedent_site, t_rect_site> nfo;
} t_site_nfo;

#define BOX2D_SCALE 100.0 // 10 is the best scale for BOX2D

void parseSheet(std::string file_sht, vector<v3f> &sheet_vertices, vector<vector<int>> &sheet_faces, vector<vector<double>> &sheet_offsets, std::map<int, vector<int>> &face2Edge);
std::vector<double> parse_doubles(const std::string &line);
std::vector<int> parseLine2int(std::string data, std::string delim);
std::vector<double> parseLine2double(std::string data, std::string delim);
void printValidity(Validity v);
void performTriangleOffset(v2d vpos0, v2d &vpos1, v2d &vpos2, double offset);
void performTriangleOffset(v2f vpos0, v2f &vpos1, v2f &vpos2, float offset);

double euclideanDistance(v3f v0, v3f v1);
double euclideanDistance(v2d v0, v2d v1);

void generateSphere(double radius, vector<double> &s_points, vector<int> &s_faces);
void findCommonVertex(int &shared_v0, int &shared_v1, vector<int> f0, vector<int> f1);
float getDistancePointToLine(v3f p, v3f v, v3f ed); // p: point v: point on edge, ed: edge direction
float getNorm(v3f v);
v3f projectPointOnPlane(v3f p, v3f normal, v3f vp); // given point, normal of plane and vertex on plane
m4x4f getRotationMatrix(float angle, v3f axis);
m4x4f valMul(m4x4f mat, float val);
v3f applyRotationMatrix(m4x4f rotationMatrix, v3f v);

void split();
void newiteration(int iter, int call_iter, double *x, double *f, double *g, double *gnorm);
void evalfunc(int N, double *x, double *prev_x, double *f, double *g);
void renderCVT();
bool checkOutOfBoundRectangles();
void refreshSitesPos(double *x);
void quickRender();
void computeEdgeLengthMetric();
v2d get_tri_pt(int i);
bool rects_inside_walls();
double polygon_area(const std::vector<v2d>& pts);
double norm(const vector<double> &vec);
double triangle_area(v2d p0, v2d p1, v2d p2);

// CAUTION: ortho normalizes
inline v2d ortho(v2d d) { d = normalize(d); return v2d(-d[1], d[0]); }

inline b2Vec2 b2(v2d p) { // converter
	return b2Vec2(p[0], p[1]);
};

inline v2d rb2(b2Vec2 p) { // reverse converter
	return v2d(p.x, p.y);
}

inline v2f rb2f(b2Vec2 p) { // reverse converter
	return v2f(p.x, p.y);
}

double average(std::vector<double> const &v); // https://stackoverflow.com/questions/28574346/find-average-of-input-to-vector-c