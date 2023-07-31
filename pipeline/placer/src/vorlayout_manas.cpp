/* -------------------------------------------------------- */
#include "helper.hpp"

#include <Sheet.hpp>
#include <FaceInfo.hpp>
#include <Settings.hpp>
#include <parsing_helpers.hpp>

#include <chrono>

using namespace std::chrono;

using namespace std;
using namespace LibSL;

// new defines
#define LED_INITIAL_GUESS 3

FaceInfo g_fi;

/* -------------------------------------------------------- */

LIBSL_WIN32_FIX;

// controls subdivision in rects (seeds in each side of the rect)
#define SQS 4
#define S (SQS*SQS)

// controls resolution
#define R 512

#define HLBFGS_PRINT 0

/* -------------------------------------------------------- */

// display
bool g_ui = true;
int g_W           = 1024;    // Window width
int g_H           = 1024;    // Window height

// target info (LED module)
// values have meaning in [-1, 1]^2
v2d g_min_corner = v2d(-1,1), g_max_corner = v2d(1,1); // global markers for bounding box info
double g_scale = 1.0; 
double g_targetLength = 0.2;	// target length for LED modules

// only used if the triangle is read from a file
double g_targetLengthMM = 10.5;
double g_targetWidthMM = 5.7;	// width < length

// connector
double g_connectorLengthMM = 20.0;
double g_connectorWidthMM = 7.5;
double g_connectorLength = 0.3;
double g_connectorAR = 0.375;
double g_connectorSize = 0.5 * g_connectorLength;

double g_targetAR = 0.5;		// aspect ratio of LED modules in ]0, 1]

double g_RectSize = 0.5 * g_targetLength; // initial rectangle length at transformation

int g_numLeds = 5;
int g_numIters = 1;
// number of pnr tries for the same num of LEDs
int g_numTries = 1;
double g_desired_lengthMM = 6.4; // used in variation example // full hinges
double g_desired_length = 6.4 / 100.0; // changed into triangle scale during init global

double g_vorlayout_offset_x = 0.2;
double g_vorlayout_offset_y = 0.2;

bool displayRigid = true;
bool save_file = false;

int g_numOOB = 0;
int g_numOvlp = 0;
int g_numSmol = 0;
int g_numSucc = 0;
int g_numEdge = 0;

vector<t_site_nfo> g_Sites;
vector<v2d>        g_SitesPos;	// redundant w/ g_Vars
vector<v2d>        g_SitesPosInspect;	
// vector<double>     g_Vars;
vector<double>     g_NewVars; // new variable that stores x0,y0 and theta lives in 0 and 1 for the x and y value
vector<t_rect>     g_Rects;

vector<t_rigid>    g_Rigid;
t_rigid            wall[3];  // Triangle walls
vector<int>        g_debug_Rects;

Array<v2d>         g_Centers;	// redundant w/ g_SitesPos + g_Vars
std::vector<double> g_modToTriAreaRatios;
std::vector<v2d> g_vertices, g_vertices_offset, g_vertices_current; 
// stores the vertices and offset vertices of the current active triangle. 

vector<double> g_offset;
b2Vec2 g_gravity(0.0f, 0.0f);
b2World g_world(g_gravity);

vector<pair<v2d, v2d>> g_Forces, g_NetForces;
vector<pair<uint, uint>> g_EdgesInspect; // voronoi edges

double w_cvt = 0.0;
double w_tri = 1.0;

vector<double> step1times, step2times, step3times, step4times;
int totalIterations = 0; // total iterations where all these steps are used. 
double totalTimeTaken = 0; 
bool compute_average_timing = true;

SimState STATE = SimState::Point;
// TODO MAKE SURE THAT THEY ARE ALL SET TO FALSE!!!!!!
bool generate_variation_example = false;
bool do_ablation_study = false;
bool take_step_images = false;
bool generate_example_offset_showcase_example = false;
bool do_led_comparision = false;
bool user_specified_avg_length = false;
/* -------------------------------------------------------- */

const char *vp_voronoi =
"\
void main() \
{ \
	vec4 wpos      = gl_Vertex;            \
	gl_Position    = gl_ModelViewProjectionMatrix  * wpos; \
	gl_TexCoord[0] = gl_ModelViewProjectionMatrix  * wpos; \
	gl_TexCoord[1] = gl_MultiTexCoord0; \
}";

// u_l_enabled: notch capsules
const char *fp_voronoi =
"\
vec2 ortho(vec2 d) { d = normalize(d); return vec2(-d.y,d.x); } \
\
uniform vec2      u_CentroidE;      \
uniform vec2      u_CentroidId;     \
uniform vec2      u_CentroidPos;    \
uniform vec3      u_CentroidColor;  \
uniform float     u_CentroidWeight; \
uniform sampler2D u_Map;            \
uniform sampler2D u_Tex;            \
uniform vec2	  u_pts[8];			\
uniform int		  u_numPts;			\
\
void main() \
{ \
	float w         = u_CentroidWeight;  \
	float d         = length( gl_TexCoord[1].xy - 0.5 ) * w; \
	gl_FragData[0]  = vec4( u_CentroidId , d.x , 0.0 ); \
	gl_FragData[1]  = vec4( u_CentroidColor, 0.0 ); \
	gl_FragDepth    = d.x; \
	vec2 p          = gl_TexCoord[0].xy; \
\
	float tri_dist = 1.0; \
	for (int i = 0; i < u_numPts; ++i) { \
		int n = mod(i + 1, u_numPts); \
		float din = dot(p - u_pts[i], ortho(u_pts[n] - u_pts[i])); \
		if (din < 0.0) { \
			float uin = dot(p - u_pts[i], normalize(u_pts[n] - u_pts[i])) / length(u_pts[n] - u_pts[i]); \
			if (uin > 0.0 && uin < 1.0) { \
				tri_dist = -din; \
			} else if (uin <= 0.0) { \
				tri_dist = length(p - u_pts[i]); \
			} else { \
				tri_dist = length(p - u_pts[n]); \
			} \
			gl_FragData[0] = vec4(1.0); \
			gl_FragData[1] = vec4( max(tri_dist,0.0) ); \
			gl_FragDepth   = 0.0; \
		} \
	} \
}";

GLShader     g_CVTShader;

/* -------------------------------------------------------- */

GLParameter  g_glCentroidId;
GLParameter  g_glCentroidPos;
GLParameter  g_glCentroidE;
GLParameter  g_glCentroidClr;
GLParameter  g_glMap;

RenderTarget2DRGBA_Ptr g_RTVoronoi;


/* -------------------------------------------------------- */
// header files!! 
void clearGlobals();
void optCVT(int optState);
void computeEdgeLengthMetric();
void quickRender();
void displayCVT();
void renderCVT();
pair<double,v2d> externalDistanceToTriangle(v2d q, v2d p0, v2d p1, v2d p2);
bool is_rect_in_domain(size_t rid);
void takeImage(string filename);

float average(std::vector<float> const& v){
    if(v.empty()){
        return 0;
    }
    auto const count = static_cast<float>(v.size());
    return std::accumulate(v.begin(), v.end(), 0.0) / count;
}


/* -------------------------------------------------------- */
void b2InitScene() { // need to spawn walls where the original sheet triangles are. 
	ForIndex(i, 3) {
		if (wall[i].body != nullptr) {
			g_world.DestroyBody(wall[i].body);
		}
	}

	v2d pos[3] = {g_vertices[0] * 0.5 + v2d(0.5), g_vertices[1] * 0.5 + v2d(0.5), g_vertices[2] * 0.5 + v2d(0.5)};
	// v2d pos[3] = {get_tri_pt_orig(0) * 0.5 + v2d(0.5), get_tri_pt_orig(1) * 0.5 + v2d(0.5), get_tri_pt_orig(2) * 0.5 + v2d(0.5)};
	// v2d pos[3] = { get_tri_pt(0)*0.5 + v2d(0.5), get_tri_pt(1)*0.5 + v2d(0.5), get_tri_pt(2)*0.5 + v2d(0.5) };
	// printf("Thickness : %f\n", thickness);
	double thickness = 0.005; // wall thickness to mimick the edge offset.. // TODO do it for hinge edges and not cut edges
	// printf("0: %f %f | 1: %f %f | 2: %f %f ", pos[0][0], pos[0][1], pos[1][0], pos[1][1], pos[2][0], pos[2][1]);

	b2BodyDef wallBoxBody;
	ForIndex(i, 3) {
		v2d a = pos[i];
		v2d b = pos[(i + 1) % 3];
		wallBoxBody.type = b2_staticBody;
		v2d dir = b - a;
		v2d com = v2d((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0);
		wallBoxBody.position.Set(com[0] * BOX2D_SCALE, com[1] * BOX2D_SCALE);
		wall[i].sx = BOX2D_SCALE * length(dir);
		wall[i].sy = BOX2D_SCALE * thickness;
		double angle = acos(dot(normalize(dir), v2d(1, 0)));
		double a_sgn = sign(dot(cross(v3d(1, 0, 0), v3d(dir)), v3d(0, 0, 1)));
		wallBoxBody.angle = angle * a_sgn;
		wall[i].body = g_world.CreateBody(&wallBoxBody);

		b2PolygonShape bx;
		bx.SetAsBox(wall[i].sx * 0.5, wall[i].sy * 0.5);
		b2FixtureDef fx;
		fx.shape = &bx;
		fx.density = 1.0f;
		fx.restitution = 1.0f;
		wall[i].body->CreateFixture(&fx);
	}
}

t_rect_geometry get_rect(size_t rid, double offset_x = 0.0, double offset_y = 0.0) {
	// g_NewVars in [0, 1], [0, 1] and [0, pi]

	bool is_connector = g_Rigid[int(rid/3)].is_connector;
	double len = is_connector ? g_connectorLength : g_targetLength;
	double ar = is_connector ? g_connectorAR : g_targetAR;

	double offLength = len + offset_x * g_scale;
	double offWidth = len * ar + offset_y * g_scale;

	double theta = g_NewVars[rid + 2];
	v2d dr = v2d(cos(theta), sin(theta));
	v2d nr = v2d(-dr[1], dr[0]);
	v2d rctr = 2.0 * v2d(g_NewVars[rid], g_NewVars[rid + 1]) - v2d(1.0);
	v2d p0 = rctr - 0.5 * dr * offLength - 0.5 * nr * offWidth;
	v2d p1 = p0 + v2d(cos(theta), sin(theta)) * offLength;
	v2d p2 = p0 + dr * offLength + nr * offWidth;
	v2d p3 = p0 + nr * offWidth;
	return std::array<v2d, 4>({p0, p1, p2, p3});
}

// negative if in the left half-plane of the segment, positive if in the right
pair<double, v2d> point_to_segment_distance_sgn(v2d pt, const Segment& s) {
    auto [p0, p1] = s;
    double t = dot(pt - p0, p1 - p0) / sqDistance(p0, p1);
    t = std::clamp(t, 0.0, 1.0);
    v2d h = p0 + t * (p1 - p0);

    v2d edge = p1 - p0;
    v2d ortho = v2d(-edge[1], edge[0]);
    double sign = (dot(ortho, pt - h) >= 0.0) ? -1.0 : 1.0;

    return std::make_pair(sign * distance(pt, h), normalize(pt - h));
}

// works for convex polygons
pair<double, v2d> externalDistanceToFace(v2d pt, const std::vector<v2d>& verts) {
	double min_abs_dist = std::numeric_limits<double>::max();
	v2d min_grad = v2d(0.0);
	bool outside = false;

	for (size_t i = 0; i < verts.size(); ++i) {
		size_t n = (i + 1) % verts.size();
		auto [d, g] = point_to_segment_distance_sgn(pt, make_pair(verts[i], verts[n]));

		if (d >= 0.0) { outside = true; }

		if (std::abs(d) < min_abs_dist) {
			min_abs_dist = std::abs(d);
			min_grad = g;
		}
	}

	if (!outside) {
		min_abs_dist = 0.0;
		min_grad = v2d(0.0);
	}

	return make_pair(min_abs_dist, -min_grad);
}

pair<double, v2d> externalDistance(v2d pt) {
	return externalDistanceToFace(pt, g_vertices);
}

// --------------------------------------------------------------
// Marco

int guess_num_leds(const std::vector<v2d>& pts) {
	constexpr double k = 0.55;
	double area_poly = polygon_area(pts);
	double area_led = g_targetAR * g_targetLength * g_targetLength;
	std::cout << "Area poly / led\t" << area_poly << '\t' << area_led << '\n';
	std::cout << "LED GUESS\t" << std::floor(k * area_poly / area_led) << '\n';
	return std::max(static_cast<int>(std::floor(k * area_poly / area_led)), 1);
}

double guess_target_length(const std::vector<v2d>& pts) {
	constexpr double k = 5.0;
	double area_poly = polygon_area(pts);
	return std::sqrt(area_poly / (k * g_numLeds * g_targetAR));
}

bool is_v2d_in_domain(v2d pt, const std::vector<v2d>& verts) {
	double min_abs_dist = std::numeric_limits<double>::max();
	v2d min_grad = v2d(0.0);
	bool outside = false;

	for (size_t i = 0; i < verts.size(); ++i) {
		size_t n = (i + 1) % verts.size();
		auto [d, g] = point_to_segment_distance_sgn(pt, make_pair(verts[i], verts[n]));

		if (d >= 0.0) { outside = true; }

		if (std::abs(d) < min_abs_dist) {
			min_abs_dist = std::abs(d);
			min_grad = g;
		}
	}

	return !outside;
}

pair<double, double> stats_rect_sizes() {
	auto sgn_dist_l1 = [](int rect_id) {
		const auto& [p0, p1, p2, p3] = get_rect(rect_id, g_vorlayout_offset_x, g_vorlayout_offset_y);
		double is_connector = false; 
		if(STATE == SimState::PointRigidBody) {
			is_connector = g_Rigid[int(rect_id/3)].is_connector;
		}
		else {
			printf("Haven't implemented yet. DEBUG!!\n");
			exit(1);
		}
		
		double len = is_connector ? g_connectorLength : g_targetLength;
		double ar  = is_connector ? g_connectorAR : g_targetAR;
		double targetWidth = len * ar;

		// if (len < targetWidth) {
		// 	// std::cout << "UwU\n";
		// 	double tmp = len;
		// 	len = targetWidth;
		// 	targetWidth = len;
		// }

		double length = std::max(distance(p0, p1), distance(p0, p3));
		double width = std::min(distance(p0, p1), distance(p0, p3));

		// printf("length : %f width : %f\n", length, width);

		return std::make_pair(length - len, width - targetWidth);
	};

	std::pair<double, double> min_pair;
	double min_val = std::numeric_limits<double>::max();
	for (auto r : g_Rigid) {
		uint rid = r.first_var_id;
		auto dl1 = sgn_dist_l1(rid);
		double val = dl1.first + dl1.second;
		if (val < min_val) {
			min_val = val;
			min_pair = dl1; 
		}
	}
	return min_pair;
}

bool are_rects_separate() {
	ClipperLib::Clipper clip;
	size_t num_rects = 0;
	for (size_t rid = 0; rid < g_NewVars.size(); rid += 3) {
		++num_rects;
		auto rect = get_rect(rid, g_vorlayout_offset_x, g_vorlayout_offset_y);
		ClipperLib::Path path;
		for (v2d pt : rect) {
			ClipperLib::IntPoint pti(1e6 * pt[0], 1e6 * pt[1]);
			path.push_back(pti);
		}
		clip.AddPath(path, ClipperLib::PolyType::ptClip, true);
	}
	ClipperLib::Paths sol;
	clip.Execute(ClipperLib::ClipType::ctUnion, sol, ClipperLib::PolyFillType::pftNonZero, ClipperLib::PolyFillType::pftNonZero);
	return num_rects == sol.size();
}

tuple<Validity, double, double> is_solution_valid() {
	auto[mdL, mdW] = stats_rect_sizes();
	if (!are_rects_separate()) {
		// std::cout << "Invalid solution: some rects collide\n";
		return make_tuple(Validity::overlap, mdL, mdW);
	}
	for (size_t rid = 0; rid < g_NewVars.size(); rid += 3) {
		// std::cout << "RID " << rid << '\n';
		if (!is_rect_in_domain(rid)) {
			return make_tuple(Validity::outOfBounds, mdL, mdW);
		}
	}
	// std::cout << "Valid solution\n";
	return make_tuple(Validity::success, mdL, mdW);
}


/* -------------------------------------------------------- */
// computes g_SitesPos from g_Vars

void refreshSitesPos(double *x) {
	// g_NewVars lives in [0:1]
	if (STATE == SimState::Point or STATE == SimState::RigidBody or STATE == SimState::PointRigidBody) {
		for (const auto &s : g_Sites) {
			int n = s.first_var_id;
			if (holds_alternative<t_indepedent_site>(s.nfo)) {
				if (STATE == SimState::Point) {
					g_SitesPos[s.site_id] = v2d(x[n + 0], x[n + 1]);
				}
				else if (STATE == SimState::PointRigidBody) {
					g_SitesPos[s.site_id] = v2d(x[n + 0], x[n + 1]);
				}
			}
			else {
				sl_assert(false);
			}
		}
	}
}

/* -------------------------------------------------------- */

void initSites() {
	clearGlobals();
	int N_POINTS = g_numLeds; // number of LEDs
	ForIndex(n, N_POINTS) { 
		// CVT initialization w/ points instead of rects
		v2d p = v2d(0.5) + 0.5 * v2d(srnd(), srnd());
		t_site_nfo c;
		c.site_id = (int)g_Sites.size();
		c.first_var_id = (int)g_NewVars.size();
		g_NewVars.push_back(p[0]);
		g_NewVars.push_back(p[1]);
		t_indepedent_site ci;
		c.nfo = ci;
		g_Sites.push_back(c);
	}
	// refresh sites pos
	g_SitesPos.resize(g_Sites.size());
	refreshSitesPos(&g_NewVars[0]);
	STATE = SimState::Point;
}

/* -------------------------------------------------------- */
pair<double,v2d> externalDistanceToTriangle(v2d q, v2d p0, v2d p1, v2d p2) {
	double dist = 0.0;
	v2d    grad(0.0); // only stores the x and y component
	double d01 = dot(q - p0, ortho(p1 - p0));
	double d12 = dot(q - p1, ortho(p2 - p1));
	double d20 = dot(q - p2, ortho(p0 - p2));
	if (d01 < 0.0 || d12 < 0.0 || d20 < 0.0) { // the point is outside the triangle and thus demands to be accounted for
		float u01 = dot(q - p0, normalize(p1 - p0)) / length(p1 - p0);
		float u12 = dot(q - p1, normalize(p2 - p1)) / length(p2 - p1);
		float u20 = dot(q - p2, normalize(p0 - p2)) / length(p0 - p2);
		if (d01 < 0.0) {
			if (u01 > 0.0 && u01 < 1.0) {
				dist = -d01;
				grad = ortho(p1 - p0);
			} else if (u01 <= 0.0) {
				dist = length(q - p0);
				grad = normalize(p0 - q);
			} else {
				dist = length(q - p1);
				grad = normalize(p1 - q);
			}
		} else if (d12 < 0.0) {
			if (u12 > 0.0 && u12 < 1.0) {
				dist = -d12;
				grad = ortho(p2 - p1);
			} else if (u12 <= 0.0) {
				dist = length(q - p1);
				grad = normalize(p1 - q);
			} else {
				dist = length(q - p2);
				grad = normalize(p2 - q);
			}
		} else /* (d20 < 0.0) */ {
			if (u20 > 0.0 && u20 < 1.0) {
				dist = -d20;
				grad = ortho(p0 - p2);
			} else if (u20 <= 0.0) {
				dist = length(q - p2);
				grad = normalize(p2 - q);
			} else {
				dist = length(q - p0);
				grad = normalize(p0 - q);
			}
		}
		// printf("Gradient val: %f %f\n", grad[0], grad[1]);
	}
	return make_pair(dist,grad);
}

double energyFunc(vector<double> var) {
	// TODO in this state it would only work for points.
	// refreshSitesPos(&var[0]);
	// // render CVT
	// renderCVT();

	// read back
	Array2D<v4b> pixs;
	g_RTVoronoi->readBack(pixs, 0);

	// compute new centroids
	Array<int> cnt(g_Sites.size());
	Array<v2d> gctr(g_Sites.size());
	cnt.fill(0);
	gctr.fill(v2d(0));

	double cvt_energy = 0;
	double energy = 0.0;
	if (w_cvt > 0) {
		ForArray2D(pixs, i, j) {
			int id = pixs.at(i, j)[0] + (pixs.at(i, j)[1] << 8); // cell id
			if (id < g_Sites.size()) {																			  // false if outside the domain
				v2d p = (v2d(i, j) + v2d(0.5)) / v2d(g_RTVoronoi->w(), g_RTVoronoi->h()); // pixel center coordinates (screen space) [0 and 1]
				gctr[id] += p;
				cnt[id]++;											// cell area
				cvt_energy += w_cvt * sqLength(g_SitesPos[id] - p); // TODO there should be a 0.5 factor here...
				energy += w_cvt * sqLength(g_SitesPos[id] - p);
			}
		}
	}
	// normalize weights
	double domain_area = 0.0;
	ForIndex(i, cnt.size()) {
		domain_area += cnt[i];
	}

	// triangle distance energy
	if (w_tri > 0) {
		double distance_energy = 0;
		for (auto s : g_Sites) {
			if (holds_alternative<t_indepedent_site>(s.nfo)) {
				auto dg = externalDistance(g_SitesPos[s.site_id] * 2.0 - v2d(1.0)); // in -1 to 1
				distance_energy += w_tri * dg.first;
				energy += w_tri * dg.first;
			}
			else {
				sl_assert(false);
			}
		}
	}
	return energy;
}

void myGradientDescent(vector<double> &g_NewVars, int numIters = 1)  {

	vector<double> g_NewVarsTemp = g_NewVars;
	vector<double> gradient(g_NewVars.size());
	double energy_prev = 0, energy_curr = 0;

	Array<double> g;
	g.allocate(g_NewVarsTemp.size());
	g.fill(0);

	energy_prev = energyFunc(g_NewVars);

	for (int itern = 0; itern < numIters; itern++) {
		double alpha = 1;
		energy_curr = energy_prev;
		while (alpha > 1e-2)
		{ // stopping criterion for line search
			alpha = alpha / 10;
			g_NewVarsTemp = g_NewVars;
			evalfunc(g_NewVarsTemp.size(), &g_NewVarsTemp[0], &g_NewVarsTemp[0], &energy_curr, g.raw());
			// gradient_fd(g_NewVarsTemp, g);
			for (size_t i = 0; i < g.size(); i++) {
				gradient[i] = g[i];
				g_NewVarsTemp[i] -= gradient[i] * alpha;
			}
			// printf("energy before gradient update: %f\n", energy_curr);
			energy_curr = energyFunc(g_NewVars);
			// printf("energy before gradient update: %f\n", energy_curr);
			if (energy_curr < energy_prev or norm(gradient) < 1e-3)
				break;
		}
		energy_prev = energy_curr;
		// printf("Iteration: %d alpha: %f energy: %f gradient norm: %0.10f\n", itern, alpha, energy_prev, norm(gradient));

		g_NewVars = g_NewVarsTemp;
		if (norm(gradient) < 1e-3)
			break;
	}
}

void transformRects() {
	g_Rigid.clear();
	// g_Sites do not change just add a rigid body on top of rectangles or on top of points with same sites
	if (STATE == SimState::Point) {
		auto prev_pos = g_SitesPos;
		g_NewVars.clear();
		int rid = 0;
		int pid = 0;
		for (auto p : prev_pos) {
			// create all sites for a rectangle (square)
			t_rigid r;
			v2d rctr = p; // site center
			r.first_var_id = (int)g_NewVars.size();
			// double agl  = 0;
			double agl = M_PI * 2.0 * rnd();
			v2d dr = v2d(cos(agl), sin(agl));
			v2d nr = v2d(-dr[1], dr[0]);
			v2d p0 = rctr;
			// v2d p0 = rctr - 0.5 * dr * g_RectSize - 0.5 * nr * g_RectSize*g_targetAR;
			g_NewVars.push_back(p0[0]);
			g_NewVars.push_back(p0[1]);
			g_NewVars.push_back(agl);

			b2BodyDef bodyDef;
			// bodyDef.type = b2_staticBody;
			bodyDef.type = b2_dynamicBody;
			double box_center_x = BOX2D_SCALE * (rctr[0]);
			double box_center_y = BOX2D_SCALE * (rctr[1]);
			bodyDef.position.Set(box_center_x, box_center_y); // scaling for the box2d sensitivity
			// have to define the body center where the com is kept

			bodyDef.allowSleep = false;
			bodyDef.angle = agl;
			r.body = g_world.CreateBody(&bodyDef);
			b2PolygonShape bx;
			if(pid == 0 and g_fi.has_connector) {// 1st one is connector
				r.is_connector = true;
				r.sx = BOX2D_SCALE * (g_connectorSize + g_vorlayout_offset_x * g_scale * 0.5);
				r.sy = BOX2D_SCALE * (g_connectorAR * g_connectorSize + g_vorlayout_offset_y * g_scale * 0.5);
			}
			else {
				r.sx = BOX2D_SCALE * (g_RectSize + g_vorlayout_offset_x * g_scale * 0.5);
				r.sy = BOX2D_SCALE * (g_targetAR * g_RectSize + g_vorlayout_offset_y * g_scale * 0.5);
			}
			
			bx.SetAsBox(r.sx * 0.5, r.sy * 0.5);
			b2FixtureDef fx;
			fx.shape = &bx;
			fx.density = 1.f;
			fx.friction = 0.0f;
			r.body->CreateFixture(&fx);
			r.body->SetAngularDamping(1000.0f);
			r.body->SetLinearDamping(1000.0f);
			r.body->SetLinearVelocity(BOX2D_SCALE * b2Vec2(0.0, 0));
			// DEBUG
			b2Vec2 pos_d = r.body->GetPosition();
			float agl_d = r.body->GetAngle();
			g_Rigid.push_back(r);
			pid++;
		}
		STATE = SimState::PointRigidBody;
	}
	else {
		printf("Incorrect transition away from RIGID Bodies\n");
	}
}

void quickRender() {
	if (g_ui) {
		glClearColor(0, 0, 0.5, 0);
		glViewport(0, 0, g_W, g_H);
		displayCVT();
		SimpleUI::glSwapBuffers();
	}
}


bool rects_inside_walls() {
	for (auto r : g_Rigid) {
		auto rect = get_rect(r.first_var_id, g_vorlayout_offset_x, g_vorlayout_offset_y);
		for (auto p : rect) {
			v2d wall_p0 = g_vertices[0];
			v2d wall_p1 = g_vertices[1];
			v2d wall_p2 = g_vertices[2];
			double dist = externalDistanceToTriangle(p, wall_p0, wall_p1, wall_p2).first;
			if (dist > 1e-3)
				return false;
		}
	}
	return true;
}

void applyCVTForce() {
	double scl_force = 10000000;
	if(STATE == SimState::PointRigidBody and w_cvt > 0) {
		// force them to be more nicely distributed.
		g_Forces.clear();
		g_NetForces.clear();
		Array<int> cnt(g_Sites.size());
		Array<v2d> gctr(g_Sites.size());
		g_Centers.erase();
		g_Centers.allocate(g_Sites.size());
		cnt.fill(0);
		gctr.fill(v2d(0));
		// read back
		Array2D<v4b> pixs;
		g_RTVoronoi->readBack(pixs, 0);
		ForArray2D(pixs,i,j) {
			int id = pixs.at(i,j)[0] + (pixs.at(i,j)[1] << 8);	// cell id
			if (id < g_Sites.size()) {	// false if outside the domain
				v2d p     = (v2d(i,j) + v2d(0.5)) / v2d(g_RTVoronoi->w(),g_RTVoronoi->h());// pixel center coordinates (screen space) [0 and 1]
				gctr[id] += p;
				cnt [id] ++;	// cell area
			}
		}
		// double scl_force = BOX2D_SCALE/0.1;
		vector<v2d> netForce(g_Rigid.size());
		for(auto &f: netForce) {f = v2d(0); } // init to zero // don't forget the reference :/
		ForIndex(i, gctr.size()) {
			if (cnt[i] > 0) {
				g_Centers[i] = gctr[i] / (double)cnt[i];
				if (holds_alternative<t_indepedent_site>(g_Sites[i].nfo)) {
					auto rigid = g_Rigid[g_Sites[i].site_id];
					v2d f = (g_Centers[i] - g_SitesPos[i]);
					rigid.body->ApplyForce(b2(scl_force*f), b2(g_SitesPos[i] * BOX2D_SCALE), true);
					// g_Forces.push_back(make_pair(scl_force*f, g_SitesPos[i])); // acts as a torque
					netForce[int(rigid.first_var_id/3)] += scl_force * f;
					g_Forces.push_back(make_pair(0.01 * normalize(f), g_SitesPos[i]));
				}
			}
		}
	}	
}

void updatePosition() {
	// update g_NewVars based on the information provided by the rigid body rectangles
	vector<double> oldVar = g_NewVars;
	g_Sites.clear();
	// g_NewVars.clear();
	int rid = 0;
	for (auto &r : g_Rigid) {
		b2Vec2 position = r.body->GetPosition();
		float angle = r.body->GetAngle();
		v2d dr = v2d(cos(angle), sin(angle));
		v2d nr = v2d(-dr[1], dr[0]);
		// create all sites for a rectangle (square)
		v2d rctr(position.x / BOX2D_SCALE, position.y / BOX2D_SCALE);

		if (STATE == SimState::PointRigidBody) { // for point rigid body
			v2d p0 = rctr;
			// no need to delete and respawn it. I can just redo the calculation based on update of position
			g_NewVars[r.first_var_id + 0] = p0[0];
			g_NewVars[r.first_var_id + 1] = p0[1]; // x0 y0
			g_NewVars[r.first_var_id + 2] = angle; // angle

			t_site_nfo c;
			c.site_id = (int)g_Sites.size();
			c.first_var_id = r.first_var_id;
			t_indepedent_site ci;
			c.nfo = ci;
			g_Sites.push_back(c);
		}
	}
	refreshSitesPos(&g_NewVars[0]); // g_SitesPos will get updated here..
}

double getVelocity() {
	double netVelocity = 0;
	for (auto &r : g_Rigid)
	{
		b2Vec2 vel = r.body->GetLinearVelocity();
		float avl = r.body->GetAngularVelocity();
		netVelocity += length(v2d(vel.x, vel.y)) + fabs(avl);
	}
	return netVelocity;
}

void printMetric() {
	// arial coverage.
	refreshSitesPos(&g_NewVars[0]);
	// render CVT
	renderCVT();

	// read back
	Array2D<v4b> pixs;
	g_RTVoronoi->readBack(pixs, 0);

	vector<double> area(g_Rigid.size(), 0);
	ForArray2D(pixs, i, j) {
		int id = pixs.at(i, j)[0] + (pixs.at(i, j)[1] << 8); // cell id
		if (id < g_Rigid.size()) {				// false if outside the domain
			area[id]++; // cell area
		}
	}
	double total_area = 0.0;
	ForArray(area, i) {
		total_area += area[i];
	}
	double min_area = 1000, max_area = -1000;
	ForArray(area, i) {
		// printf("index : %d area ratio : %f\n", i, area[i]/total_area);
		area[i] = area[i] / total_area;
		min_area = std::min(min_area, area[i]);
		max_area = std::max(max_area, area[i]);
	}
	printf("Number of LEDs: %d Ratio of max to min area: %f\n", g_numLeds, max_area / min_area);
	double area_tri = triangle_area(g_vertices[0], g_vertices[1], g_vertices[2]);

	bool is_connector = g_fi.has_connector;
	double area_ratio = (g_numLeds * g_targetLength * g_targetLength * g_targetAR) / (area_tri);
	printf("Number of LEDs: %d Area coverage by LEDs: %f\n", g_numLeds, area_ratio);
	computeEdgeLengthMetric();
}

void generateDelaunayTriangulation() {
	if (g_EdgesInspect.size() != 0) {
		g_EdgesInspect.clear(); // generate a new one
	}
	// use g_SitesPosInspect as the input set of points
	CDT::Triangulation<double> cdt;
	vector<CDT::V2d<double>> points;
	for (auto p : g_SitesPosInspect)
	{
		points.push_back(CDT::V2d<double>::make(p[0], p[1]));
	}
	cdt.insertVertices(points);
	cdt.eraseSuperTriangle();
	std::vector<CDT::Triangle> triangles = cdt.triangles;
	for (auto tri : triangles) {
		std::array<unsigned int, 3> vindices = tri.vertices;
		std::sort(vindices.begin(), vindices.end());

		uint v0 = vindices[0], v1 = vindices[1], v2 = vindices[2];
		v2d pos0 = g_SitesPosInspect[v0], pos1 = g_SitesPosInspect[v1], pos2 = g_SitesPosInspect[v2];
		double theta0, theta1, theta2;
		theta0 = acos(dot(normalize(pos1 - pos0), normalize(pos2 - pos0)));
		theta1 = acos(dot(normalize(pos0 - pos1), normalize(pos2 - pos1)));
		theta2 = acos(dot(normalize(pos0 - pos2), normalize(pos1 - pos2)));
		// printf("%d %d %d %f %f %f\n", v0, v1, v2, theta0, theta1, theta2);
		// printf("v0: %f %f v1: %f %f v2: %f %f\n", pos0[0], pos0[1], pos1[0], pos1[1], pos2[0], pos2[1]);
		// printf("l0: %f %f l1 : %f %f\n", normalize(pos1 - pos0)[0], normalize(pos1 - pos0)[1], normalize(pos2 - pos0)[0], normalize(pos2 - pos0)[1]);

		pair<uint, uint> e0 = std::make_pair(v0, v1), e1 = std::make_pair(v1, v2), e2 = std::make_pair(v0, v2);
		// don't add edges that are longer edges created as part of obtuse angle
		if ((std::find(g_EdgesInspect.begin(), g_EdgesInspect.end(), e0) == g_EdgesInspect.end()) and (theta2 < M_PI_2 * 1.33) and (theta2 > M_PI_2 * 0.1)) {
			g_EdgesInspect.push_back(e0);
			// printf("theta2 : %f ",theta2);
		}
		if ((std::find(g_EdgesInspect.begin(), g_EdgesInspect.end(), e1) == g_EdgesInspect.end()) and (theta0 < M_PI_2 * 1.33) and (theta0 > M_PI_2 * 0.1)) {
			g_EdgesInspect.push_back(e1);
			// printf("theta0 : %f ", theta0);
		}
		if ((std::find(g_EdgesInspect.begin(), g_EdgesInspect.end(), e2) == g_EdgesInspect.end()) and (theta1 < M_PI_2 * 1.33) and (theta1 > M_PI_2 * 0.1)) {
			g_EdgesInspect.push_back(e2);
			// printf("theta1 : %f ", theta1);
		}

		// printf("\n");
		// quickRender();
		// printf("DEBUG\n");
	}
	// printf("Done Delaunay Triangulation\n");
}

void computeEdgeLengthMetric() {
	g_SitesPosInspect.clear();
	if (STATE == SimState::RigidBody) {
		// generate the LED point from the rectangle
		// rectangle centers
		for (auto r : g_Rigid) {
			b2Vec2 position = r.body->GetPosition();
			v2d positionX = v2d(position.x / BOX2D_SCALE, position.y / BOX2D_SCALE);
			g_SitesPosInspect.push_back(positionX);
		}
	}
	else { // it is in Point State already
		g_SitesPosInspect = g_SitesPos;
	}
	// Spawn voronoi diagram from these points
	// Compute the delaunay triangulate of these points
	generateDelaunayTriangulation();
	// compute the edge lengths of all the triangles in this triangulation
	double min_edge_length = std::numeric_limits<double>::max(), max_edge_length = -std::numeric_limits<int>::max();
	if(g_EdgesInspect.size() > 2) {
		ForIndex(ei, g_EdgesInspect.size()) {
			v2d pos0 = g_SitesPosInspect[get<0>(g_EdgesInspect[ei])];
			v2d pos1 = g_SitesPosInspect[get<1>(g_EdgesInspect[ei])];
			double length = distance(pos0, pos1);
			// printf("Length : %f\n", length);
			min_edge_length = std::min(min_edge_length, length);
			max_edge_length = std::max(max_edge_length, length);
		}
		// report the ratio of shorest to longest edge lengths.
		printf("Ratio of shorest to longest edge lengths : %f\n", min_edge_length / max_edge_length);
	}
}

void optCVT(int optState) {
	// Timer tm("[opt]");
	// Optimize
	double parameter[20];
	int info[20];
	INIT_HLBFGS(parameter, info);
	info[0] = 10;	  // max_fev_in_linesearch
	info[1] = 0;	  // total_num_fev
	info[2] = 0;	  // iter
	info[3] = 0;	  // update strategy. 0: standard lbfgs, 1: m1qn3;
	info[4] = 100000; // max iterations
	// info[5] = 0;
	info[5] = 0;
	info[6] = 10;
	info[7] = 0;  // no hessian
	info[8] = 15; // icfc parameter
	info[9] = 0;  // no linesearch
	info[10] = 0;
	info[11] = 1;
	// run optimizer
	info[12] = 1; // internal usage. 0: only update diag in USER_DEFINED_HLBFGS_UPDATE_H
	info[13] = 0; // 0: standard lbfgs update, 1: Biggs's update, 2: Yuan's update; 3: Zhang and Xu's update

	// double noi = 1e-1;
	double noi = 0 * srnd();
	// double noi = 1e-5 * srnd();
	// uint i=0;
	for (auto &gvar : g_NewVars) {
		// if(i%3==2)
		gvar += noi;
		// i++;
	}

	g_debug_Rects.clear(); // just clear it before starting a new loop

	if (optState == 0) {
		w_cvt = 0.0; // all points are inside do this again with some cvt weight.
		w_tri = 1.0;
		HLBFGS(g_NewVars.size(), 20 /*3-20*/, &g_NewVars[0], evalfunc, 0, HLBFGS_UPDATE_Hessian, newiteration, parameter, info);
	}
	else if (optState == 1) {
		w_cvt = 1.0; // all points are inside do this again with some cvt weight.
		w_tri = 100.0;
		HLBFGS(g_NewVars.size(), 20 /*3-20*/, &g_NewVars[0], evalfunc, 0, HLBFGS_UPDATE_Hessian, newiteration, parameter, info);
	}
	else if (optState == 2) {
		w_cvt = 1.0; // all points are inside do this again with some cvt weight.
		w_tri = 0.0;
		HLBFGS(g_NewVars.size(), 20 /*3-20*/, &g_NewVars[0], evalfunc, 0, HLBFGS_UPDATE_Hessian, newiteration, parameter, info);
	}

	else if (optState == 3) {
		w_cvt = 0.0; // all points are inside do this again with some cvt weight.
		w_tri = 1.0;
		myGradientDescent(g_NewVars);
	}
	else if (optState == 4) {
		w_cvt = 1.0; // all points are inside do this again with some cvt weight.
		w_tri = 100.0;
		myGradientDescent(g_NewVars);
	}
	else if (optState == 5) {
		w_cvt = 1.0; // all points are inside do this again with some cvt weight.
		w_tri = 0.0;
		myGradientDescent(g_NewVars, 1);
	}
}

tuple<Validity, double, double> iterCVT(int try_num = 0) {
	// Timer tm1("[full opt]");
	auto timerStart = std::chrono::system_clock::now();

	// itercvt is called
	if (compute_average_timing) {
		totalIterations ++;
	}

		g_vertices_current = g_vertices;
	initSites();
	optCVT(1);
	quickRender();
	std::chrono::duration<double> step1Time = std::chrono::system_clock::now() - timerStart;
	// int tmp;
	// tmp = getc(stdin);
	if(take_step_images) {
		takeImage("step1.png");
	}


	// TO make this fast already check at this stage that if minimum number is satisfied or not
	if(generate_variation_example or user_specified_avg_length) { // at this stage newVars store the position only
		// vector<double> els; 
		double avg_length = 0; 

		for(uint v0 = 0; v0 < g_SitesPos.size(); v0++) {
			v2d wv0 = g_SitesPos[v0];
			uint bestNeighbour = -1;
			double bestDistance = std::numeric_limits<double>::max();
			for(uint v1 = 0; v1 <  g_SitesPos.size(); v1++) {
				v2d wv1 = g_SitesPos[v1];
				if(v0 != v1) {
					double distance = euclideanDistance(wv0, wv1);
					// g_sites_pos lives in 0 and 1. g_scale takes from -1:1 to world. so should be the right scaling factor to bring it to -1,1 space
					// distance = distance * 2; 
					if(distance < bestDistance) {
						bestNeighbour = v1;
						bestDistance = distance;
					}
				}
			}
			// els.push_back(bestDistance);
			avg_length += bestDistance / g_SitesPos.size();
		}
		if(g_SitesPos.size() == 1) {
			avg_length = 0; 
		}
		// printf("g_scale is : %f\n", g_scale);
		printf("avg edge length : %f desired edge length : %f okay : %d\n", avg_length, g_desired_length, (avg_length>=g_desired_length)? 1: 0);
		printf("avg edge lengthMM : %f desired edge lengthMM : %f\n", avg_length * 2.0 / g_scale, g_desired_lengthMM);
		if(avg_length <= g_desired_length) {
			printf("AVG length is not enough\n");
			return make_tuple(Validity::edgeLengthConstrainViolated, 0.0, 0.0);
		}
	}


	// apply the offsets based on the offset vals
	// optCVT(0);
	transformRects();
	quickRender();

	std::chrono::duration<double> step2Time = std::chrono::system_clock::now() - timerStart;
	step2Time = step2Time - step1Time;
	if(take_step_images) {
		takeImage("step2.png");
	}

	// Here I have to bleed out the triangles..
	g_vertices_current = g_vertices_offset;

	// perform box2d steps
	int numStep = 200;
	int originalNumStep = numStep;
	// int numStep = 1000* try_num;
	float timeStep = 1.0f / 30.0f;
	int32 velocityIterations = 10;
	int32 positionIterations = 20;
	bool justSeparated = true;
	// int timeForce = 50;
	int timeForce = 200 * try_num;
	// int timeForce = 200;
	// int timeForce = 400;
	// int timeForce = 600;
	// int timeForce = 800;
	// int timeForce = 1200;
	// int timeForce = 1400;
	// int timeForce = 200;
	// int timeForce = 1000;
	// int timeZForce = 10;
	int timeZForce = 50;
	// https://gamedev.stackexchange.com/questions/128467/box2d-how-can-i-speed-up-a-physics-simulation-with-many-many-overlapping-box-b
	vector<double> avgVelocityVector; // 10 long array size 0
	std::chrono::duration<double> step3Time;
	ForIndex(i, numStep) {
		// printf("\ri : %04d", i);
		fflush(stdout);
		// if(not rects_inside_walls()) printf("Rects are not inside the walls\n");
		// if(not are_rects_separate()) printf("Rects are not separate\n");
		
		renderCVT();
		if(are_rects_separate() and rects_inside_walls()) { // before doing this step ensure that the areas are separate
			if(justSeparated) { // one time bonus for timesteps since it succesfully resolved the collisions
				numStep = numStep + timeForce + timeZForce;
				step3Time = std::chrono::system_clock::now() - timerStart;
				step3Time = step3Time - step2Time;
				if(take_step_images) {
					takeImage("step3.png");
				}
			}

			if (i < originalNumStep + timeForce) { // for 8/10 of the reamining time apply force to center the object
				applyCVTForce();
			}
			
			justSeparated = false;
		} 
		g_world.Step(timeStep, velocityIterations, positionIterations);
		updatePosition();
		quickRender();
		auto res = is_solution_valid();
		auto [vtype, sum, avg ] = res;
		double totalVelocity = getVelocity();
		if(i < 10) {
			avgVelocityVector.push_back(totalVelocity);
		}
		else {
			avgVelocityVector[i%10] = totalVelocity;
		}
		double avg_velocity = average(avgVelocityVector);
		bool valid = are_rects_separate() and rects_inside_walls();
		// printf("Iter: %04d Net velocity : %0.7f posDiff : %0.7f\n", i, totalVelocity, totalPosDiff);
		// printf("\rIter: %04d avg velocity : %0.7f", i, avg_velocity);
		if(valid and avg_velocity < 0.05 and not justSeparated) {
			if(take_step_images) {
				string step4 = "step4-" + to_string(timeForce) + ".png";
				takeImage(step4.c_str());
				if(do_ablation_study) {
					exit(1); // remove this line after generating the ablation study results
				}
			}
			break;
		}
		// int tmp;
		// tmp = getc(stdin);
	}
	cout << endl;

	// optCVT(0);
	refreshSitesPos(&g_NewVars[0]);
	renderCVT();
	quickRender();
	std::chrono::duration<double> step4Time = std::chrono::system_clock::now() - timerStart;
	step4Time = step4Time - step3Time;


	auto res = is_solution_valid();
	auto[vtype, sum, avg] = res;

	auto prec = std::cout.precision();
	printValidity(vtype);
	// if(vtype == Validity::success) { // compute the metric result. 
	// 	printMetric();
	// }

	if (compute_average_timing and try_num > 0 and vtype == Validity::success) { // try_num > 0 means it is the final step
		cout << "Time spend in different steps:"
			 << "\nStep1Time:" << step1Time.count() << " s"
			 << "\nStep2Time:" << step2Time.count() << " s"
			 << "\nStep3Time:" << step3Time.count() << " s"
			 << "\nStep4Time:" << step4Time.count() << " s" << endl;
		step1times.push_back(step1Time.count());
		step2times.push_back(step2Time.count());
		step3times.push_back(step3Time.count());
		step4times.push_back(step4Time.count());
	}
	quickRender();
	// int tmp;
	// tmp = getc(stdin);
	// std::cout << '\n' << setprecision(prec);
	return res;
}

bool iter_until_success_or_done(std::vector<double>& bak, tuple<Validity, double, double>& bak_res) {
	for (int i = 0; i < g_numIters; ++i) {
		std::cout << "Global Iteration " << i << std::endl;
		auto res = iterCVT();
		if (get<0>(res) == Validity::success) { 
			bak = g_NewVars;
			bak_res = res;
			return true; 
		}
	}
	return false;
}

int find_num_leds(int init_leds, std::vector<double>& bak, tuple<Validity, double, double>& bak_res, bool can_rise) {
	std::cout << "Finding max #LEDs...\n";
	// assert(init_leds > 0);
	g_numLeds = init_leds;
	int max_leds = 0;
	bool prev_fail = false;;

	do {
		std::cout << g_numLeds << "?\n";
		if (iter_until_success_or_done(bak, bak_res)) {
			max_leds = g_numLeds;
			if (prev_fail) { break; }
			if (can_rise) { ++g_numLeds; } else { break; }
		} else {
			if (max_leds > 0) { break; }
			prev_fail = true;
			--g_numLeds;
		}
	} while (g_numLeds > 0);
	assert(max_leds >= 0);

	return max_leds;
}

// if num_leds == -1, guess number of LEDs and find max from there
// else start at num_leds and only go down
int solve_stuff(int num_leds = -1) {
	std::vector<double> bakVars;
	tuple<Validity, double, double> bak_res;
	
	if (num_leds < 0) {
		g_numLeds = find_num_leds(guess_num_leds(g_vertices), bakVars, bak_res, true);
	} else {
		g_numLeds = find_num_leds(num_leds, bakVars, bak_res, false);
	}
	// g_numLeds = find_num_leds(LED_INITIAL_GUESS, bakVars, bak_res);
	
	if (g_numLeds == 0) {
		if (g_fi.has_connector) {
			std::cout << "[ERROR] Cannot fit connector in triangle\n";
			exit(EXIT_FAILURE);
		}
		g_NewVars.clear();
		std::cout << "\nEMPTY TRIANGLE\n";
		return g_numLeds;
	}

	std::vector<double> max_vars;
	tuple<Validity, double, double> final_res;
	bool found = false;
	std::cout << "\nRunning for max LEDs (" << g_numLeds << ")";

	for (int i = 0; i < g_numIters; ++i) {
		std::cout << "Global Iteration " << i << std::endl;
		auto res = iterCVT(4);
		auto [v, mdL, mdW] = res;
		
		if (v == Validity::success) {
			found = true;
			max_vars = g_NewVars;
			final_res = res;
			if(generate_example_offset_showcase_example) {
				exit(1);
			}
			break;
		}
	}
	
	bool is_bak = false; 
	// very convoluted way of handling both increase and decrease in LED guesses but it works!
	if (found) {
		std::cout << "Final solution\n";
	} else {
		std::cout << "Fallback solution\n";
		is_bak = false;
		g_NewVars = bakVars;
		final_res = bak_res;

		g_numLeds--;
		for (int i = 0; i < g_numIters; ++i) {
			std::cout << "FALLBACK Global Iteration " << i << std::endl;
			auto res = iterCVT(4);
			auto [v, mdL, mdW] = res;

			if (v == Validity::success) {
				found = true;
				max_vars = g_NewVars;
				final_res = res;
				break;
			}
		}

	}

	auto scale = inv_transform_AAB(transform_AAB(g_fi.bbox(), AAB<2, double>(v2d(-1.0, -1.0), v2d(1.0, 1.0)))).first;

	auto [v, mdL, mdW] = final_res;
	auto prec = std::cout.precision();
	std::cout << "iterCVT:\tE ";
	printValidity(v);
	printMetric();
	std::cout << setprecision(4) << "\tminRelDL1 " << scale * mdL / g_targetLength;
	std::cout << '\n' << setprecision(prec);

	// int tmp;
	// tmp = getc(stdin);
	if(do_led_comparision) {
		exit(1);
	}

	return g_numLeds;
}

// try g_numIters times to fit g_numLeds LEDs in the triangle
tuple<Validity, double, double> allCVT(std::vector<double>& solVars) {
	auto best_res = make_tuple(Validity::overlap, 0.0, 0.0);
	std::vector<double> bestVars(g_NewVars.size());
	double max_dl1 = std::numeric_limits<double>::min();
	for (int i = 0; i < g_numIters; ++i) {
		auto res = iterCVT();
		double dl1 = get<1>(res) + get<2>(res);
		switch (get<0>(res)) {
			case Validity::outOfBounds:
				++g_numOOB;
				break;
			case Validity::overlap:
				++g_numOvlp;
				break;
			case Validity::success:
				++g_numSucc;
				break;
			case Validity::tooSmall:
				++g_numSmol;
				break;
			case Validity::notInsideWall:
				++g_numOOB;
				break;
			case Validity::edgeLengthConstrainViolated:
				++g_numEdge;
				break;
		}
		if ((get<0>(res) == Validity::success) && dl1 > max_dl1) {
			best_res = res;
			bestVars = g_NewVars;
			max_dl1 = dl1;
		} 
	}
	
	if (get<0>(best_res) == Validity::success) { solVars = bestVars; }
	return best_res;
}

void compare() {
	// run through all the leds and comapre their metric result
	auto res = make_tuple(Validity::success, 0.0, 0.0);
	while (g_numLeds > 0) {
		std::cout << "Placing " << g_numLeds << " LEDs...\n";
		res = allCVT(g_NewVars);
		if (get<0>(res) == Validity::success) {
			std::cout << "SUCCESS\n\n";
		}
		else {
			std::cout << "FAILURE\n\n";
		}
		// reduce the count
		g_numLeds--;
	}
}


// solve for best number of LEDs to fit in the triangle
int solve(int num_leds, bool once = false) {
	g_numLeds = num_leds;
	
	auto res = make_tuple(Validity::success, 0.0, 0.0);
	auto best_res = res;
	std::vector<double> best_vars(g_NewVars.size());
	bool first = true;
	bool found = false;
	bool prev_fail = false;
	while (first && g_numLeds > 0) {
		std::cout << "Placing " << g_numLeds << " LEDs...\n";
		res = allCVT(g_NewVars);
		if (get<0>(res) == Validity::success) {
			std::cout << "SUCCESS\n\n";
			found = true;
			if (prev_fail) { break; }
			best_vars = g_NewVars;
			best_res = res;
			++g_numLeds;
		} else {
			std::cout << "FAILURE\n\n";
			if (found) {
				std::cout << "Choosing previous best solution\n\n";
				g_NewVars = best_vars;
				res = best_res;
				--g_numLeds;
				break;
			}
			prev_fail = true;
			--g_numLeds;
		}
		if (once) first = false;
	}
	sl_assert(g_numLeds >= 0 || (get<0>(res) == Validity::success));	// sanity check
	return g_numLeds;
}

void clearGlobals() {
	g_Sites.clear();
	g_SitesPos.clear();
	g_NewVars.clear(); 
	g_Rects.clear();

	for (auto rigid : g_Rigid) 	{
		int rect_index = rigid.first_var_id;
		v2d p = v2d(g_NewVars[rect_index], g_NewVars[rect_index + 1]);
		double agl = g_NewVars[rect_index + 2];
		g_world.DestroyBody(rigid.body);
	}
	g_Rigid.clear();
	g_debug_Rects.clear();
	g_EdgesInspect.clear();
	g_Centers.fill(0);
	g_Forces.clear();

}

std::vector<v2d> initialize_globals(const FaceInfo& fi, const ModuleInfo& mi, const ModuleInfo& ci, const std::vector<double>& insets_mm) {

	g_fi = fi; // other information accept the scale and boundary box info that will get updated in this function
	auto [len, wid] = mi.dims;
	g_targetLengthMM = len;
	g_targetWidthMM = wid;
	g_targetAR = g_targetWidthMM / g_targetLengthMM;
	// update if it is a connector

	// std::cout << "LED\t" << len << " x " << wid << " (AR " << g_targetAR << ")\n";

	if (fi.has_connector) {
		auto [clen, cwid] = ci.dims;
		g_connectorLengthMM = clen;
		g_connectorWidthMM = cwid;
		g_connectorAR = g_connectorWidthMM / g_connectorLengthMM;
		// std::cout << "CON\t" << clen << " x " << cwid << " (AR " << g_connectorAR << ")\n";
	}

	// DONT LEAVE IT UNATTENDED
	g_offset = fi.hinge_offsets;
	if(generate_example_offset_showcase_example) {
		g_offset = vector<double>{3, 3, 3}; // Just for artifically creating offsets for an example in the paper 
	}

	// temporary calculations to make sure that right size bbox is generated taking outer offset triangle as the lead
	std::vector<v2d> tvertices = fi.vertices; 
	std::vector<v2d> tvertices_offset = tvertices;  
	ForIndex(i, 3) { // TRIANGLE DEPENDENT ASSUMPTION
		// negative offset we need to bleed out!!
		performTriangleOffset(tvertices_offset[(i + 2) % 3], tvertices_offset[i], tvertices_offset[(i + 1) % 3], -g_offset[i]); // creates an inverse offset val
	}

	auto [vmin, vmax] = face_bbox(tvertices_offset);
	g_min_corner = vmin;
	g_max_corner = vmax;

	auto to_neg1_pos1 = transform_AAB(AAB<2, double>(g_min_corner, g_max_corner), AAB<2, double>(v2d(-1.0, -1.0), v2d(1.0, 1.0)));

	g_vertices.clear();	
	auto ivs = inset_verts(fi.vertices, insets_mm); // add additional barrier around sheet file to add space for hinges
	for (size_t i = 0; i < ivs.size(); ++i) {
		g_vertices.push_back(apply_transform_AAB(to_neg1_pos1, ivs[i])); // this is where initial cvt will take place
	}

	// apply offset and calculate the g_vertices_offset as well. 
	g_vertices_offset.clear();
	for (size_t i = 0; i < fi.vertices.size(); ++i) { 
		// offset takes place from the original triangle and not the one where hinge dependent inset has taken place
		g_vertices_offset.push_back(apply_transform_AAB(to_neg1_pos1, fi.vertices[i]));
	}

	g_scale = to_neg1_pos1.first;
	ForIndex(i, 3) { // TRIANGLE DEPENDENT ASSUMPTION
		// negative offset we need to bleed out!!
		performTriangleOffset(g_vertices_offset[(i + 2) % 3], g_vertices_offset[i], g_vertices_offset[(i + 1) % 3], -g_offset[i] * g_scale); // creates an inverse offset val
	}

	g_vertices_current = g_vertices; // set the current triangle for which cvt is computed

	g_numLeds = LED_INITIAL_GUESS;
	g_targetLength = g_targetLengthMM * g_scale;
	g_connectorLength = g_connectorLengthMM * g_scale;
	g_RectSize = 0.5 * g_targetLength;
	g_connectorSize = 0.5 * g_connectorLength;

	if(generate_variation_example) {
		double max = *max_element(g_offset.begin(), g_offset.end()); // this gives me the offset element for the triangle
		if(max < 1) { // the triangle does not have any offsets. Half Hinge offsets start from 1.5, 1.9. NOTE BUT IT MAY HAVE SOME OFFSET FROM UNFOLDING HACK
			g_desired_lengthMM = 0; 

		}
		else if(max < 2.0) {// then the largest offset is created by a half hinge. in this case, use half hinge offset as the max-limit
			g_desired_lengthMM = 1.7 + 1.0 + 1.0;  // TODO add this in the global header file. params for hw, od and id
		}
		else {// then the largest offset is created by a full hinge. in this case, use full hinge offset as the max-limit
			g_desired_lengthMM = 2*(1.7+1.0) + 1.0;  // TODO add this in the global header file. params for hw, od and id
		}
		g_desired_length = g_desired_lengthMM * g_scale * 0.5;
	}

	if(user_specified_avg_length) { // take g_desired_lengthMM as input from user
		g_desired_length = g_desired_lengthMM * g_scale * 0.5; // because when computation happens it happens in [0,1] space
	}
	return ivs;
}

tuple<v2d, v2d, v2d> rect_to_module(t_rigid rect, pair<double, v2d> to_world) {
	int n = rect.first_var_id;
	bool is_connector = rect.is_connector;
	double lenMM = is_connector ? g_connectorLengthMM : g_targetLengthMM;
	double widMM = is_connector ? g_connectorWidthMM  : g_targetWidthMM;

	// vars in [0, 1], p0 & p1 in [-1, 1]
	// v2d p0 = 2.0 * v2d(g_Vars[n]    , g_Vars[n + 1]) - v2d(1.0);
	// v2d p1 = 2.0 * v2d(g_Vars[n + 2], g_Vars[n + 3]) - v2d(1.0);

	t_rect_geometry rect2 = get_rect(n);

	v2d p0 = rect2[0], p1 = rect2[1], p2 = rect2[2], p3 = rect2[3];

	// original coords (in mm)
	p0 = apply_transform_AAB(to_world, p0);
	p1 = apply_transform_AAB(to_world, p1);

	p2 = apply_transform_AAB(to_world, p2);
	p3 = apply_transform_AAB(to_world, p3);

	return make_tuple(p0, p1, p3);
}

void process_face_info(const Settings& stgs, FaceInfo& fi, const ModuleInfo& mi, const ModuleInfo& ci, size_t& total_leds) {
	// std::cout << "SFID " << fi.fid << '\t' << fi.run_pnr << ' ' << fi.run_verif << ' ' << fi.failed_pnr << ' ' << fi.failed_verif << '\n';
	if (!fi.run_pnr) { return; } // run placement and routing (if false that means this triangle is routed)
	Timer tm("[place_fi]");
	debug_info(fi);
	
	// std::cout << "\nFace " << fi.fid << '\n';

	auto insets = led_insets(stgs, fi);
	auto ivs = initialize_globals(fi, mi, ci, insets);

	auto to_neg1_pos1 = transform_AAB(AAB<2, double>(g_min_corner, g_max_corner), AAB<2, double>(v2d(-1.0, -1.0), v2d(1.0, 1.0)));
	auto to_world = inv_transform_AAB(to_neg1_pos1);

	// std::cout << "BBOX " << g_min_corner << ' ' << g_max_corner << '\n';
	// std::cout << "SV ";
	// for (auto v : g_vertices) { //in [-1:1] space
	// 	std::cout << v << ' ';
	// }	std::cout << '\n';
	
	// std::cout << "OSV ";
	// for (auto v : g_vertices_offset) { // in [-1,1] space
	// 	std::cout << v << ' ';
	// }	std::cout << '\n';

	initSites();
	b2InitScene();
	renderCVT();

	int num_rects = fi.modules.size() + (fi.has_connector ? 1 : 0);
	int num_rects_new = 0;
	if (fi.failed_pnr || fi.failed_verif) {
		// if done specified amount of tries, decrease num LEDs
		std::cout << fi.pnr_runs_same_LEDs << " of " << g_numTries << '\n';
		if (fi.pnr_runs_same_LEDs >= g_numTries) {
			std::cout << "Done with tries, removing an LED\n";
			fi.pnr_runs_same_LEDs = 0;
			num_rects_new = solve_stuff(num_rects - 1);
		} else {
			std::cout << "Some tries left, trying same number of LEDs\n";
			num_rects_new = solve_stuff(num_rects);
		}
		// if same number of LEDs than before, increment number of tries
		std::cout << "LEDs bef/after: " << num_rects << " / " << num_rects_new << '\n';
		if (num_rects == num_rects_new) { 
			std::cout << "Same number of LEDs\n";	
			++fi.pnr_runs_same_LEDs;
		} else { 
			std::cout << "Different number of LEDs\n";
			fi.pnr_runs_same_LEDs = 1;
		}
	} else {
		num_rects_new = solve_stuff();
		fi.pnr_runs_same_LEDs = 1;
	}
	total_leds += num_rects_new;
	// increase stats for placer
	
	fi.modules.clear();
	for (auto r : g_Rigid) {
		if (r.is_connector) {
			fi.connector = rect_to_module(r, to_world);
			auto [p0, p1, p2] = fi.connector;
			// std::cout << "C " << p0 << '\t' << p1 << '\t' << p2 << '\n';
		} else {
			fi.modules.push_back(rect_to_module(r, to_world));
			auto [p0, p1, p2] = fi.modules[fi.modules.size() - 1];
			// std::cout << "M " << p0 << '\t' << p1 << '\t' << p2 << '\n';
		}
	}

	++fi.stats_overall[0];
	PlacerRecord rec = {fi.num_rects(), tm.elapsed()};
	fi.stats_detailed.push_back(rec);
}

size_t main_function2(
	const std::string& file_stg, const std::string& file_fis, const std::string& file_out)
{
	Settings stgs = read_settings(file_stg);
	g_numIters = stgs.placer_iters;
	g_numTries = stgs.placer_tries;
	g_vorlayout_offset_x = stgs.led_offset_x_mm;
	g_vorlayout_offset_y = stgs.led_offset_y_mm;

	std::vector<FaceInfo> fis;
	fis = parse_face_info(file_fis);

	std::vector<FaceInfo> modified_faces;
	ModuleInfo mi = parse_module_info(stgs.file_module);
	ModuleInfo ci = parse_module_info(stgs.file_connector);

	std::cout << "LED\t" << mi.dims.first << " x " << mi.dims.second << "\n";
	std::cout << "CON\t" << ci.dims.first << " x " << ci.dims.second << "\n";

	size_t total_leds = 0;
	// size_t specific_face = 14;

	for (auto& fi : fis) {
		process_face_info(stgs, fi, mi, ci, total_leds);
	}

	write_face_info(file_out, fis);
	std::cout << "\n\nTotal number of LEDs placed: " << total_leds << '\n';

	if(compute_average_timing) {
		printf("Number of times: ITERCVT was called: %d\n", totalIterations);
		printf("Average time for different steps each time iterCVT is called:\n");
		printf("Step1: %f\n", average(step1times));
		printf("Step2: %f\n", average(step2times));
		printf("Step3: %f\n", average(step3times));
		printf("Step4: %f\n", average(step4times));
	}

	return total_leds;
}

// size_t main_function(
// 	const std::string& file_stg,
// 	const std::string& file_sht, const std::string& file_mod,
// 	const std::string& file_con, const std::string& file_rct,
// 	const std::string& file_filter, const std::vector<size_t>& conn_fids)
// { // fcs --> sht
// 	Settings stgs = read_settings(file_stg);
// 	Sheet sh;
// 	sh.read_sheet(file_sht);
// 	sh.validate_sheet();
// 	sh.compute_order();
// 	sh.compute_bfs_depth();
// 	sh.compute_patch_bbox();
// 	auto faces = sh.get_faces({-1});

// 	// auto faces = parse_sheet_info(file_sht);
// 	// auto faces = parse_face_info(file_fcs);

// 	std::vector<std::pair<size_t, std::optional<size_t>>> face_filter;
// 	std::vector<std::pair<size_t, std::optional<size_t>>> output_filter;
	
// 	if(file_filter.size()!= 0) {
// 		face_filter = parse_filter(file_filter);	
// 	}
// 	else {
// 		for(int fi = 0; fi < faces.size(); fi++) {
// 			auto val = std::make_pair(fi, std::optional<size_t>());
// 			face_filter.push_back(val);
// 		}
// 	}


// 	std::vector<size_t> excl;
// 	std::transform(face_filter.begin(), face_filter.end(), std::back_inserter(excl),
// 		[](auto p){ return p.first; });

// 	std::vector<FaceInfo> modified_faces;
// 	ModuleInfo mi = parse_module_info(file_mod);
// 	ModuleInfo ci = parse_module_info(file_con);

// 	size_t total_leds = 0;
// 	size_t specific_face = 14;

// 	for (const auto& [fid, opt_max] : face_filter) {
// 		size_t k_fid = 0;
// 		for (k_fid = 0; k_fid < faces.size(); ++k_fid) {
// 			if (faces[k_fid].fid == fid) { break; }
// 		}
// 		auto& face = faces.at(k_fid);
// 		if (std::find(conn_fids.begin(), conn_fids.end(), fid) != conn_fids.end()) {
// 			face.has_connector = true;
// 		}
// 		// if (fid != specific_face) { continue; }
// 		std::cout << "\nFace " << fid << '\n';

// 		auto insets = led_insets(stgs, face);
// 		auto ivs = initialize_globals(face, mi, ci, insets);

// 		auto to_neg1_pos1 = transform_AAB(AAB<2, double>(g_min_corner, g_max_corner),
// 										  AAB<2, double>(v2d(-1.0, -1.0), v2d(1.0, 1.0)));
// 		auto to_world = inv_transform_AAB(to_neg1_pos1);
		
// 		std::cout << "BBOX " << g_fi.min_corner << ' ' << g_fi.max_corner << '\n';

// 		std::cout << "SV ";
// 		for (auto v : g_vertices) {
// 			std::cout << v << ' ';
// 		}	std::cout << '\n';
		
// 		std::cout << "OSV ";
// 		for (auto v : g_vertices_offset) {
// 			std::cout << v << ' ';
// 		}	std::cout << '\n';

// 		initSites();
// 		b2InitScene();
// 		renderCVT();

// 		if (opt_max.has_value()) {
// 			assert(opt_max > 0);
// 			total_leds += solve_stuff(opt_max.value() - 1);
// 		} else {
// 			total_leds += solve_stuff();
// 		}

// 		if(STATE == SimState::PointRigidBody) {
// 			if (!g_fi.has_connector) {
// 				double face_area = polygon_area(ivs);
// 				double total_mod_area = 0.0;
// 				for (auto r : g_Rigid) {
// 					auto [p0, p1, p2] = rect_to_module(r, to_world);
// 					total_mod_area += distance(p0, p1) * distance(p0, p2);
// 				}
// 				std::cout << "Face area / Mod area\t" << face_area << '\t' << total_mod_area << '\n';
// 				g_modToTriAreaRatios.push_back(total_mod_area / face_area);
// 			}

// 			for (auto r : g_Rigid) {
// 				if (r.is_connector) {
// 					face.connector = rect_to_module(r, to_world);
// 					auto [p0, p1, p2] = face.connector;
// 					std::cout << "C " << p0 << '\t' << p1 << '\t' << p2 << '\n';
// 				} else {
// 					face.modules.push_back(rect_to_module(r, to_world));
// 					auto [p0, p1, p2] = face.modules[face.modules.size() - 1];
// 					std::cout << "M " << p0 << '\t' << p1 << '\t' << p2 << '\n';
// 				}
// 			}
// 		}
// 		else { 
// 			printf("NOT IMPLEMENTED YET!!\n");
// 			exit(1);
// 		}

// 		modified_faces.push_back(face);
// 		output_filter.push_back({fid, face.modules.size() + (face.has_connector ? 1 : 0)});
// 	}

// 	update_face_info(file_rct, modified_faces);
// 	if (!file_filter.empty()) { write_filter(file_filter, output_filter); }

// 	std::cout << "\n\nTotal number of LEDs placed: " << total_leds << '\n';

// 	if(compute_average_timing) {
// 		printf("Number of times: ITERCVT was called: %d\n", totalIterations);
// 		printf("Average time for different steps each time iterCVT is called:\n");
// 		printf("Step1: %f\n", average(step1times));
// 		printf("Step2: %f\n", average(step2times));
// 		printf("Step3: %f\n", average(step3times));
// 		printf("Step4: %f\n", average(step4times));
// 	}

// 	return total_leds;
// }

void mainKeyboard(uchar k) {
	bool do_opt = false;
	bool do_step = false;
	bool do_iter = false; // do single iteration

	if (k == 'q') {
		SimpleUI::exit();
	}
	else if (k == 'w') {
		int tmp;
		tmp = getc(stdin);
	}
	else if (k == '+') {
		g_targetLength = g_targetLength * 2.0;
		do_opt = true;
	}
	else if (k == '-') {
		g_targetLength = g_targetLength / 2.0;
		do_opt = true;
	}
	else if (k == ' ') {
		optCVT(1);
	}
	else if (k == 'l') {
		computeEdgeLengthMetric();
	}
	else if (k == '0') {
		optCVT(0);
	}
	else if (k == '1') {
		optCVT(1);
	}
	else if (k == '2') {
		optCVT(2);
	}
	else if (k == '3') {
		optCVT(3);
	}
	else if (k == '4') {
		optCVT(4);
	}
	else if (k == '5') {
		optCVT(5);
	}
	else if (k == '*') {
		// save result
		static int cnt = 0;
		ImageRGBA img;
		RenderTarget2DRGBA_Ptr rt(new RenderTarget2DRGBA(R, R));
		rt->bind();
		glViewport(0, 0, rt->w(), rt->h());
		displayCVT();
		rt->unbind();
		rt->readBack(img.pixels());
		img.flipH();
		saveImage(sprint("../results/iter_%d.png", cnt++), img.cast<ImageRGB>());
		printf("Image taken\n");
	}
	else if (k == 'r') { // reset
		initSites();
	}
	else if (k == 'o') { // Test overlap of triangles
		bool valid = are_rects_separate();
		if (not valid) {
			printf("there are overlapping rectangles\n");
		}
	}
	else if (k == 'y') { // rectangles --> rigid bodies
		transformRects();
	}
	else if (k == 'd') { // rectangles --> rigid bodies
		displayRigid = !displayRigid;
	}
	// else if (k == 'g') { // 	do_step = true;
	// }
	else if (k == 'a') {
		solve(g_numLeds, false);
	}
	else if (k == 'i') {
		do_iter = true;
	}
	else if (k == 's') {
		solve(g_numLeds, true);
	}
	else if (k == 'c') {
		compare();
	}
	else if (k == 'v') {
		float timeStep = 1.0f / 30.0f;
		int32 velocityIterations = 6;
		int32 positionIterations = 2;
		g_world.Step(timeStep, velocityIterations, positionIterations);
		// refreshSitesPos(&g_NewVars[0]);
		// renderCVT();
	}

	if (do_opt) {
		optCVT(1);
	}
	if (do_iter) {
		iterCVT(1);
	}
}

/* -------------------------------------------------------- */

void mainReshape(uint w,uint h)
{
	glViewport(0,0,w,h);
	g_W = w;
	g_H = h;
}

/* -------------------------------------------------------- */

void quad()
{
	glBegin(GL_QUADS);
	glTexCoord2f(0,0); glVertex3f(-0.5f,-0.5f,0);
	glTexCoord2f(1,0); glVertex3f( 0.5f,-0.5f,0);
	glTexCoord2f(1,1); glVertex3f( 0.5f, 0.5f,0);
	glTexCoord2f(0,1); glVertex3f(-0.5f, 0.5f,0);
	glEnd();
}


/* -------------------------------------------------------- */
// displays to screen
void displayCVT() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	GPUHelpers::Transform::ortho2D(LIBSL_PROJECTION_MATRIX, 0.0f, 1.0f, 0.0f, 1.0f, -1.0f, 1.0f);
	/*
	AAB<2,double> bx;
	for (auto s : g_SitesPos) {
		bx.addPoint(s);
	}
	GPUHelpers::Transform::ortho2D(LIBSL_PROJECTION_MATRIX,
		bx.minCorner()[0], bx.maxCorner()[0], bx.minCorner()[1], bx.maxCorner()[1],
		-1.0f, 1.0f);
	*/

	GPUHelpers::Transform::identity(LIBSL_MODELVIEW_MATRIX);

	glColor3f(1, 1, 1);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glBindTexture(GL_TEXTURE_2D, g_RTVoronoi->texture(1)); // each pixel contains voronoi cell id
	glEnable(GL_TEXTURE_2D);
	glPushMatrix();
	glTranslated(0.5, 0.5, 0);
	quad();
	glPopMatrix();
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);

	// render rectangles
	glColor3f(0.5, 0, 0.5);
	// rectangles
	if ((STATE == SimState::RigidBody or STATE == SimState::PointRigidBody) and displayRigid) { // it has to be in the state of rectangles for it be actiavated.
		glColor3f(0.1, 0.1, 0.7);
		for (auto r : g_Rigid) {
			b2Vec2 position = r.body->GetPosition();
			float angle = r.body->GetAngle();
			glPushMatrix();
			glTranslatef((position.x / BOX2D_SCALE), (position.y / BOX2D_SCALE), 0.0f);
			glRotatef(angle * 180.0f / M_PI, 0, 0, 1);
			glScalef(r.sx / BOX2D_SCALE, r.sy / BOX2D_SCALE, 1.0f);
			quad();
			glPopMatrix();
		}
	}

	if ((STATE == SimState::PointRigidBody) and displayRigid) { // it has to be in the state of rectangles for it be actiavated.
		glColor3f(0.7, 0.1, 0.1);
		for (auto r : g_Rigid) {
			b2Vec2 position = r.body->GetPosition();
			float angle = r.body->GetAngle();
			glPushMatrix();
			glTranslatef((position.x / BOX2D_SCALE), (position.y / BOX2D_SCALE), 0.0f);
			glRotatef(angle * 180.0f / M_PI, 0, 0, 1);
			bool is_connector = r.is_connector;
			double len = is_connector ? g_connectorLength : g_targetLength;
			double ar = is_connector ? g_connectorAR : g_targetAR;
			if(is_connector) {
				glScalef(g_connectorSize, g_connectorAR * g_connectorSize, 1.0f);
			}
			else {
				glScalef(g_RectSize, g_targetAR * g_RectSize, 1.0f);
			}
			quad();
			glPopMatrix();
		}
	}	

	// render centers
	glPointSize(5.0f);
	glColor3f(0, 0, 0);
	glBegin(GL_POINTS);
	ForArray(g_Centers, c) {
		glVertex2dv(&g_Centers[c][0]);
	}
	glEnd();
	glPointSize(3.0f);
	glColor3f(1, 0, 0);
	glBegin(GL_POINTS);
	ForArray(g_Centers, c) {
		glVertex2dv(&g_Centers[c][0]);
	}
	glEnd();

	glPointSize(5.0f);
	glColor3f(0, 0, 0);
	glBegin(GL_POINTS);
	ForArray(g_SitesPos, c) {
		glVertex2dv(&g_SitesPos[c][0]);
	}
	glEnd();
	glPointSize(3.0f);
	glColor3f(0, 1, 0);
	glBegin(GL_POINTS);
	ForArray(g_SitesPos, c) {
		glVertex2dv(&g_SitesPos[c][0]);
	}
	glEnd();

	// // display static wall when in rigid body mode
	// printf("------------------------------\n");
	if (STATE == SimState::RigidBody or STATE == SimState::PointRigidBody) {
		ForIndex(i, 3) {
			glPushMatrix();
			b2Vec2 wall_pos = wall[i].body->GetPosition();
			float angle = wall[i].body->GetAngle();
			glTranslatef(wall_pos.x / BOX2D_SCALE, wall_pos.y / BOX2D_SCALE, 0.0f);
			glRotatef(angle * 180.0f / M_PI, 0, 0, 1);
			glScalef(wall[i].sx / BOX2D_SCALE, wall[i].sy / BOX2D_SCALE, 1.0f);
			quad();
			glPopMatrix();
			// printf("Wall[%d] position: %f %f angle : %f\n", i, wall_pos.x / BOX2D_SCALE, wall_pos.y / BOX2D_SCALE, angle);
		}
	}

	// rectangle centers
	if (STATE == SimState::RigidBody or STATE == SimState::PointRigidBody) { // it has to be in the state of rectangles for it be actiavated.
		glColor3f(1.0, 1.0, 0.0);
		glPointSize(1.0f);
		glBegin(GL_POINTS);
		for (auto r : g_Rigid) {
			b2Vec2 position = r.body->GetPosition();
			v2d positionX = v2d(position.x / BOX2D_SCALE, position.y / BOX2D_SCALE);
			glVertex2dv(&positionX[0]);
		}
		glEnd();
	}

	if ((STATE == SimState::RigidBody or STATE == SimState::PointRigidBody) and displayRigid) {
		glBegin(GL_LINES);
		glColor3f(1, 1, 1);
		for (auto f : g_Forces) {
			glVertex2dv(&(f.second)[0]);
			glVertex2dv(&(f.second + f.first)[0]);
		}
		glEnd();

		// Net force
		glBegin(GL_LINES);
		glColor3f(1, 0, 0);
		for (auto f : g_NetForces) {
			glVertex2dv(&(f.second)[0]);
			glVertex2dv(&(f.second + f.first)[0]);
		}
		glEnd();
	}

	glBegin(GL_LINES);
	glColor3f(0, 0, 1);
	ForIndex(s, g_EdgesInspect.size()) {
		glVertex2dv(&g_SitesPosInspect[get<0>(g_EdgesInspect[s])][0]);
		glVertex2dv(&g_SitesPosInspect[get<1>(g_EdgesInspect[s])][0]);
	}
	glEnd();

	// draw triangle distance
	glBegin(GL_LINES);
	glColor3f(1, 1, 1);
	ForIndex(s, g_Sites.size()) {
		auto dg = externalDistance(g_SitesPos[s] * 2.0 - v2d(1.0));
		v2d pt = g_SitesPos[s] + normalize(dg.second) * dg.first * 0.5;
		// 0.5 is just the scaling factor to bring it down to 0 and 1 scaling range rather than -1 and 1 range.
		glVertex2dv(&g_SitesPos[s][0]);
		glVertex2dv(&pt[0]);
	}
	glEnd();

	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);

	if (0) {
		// save result
		static int cnt = 0;
		ImageRGBA img;
		g_RTVoronoi->readBack(img.pixels(), 2);
		img.flipH();
		saveImage(sprint("result\\iter_%d.tga", cnt++), img.cast<ImageRGB>());
	}
}

// only checks corners
bool is_rect_in_domain(size_t rid) {
	auto rect = get_rect(rid, g_vorlayout_offset_x, g_vorlayout_offset_y);
	// std::cout << "rect: " << rect[0] << '\t' << rect[1] << '\t' << rect[2] << '\t' << rect[3] << '\n';
	for (auto pt : rect) {
		// std::cout << pt << '\n';
		if (!is_v2d_in_domain(pt, g_vertices))  {
			return false;
		}
	}
	return true;
}

/* -------------------------------------------------------- */
// computes voronoi diagram
void renderCVT()
{
	g_RTVoronoi->bind();	// draw in texture (render target)
	glViewport(0, 0, g_RTVoronoi->w(), g_RTVoronoi->h());

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	GPUHelpers::Transform::ortho2D(LIBSL_PROJECTION_MATRIX, 0.0f, 1.0f, 0.0f, 1.0f, -1.0f, 1.0f);
	GPUHelpers::Transform::identity(LIBSL_MODELVIEW_MATRIX);

	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glColor4f(1, 1, 1, 1);

	glEnable(GL_DEPTH_TEST);

	float s = 1.5f;
	g_CVTShader.begin();

	// face vertices
	int num_verts = g_vertices_current.size(); 
	GLParameter pts; 	pts.init(g_CVTShader, "u_pts");
	GLParameter numPts; numPts.init(g_CVTShader, "u_numPts");
	numPts.set(num_verts);
	Array<v2f> ps(num_verts);
	ps.fill(v2f(0.0));
	for (size_t i = 0; i < num_verts; ++i) {
		ps[i] = v2f(g_vertices_current[i]);
	}
	pts.setArray(ps.raw(), num_verts);

	GLParameter weight;	weight.init(g_CVTShader, "u_CentroidWeight");
	weight.set(1.0f);
	ForIndex(c,g_Sites.size()) {
		glPushMatrix(); 
		glTranslatef(g_SitesPos[c][0], g_SitesPos[c][1],0);
		glScalef    (s,s,1.0f);
		int id0 = c      & 255;
		int id1 = (c>>8) & 255;
		g_glCentroidPos.set(v2f(g_SitesPos[c]));
		g_glCentroidId .set( (float)(id0+0.5f)/255.0f , (float)(id1+0.5f)/255.0f );
		// frac(vec4(id*1.231357487,id*id*31.7879,1.978756412/id,1))
		g_glCentroidClr.set( 0.9f * randomColorFromIndex(g_Sites[c].first_var_id) );
		quad();	// draws quad per site
		glPopMatrix(); 
	}

	g_CVTShader.end();

	glDisable (GL_DEPTH_TEST);

	g_RTVoronoi->unbind();
}

/* -------------------------------------------------------- */

void newiteration(int iter, int call_iter, double *x, double* f, double *g,  double* gnorm)
{
	//std::cerr << iter <<": " << call_iter <<" " << *f <<" " << *gnorm  << std::endl;
}

/* -------------------------------------------------------- */

// from new x compute function value and gradient
void evalfunc(int N, double *x, double *prev_x, double *f, double *g) {
	// dispatch current positions to sites
	refreshSitesPos(x); // g_SitesPos gets updated with this! x is basically g_NewVars

	// render CVT
	renderCVT();

	// read back
	Array2D<v4b> pixs;
	g_RTVoronoi->readBack(pixs, 0);

	// compute new centroids
	Array<int> cnt(g_Sites.size());
	Array<v2d> gctr(g_Sites.size());
	cnt.fill(0);
	gctr.fill(v2d(0));

	g_Centers.allocate(g_Sites.size());

	// => f
	double cvt_energy = 0;
	*f = 0.0;
	if (w_cvt > 0) {
		ForArray2D(pixs, i, j) {
			int id = pixs.at(i, j)[0] + (pixs.at(i, j)[1] << 8); // cell id
			if (id < g_Sites.size()) {																			  // false if outside the domain
				v2d p = (v2d(i, j) + v2d(0.5)) / v2d(g_RTVoronoi->w(), g_RTVoronoi->h()); // pixel center coordinates (screen space) [0 and 1]
				gctr[id] += p;
				cnt[id]++; // cell area
				cvt_energy += w_cvt * sqLength(g_SitesPos[id] - p);
				*f += w_cvt * sqLength(g_SitesPos[id] - p);
			}
		}
	}

	// normalize weights
	double domain_area = 0.0;
	ForIndex(i, cnt.size()) {
		domain_area += cnt[i];
	}

	// triangle distance energy
	if (w_tri > 0) {
		for (auto s : g_Sites) {
			if (holds_alternative<t_indepedent_site>(s.nfo)) {
				auto dg = externalDistance(g_SitesPos[s.site_id] * 2.0 - v2d(1.0)); // in -1 to 1
				*f += w_tri * dg.first;
			}
			else {
				sl_assert(false);
			}
		}
	}

	// rectangles
	double len_target = g_RectSize * g_RectSize;
	// printf("length target is : %f\n", len_target);

	// debug
	ForIndex(i, gctr.size()) {
		g_Centers[i] = gctr[i] / (double)cnt[i]; // cell centers
	}

	// => g (The gradient --> same size as the variables)
	memset(g, 0, sizeof(double) * g_NewVars.size());
	double distance_energy2 = 0;
	for (auto &s : g_Sites) {
		int n = s.first_var_id;
		int c = s.site_id;
		double px = gctr[c][0];
		double py = gctr[c][1];
		if (holds_alternative<t_indepedent_site>(s.nfo)) {
			auto dg = externalDistance(g_SitesPos[s.site_id] * 2.0 - v2d(1.0)); // in -1 and 1
			double x0 = x[n + 0] * cnt[c];
			double y0 = x[n + 1] * cnt[c];
			// derivative in x0
			g[n + 0] += w_cvt * (2 * x0 - 2 * px);
			// derivative in y0
			g[n + 1] += w_cvt * (2 * y0 - 2 * py);
			// triangle distance
			g[n + 0] += -w_tri * dg.second[0];
			g[n + 1] += -w_tri * dg.second[1];
			// printf("gradient : %d %f %f\n", s.first_var_id, w_tri * dg.second[0], w_tri * dg.second[1]);
			// printf("gradient : %d %f %f\n", s.first_var_id, w_cvt*(2*x0 - 2*px), w_cvt*(2*y0 - 2*py));
		}
		else {
			sl_assert(false);
		}
	}

	// show intermediate steps on screen
	quickRender();
}

/* -------------------------------------------------------- */

void mainRender()
{
	glClearColor(0,0,0.5,0);
	glViewport(0,0,g_W,g_H);

	displayCVT();
}

// --------------------------------------------------------------

bool g_Painting = false;

void mainMouseButton(uint x,uint y,uint button,uint flags)
{
	g_Painting = (flags == LIBSL_BUTTON_DOWN);
}

void mainMouseMotion(uint x,uint y)
{
	/*
	static Every ev(25);

	if (g_Painting && ev.expired()) {
	t_Centroid c;
	c.s      = V2F(x / (float)g_W , y / (float)g_H)  + 0.0001f * V2F(srnd(),srnd());
	c.weight = 1.0f;
	g_Centroids.push_back( c );
	}
	*/
}

// --------------------------------------------------------------

std::string interface() {
	return "./Cutter inp:<*.cfg> inp:<*.fis> (out:<*.fis>)\n";
}

void print_info(
	const std::string& file_stg,
	const std::string& file_fis,
	const std::string& file_out)
{
	std::cout << "PLACER\n";
    std::cout << "file_stg\t" << file_stg << '\n';
    std::cout << "file_fis\t" << file_fis << '\n';
	std::cout << "file_out\t" << file_out << '\n';
}

int main(int argc, char **argv) {
	std::string file_stg{};
	std::string file_fis{};
	std::string file_out{};

	// Input handling
	if (argc == 3) {
		file_stg = argv[1];
		file_fis = argv[2];
		file_out = file_fis;
	} else if (argc == 4) {
		file_stg = argv[1];
		file_fis = argv[2];
		file_out = argv[3];
	} else {
		std::cerr << "[ERROR] Wrong number of arguments\n";
		std::cerr << interface();
		exit(EXIT_FAILURE);
	}
	print_info(file_stg, file_fis, file_out);

	auto globalTimerStart = std::chrono::system_clock::now();
	try {

		SimpleUI::onRender             = mainRender;
		SimpleUI::onKeyPressed         = mainKeyboard;
		SimpleUI::onReshape            = mainReshape;
		SimpleUI::onMouseMotion        = mainMouseMotion;
		SimpleUI::onMouseButtonPressed = mainMouseButton;

		SimpleUI::init(g_ui ? g_W : 1, g_ui ? g_H : 1, "");

		// interval tex. shader
		g_CVTShader    .init (vp_voronoi,fp_voronoi);
		g_CVTShader    .setStrict(false);
		g_CVTShader    .begin();
		g_glCentroidId .init(g_CVTShader,"u_CentroidId");
		g_glCentroidPos.init(g_CVTShader,"u_CentroidPos");
		g_glCentroidE  .init(g_CVTShader,"u_CentroidE");
		g_glCentroidClr.init(g_CVTShader,"u_CentroidColor");
		g_glMap        .init(g_CVTShader,"u_Map");
		g_CVTShader    .end();

		// rt
    	g_RTVoronoi = RenderTarget2DRGBA_Ptr(new RenderTarget2DRGBA(R, R, 0, 2));
		
		size_t face_leds;
		face_leds = main_function2(file_stg, file_fis, file_out);
		
		// if (g_ui) { SimpleUI::loop(); }
		if(compute_average_timing) {
			std::chrono::duration<double> endTime = std::chrono::system_clock::now() - globalTimerStart;
			printf("Total time taken to run: %f\n", endTime.count());
		}

		// cleanup
		g_CVTShader.terminate();
		SimpleUI::exit();
	} catch (Fatal& e) {
		cerr << "[ERROR] " << e.message() << endl;
	}
}

void takeImage(string filename) {
	ImageRGBA img;
	RenderTarget2DRGBA_Ptr rt(new RenderTarget2DRGBA(512, 512));
	rt->bind();
	glViewport(0, 0, rt->w(), rt->h());
	displayCVT();
	rt->unbind();
	rt->readBack(img.pixels());
	img.flipH();
	saveImage(filename.c_str(), img.cast<ImageRGB>());
}
/* -------------------------------------------------------- */
