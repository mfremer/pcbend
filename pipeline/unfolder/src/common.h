#pragma once
#include "Settings.hpp"
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <imgui.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/false_barycentric_subdivision.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/edge_topology.h>
#include <igl/edges.h>
#include <igl/unproject.h>
#include <igl/barycenter.h>
#include <igl/orient_outward.h>
#include <igl/material_colors.h>
#include <igl/png/writePNG.h>
#include <bits/stdc++.h>
#include <igl/gaussian_curvature.h>
#include <map>
#include <stack>
#include <functional>
#include <queue>
#include <algorithm>
/*#include <clipper.hpp>*/

// #include <LBFGS.h>

using namespace std;

//------------------------------------------------------------------------------
//	Common variables
//------------------------------------------------------------------------------
#define green_color Eigen::Vector3d(0,1,0);
#define red_color Eigen::Vector3d(1.0,0.0,0.0);
#define blue_color Eigen::Vector3d(0.0,0.0,1.0);
#define black_color Eigen::Vector3d(0.0,0.0,0.0);

// extern double trans_pos;

typedef std::tuple<int, int, int> triplet;
typedef std::tuple<int, int, int, int> quadret;

typedef std::pair<double, double> TriPoint;

struct queueComp {
    constexpr bool operator()(
        pair<int, double> const& a,
        pair<int, double> const& b)
        const noexcept
    {
        return a.second < b.second;
    }
};

struct queueComp2 { // high to low queue
    constexpr bool operator()(
        pair<int,int>& a,
        pair<int,int>& b)
        const noexcept
    {
        return a.second < b.second;
    }
};


struct queueComp3 { // low to high queue
    constexpr bool operator()(
        pair<int,int>& a,
        pair<int,int>& b)
        const noexcept
    {
        return a.second > b.second;
    }
};

struct queueComp4 { // low to high queue
    constexpr bool operator()(
        pair<pair<int, int>,int>& a,
        pair<pair<int, int>,int>& b)
        const noexcept
    {
        return a.second > b.second;
    }
};

struct meshEdge {
    int hingeId = -1;
    pair<int, int> faces;
    double offset = 0; // remember this offset is defined per edge while the offset in computeOffset is from its opposite vertex
    bool isHinge = false;
    bool isBoundary = false; 
    pair<int, int> orderedVertices;
    bool isHalfHinge = false; 
};

struct transNode {
    int id;
    shared_ptr<transNode> parent = nullptr;
    vector<shared_ptr<transNode>> kids;
    Eigen::Vector3d translation; 
};

// In general implementation
class DSU // disjoint set union
{
    int *parent;
    int *rank;

public:
    DSU(int n);
    // Find function
    int find(int i);
    // union function
    void unite(int x, int y);
};

class Graph
{
    vector<vector<double>> edgelist; // 
    int N; // number of vertices
public:
    Graph(int N); 
    void addEdge(int v0, int v1, double weight, int edge_id);
    double kruskals_mst(std::vector<std::vector<int>> &mst_edges);
};

//------------------------------------------------------------------------------
//	Functions in mesh.cpp
//------------------------------------------------------------------------------

void computeBoundedBox(const igl::opengl::ViewerData &sheet_mesh, double &xmin, double &xmax, double &ymin, double &ymax);

std::pair<int, int> getCommonVertices(const igl::opengl::ViewerData &mesh, int face0, int face1, bool orderThem);

void saddleMesh(const igl::opengl::ViewerData & mesh, igl::opengl::ViewerData & saddle_mesh); 
// takes the original mesh and adds spheres in the mesh at saddle vertices...

void addSphere(Eigen::Vector3d pos, std::vector<Eigen::Vector3d> &vertices, std::vector<Eigen::Vector3i> &faces, double radius = 0.02);
// add a sphere at a given location pos

void isSaddle(const igl::opengl::ViewerData &mesh, Eigen::VectorXi &saddleBoolArray); 
// checks if a given vertex is saddle vertex or not..


void getMeshData(const igl::opengl::ViewerData &mesh, Eigen::MatrixXi &E2V, Eigen::MatrixXi &F2E, Eigen::MatrixXi &F2V);

void getMeshEdges(const igl::opengl::ViewerData &mesh, Eigen::MatrixXi &edges);

void saveMesh(std::string fname, const igl::opengl::ViewerData &mesh_data);

void saveSheet(const igl::opengl::ViewerData &sheet_data, const igl::opengl::ViewerData &sheet_clean_data, 
            const vector<vector<int>> &sheet_hinge_quads, vector<meshEdge> &mesh_edges, map<int, triplet> &meshFace2Edge,
            std::vector<std::vector<int>> &flatFaceVisitOrder, int numFaces, string fname, double drill_radius, double user_margin);

void fixMeshOrientation(igl::opengl::ViewerData &mesh);

//------------------------------------------------------------------------------
//	Functions in dualMesh.cpp
//------------------------------------------------------------------------------
void dualMesh(const igl::opengl::ViewerData &mesh, igl::opengl::ViewerData &dual_mesh, Eigen::MatrixXi &dual_edges, Eigen::MatrixXi &visual_dual_edges, Eigen::MatrixXi &boundary_edges);
// computes dual mesh (for visualization) and stores it in dual_mesh. Also computes dual_edges for computation.

void setDualMeshColor(igl::opengl::ViewerData &dual_mesh, const Eigen::MatrixXi &visual_dual_edges, const Eigen::MatrixXd MST_color = Eigen::MatrixXd()); 
// takes the dual mesh and dual edges and sets its color based on MST_color preformed after MST

void getDualEdgeCorrespondance(const Eigen::MatrixXi &dual_edges, Eigen::MatrixXi &edges);

//------------------------------------------------------------------------------
//	Functions in mst.cpp
//------------------------------------------------------------------------------
void MST(const igl::opengl::ViewerData &mesh, const igl::opengl::ViewerData &dual_mesh, const Eigen::MatrixXi &dual_edges,
         igl::opengl::ViewerData &mst_mesh, Eigen::MatrixXi &hinge_edges, Eigen::MatrixXi &cut_edges, Eigen::MatrixXi &boundary_edges, vector<bool> &bool_dual_edges,
         vector<meshEdge> &mesh_edges, Eigen::MatrixXd &MST_color, int current_mst_heuristic = 0);
/* Description ensues...
    mesh contains the original mesh. Useful information in there is V.rows() and F.rows()

    takes in the dual mesh for vertex information that might be used.. remember the order placed in the mesh --> V --> Face centers --> Edge centers
    sheet mesh --> The output flat configuration of the dual_mesh..

    dual-edges are the dual reprsentation of the edges storing the edge to Face map as a sparse matrix reprsentation. Uses the indexing based on the above Vertex ordering.
    So to obtain the absolute index subtract the V.rows(). where V is the number of vertices in the original mesh

    To get this first compute the MST on the mesh. All the edegs that form the part of MST are the edges for which we would need hinges. Store such edges also in the MST_edges matrix

    The remaining edges that we are left with are all the cut_edges and should be stored in its corresponding edge matrix. Ideally we would like to have this cut-edegs where the curvature is high so as to prevent the problem of hinges.

    color scheme for MST edges.. red if part of MST else Grey. Red reprsents unfolding, grey reprsents cut-edges

*/

void setMSTMesh(const igl::opengl::ViewerData &mesh, const Eigen::MatrixXi &edges, Eigen::MatrixXd MST_edge_color, igl::opengl::ViewerData &mst_mesh);
// set the mesh color of edges based on cut edge vs hinge edge

std::vector<double> minPerimeterWeights(const igl::opengl::ViewerData &mesh, const Eigen::MatrixXi &dual_edges);
// set the triangles based on minimum perimeter heuristic

std::vector<double> maxDihedralAngleWeights(const igl::opengl::ViewerData &mesh, const Eigen::MatrixXi &dual_edges);
// set the triangles based on maximumDihedral angle heuristic


//------------------------------------------------------------------------------
//	Functions in unfold.cpp
//------------------------------------------------------------------------------
void unfoldMesh(igl::opengl::ViewerData &mesh, igl::opengl::ViewerData &dual_mesh, const Eigen::MatrixXi &dual_edges,
                const Eigen::MatrixXi &hinge_edges, vector<meshEdge> &mesh_edges, igl::opengl::ViewerData &sheet_mesh, igl::opengl::ViewerData &sheet_clean_mesh,
                std::vector<std::vector<int>> &sheet_qindices_hinge, vector<vector<int>> &flatFaceVisitOrder,
                map<int, triplet> &meshFace2Edge, bool bool_no_cut, bool merge_patches, int bp = -1,
                bool with_offset = true, double obj_scale = 1.0, double drill_bit_radius = 0.0, 
                double user_margin = 0.0, int min_patches = 1, bool adaptiveScaling = false, 
                bool useHalfHinges = false);

void fixMeshOrder(igl::opengl::ViewerData &mesh, vector<Eigen::Vector3d> &sheet_vertices, vector<Eigen::Vector3i> &sheet_indices, 
                    map<int, int> &world2flatmap, vector<vector<int>> &flatFaceVisitOrder);

void flattenTriangle(Eigen::Vector3d vpos0, Eigen::Vector3d vpos1, Eigen::Vector3d vpos2, 
                    Eigen::Vector3d &vpos0_ref, Eigen::Vector3d &vpos1_ref, Eigen::Vector3d &vpos2_ref, double hoffset = 0, double voffset = 0);            

void stitchMST(vector<int> visit_order, igl::opengl::ViewerData &mesh, Eigen::MatrixXi mesh_FV, 
                vector<Eigen::Vector3d> &sheet_vertices, vector<Eigen::Vector3i> &sheet_indices);

void unfoldViaMST(const vector<int> &visit_order, igl::opengl::ViewerData &mesh, Eigen::MatrixXi &hinge_edges, 
                  const Eigen::MatrixXi &dual_edges, vector<meshEdge> &mesh_edges_data, 
                  map<int, triplet> &meshFace2Edge, vector<Eigen::Vector3d> &sheet_vertices, 
                  vector<Eigen::Vector3i> &sheet_indices, std::vector<std::vector<int>> &sheet_qindices_hinge, 
                  igl::opengl::ViewerData &sheet_clean_mesh, std::map<int, vector<int>> face2dualEdge,
                  vector<vector<int>> &flatFaceVisitOrder_return, bool bool_no_cut, bool merge_patches, 
                  int bp, bool with_offset = true, double obj_scale = 1.0, double drill_bit_radius = 0.0, 
                  double user_margin = 0.0, int min_patches = 1, bool adaptiveScaling = false, 
                  bool useHalfHinges = false);

void findAllIntersectingFaces(const vector<Eigen::Vector3d> &sheet_vertices, const vector<Eigen::Vector3i> &sheet_indices, 
                                vector<pair<int, int>> &intersectingFaceList);

void findAllPaths(const Eigen::MatrixXi &hinge_edges, const vector<pair<int, int>> &intersectingFaceList, vector<vector<int>> &paths);

bool pathSearchDFS(vector<int> &face_stack, vector<bool> &visited, std::map<int, vector<int>> &face2dualEdge, const Eigen::MatrixXi &hinge_edges, int f1);

void separatePatchDfs(vector<int> &face_stack, vector<bool> &visited, std::map<int, vector<int>> &face2dualEdge, 
        const Eigen::MatrixXi &hinge_edges, int f1, int prev_patch, int new_patch, vector<int> &face2Patch);

void updateFace2Patch(const vector<vector<int>> &flatFaceVisitOrder, vector<int> &face2Patch);

void getFlatHinges(Eigen::MatrixXi &flat_hinge_edges, const Eigen::MatrixXi &hinge_edges);

void minCutEdgeAddition(const Eigen::MatrixXi &hinge_edges, const vector<vector<int>> &paths, Eigen::MatrixXi &new_cut_edges);

int findBestCutEdge(const Eigen::MatrixXi &hinge_edges, map<pair<int, int>, int> edgeToIndex, const vector<vector<int>> &paths);

void applyCutEdges(igl::opengl::ViewerData &mesh, std::vector<Eigen::Vector3d> &sheet_vertices,
                std::vector<Eigen::Vector3i> &sheet_indices, Eigen::MatrixXi &hinge_edges, vector<int> &face2Patch, 
                map<int, vector<int>> &world2flatVind, vector<vector<int>> &flatFaceVisitOrder, 
                vector<meshEdge> &mesh_edges_data, map<int, triplet> &meshFace2Edge);

// void createPriorityQueue(priority_queue<pair<int,int>, vector<pair<int,int>>, queueComp2> &patchSize2PatchIndex, const vector<vector<int>> &flatFaceVisitOrder);
void createPriorityQueue(priority_queue<pair<int,int>, vector<pair<int,int>>, queueComp3> &patchSize2PatchIndex, const vector<vector<int>> &flatFaceVisitOrder);

void mergePatches(igl::opengl::ViewerData &mesh, std::vector<Eigen::Vector3d> &sheet_vertices,
                  std::vector<Eigen::Vector3i> &sheet_indices, Eigen::MatrixXi &hinge_edges, const Eigen::MatrixXi &dual_edges, vector<meshEdge> &mesh_edges_data, 
                  map<int, triplet> &meshFace2Edge, vector<int> &face2Patch, map<int, vector<int>> &world2flatVind,
                  vector<vector<int>> &flatFaceVisitOrder, double obj_scale = 1.0, double drill_bit_radius = 0.0, int min_patches = 1);

bool canIMergePatches( igl::opengl::ViewerData &mesh, const Eigen::MatrixXi &dual_edges, vector<Eigen::Vector3d> &sheet_vertices, 
                       vector<Eigen::Vector3i> &sheet_indices, vector<int> patch_0, vector<int> patch_1, map<int, vector<int>> &world2flatVind, map<int, int> &flat2worldVind, 
                       vector<int> face_edges, int face_0, int face_1);

void getMergeIndices(std::vector<Eigen::Vector3i> & sheet_indices, map<int, int> &flat2WorldVind, int face_0, int face_1, int &pfsid2 , int &cfsid2 , int &shared_v0 , int &shared_v1 );

void findPreviousFacePatch(int &prev_face_id, int &dual_edge, vector<int> f2dualMap, const Eigen::MatrixXi &hinge_edges, int face_id, const vector<bool> &isVisited);

void merge2Patches(vector<vector<int>> &flatFaceVisitOrder, int flat_curr_patch_index, int next_flat_curr_patch_index, int flat_face_curr, int flat_face_next, const Eigen::MatrixXi &flat_hinge_edges);

void applyOffset(igl::opengl::ViewerData &mesh, std::vector<Eigen::Vector3d> &sheet_vertices, 
                 const std::vector<Eigen::Vector3i> &sheet_indices, const vector<vector<int>> &visitOrder, 
                 const Eigen::MatrixXi &flat_hinge_edges, std::vector<Eigen::Vector3d> &sheet_vertices_offset, 
                 std::vector<Eigen::Vector3i> &sheet_indices_offset, std::vector<Eigen::Vector3d> &sheet_vertices_hinge, 
                 std::vector<Eigen::Vector3i> &sheet_indices_hinge, std::vector<std::vector<int>> &sheet_qindices_hinge,
                 vector<meshEdge> &mesh_edges_data, map<int, triplet> &meshFace2Edge, double drill_bit_radius = 0.0, 
                 double user_margin = 0.0, bool adaptiveScaling = false, bool useHalfHinges = false);

void applyOffsetUpdated(igl::opengl::ViewerData &mesh, const std::vector<Eigen::Vector3d> &sheet_vertices, 
                 const std::vector<Eigen::Vector3i> &sheet_indices, const vector<int> &visitOrder, 
                 const Eigen::MatrixXi &flat_hinge_edges, std::vector<Eigen::Vector3d> &sheet_vertices_offset, 
                 std::vector<Eigen::Vector3i> &sheet_indices_offset, std::vector<Eigen::Vector3d> &sheet_vertices_hinge, 
                 std::vector<Eigen::Vector3i> &sheet_indices_hinge, std::vector<std::vector<int>> &sheet_qindices_hinge,
                 vector<meshEdge> &mesh_edges_data, map<int, triplet> &meshFace2Edge, double drill_bit_radius = 0.0, double user_margin = 0.0);

void applyOffsetStep1(const vector<int> &visitOrder, const std::vector<Eigen::Vector3d> &sheet_vertices, 
                    const std::vector<Eigen::Vector3i> &sheet_indices, const Eigen::MatrixXi &flat_hinge_edges, 
                    std::map<int, vector<int>> &face2dualEdge, vector<meshEdge> &mesh_edges_data, 
                    map<int, triplet> &meshFace2Edge, std::vector<Eigen::Vector3d> &sheet_vertices_offset, 
                    std::vector<Eigen::Vector3i> &sheet_indices_offset, map<int, shared_ptr<transNode>> &triangleNodes,
                    float drill_bit_radius);

void applyOffsetStep2(igl::opengl::ViewerData &mesh, const vector<int> &visitOrder,
        const std::vector<Eigen::Vector3i> &sheet_indices, const Eigen::MatrixXi &flat_hinge_edges, 
        std::map<int, vector<int>> &face2dualEdge, std::vector<Eigen::Vector3d> &sheet_vertices_offset, 
        std::vector<Eigen::Vector3i> &sheet_indices_offset, map<int, shared_ptr<transNode>> &triangleNodes, 
        vector<meshEdge> &mesh_edges_data, map<int, triplet> &meshFace2Edge);

void applyOffsetStep3(const vector<int> &visitOrder, 
        const std::vector<Eigen::Vector3d> &sheet_vertices, const std::vector<Eigen::Vector3i> &sheet_indices,
        const Eigen::MatrixXi &flat_hinge_edges, std::map<int, vector<int>> &face2dualEdge, 
        std::vector<Eigen::Vector3d> &sheet_vertices_offset, std::vector<Eigen::Vector3i> &sheet_indices_offset, 
        std::vector<Eigen::Vector3d> &sheet_vertices_hinge, std::vector<Eigen::Vector3i> &sheet_indices_hinge, 
        std::vector<std::vector<int>> &sheet_qindices_hinge, vector<meshEdge> &mesh_edges_data, 
        map<int, triplet> &meshFace2Edge);

void trimPerimeterEdges(const vector<int> &visitOrder, std::vector<Eigen::Vector3d> &sheet_vertices_offset, 
            std::vector<Eigen::Vector3i> &sheet_indices_offset, vector<meshEdge> &mesh_edges_data, 
            map<int, triplet> &meshFace2Edge, float user_margin );        

void correctOrientation(igl::opengl::ViewerData &mesh, std::vector<Eigen::Vector3d> &sheet_vertices, std::vector<Eigen::Vector3i> &sheet_indices);

//------------------------------------------------------------------------------
//	Functions in triangleHelper.cpp
//------------------------------------------------------------------------------

double getSafeBendingAngle(double length); // returns the angle value in degrees

double getSafeHingeLength(double angle); // returns the minimum hinge length that is needed to support the bending angle

void orderVertices(int &v0, int &v1);

int findCommonFaceEdge(triplet e0s, triplet e1s);
int getIndexinTriplet(triplet tri, int el);

void chooseCorrectOrder(int &nv0, int &nv1, int &nv2, vector<Eigen::Vector3d> &sheet_vertices, int v0, int v1, int v2, double l0, double l1, double l2);

double Det2D(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3);

void CheckTriWinding(Eigen::Vector3d &p1, Eigen::Vector3d &p2, Eigen::Vector3d &p3, bool allowReversed);

bool BoundaryCollideChk(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, double eps);

bool BoundaryDoesntCollideChk(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, double eps);

bool TriTri2D(std::vector<Eigen::Vector3d> t1, std::vector<Eigen::Vector3d> t2, double eps = 0, bool allowReversed = true, bool onBoundary = false);

bool checkTriangleTriangleIntersection(vector<Eigen::Vector3d> triangle, vector<Eigen::Vector3d>vertices, vector<Eigen::Vector3i>faces, int &intersecting_face);

bool checkTriangleTriangleIntersection(vector<Eigen::Vector3d>vertices, vector<Eigen::Vector3i>faces);

int countIntersectingTriangles(vector<Eigen::Vector3d> triangle, vector<Eigen::Vector3d>vertices, vector<Eigen::Vector3i>faces);

int countTotalIntersectingTriangles(vector<Eigen::Vector3d>vertices, vector<Eigen::Vector3i>faces, vector<int> face2patch);

bool checkTriangleTriangleIntersection2(vector<Eigen::Vector3d> triangle, vector<Eigen::Vector3d>vertices, vector<Eigen::Vector3i>faces);

bool checkTriangleTriangleIntersection2(vector<Eigen::Vector3d> T1, vector<Eigen::Vector3d> T2);

bool checkTriangleTriangleIntersectionWithClipper(vector<Eigen::Vector3d> triangle, vector<Eigen::Vector3d>vertices, vector<Eigen::Vector3i>faces);

bool edgeEdgeIntersection(Eigen::Vector3d p1, Eigen::Vector3d q1, Eigen::Vector3d p2, Eigen::Vector3d q2);

int orientation(Eigen::Vector3d p, Eigen::Vector3d q, Eigen::Vector3d r);

bool onSegment(Eigen::Vector3d p, Eigen::Vector3d q, Eigen::Vector3d r);

void findCommonEdge(const Eigen::MatrixXi &hinge_edges, int dual_edge, const Eigen::MatrixXi &mesh_FE, int &shared_edge);

bool isPositiveOrientation(Eigen::Vector3d v0, Eigen::Vector3d v1, Eigen::Vector3d v2);

bool checkVertexPolygonIntersection(Eigen::Vector3d pos, vector<Eigen::Vector3d> vertices, vector<Eigen::Vector3i> faces);

double sign (Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3);

bool pointOnTriangle (Eigen::Vector3d pt, Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3);

bool pointInTriangle (Eigen::Vector3d pt, Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3);

double area(double x1, double y1, double x2, double y2, double x3, double y3);

void printvec(Eigen::Vector3d vec, const char* msg);

void printvec(Eigen::VectorXd vec, const char* msg);

void printvec(Eigen::Vector3i vec, const char* msg);

double computeDihedralAngle(const Eigen::MatrixXd &mesh_V, const Eigen::MatrixXi &mesh_FV , int face_id, int prev_face_id);

int getIndex(vector<int> v, int K);

void intersect2(vector<int> A, vector<int> B, int &shared_v0, int &shared_v1);
int intersect(Eigen::Vector3i A, vector<int> B);
int intersect(vector<int> A, vector<int> B);

int subtract(Eigen::Vector3i A, vector<int> B);
int subtract(vector<int> A, vector<int> B);

void emplaceIndex(map<int, vector<int>> &world2flatVind, int key, int old_index, int new_index);

void removeEntry(map<int, vector<int>> &world2flatVind, int key, int old_index);

void replace(Eigen::Vector3i &vec, int i0, int i1);

void remove(vector<int> &vec, int val);

void removeRow(Eigen::MatrixXi& matrix, int rowToRemove);

void removeColumn(Eigen::MatrixXi& matrix, int colToRemove);

void getIndices(int &sid0, int &sid1, int &sid2, int shared_v0, int shared_v1, int v0, int v1, int v2);

int getIndices2(int &sid0, int &sid1, int &sid2, int shared_v0, int shared_v1, int v0, int v1, int v2);

void createFace2dualEdgeMap(std::map<int, vector<int>> &Face2dualEdge, Eigen::MatrixXi hinge_edges);

void createFace2occuranceMap(std::map<int, int> &face2occurannce, Eigen::MatrixXi hinge_edges);

void findCommonVertex(int &shared_v0, int &shared_v1, Eigen::Vector3i f0, Eigen::Vector3i f1);

void mirrorVertex(vector<Eigen::Vector3d> &vec, int sid0, int sid1, int sid2);

double computeOffset(double angle, double scale = 1.0, double drill_bit_radius= 0.0, bool isHalfHinge = false);

double compute_hinge_width(double scale = 1.0);

void getEndIndices(Eigen::Vector3i p0, Eigen::Vector3i p1, int &id0, int &id1);

void performTriangleOffset(Eigen::Vector3d vpos0, Eigen::Vector3d &vpos1, Eigen::Vector3d &vpos2, double offset);

void uniformOffseting(Eigen::Vector3d vpos0, Eigen::Vector3d &vpos1, Eigen::Vector3d &vpos2, double offset);

void findOppositeVertices(const vector<Eigen::Vector3i> &sheet_indices, const vector<int> &fvertices_visited, int prev_flatv0, int prev_flatv1, int &new_flatv2, int &prev_flatv2);

void connectTriangle2Previous(const vector<Eigen::Vector3d> &sheet_vertices, int prev_flatv0, int prev_flatv1, int sid0, int sid1, int sid2, vector<Eigen::Vector3d> &vpos_refs);

void findSharedIndices(vector<int> f0vids, vector<int> f1vids, int &csid0, int &csid1, int &csid2, int &psid0, int &psid1, int &psid2);

Eigen::Vector3d getDistanceBwLines(const vector<Eigen::Vector3d> & sheet_vertices_offset, int cv0, int cv1, int pv0, int pv1);

shared_ptr<transNode> findCorrectDistanceBetweenTriangle(const std::vector<Eigen::Vector3i> &sheet_indices, 
            const std::vector<Eigen::Vector3i> &sheet_indices_offset,
            std::vector<Eigen::Vector3d> &sheet_vertices_offset, int curr_face_id, 
            int prev_face_id, double drill_bit_radius = 0.0, bool isHalfHinge = false);

bool checkHit(Eigen::Vector3d v0, Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d &h0, Eigen::Vector3d &h1);

double getDistance(Eigen::Vector3d v0, Eigen::Vector3d v1);

string dtos(double x); 

double perpendicularLength(Eigen::Vector3d v0, Eigen::Vector3d v1, Eigen::Vector3d v2);

void EnergyGradient(double l0, Eigen::Vector3d v0, Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d &de0dv0, Eigen::Vector3d &de0dv1, Eigen::Vector3d &de0dv2);

void addHinge(const std::vector<Eigen::Vector3i> sheet_indices, const std::vector<Eigen::Vector3i> sheet_indices_offset, const std::vector<Eigen::Vector3d> sheet_vertices_offset, int curr_face_id, int prev_face_id, 
            Eigen::Vector3d &shared_hpos0, Eigen::Vector3d &shared_hpos1, Eigen::Vector3d &shared_hpos2, Eigen::Vector3d &shared_hpos3);


