//2D Triangle-Triangle collisions in C++
//Release by Tim Sheerman-Chase 2016 under CC0
// https://gist.github.com/TimSC/5ba18ae21c4459275f90

// Modified by Manas Bhargava 17/12/2021
#include "common.h"

double Det2D(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3) 
{
	return p1[(0)]*(p2(1)-p3(1))
		+p2[(0)]*(p3(1)-p1(1))
		+p3[(0)]*(p1(1)-p2(1));
}

void CheckTriWinding(Eigen::Vector3d &p1, Eigen::Vector3d &p2, Eigen::Vector3d &p3, bool allowReversed)
{
	double detTri = Det2D(p1, p2, p3);
	if(detTri < 0.0)
	{   
        // printf("Triangle has wrong winding direction\n");
		if (allowReversed)
		{
			Eigen::Vector3d a = p3;
			p3 = p2;
			p2 = a;
		}
		// else throw std::runtime_error("triangle has wrong winding direction");
	}
}

bool BoundaryCollideChk(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, double eps)
{
	return Det2D(p1, p2, p3) < eps;
}

bool BoundaryDoesntCollideChk(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, double eps)
{
	return Det2D(p1, p2, p3) <= eps;
}

bool TriTri2D(std::vector<Eigen::Vector3d> t1, std::vector<Eigen::Vector3d> t2, double eps, bool allowReversed, bool onBoundary)
{
	//triangles must be expressed anti-clockwise
	CheckTriWinding(t1[0], t1[1], t1[2], allowReversed);
	CheckTriWinding(t2[0], t2[1], t2[2], allowReversed);

	bool (*checkEdge)(const Eigen::Vector3d &, const Eigen::Vector3d &, const Eigen::Vector3d &, double) = NULL;
	if(onBoundary) //Points on the boundary are considered as colliding
		checkEdge = BoundaryCollideChk;
	else //Points on the boundary are not considered as colliding
		checkEdge = BoundaryDoesntCollideChk;

	//For edge E of triangle 1,
	for(int i=0; i<3; i++)
	{
		int j=(i+1)%3;
        // printf("i : %d j : %d ",i,j);
		//Check all points of triangle 2 lay on the external side of the edge E. If
		//they do, the triangles do not collide.
		if (checkEdge(t1[i], t1[j], t2[0], eps) &&
			checkEdge(t1[i], t1[j], t2[1], eps) &&
			checkEdge(t1[i], t1[j], t2[2], eps)) {
                // printf("confirms it is outside 1\n");
                return false;
            }
        // printf("fails to check 1\n");
	}

	//For edge E of triangle 2,
	for(int i=0; i<3; i++)
	{
		int j=(i+1)%3;
        // printf("i : %d j : %d ",i,j);
		//Check all points of triangle 1 lay on the external side of the edge E. If
		//they do, the triangles do not collide.
		if (checkEdge(t2[i], t2[j], t1[0], eps) &&
			checkEdge(t2[i], t2[j], t1[1], eps) &&
			checkEdge(t2[i], t2[j], t1[2], eps)) {
                // printf("confirms it is outside 2\n");
                return false;
            }
        // printf("fails to check 2\n");
	}

	//The triangles collide
	return true;
}

bool checkTriangleTriangleIntersectionWithClipper(vector<Eigen::Vector3d> triangle, vector<Eigen::Vector3d>vertices, vector<Eigen::Vector3i>faces) {
    bool isInside = false;
    // TODO implement it using clipper
    return isInside;
}

bool checkTriangleTriangleIntersection2(vector<Eigen::Vector3d> T1, vector<Eigen::Vector3d> T2) {

    // point on triangle check

    bool pcheck0 = pointOnTriangle(T1[0], T2[0], T2[1], T2[2]);
    bool pcheck1 = pointOnTriangle(T1[1], T2[0], T2[1], T2[2]);
    bool pcheck2 = pointOnTriangle(T1[2], T2[0], T2[1], T2[2]);    

    if(pcheck0 and pcheck1 and pcheck2) {
        // printf("All 3 points lie on the triangle thus confirming the triangles are exactly overlapping\n");
        return true;
    }

    bool check0 = pointInTriangle(T1[0], T2[0], T2[1], T2[2]);
    bool check1 = pointInTriangle(T1[1], T2[0], T2[1], T2[2]);
    bool check2 = pointInTriangle(T1[2], T2[0], T2[1], T2[2]);

    bool check3 = pointInTriangle(T2[0], T1[0], T1[1], T1[2]);
    bool check4 = pointInTriangle(T2[1], T1[0], T1[1], T1[2]);
    bool check5 = pointInTriangle(T2[2], T1[0], T1[1], T1[2]);
    // printf("check values : %d %d %d %d %d %d\n", check0, check1, check2, check3, check4, check5);


    bool isInside = check0 or check1 or check2 or check3 or check4 or check5;

    if (isInside)
        return true;

    // Edge check to make sure that there edges also do not collide
    bool echeck0 = edgeEdgeIntersection(T1[0], T1[1], T2[0], T2[1]);
    bool echeck1 = edgeEdgeIntersection(T1[0], T1[1], T2[1], T2[2]);
    bool echeck2 = edgeEdgeIntersection(T1[0], T1[1], T2[2], T2[0]);

    bool echeck3 = edgeEdgeIntersection(T1[1], T1[2], T2[0], T2[1]);
    bool echeck4 = edgeEdgeIntersection(T1[1], T1[2], T2[1], T2[2]);
    bool echeck5 = edgeEdgeIntersection(T1[1], T1[2], T2[2], T2[0]);

    bool echeck6 = edgeEdgeIntersection(T1[2], T1[0], T2[0], T2[1]);
    bool echeck7 = edgeEdgeIntersection(T1[2], T1[0], T2[1], T2[2]);
    bool echeck8 = edgeEdgeIntersection(T1[2], T1[0], T2[2], T2[0]);
    // printf("Edge check values : %d %d %d %d %d %d %d %d %d\n", echeck0, echeck1, echeck2, echeck3, echeck4, echeck5, echeck6, echeck7, echeck8);
    isInside = isInside or echeck0 or echeck1 or echeck2 or echeck3 or echeck4 or echeck5 or echeck6 or echeck7 or echeck8;
    if (isInside)
        return true;
    else    
        return false;
    return false;
}

int countIntersectingTriangles(vector<Eigen::Vector3d> triangle, vector<Eigen::Vector3d>vertices, vector<Eigen::Vector3i>faces) {

    int intersectingTrianglesCount = 0;
    std::vector<Eigen::Vector3d> T2;
    T2.push_back(triangle[0]); T2.push_back(triangle[1]); T2.push_back(triangle[2]);

    int v0,v1,v2;
    Eigen::Vector3d vpos0, vpos1, vpos2;
    // printf("Checking for all 6 point in triangle existence\n");
    for (size_t i = 0; i < faces.size(); i++)
    {
        std::vector<Eigen::Vector3d> T1;
        v0 = faces[i](0); v1 = faces[i](1); v2 = faces[i](2);
        vpos0 = vertices[v0];
        vpos1 = vertices[v1];
        vpos2 = vertices[v2];

        T1.push_back(vpos0); T1.push_back(vpos1); T1.push_back(vpos2);
        // printf("Checking for triangle with indices in ref plane %d %d %d\n", v0, v1, v2);
        if (checkTriangleTriangleIntersection2(T1, T2))
            intersectingTrianglesCount++;
    }

    return intersectingTrianglesCount;
}

int countTotalIntersectingTriangles(vector<Eigen::Vector3d>vertices, vector<Eigen::Vector3i>faces, vector<int> face2patch) {


    int intersectingTrianglesCount = 0;
    // printf("Checking for all 6 point in triangle existence\n");
    int v0,v1,v2;
    Eigen::Vector3d vpos0, vpos1, vpos2;
    for (size_t i = 0; i < faces.size(); i++) {
        std::vector<Eigen::Vector3d> T2;
        v0 = faces[i](0); v1 = faces[i](1); v2 = faces[i](2);
        vpos0 = vertices[v0];
        vpos1 = vertices[v1];
        vpos2 = vertices[v2];
        T2.push_back(vpos0); T2.push_back(vpos1); T2.push_back(vpos2);
        for (size_t j = i+1; j < faces.size(); j++) {
            if(face2patch[i] != face2patch[j])
                continue;
            std::vector<Eigen::Vector3d> T1;
            v0 = faces[j](0); v1 = faces[j](1); v2 = faces[j](2);
            vpos0 = vertices[v0];
            vpos1 = vertices[v1];
            vpos2 = vertices[v2];

            T1.push_back(vpos0); T1.push_back(vpos1); T1.push_back(vpos2);
            // printf("Checking for triangle with indices in ref plane %d %d %d\n", v0, v1, v2);
            if (checkTriangleTriangleIntersection2(T1, T2))  {
                // printf("Intersecting triangles in plane are : %ld %ld\n", i, j);
                intersectingTrianglesCount++;
            }
        }
        // printf("face count : %d total : %ld\n", i, faces.size());
    }

    return intersectingTrianglesCount;
}

bool checkTriangleTriangleIntersection2(vector<Eigen::Vector3d> triangle, vector<Eigen::Vector3d>vertices, vector<Eigen::Vector3i>faces) {
    bool isInside = false;
    std::vector<Eigen::Vector3d> T2;
    T2.push_back(triangle[0]); T2.push_back(triangle[1]); T2.push_back(triangle[2]);

    int v0,v1,v2;
    Eigen::Vector3d vpos0, vpos1, vpos2;
    // printf("Checking for all 6 point in triangle existence\n");
    for (size_t i = 0; i < faces.size(); i++)
    {
        std::vector<Eigen::Vector3d> T1;
        v0 = faces[i](0); v1 = faces[i](1); v2 = faces[i](2);
        vpos0 = vertices[v0];
        vpos1 = vertices[v1];
        vpos2 = vertices[v2];

        T1.push_back(vpos0); T1.push_back(vpos1); T1.push_back(vpos2);
        // printf("Checking for triangle with indices in ref plane %d %d %d\n", v0, v1, v2);
        isInside = checkTriangleTriangleIntersection2(T1, T2);
        if (isInside)
            return true;
    }

    return false;
}


// https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
int orientation(Eigen::Vector3d p, Eigen::Vector3d q, Eigen::Vector3d r) {
    double val = (q[1] - p[1]) * (r[0] - q[0]) -
              (q[0] - p[0]) * (r[1] - q[1]);
 
    if (val == 0) return 0;  // collinear
    return (val > 0)? 1: 2; // clock or counterclock wise

}

bool edgeEdgeIntersection(Eigen::Vector3d p1, Eigen::Vector3d q1, Eigen::Vector3d p2, Eigen::Vector3d q2) {
    // p1, q1 from same edge
    // p2, q2 from same edge

    double eps = 1e-6;
    // edges share a common end point no worries allowed 
    if((p1-p2).norm() < eps or (p1-q2).norm() < eps  or (q1-p2).norm() < eps or (q1-q2).norm() < eps){
        // printf("The edge shares a common end point ... so no intersection...\n");
        return false;
    } 

    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4) {
        // printvec(p1, "p1");
        // printvec(p2, "p2");
        // printvec(q1, "q1");
        // printvec(q2, "q2");
        // printf("%d %d %d %d\n", o1, o2, o3, o4);
        return true;
    }
 
    // Special Cases
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
    // printf("Either collinear or does not intersect\n");
    if (o1 == 0 && onSegment(p1, p2, q1)) return false; // collinear points are not allowed
 
    // p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return false;
 
    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return false;
 
     // p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return false;
 
    return false; // Doesn't fall in any of the above cases
}

bool onSegment(Eigen::Vector3d p, Eigen::Vector3d q, Eigen::Vector3d r) // if they are collinear points check if q lies on p and r
{
    if (q[0] <= max(p[0], r[0]) && q[0] >= min(p[0], r[0]) &&
        q[1] <= max(p[1], r[1]) && q[1] >= min(p[1], r[1]))
       return true;
 
    return false;
}

bool checkTriangleTriangleIntersection(vector<Eigen::Vector3d>vertices, vector<Eigen::Vector3i>faces) {
    bool isInside = false;
    // printf("Checking for all 6 point in triangle existence\n");
    int v0,v1,v2;
    Eigen::Vector3d vpos0, vpos1, vpos2;
    for (size_t i = 0; i < faces.size(); i++)
    {
        std::vector<Eigen::Vector3d> T2;
        v0 = faces[i](0); v1 = faces[i](1); v2 = faces[i](2);
        vpos0 = vertices[v0];
        vpos1 = vertices[v1];
        vpos2 = vertices[v2];
        T2.push_back(vpos0); T2.push_back(vpos1); T2.push_back(vpos2);
        for (size_t j = i+1; j < faces.size(); j++) {
            std::vector<Eigen::Vector3d> T1;
            v0 = faces[j](0); v1 = faces[j](1); v2 = faces[j](2);
            vpos0 = vertices[v0];
            vpos1 = vertices[v1];
            vpos2 = vertices[v2];

            T1.push_back(vpos0); T1.push_back(vpos1); T1.push_back(vpos2);
            // printf("Checking for triangle with indices in ref plane %d %d %d\n", v0, v1, v2);
            if (checkTriangleTriangleIntersection2(T1, T2))
                return true;
        }
    }
    return false;
}

int getIndexinTriplet(triplet tri, int el) {
    if(el == get<0>(tri))
        return 0;
    else if (el == get<1>(tri))
        return 1;
    else if (el == get<2>(tri))
        return 2;
    else {
        printf("Something went wrong in finding the correct INDEX DEBUG!!!\n");
        exit(1);
    }
}

bool checkTriangleTriangleIntersection(vector<Eigen::Vector3d> triangle, vector<Eigen::Vector3d>vertices, vector<Eigen::Vector3i>faces, int &intersecting_face) {
    std::vector<Eigen::Vector3d> T2;
    T2.push_back(triangle[0]); T2.push_back(triangle[1]); T2.push_back(triangle[2]);

    int v0,v1,v2;
    Eigen::Vector3d vpos0, vpos1, vpos2;
    for (int i = 0; i < faces.size(); i++) {
        v0 = faces[i](0); v1 = faces[i](1); v2 = faces[i](2);
        vpos0 = vertices[v0];
        vpos1 = vertices[v1];
        vpos2 = vertices[v2];
        std::vector<Eigen::Vector3d> T1;
        T1.push_back(vpos0); T1.push_back(vpos1); T1.push_back(vpos2);

        // printf("Checking for triangle with indices in ref plane %d %d %d\n", v0, v1, v2);
        if(checkTriangleTriangleIntersection2(T1, T2)) {
            intersecting_face = i;
            // printf("Triangles atlest intersects with face %d\n", i);
            return true;
        }        
    }
    return false;
}


bool checkVertexPolygonIntersection(Eigen::Vector3d pos, vector<Eigen::Vector3d> vertices, vector<Eigen::Vector3i> faces) {
    bool isInside = false;

    // printf("Checking vertex Polygon intersection\n");

    int v0,v1,v2;
    Eigen::Vector3d vpos0, vpos1, vpos2;
    for (size_t i = 0; i < faces.size(); i++)
    {
        v0 = faces[i](0); v1 = faces[i](1); v2 = faces[i](2);
        vpos0 = vertices[v0];
        vpos1 = vertices[v1];
        vpos2 = vertices[v2];


        if(pointInTriangle(pos, vpos0, vpos1,vpos2)) {
            // printf("Point atlest lies inside face %d %d %d So need to flip the last added vertex\n", v0, v1, v2);
            isInside = true;
            return isInside;
        }        
    }
    return isInside;
}

double area(double x1, double y1, double x2, double y2, double x3, double y3)
{
    return fabs((x1*(y2-y3) + x2*(y3-y1)+ x3*(y1-y2))/2.0);
}


double sign (Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3)
{
    https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
    return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1]);
}

bool pointOnTriangle (Eigen::Vector3d pt, Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3) {
   double eps = 1e-6;
   double x1 = v1(0), y1 = v1(1);
   double x2 = v2(0), y2 = v2(1);
   double x3 = v3(0), y3 = v3(1);
   double x = pt(0), y = pt(1);

   /* Calculate area of triangle ABC */ 
   double A = area (x1, y1, x2, y2, x3, y3);
  
   /* Calculate area of triangle PBC */ 
   double A1 = area (x, y, x2, y2, x3, y3);
  
   /* Calculate area of triangle PAC */ 
   double A2 = area (x1, y1, x, y, x3, y3);
  
   /* Calculate area of triangle PAB */  
   double A3 = area (x1, y1, x2, y2, x, y);

   if(fabs(A1) < eps or fabs(A2) < eps or fabs(A3) < eps) {
       return true;
   }
   return false;
}



bool pointInTriangle (Eigen::Vector3d pt, Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3)
{
    https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
//     double d1, d2, d3;
//     bool has_neg, has_pos;

//     d1 = sign(pt, v1, v2);
//     d2 = sign(pt, v2, v3);
//     d3 = sign(pt, v3, v1);

//     has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
//     has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
//     return !(has_neg && has_pos);

   /* Calculate area of triangle ABC */
   double eps = 1e-4;
   double x1 = v1(0), y1 = v1(1);
   double x2 = v2(0), y2 = v2(1);
   double x3 = v3(0), y3 = v3(1);
   double x = pt(0), y = pt(1);

   /* Calculate area of triangle ABC */ 
   double A = area (x1, y1, x2, y2, x3, y3);
  
   /* Calculate area of triangle PBC */ 
   double A1 = area (x, y, x2, y2, x3, y3);
  
   /* Calculate area of triangle PAC */ 
   double A2 = area (x1, y1, x, y, x3, y3);
  
   /* Calculate area of triangle PAB */  
   double A3 = area (x1, y1, x2, y2, x, y);

   if(fabs(A1) < eps or fabs(A2) < eps or fabs(A3) < eps) {
    //    printf("The point lies on the triangle %f %f %f %f\n", A, A1, A2, A3);
       return false;
   }
    
   /* Check if sum of A1, A2 and A3 is same as A */
   return (fabs(A - (A1 + A2 + A3)) < eps);

}

double computeDihedralAngle(const Eigen::MatrixXd &mesh_V, const Eigen::MatrixXi &mesh_FV , int face_id, int prev_face_id) {

    int f0v0,f0v1,f0v2;
    Eigen::Vector3d f0vpos0, f0vpos1, f0vpos2;

    // printf("Faces : %d %d\n", face_id, prev_face_id);

    f0v0 = mesh_FV.row(face_id)(0); f0vpos0 = mesh_V.row(f0v0);
    f0v1 = mesh_FV.row(face_id)(1); f0vpos1 = mesh_V.row(f0v1);
    f0v2 = mesh_FV.row(face_id)(2); f0vpos2 = mesh_V.row(f0v2);
    Eigen::Vector3d n0 = ((f0vpos1 - f0vpos0).cross(f0vpos2 - f0vpos1)).normalized();

    int f1v0,f1v1,f1v2;
    Eigen::Vector3d f1vpos0, f1vpos1, f1vpos2;
    f1v0 = mesh_FV.row(prev_face_id)(0); f1vpos0 = mesh_V.row(f1v0);
    f1v1 = mesh_FV.row(prev_face_id)(1); f1vpos1 = mesh_V.row(f1v1);
    f1v2 = mesh_FV.row(prev_face_id)(2); f1vpos2 = mesh_V.row(f1v2);
    Eigen::Vector3d n1 = ((f1vpos1 - f1vpos0).cross(f1vpos2 - f1vpos1)).normalized();


    // printf("Fcurr : %d : %d %d %d fprev : %d : %d %d %d\n", face_id,  f0v0,f0v1,f0v2, prev_face_id, f1v0,f1v1,f1v2);
    // printvec(f0vpos0, "f0vpos0");
    // printvec(f0vpos1, "f0vpos1");
    // printvec(f0vpos2, "f0vpos2");

    // printvec(f1vpos0, "f1vpos0");
    // printvec(f1vpos1, "f1vpos1");
    // printvec(f1vpos2, "f1vpos2");

    // printvec(n0, "n0");
    // printvec(n1, "n1");
    // printf("Cos theta : %f\n", n0.dot(n1));

    return acos(n0.dot(n1));
}




void findCommonEdge(const Eigen::MatrixXi &hinge_edges, int dual_edge, const Eigen::MatrixXi &mesh_FE, int &shared_edge) {
    int e0f0, e1f0, e2f0, e0f1, e1f1, e2f1;
    int f0 = hinge_edges.row(dual_edge)(0);
    int f1 = hinge_edges.row(dual_edge)(1);

    e0f0 = mesh_FE.row(f0)(0);
    e1f0 = mesh_FE.row(f0)(1);
    e2f0 = mesh_FE.row(f0)(2);

    e0f1 = mesh_FE.row(f1)(0);
    e1f1 = mesh_FE.row(f1)(1);
    e2f1 = mesh_FE.row(f1)(2);    

    // printf("Common edge: %d %d %d %d %d %d\n", e0f0,e1f0,e2f0,e0f1,e1f1,e2f1);

    vector<int> array =  {e0f0,e1f0,e2f0,e0f1,e1f1,e2f1};
    for (size_t i = 0; i < array.size(); i++) {
        for (size_t j = i+1; j < array.size(); j++) {
            if(array[i] == array[j]) {
                shared_edge = array[i];
                return; // have set the edge and can now return in peace
            }
            
        }
    }
    printf("Something went wrong cannot find the common edge. DEBUG\n");
    exit(1);
}

bool isPositiveOrientation(Eigen::Vector3d v0, Eigen::Vector3d v1, Eigen::Vector3d v2) {
    Eigen::Vector3d kcap = Eigen::Vector3d(0,0,1);
    Eigen::Vector3d cp = (v0 - v1).cross((v1 - v2));
    if(kcap.dot((v1-v0).cross(v2-v0)) > 0)
        return true;
    else {
        // printf("CP : %f %f %f\n", cp[0], cp[1], cp[2]);
        return false;
    }
} 

void flattenTriangle(Eigen::Vector3d vpos0, Eigen::Vector3d vpos1, Eigen::Vector3d vpos2, 
                    Eigen::Vector3d &vpos0_ref, Eigen::Vector3d &vpos1_ref, Eigen::Vector3d &vpos2_ref, double hoffset, double voffset) {

    vpos0_ref = Eigen::Vector3d(hoffset,voffset,0);
    vpos1_ref = Eigen::Vector3d(hoffset + (vpos1-vpos0).norm(),voffset,0); // lies on the xaxis 
    double l2_ref = (vpos2-vpos0).norm();
    double cosTheta = (vpos1-vpos0).dot((vpos2-vpos0)) / ((vpos1-vpos0).norm()*(vpos2-vpos0).norm());
    double sinTheta = ((vpos1-vpos0).cross((vpos2-vpos0))).norm() / ((vpos1-vpos0).norm()*(vpos2-vpos0).norm());
    vpos2_ref = Eigen::Vector3d(hoffset + l2_ref*cosTheta,voffset + l2_ref*sinTheta,0); // lies on the xaxis   
}

void printvec(Eigen::VectorXd vec, const char* msg) {
    cout << msg << " : " << endl;
    for(int i =0; i< vec.size(); i++) {
        printf("%f\n", vec(i));
    }
}

void printvec(Eigen::Vector3d vec, const char* msg) {
    cout << msg;
    printf(" : %f %f %f\n", vec(0), vec(1), vec(2));
}

void printvec(std::vector<int> vec, const char* msg) {
    cout << msg;
    for(auto v: vec)
        printf(" %d", v);
    cout << endl;
}


int getIndex(vector<int> v, int K)
{
    auto it = find(v.begin(), v.end(), K);
    if (it != v.end())
    {
        int index = it - v.begin();
        return index;
    }
    else {
        return -1;
    }
}
void intersect2(vector<int> A, vector<int> B, int &shared_v0, int &shared_v1) {
    // expects the output to be two values
    bool found0 = false, found1 = false;
    for(auto va : A){
        for(auto vb : B){
            if(va == vb) {
                if(not found0) {
                    found0 = true;
                    shared_v0 = va;
                } 
                else if(not found1) {
                    found1 = true;
                    shared_v1 = va;
                    break;
                }
            }
        }
        if(found1) 
            break;
    }

    if(shared_v0 == -1 or shared_v1 == -1) {
        printf("Something went wrong in intersect2 function DEBUG\n");
        exit(1);
    }

}

int intersect(vector<int> A, vector<int> B) {

    sort(A.begin(), A.end());
    sort(B.begin(), B.end());
    vector<int> C;
    set_intersection(A.begin(),A.end(),
                          B.begin(),B.end(),
                          back_inserter(C));

    if(C.size() != 1) {

        printf("Something went wrong in this function there should not be more than one intersected vertex DEBUG %ld!!!\n", C.size());

        printf("Entries in A are : ");
        for (size_t i = 0; i < A.size(); i++)
            printf("%d ", A[i]);
        cout << endl; 

        printf("Entries in B are : ");
        for (size_t i = 0; i < B.size(); i++)
            printf("%d ", B[i]);
        cout << endl; 

        printf("Entries in C are : ");
        for (size_t i = 0; i < C.size(); i++)
            printf("%d ", C[i]);
        cout << endl; 

    }
    return C[0];

}

int intersect(Eigen::Vector3i A, vector<int> B) {
    int v0 = A(0);
    int v1 = A(1);
    int v2 = A(2);

    vector<int> Avec{v0,v1,v2};
    return intersect(Avec, B);
}


int subtract(Eigen::Vector3i A, vector<int> B) {
    int v0 = A(0);
    int v1 = A(1);
    int v2 = A(2);

    vector<int> Avec{v0,v1,v2};
    return subtract(Avec, B);
}

int subtract(vector<int> A, vector<int> B) {
    // provide lonely vertex A- B
    vector<int> C;


    sort(A.begin(), A.end());
    sort(B.begin(), B.end());

    set_difference(A.begin(),A.end(),
                          B.begin(),B.end(),
                          back_inserter(C));

    if(C.size() != 1) {
        printf("Something went wrong in this function there should not be more than one subtracted vertex DEBUG!!!\n");
        printf("Entries in C are : ");
        for (size_t i = 0; i < C.size(); i++)
            printf("%d ", C[i]);
        cout << endl;
    }
    return C[0];
}

void emplaceIndex(map<int, vector<int>> &world2flatVind, int key, int old_index, int new_index) {
    for(int i =0; i< world2flatVind[key].size(); i++) {
        if(world2flatVind[key][i] == old_index) {
            world2flatVind[key].push_back(new_index);
            return;
        }
    }
    printf("Cannot find the right index to replace in world2flatVind DEBUG!!!!\n");
}

void removeEntry(map<int, vector<int>> &world2flatVind, int key, int old_index) {
    for(int i =0; i< world2flatVind[key].size(); i++) {
        world2flatVind[key].erase(std::remove(world2flatVind[key].begin(), world2flatVind[key].end(), old_index), world2flatVind[key].end());
        return;
    }
    printf("Cannot find the right entry to remove in world2flatVind DEBUG!!!!\n");
}

void replace(Eigen::Vector3i &vec, int i0, int i1) {
    for(int i =0; i< vec.size(); i++) {
        if(vec(i) == i0) {
            vec(i) = i1;
            return;
        }
    }
}

void removeRow(Eigen::MatrixXi& matrix, int rowToRemove)
{
    int numRows = matrix.rows()-1;
    int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}


void removeColumn(Eigen::MatrixXi& matrix, int colToRemove)
{
    int numRows = matrix.rows();
    int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}

void getIndices(int &sid0, int &sid1, int &sid2, int shared_v0, int shared_v1, int v0, int v1, int v2) {

    if(shared_v0 == v0) 
        sid0 = 0;
    else if(shared_v0 == v1)
        sid0 = 1;
    else if(shared_v0 == v2) 
        sid0 = 2;
    else {
        printf("Indices 1 Somethign went wrong...\n");
        printf("%d %d %d %d %d \n", v0, v1, v2, shared_v0, shared_v1);
        exit(1);
    }    
    if(shared_v1 == v0) 
        sid1 = 0;
    else if(shared_v1 == v1)
        sid1 = 1;
    else if(shared_v1 == v2) 
        sid1 = 2;
    else {
        printf("Indices 2 Somethign went wrong...\n");
        printf("%d %d %d %d %d \n", v0, v1, v2, shared_v0, shared_v1);
        exit(1);
    }       
    assert(sid0 != sid1); 
    if((sid0 + sid1 == 1)) // i.e. they are either 0 or 1
        sid2 = 2;
    else if(sid0 + sid1 == 2) // i.e. they are either 0 or 2
        sid2 = 1;
    else if(sid0 + sid1 == 3) // i.e. they are either 1 or 2
        sid2 = 0;
    else {
        printf("Indices 3 Somethign went wrong with sids...\n");
        printf("%d %d %d \n", sid0, sid1, sid2);
        exit(1);
    }
}

int getIndices2(int &sid0, int &sid1, int &sid2, int shared_v0, int shared_v1, int v0, int v1, int v2) {
    
    if(shared_v0 == v0) 
        sid0 = 0;
    else if(shared_v0 == v1)
        sid0 = 1;
    else if(shared_v0 == v2) 
        sid0 = 2;
    else {
        return -1;
    }    
    if(shared_v1 == v0) 
        sid1 = 0;
    else if(shared_v1 == v1)
        sid1 = 1;
    else if(shared_v1 == v2) 
        sid1 = 2;
    else {
        return -1;
    }       
    assert(sid0 != sid1); 
    if((sid0 + sid1 == 1)) // i.e. they are either 0 or 1
        sid2 = 2;
    else if(sid0 + sid1 == 2) // i.e. they are either 0 or 2
        sid2 = 1;
    else if(sid0 + sid1 == 3) // i.e. they are either 1 or 2
        sid2 = 0;
    else {
        return -1;
    }
    return 0;
}

void remove(vector<int> &vec, int val) {
    vec.erase(std::remove(vec.begin(), vec.end(), val), vec.end());
}

void createFace2dualEdgeMap(std::map<int, vector<int>> &Face2dualEdge, Eigen::MatrixXi hinge_edges) {
    for (int hi = 0; hi < hinge_edges.rows(); hi++)  {
        int f0 = hinge_edges.row(hi)(0);
        int f1 = hinge_edges.row(hi)(1);
        if(f0 == -1 or f1 == -1)
            continue; // not a hinge edge

        if(Face2dualEdge.find(f0) == Face2dualEdge.end()) { // add key
            vector<int> vect {hi};
            Face2dualEdge.insert({f0, vect}) ;
        }
        else { // key exists
            Face2dualEdge[f0].push_back(hi);
        }
        if(Face2dualEdge.find(f1) == Face2dualEdge.end()) { // add key
            vector<int> vect {hi};
            Face2dualEdge.insert({f1, vect}) ;
        }
        else { // key exists
            Face2dualEdge[f1].push_back(hi);
        }
    }    
}

void createFace2occuranceMap(std::map<int, int> &face2occurannce, Eigen::MatrixXi hinge_edges) {
    for (int hi = 0; hi < hinge_edges.rows(); hi++) 
    {
        int f0 = hinge_edges.row(hi)(0);
        int f1 = hinge_edges.row(hi)(1);

        if(face2occurannce.find(f0) == face2occurannce.end()) { // add key
            face2occurannce.insert({f0,1}); 
        }
        else { // key exists
            face2occurannce[f0] += 1;
        }
        if(face2occurannce.find(f1) == face2occurannce.end()) { // add key
            face2occurannce.insert({f1,1});  
        }
        else { // key exists
            face2occurannce[f1] += 1;
        }
    }    
}

void orderVertices(int &v0, int &v1) {
    if(v0 > v1) {
        swap(v0,v1);
    }
}

int findCommonFaceEdge(triplet e0s, triplet e1s) {
    vector<int> Avec{get<0>(e0s), get<1>(e0s), get<2>(e0s)};
    vector<int> Bvec{get<0>(e1s), get<1>(e1s), get<2>(e1s)};
    // printvec(Avec, "Avec");
    // printvec(Bvec, "Bvec");

    vector<int> C;
    sort(Avec.begin(), Avec.end());
    sort(Bvec.begin(), Bvec.end());
    set_intersection(Avec.begin(), Avec.end(),
                     Bvec.begin(), Bvec.end(),
                     back_inserter(C));
    
    if(C.size() == 0) {
        cout << "ERROR NO COMMON EDGE DEBUGGGGG!!!!\n";
        // exit(1);
        return -1; 
    }
    if(C.size() > 1) {
        cout << "ERROR MORE THAN 1 COMMON EDGE DEBUGGGGG!!!!\n";
        exit(1);
    }

    return C[0];
}

void findCommonVertex(int &shared_v0, int &shared_v1, Eigen::Vector3i f0, Eigen::Vector3i f1) {
    int v0f0 = f0(0), v1f0 = f0(1), v2f0 = f0(2);
    int v0f1 = f1(0), v1f1 = f1(1), v2f1 = f1(2);

    vector<int> Avec{v0f0, v1f0, v2f0};
    vector<int> Bvec{v0f1, v1f1, v2f1};
    // printvec(Avec, "Avec");
    // printvec(Bvec, "Bvec");

    vector<int> C;
    sort(Avec.begin(), Avec.end());
    sort(Bvec.begin(), Bvec.end());
    set_intersection(Avec.begin(),Avec.end(),
                          Bvec.begin(),Bvec.end(),
                          back_inserter(C));
    
    // printvec(C, "C contains");
    shared_v0 = C[0];
    shared_v1 = C[1];
}

void mirrorVertex(vector<Eigen::Vector3d> &vec, int sid0, int sid1, int sid2) {
    Eigen::Vector3d e1_ref, e2_ref;
    e1_ref = vec[sid1] - vec[sid0]; // refresh these values since rotation is done
    e2_ref = vec[sid2] - vec[sid0]; // refresh these values since rotation is done
    // printf("Applying mirror transformation again and flip the indices \n");
    Eigen::Vector3d etemp = e2_ref;
    Eigen::Vector3d enorm = e1_ref.normalized();
    vec[sid2] = vec[sid2] -2*(etemp - etemp.dot(enorm)*enorm); 
}

double compute_hinge_width(double scale, bool isHalfHinge) {
    // hinge parameters:
    double scale_factor = 1.0; // fox example for which scaling makes sense.. TODO check this factor with Marco (90 * 2.23)
    // double scale_factor = 1.0 / (200.0); // fox example for which scaling makes sense.. TODO check this factor with Marco (90 * 2.23)
    // double scale_factor = 1 / (2.23 * 90); // fox example for which scaling makes sense..
    double track_width = 1.7 * scale_factor;
    double inner_diameter = 1.0 * scale_factor;
    double outer_diameter = 1.0 * scale_factor;
    double hinge_width = 2*(outer_diameter + track_width) + inner_diameter;
    double half_hinge_width = outer_diameter + track_width + inner_diameter;

    if(isHalfHinge) {
        return half_hinge_width * scale;
    }
    else {
        return hinge_width * scale;
    }
}

double computeOffset(double angle, double scale, double drill_bit_radius, bool isHalfHinge) {
    // angle is in radians..
    // the dihedral angle used here is the angle by which one of the half planes has to be rotated around the intersection line to match with the other half-planes
    // 0 dihedral angle means the two triangles are superimposed
    // 180 dihedral angle means the edge is flat

    // hinge parameters:
    angle = M_PI - angle; // following first todo
    double scale_factor = 1.0 * scale; // fox example for which scaling makes sense.. TODO check this factor with Marco (90 * 2.23)
    // double scale_factor = 1 / (2.23 * 90); // fox example for which scaling makes sense..
    double track_width = 1.7 * scale_factor;
    double inner_diameter = 1.0 * scale_factor;
    double outer_diameter = 1.0 * scale_factor;
    double rigid_width = 2 * track_width + inner_diameter;

    double half_rigid_width = track_width;

    // amount of matter that needs to be shaved FROM EACH of the triangles
    if(isHalfHinge) {
        double d_theta = (half_rigid_width / 2) / sin(angle / 2);
        return d_theta  + outer_diameter - drill_bit_radius;
    }
    else {
        double d_theta = (rigid_width / 2) / sin(angle / 2);
        return d_theta  + outer_diameter - drill_bit_radius;
    }
}

void getEndIndices(Eigen::Vector3i p0, Eigen::Vector3i p1, int &id0, int &id1) {
        
    int f0v0 = p0(0), f0v1 = p0(1), f0v2 = p0(2);
    int f1v0 = p1(0), f1v1 = p1(1), f1v2 = p1(2);
    int f01shared_v0, f01shared_v1;

    int f0sid0 = -1, f0sid1 = -1, f0sid2 = -1;
    int f1sid0 = -1, f1sid1 = -1, f1sid2 = -1;


    findCommonVertex(f01shared_v0, f01shared_v1, p0, p1);
    getIndices(f0sid0, f0sid1, f0sid2, f01shared_v0, f01shared_v1, f0v0, f0v1, f0v2);
    getIndices(f1sid0, f1sid1, f1sid2, f01shared_v0, f01shared_v1, f1v0, f1v1, f1v2);

    if(f0sid0 == -1 or f0sid1 == -1 or f0sid2 == -1 or f1sid0 == -1 or f1sid1 == -1 or f1sid2 == -1) {
        printf("Something went wrong in finding right indices DEBUG\n"); 
        exit(1);
    }

    id0 = f0sid2;
    id1 = f1sid2;
}

void performTriangleOffset(Eigen::Vector3d vpos0, Eigen::Vector3d &vpos1, Eigen::Vector3d &vpos2, double offset) {
    Eigen::Vector3d v12 = vpos2 - vpos1, v12n = v12 / v12.norm();
    Eigen::Vector3d v01 = vpos0 - vpos1, v01n = v01 / v01.norm();
    Eigen::Vector3d v02 = vpos0 - vpos2, v02n = v02 / v02.norm();
    Eigen::Vector3d perp = v01 - v01.dot(v12n)*v12n;

    Eigen::Vector3d perp_u = perp / perp.norm();

    Eigen::Vector3d v01_d = v01n * (offset / (v01n.dot(perp_u)) );
    Eigen::Vector3d v02_d = v02n * (offset / (v02n.dot(perp_u)) );
    
    // update v1... 
    vpos1 = vpos1 + v01_d;
    vpos2 = vpos2 + v02_d;
}

void uniformOffseting(Eigen::Vector3d vpos0, Eigen::Vector3d &vpos1, Eigen::Vector3d &vpos2, double offset) {
    Eigen::Vector3d vcom = (vpos0 + vpos1 + vpos2) / 3.0;
    Eigen::Vector3d v0c = vpos0 - vcom, v0cn = v0c / v0c.norm();
    Eigen::Vector3d v1c = vpos1 - vcom, v1cn = v1c / v1c.norm();
    Eigen::Vector3d v2c = vpos2 - vcom, v2cn = v2c / v2c.norm();

    Eigen::Vector3d v0c_d = v0cn * offset;
    Eigen::Vector3d v1c_d = v1cn * offset;
    Eigen::Vector3d v2c_d = v2cn * offset;
    
    // update v'is... 
    vpos0 = vpos0 - v0c_d;
    vpos1 = vpos1 - v1c_d;
    vpos2 = vpos2 - v2c_d;
}

void findOppositeVertices(const vector<Eigen::Vector3i> &sheet_indices, const vector<int> &fvertices_visited, int prev_flatv0, int prev_flatv1, int &new_flatv2, int &prev_flatv2) {
    bool found_prev = false, found_new = false;
    for(int f2i = 0; f2i < sheet_indices.size(); f2i++) {
        int sv0 = sheet_indices[f2i](0);  int sv1 = sheet_indices[f2i](1);  int sv2 = sheet_indices[f2i](2);
        // printf("f2i : %d vertices:  %d %d %d \n", f2i, sv0, sv1, sv2);
        if((sv0 == prev_flatv0 and sv1 == prev_flatv1) or (sv0 == prev_flatv1 and sv1 == prev_flatv0))  {
            if(find(fvertices_visited.begin(), fvertices_visited.end(), sv2) == fvertices_visited.end())
                new_flatv2 = sv2;
            else 
                prev_flatv2 = sv2;
        }

        if((sv2 == prev_flatv0 and sv1 == prev_flatv1) or (sv2 == prev_flatv1 and sv1 == prev_flatv0)) {
            if(find(fvertices_visited.begin(), fvertices_visited.end(), sv0) == fvertices_visited.end())
                new_flatv2 = sv0;
            else 
                prev_flatv2 = sv0;
        }

        if((sv0 == prev_flatv0 and sv2 == prev_flatv1) or (sv0 == prev_flatv1 and sv2 == prev_flatv0)) {
            if(find(fvertices_visited.begin(), fvertices_visited.end(), sv1) == fvertices_visited.end())
                new_flatv2 = sv1;
            else 
                prev_flatv2 = sv1;
        }
        if(prev_flatv2 != -1 and new_flatv2 != -1)
            break;
    }
    if(prev_flatv0 == -1 or prev_flatv1 == -1 or prev_flatv2 == -1 or new_flatv2 == -1) {
        printf("Something went wrong there... DEBUG!!!! %d %d %d %d\n ",prev_flatv0, prev_flatv1, prev_flatv2, new_flatv2);
        exit(1);
        // break;
        // also check if i am updating the sheet_vertices and sheet_indices with correct face indexing that might be incorrect somehow...
    }
}


void connectTriangle2Previous(const vector<Eigen::Vector3d> &sheet_vertices, int prev_flatv0, int prev_flatv1, int sid0, int sid1, int sid2, vector<Eigen::Vector3d> &vpos_refs) {
    Eigen::Vector3d trans = sheet_vertices[prev_flatv0] - vpos_refs[sid0];
    // printf("Trans %f %f %f\n", trans(0), trans(1), trans(2));
    vpos_refs[sid0] += trans;
    vpos_refs[sid1] += trans;
    vpos_refs[sid2] += trans; 

    // printf("Local Edge lengths trans.. %f %f %f\n", (vpos_refs[sid0]-vpos_refs[sid1]).norm(),(vpos_refs[sid2]-vpos_refs[sid1]).norm(),(vpos_refs[sid2]-vpos_refs[sid0]).norm() );

    Eigen::Vector3d e0_ref = sheet_vertices[prev_flatv1] - sheet_vertices[prev_flatv0]; // from previous step e0 from prev step
    Eigen::Vector3d e1_ref = vpos_refs[sid1] - vpos_refs[sid0];
    Eigen::Vector3d e2_ref = vpos_refs[sid2] - vpos_refs[sid0];

    double cosTheta = e0_ref.dot(e1_ref) / (e0_ref.norm()*e1_ref.norm());
    double sinTheta = (e1_ref.cross(e0_ref)).norm() / (e1_ref.norm()*e0_ref.norm());
    if(e1_ref.cross(e0_ref).dot(Eigen::Vector3d(0,0,1)) < 0)
        sinTheta *= -1;

    Eigen::Matrix3d rotMat;
    rotMat << cosTheta , -sinTheta, 0,
                sinTheta, cosTheta,  0,
                0, 0, 1;
    
    vpos_refs[sid1] = vpos_refs[sid0] +  rotMat * e1_ref; // puts the edge back.. so in theory this vertex should match the sheet_vertices[prev_flatv0]
    if((sheet_vertices[prev_flatv1] - vpos_refs[sid1]).norm() > 1e-6) {
        printf("Things which were close previously are now super far and do not share an edge between them.. wierd DEBUG!!!\n");
        exit(1);
    }

    vpos_refs[sid2] = vpos_refs[sid0] + rotMat * e2_ref; 
}

void chooseCorrectOrder(int &nv0, int &nv1, int &nv2, vector<Eigen::Vector3d> &sheet_vertices, int v0, int v1, int v2, double l0, double l1, double l2) {
    
    // orientation should not change but have an assert at the end
    Eigen::Vector3d cp = (sheet_vertices[v0] - sheet_vertices[v1]).cross((sheet_vertices[v1] - sheet_vertices[v2]));
    if(cp.dot(Eigen::Vector3d(0,0,1)) < 0 ) {
        // printf("CP : %f %f %f\n", cp[0], cp[1], cp[2]);
        // fix orientation here. 
        std::swap(v0, v1); 
    } 

    double nl0 = (sheet_vertices[v0] - sheet_vertices[v1]).norm();
    double nl1 = (sheet_vertices[v1] - sheet_vertices[v2]).norm();
    double nl2 = (sheet_vertices[v2] - sheet_vertices[v0]).norm();
    double eps = 1e-6;


    if(((nl0-l0) < eps) and ((nl1-l1) < eps) and ((nl2-l2) < eps)) {
        nv0 = v0; nv1 = v1; nv2 = v2; 
    } 
    else if( ((nl2-l0) < eps) and ((nl0-l1) < eps) and ((nl1-l2) < eps)) {
        nv0 = v2; nv1 = v0; nv2 = v1; 
    }
    else if( ((nl1-l0) < eps) and ((nl2-l1) < eps) and ((nl0-l2) < eps)) {
        nv0 = v1; nv1 = v2; nv2 = v0; 
    }
    else {
        printf("index order : %d %d %d\nflat lengths : %f %f %f old lengths %f %f %f\n", v0,v1,v2, nl0, nl1, nl2, l0, l1, l2);
        printf("Something went wrong in choosing correct order DEBUG\n");
        exit(1);
    }

    if(nv0 == -1 or nv1 == -1 or nv2 == -1) {
        printf("Something went wrong in choosing correct order DEBUG\n");
        exit(1);
    }
}


void findSharedIndices(vector<int> f0vids, vector<int> f1vids, int &csid0, int &csid1, int &csid2, int &psid0, int &psid1, int &psid2) {
    // f0vids is for the face with current and f1vids is for the previous face.. 

    int bool_mesh_cut = -1, shared_fv1 = -1; // shared vertices in non-offset space shares dofs
    bool found1 = false, found2 = false; 

    for(auto vi: f0vids) {
        for(auto vj: f1vids) {
            if(vi == vj and not found1) {
                found1 = true;
                bool_mesh_cut = vi;
            }
            else if (vi == vj and not found2){
                found2 = true;
                shared_fv1 = vi;
            }
            else   
                continue;

            if(found1 and found2) 
                break;
        }
        if(found1 and found2) 
            break;
    }
    
    if(not found1 or not found2) {
        printf("something went wrong in finding shared_vertex : %d %d DEBUGGGG!!!!\n", bool_mesh_cut, shared_fv1);
    }

    getIndices(csid0, csid1, csid2, bool_mesh_cut, shared_fv1, f0vids[0], f0vids[1], f0vids[2]);
    getIndices(psid0, psid1, psid2, bool_mesh_cut, shared_fv1, f1vids[0], f1vids[1], f1vids[2]);

    if(csid0 == -1 or csid1 == -1 or csid2 == -1 or psid0 == -1 or psid1 == -1 or psid2 == -1) {
        printf("something went wrong debug in finding the correct vertices.. DEBUG!!! \n");
        exit(1);
    }
}
Eigen::Vector3d getDistanceBwLines(const vector<Eigen::Vector3d> & sheet_vertices_offset, int cv0, int cv1, int pv0, int pv1) {
    // v0, v1 lies on one line and v2, v3 lies on line.. special case distance between two lines

    Eigen::Vector3d cv0pos = sheet_vertices_offset[cv0];
    Eigen::Vector3d cv1pos = sheet_vertices_offset[cv1];
    Eigen::Vector3d pv0pos = sheet_vertices_offset[pv0];
    Eigen::Vector3d pv1pos = sheet_vertices_offset[pv1];

    // check if they are parallel lines or coincident (or not)
    if( fabs(fabs(((cv0pos-cv1pos).normalized()).dot((pv0pos-pv1pos).normalized())) - 1) > 1e-10) { 
        // handling for both 0 degree and 180 degree case
        printf("Lines are not parallel DEBUG\n");
        exit(1);
    } 

    if( (pv1pos - pv0pos).norm() < 1e-18) {
        return Eigen::Vector3d(0,0,0);
    }

    Eigen::Vector3d base_vec_norm = (pv1pos - pv0pos).normalized();
    Eigen::Vector3d distance_vec = (cv0pos - pv0pos) - (base_vec_norm).dot((cv0pos - pv0pos))*(base_vec_norm);
    return distance_vec;

}

// returns the angle value in degrees
double getSafeBendingAngle(double length) {
    if(length < 9) {
        return 0.0;
    }
    if (length > 21) {
        return 180.0; 
    }
    return (length - 8.97) * (180-26.26) / (20.92 - 8.97) + 26.26;
}

double getHalfHingeSafeBendingAngle(double length) {
    if(length < 9) {
        return 0.0;
    }
    if (length > 38.0) { // interpolated value based on the theta limit. 
        return 180.0; 
    }
    return (17 * length / 3) - 36; 
}

// returns the minimum hinge length that is needed to support the bending angle
double getSafeHingeLength(double angle) {
    if (angle > 180.0 or angle < 0.0) {
        printf("BENDING ANGLE CANNOT EXCEDE 180 degrees. DEBUG!!!!\n");
        exit(1);
    }
    else {
        return 9.0 + (angle*12.0/180.0);
    }
}

shared_ptr<transNode> findCorrectDistanceBetweenTriangle(const std::vector<Eigen::Vector3i> &sheet_indices, 
            const std::vector<Eigen::Vector3i> &sheet_indices_offset,  
            std::vector<Eigen::Vector3d> &sheet_vertices_offset, int curr_face_id, 
            int prev_face_id, double drill_bit_radius, bool isHalfHinge) {

    double desired_distance = compute_hinge_width(1, isHalfHinge) - 2 * drill_bit_radius; // which is equal to the hinge offset.

    // since the order is same for sheet_indices and sheet_indices_offset i can use this information to find the right vertex order to querry the common edge
    vector<int> f0vids = {sheet_indices[curr_face_id](0), sheet_indices[curr_face_id](1), sheet_indices[curr_face_id](2)}; 
    vector<int> f1vids = {sheet_indices[prev_face_id](0), sheet_indices[prev_face_id](1), sheet_indices[prev_face_id](2)};
    // printf("f0 : %d vids : %d %d %d f1 : %d vids : %d %d %d\n", curr_face_id, f0vids[0], f0vids[1], f0vids[2], prev_face_id, f1vids[0], f1vids[1], f1vids[2]);

    int csid0 = -1, csid1 = -1, csid2 = -1, psid0 = -1, psid1 = -1, psid2 = -1; // order from shared vertex...
    findSharedIndices(f0vids, f1vids, csid0, csid1, csid2, psid0, psid1, psid2); // csid0 and psid0 maps their corresponding 0shared_f{i}v0... 
    // printf("indices of csids : %d %d %d pvids : %d %d %d\n", csid0, csid1, csid2, psid0, psid1, psid2);

    // order is same in offset and not offset edges so just need to find 0,1,2 map to vertices
    vector<int> of0vids = {sheet_indices_offset[curr_face_id](0), sheet_indices_offset[curr_face_id](1), sheet_indices_offset[curr_face_id](2)}; 
    vector<int> of1vids = {sheet_indices_offset[prev_face_id](0), sheet_indices_offset[prev_face_id](1), sheet_indices_offset[prev_face_id](2)};
    int osf0v0 = of0vids[csid0], osf0v1 = of0vids[csid1], osf1v0 = of1vids[psid0], osf1v1 = of1vids[psid1]; // shared vertices in offset space

    // printf("on the offset surface the indices of current and previous are : %d %d %d %d\n", osf0v0, osf0v1, osf1v0, osf1v1);
    Eigen::Vector3d dis_vector = getDistanceBwLines(sheet_vertices_offset, osf0v0, osf0v1, osf1v0, osf1v1);

    // printf("The distance between the two faces : %d %d is : %f\n", prev_face_id, curr_face_id, dis_vector.norm());
    Eigen::Vector3d trans_vec;
    if(dis_vector.norm() < 1e-10) {
        // there is no direction that it can take so provide a direction that seprates the two opposite vertices.
        Eigen::Vector3d cvec2 = sheet_vertices_offset[of0vids[csid2]], pvec2 = sheet_vertices_offset[of1vids[psid2]];
        Eigen::Vector3d horiz_vec = sheet_vertices_offset[of0vids[csid1]] - sheet_vertices_offset[of0vids[csid0]]; horiz_vec.normalize();
        dis_vector = (pvec2 - cvec2) - horiz_vec.dot((pvec2 - cvec2))*horiz_vec;
        trans_vec = -(dis_vector.normalized()) * desired_distance; // modify the distance between the two faces..
        // printf("trans_vec norm : %f %f %f %f\n", trans_vec.norm(), trans_vec[0], trans_vec[1], trans_vec[2]);
    }
    else {
        trans_vec = -dis_vector + (dis_vector.normalized()) * desired_distance; // modify the distance between the two faces.. 

    }


    // Eigen::Vector3d trans_vec = Eigen::Vector3d(0,0,0); // DEBUG makes the two face stuck together.. 
    // Eigen::Vector3d trans_vec = -dis_vector; // DEBUG makes the two face stuck together..
    shared_ptr<transNode> node = make_shared<transNode>();
    node->id = curr_face_id;
    node->translation = trans_vec;
    return node; 
}


bool checkHit(Eigen::Vector3d v0, Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d &h0, Eigen::Vector3d &h1) {
    // checks if the ray perpendicular from v0 to the line segment v1 and v2 hits the line segment or not
    Eigen::Vector3d v12 = v1-v2, v12n = v12 / v12.norm();
    Eigen::Vector3d vi  = v0 + (v1-v0) - (v1-v0).dot(v12n)*v12n;
    double d1i = getDistance(v1, vi);
    double d2i = getDistance(v2, vi);
    double d12 = getDistance(v1, v2);

    // printf("v0 : %f %f %f \n", v0(0), v0(1), v0(2));
    // printf("v1 : %f %f %f \n", v1(0), v1(1), v1(2));
    // printf("v2 : %f %f %f \n", v2(0), v2(1), v2(2));
    // printf("vi : %f %f %f \n", vi(0), vi(1), vi(2));

    // printf("Distance values are %f %f %f\n", d1i, d2i, d12);

    if(fabs(d1i + d2i - d12) < 1e-6) {
        h0 = v0;
        h1 = vi;
        return true;
    }
    return false;
}

double getDistance(Eigen::Vector3d v0, Eigen::Vector3d v1) {
    return (v0-v1).norm();
}

std::string dtos(double x) {
    // https://stackoverflow.com/questions/16117528/converting-double-to-string-function-memory-issues
    std::stringstream s;  // Allocates memory on stack
    s << x;
    return s.str();       // returns a s.str() as a string by value
                          // Frees allocated memory of s
} 

void addHinge(const std::vector<Eigen::Vector3i> sheet_indices, const std::vector<Eigen::Vector3i> sheet_indices_offset, const std::vector<Eigen::Vector3d> sheet_vertices_offset, int curr_face_id, int prev_face_id, 
            Eigen::Vector3d &shared_hpos0, Eigen::Vector3d &shared_hpos1, Eigen::Vector3d &shared_hpos2, Eigen::Vector3d &shared_hpos3) {


    vector<int> f0vids = {sheet_indices_offset[curr_face_id](0), sheet_indices_offset[curr_face_id](1), sheet_indices_offset[curr_face_id](2)}; 
    vector<int> f1vids = {sheet_indices_offset[prev_face_id](0), sheet_indices_offset[prev_face_id](1), sheet_indices_offset[prev_face_id](2)};

    vector<int> ff0vids = {sheet_indices[curr_face_id](0), sheet_indices[curr_face_id](1), sheet_indices[curr_face_id](2)}; 
    vector<int> ff1vids = {sheet_indices[prev_face_id](0), sheet_indices[prev_face_id](1), sheet_indices[prev_face_id](2)};
    int csid0 = -1, csid1 = -1, csid2 = -1, psid0 = -1, psid1 = -1, psid2 = -1; // order from shared vertex...
    findSharedIndices(ff0vids, ff1vids, csid0, csid1, csid2, psid0, psid1, psid2); // csid0 and psid0 maps their corresponding 0shared_f{i}v0...
    int osf0v0 = f0vids[csid0], osf0v1 = f0vids[csid1], osf1v0 = f1vids[psid0], osf1v1 = f1vids[psid1]; // shared vertices in offset space
    // printf("Current face : %d previous face : %d\n", curr_face_id, prev_face_id);
    bool found_first = false, found_second = false;
    
    // send ray from osf0v0 to the f1
    found_first = checkHit(sheet_vertices_offset[osf0v0], sheet_vertices_offset[osf1v0], sheet_vertices_offset[osf1v1], shared_hpos0, shared_hpos1 );
    if(found_first) {
        // printf("use : %d as the first one\n", osf0v0);
    }
    else {
        // send ray from osf1v0 to the f0
        found_first = checkHit(sheet_vertices_offset[osf1v0], sheet_vertices_offset[osf0v0], sheet_vertices_offset[osf0v1], shared_hpos1, shared_hpos0);
        if(found_first) {
            // printf("use : %d as the first one\n", osf1v0);
        }
        else  {
            printf("Could not find anything Debug %d %d %d %d %d %d... \n", curr_face_id, prev_face_id, osf0v0, osf0v1, osf1v0, osf1v1);
            printf("Most likely the hinges cannot be insered because the triangles are of too small size\n");
            exit(1);

        }
    }

    // send ray from osf0v1 to the f1
    found_second = checkHit(sheet_vertices_offset[osf0v1], sheet_vertices_offset[osf1v0], sheet_vertices_offset[osf1v1], shared_hpos2, shared_hpos3 );
    if(found_second) {
        // printf("use : %d as the second one\n", osf0v1);
    }
    else {
        // send ray from osf1v0 to the f0
        found_second = checkHit(sheet_vertices_offset[osf1v1], sheet_vertices_offset[osf0v0], sheet_vertices_offset[osf0v1], shared_hpos3, shared_hpos2);
        if(found_second) {
            // printf("use : %d as the second one\n", osf1v1);
        }
        else {
            printf("Could not find anything Debug %d %d %d %d %d %d... \n", curr_face_id, prev_face_id, osf0v0, osf0v1, osf1v0, osf1v1);
            printf("Most likely the hinges cannot be insered because the triangles are of too small size\n");
            exit(1);
        } 
            
    }
}

double perpendicularLength(Eigen::Vector3d v0, Eigen::Vector3d v1, Eigen::Vector3d v2) {
    Eigen::Vector3d v12 = v2 - v1, v12n = v12 / v12.norm();
    Eigen::Vector3d v01 = v0 - v1;
    Eigen::Vector3d perp = v01 - v01.dot(v12n)*v12n;    
    return perp.norm();
}

void EnergyGradient(double l0, Eigen::Vector3d v0, Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d &de0dv0, Eigen::Vector3d &de0dv1, Eigen::Vector3d &de0dv2) {

    double v0x = v0(0), v0y = v0(1);
    double v1x = v1(0), v1y = v1(1);
    double v2x = v2(0), v2y = v2(1);

    double copt1 = -v0x;
    double copt2 = -v1x;
    double copt3 = copt2 + v2x;
    double copt4 = fabs(copt3);
    double copt5 = pow(copt4,2);
    double copt6 = -v1y;
    double copt7 = copt6 + v2y;
    double copt8 = fabs(copt7);
    double copt9 = pow(copt8,2);
    double copt10 = copt5 + copt9;
    double copt11 = sqrt(copt10);
    double copt12 = 1/copt11;
    double copt13 = copt1 + v1x;
    double copt14 = copt12*copt13*copt3;
    double copt15 = -v0y;
    double copt16 = copt15 + v1y;
    double copt17 = copt12*copt16*copt7;
    double copt18 = copt14 + copt17;
    double copt19 = -(copt12*copt18*copt3);
    double copt20 = copt1 + copt19 + v1x;
    double copt21 = fabs(copt20);
    double copt22 = pow(copt21,2);
    double copt23 = -(copt12*copt18*copt7);
    double copt24 = copt15 + copt23 + v1y;
    double copt25 = fabs(copt24);
    double copt26 = pow(copt25,2);
    double copt27 = copt22 + copt26;
    double copt28 = sqrt(copt27);
    double copt29 = 1/copt28;
    double copt33 = pow(copt3,2);
    double copt34 = 1/copt10;
    double copt30 = l0;
    double copt31 = -copt28;
    double copt32 = copt30 + copt31;
    double copt37 = (copt20) / fabs(copt20);
    double copt44 = pow(copt7,2);
    double copt39 = (copt24) / fabs(copt24);
    double copt51 = copt10*copt11;
    double copt52 = 1/copt51;
    double copt53 = (copt3) / fabs(copt3);
    double copt55 = -(copt12*copt13);
    double copt56 = copt12*copt3;
    double copt57 = copt13*copt3*copt4*copt52*copt53;
    double copt58 = copt16*copt4*copt52*copt53*copt7;
    double copt59 = copt55 + copt56 + copt57 + copt58;
    double copt69 = (copt7) / fabs(copt7);
    double copt50 = copt12*copt18;
    double copt71 = -(copt12*copt16);
    double copt72 = copt12*copt7;
    double copt73 = copt13*copt3*copt52*copt69*copt8;
    double copt74 = copt16*copt52*copt69*copt7*copt8;
    double copt75 = copt71 + copt72 + copt73 + copt74;
    double copt87 = copt12*copt13;
    double copt88 = -(copt13*copt3*copt4*copt52*copt53);
    double copt89 = -(copt16*copt4*copt52*copt53*copt7);
    double copt90 = copt87 + copt88 + copt89;
    double copt85 = -(copt12*copt18);
    double copt101 = copt12*copt16;
    double copt102 = -(copt13*copt3*copt52*copt69*copt8);
    double copt103 = -(copt16*copt52*copt69*copt7*copt8);
    double copt104 = copt101 + copt102 + copt103;
    
    de0dv0(0) = -0.5*copt29*copt32*(2*copt21*(-1 + copt33*copt34)*copt37 + 2*copt25*copt3*copt34*copt39*copt7);
    de0dv0(1) = -0.5*copt29*copt32*(2*copt25*copt39*(-1 + copt34*copt44) + 2*copt21*copt3*copt34*copt37*copt7);
    de0dv0(2) = 0;
    
    de0dv1(0) = -0.5*copt29*copt32*(2*copt21*copt37*(1 + copt50 - copt18*copt3*copt4*copt52*copt53 - copt12*copt3*copt59) + \
                2*copt25*copt39*(-(copt18*copt4*copt52*copt53*copt7) - copt12*copt59*copt7));
    de0dv1(1) = -0.5*copt29*copt32*(2*copt21*copt37*(-(copt12*copt3*copt75) - \
                copt18*copt3*copt52*copt69*copt8) + 2*copt25*copt39*(1 + copt50 - \
                copt12*copt7*copt75 - copt18*copt52*copt69*copt7*copt8));
    de0dv1(2) = 0;
    
    de0dv2(0) = -0.5*copt29*copt32*(2*copt21*copt37*(copt18*copt3*copt4*copt52*copt53 \
                + copt85 - copt12*copt3*copt90) + \
                2*copt25*copt39*(copt18*copt4*copt52*copt53*copt7 - copt12*copt7*copt90));
    de0dv2(1) = -0.5*copt29*copt32*(2*copt21*copt37*(-(copt104*copt12*copt3) + \
                copt18*copt3*copt52*copt69*copt8) + \
                2*copt25*copt39*(-(copt104*copt12*copt7) + \
                copt18*copt52*copt69*copt7*copt8 + copt85));
    de0dv2(2) = 0;
}