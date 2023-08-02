#include "Settings.hpp"
#include "helper.hpp"

LIBSL_WIN32_FIX; // NOTE it adds some wierd global fixes so add to individual files and not to the global helper.hpp file

int g_W = 1024; // Window width
int g_H = 1024; // Window height
int ledThreshold = 3; // ledThreshold works for both small and bigLEds

v3f scene_eye = v3f(0, 0, 10);
v3f scene_at = v3f(0, 0, 0);
v3f scene_up = v3f(0, 1, 0);
double mesh_scale = 100.0;
int frame_count = 0; 
vector<vector<int>> worldFaceLEDPos;
vector<vector<v3f>> offsetVerticesPlane; // per face store all the 3 offset vertices in plane

std::string file_stg{};
std::string file_off{};
std::string file_sht{};
std::string file_rct{};
std::string file_led{};
int useBigLed = 0;
bool use_cli_mode = false;

std::string file_name;
string file_name_num;
std::string file_rcts{};
std::map<int, vector<int>> sheet_face2Edge;


// declare corresponding vertex format ** must match vertex data **
typedef MVF3(mvf_vertex_3f, mvf_normal_3f, mvf_texcoord0_2f) t_VertexDesc;

// declare custom mesh
typedef TriangleMesh_generic<t_VertexData> t_Mesh;

// declare custom mesh renderer
typedef MeshRenderer<t_VertexDesc> t_Renderer;

AutoPtr<t_Mesh> g_Mesh;
AutoPtr<t_Renderer> g_Renderer;

vector<vector<int>> ledIndicesPerFace;
vector<v3f> led_points; // led point directly in 3D
vector<v3f> randomColors;// Random color for that led point
vector<pair<uint, uint>> led_edges; // led edge directly in 3D
//  ------------------------------------------- //
// variables for CVT reprsenetation of the mesh
vector<vector<double>> edgeLengthsPerTriangle;
vector<vector<pair<uint, uint>>> edgesPerTriangle;
vector<vector<v3f>> ledVerticesPerTriangle;
vector<v3f> normalPerTriangle;
bool boolDisplyCVT = false, boolDisplyNN = false, boolVisualizeRects = false, boolDisplyPoints = false;
bool boolDisplyTriangle = true; 
bool boolDisplyOffsetTriangle = true;
bool boolUseNormalOffset = true;
bool variation_example = false; 

uint file_format = 1; // false means old true means new
bool e_without_triangle_format = false;
bool without_offset_format = false;
bool mapping_old_format = false;
//  ------------------------------------------- //
// variables for CVT reprsenetation of the mesh
vector<double> nedgeLengthsMesh;
vector<pair<uint, uint>> nedgesMesh, nedgesMeshNeighbour; // can directly reference points from mesh->vertices
vector<uint> ledVertex2WorldFace; // maps led points to the face that contains that LED
//  ------------------------------------------- //

//  ------------------------------------------- //
// variables for RECT representation of LEDs
vector<vector<v3f>> led_points_extra; // other points of the LEDs
//  ------------------------------------------- //

vector<double> sphere_points; 
vector<int> sphere_faces; 

v3f v3d2f(v3d v) {
	return v3f(v[0],v[1], v[2]);
}
v3d v3f2d(v3f v) {
	return v3d(v[0],v[1], v[2]);
}

void mainMouseWheel(int val) {
	TrackballUI::trackball().translation() = TrackballUI::trackball().translation() + v3f(0, 0, 0.4*val);
}

void mainReshape(uint w,uint h) {
	glViewport(0,0,w,h);
	g_W = w;
	g_H = h;
}

v3f getNormal(int findex) {
	auto mesh_vertices = g_Mesh.raw()->vertices();
	auto mesh_triangles = g_Mesh.raw()->triangles();
	v3f wv0 = mesh_vertices[mesh_triangles[findex][0]].pos;
	v3f wv1 = mesh_vertices[mesh_triangles[findex][1]].pos;
	v3f wv2 = mesh_vertices[mesh_triangles[findex][2]].pos;
	v3f n = cross((wv0-wv1), (wv1-wv2));
	return n; 
}

vector<pair<int,int>> getEdges() {
	auto mesh_triangles = g_Mesh.raw()->triangles();
	vector<pair<int,int>> edges; 
	for(auto tri : mesh_triangles) {
		edges.push_back(make_pair(tri[0], tri[1]));
		edges.push_back(make_pair(tri[1], tri[2]));
		edges.push_back(make_pair(tri[2], tri[0]));
	}
	return edges; 
}

void mainRender() {
	
	// enable gl flags
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);

	/// render on screen
	GPUHelpers::clearScreen(LIBSL_COLOR_BUFFER | LIBSL_DEPTH_BUFFER, 0.0, 0.0, 0.0);
	GPUHelpers::Transform::set(LIBSL_MODELVIEW_MATRIX, TrackballUI::matrix());
	// printf("TrackballUI trans : %f %f %f\n", TrackballUI::translation()[0], TrackballUI::translation()[1], TrackballUI::translation()[2]);
	// printf("TrackballUI rotat : %f %f %f %f\n", TrackballUI::rotation()[0], TrackballUI::rotation()[1], TrackballUI::rotation()[2], TrackballUI::rotation()[3]);

	// render mesh
	glColor3f(0.1, 0.4, 0.2);
	g_Renderer->render();

	float normal_offset;

	if(boolUseNormalOffset) {
		normal_offset = 0.01; 
	}
	else { 
		normal_offset = 0.0; 
	}

	if (boolDisplyPoints) {
		// glColor3f(1, 1, 1);
		glBegin(GL_QUADS);
		int f_iter = 0;
		int led_iter = 0;
		static vector<double> rcs, gcs , bcs; 
		static vector<double> randomColor; 
		int totalLeds = led_points.size();

		rcs.resize(totalLeds, 1); gcs.resize(totalLeds, 1); bcs.resize(totalLeds, 1); 
		for (auto e : worldFaceLEDPos) {
			v3f n = getNormal(f_iter); 
			for (auto ee : e) {
				if(frame_count % 30 == 0) {
					// rcs[led_iter] = 1, gcs[led_iter] = 1, bcs[led_iter] = 1;
					// rcs[led_iter] = ((double) rand() / (RAND_MAX)), gcs[led_iter] = ((double) rand() / (RAND_MAX)), bcs[led_iter] = ((double) rand() / (RAND_MAX)); 
				}
				v3f rc = randomColors[led_iter];
				glColor3f(rc[0], rc[1], rc[2]);
				// glColor3f(rcs[led_iter], gcs[led_iter], bcs[led_iter]);
				// glColor3f(rcs[led_iter], gcs[led_iter], bcs[led_iter]);
				v3d ledCpos = v3d(led_points[ee]);
				// v3d ledCpos = v3d(ee);
				for(int qi = 0; qi < sphere_faces.size()/4; qi++) { // quad index
					int iv0 = sphere_faces[qi*4], iv1 = sphere_faces[qi*4 + 1], iv2 = sphere_faces[qi*4 + 2], iv3 = sphere_faces[qi*4 + 3];
					v3d v1 = v3d(sphere_points[iv0*3 + 0], sphere_points[iv0*3 + 1], sphere_points[iv0*3 + 2]) + ledCpos;
					v3d v2 = v3d(sphere_points[iv1*3 + 0], sphere_points[iv1*3 + 1], sphere_points[iv1*3 + 2]) + ledCpos;
					v3d v3 = v3d(sphere_points[iv2*3 + 0], sphere_points[iv2*3 + 1], sphere_points[iv2*3 + 2]) + ledCpos;
					v3d v4 = v3d(sphere_points[iv3*3 + 0], sphere_points[iv3*3 + 1], sphere_points[iv3*3 + 2]) + ledCpos;
					
					glVertex3dv(&v1[0]);
					glVertex3dv(&v2[0]);
					glVertex3dv(&v3[0]);
					glVertex3dv(&v4[0]);
				}
				led_iter++; 
			}
			f_iter++; 
		}
		glEnd();
	}

	if(boolDisplyTriangle) {
		glBegin(GL_LINES);
		glLineWidth(3.0f);
		glColor3f(1, 0, 0);
		auto mesh_triangles = g_Mesh.raw()->triangles();
		auto mesh_vertices = g_Mesh.raw()->vertices();
		for (int fi = 0 ; fi < mesh_triangles.size() ; fi++) {
			v3f n = normalPerTriangle[fi];
			for(int i = 0; i < 3; i++) {
				v3f pos0 = mesh_vertices[mesh_triangles[fi][i]].pos       + n*normal_offset;
				v3f pos1 = mesh_vertices[mesh_triangles[fi][(i+1)%3]].pos + n*normal_offset;
				glVertex3fv(&pos0[0]);
				glVertex3fv(&pos1[0]);
			}
		}
		glEnd();
	}

	if(boolDisplyOffsetTriangle and not without_offset_format) {
		glBegin(GL_LINES);
		glLineWidth(5.0f);
		glColor3f(1, 1, 1);
		for (int fi = 0 ; fi < offsetVerticesPlane.size(); fi++) {
			v3f n = normalPerTriangle[fi];
			for(int i = 0 ; i < 3; i ++) {
				v3f pos0 = offsetVerticesPlane[fi][i]       + n*normal_offset;
				v3f pos1 = offsetVerticesPlane[fi][(i+1)%3] + n*normal_offset;
				glVertex3fv(&pos0[0]);
				glVertex3fv(&pos1[0]);
			}
		}
		glEnd();
	}


	if(boolDisplyCVT) { 
		glBegin(GL_LINES);
		glColor3f(0, 1, 1);
		for (int evi = 0; evi < edgesPerTriangle.size(); evi++) {
			v3f n = normalPerTriangle[evi];

			vector<v3f> ledpoints = ledVerticesPerTriangle[evi];
			vector<pair<uint, uint>> ledEdges = edgesPerTriangle[evi];

			for(auto e: ledEdges) {
				v3f pos0 = ledpoints[e.first] + n*normal_offset;
				v3f pos1 = ledpoints[e.second] + n*normal_offset;
				glVertex3fv(&pos0[0]);
				glVertex3fv(&pos1[0]);
			}
		}
		
		// between triangles shortest partner
		for (auto e : nedgesMeshNeighbour) {
			v3f n0 = getNormal(ledVertex2WorldFace[e.first]); 
			v3f n1 = getNormal(ledVertex2WorldFace[e.second]); 
			v3f pos0 = led_points[e.first] + n0 * normal_offset * 10.0f;
			v3f pos1 = led_points[e.second] + n1 * normal_offset * 10.0f;
			glVertex3fv(&pos0[0]);
			glVertex3fv(&pos1[0]);	
		}

		glEnd();
	}


	if(boolDisplyNN) {
		glBegin(GL_LINES);
		glColor3f(1, 0, 1);
		for (auto e : nedgesMesh) {
			v3f n0 = getNormal(ledVertex2WorldFace[e.first]); 
			v3f n1 = getNormal(ledVertex2WorldFace[e.second]); 
			v3f pos0 = led_points[e.first] + n0 * normal_offset;
			v3f pos1 = led_points[e.second] + n1 * normal_offset;
			glVertex3fv(&pos0[0]);
			glVertex3fv(&pos1[0]);	
		}
		glEnd();
	}

	if(boolVisualizeRects) {
		glBegin(GL_LINES);
		glColor3f(1.0, 1.0, 1.0);
		// glColor3f(1.0, 0.0, 1.0);
		// display the LED rect form
		int ledIndex = 0; 
		for(auto rect : led_points_extra) {
			v3f n0 = getNormal(ledVertex2WorldFace[ledIndex]);
			for(int i = 0; i < 4; i++) {
				v3f pos0 = rect[i]       + n0 * normal_offset;
				v3f pos1 = rect[(i+1)%4] + n0 * normal_offset;
				glVertex3fv(&pos0[0]);
				glVertex3fv(&pos1[0]);
			}
			ledIndex++; 
		}
		glEnd();
	}

	frame_count++; 
}

void saveLedInfo(std::string file_led) {
	std::ofstream outfile(file_led, std::ios::out | std::ios::trunc);
	int perTriangleEdges = 0;
	for(auto e : edgesPerTriangle) {
		perTriangleEdges += e.size();
	}
	int totalLeds = led_points.size(), totalEdges = nedgesMeshNeighbour.size() + perTriangleEdges;
	outfile << "# totalFaces: " << worldFaceLEDPos.size() << " totalLeds:" << totalLeds << " totalLedEdges:" << totalEdges << endl;
	// int v_index = 0;
	for(auto led_indices: ledIndicesPerFace) {
		for (auto v_index : led_indices) {
			auto v = led_points[v_index];
			v3f ov0 = (led_points_extra[v_index][1] - led_points_extra[v_index][0]); // full vector
			v3f ov1 = (led_points_extra[v_index][3] - led_points_extra[v_index][0]); // full vector

			outfile << "v " << v[0] << " " << v[1] << " " << v[2] << " " << ledVertex2WorldFace[v_index] << " ";
			outfile << ov0[0] / 2.0 << " " << ov0[1] / 2.0 << " " << ov0[2] / 2.0 << " ";
			outfile << ov1[0] / 2.0 << " " << ov1[1] / 2.0 << " " << ov1[2] / 2.0 << endl;
		}
	}

	for(auto e : led_edges) {
		outfile << "e " << e.first << " " << e.second << "\n";
	}
	for(auto e : nedgesMeshNeighbour) {
		outfile << "e " << e.first << " " << e.second << "\n";
	}
	outfile.close();
}

void loadMesh(std::string file_off) {

	std::ifstream file_in(file_off);
	if (!file_in.is_open()) {
		std::cerr << "[ERROR] Could not open file " << file_off << '\n';
		return;
	}

	
	std::string line;
	std::string delimiter = " ";

	int tri_i = 0;
	int point_i = 0;
	int line_num = 0;
	bool start_vertices = false;
	bool start_faces = false;
	int vertex_count = 0, total_vertices = 0;
	int face_count = 0, total_faces = 0;
	while (std::getline(file_in, line)) {

		
		if(line_num == 1) {
			// cout << line << endl; 
			// for(auto l : lvals)
			// 	printf("%f ", l);
			// cout << endl; 
			vector<double> lvals = parse_doubles(line);
			total_vertices = int(lvals[0]);
			total_faces = int(lvals[1]);
			g_Mesh = AutoPtr<t_Mesh>(new t_Mesh(total_vertices, total_faces));
			start_vertices = true;
		}

		else if(start_vertices) {
			vector<double> lvals = parse_doubles(line);
			g_Mesh->vertexAt(point_i).pos = V3F(lvals[0] / mesh_scale, lvals[1] / mesh_scale, lvals[2] / mesh_scale);
			// std::cout << g_Mesh->vertexAt(point_i).pos << std::endl;
			point_i++;
			if(point_i >= total_vertices) {
				start_vertices = false; 
				start_faces = true;
			}
		}
		else if(start_faces) {
			// add a triangle.
			vector<double> lvals = parse_doubles(line);
			g_Mesh->triangleAt(tri_i)[0] = int(lvals[1]);
			g_Mesh->triangleAt(tri_i)[1] = int(lvals[2]);
			g_Mesh->triangleAt(tri_i)[2] = int(lvals[3]);
			// std::cout << g_Mesh->triangleAt(tri_i) << std::endl;
			tri_i++ ; 
		}
		else {
			// printf("Nothing to be found here.. \n");
		}
		line_num++; 
	}	
	g_Mesh.raw()->centerOn(v3f(0,0,0));
	// g_Mesh.raw()->reorientTriangles();
	printf("Num points : %d num tris : %d\n", point_i, tri_i);
	ledIndicesPerFace.resize(tri_i); // resize this based on number of triangles
}

v3f computeBC(v2f p0, v2f p1, v2f p2, v2f pt) {
	// https://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
	// The area of a triangle is
	v3f a = v3f(p0[0], p0[1], 0);
	v3f b = v3f(p1[0], p1[1], 0);
	v3f c = v3f(p2[0], p2[1], 0);
	v3f P = v3f(pt[0], pt[1], 0);

	v3f normal = v3f(0,0,1);
	double areaABC = fabs(dot(normal, cross((b - a), (c - a))));
	double areaPBC = fabs(dot(normal, cross((b - P), (c - P))));
	double areaPCA = fabs(dot(normal, cross((c - P), (a - P))));

	v3f bary; 
	bary[0] = areaPBC / areaABC;		 // alpha
	bary[1] = areaPCA / areaABC;		 // beta
	bary[2] = 1.0f - bary[0] - bary[1];  // gamma
	
	return bary;
}



void performNearestNeighbourAnalysis() { 
	// if I have to do analysis based on geodesic distance then i would need the face information as well.
	vector<v3d> vposs; 
	for(uint v0 = 0; v0 < led_points.size(); v0++) {
		v3f wv0 =  led_points[v0];
		uint bestNeighbour = -1;
		double bestDistance = std::numeric_limits<double>::max();
		for(uint v1 = 0; v1 <  led_points.size(); v1++) {
			v3f wv1 =  led_points[v1];
			if(v0 != v1) {
				double distance = euclideanDistance(wv0, wv1);
				if(distance < bestDistance) {
					bestNeighbour = v1; 
					bestDistance = distance; 
				}
			}
		}
		nedgeLengthsMesh.push_back(bestDistance);
		nedgesMesh.push_back(make_pair(v0, bestNeighbour));
	}

	// sheet_face2Edge;
	map<int, vector<int>> edge2Face; 
	for(auto f : sheet_face2Edge) {
		int face = f.first;
		vector<int> edges = f.second;
		for(auto e : edges) {
			if(edge2Face.find(e) == edge2Face.end()) {
				edge2Face[e] = vector<int> {face};
			}
			else {
				edge2Face[e].push_back(face);
			}
		}
	}

	// O(nv**2 * E) // nv == nleds in triangle times Number of edges
	map<pair<int, int>, bool> faceExistencePair;
	auto mesh_triangles = g_Mesh.raw()->triangles();
	auto mesh_vertices  = g_Mesh.raw()->vertices();
	v3f zaxis = v3f(0,0,1);
	for(int f0 = 0; f0 < worldFaceLEDPos.size(); f0++) {
		vector<int> v0s = worldFaceLEDPos[f0]; // all vertices in face0
		vector<int> edges = sheet_face2Edge[f0];
		int f1 = -1;
		for(auto e : edges) {
			vector<int> faces = edge2Face[e];
			// printf("face : %d edge : %d nfs : %ld\n", f0, e, faces.size());
			if(faces.size() == 1) // boundary face
				continue;
			// printf("faces0 : %d faces1 : %d\n", faces[0], faces[1]);
			f1 = (faces[0] == f0) ? faces[1] : faces[0];
			pair<int, int> face_pair = (f0 < f1) ? make_pair(f0,f1) : make_pair(f1, f0);

			if(faceExistencePair.find(face_pair) != faceExistencePair.end())
				continue; 
			
			// find common vertices on .off mesh that are shared between f0 and f1
			int shared_v0 = -1, shared_v1 = -1; 
			vector<int> f0vs = {int(mesh_triangles[f0][0]), int(mesh_triangles[f0][1]), int(mesh_triangles[f0][2])};
			vector<int> f1vs = {int(mesh_triangles[f1][0]), int(mesh_triangles[f1][1]), int(mesh_triangles[f1][2])};
			findCommonVertex(shared_v0, shared_v1, f0vs, f1vs);

			v3f edge_dierction = mesh_vertices[shared_v0].pos - mesh_vertices[shared_v1].pos;
			vector<pair<int, float>> distanceV0sEdge = {};
			// printf("face0 : %d face1 : %d\n", f0, f1);
			for(auto v0i : v0s) {
				float distance = getDistancePointToLine(led_points[v0i], mesh_vertices[shared_v0].pos, edge_dierction);				
				distanceV0sEdge.push_back(make_pair(v0i, distance));
			}
			std::sort(distanceV0sEdge.begin(), distanceV0sEdge.end(), [](auto &left, auto &right) { return left.second < right.second; });
			float running_mean;
			vector<int> bv0s, bv1s;
			if(distanceV0sEdge.size() > 0) {
				bv0s = {distanceV0sEdge[0].first};
				running_mean = distanceV0sEdge[0].second;
				for(int i = 1; i < distanceV0sEdge.size(); i++) {
					auto pairi = distanceV0sEdge[i];
					auto v0i = pairi.first;
					auto distance = pairi.second;
					float df = 	distance - running_mean; 
					if(fabs(distance - running_mean) < ledThreshold/mesh_scale) {
						running_mean = (running_mean*i + distance) / (i+1);
						bv0s.push_back(v0i);
					}
					else {
						// printf("giving up from vertex : %d distance to line : %f\n", v0i, distance*mesh_scale);
						break; 
					}
					// printf("vertex : %d distance to line : %f\n", v0i, distance*mesh_scale);
				}
			}

			vector<int> v1s = worldFaceLEDPos[f1]; // all vertices in face0
			vector<pair<int, float>> distanceV1sEdge = {};
			for(auto v1i : v1s) {
				float distance = getDistancePointToLine(led_points[v1i], mesh_vertices[shared_v0].pos, edge_dierction);				
				distanceV1sEdge.push_back(make_pair(v1i, distance));
			}

			std::sort(distanceV1sEdge.begin(), distanceV1sEdge.end(), [](auto &left, auto &right) { return left.second < right.second; });
			if(distanceV1sEdge.size() > 0) {
				bv1s = {distanceV1sEdge[0].first};
				running_mean = distanceV1sEdge[0].second;
				for(int i = 1; i < distanceV1sEdge.size(); i++) {
					auto pairi = distanceV1sEdge[i];
					auto v1i = pairi.first;
					auto distance = pairi.second;
					float df = 	distance - running_mean; 
					if(fabs(distance - running_mean) < ledThreshold/mesh_scale) {
						running_mean = (running_mean*i + distance) / (i+1);
						bv1s.push_back(v1i);
					}
					else {
						// printf("giving up from vertex : %d distance to line : %f\n", v1i, distance*mesh_scale);
						break; 
					}
					// printf("vertex : %d distance to line : %f\n", v1i, distance*mesh_scale);
				}
			}

			// now i have boundary vertices // bv0 and bv1.. sort them in order of distance from sv0
			vector<pair<int, float>> bv0distancepair, bv1distancepair;
			for(auto bv0 : bv0s) {
				float distance = getNorm(led_points[bv0] - mesh_vertices[shared_v0].pos);
				bv0distancepair.push_back(make_pair(bv0, distance));
			}
			for(auto bv1 : bv1s) {
				float distance = getNorm(led_points[bv1] - mesh_vertices[shared_v0].pos);
				bv1distancepair.push_back(make_pair(bv1, distance));
			}

			CDT::Triangulation<double> cdt_nbr;
			vector<CDT::V2d<double>> points_nbr;
			v3f edge_normal = (getNormal(f0) + getNormal(f1)) / 2.0f;
			edge_normal = edge_normal / getNorm(edge_normal);
			// so i have the normal and the points on the plane shared_v0 and shared_v1. Now i have to project all the points in this plane
			// and tranpose this plane to z = 0 plane and all the projected points in it. Take the x and y coordinate and perform the delaunay on them. 
			// then delete the edges that are on the same side. // time complexity --> O(n log(n)) n --> total number of points + O(E)

			// create a cvt out of these two liens and create a nice triangulation. 
			float rot_angle = acos(dot(edge_normal,zaxis));
			v3f rot_axis = cross(zaxis, edge_normal);
			// now project these points on the plane with edge_normal and then apply [-rot_angle] with rot_axis
			m4x4f rotationMatrix = getRotationMatrix(rot_angle, rot_axis);
			map<int, int> delaunay2Led; 
			for(auto v0i: bv0s) {
				v3f pv0i = projectPointOnPlane(led_points[v0i], edge_normal, mesh_vertices[shared_v0].pos);
				v3f rpv0i = applyRotationMatrix(rotationMatrix, pv0i);
				// printf("projected point should be on zplane : %f %f %f\n", rpv0i[0], rpv0i[1], rpv0i[2]); 
				// check that the z coordiante is constant. if yes then ignore it as it it is just offseted
				points_nbr.push_back(CDT::V2d<double>::make(rpv0i[0], rpv0i[1]));
				delaunay2Led[points_nbr.size()-1] = v0i;
			}
			for(auto v1i: bv1s) {
				v3f pv1i = projectPointOnPlane(led_points[v1i], edge_normal, mesh_vertices[shared_v0].pos);
				v3f rpv1i = applyRotationMatrix(rotationMatrix, pv1i);
				// printf("projected point should be on zplane : %f %f %f\n", rpv1i[0], rpv1i[1], rpv1i[2]);
				points_nbr.push_back(CDT::V2d<double>::make(rpv1i[0], rpv1i[1]));
				delaunay2Led[points_nbr.size()-1] = v1i;
			}
			cdt_nbr.insertVertices(points_nbr);
			cdt_nbr.eraseSuperTriangle();
			std::vector<CDT::Triangle> triangles_nbr = cdt_nbr.triangles;
			map<int, int> vertex2side; // vertex to the side of the bi-partite graph that it resides in
			for(auto bv0: bv0s) {vertex2side[bv0] = 0; }
			for(auto bv1: bv1s) {vertex2side[bv1] = 1; }

			for(auto triangles : triangles_nbr) {
				int v0 = delaunay2Led[triangles.vertices[0]], v1 = delaunay2Led[triangles.vertices[1]], v2 = delaunay2Led[triangles.vertices[2]];
				if(vertex2side[v0] != vertex2side[v1]) {
					nedgesMeshNeighbour.push_back(make_pair(v0, v1));
				}
				if(vertex2side[v1] != vertex2side[v2]) {
					nedgesMeshNeighbour.push_back(make_pair(v1, v2));
				}				
				if(vertex2side[v2] != vertex2side[v0]) {
					nedgesMeshNeighbour.push_back(make_pair(v2, v0));
				}				
			}

			// put all of them in the nedgesMeshNeighbour // old stuff
			// for(auto bv0 : bv0s) { for(auto bv1 : bv1s) { auto pairi = make_pair(bv0, bv1); nedgesMeshNeighbour.push_back(pairi); } }

			// float mindistance = std::numeric_limits<double>::max();
			// pair<uint, uint> bestPair;
			// for(auto v0 : v0s) {
			// 	for(auto v1 : v1s) {
			// 		float distance = euclideanDistance(led_points[v0] , led_points[v1]);
			// 		if(distance < mindistance) {
			// 			mindistance = distance;
			// 			bestPair = make_pair(v0, v1);
			// 		}
			// 	}
			// }
			// // printf("best pair for f0 : %d f1: %d is : %d %d min distance : %f\n", f0, f1, bestPair.first, bestPair.second, mindistance);
			// // nedgesMeshNeighbour.push_back(bestPair);
			
			
			faceExistencePair[face_pair] = true;
		}		
	}

}

double lengthV(v3f v0, v3f v1) {
	return sqrt(dot(v0-v1, v0-v1));
}
double lengthV(v2f v0, v2f v1) {
	return sqrt(dot(v0-v1, v0-v1));
}
double lengthV(v2d v0, v2d v1) {
	return sqrt(dot(v0-v1, v0-v1));
}

vector<int> permOrder(vector<v3f> wvs, vector<v2f> fvs) {
	v3f ew0 = normalize(wvs[1] - wvs[0]);
	v3f ew1 = normalize(wvs[2] - wvs[1]);
	v3f ew2 = normalize(wvs[0] - wvs[2]);

	vector<v3f> ews = {ew0, ew1, ew2};

	v2f rw0 = normalize(fvs[1] - fvs[0]); v3f rn0 = v3f(rw0[0], rw0[1]);
	v2f rw1 = normalize(fvs[2] - fvs[1]); v3f rn1 = v3f(rw1[0], rw1[1]);
	v2f rw2 = normalize(fvs[0] - fvs[2]); v3f rn2 = v3f(rw2[0], rw2[1]);
	vector<v3f> rns = {rn0, rn1, rn2};

	vector<int> order; 
	for(int i =0; i < 3; i++) {
		int j;
		for(j = 0; j < 3; j++) {
			if( fabs(dot(ews[i], rns[j])-1) < 1e-3 ) {
				order.push_back(j);
				break;
			}
		}
		if(j == 3) {
			cout << "something went wrong debug!!!!\n";
		}
	}
	return order;

}

void loadLEDs(std::string file_sht, std::string file_rct) {

	std::ifstream file_in_sht(file_sht);
	if (!file_in_sht.is_open()) {
		std::cerr << "[ERROR] Could not open file " << file_sht << '\n';
		return;
	}
	vector<v3f> sheet_vertices;
	vector<vector<int>> sheet_faces;
	vector<vector<double>> sheet_offsets;
	parseSheet(file_sht, sheet_vertices, sheet_faces, sheet_offsets, sheet_face2Edge);

	auto mesh_triangles = g_Mesh.raw()->triangles();
	auto mesh_vertices = g_Mesh.raw()->vertices();

	std::ifstream file_in_rct(file_rct);
	if (!file_in_rct.is_open()) {
		std::cerr << "[ERROR] Could not open file " << file_rct << '\n';
		return;
	}

	// std::map<int, vector<v3f>> flatFace2bcmap;
	vector<v2f> tpts;
	vector<v2f> rect_centers, lcorners, r1corners, r2corners;

	// create the flatface2worldface mapping
	std::map<int, int> flatface2worldface; 
	if(mapping_old_format) {
		std::string line;
		bool start_mapping = false;
		while (std::getline(file_in_sht, line)) {
			if(not start_mapping and line.substr(0,7) == "# World") {
				start_mapping = true; 
				continue; 
			}
			else if(start_mapping and not (line.substr(0,7) == "# Patch")) {
				// make the mapping here.. 
				// cout << line << endl; 
				string delim = " ";
				vector<int> entries = parseLine2int(line, delim);
				flatface2worldface.insert(std::make_pair(entries[1], entries[0])); 
			}
			else if(start_mapping and (line.substr(0,7) == "# Patch")) {
				start_mapping = false; 
			}
			else {
				continue; 
			}
		}
	}

	// init the worldFaceLEDPos to be empty based on the number of faces
	worldFaceLEDPos.resize(g_Mesh.raw()->numTriangles());
	offsetVerticesPlane.resize(g_Mesh.raw()->numTriangles());
	
	string line; 
	int tri_i = 0;
	int line_num = 0;
	int numLeds = 0;
	while (std::getline(file_in_rct, line)) {
		char fl = line.at(0);
		if(fl == 't') {
			// cout << line << endl;
			vector<double> lvals = parse_doubles(line.substr(2));
			ForIndex(i, 3) {
				tpts.push_back(v2f(lvals[2*i], lvals[2*i+1]));
			}
		}
		else if(fl == 'm') {
			// just add all the rectangle centers for now. Gonna process it in 'e'
			vector<double> lvals = parse_doubles(line.substr(2));
			v2f l1corner = v2f(lvals[0], lvals[1]);
			v2f r1corner = v2f(lvals[2], lvals[3]);

			v2f r2corner, l2corner;

			if(file_format == 0){ // old code for .rct file
				r2corner = v2f(lvals[4], lvals[5]);
				l2corner = l1corner + (r2corner - r1corner); // new change in storing .rct file to be compatible with the router
			}
			else { // new code for .rct file
				l2corner = v2f(lvals[4], lvals[5]);
				r2corner = l2corner + (r1corner - l1corner);
			}

			// v2f rcenter = v2f((lvals[2]+lvals[4])/2.0, (lvals[1]+lvals[3])/2.0);
			v2f rcenter = (l1corner + l2corner + r1corner + r2corner) / 4.0f; 
			rect_centers.push_back(rcenter);

			lcorners.push_back(l1corner);
			r1corners.push_back(r1corner);
			r2corners.push_back(r2corner);
			numLeds++; 
		}
		else if(fl == 'e') {
			// compute the barycenter for each of the points
			v2f t0 = tpts[0], t1 = tpts[1], t2 = tpts[2];

			vector<double> vals;
			if(e_without_triangle_format) {  // old format
				vals.push_back(tri_i);
			}
			else { // new format
				vals = parse_doubles(line.substr(2));
				assert(vals.size() == 1);
			}

			int worldFaceIndex = static_cast<int>(vals.at(0));
			int sheetFaceIndex = static_cast<int>(vals.at(0));

			if(mapping_old_format) {
				worldFaceIndex = flatface2worldface[sheetFaceIndex];
			}

			v3f wv0 = mesh_vertices[mesh_triangles[worldFaceIndex][0]].pos;
			v3f wv1 = mesh_vertices[mesh_triangles[worldFaceIndex][1]].pos;
			v3f wv2 = mesh_vertices[mesh_triangles[worldFaceIndex][2]].pos;
			vector<v3f> wvs = {wv0, wv1, wv2};
			vector<v2f> tvs = {t0, t1, t2};

			// vector<int> pOrder = permOrder(wvs, tvs);
			// wv0 = wvs[pOrder[0]];
			// wv1 = wvs[pOrder[1]];
			// wv2 = wvs[pOrder[2]];

			v3f sv0 = sheet_vertices[sheet_faces[sheetFaceIndex][0]];
			v3f sv1 = sheet_vertices[sheet_faces[sheetFaceIndex][1]];
			v3f sv2 = sheet_vertices[sheet_faces[sheetFaceIndex][2]];


			// invert the sheet_vertices offset to get back the original triangle for the purpose of bc computation
			v2f osv0 = v2f(sv0[0], sv0[1]), osv1 = v2f(sv1[0], sv1[1]), osv2 = v2f(sv2[0], sv2[1]);
			performTriangleOffset(osv0, osv1, osv2, -sheet_offsets[tri_i][0]);
			performTriangleOffset(osv1, osv2, osv0, -sheet_offsets[tri_i][1]);
			performTriangleOffset(osv2, osv0, osv1, -sheet_offsets[tri_i][2]);


			// printf("t0 : %f %f sv0 : %f %f 0sv0 : %f %f\n", t0[0], t0[1], sv0[0], sv0[1], osv0[0], osv0[1]);
			// printf("t1 : %f %f sv1 : %f %f 0sv1 : %f %f\n", t1[0], t1[1], sv1[0], sv1[1], osv1[0], osv1[1]);
			// printf("t2 : %f %f sv2 : %f %f 0sv2 : %f %f\n\n", t2[0], t2[1], sv2[0], sv2[1], osv2[0], osv2[1]);

			// so the mapping is fixed and now 
			// THIS WORKS!!!!!!!!! :) 
			// printf("Triangle : %d\n", tri_i);
			// printf("Edge length world : %f %f %f\n", lengthV(wv0,wv1)*mesh_scale, lengthV(wv1,wv2)*mesh_scale, lengthV(wv2,wv0)*mesh_scale);
			// printf("Edge length flat : %f %f %f\n", lengthV(sv0, sv1), lengthV(sv1, sv2), lengthV(sv2, sv0));
			// printf("Edge length route : %f %f %f\n", lengthV(t0,t1), lengthV(t1,t2), lengthV(t2,t0));
			// printf("Edge length unoffset : %f %f %f\n", lengthV(osv0, osv1), lengthV(osv1, osv2), lengthV(osv2, osv0));
			// printf("Sheet offsets : %f %f %f\n\n", sheet_offsets[tri_i][0], sheet_offsets[tri_i][1], sheet_offsets[tri_i][2]);

			// compute the world coordinates of offset triangles using bcs
			if(boolDisplyOffsetTriangle and not without_offset_format) {
				v2f oosv0 = v2f(sv0[0], sv0[1]), oosv1 = v2f(sv1[0], sv1[1]), oosv2 = v2f(sv2[0], sv2[1]);
				v3f bcs0 = computeBC(osv0, osv1, osv2, oosv0); 
				v3f bcs1 = computeBC(osv0, osv1, osv2, oosv1); 
				v3f bcs2 = computeBC(osv0, osv1, osv2, oosv2);

				v3f owv0 = wv0 * bcs0[0] + wv1 * bcs0[1] + wv2 * bcs0[2];
				v3f owv1 = wv0 * bcs1[0] + wv1 * bcs1[1] + wv2 * bcs1[2];
				v3f owv2 = wv0 * bcs2[0] + wv1 * bcs2[1] + wv2 * bcs2[2];
				offsetVerticesPlane[worldFaceIndex] = vector<v3f>{owv0, owv1, owv2};
			}

			vector<v3f> ledPos; 
			vector<int> ledPosIndex; 
			// compute the led points in the world using bc coordinates
			CDT::Triangulation<double> cdt;
			vector<CDT::V2d<double>> points;
			vector<v3f> ledVerticesTriangle; 
			v3f normalT = normalize(cross(wv0-wv1, wv1-wv2)); 
			normalPerTriangle.push_back(normalT);
			int previous_indexing = led_points.size();
			for(int ri = 0; ri < rect_centers.size(); ri++) {
				auto rct = rect_centers[ri];
				v3f bcs; 
				if(mapping_old_format) // only valid for icosa model
					bcs = computeBC(t0, t1, t2, rct);
				else
					bcs = computeBC(osv0, osv1, osv2, rct); // compute it with offseted triangles
				// printf("tri_i : %d ri : %d\n", tri_i, ri);
				// printf("bcs : %f %f %f\n", bcs[0], bcs[1], bcs[2]);

				// now compute the led pos in world coord
				v3f wlp = wv0*bcs[0] + wv1*bcs[1] + wv2*bcs[2];
				v2f tlp = t0*bcs[0] + t1*bcs[1] + t2*bcs[2];
				ledPosIndex.push_back(led_points.size());
				// ledPos.push_back(wlp);
				ledVerticesTriangle.push_back(wlp);
				led_points.push_back(wlp);

				v2f lct = lcorners[ri];
				v3f lbcs, r1bcs, r2bcs;
				if (mapping_old_format) 
					lbcs = computeBC(t0, t1, t2, lct);
				else
					lbcs = computeBC(osv0, osv1, osv2, lct);
				v3f cp0 = wv0*lbcs[0] + wv1*lbcs[1] + wv2*lbcs[2];

				v2f r1ct = r1corners[ri]; 
				if (mapping_old_format) 
					r1bcs = computeBC(t0, t1, t2, r1ct);
				else
					r1bcs = computeBC(osv0, osv1, osv2, r1ct);
				v3f cp1 = wv0*r1bcs[0] + wv1*r1bcs[1] + wv2*r1bcs[2];

				v2f r2ct = r2corners[ri]; 
				if(mapping_old_format) 
					r2bcs = computeBC(t0, t1, t2, r2ct);
				else
					r2bcs = computeBC(osv0, osv1, osv2, r2ct);
				v3f cp2 = wv0*r2bcs[0] + wv1*r2bcs[1] + wv2*r2bcs[2];
				v3f cp3 = cp0 + (cp2-cp1);

				points.push_back(CDT::V2d<double>::make(tlp[0], tlp[1]));
				led_points_extra.push_back(vector<v3f>{cp0, cp1, cp2, cp3});
				// printf("length : %f %f\n", sqrt(dot(cp0 - cp1, cp0 - cp1)), sqrt(dot(cp1 - cp2, cp1 - cp2)));

				// add world Face to the ledVertex2WorldFace
				ledVertex2WorldFace.push_back(worldFaceIndex);
			}
			ledIndicesPerFace[worldFaceIndex] = {};
			for(int i = previous_indexing; i < led_points.size(); i++) {
				ledIndicesPerFace[worldFaceIndex].push_back(i);
			}

			cdt.insertVertices(points);
			cdt.eraseSuperTriangle();
			std::vector<CDT::Triangle> triangles = cdt.triangles;

			vector<pair<uint, uint>> edgeinTriangle;
			for (auto tri : triangles) {
				std::array<unsigned int, 3> vindices = tri.vertices;
				std::sort(vindices.begin(), vindices.end());

				uint v0 = vindices[0], v1 = vindices[1], v2 = vindices[2];
				v2f pos0 = rect_centers[v0], pos1 = rect_centers[v1], pos2 = rect_centers[v2];
				double theta0, theta1, theta2;
				theta0 = acos(dot(normalize(pos1 - pos0), normalize(pos2 - pos0)));
				theta1 = acos(dot(normalize(pos0 - pos1), normalize(pos2 - pos1)));
				theta2 = acos(dot(normalize(pos0 - pos2), normalize(pos1 - pos2)));
				// printf("%d %d %d %f %f %f\n", v0, v1, v2, theta0, theta1, theta2);
				// printf("v0: %f %f v1: %f %f v2: %f %f\n", pos0[0], pos0[1], pos1[0], pos1[1], pos2[0], pos2[1]);
				// printf("l0: %f %f l1 : %f %f\n", normalize(pos1 - pos0)[0], normalize(pos1 - pos0)[1], normalize(pos2 - pos0)[0], normalize(pos2 - pos0)[1]);

				pair<uint, uint> e0 = std::make_pair(v0, v1), e1 = std::make_pair(v1, v2), e2 = std::make_pair(v0, v2);
				// don't add edgeinTriangle that are longer edgeinTriangle created as part of obtuse angle
				if ((std::find(edgeinTriangle.begin(), edgeinTriangle.end(), e0) == edgeinTriangle.end()) and (theta2 < M_PI_2 * 1.33) and (theta2 > 0.1 * M_PI_2 )) {
					edgeinTriangle.push_back(e0);
					led_edges.push_back(std::make_pair(v0 + previous_indexing, v1 + previous_indexing));
					// printf("theta2 : %f ",theta2);
				}
				if ((std::find(edgeinTriangle.begin(), edgeinTriangle.end(), e1) == edgeinTriangle.end()) and (theta0 < M_PI_2 * 1.33) and (theta0 > 0.1 * M_PI_2 )) {
					edgeinTriangle.push_back(e1);
					led_edges.push_back(std::make_pair(v1 + previous_indexing, v2 + previous_indexing));
					// printf("theta0 : %f ", theta0);
				}
				if ((std::find(edgeinTriangle.begin(), edgeinTriangle.end(), e2) == edgeinTriangle.end()) and (theta1 < M_PI_2 * 1.33) and (theta1 > 0.1 * M_PI_2 )) {
					edgeinTriangle.push_back(e2);
					led_edges.push_back(std::make_pair(v0 + previous_indexing, v2 + previous_indexing));
					// printf("theta1 : %f ", theta1);
				}
			}
			vector<double> edgeLengths; 
			for(auto e: edgeinTriangle) {
				v2f diff = rect_centers[e.first] - rect_centers[e.second] ;
				double length = sqrt(dot(diff, diff));
				edgeLengths.push_back(length);
			}

			edgeLengthsPerTriangle.push_back(edgeLengths);
			edgesPerTriangle.push_back(edgeinTriangle);
			ledVerticesPerTriangle.push_back(ledVerticesTriangle);

			randomColors.resize(led_points.size());
			for(auto &r: randomColors) {
				r = v3f(rand() / double(RAND_MAX), rand() / double(RAND_MAX), rand() / double(RAND_MAX));
			}

			// printf("For triangle : %d \n", tri_i);

			// cout << endl; 

			worldFaceLEDPos[worldFaceIndex] = ledPosIndex;
			// worldFaceLEDPos[worldFaceIndex] = ledPos;

			tri_i++;
			tpts.clear();
			rect_centers.clear();
			lcorners.clear(); r1corners.clear(); r2corners.clear();
		}
		else {
			line_num++; 
			continue;
		}
		line_num++; 
	}


	// i hav all the points stored in 
	// now i have to run a nearest neighbour search and calculate the edge length for all the pairs of vertices which are closest to their neighbours
	performNearestNeighbourAnalysis();
	printf("Total num LEDs : %d\n", numLeds);
}

void mainKeyboard(uchar k) {
	static float speed = 1.0f;
	static char last = ' ';

	if (k == 'q') {
		TrackballUI::exit();
	
	}
	else if (k == 'c') {
		boolDisplyCVT = !boolDisplyCVT;
	}
	else if (k == 'v') {
		boolDisplyNN = !boolDisplyNN;
	}
	else if (k == 'l') {
		string file_led = file_rct.substr(0, file_rct.size()-3) + "led";
		cout << file_led << endl;
		saveLedInfo(file_led);
	}
	else if (k == 't') {
		boolDisplyTriangle = !boolDisplyTriangle;
	}	
	
	else if (k == 'o') {
		boolDisplyOffsetTriangle = !boolDisplyOffsetTriangle;
	}

	else if (k == 'n') {
		boolUseNormalOffset = !boolUseNormalOffset;
	}

	else if (k == 'p') {
		boolDisplyPoints = !boolDisplyPoints;
	}

	else if (k == 'e') {
		boolVisualizeRects = !boolVisualizeRects;
	}

	else if (k == ' ') {
		static bool swap = true;
		if (swap) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		}
		else {
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		}
		swap = !swap;
	}
}

int main(int argc, char **argv) {
	Settings stgs;
	if(argc == 1) {
		file_off = "/home/mbhargav/Desktop/phd/electronics/pcbend/data/meshes/icosa-020.off";
		file_sht = "/home/mbhargav/Desktop/phd/electronics/pcbend/data/results/icosa-020_big-dihedral/icosa-020.sheet";
		file_rct = "/home/mbhargav/Desktop/phd/electronics/pcbend/data/results/icosa-020_big-dihedral/faces/icosa-020.fis";
		file_led = "/home/mbhargav/Desktop/phd/electronics/pcbend/data/results/icosa-020_big-dihedral/icosa-020.led";

		string file_module = stgs.file_module;
		use_cli_mode = false;

		// won't generalize to more modules
		if(file_module.find("5050") != string::npos) { 
			useBigLed = true;
		} else {
			useBigLed = false;
		}

		// set this two values manually if using old files
		file_format = 1;
		e_without_triangle_format = false;
		without_offset_format = false;
		mapping_old_format = false;	
	}
    else if(argc == 6) {
		file_stg = argv[1];
		file_off = argv[2];
		file_sht = argv[3];
		file_rct = argv[4];
		file_led = argv[5];

		stgs = read_settings(file_stg);
		string file_module = stgs.file_module;
		use_cli_mode = stgs.use_cli_interface_ledifier;

		// won't generalize to more modules
		if(file_module.find("5050") != string::npos) { 
			useBigLed = true;
		} else {
			useBigLed = false;
		}

		// set this two values manually if using old files
		file_format = 1;
		e_without_triangle_format = false;
		without_offset_format = false;
		mapping_old_format = false;

	}
    else {
        printf("./ledifier <inp:.cfg> <inp:.off> <inp:.sheet> <inp:global.fis> (<out:.led>)\n"); 
		exit(1);
	}

	if(useBigLed) {
		generateSphere(2.5 / mesh_scale, sphere_points, sphere_faces);
	}
	else {
		generateSphere(0.75 / mesh_scale, sphere_points, sphere_faces);
	}

	try {

        TrackballUI::onRender = mainRender;
        TrackballUI::onKeyPressed = mainKeyboard;
        TrackballUI::onReshape = mainReshape;
		TrackballUI::onMouseWheel = mainMouseWheel;

		TrackballUI::init(g_W, g_H, "Layout Visualizer");

		loadMesh(file_off); // put thigns in g_Mesh
        loadLEDs(file_sht, file_rct);


		cerr << "[OK]" << endl;
		cerr << sprint("mesh contains %d vertices and %d triangles\n", g_Mesh->numVertices(), g_Mesh->numTriangles());

		cerr << "Creating renderer ";
		// create render with this vertex format
		g_Renderer = AutoPtr<t_Renderer>(new t_Renderer(g_Mesh.raw()));
		cerr << "[OK]" << endl;

		// setup view
		Transform::perspective(LIBSL_PROJECTION_MATRIX, float(M_PI / 4.0), g_W / float(g_H), 0.001f, 100.0f);

		// init trackball viewpoint
		TrackballUI::trackball().rotation() = quatf(0.707f, -0.03159f, -0.00114f, -0.70614f);
		TrackballUI::trackball().translation() = V3F(0.0f, -0.0f, -10.163f);

		if(use_cli_mode) {
			cout << file_led << endl;
			saveLedInfo(file_led);	
			exit(1);
		}

		/// main loop
		TrackballUI::loop();

		/// clean exit
		// shutdown UI
		TrackballUI::shutdown();
    }
    catch (Fatal &e) {
        cerr << "[ERROR] " << e.message() << endl;
    }
}