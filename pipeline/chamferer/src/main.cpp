#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/edge_topology.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <string>
#include <iostream>
#include <map>
#include <igl/MeshBooleanType.h>
#include <igl/rotate_vectors.h>
#define CGAL_KERNEL_NO_ASSERTIONS
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/readOFF.h>
#include <igl/centroid.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/rotate_by_quat.h>
#include "Settings.hpp"
// #include <CGAL/gmpxx.h>

using namespace std;

struct sheet_data { 
	vector<vector<double>> vertices; 
	vector<vector<int>> faces, hinges; 
	std::map<int, double> edge2offset;
	std::map<int, bool> isHalfHinge, isHinge;
	map<int, vector<int>> faceToEdgeMap; 
	vector<vector<double>> face2Offset;
	double drillBitRadius, userFabMargin;
	int numTrianglesFaces, numHingeFaces;
	std::map<int, std::pair<int,int>> edge2facepair;

	void clear() {
		vertices.clear(); 
		faces.clear(); hinges.clear(); 
		edge2offset.clear(); edge2facepair.clear();
		faceToEdgeMap.clear(); 
		face2Offset.clear();
		isHinge.clear();
		isHalfHinge.clear();
		drillBitRadius = 0;
		userFabMargin = 0;
		numTrianglesFaces = 0; numHingeFaces = 0;
	}
};

class CustomPlugin : public igl::opengl::glfw::imgui::ImGuiPlugin {
};

class createUiMenu : public igl::opengl::glfw::imgui::ImGuiMenu
{

public:
	string input_off, input_sheet, output_off;
	igl::opengl::ViewerData mesh_data, sheet_mesh_data, current_mesh_data;
	uint left_view, right_view;
	int selected_view;

	uint input_mesh_id, input_sheet_id, output_mesh_id;
	uint input_mesh_ind, input_sheet_ind, output_mesh_ind;

	bool show_wire_frame = true, bool_show_faces = true, bool_show_vertex_labels = false, bool_show_face_labels = 0; 
	double extrude_depth = 0.6; /*in mm*/ 
	bool already_extruded = false; bool already_removed = false; 
	bool keep_original_mesh = false; 
	sheet_data sheet; 
	bool run_cli_flag = false;
	double track_width_mm = 1.7;
	double inner_diameter_mm = 1.0;
	size_t chamfer_mode = 0;

	// igl::MeshBooleanType boolean_type(igl::MeshBooleanType::MESH_BOOLEAN_TYPE_MINUS); // MESH_BOOLEAN_TYPE_MINUS is the enum for 2
	// igl::MeshBooleanType boolean_type(igl::MESH_BOOLEAN_TYPE_MINUS);

	createUiMenu(string _input_off, string _input_sheet, string _output_off , uint _left_view, uint _right_view, 
				bool _run_cli_flag, double _track_width_mm, double _inner_diameter_mm, size_t _chamfer_mode) {
		input_off = _input_off;
		input_sheet = _input_sheet;
		output_off = _output_off;
		left_view = _left_view; 
		right_view = _right_view;
		run_cli_flag = _run_cli_flag;
		track_width_mm = _track_width_mm;
		inner_diameter_mm = _inner_diameter_mm;
		chamfer_mode = _chamfer_mode;
	}
	
	void setViewer(igl::opengl::glfw::Viewer *_viewer) {
		viewer = _viewer;
	}

	virtual void draw_viewer_window() override {
		float menu_width = 180.f * menu_scaling();
		ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f), ImVec2(menu_width, -1.0f));
		bool _viewer_menu_visible = true;
		ImGui::Begin(
			"Mesh view", &_viewer_menu_visible,
			ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize);
		ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);
		draw_custom_viewer_menu();
		ImGui::PopItemWidth();
		ImGui::End();
	}

	void reset() { 
		
		mesh_data.clear();
		current_mesh_data.clear();
		sheet_mesh_data.clear();

		if(input_off.size() != 0) {
			load_mesh(input_off);
		}
		
		if(input_sheet.size() != 0) {
			load_sheet(input_sheet);
		}
		already_extruded = false;
	}

	virtual void init(igl::opengl::glfw::Viewer *_viewer, igl::opengl::glfw::imgui::ImGuiPlugin *_plugin) override {
		viewer = _viewer;
		mesh_data.point_size = 10;
		mesh_data.line_width = 0.1;

		sheet_mesh_data.point_size = 10;
		sheet_mesh_data.line_width = 0.1;

		current_mesh_data.point_size = 10;
		current_mesh_data.line_width = 0.1;		
		selected_view = -1;

		input_mesh_id = -1; input_sheet_id = -1; output_mesh_id = -1;
		input_mesh_ind = -1; input_sheet_ind = -1; output_mesh_ind = -1;

		cout << input_off << endl;
		if(input_off.size() != 0) {
			load_mesh(input_off);
		}
		
		if(input_sheet.size() != 0) {
			load_sheet(input_sheet);
		}
		ImGuiMenu::init(_viewer, _plugin);

		if(run_cli_flag) {
			if(chamfer_mode == 0) {
				chamfering();
			}
			else if(chamfer_mode == 1) {
				booleanRemove();
			}
			else {
				printf("Method not implemented choose chamfer_mode 0 or 1\n");
				exit(1);
			}
			saveMesh(output_off, current_mesh_data);
			printf("File saved: "); cout << output_off << endl;
			exit(1);
		}
	}

	virtual bool mouse_down(int button, int modifier) override {
		return false;
	}

	virtual bool key_pressed(unsigned int key, int modifiers) {
		if (key == 'Q' or key == 'q') {
			exit(1);
		}
		if (key == 'G' or key == 'g') { // 'F' for some reason F is reserved for changing shader code in viewport 2. 
			bool_show_face_labels = !bool_show_face_labels;
			if(bool_show_face_labels)
				printf("show face indices on!\n");
			else	
				printf("show face indices off!\n");
		}
		if (key == 'E' or key == 'e') {
			if(!already_extruded) {
				extrudeMesh(extrude_depth); 
				already_extruded = true; 
			}
		}

		if (key == 'T' or key == 't') {
			booleanRemove();
			// if(already_extruded and not already_removed) {
			// 	booleanRemove();
			// 	already_extruded = true;
			// }
		}

		if (key == 'C' or key == 'c') {
			// perform the marco method remove
			chamfering();
			// if(already_extruded and not already_removed) {
			// 	booleanRemove();
			// 	already_extruded = true;
			// }
		}

		if (key == 'r' or key == 'R') {
			reset(); 
		}


		if (key == 'V' or key == 'v') {
			bool_show_vertex_labels = !bool_show_vertex_labels;
			if(bool_show_vertex_labels)
				printf("show vertex indices on!\n");
			else	
				printf("show vertex indices off!\n");
		}
		return false;
	}

	std::string dtos(double x) {
		std::stringstream s;  // Allocates memory on stack
		s << x;
		return s.str();       // returns a s.str() as a string by value
							// Frees allocated memory of s
	} 

	void swapInEigenVector(Eigen::Vector3i &vec, int gi, int ri) {
		int v0 = vec[0], v1 = vec[1], v2 = vec[2];
		if(v0 == gi) {
			vec = Eigen::Vector3i(ri, v1, v2); 
		}
		else if(v1 == gi) {
			vec = Eigen::Vector3i(v0, ri, v2); 
		}
		else if(v2 == gi) {
			vec = Eigen::Vector3i(v0, v1, ri); 
		}
		else {
			printf("something went wrong in swapping values debug!!\n"); 
		}
	}

	Eigen::Vector3d rotateVectorByAxisAndAngle(Eigen::Vector3d vec, Eigen::Vector3d axis, double angle) {
		double sin_t = sin(angle);
        double cos_t = cos(angle);

		 
		double kx = axis(0), ky = axis(1), kz = axis(2); 
		Eigen::Matrix3d rotation_matrix = (Eigen::Matrix3d() <<
			cos_t + kx*kx*(1-cos_t), kx*ky*(1-cos_t) - kz*sin_t, kx*kz*(1-cos_t) + ky*sin_t,
			kx*ky*(1-cos_t) + kz*sin_t, cos_t + ky*ky*(1-cos_t), ky*kz*(1-cos_t) - kx*sin_t,
			kx*kz*(1-cos_t) - ky*sin_t, ky*kz*(1-cos_t) + kx*sin_t, cos_t + kz*kz*(1-cos_t)).finished();
		
        // rotating a vector by axis and angle
		// double qrot[4], qres[4];
		// igl::axis_angle_to_quat(axis,angle,qrot);
		// igl::rotate_by_quat(vec, qrot); 
        Eigen::Vector3d rvec = rotation_matrix * vec; 
        // Eigen::Vector3d rvec = cos_t*vec + sin_t*(axis.cross(vec)) + (1-cos_t)*(axis.dot(vec))*vec;
		return rvec;
	}



	void performTriangleOffset(Eigen::Vector3d vpos0, Eigen::Vector3d &vpos1, Eigen::Vector3d &vpos2, double offset) {
		Eigen::Vector3d v12 = vpos2 - vpos1, v12n = v12 / v12.norm();
		Eigen::Vector3d v01 = vpos0 - vpos1, v01n = v01 / v01.norm();
		Eigen::Vector3d v02 = vpos0 - vpos2, v02n = v02 / v02.norm();
		Eigen::Vector3d perp = v01 - v01.dot(v12n) * v12n;
		Eigen::Vector3d perp_u = perp / perp.norm();

		Eigen::Vector3d v01_d = v01n * (offset / (v01n.dot(perp_u)));
		Eigen::Vector3d v02_d = v02n * (offset / (v02n.dot(perp_u)));

		// update v1...
		vpos1 = vpos1 + v01_d;
		vpos2 = vpos2 + v02_d;
	}

	Eigen::Matrix3d createRotationMatrix(Eigen::Vector3d fromV, Eigen::Vector3d toV) {
		// https://www.theochem.ru.nl/~pwormer/Knowino/knowino.org/wiki/Rotation_matrix.html#Vector_rotation
		Eigen::Vector3d u = fromV.cross(toV); 
		double s = u.norm(); 
		double c = fromV.dot(toV); 
		double h = (1-c)/(s*s);

		Eigen::Matrix3d R;
		R << c+h*u(0)*u(0), h*u(0)*u(1)-u(2), h*u(0)*u(2)+u(1), 
			 h*u(0)*u(1)+u(2), c+h*u(1)*u(1), h*u(1)*u(2)-u(0), 
			 h*u(0)*u(2)-u(1), h*u(1)*u(2)+u(0), c+h*u(2)*u(2);

		return R; 
	}

	int getOppositeVertex(int v0, int v1, int f, const Eigen::MatrixXi &F) {
		int fv0 = F.row(f)[0], fv1 = F.row(f)[1], fv2 = F.row(f)[2];
		if(fv0 != v0 and fv0 != v1)
			return fv0;
		if(fv1 != v0 and fv1 != v1)
			return fv1;
		if(fv2 != v0 and fv2 != v1)
			return fv2;
		printf("Something went wrong DEBUG!!\n");
		return -1;
	}

	void getCommonVertex(int f0v0, int f0v1, int f0v2, int f1v0, int f1v1, int f1v2, int &shared_v0, int &shared_v1) {
		vector<int> Avec{f0v0, f0v1, f0v2};
		vector<int> Bvec{f1v0, f1v1, f1v2};
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

	void chamfering() { 
		cout << "chamfering\n"; 
		Eigen::MatrixXd VA = current_mesh_data.V; 		
		Eigen::MatrixXi FA = current_mesh_data.F;
		// Eigen::MatrixXd VB; Eigen::MatrixXi FB;

		// VB = igl::rotate_vectors(VB, angle, Eigen::Vector3d(1,0,0), Eigen::Vector3d(0,1,0));
		Eigen::MatrixXd sV = sheet_mesh_data.V;
		Eigen::MatrixXi sF = sheet_mesh_data.F;

		Eigen::MatrixXd gV = mesh_data.V;
		Eigen::MatrixXi gF = mesh_data.F;
		Eigen::MatrixXd gN = mesh_data.F_normals;
		// Eigen::MatrixXi gEV, gFE, gEF; // variables for edge_topology function
		// igl::edge_topology(gV, gF, gEV, gFE, gEF); // this will not work for open meshes as the original mesh is hacked up by adding offset so topology is changed.
		
		// perform the boolean operation per face. 
		// thicken the mesh with blender (only rim option with 1mm) and run this code and apply thicken again by 1mm (without rim)
		std::vector<std::vector<Eigen::Vector3d>> auxPoints;
		printf("Number of edges: %ld\n", sheet.edge2facepair.size());
		// printf("Number of edges: %ld\n", gEF.rows());
		// for(int ei=0; ei < gEF.rows(); ei++) {
		for(auto edgeMapEntry: sheet.edge2facepair) {
			int ei = edgeMapEntry.first;
			int f0 = sheet.edge2facepair[ei].first, f1 = sheet.edge2facepair[ei].second; 
			// int f0 = gEF.row(ei)[0], f1 = gEF.row(ei)[1];
			// int v0 = gEV.row(ei)[0], v1 = gEV.row(ei)[1];

			int f0v0 = gF.row(f0)[0], f0v1 = gF.row(f0)[1], f0v2 = gF.row(f0)[2];
			int f1v0 = gF.row(f1)[0], f1v1 = gF.row(f1)[1], f1v2 = gF.row(f1)[2];

			int v0 = -1, v1 = -1; 
			getCommonVertex(f0v0, f0v1, f0v2, f1v0, f1v1, f1v2, v0, v1);

			int ov0 = getOppositeVertex(v0, v1, f0, gF);
			int ov1 = getOppositeVertex(v0, v1, f1, gF);
			Eigen::Vector3d oppositeVertexdir = gV.row(ov1) - gV.row(ov0);
			oppositeVertexdir = oppositeVertexdir / oppositeVertexdir.norm();
			Eigen::Vector3d Vn0 = gN.row(f0), Vn1 = gN.row(f1);
			Vn0 = Vn0 / Vn0.norm(); 
			Vn1 = Vn1 / Vn1.norm(); 
			double angle = acos(Vn0.dot(Vn1)); 
			double dotP   = oppositeVertexdir.dot(Vn0);
			// if(dotP > 0) {
			// 	printf("edge ei: %d f0: %d f1: %d angle : %f dot: %f is concave\n", ei, f0, f1, angle*180.0/M_PI, dotP);
			// }
			if(dotP <= 0) {
				printf("edge ei: %d f0: %d f1: %d angle : %f dot: %f is convex\n", ei, f0, f1, angle*180.0/M_PI, dotP);
				// the edge is convex and thus should be chamfered!
				double wr = 0;
				bool chamferIt = false;
				if(sheet.isHalfHinge[ei] and sheet.isHinge[ei]) {
					wr = track_width_mm; chamferIt = true;
				}
				else if(not sheet.isHalfHinge[ei] and sheet.isHinge[ei]) {
					wr = track_width_mm * 2 + inner_diameter_mm; chamferIt = true;
				}
				else {
					// it was just a cut edge don't need to do anything
				}
				if(chamferIt) {
					// variables from paper figure 10
					double theta = M_PI - angle; 
					double dtheta = wr / (2*sin(theta/2));
					// add triangles vertices v0, v0 + dtheta*(ov0-v0)N, v0 + dtheta*(ov1-v0)N
					Eigen::Vector3d dir0 = gV.row(ov0) - gV.row(v0); dir0 = dir0 / dir0.norm(); 
					Eigen::Vector3d dir1 = gV.row(ov1) - gV.row(v0); dir1 = dir1 / dir1.norm(); 
					Eigen::Vector3d dir2 = gV.row(ov0) - gV.row(v1); dir2 = dir2 / dir2.norm(); 
					Eigen::Vector3d dir3 = gV.row(ov1) - gV.row(v1); dir3 = dir3 / dir3.norm(); 
					Eigen::Vector3d t0v0 = gV.row(v0) + dir0.transpose()*dtheta;
					Eigen::Vector3d t0v1 = gV.row(v0) + dir1.transpose()*dtheta;
					Eigen::Vector3d t1v0 = gV.row(v1) + dir2.transpose()*dtheta;
					Eigen::Vector3d t1v1 = gV.row(v1) + dir3.transpose()*dtheta;
					std::vector<Eigen::Vector3d> trapPoints = {gV.row(v0), t0v0, t0v1, gV.row(v1), t1v0, t1v1};
					auxPoints.push_back(trapPoints);
					// make a trapezium and add it to the geometry of VB
				}
			}
		}
		Eigen::MatrixXd VB; 
		Eigen::MatrixXi FB;
		VB.resize(auxPoints.size()*6 , 3); // 2 per element timesfor the bottom row as well + 4 for extrude operation as well.
		FB.resize(auxPoints.size()*8 , 3); // 2 per element timesfor the bottom row as well + 4 for extrude operation as well.
		printf("Number of chamfer edges : %ld\n", auxPoints.size());
		for(int i = 0; i < auxPoints.size(); i++) {
			vector<Eigen::Vector3d> trapPoints = auxPoints[i];
			for(int j=0; j < trapPoints.size(); j++) {
				VB.row(6*i + j) = trapPoints[j];
			}

			FB.row(8*i)   = Eigen::Vector3i(6*i+0, 6*i+1, 6*i+2);
			FB.row(8*i+1) = Eigen::Vector3i(6*i+3, 6*i+5, 6*i+4);

			FB.row(8*i+2) = Eigen::Vector3i(6*i+0, 6*i+4, 6*i+1);
			FB.row(8*i+3) = Eigen::Vector3i(6*i+0, 6*i+3, 6*i+4);

			FB.row(8*i+4) = Eigen::Vector3i(6*i+0, 6*i+2, 6*i+5);
			FB.row(8*i+5) = Eigen::Vector3i(6*i+0, 6*i+5, 6*i+3);

			FB.row(8*i+6) = Eigen::Vector3i(6*i+1, 6*i+5, 6*i+2);
			FB.row(8*i+7) = Eigen::Vector3i(6*i+1, 6*i+4, 6*i+5);

		}
		current_mesh_data.clear();
		Eigen::MatrixXd VC; Eigen::MatrixXi FC;
		Eigen::VectorXi J;
		// igl::MeshBooleanType boolean_type  = igl::MeshBooleanType::MESH_BOOLEAN_TYPE_UNION; // remove it from the original mesh
		igl::MeshBooleanType boolean_type  = igl::MeshBooleanType::MESH_BOOLEAN_TYPE_MINUS; // remove it from the original mesh
		igl::copyleft::cgal::mesh_boolean(VA,FA,VB,FB,boolean_type ,VC,FC,J);
		VA = VC; FA = FC;

		current_mesh_data.set_mesh(VA, FA);
		// current_mesh_data.set_mesh(VB, FB);
		// current_mesh_data.set_points(VB, Eigen::Vector3d(0.4,0.4,0.4));
		viewer->data_list[2] = current_mesh_data; // the current mesh
		viewer->data_list[0] = mesh_data; 	

	}

	void booleanRemove() {
		cout << "Removing space for hinges\n"; 

		current_mesh_data.clear();
		Eigen::MatrixXd VA = current_mesh_data.V; 		
		Eigen::MatrixXi FA = current_mesh_data.F;
		// Eigen::MatrixXd VB; Eigen::MatrixXi FB;

		// VB = igl::rotate_vectors(VB, angle, Eigen::Vector3d(1,0,0), Eigen::Vector3d(0,1,0));
		Eigen::MatrixXd sV = sheet_mesh_data.V;
		Eigen::MatrixXi sF = sheet_mesh_data.F;

		Eigen::MatrixXd gV = mesh_data.V;
		Eigen::MatrixXi gF = mesh_data.F;
		Eigen::MatrixXd gN = mesh_data.F_normals;
		
		
		// perform the boolean operation per face. 
		// thicken the mesh with blender (only rim option with 1mm) and run this code and apply thicken again by 1mm (without rim)
		for(int fi=0; fi < sheet.numTrianglesFaces; fi++) {
			double ed = extrude_depth; 
			int v0  = sF.row(fi)[0],  v1 = sF.row(fi)[1],  v2 = sF.row(fi)[2]; 
			int gv0 = gF.row(fi)[0], gv1 = gF.row(fi)[1], gv2 = gF.row(fi)[2]; 
			Eigen::Vector3d sv0 = sV.row(v0), gvv0 = gV.row(gv0);
			Eigen::Vector3d sv1 = sV.row(v1), gvv1 = gV.row(gv1);
			Eigen::Vector3d sv2 = sV.row(v2), gvv2 = gV.row(gv2);
			Eigen::Vector3d vn  = Eigen::Vector3d(0,0,1); 

			double offset0 = (sheet.face2Offset[fi][0] > 0) ? sheet.face2Offset[fi][0] : sheet.userFabMargin;
			double offset1 = (sheet.face2Offset[fi][1] > 0) ? sheet.face2Offset[fi][1] : sheet.userFabMargin;
			double offset2 = (sheet.face2Offset[fi][2] > 0) ? sheet.face2Offset[fi][2] : sheet.userFabMargin;
			// performTriangleOffset(sv0, sv1, sv2, -offset0);
			// performTriangleOffset(sv1, sv2, sv0, -offset1);
			// performTriangleOffset(sv2, sv0, sv1, -offset2);

			// STEP 1: compute the normal of the triangle in the original mesh.
			Eigen::Vector3d Vn = gN.row(fi); 
			vn = vn / vn.norm(); 
			Vn = Vn / Vn.norm(); 

			// STEP 2: compute the angle requrire to rotate the triangles. 

			Eigen::Matrix3d Rglobal, Rlocal; 
			Rglobal = createRotationMatrix(vn,Vn);

			// STEP 3: rotate and translate the triangle global and local
			Eigen::Vector3d rv0 = Rglobal*(sv0-sv0) + sv0 + (gvv0-sv0); 
			Eigen::Vector3d rv1 = Rglobal*(sv1-sv0) + sv0 + (gvv0-sv0); 
			Eigen::Vector3d rv2 = Rglobal*(sv2-sv0) + sv0 + (gvv0-sv0); 

			Eigen::Vector3d dir0 = rv1  - rv0;  dir0 = dir0 / dir0.norm();
			Eigen::Vector3d dir1 = gvv1 - gvv0; dir1 = dir1 / dir1.norm();		
			Rlocal = createRotationMatrix(dir0, dir1);

			rv0 = Rlocal * (rv0 - rv0) + rv0; 
			rv1 = Rlocal * (rv1 - rv0) + rv0; 
			rv2 = Rlocal * (rv2 - rv0) + rv0; 		

			Eigen::Vector3d e0, e1, e2;
			e0 = rv1 - rv0; e0 = e0 / e0.norm();
			e1 = rv2 - rv1; e1 = e1 / e1.norm();
			e2 = rv0 - rv2; e2 = e2 / e2.norm();

			// applyOffsetTranslation();
			double costheta = -e2.dot(e0);
			double sintheta = (-e2.cross(e0)).norm();
			double cottheta = costheta / sintheta;
			
			Eigen::Vector3d translationVector = ((offset1 + offset2*costheta)/sintheta)*e0 + offset2*Vn.cross(e0);
			// Eigen::Vector3d translationVector = ((offset2 + offset0*costheta)/sintheta)*e0 + offset0*Vn.cross(e0);
			rv0 = rv0 + translationVector;
			rv1 = rv1 + translationVector;
			rv2 = rv2 + translationVector;

			// STEP 4: Perform extrude operation
			vector<Eigen::Vector3d> rvs = {rv0, rv1, rv2};
			vector<Eigen::Vector3d> edgeDirs = {e0, e1, e2};
			double smallOffset = 0.1;
			vector<double> ofs = {(offset2 > 0.5) ? offset2 : smallOffset, (offset0>0.5) ? offset0 : smallOffset, (offset1>0.5) ? offset1 : smallOffset};

			Eigen::MatrixXd VB; 
			// VB.resize(2*3,3); 
			VB.resize(2*3+4*3 , 3); // 2 per element timesfor the bottom row as well + 4 for extrude operation as well. 
			// The points in VB are arrange as follows: 
			// up plane: 0,1,2 --> e0 square: 0,6,12,1 e1 sqaure: 1,7,13,2 e2 square: 2,8,14,0 [clockwise]
			// down plane: 3,4,5 --> e0 square: 3,9,15,4 e1 sqaure: 4,10,16,5 e2 square: 5,11,17,3 [clockwise]

			// cout << Vn << endl;
			// printf("ed: %f ofs: %f %f %f \n", ed, offset0, offset1, offset2);


			for(int i = 0; i < 2*3 + 4*3; i++) {
				if(i<3) {
					VB.row(i) = rvs[i%3] + ed*Vn; // adding some upper crust to avoid floating point errors
				}
				else if(i < 6){
					VB.row(i) = rvs[i%3] - ed*Vn;
				}
				else if(i < 9){
					VB.row(i) = rvs[i%3] + ed*Vn - Vn.cross(edgeDirs[i%3])*ofs[i%3];
				}
				else if(i < 12){
					VB.row(i) = rvs[i%3] - ed*Vn - Vn.cross(edgeDirs[i%3])*ofs[i%3];
				}
				else if(i < 15){
					VB.row(i) = rvs[(i+1)%3] + ed*Vn - Vn.cross(edgeDirs[i%3])*ofs[i%3];
				}
				else if(i < 18){
					VB.row(i) = rvs[(i+1)%3] - ed*Vn - Vn.cross(edgeDirs[i%3])*ofs[i%3];
				}
			}


			Eigen::MatrixXi FB; FB.resize(2 + 3*5*2,3); // new config with cuboids attached 
			// per edge a cuboid of 5 faces with two triangle per face

			FB.row(0) = Eigen::Vector3i(0,1,2); // bottom original triangle 
			FB.row(1) = Eigen::Vector3i(3,5,4); // top triangle
			
			vector<vector<int>> cuboidConfigs = {{0,6,12,1,3,9,15,4},{1,7,13,2,4,10,16,5},{2,8,14,0,5,11,17,3}};
			
			int i = 2;
			while(i < 2+3*5*2) {
				int ci = (i-2)/10; // cuboid id
				vector<int> cids = cuboidConfigs[ci];
				FB.row(i) = Eigen::Vector3i(cids[0],cids[1],cids[2]); i++;
				FB.row(i) = Eigen::Vector3i(cids[0],cids[2],cids[3]); i++;

				FB.row(i) = Eigen::Vector3i(cids[2],cids[6],cids[3]); i++;
				FB.row(i) = Eigen::Vector3i(cids[3],cids[6],cids[7]); i++;

				FB.row(i) = Eigen::Vector3i(cids[7],cids[6],cids[4]); i++;
				FB.row(i) = Eigen::Vector3i(cids[6],cids[5],cids[4]); i++;

				FB.row(i) = Eigen::Vector3i(cids[1],cids[6],cids[2]); i++;
				FB.row(i) = Eigen::Vector3i(cids[1],cids[5],cids[6]); i++;

				FB.row(i) = Eigen::Vector3i(cids[0],cids[5],cids[1]); i++;
				FB.row(i) = Eigen::Vector3i(cids[0],cids[4],cids[5]); i++;
			}

			Eigen::MatrixXd VC; Eigen::MatrixXi FC;
			Eigen::VectorXi J;
			igl::MeshBooleanType boolean_type  = igl::MeshBooleanType::MESH_BOOLEAN_TYPE_UNION; // patch up all the elements.
			igl::copyleft::cgal::mesh_boolean(VA,FA,VB,FB,boolean_type ,VC,FC,J);
			VA = VC; FA = FC;
			printf("Done for face: %d\n", fi); 
			printf("Current number of vertices: %ld num faces: %ld\n", VA.rows(), FA.rows());
		}
		
		Eigen::MatrixXd VC, VB; Eigen::MatrixXi FC, FB;
		Eigen::VectorXi J;
		igl::MeshBooleanType boolean_type  = igl::MeshBooleanType::MESH_BOOLEAN_TYPE_MINUS; // remove it from the original mesh
		VB = mesh_data.V;
		FB = mesh_data.F;
		igl::copyleft::cgal::mesh_boolean(VB,FB,VA,FA,boolean_type ,VC,FC,J);
		VA = VC; FA = FC;
		current_mesh_data.clear();
		// current_mesh_data.set_mesh(VB, FB);
		current_mesh_data.set_mesh(VA, FA);
		// current_mesh_data.set_points(VB, Eigen::Vector3d(0.4,0.4,0.4));
		viewer->data_list[2] = current_mesh_data; // the current mesh
		viewer->data_list[0] = mesh_data; 	


	}

	void extrudeMesh(double ed) {

		// for extrude a simple scale is enough. 
		Eigen::MatrixXd cV = current_mesh_data.V;
		Eigen::MatrixXi cF = current_mesh_data.F;
		double scale = 1.0; 
		Eigen::Vector3d VBC; igl::centroid(cV, cF, VBC); 
		double avgscale = 0; 
		for(int fi = 0; fi < cF.rows(); fi++) {
			int v0, v1, v2; 
			v0 = cF.row(fi)[0], v1 = cF.row(fi)[1], v2 = cF.row(fi)[2];
			Eigen::Vector3d cv = (cV.row(v0) + cV.row(v0) + cV.row(v0))/3.0; 
			double distance = (cv - VBC).norm(); 
			double tscale = (distance + ed ) / distance; 
			avgscale += tscale / cF.rows();
			// printf("Face fi: %d distance: %f tscale: %f\n", fi, distance, tscale);
		}
		printf("avg scale: %f\n", avgscale);

		for(int vi = 0; vi < cV.rows(); vi++) { 
			Eigen::Vector3d v0 = cV.row(vi); 
			Eigen::Vector3d vnew = (v0 - VBC) * avgscale + VBC; 
			cV.row(vi) = vnew; 
		}


		viewer->data(2).clear(); 
		current_mesh_data.clear();
		current_mesh_data.set_mesh(cV, cF);
		viewer->data_list[2] = current_mesh_data; // the current mesh
		viewer->data_list[0] = mesh_data; 		
	}

	void draw_custom_viewer_menu() {
		// Mesh
		if (ImGui::CollapsingHeader("Mesh", ImGuiTreeNodeFlags_DefaultOpen)) {
			float w = ImGui::GetContentRegionAvail().x;
			float p = ImGui::GetStyle().FramePadding.x;
			if (ImGui::Button("Load##Mesh", ImVec2((w - p) / 2.f, 0))) {
				load_mesh();
			}
			ImGui::SameLine(0, p);
			if (ImGui::Button("Save##Mesh", ImVec2((w - p) / 2.f, 0))) {
				// std::string fname = igl::file_dialog_save();
				std::string fname = output_off;
				saveMesh(fname, current_mesh_data);
				printf("File saved: "); cout << output_off << endl;
				// viewer->open_dialog_save_mesh();
			}
			ImGui::InputDouble("extrude depth (in mm)", &extrude_depth, 0.1, 1, "%.4f");
			ImGui::Checkbox("Keep Original mesh", &keep_original_mesh);
			if (ImGui::Button("Extrude Mesh", ImVec2((w - p) / 2.f, 0))) {
				if(not already_extruded) {
					extrudeMesh(extrude_depth);
					already_extruded = true;
				}
			}
			if (ImGui::Button("Remove Hinge space", ImVec2((w - p) / 2.f, 0))) {
				booleanRemove();
				// if(already_extruded and not already_removed) {
				// 	booleanRemove();
				// 	already_extruded = true;
				// }
			}
			if (ImGui::Button("Remove Hinge space with dihedral", ImVec2((w - p) / 2.f, 0))) {
				chamfering();
				// if(already_extruded and not already_removed) {
				// 	booleanRemove();
				// 	already_extruded = true;
				// }
			}
		} 

		static bool debug_menu = false; 
		ImGui::Checkbox("Debug Menu", &debug_menu);
		if (debug_menu) {
			// Viewing options
			if (ImGui::CollapsingHeader("Viewing Options", ImGuiTreeNodeFlags_DefaultOpen))
			{
				if (ImGui::Button("Center object", ImVec2(-1, 0))) {
					viewer->core(left_view).align_camera_center(viewer->data_list[input_mesh_ind].V, viewer->data_list[input_mesh_ind].F);
					// viewer->core(left_view).align_camera_center(viewer->data_list[input_sheet_ind].V, viewer->data_list[input_sheet_ind].F);
					viewer->core(right_view).align_camera_center(viewer->data_list[output_mesh_ind].V, viewer->data_list[output_mesh_ind].F);
				}
				if (ImGui::Button("Snap canonical view", ImVec2(-1, 0))) {
					viewer->snap_to_canonical_quaternion();
				}
				// Orthographic view
				ImGui::Checkbox("Orthographic view left", &(viewer->core(left_view).orthographic));
				ImGui::Checkbox("Orthographic view right", &(viewer->core(right_view).orthographic));
			}

			// Overlays
			if (ImGui::CollapsingHeader("Overlays", ImGuiTreeNodeFlags_DefaultOpen)) {
				// make_checkbox("Wireframe", viewer->data().show_lines);
				ImGui::Checkbox("Show wireframe", &show_wire_frame); ImGui::SameLine(0, ImGui::GetStyle().FramePadding.x);
				ImGui::Checkbox("Fill", &bool_show_faces); 
			}
		}
		viewer->data_list[input_mesh_ind].set_visible(true, viewer->core_list[0].id);
		// viewer->data_list[input_sheet_ind].set_visible(true, viewer->core_list[0].id);
		viewer->data_list[output_mesh_ind].set_visible(false, viewer->core_list[0].id);

		viewer->data_list[input_mesh_ind].set_visible(false, viewer->core_list[1].id);
		// viewer->data_list[input_sheet_ind].set_visible(false, viewer->core_list[1].id);
		viewer->data_list[output_mesh_ind].set_visible(true, viewer->core_list[1].id);


		for (auto &mesh : viewer->data_list) {
			mesh.show_lines = show_wire_frame ? 63 : 0;
			mesh.show_faces = bool_show_faces ? 63 : 0; // 63 has ones in binary // this triggers swap of shading method in a different viewport
			mesh.show_vertex_labels = bool_show_vertex_labels ? 63 : 0;
			mesh.show_face_labels = bool_show_face_labels ? 63 : 0;
		}

	}
	void load_mesh(string filename = "") {
		// Remember that there is an assert statment in there .. assert(data_list.size() >= 1);
		// but we want to manage only one mesh not more than that. so wheenver it is >1 just clear the previous mesh out
		mesh_data.clear();
		if (filename.size() == 0)
			viewer->open_dialog_load_mesh(); // it will only go to the only defined one
		else
			viewer->load_mesh_from_file(filename.c_str());
		while (viewer->data_list.size() > 1) {
			viewer->erase_mesh(0);
		}
		mesh_data = viewer->data_list[0]; // the current mesh
		viewer->data_list[0] = mesh_data; 

		sheet_mesh_data.clear();
		current_mesh_data.clear();
		input_mesh_id = 0;
		input_sheet_id  = viewer->append_mesh();
		output_mesh_id = viewer->append_mesh();

		input_mesh_ind = viewer->mesh_index(input_mesh_id);
		input_sheet_ind = viewer->mesh_index(input_sheet_id);
		output_mesh_ind  = viewer->mesh_index(output_mesh_id);


		current_mesh_data.set_mesh(mesh_data.V, mesh_data.F);
		viewer->data_list[output_mesh_ind] = current_mesh_data; 

	}

	void load_sheet(string filename = "") {
		sheet_mesh_data.clear();
		if (filename.size() == 0)
			filename = igl::file_dialog_open();

		sheet_mesh_data.clear();
		sheet.clear(); 
		string line;
		int numV = 0, eindex = 0; 
		ifstream myfile (filename);
		if (myfile.is_open())
		{
			while ( getline (myfile,line) ) {
				// cout << line << '\n';
				if((line.substr(0,2).compare(string("si")) == 0)) {
					stringstream ss(line);  
					string word;
					int i = 0; 
					while (ss >> word) { // Extract word from the stream.
						if(i == 1)
							numV = stoi(word);
						if(i == 2)
							sheet.numTrianglesFaces = stoi(word);
						if(i == 3)
							sheet.numHingeFaces = stoi(word);							
						i++;
					}
				}
				if((line.substr(0,2).compare(string("up")) == 0)) {
					stringstream ss(line);  
					string word;
					int i = 0; 
					while (ss >> word) { // Extract word from the stream.
						if(i == 1)
							sheet.drillBitRadius = stoi(word);
						if(i == 2)
							sheet.userFabMargin = stod(word);						
						i++;
					}
				}				
				if((line.substr(0,1).compare(string("v")) == 0)) {
					stringstream ss(line);  
					string word;
					double a, b;
					int i = 0; 
					while (ss >> word) { // Extract word from the stream.
						if(i == 1)
							a = stod(word);
						if(i == 2)
							b = stod(word);
						i++;
					}
					vector<double> v = {a, b};
					// cout << v[0] << " " << v[1] << endl;
					sheet.vertices.push_back(v);
				}
				if((line.substr(0,2).compare(string("f ")) == 0)) {
					stringstream ss(line);  
					string word;
					int a, b, c;
					int i = 0; 
					while (ss >> word) { // Extract word from the stream.
						if(i == 1)
							a = stoi(word);
						if(i == 2)
							b = stoi(word);
						if(i == 3)
							c = stoi(word);

						i++;
					}
					vector<int> f = {a, b, c};
					// cout << f[0] << " " << f[1] << " " << f[2] << endl;
					sheet.faces.push_back(f);
				}
				if((line.substr(0,2).compare(string("h ")) == 0)) {
					stringstream ss(line);  
					string word;
					int a, b, c, d;
					int i = 0; 
					while (ss >> word) { // Extract word from the stream.
						if(i == 1)
							a = stoi(word);
						if(i == 2)
							b = stoi(word);
						if(i == 3)
							c = stoi(word);
						if(i == 4)
							d = stoi(word);
						i++;
					}
					vector<int> h = {a, b, c, d};
					// cout << h[0] << " " << h[1] << " " << h[2] << " " << h[3] << endl;
					sheet.hinges.push_back(h);
				}
				if((line.substr(0,2).compare(string("e ")) == 0)) {
					// # Map Edges to [hingeId Face0 Face1 Offset IsHinge IsHalfHinge IsBoundary]
					stringstream ss(line);  
					string word;
					int hingeId, face0, face1,isHinge, isHalfHinge, isBoundary;
					double offset;
					int i = 0; 
					while (ss >> word) { // Extract word from the stream.
						if(i == 1)
							hingeId = stoi(word);
						if(i == 2)
							face0 = stoi(word);
						if(i == 3)
							face1 = stoi(word);
						if(i == 4)
							offset = stod(word);
						if(i == 5)
							isHinge = stoi(word);							
						if(i == 6)
							isHalfHinge = stoi(word);
						if(i == 7)
							isBoundary = stoi(word);														
						i++;
					}
					vector<int> ei = {hingeId, face0, face1,isHinge, isHalfHinge, isBoundary};
					sheet.edge2offset[eindex] = offset;
					if(hingeId != -1) {
						sheet.edge2facepair[hingeId] = std::make_pair(face0, face1);
					}
					sheet.isHinge[eindex] = isHinge;
					sheet.isHalfHinge[eindex] = isHalfHinge;
					eindex++;
					// cout << ei[0] << " " << ei[1] << " " << ei[2] << " " << ei[3] << " " << ei[4] << " " << ei[5] << " " << offset << endl;
				}	
				
				if((line.substr(0,2).compare(string("fe")) == 0)) {
					// # Map Face to Edges [e0 e1 e2] in order of vertices
					stringstream ss(line);  
					string word;
					int a, b, c, d;
					int i = 0; 
					while (ss >> word) { // Extract word from the stream.
						if(i == 1)
							a = stoi(word);
						if(i == 2)
							b = stoi(word);
						if(i == 3)
							c = stoi(word);
						if(i == 4)
							d = stoi(word);
						i++;
					}
					vector<int> edges = {b, c, d};
					sheet.faceToEdgeMap[a] = edges; 
				}
			}

			// generate face2offset map that i use. [0,1,2] correspond to their opposite vertex.
			sheet.face2Offset.resize(sheet.faces.size(), vector<double>{0, 0, 0});
			if(sheet.faceToEdgeMap.size() != 0) { // if it is a really old format this was empty (icosa)
				for (int fi=0; fi<sheet.face2Offset.size(); fi++) {
					vector<int> edges = sheet.faceToEdgeMap[fi];
					
					double offsete0 = sheet.edge2offset[edges[0]];
					double offsete1 = sheet.edge2offset[edges[1]];
					double offsete2 = sheet.edge2offset[edges[2]];
					sheet.face2Offset[fi] = {offsete1, offsete2 , offsete0}; // because edge0 is between 0 and 1. Offsets are mapped from opposite vertex
				}
			}

			// for(auto fe: faceToEdgeMap) {
			// 	cout << fe.first << " " << fe.second[0] << " " << fe.second[1] << " " << fe.second[2] << endl;
			// }
			// printf("V: %d F: %d V: %d\n", numV, numTrianglesFaces, numHingeFaces);
			myfile.close();
		}
		Eigen::MatrixXd sheet_vertices(sheet.vertices.size(), 3);
		Eigen::MatrixXi sheet_faces(sheet.faces.size() + sheet.hinges.size()* 2, 3);

		int vi = 0; 
		for(auto v: sheet.vertices) {
			sheet_vertices.row(vi) = Eigen::Vector3d(v[0], v[1], v[2]);
			vi++;
		}
		int fi = 0; 
		for(auto f: sheet.faces) {
			sheet_faces.row(fi) = Eigen::Vector3i(f[0], f[1], f[2]);
			fi++;
		}
		for(auto h: sheet.hinges) {
			sheet_faces.row(fi) = Eigen::Vector3i(h[0], h[1], h[2]);
			fi++;
			sheet_faces.row(fi) = Eigen::Vector3i(h[0], h[2], h[3]);
			fi++;
		}

		sheet_mesh_data.set_mesh(sheet_vertices, sheet_faces);
		// viewer->data_list[1] = sheet_mesh_data; // the current mesh
	}

	void saveMesh(std::string fname, const igl::opengl::ViewerData &mesh_data) {
		ofstream outfile;
		outfile.open(fname.c_str());
		outfile << "OFF\n";
		outfile << mesh_data.V.rows() << " " << mesh_data.F.rows() << " 0\n";
		for(int i=0; i<mesh_data.V.rows(); i++) { 
			Eigen::Vector3d p = mesh_data.V.row(i);
			outfile << dtos(p(0)) << " " << dtos(p(1)) << " " << dtos(p(2)) << "\n";
		}
		for(int i = 0; i < mesh_data.F.rows(); i++) { 
			Eigen::Vector3i f = mesh_data.F.row(i);
			outfile << "3 " << to_string(f(0)) << " " << to_string(f(1)) << " " << to_string(f(2)) << "\n"; // normal one
		}
	}

};

int main(int argc, char *argv[])
{
	igl::opengl::glfw::Viewer viewer;
	unsigned int mesh_viewer, sheet_viewer;
	string input_off_file, input_sheet_file, output_support_file;
	bool run_cli_flag = false;
	uint left_view;
	uint right_view;
	Settings stgs;
	double track_width_mm = 1.7, inner_diameter_mm = 1.0;
	uint chamfer_mode = 0; 

	if (argc == 1) {
		input_off_file   = string("../data/meshes/batman-092.off");
		input_sheet_file = string("../data/sheets/batman-092_half_05.sheet");
		output_support_file = string("../data/chamfered_mesh/batman-support.off");
	}
	else if (argc == 5) {
		stgs = read_settings(argv[1]);
		input_off_file = argv[2];
		input_sheet_file = argv[3];
		output_support_file = argv[4];
		// cout << input_off_file << endl; 
		// cout << input_sheet_file << endl; 
		// cout << output_support_file << endl; 

		run_cli_flag = stgs.use_cli_interface_chamfer;
		track_width_mm = stgs.track_width_mm;
		inner_diameter_mm = stgs.inner_diameter_mm;
		chamfer_mode = stgs.chamfer_mode; 
	}
	else {
		printf("Input Format: ./Chamferer <inp:.cfg> <inp:mesh.off> <inp:mesh.sheet> (<out:mesh-support.off>)");
	}

	viewer.core().viewport = Eigen::Vector4f(0, 0, 1280, 1600);
	left_view = viewer.core_list[0].id;
	right_view = viewer.append_core(Eigen::Vector4f(1280, 0, 1280, 1600));

	// viewer.core(left_view).background_color = Eigen::Vector4f(1, 1, 1, 1);
	viewer.core(left_view).is_animating = true;
	viewer.core(right_view).background_color = Eigen::Vector4f(1, 1, 1, 1);
	viewer.core(right_view).is_animating = true;
	// // sets the rendering speed to not wait for poll events for rendering

	CustomPlugin plugin;
	viewer.plugins.push_back(&plugin);
	createUiMenu create_menu(input_off_file, input_sheet_file, output_support_file, left_view, right_view, run_cli_flag, track_width_mm, inner_diameter_mm, chamfer_mode);
	plugin.widgets.push_back(&create_menu);

	viewer.callback_post_resize = [&](igl::opengl::glfw::Viewer &v, int w, int h)
	{
		v.core(left_view).viewport = Eigen::Vector4f(0, 0, w / 2, h);
		v.core(right_view).viewport = Eigen::Vector4f(w / 2, 0, w - (w / 2), h);
		return true;
	};

	viewer.launch();
	return EXIT_SUCCESS;
}