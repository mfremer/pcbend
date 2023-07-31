#include "common.h"

class CustomPlugin : public igl::opengl::glfw::imgui::ImGuiPlugin {
};

class createUiMenu : public igl::opengl::glfw::imgui::ImGuiMenu {


public:

	string filename;
	string sheetFileName; 

	igl::opengl::ViewerData mesh_data, sheet_data, sheet_clean_data; // to keep track of mesh and sheet_data
	igl::opengl::ViewerData dual_mesh_data,	dual_mst_data;	   // to keep track of dual_mesh and mst mesh
	// Note How this dual_mesh_data stores data: 
	// 1) Vertices are stored as Original Vertices : Face Center: Edge center
	// 2) Dual edges for the unfolded version are stored in the matrix dual_edges defined below.. 
	igl::opengl::ViewerData saddle_mesh_data;		   // to keep track of dual_mesh
	Eigen::MatrixXi visual_dual_edges, dual_edges, boundary_edges;
	vector<meshEdge> mesh_edge_data; // mesh_edge_data contains the mapping between the two faces. // can be either hinge or cut though
	std::vector<bool> bool_dual_edges; // true --> is a hinge false --> is a cut edge
	Eigen::MatrixXi mesh_edges_visual, cut_edges, hinge_edges; // hinge_edges are same as mst_edges but just for clearer name convention..
	std::vector<std::vector<int>> sheet_hinge_quads; 
	// the difference between visual_dual_edge and dual_edge is in folded and unfolded configuration. The visual_dual_edge contains the mid-point as well..
	// which is not needed for the dual edge of the graph for computation of MST
	Eigen::MatrixXd MST_color;

	unsigned int mesh_state_enum; // 0 for not used; 1 for used without MST; 2 for used but with MST

	igl::opengl::ViewerData split_mesh_data, merge_mesh_data;
	bool show_mesh, show_sheet, with_offset, bool_mesh_cut, merge_patches;
	bool show_sheet_checkbox;
	bool debug_menu;
	bool bool_unfold, sheet_show_dirty;

	bool show_dual_mesh, show_mst_state;
	bool show_wire_frame, bool_show_faces, bool_show_vertex_labels, bool_show_face_labels;

	bool show_saddle_vertices;
	bool compute_time;

	uint mesh_id, dual_mesh_id, sheet_mesh_id, sheet_clean_mesh_id, saddle_mesh_id, dual_mst_id;
	int num_unfolded;
	double obj_scale;
	double drill_bit_radius;
	double user_margin;
	int min_patches;
	vector<vector<int>> flatFaceVisitOrder;
	map<int, triplet> meshFace2Edge;

	int current_mst_heuristic = 0; // 0 for max_dihedral_angle and 1 for min_perimeter_heuristic

	bool adaptiveScaling, useHalfHinges;

	double bb_xmin, bb_xmax, bb_ymin, bb_ymax; // boundary box info.
	bool show_sheet_bounding_box;
	igl::opengl::ViewerData bbox_mesh;		   // to keep track of dual_mesh

	bool run_cli_flag = false;

	uint _dual_mesh_ind, _mst_mesh_ind, _mesh_ind, _sheet_mesh_ind, _sheet_clean_mesh_ind, _saddle_mesh_ind;

	createUiMenu(string _filename, string _sheetFileName, bool _useHalfHinges, double _fab_margin, int _minPatches, bool _run_cli_flag, int _current_mst_heuristic) {
		filename = _filename;
		sheetFileName = _sheetFileName;
		useHalfHinges = _useHalfHinges; 
		min_patches = _minPatches;
		user_margin = _fab_margin;
		run_cli_flag = _run_cli_flag;
		current_mst_heuristic = _current_mst_heuristic;
	}

	virtual void init(igl::opengl::glfw::Viewer *_viewer, igl::opengl::glfw::imgui::ImGuiPlugin *_plugin) override {
		mesh_data.point_size = 10;
		mesh_data.line_width = 0.1;

		// viewer->data().shininess = 10;

		sheet_data.point_size = 10;
		sheet_data.line_width = 0.1;


		saddle_mesh_data.point_size = 10;
		saddle_mesh_data.line_width = 0.1;


		show_mesh = false;
		// show_sheet = true; // set by default for both of them
		show_sheet = false; // set by default for both of them
		sheet_show_dirty = false;
		// with_offset = false;
		with_offset = true;
		bool_mesh_cut = true;
		// bool_mesh_cut = false;
		merge_patches = true;
		// merge_patches = false;
		debug_menu = true;
		show_dual_mesh = false;
		show_mst_state = false;
		show_wire_frame = true;
		bool_show_faces = true;
		bool_show_vertex_labels = false;
		bool_show_face_labels = false;
		show_saddle_vertices = false;
	
		ImGuiMenu::init(_viewer, _plugin);
		
		// by default mesh id == 0
		mesh_id = 0;  // dual_mesh_id = 1 Set this variable when you visit it first time 
		dual_mesh_id = -1;  // dual_mesh_id = 1 Set this variable when you visit it first time 
		sheet_mesh_id = -1; // sheet_mesh_id = 2 Set this variable when you visit it first time
		sheet_clean_mesh_id = -1; // sheet_clean_mesh_id = 2 Set this variable when you visit it first time
		saddle_mesh_id = -1;
		dual_mst_id = -1;

		_dual_mesh_ind = -1;
		_mst_mesh_ind = -1;
		_mesh_ind = -1;
		_sheet_mesh_ind = -1;
		_sheet_clean_mesh_ind = -1;
		_saddle_mesh_ind = -1;

		mesh_state_enum = 0; // State variable to keep track of current dual mesh state 
		num_unfolded = -1;
		obj_scale=1.0;
		drill_bit_radius = 0.0;
		// drill_bit_radius = 0.3; // used for fabrication in the machine shop
		compute_time = true;

		// adaptiveScaling = true;
		adaptiveScaling = false;

		show_sheet_bounding_box = true;



		if(filename.size() != 0) {
			load_mesh(filename);
		}

		if(run_cli_flag) {

			dualMesh(mesh_data, viewer->data_list[_dual_mesh_ind], dual_edges, visual_dual_edges, boundary_edges);
			dual_mesh_data = viewer->data_list[_dual_mesh_ind];

			// TODO some bug with mst display and sheet_display together.....
			// perform mst on it
			MST(mesh_data, dual_mesh_data, dual_edges, viewer->data_list[_mst_mesh_ind], hinge_edges, cut_edges, boundary_edges, bool_dual_edges, mesh_edge_data, MST_color, current_mst_heuristic); // bool dual edges contain the right data
			setMSTMesh(viewer->data_list[_mesh_ind], mesh_edges_visual, MST_color, viewer->data_list[_mst_mesh_ind]);																				 // based on edges... should be same as dual_edges size
			dual_mst_data = viewer->data_list[_mst_mesh_ind];
			// just compute dual mesh and mst anyways since the heuristics might have changed.

			viewer->data_list[_sheet_mesh_ind].clear();
			unfoldMesh(viewer->data_list[_mesh_ind], dual_mesh_data, dual_edges, hinge_edges,
						mesh_edge_data, viewer->data_list[_sheet_mesh_ind],
						viewer->data_list[_sheet_clean_mesh_ind], sheet_hinge_quads,
						flatFaceVisitOrder, meshFace2Edge, bool_mesh_cut, merge_patches,
						num_unfolded, with_offset, obj_scale, drill_bit_radius, user_margin,
						min_patches, adaptiveScaling, useHalfHinges);
			viewer->data_list[_sheet_mesh_ind].dirty = igl::opengl::MeshGL::DIRTY_ALL;		 // fixes the bug where the face indices were disappearing...
			viewer->data_list[_sheet_clean_mesh_ind].dirty = igl::opengl::MeshGL::DIRTY_ALL; // fixes the bug where the face indices were disappearing...
			viewer->data_list[_mesh_ind].dirty = igl::opengl::MeshGL::DIRTY_ALL;			 // fixes the bug where the face indices were disappearing...
			sheet_data.clear();
			sheet_clean_data.clear();
			mesh_data.clear();
			sheet_data = viewer->data_list[_sheet_mesh_ind];
			sheet_clean_data = viewer->data_list[_sheet_clean_mesh_ind];
			mesh_data = viewer->data_list[_mesh_ind];
			sheet_show_dirty = false;

			viewer->data_list[_dual_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_mst_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_sheet_mesh_ind].set_visible(true, viewer->core_list[0].id);
			viewer->data_list[_sheet_clean_mesh_ind].set_visible(false, viewer->core_list[0].id);
			mesh_state_enum = 4;

			saveSheet(sheet_data, sheet_clean_data, sheet_hinge_quads, mesh_edge_data, meshFace2Edge, 
					flatFaceVisitOrder, mesh_data.F.rows(), sheetFileName, drill_bit_radius, user_margin);

			exit(1);
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


		// BIG assumption fix mesh data some triangles might be incorrectly faces.
		// make them positive orientation fix this thing outside in blender!!

		mesh_data = viewer->data_list[0]; // the current mesh

		getMeshEdges(viewer->data_list[0], mesh_edges_visual);
		dual_mesh_data.clear();
		dual_mst_data.clear();
		sheet_data.clear();
		saddle_mesh_data.clear();
		bbox_mesh.clear();

		// now create space for other meshes as well

		dual_mesh_id = viewer->append_mesh();  // dual_mesh_id = current_new_id + 1
		sheet_mesh_id = viewer->append_mesh();  // sheet_mesh_id = current_new_id + 2
		saddle_mesh_id = viewer->append_mesh();  // saddle_mesh_id = current_new_id + 3
		dual_mst_id = viewer->append_mesh();  // dual_mst_id = current_new_id + 4
		sheet_clean_mesh_id = viewer->append_mesh(); // sheet_clean_mesh_id  = current_new_id + 5

		show_mesh = true;
		show_sheet = false;
		sheet_show_dirty = true;

		_dual_mesh_ind = viewer->mesh_index(dual_mesh_id);
		_mst_mesh_ind = viewer->mesh_index(dual_mst_id);
		_mesh_ind = viewer->mesh_index(mesh_id);
		_sheet_mesh_ind = viewer->mesh_index(sheet_mesh_id);
		_sheet_clean_mesh_ind = viewer->mesh_index(sheet_clean_mesh_id);
		_saddle_mesh_ind = viewer->mesh_index(saddle_mesh_id);
	}

	void draw_custom_viewer_menu() {
		// Workspace
		if (ImGui::CollapsingHeader("Workspace", ImGuiTreeNodeFlags_DefaultOpen))
		{
			float w = ImGui::GetContentRegionAvail().x;
			float p = ImGui::GetStyle().FramePadding.x;
			if (ImGui::Button("Load##Workspace", ImVec2((w - p) / 2.f, 0)))
			{
				viewer->load_scene();
			}
			ImGui::SameLine(0, p);
			if (ImGui::Button("Save##Workspace", ImVec2((w - p) / 2.f, 0)))
			{
				viewer->save_scene();
			}
		}

		// Mesh
		if (ImGui::CollapsingHeader("Mesh", ImGuiTreeNodeFlags_DefaultOpen))
		{
			float w = ImGui::GetContentRegionAvail().x;
			float p = ImGui::GetStyle().FramePadding.x;
			if (ImGui::Button("Load##Mesh", ImVec2((w - p) / 2.f, 0))) {
				load_mesh();
			}
			ImGui::SameLine(0, p);
			if (ImGui::Button("Save##Mesh", ImVec2((w - p) / 2.f, 0))) {
				std::string fname = igl::file_dialog_save();
				saveMesh(fname, mesh_data);
				// viewer->open_dialog_save_mesh();
			}
		} // after this the mesh is stored in data of the file

		// Sheet
		if (ImGui::CollapsingHeader("Sheet", ImGuiTreeNodeFlags_DefaultOpen)) {
			float w = ImGui::GetContentRegionAvail().x;
			float p = ImGui::GetStyle().FramePadding.x;
			// Sheet should only have a save sheet function and nothing else. One should not be allowed to load sheet
			if (ImGui::Button("Save##Sheet", ImVec2((w - p) / 2.f, 0)))
			{
				std::string fname = igl::file_dialog_save();

				if(fname.length() != 0) {
					int numFaces = mesh_data.F.rows();
					saveSheet(sheet_data, sheet_clean_data, sheet_hinge_quads, mesh_edge_data, meshFace2Edge, flatFaceVisitOrder, numFaces, fname, drill_bit_radius, user_margin);
				}
			}
		}

		if(mesh_data.F.rows()!= 0)
			ImGui::InputInt("Unfolder break", &num_unfolded);

		ImGui::InputInt("min patches", &min_patches);
		ImGui::InputDouble("Object Scale", &obj_scale, 0.1, 1, "%.4f");
		// add a manual scaling option here...
		if (ImGui::Button("Scale object", ImVec2(-1, 0))) {
			for (int i = 0; i < mesh_data.V.rows(); i++)  {
				mesh_data.V.row(i) = mesh_data.V.row(i) * obj_scale;
			}
			viewer->data_list[0] = mesh_data; 
		}

		ImGui::InputDouble("Drill bit radius", &drill_bit_radius, 0.01, 0.1, "%.4f");
		ImGui::InputDouble("user fab-error margin", &user_margin, 0.1, 1, "%.4f");
		ImGui::Checkbox("Show Mesh", &show_mesh);
		ImGui::Checkbox("Show Sheet", &show_sheet);
		ImGui::Checkbox("With Offset", &with_offset);
		ImGui::Checkbox("mesh cut", &bool_mesh_cut);
		ImGui::Checkbox("merge meshes", &merge_patches);

		ImGui::Checkbox("Show dual mesh", &show_dual_mesh);
		ImGui::Checkbox("Show mst mesh", &show_mst_state);
		ImGui::Checkbox("Use hinge adaptive scaling", &adaptiveScaling);
		ImGui::Checkbox("Use half hinges", &useHalfHinges);

		ImGui::Text("Choose a MST heuristic");

		ImGui::RadioButton("Max Dihedral angle", &current_mst_heuristic, 0);
		ImGui::RadioButton("Min Perimeter", &current_mst_heuristic, 1);

		if(flatFaceVisitOrder.size() > 0) {
			ImGui::Text("The unfolding has : %ld patches", flatFaceVisitOrder.size());
		}

		if (show_dual_mesh and not show_mst_state)
		{
			if(dual_mesh_data.V.size() == 0) {
				dualMesh(mesh_data, viewer->data_list[_dual_mesh_ind], dual_edges, visual_dual_edges, boundary_edges);
				dual_mesh_data = viewer->data_list[_dual_mesh_ind];
			}
			// transition from mst or not...
			if(mesh_state_enum != 1) {
				setDualMeshColor(viewer->data_list[_dual_mesh_ind], visual_dual_edges); // change the color back to default dual edge color
				dual_mesh_data = viewer->data_list[_dual_mesh_ind];
			}

			mesh_state_enum = 1;
			viewer->data_list[_dual_mesh_ind].set_visible(true, viewer->core_list[0].id);
			viewer->data_list[_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_mst_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_sheet_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_sheet_clean_mesh_ind].set_visible(false, viewer->core_list[0].id);

			viewer->data_list[_dual_mesh_ind].line_width = 2.0f;
		}
		else if(show_dual_mesh and show_mst_state) { // mst is only valid if the dual mesh is there...
			if(dual_mesh_data.V.size() == 0) {
				dualMesh(mesh_data, viewer->data_list[_dual_mesh_ind], dual_edges, visual_dual_edges, boundary_edges);
			}
			// transition from mst or not...
			if(mesh_state_enum != 2 or dual_mesh_data.V.size() == 0) {
				MST(mesh_data, dual_mesh_data, dual_edges, viewer->data_list[_mst_mesh_ind], hinge_edges, cut_edges, boundary_edges, bool_dual_edges, mesh_edge_data, MST_color, current_mst_heuristic);
				setDualMeshColor(viewer->data_list[_dual_mesh_ind], visual_dual_edges, MST_color); // based on edges... should be same as dual_edges size
				dual_mesh_data = viewer->data_list[_dual_mesh_ind];			
			}
			
			mesh_state_enum = 2;
			viewer->data_list[_dual_mesh_ind].set_visible(true, viewer->core_list[0].id);
			
			viewer->data_list[_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_mst_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_sheet_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_sheet_clean_mesh_ind].set_visible(false, viewer->core_list[0].id);

			viewer->data_list[_dual_mesh_ind].line_width = 2.0f;
		}
		else if(not show_dual_mesh and show_mst_state) { // mst is only valid if the dual mesh is there...
			// transition from mst or not...
			if(dual_mst_data.V.size() == 0) {
				dualMesh(mesh_data, viewer->data_list[_dual_mesh_ind], dual_edges, visual_dual_edges, boundary_edges);
			}			
			if(mesh_state_enum != 3 or dual_mst_data.V.size() == 0) {
				MST(mesh_data, dual_mesh_data, dual_edges, viewer->data_list[_mst_mesh_ind], hinge_edges, cut_edges, boundary_edges, bool_dual_edges, mesh_edge_data, MST_color, current_mst_heuristic);
				setMSTMesh(viewer->data_list[_mesh_ind], mesh_edges_visual, MST_color, viewer->data_list[_mst_mesh_ind]); // based on edges... should be same as dual_edges size
				dual_mst_data = viewer->data_list[_mst_mesh_ind];			
				dual_mesh_data = viewer->data_list[_dual_mesh_ind];			
			}
			
			mesh_state_enum = 3;
			viewer->data_list[_dual_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_mst_mesh_ind].set_visible(true, viewer->core_list[0].id);
			viewer->data_list[_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_sheet_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_sheet_clean_mesh_ind].set_visible(false, viewer->core_list[0].id);
		}	
		else if(show_sheet) {
			// call unfold function ...
			if (mesh_state_enum != 4) {
				// if the dual mesh and mst is not performed peform them
					
				// get the dual mesh
				dualMesh(mesh_data, viewer->data_list[_dual_mesh_ind], dual_edges, visual_dual_edges, boundary_edges);
				dual_mesh_data = viewer->data_list[_dual_mesh_ind];

				// TODO some bug with mst display and sheet_display together.....
				// perform mst on it
				MST(mesh_data, dual_mesh_data, dual_edges, viewer->data_list[_mst_mesh_ind], hinge_edges, cut_edges, boundary_edges, bool_dual_edges, mesh_edge_data, MST_color, current_mst_heuristic); // bool dual edges contain the right data
				setMSTMesh(viewer->data_list[_mesh_ind], mesh_edges_visual, MST_color, viewer->data_list[_mst_mesh_ind]); // based on edges... should be same as dual_edges size
				dual_mst_data = viewer->data_list[_mst_mesh_ind];			

				// just compute dual mesh and mst anyways since the heuristics might have changed. 
				if(sheet_data.V.size() == 0 or sheet_show_dirty) {
					viewer->data_list[_sheet_mesh_ind].clear();
					unfoldMesh(viewer->data_list[_mesh_ind], dual_mesh_data, dual_edges, hinge_edges, 
							 mesh_edge_data, viewer->data_list[_sheet_mesh_ind], 
							 viewer->data_list[_sheet_clean_mesh_ind], sheet_hinge_quads,
							   flatFaceVisitOrder, meshFace2Edge, bool_mesh_cut, merge_patches, 
							   num_unfolded, with_offset, obj_scale, drill_bit_radius, user_margin, 
							   min_patches, adaptiveScaling, useHalfHinges);
					viewer->data_list[_sheet_mesh_ind].dirty = igl::opengl::MeshGL::DIRTY_ALL; // fixes the bug where the face indices were disappearing...
					viewer->data_list[_sheet_clean_mesh_ind].dirty = igl::opengl::MeshGL::DIRTY_ALL; // fixes the bug where the face indices were disappearing...
					viewer->data_list[_mesh_ind].dirty = igl::opengl::MeshGL::DIRTY_ALL; // fixes the bug where the face indices were disappearing...
					sheet_data.clear();
					sheet_clean_data.clear();
					mesh_data.clear();
					sheet_data = viewer->data_list[_sheet_mesh_ind];
					sheet_clean_data = viewer->data_list[_sheet_clean_mesh_ind];
					mesh_data = viewer->data_list[_mesh_ind];
					sheet_show_dirty = false;
				}
			}

			viewer->data_list[_dual_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_mst_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_sheet_mesh_ind].set_visible(true, viewer->core_list[0].id);
			viewer->data_list[_sheet_clean_mesh_ind].set_visible(false, viewer->core_list[0].id);
			mesh_state_enum = 4;
			// if(compute_time) // use time from terminal to get the correct time
			// 	exit(1);
		}	
		else {
			viewer->data_list[_dual_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_mst_mesh_ind].set_visible(false, viewer->core_list[0].id);
			viewer->data_list[_sheet_mesh_ind].set_visible(false, viewer->core_list[0].id);
			if(show_mesh)
				viewer->data_list[_mesh_ind].set_visible(true, viewer->core_list[0].id);
			
			mesh_state_enum = 0;
		}
		// printf("current mesh enum : %d\n", mesh_state_enum);

		ImGui::Checkbox("Debug Menu", &debug_menu);
		if (debug_menu)
		{
			// Viewing options
			if (ImGui::CollapsingHeader("Viewing Options", ImGuiTreeNodeFlags_DefaultOpen))
			{
				if (ImGui::Button("Center object", ImVec2(-1, 0)))
				{
					viewer->core().align_camera_center(viewer->data().V, viewer->data().F);
				}
				if (ImGui::Button("Snap canonical view", ImVec2(-1, 0)))
				{
					viewer->snap_to_canonical_quaternion();
				}
				// Orthographic view
				ImGui::Checkbox("Orthographic view", &(viewer->core().orthographic));
			}

			// Overlays
			if (ImGui::CollapsingHeader("Overlays", ImGuiTreeNodeFlags_DefaultOpen))
			{
				// make_checkbox("Wireframe", viewer->data().show_lines);
				ImGui::Checkbox("Show wireframe", &show_wire_frame); ImGui::SameLine(0, ImGui::GetStyle().FramePadding.x);
				ImGui::Checkbox("Fill", &bool_show_faces); 
				ImGui::Checkbox("Show vertex labels", &bool_show_vertex_labels); ImGui::SameLine(0, ImGui::GetStyle().FramePadding.x);
				ImGui::Checkbox("Show faces labels", &bool_show_face_labels);
				ImGui::Checkbox("Show saddle vertices", &show_saddle_vertices);
				if(show_sheet) {
					ImGui::Checkbox("Show Sheet bounding box", &show_sheet_bounding_box);
					if (show_sheet_bounding_box) {
						double bb_width = -1, bb_length = -1;
						computeBoundedBox(sheet_data, bb_xmin, bb_xmax, bb_ymin, bb_ymax);
						bb_width = bb_xmax - bb_xmin;
						bb_length = bb_ymax - bb_ymin;
						ImGui::Text("BBox width: %0.3f\nlength :%0.3f", bb_width, bb_length);
					}
				}
			}
		}
		if(show_saddle_vertices) {
			// write code to add sphere at this point...
			// this sphere can be stored in a new data object
			if(saddle_mesh_data.V.size() == 0) {
				saddleMesh(viewer->data_list[_mesh_ind], viewer->data_list[_saddle_mesh_ind]);
				saddle_mesh_data = viewer->data_list[_saddle_mesh_ind];
			}
			viewer->data_list[_saddle_mesh_ind].set_visible(true, viewer->core_list[0].id);
		}
		else {
			viewer->data_list[_saddle_mesh_ind].set_visible(false, viewer->core_list[0].id);
		}

		for (auto &mesh : viewer->data_list) {
			if(not mesh.id == dual_mst_id) { // find a better fix for displaying edge specific color 
				mesh.show_lines = show_wire_frame;
				// other meshes whose edges have to be colored
			}
			mesh.show_faces  = bool_show_faces;
			mesh.show_vertex_labels  = bool_show_vertex_labels;
			mesh.show_face_labels  = bool_show_face_labels;

			// if(bool_show_face_labels and mesh.id == sheet_mesh_id) {
			// 	printf("value of mesh face : %ld\n", mesh.face_labels_positions.rows());
			// } 

			viewer->core().orthographic = true; // this is just so that i can have better looking indices...
		}
	}

	virtual void draw_viewer_window() override
	{
		float menu_width = 180.f * menu_scaling();
		ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f), ImVec2(menu_width, -1.0f));
		bool _viewer_menu_visible = true;
		ImGui::Begin(
			"Viewer", &_viewer_menu_visible,
			ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize);
		ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);
		draw_custom_viewer_menu();
		ImGui::PopItemWidth();
		ImGui::End();
	}

	virtual bool key_pressed(unsigned int key, int modifiers)
	{

		// ImGui_ImplGlfw_CharCallback(nullptr, key); // fix this line...
		if (key == 'L' or key == 'l')
		{
			load_mesh();
		}
		if (key == 'n' or key == 'N')
		{
			// New();
		}
		if (key == 'S' or key == 's')
		{
			// SaveMesh();
		}
		if (key == 'F' or key == 'f')
		{
			// Split mesh;
		}
		if (key == '-' )
		{
			user_margin -= 1;
		}
		if (key == '+' )
		{
			user_margin += 1;
		}
		if (key == 'c' or key == 'C')
		{
			bool_mesh_cut = !bool_mesh_cut;
		}

		if (key == 'U' or key == 'u')
		{
			show_sheet = !show_sheet;
			if(show_sheet)
				printf("unfold on!\n");
			else	{
				printf("unfold off!\n");
				sheet_show_dirty = true; // if you need to apply unfold everytime you fold uncomment this line
			}
		}

		if (key == 'D' or key == 'd')
		{
			show_dual_mesh = !show_dual_mesh;
			if(show_dual_mesh)
				printf("Dual mesh on!\n");
			else	
				printf("Dual mesh off!\n");
		}
		if (key == 'M' or key == 'm')
		{
			show_mst_state = !show_mst_state;
			if(show_mst_state)
				printf("MST view on!\n");
			else	
				printf("MST view off!\n");
		}

		if (key == 'W' or key == 'w')
		{
			with_offset = !with_offset;
			if(with_offset)
				printf("with offset on!\n");
			else	
				printf("with offset off!\n");
		}
		
		if (key == 'F' or key == 'f')
		{
			bool_show_face_labels = !bool_show_face_labels;
			if(bool_show_face_labels)
				printf("show face indices on!\n");
			else	
				printf("show face indices off!\n");
		}

		if (key == 'V' or key == 'v')
		{
			bool_show_vertex_labels = !bool_show_vertex_labels;
			if(bool_show_vertex_labels)
				printf("show vertex indices on!\n");
			else	
				printf("show vertex indices off!\n");
		}



		if (key == 'Z' or key == 'z')
		{
			show_saddle_vertices = !show_saddle_vertices;
			if(show_saddle_vertices)
				printf("Saddle vertices on!\n");
			else	
				printf("Saddle vertices off!\n");
		}


		if (key == 'Q' or key == 'q')
		{
			exit(1);
		}
		return ImGui::GetIO().WantCaptureKeyboard;
	}

	virtual bool mouse_down(int button, int modifier) override
	{
		return false; // keep it false if one is not interested in selecting anything
	}
};

int main(int argc, char *argv[])
{


	igl::opengl::glfw::Viewer viewer;
	unsigned int mesh_viewer, sheet_viewer;
	string input_off_file, output_sheet_file;
	bool useHalfHinges = false; 
	int minPatches = 1; 
	double fab_margin = 0; 
	bool run_cli_flag = false; 
	int current_mst_heuristic = 0;
	Settings stgs;
	if(argc == 1) {
		input_off_file = string("../../data/meshes/cat-102.off");
	}
	else if(argc == 4) {
		// printf("%s %s\n",argv[0], argv[1]);
		stgs = read_settings(argv[1]);
		input_off_file= argv[2];
		output_sheet_file = argv[3];
		fab_margin = stgs.use_half_hinges;
		minPatches = stgs.num_patches_desired;
		run_cli_flag = stgs.use_cli_interface;
		useHalfHinges = stgs.use_half_hinges;
		if(stgs.heuristic == "max_dihedral_angle") {
			current_mst_heuristic = 0;
		}
		else if (stgs.heuristic == "min_perimeter_heuristic") {
			current_mst_heuristic = 1;
		}
		else {
			printf("heuristic not implemented. Modify the settings file\n");
			exit(1);
		}
	}
	else {
		printf("USAGE: ./unfolder <inp:.cfg> <inp:mesh.off> (<out:.sheet>)\n");
		exit(1);
	}



	viewer.core().background_color = Eigen::Vector4f(1, 1, 1, 1);
	viewer.core().is_animating = true;
	// // sets the rendering speed to not wait for poll events for rendering

	CustomPlugin plugin;
	viewer.plugins.push_back(&plugin);

	createUiMenu create_menu(input_off_file, output_sheet_file, useHalfHinges, fab_margin, minPatches, run_cli_flag, current_mst_heuristic);
	plugin.widgets.push_back(&create_menu);

	viewer.launch();
	return EXIT_SUCCESS;
}
