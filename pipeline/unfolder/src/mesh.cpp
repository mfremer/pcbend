#include "common.h"

void getMeshData(const igl::opengl::ViewerData &mesh, Eigen::MatrixXi &E2V, Eigen::MatrixXi &F2E, Eigen::MatrixXi &F2V) {
    Eigen::MatrixXi EV, FE, EF;
    igl::edge_topology(mesh.V, mesh.F, EV, FE, EF);
    E2V = EV;
    F2E = FE;
    F2V.resize(FE.rows(), 3);
    uint e0, e1,e2;
    uint v0, v1,v2;
    for(uint fi=0; fi<FE.rows(); fi++) {
        e0 = FE.row(fi)(0); e1 = FE.row(fi)(1); e2 = FE.row(fi)(2);
        v0 = EV.row(e0)(0); 
        v1 = EV.row(e0)(1); 
        v2 = ( (v0==EV.row(e1)(0) or v1==EV.row(e1)(0)) ? EV.row(e1)(1): EV.row(e1)(0)); // picking up vertices so as to not repeat them
        if(v0 == v1 or v1==v2 or v0==v2) {
            printf("face index : %d\n", fi);
            printf("e0 : %d e1 : %d e2 : %d\n", e0, e1, e2);
            printf("v0 : %d v1 : %d v2 : %d\n", v0, v1, v2);
            printf("Assumption went wrong carefully handle this case here...\n");
            exit(1);
        }
        F2V.row(fi) = mesh.F.row(fi);
    }

}
void getMeshEdges(const igl::opengl::ViewerData &mesh, Eigen::MatrixXi &edges) {
    Eigen::MatrixXi EV, FE, EF;
    igl::edge_topology(mesh.V, mesh.F, EV, FE, EF);
    edges = EV;
    // F2E = FE;
    // F2V = EF;
}

void saddleMesh(const igl::opengl::ViewerData & mesh, igl::opengl::ViewerData & saddle_mesh) {

    Eigen::VectorXi bool_saddle;
    if(mesh.V.size() == 0) 
        return;
    isSaddle(mesh, bool_saddle);
    Eigen::MatrixXd saddle_V;
    Eigen::MatrixXi saddle_F;
    std::vector<std::vector<Eigen::Vector3d>> vertices_vector;
    std::vector<std::vector<Eigen::Vector3i>> faces_vector;
    int vertices_size = 0, faces_size = 0;
    int num_of_saddle_points = 0;
    for (int vi=0; vi< mesh.V.rows(); vi++) {			
        if(bool_saddle(vi)) {
            Eigen::Vector3d pos = mesh.V.row(vi);
            std::vector<Eigen::Vector3d> vertices;
            std::vector<Eigen::Vector3i> faces;
            addSphere(pos, vertices, faces);
            vertices_size += vertices.size();
            faces_size += faces.size();
            num_of_saddle_points++;
            vertices_vector.push_back(vertices);
            faces_vector.push_back(faces);
            printf("The %d th vertex is a saddle vertex at location: %f %f %f\n", vi, pos(0), pos(1), pos(2));
        }
    }

    saddle_V.resize(vertices_size, 3);
    saddle_F.resize(faces_size, 3);


    printf("Number of saddle vertices : %d \n", num_of_saddle_points);
    printf("Total saddle_V size : %d saddle_F size : %d\n", vertices_size, faces_size);

    for (size_t i = 0; i < num_of_saddle_points; i++) {
        std::vector<Eigen::Vector3d> Svs = vertices_vector[i];
        std::vector<Eigen::Vector3i> SFs = faces_vector[i];
        for (size_t j = 0; j < Svs.size(); j++) {
            saddle_V.row(i*Svs.size() + j) = Svs[j];
            // printf("current saddle row index : %ld\n", i*Svs.size() + j);
        }
        for (size_t j = 0; j < SFs.size(); j++) {
            saddle_F.row(i*SFs.size() + j) = SFs[j] + Eigen::Vector3i(i*Svs.size(), i*Svs.size(), i*Svs.size());;
        }        
    }


    saddle_mesh.set_mesh(saddle_V, saddle_F);

}

void addSphere(Eigen::Vector3d pos, std::vector<Eigen::Vector3d> &vertices, std::vector<Eigen::Vector3i> &faces, double radius) {
    // code from http://www.songho.ca/opengl/gl_sphere.html

    int sectorCount = 4;
    int stackCount = 4;
    double sectorStep = 2.0 * M_PI / sectorCount;
    double stackStep = M_PI / stackCount;
    double sectorAngle, stackAngle;
    double x,y,z, xy;

    for(int i = 0; i <= stackCount; ++i)
    {
        stackAngle = (M_PI / 2.0) - i * stackStep;        // starting from pi/2 to -pi/2
        xy = radius * cos(stackAngle);             // r * cos(u)
        z = radius * sin(stackAngle);              // r * sin(u)

        // add (sectorCount+1) vertices per stack
        // the first and last vertices have same position and normal, but different tex coords
        for(int j = 0; j <= sectorCount; ++j)
        {
            sectorAngle = j * sectorStep;           // starting from 0 to 2pi
            x = xy * cos(sectorAngle);             // r * cos(u) * cos(v)
            y = xy * sin(sectorAngle);             // r * cos(u) * sin(v)
            Eigen::Vector3d Spos = Eigen::Vector3d(x,y,z);
            vertices.push_back(pos + Spos);
        }
    }

    int k1, k2;
    for(int i = 0; i < stackCount; ++i)
    {
        k1 = i * (sectorCount + 1);     // beginning of current stack
        k2 = k1 + sectorCount + 1;      // beginning of next stack

        for(int j = 0; j < sectorCount; ++j, ++k1, ++k2)
        {
            // k1 => k2 => k1+1
            if(i != 0)
                faces.push_back(Eigen::Vector3i(k1, k2, k1+1));
            // k1+1 => k2 => k2+1
            if(i != (stackCount-1))
                faces.push_back(Eigen::Vector3i(k1+1, k2, k2+1));
        }
    }   
}

void isSaddle(const igl::opengl::ViewerData &mesh, Eigen::VectorXi &saddleBoolArray) {
	const double eps = 1.0e-4;
    Eigen::VectorXd K; // stores the gaussian curvature. 
	igl::gaussian_curvature(mesh.V, mesh.F, K);
    saddleBoolArray.resize(K.size());

    for (int vi=0; vi< mesh.V.rows(); vi++) {			
        double gaussian_angle = K(vi);
        if (gaussian_angle < eps)
            saddleBoolArray(vi) = 1; // yes it is a saddle vertex
        else
            saddleBoolArray(vi) = 0; // yes it is a saddle vertex
    }
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

void computeBoundedBox(const igl::opengl::ViewerData &sheet_mesh, double &xmin, double &xmax, double &ymin, double &ymax) {
    double _xmin = std::numeric_limits<double>::max(), _ymin = std::numeric_limits<double>::max();
    double _xmax = std::numeric_limits<double>::lowest(), _ymax = std::numeric_limits<double>::lowest();

    for (int vi = 0; vi < sheet_mesh.V.rows(); vi++) {
        Eigen::Vector3d p = sheet_mesh.V.row(vi);

        _xmin = std::min(p(0), _xmin);
        _xmax = std::max(p(0), _xmax);
        _ymin = std::min(p(1), _ymin);
        _ymax = std::max(p(1), _ymax);
    }
    xmin = _xmin;
    xmax = _xmax;
    ymin = _ymin;
    ymax = _ymax;
}

void saveSheet(const igl::opengl::ViewerData &sheet_data, const igl::opengl::ViewerData &sheet_clean_data, 
                const vector<vector<int>> &sheet_hinge_quads, vector<meshEdge> &mesh_edges, map<int, triplet> &meshFace2Edge,
                std::vector<std::vector<int>> &flatFaceVisitOrder, int numFaces, string fname, double drill_radius, double user_margin)
{
    // can clean up the flatFaceVisitOrder if needed. 
    // num faces is important because the way I have stored the data I would need to distinguis which faces are hinge faces and which are normal faces
    ofstream outfile;
    outfile.open(fname.c_str());

    outfile << "# Sheet info : number of vertices, triangle faces and hinge faces" << endl;
    outfile << "si "<< sheet_data.V.rows() << " " << numFaces << " " << sheet_hinge_quads.size() << endl;
    // outfile << "# number of Hinge Faces " << sheet_data.F.rows() - numFaces << endl;
    outfile << "# Unfolder parameters: drill bit radius, user fabrication margin" << endl;
    outfile << "up " << drill_radius << " " << user_margin << endl;
    outfile << "# Hinge parameters: track width, inner diameter, outer diameter" << endl;
    outfile << "hp 1.7 1 1" << endl;
    outfile << "# Storing all the vertices both for flattened faces and hinges\n";
    for(int i=0; i<sheet_data.V.rows(); i++) { 
        Eigen::Vector3d p = sheet_data.V.row(i);
        // cout << "v " << dtos(p(0)) << " " << dtos(p(1)) << " " << dtos(p(2)) << "\n";
        outfile << "v " << dtos(p(0)) << " " << dtos(p(1)) << " " << dtos(p(2)) << "\n";
    }
    
    outfile << "# Indices for flattened faces\n";
    for(int i = 0; i < numFaces; i++) { 
        Eigen::Vector3i f = sheet_data.F.row(i);
        // cout << "f " << to_string(f(0)) << " " << to_string(f(1)) << " " << to_string(f(2)) << "\n";
        outfile << "f " << to_string(f(0)) << " " << to_string(f(1)) << " " << to_string(f(2)) << "\n"; // normal one
        // outfile << "f " << to_string(f(0)+1) << " " << to_string(f(1)+1) << " " << to_string(f(2)+1) << "\n"; // blender
    }

    // old code. 
    // // outfile << "# Indices for hinges\n";
    // for(int i = numFaces; i < sheet_data.F.rows(); i++) { 
    //     Eigen::Vector3i f = sheet_data.F.row(i);
    //     cout << "f " << to_string(f(0)) << " " << to_string(f(1)) << " " << to_string(f(2)) << "\n";
    //     // outfile << "f " << to_string(f(0)) << " " << to_string(f(1)) << " " << to_string(f(2)) << "\n";
    //     // outfile << "f " << to_string(f(0)+1) << " " << to_string(f(1)+1) << " " << to_string(f(2)+1) << "\n";
    // }
    outfile << "# Indices for hinges\n";
    for (int i = 0; i < sheet_hinge_quads.size(); i++) {
        vector<int> quad = sheet_hinge_quads[i];
        outfile << "h " << to_string(quad[0]) << " " << to_string(quad[1]) << " " << to_string(quad[2]) << " " << to_string(quad[3]) << "\n";
    }

    outfile << "# Map Edges to [hingeId Face0 Face1 Offset IsHinge IsHalfHinge IsBoundary]\n"; // order does not matter
    for(int ei = 0; ei < mesh_edges.size(); ei++ ) { // change it into something that can iterate Es
        // printf all the edge information 
        meshEdge edge = mesh_edges[ei];
        // printf("edge : %d face : %d %d offset : %f isHinge :%d v0 : %d v1: %d:\n", 
        // edge.hingeId, edge.faces.first, edge.faces.second, edge.offset, edge.isHinge, edge.orderedVertices.first, edge.orderedVertices.second);
        outfile << "e " << edge.hingeId << " " << edge.faces.first << " " << edge.faces.second << " " << edge.offset << " " << edge.isHinge << " " << edge.isHalfHinge << " " << edge.isBoundary << endl;
    }
    outfile << "# Map Face to Edges [e0 e1 e2] in order of vertices\n"; // both cut and hinge [id offset isHinge]";
    for(int fi = 0; fi < meshFace2Edge.size(); fi++ ) { // change it into something that can iterate Es
        // printf all the edge information 
        int e0 = std::get<0>(meshFace2Edge[fi]), e1 = std::get<1>(meshFace2Edge[fi]), e2 = std::get<2>(meshFace2Edge[fi]);
        outfile << "fe " << fi << " " << e0 << " " << e1 << " " << e2 << "\n";
        // cout << "fe " << fi << " " << e0 << " " << e1 << " " << e2 << "\n";
    }
    printf("Done saving sheet file\n");
}

std::pair<int, int> getCommonVertices(const igl::opengl::ViewerData &mesh, int face0, int face1, bool orderThem) {
    Eigen::Vector3i fvace0 = mesh.F.row(face0);
    Eigen::Vector3i fvace1 = mesh.F.row(face1);
    int cv0 = -1, cv1 = -1;
    findCommonVertex(cv0, cv1, fvace0, fvace1);

    if(orderThem) {
        orderVertices(cv0, cv1); // make sure that cv1 is more than than cv0
    }
    return make_pair(cv0, cv1);
}

void fixMeshOrientation(igl::opengl::ViewerData &mesh) {

    // removing dependency on embree
    // igl::orient_outward(mesh.V, mesh.F)
    // Eigen::VectorXi I;
    // Eigen::VectorXi C;
    // Eigen::MatrixXd V = mesh.V;
    // Eigen::MatrixXi F = mesh.F;
    // igl::embree::reorient_facets_raycast(V, F, F.rows()*100, 10, true, false,false, I, C);
    
    // Eigen::MatrixXi FF; 
    // FF.resize(F.rows(), F.cols());
    // for(int i = 0;i<I.rows();i++)
    // {
    //     if(I(i))
    //         FF.row(i) = (F.row(i).reverse()).eval();
    //     else
    //         FF.row(i) = F.row(i);
    // }
    // mesh.clear();
    // mesh.set_mesh(V, FF);
}