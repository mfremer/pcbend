#include "common.h"


DSU::DSU(int n)
{
    parent = new int[n];
    rank = new int[n];

    for (int i = 0; i < n; i++)
    {
        parent[i] = -1;
        rank[i] = 1;
    }
}

// Find function
int DSU::find(int i)
{
    if (parent[i] == -1)
        return i;

    return parent[i] = find(parent[i]);
}
// union function
void DSU::unite(int x, int y)
{
    int s1 = find(x);
    int s2 = find(y);

    if (s1 != s2)
    {
        if (rank[s1] < rank[s2])
        {
            parent[s1] = s2;
            rank[s2] += rank[s1];
        }
        else
        {
            parent[s2] = s1;
            rank[s1] += rank[s2];
        }
    }
}



Graph::Graph(int N) {
    this->N = N;
}
 
void Graph::addEdge(int v0, int v1, double weight, int edge_id) {
    edgelist.push_back({weight, double(v0), double(v1), double(edge_id)}); // since it is a vector i have to type cast it. Remember to type cast it back to int while using it
}

double Graph::kruskals_mst(std::vector<std::vector<int>> &mst_edges) {
    // 1. Sort all edges
    sort(edgelist.begin(), edgelist.end()); // sort based on the first element of the vector of vector. 

    // Initialize the DSU
    DSU s(N); // number of vertices
    double total_weight = 0;
    for (auto edge : edgelist) // O(E)
    {
        double w = edge[0];
        int x = int(edge[1]);
        int y = int(edge[2]);
        int eid = int(edge[3]);

        // take that edge in MST if it does form a cycle
        if (s.find(x) != s.find(y)) // the source of the two sets do not match. // O(log(E))
        {
            s.unite(x, y);
            total_weight += w;
            mst_edges.push_back({eid, x,y}); // forms an edge between them
        }
    }
    return total_weight;
}

std::vector<double> maxDihedralAngleWeights(const igl::opengl::ViewerData &mesh, const Eigen::MatrixXi &dual_edges) {
    unsigned int nEdges = dual_edges.rows();
	vector<double> weight(nEdges); 
	vector<double> length(nEdges);

    Eigen::MatrixXi edges;
    getMeshEdges(mesh, edges);

	// Dihedral angle heuristic weights

    // assert(dual_edges.size() == edges.size());

	for (int ei = 0; ei < dual_edges.rows(); ei++) {
        int face0 = dual_edges.row(ei)(0);
        int face1 = dual_edges.row(ei)(1);

        if(face0 == -1 or face1 == -1) { // open mesh.. does not have a face attached to it
            weight[ei] = 1; // high weight so it does not form part of mst i don't want corner edges to be in mst
            continue;    
        }

        // face center and edge center angle... 
        Eigen::Vector3d e_center = (mesh.V.row(edges.row(ei)(0)) + mesh.V.row(edges.row(ei)(1)))/2;
        Eigen::Vector3d f0_center = (mesh.V.row(mesh.F.row(face0)(0)) + mesh.V.row(mesh.F.row(face0)(1)) + mesh.V.row(mesh.F.row(face0)(2)))/3;
        Eigen::Vector3d f1_center = (mesh.V.row(mesh.F.row(face1)(0)) + mesh.V.row(mesh.F.row(face1)(1)) + mesh.V.row(mesh.F.row(face1)(2)))/3;

        Eigen::Vector3d vf0e = f0_center - e_center;
        Eigen::Vector3d vf1e = f1_center - e_center;

        double dotProduct = vf0e.dot(vf1e) / (vf0e.norm() * vf1e.norm());

        weight[ei] = 0.5 * (dotProduct + 1);

        // double dihedral_angle = computeDihedralAngle(mesh.V, mesh.F , face0, face1);
		// weight[ei] = 0.5 * (cos(dihedral_angle) + 1);
        assert((0.0 <= weight[ei]) && (weight[ei] <= 1.0));

	}
    return weight;

}

std::vector<double> minPerimeterWeights(const igl::opengl::ViewerData &mesh, const Eigen::MatrixXi &dual_edges) {
    unsigned int nEdges = dual_edges.rows();
	vector<double> weight(nEdges); 
	vector<double> length(nEdges);
	double minLength = 1000000.0, maxLength = -1000000.0;

    Eigen::MatrixXi edges;
    getMeshEdges(mesh, edges);

	// Perimeter heuristic weights

    // assert(dual_edges.size() == edges.size());

	for (int ei = 0; ei < dual_edges.rows(); ei++) {
		Eigen::Vector3d vec = mesh.V.row(edges.row(ei)(0)) - mesh.V.row(edges.row(ei)(1));
		length[ei] = vec.norm();
		if (length[ei] < minLength)
			minLength = length[ei];
		if (length[ei] > maxLength)
			maxLength = length[ei];
	}
	// cout << " minLength = " << minLength << " maxLength = " << maxLength << endl;
	for (int ei = 0; ei < dual_edges.rows(); ei++) {
		assert(length[ei] > 0.0);
		weight[ei] = 2.0 * ((maxLength - length[ei]) / (maxLength - minLength)) - 1.0;
	}
    return weight;
}

void MST(const igl::opengl::ViewerData &mesh, const igl::opengl::ViewerData &dual_mesh, const Eigen::MatrixXi &dual_edges,
         igl::opengl::ViewerData &mst_mesh, Eigen::MatrixXi &hinge_edges, Eigen::MatrixXi &cut_edges, Eigen::MatrixXi &boundary_edges, vector<bool> &bool_dual_edges,
         vector<meshEdge> &mesh_edges, Eigen::MatrixXd &MST_color, int current_mst_heuristic)
{

    // For now all the edges of the Graph has constant value.
    // implementing max-dihedral angle and minimum perimeter heurstic together

    // Create the graph using mesh, dual mesh and dual_edges data
    // implementation - https://www.geeksforgeeks.org/kruskals-minimum-spanning-tree-algorithm-greedy-algo-2/
    Graph mst_graph(mesh.F.rows()); // equal to number of face centers...
    std::vector<double> min_per_weight = minPerimeterWeights(mesh, dual_edges);
    std::vector<double> max_da_per_weight = maxDihedralAngleWeights(mesh, dual_edges);

    for(int ei = 0; ei<dual_edges.rows(); ei++) {
        int v0 = dual_edges.row(ei)(0);
        int v1 = dual_edges.row(ei)(1);
        // mst_graph.addEdge(v0,v1, 1.0, ei); // for now just constant weight.. TODO replace this with a more suitable weight later. 

        if(current_mst_heuristic == 0)
            mst_graph.addEdge(v0,v1, max_da_per_weight[ei], ei); // for now just constant weight.. TODO replace this with a more suitable weight later. 
        else if(current_mst_heuristic == 1)
            mst_graph.addEdge(v0,v1, min_per_weight[ei], ei); // for now just constant weight.. TODO replace this with a more suitable weight later. 
        else {
            printf("Heuristic not found!!! DEBUG");
            exit(1);
        }

    }

    std::vector<std::vector<int>> mst_edges_vector;
    double mst_min_weight = mst_graph.kruskals_mst(mst_edges_vector);
    // printf("weight of mst : %f size of mst_edge_vector : %ld dual_edge number : %ld\n", mst_min_weight, mst_edges_vector.size(), dual_edges.size());
    hinge_edges.resize(mst_edges_vector.size(), 2); // size of mst_edge vector
    cut_edges.resize(dual_edges.rows() - hinge_edges.rows(), 2);  // size of total dual edges - edges in mst
    // rest are boundary edges

    sort(mst_edges_vector.begin(), mst_edges_vector.end()); // sort based on the first element of the vector of vector. which is the edge index
    // sort all the mst_edges based on their edge index.. 
    int total_mst_edges = mst_edges_vector.size();
    int total_cut_edges = dual_edges.rows() - mst_edges_vector.size();


    // INIT all the return parameters with default values
    MST_color.resize(dual_edges.rows()*2, 3); // number of edges time 3 for RGB value // since visual dual edges are twice the number

    bool_dual_edges.resize(dual_edges.rows());
    uint mei = 0; // iterator for all edges
    uint ci = 0; // iterator for cut edges
    mesh_edges.clear(); // empty out prev stuff before putting in new stuff

    for (size_t ei = 0; ei < dual_edges.rows(); ei++) { // iterator for mst_edge 
        
        // prepare data for meshEdge
        meshEdge edge;
        edge.faces = make_pair(dual_edges.row(ei)[0], dual_edges.row(ei)[1]);
        edge.orderedVertices = getCommonVertices(mesh, dual_edges.row(ei)[0], dual_edges.row(ei)[1], true);
        
        if(mst_edges_vector.size() > 0) {
            std::vector<int> elem =  mst_edges_vector[0];
            int edge_index = elem[0];
            if (edge_index == ei) {
                // printf("P elem index : %d %d %d\n", edge_index, elem[1], elem[2]);
                bool_dual_edges[ei] = true;
                hinge_edges.row(mei) = dual_edges.row(ei);    
                
                edge.isHinge = true;

                mst_edges_vector.erase(mst_edges_vector.begin());
                MST_color.row(ei*2) = red_color;
                MST_color.row(ei*2+1) = red_color; // color edges are same as visual edges which are twice and placed sequentially..
                mei++;  // mst edges which are to be hinged are in red
            }
            else{
                // printf("N elem index : %d %d %d\n", edge_index, elem[1], elem[2]);
                bool_dual_edges[ei] = false;
                cut_edges.row(ci) = dual_edges.row(ei);
                
                edge.isHinge = false;
                
                // edge1.faces = make_pair(dual_edges.row(ei)[1], -1);
                // edge1.isHinge = false;
                // mesh_edges.push_back(edge1);

                MST_color.row(ei*2) = blue_color;
                MST_color.row(ei*2+1) = blue_color; // cut edges are in blue
                ci++;
            }
        }
        else {
            // mst-edges are now done. Just finish the rest with cut-edges
            bool_dual_edges[ei] = false;
            cut_edges.row(ci) = dual_edges.row(ei);

            edge.isHinge = false;

            MST_color.row(ei*2) = blue_color;
            MST_color.row(ei*2+1) = blue_color;
            ci++;
        }
        mesh_edges.push_back(edge);
        // find a fix to include the edges that have boundary...
    }
    

    int ei = 0; 
    for(auto edge : mesh_edges) {
        // printf("e : %d %d %d isHInge : %d\n", ei, get<0>(edge.faces), get<1>(edge.faces), edge.isHinge);
        ei++;
    }
    // exit(1);

    for (size_t bi = 0; bi < boundary_edges.rows(); bi++) { // iterator for boundary_edges
        meshEdge edge;
        edge.faces = make_pair(boundary_edges.row(bi)[0], -1); // face and -1
        edge.orderedVertices = make_pair(boundary_edges.row(bi)[1], boundary_edges.row(bi)[2]);
        edge.isHinge = false;
        edge.isBoundary = true;
        mesh_edges.push_back(edge);
    }
}

void setMSTMesh(const igl::opengl::ViewerData &mesh, const Eigen::MatrixXi &edges, Eigen::MatrixXd MST_edge_color, igl::opengl::ViewerData &mst_mesh) {
    // remember MST_edge-color was set for visual edges so only take the odd or even value and skip the other
    mst_mesh.clear_edges();
    Eigen::MatrixXd color;
    color.resize(int(MST_edge_color.rows()/2), 3);
    if(MST_edge_color.size() == 0) {
        color.resize(edges.rows(), 3);
        for (size_t i = 0; i < edges.rows(); i++) {
            color.row(i) = black_color;
        }
    }
    else {
        for (size_t i = 0; i < int(MST_edge_color.rows()/2); i++) {
            color.row(i) = MST_edge_color.row(2*i);
        }
    }
        
    mst_mesh.set_mesh(mesh.V, mesh.F);
    mst_mesh.show_lines = false; // it will only show the wireframe for things whose edges are set explicity
    mst_mesh.set_edges(mesh.V, edges, color);
    mst_mesh.line_width = 2;
}