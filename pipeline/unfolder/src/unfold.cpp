#include "common.h"
double trans_pos = 1000; // TODO weird floating point error when trans_pos == 5000 DEBUG later.

void fixMeshOrder(igl::opengl::ViewerData &mesh, vector<Eigen::Vector3d> &sheet_vertices, vector<Eigen::Vector3i> &sheet_indices, 
                    map<int, int> &world2flatmap, vector<vector<int>> &flatFaceVisitOrder)
{

    vector<Eigen::Vector3i> sheet_indices_ordered(sheet_indices.size());

    map<int, int> flat2worldmap;
    for(auto kv : world2flatmap) {
        flat2worldmap.insert(make_pair(kv.second, kv.first));
    }

    for(auto &patch : flatFaceVisitOrder) {
        for(auto &f : patch) {
            f = flat2worldmap[f];
        }
    }

    for (int fi = 0; fi < mesh.F.rows(); fi++) {
        int ffi = world2flatmap[fi];
        Eigen::Vector3d wv0, wv1, wv2, fv0, fv1, fv2;
        int wvi0 = mesh.F(fi, 0), wvi1 = mesh.F(fi, 1), wvi2 = mesh.F(fi, 2);
        int fvi0 = sheet_indices[ffi][0], fvi1 = sheet_indices[ffi][1], fvi2 = sheet_indices[ffi][2];
        wv0 = mesh.V.row(wvi0); wv1 = mesh.V.row(wvi1); wv2 = mesh.V.row(wvi2);
        fv0 = sheet_vertices[fvi0];
        fv1 = sheet_vertices[fvi1];
        fv2 = sheet_vertices[fvi2];
        // printf("wfi: %d wv0: %d wv1: %d wv2: %d\n", fi, wvi0, wvi1, wvi2);
        // printf("ffi: %d fv0: %d fv1: %d fv2: %d\n", ffi, fvi0, fvi1, fvi2);
        sheet_indices_ordered[fi] = sheet_indices[ffi];
        world2flatmap[fi] = fi;
    }
    sheet_indices = sheet_indices_ordered;
    // world2flatmap is just identity after this stage so one can just dicard it


}

void unfoldMesh(igl::opengl::ViewerData &mesh, igl::opengl::ViewerData &dual_mesh, const Eigen::MatrixXi &dual_edges, const Eigen::MatrixXi &hinge_edges,
                vector<meshEdge> &mesh_edges_data, igl::opengl::ViewerData &sheet_mesh, igl::opengl::ViewerData &sheet_clean_mesh, std::vector<std::vector<int>> &sheet_qindices_hinge, 
                vector<vector<int>> &flatFaceVisitOrder_return, map<int, triplet> &meshFace2Edge, bool bool_mesh_cut, bool merge_patches, int bp,
                bool with_offset, double obj_scale, double drill_bit_radius, 
                double user_margin, int min_patches, bool adaptiveScaling, bool useHalfHinges)
{

    // mesh_edges_data only contains the edges that are cut after performing the mst. may further cut down based on cut and merge operations
    // first find the skeleton of the dual mesh i.e. the starting and the end of the MST and the whole chain

    std::map<int, int> face2occurannce;
    std::map<int, vector<int>> face2dualEdge;
    createFace2dualEdgeMap(face2dualEdge, hinge_edges);
    createFace2occuranceMap(face2occurannce, hinge_edges);

    // first face with only hinge edge.. starts the spine of the unfolding...
    vector<int> term_faces; // faces that can act as an ending face for the unfolding. 
    for (const auto& kv: face2occurannce) { // kv is the key value pair
        // std::cout << kv.first << " has value " << kv.second << std::endl;
        if(kv.second == 1) {
            term_faces.push_back(kv.first);
        }
    }

    // for (const auto& kv: face2dualEdge) { // kv is the key value pair
    //     std::cout << "face id : " <<  kv.first << " has edges : ";
    //     for (size_t i = 0; i < kv.second.size(); i++) {
    //         cout << (kv.second)[i] << " ";
    //     } cout << endl;
    // }

    // for( auto f: term_faces) {
    //     cout << "term_faces : " << f << endl;
    // }
    if (term_faces.size() == 0) {
        cout << "Something is wrong DEBUG. Exiting now...\n";
        exit(1);
    }
    int fnode = term_faces[0];
    // invariant each face is visited once.
    std::stack<int> dfs; // data-structure for traversing the tree..
    dfs.push(fnode); 
    vector<bool> visited(mesh.F.rows()); // faces to visit
    vector<int> visit_order;
    int f0, f1;
    while(dfs.size() > 0) { // still try to flatten out the mesh // perform depth first search and flatten things out...
        // hinge_edges and f2dualEdge acts as each other's duals
        int node = dfs.top(); dfs.pop(); // extract the top one and pop it out
        if(visited[node]) // do not revisit nodes
            continue;  
        // find the dual edges that it is connected to
        vector<int> f2dualMap = face2dualEdge[node]; 
        for (size_t dei = 0; dei < f2dualMap.size(); dei++) {
            int hi = f2dualMap[dei];
            f0 = hinge_edges.row(hi)(0);
            f1 = hinge_edges.row(hi)(1);
            int new_node;
            (f0==node ? new_node = f1: new_node = f0); // push the one not visited node in

            if(new_node == -1) { // it is the end part return
                continue;
            } 

            if(visited[new_node])
                continue;
            dfs.push(new_node);
            // printf("Adding Face : %d %d %d\n", f0, f1, dfs.top());
        }
        visited[node] = true;
        visit_order.push_back(node);
        // printf("Faces left to be visited : %ld\n", visited.size() - std::accumulate(visited.begin(), visited.end(), 0));
        
    }

    // vertexPair2Edge map
    map<pair<int,int>, int> vertexPair2Edge;
    for(int ei = 0; ei < mesh_edges_data.size(); ei++) {
        auto edge = mesh_edges_data[ei];
        auto orderedVertexPair = edge.orderedVertices;
        if (vertexPair2Edge.find(orderedVertexPair) == vertexPair2Edge.end()) {
            vertexPair2Edge[orderedVertexPair] = ei; 
            // printf("ei : %d vertex pair : %d %d\n", ei, orderedVertexPair.first, orderedVertexPair.second);
        }
    }
    for (int fi = 0; fi < mesh.F.rows(); fi++) { 
        // change it into something that can iterate Es
        int v0 = mesh.F.row(fi)[0], v1 = mesh.F.row(fi)[1], v2 = mesh.F.row(fi)[2];

        auto pair0 = v0 < v1 ? make_pair(v0, v1) : make_pair(v1, v0);
        auto pair1 = v1 < v2 ? make_pair(v1, v2) : make_pair(v2, v1);
        auto pair2 = v2 < v0 ? make_pair(v2, v0) : make_pair(v0, v2);

        meshFace2Edge[fi] = make_tuple(vertexPair2Edge[pair0], vertexPair2Edge[pair1], vertexPair2Edge[pair2]);

        // self consistency test..
        // int e0 = std::get<0>(meshFace2Edge[fi]), e1 = std::get<1>(meshFace2Edge[fi]), e2 = std::get<2>(meshFace2Edge[fi]);
        // printf("Face : %d vertices : %d %d %d\nedges : [%d: %d %d] [%d: %d %d] [%d: %d %d] \n", fi, v0, v1, v2, e0, mesh_edges_data[e0].faces.first, mesh_edges_data[e0].faces.second,
        //        e1, mesh_edges_data[e1].faces.first, mesh_edges_data[e1].faces.second,
        //        e2, mesh_edges_data[e2].faces.first, mesh_edges_data[e2].faces.second);
    }

    // for (int i = 0; i < mesh_edges_data.size(); i++) { 
        // // self consistency test!!
        // printf("edge : %d face : %d %d offset : %f isHinge :%d v0 : %d v1: %d:again edge: %d\n",
        //        i, edge.faces.first, edge.faces.second, edge.offset, edge.isHinge, edge.orderedVertices.first, edge.orderedVertices.second, vertexPair2Edge[edge.orderedVertices]);
        // outfile << e->id << " " << e->offset << " " << e->isHinge << endl;
    // }

    vector<Eigen::Vector3d> sheet_vertices;
    vector<Eigen::Vector3i> sheet_indices;
        
    // unfold via mst
    // debug hinge edges
    // this can also change as higne_edges are updated after a sucucesfull cut

    Eigen::MatrixXi hinge_edges_temp = hinge_edges; // as while cutting i get rid of some of the edges...
    unfoldViaMST(visit_order, mesh, hinge_edges_temp, dual_edges, mesh_edges_data, 
                meshFace2Edge, sheet_vertices, sheet_indices, sheet_qindices_hinge, 
                sheet_clean_mesh, face2dualEdge, flatFaceVisitOrder_return, 
                bool_mesh_cut, merge_patches, bp, with_offset, obj_scale, 
                drill_bit_radius, user_margin, min_patches, adaptiveScaling, useHalfHinges);

    Eigen::MatrixXd sheet_V; sheet_V.resize(sheet_vertices.size(), 3);
    Eigen::MatrixXi sheet_F; sheet_F.resize(sheet_indices.size(), 3);

    for (size_t fi = 0; fi < sheet_vertices.size(); fi++)
        sheet_V.row(fi) = sheet_vertices[fi];

    for (size_t ii = 0; ii < sheet_indices.size(); ii++)
        sheet_F.row(ii) = sheet_indices[ii];

    if(sheet_mesh.V.size() > 0) 
        sheet_mesh.clear();
    sheet_mesh.set_mesh(sheet_V, sheet_F);
    sheet_mesh.line_width = 5;

    // sanity check if orientation is correct or not!
    if(not with_offset) {
        for (int fi = 0; fi < mesh.F.rows(); fi++) {
            int ffi = fi;
            Eigen::Vector3d wv0, wv1, wv2, fv0, fv1, fv2;
            wv0 = mesh.V.row(mesh.F(fi,0)); wv1 = mesh.V.row(mesh.F(fi, 1)); wv2 = mesh.V.row(mesh.F(fi,2));
            fv0 = sheet_mesh.V.row(sheet_mesh.F(ffi,0)); fv1 = sheet_mesh.V.row(sheet_mesh.F(ffi, 1)); fv2 = sheet_mesh.V.row(sheet_mesh.F(ffi,2));
            Eigen::Vector3d zvec = Eigen::Vector3d(0,0,1);
            Eigen::Vector3d worldLengths1 = Eigen::Vector3d((wv1 - wv0).norm(), (wv2 - wv1).norm(), (wv0 - wv2).norm());
            Eigen::Vector3d worldLengths2 = Eigen::Vector3d((wv2 - wv1).norm(), (wv0 - wv2).norm(), (wv1 - wv0).norm());
            Eigen::Vector3d worldLengths3 = Eigen::Vector3d((wv0 - wv2).norm(), (wv1 - wv0).norm(), (wv2 - wv1).norm());
            
            Eigen::Vector3d flatLengths = Eigen::Vector3d((fv1 - fv0).norm(), (fv2 - fv1).norm(), (fv0 - fv2).norm());
            double eps = 1e-8;

            if((worldLengths1 - flatLengths).norm() < eps or (worldLengths2 - flatLengths).norm() < eps or (worldLengths3 - flatLengths).norm() < eps) {
                // printf("fi : %d is consistent\n", fi);
            }
            else {
                printf("\n\nSanity check fails DEBUG HERE!!\n");
                printf("fi : %d ffi : %d is not consistent\n", fi, ffi);
                printf("fi : %d edge lengths world : %f %f %f flat : %f %f %f \n", fi, (wv1 - wv0).norm(), \
                (wv2 - wv1).norm(), (wv0 - wv2).norm(), (fv1 - fv0).norm(), (fv2 - fv1).norm(), (fv0 - fv2).norm());
                exit(1);
            }
            // printf("fi : %d orientation world : %f flat : %f\n", fi, zvec.dot((wv1-wv0).cross(wv2-wv0)), zvec.dot((fv1-fv0).cross(fv2-fv0)));
        }
    }

    // Consistency check. 
    // for(int tri_i = 0 ; tri_i < mesh.F.rows(); tri_i++) {

    //     Eigen::Vector3d wv0 = mesh.V.row(mesh.F.row(tri_i)[0]);
    //     Eigen::Vector3d wv1 = mesh.V.row(mesh.F.row(tri_i)[1]);
    //     Eigen::Vector3d wv2 = mesh.V.row(mesh.F.row(tri_i)[2]);
    //     vector<Eigen::Vector3d> wvs = {wv0, wv1, wv2};

    //     Eigen::Vector3d sv0 = sheet_vertices[sheet_indices[tri_i][0]];
    //     Eigen::Vector3d sv1 = sheet_vertices[sheet_indices[tri_i][1]];
    //     Eigen::Vector3d sv2 = sheet_vertices[sheet_indices[tri_i][2]];

    //     printf("Triangle : %d\n", tri_i);
    //     printf("Edge length world : %f %f %f\n", sqrt((wv0-wv1).dot(wv0-wv1)), sqrt((wv1-wv2).dot((wv1-wv2))), sqrt((wv2-wv0).dot(wv2-wv0)));
    //     printf("Edge length sheet : %f %f %f\n", sqrt((sv0-sv1).dot(sv0-sv1)), sqrt((sv1-sv2).dot((sv1-sv2))), sqrt((sv2-sv0).dot(sv2-sv0)));
    //     // printf("Sheet offsets : %f %f %f\n\n", sheet_offsets[tri_i][0], sheet_offsets[tri_i][1], sheet_offsets[tri_i][2])

    // }
}

void unfoldViaMST(const vector<int> &visit_order, igl::opengl::ViewerData &mesh, Eigen::MatrixXi &hinge_edges, const Eigen::MatrixXi &dual_edges, 
                vector<meshEdge> &mesh_edges_data, map<int, triplet> &meshFace2Edge, vector<Eigen::Vector3d> &sheet_vertices, vector<Eigen::Vector3i> &sheet_indices, 
                std::vector<std::vector<int>> &sheet_qindices_hinge, igl::opengl::ViewerData &sheet_clean_mesh, std::map<int, vector<int>> face2dualEdge, 
                vector<vector<int>> &flatFaceVisitOrder_return,  bool bool_mesh_cut, bool merge_patches, int bp, bool with_offset, double obj_scale, 
                double drill_bit_radius, double user_margin, int min_patches, bool adaptiveScaling, bool useHalfHinges)
{

    map<int, vector<int>> world2flatVind;
    int v0,v1,v2;
    int pfv0,pfv1,pfv2;
    vector<bool> isCreated(mesh.F.rows());

    Eigen::MatrixXi mesh_EV, mesh_FE, mesh_FV;
    getMeshData(mesh, mesh_EV, mesh_FE, mesh_FV);

    // break point handle
    if(bp < 0 or bp > visit_order.size()-1)
        bp = visit_order.size()-1;

    bool mstFail = false;

    map<int, int> world2flatFaceMap;

    // the order of edges are v0-v1, v1-v2 and v2-v0 
    vector<vector<int>> flatFaceVisitOrder; // visit order in which flat faces are added per patch starting with only 1 patch... 
    flatFaceVisitOrder.push_back(vector<int>{}); // start by providing a fresh patch to unfold...

    // for(auto f: visit_order) {
    //     printf("%d ", f);
    // } printf("\n");

    for(int fi = 0; fi < visit_order.size(); fi++) { // this is in global face index
        int face_id = visit_order[fi];
        Eigen::Vector3d vpos0, vpos1, vpos2;
        v0 = mesh_FV.row(face_id)(0); vpos0 = mesh.V.row(v0);
        v1 = mesh_FV.row(face_id)(1); vpos1 = mesh.V.row(v1);
        v2 = mesh_FV.row(face_id)(2); vpos2 = mesh.V.row(v2);
        Eigen::Vector3d vpos0_ref, vpos1_ref, vpos2_ref; // configuration on the plane
        flattenTriangle(vpos0, vpos1, vpos2, vpos0_ref, vpos1_ref, vpos2_ref);
        double gl0 = (vpos1-vpos0).norm();
        double gl1 = (vpos2-vpos1).norm();
        double gl2 = (vpos0-vpos2).norm();


        // printf("vpos0_ref : %f %f %f\n", vpos0_ref(0), vpos0_ref(1), vpos0_ref(2));
        // printf("vpos1_ref : %f %f %f\n", vpos1_ref(0), vpos1_ref(1), vpos1_ref(2));
        // printf("vpos2_ref : %f %f %f\n", vpos2_ref(0), vpos2_ref(1), vpos2_ref(2));
    
        // printf("Global Edge lengths %f %f %f\n", gl0, gl1, gl2);
        // printf("Local Edge lengths  %f %f %f\n", (vpos0_ref-vpos1_ref).norm(),(vpos2_ref-vpos1_ref).norm(),(vpos0_ref-vpos2_ref).norm() ); // flattening length works!!

        if(fi!= 0) { // i.e. it needs to connect to someone else to create itself
            // modify ref position to match the previous triangle 
            // their origin match but the edges are mismatched...
            // first find the edge that they share.. 
            
            vector<int> f2dualMap = face2dualEdge[face_id];
            // find the face that is trying to create this new face.. 
            int prev_face_id = -1;
            int dual_edge = -1;
            findPreviousFacePatch(prev_face_id, dual_edge, f2dualMap, hinge_edges, face_id, isCreated);                       
            // printf("Previous Face that is creating this one is : %d %d dual edge : %d\n", prev_face_id, face_id, dual_edge);
            if(prev_face_id == -1) {
                printf("Something went wrong debug\n");
                exit(1);
            }
            int pfi = world2flatFaceMap[prev_face_id]; // previous face id 
            
            pfv0 = sheet_indices[pfi](0); // these vertices should be in flat vertex format so that I can index it using them
            pfv1 = sheet_indices[pfi](1);
            pfv2 = sheet_indices[pfi](2);

            // dual edge --> actual edge is the fact that they share the same index... 
            int shared_v0, shared_v1;
            // find the shared edge using hinge edges
            int shared_edge;
            findCommonEdge(hinge_edges, dual_edge, mesh_FE, shared_edge);

            // double offset = computeoffset(dihedralAngle, drill_bit_radius);

            shared_v0 = mesh_EV.row(shared_edge)(0); 
            shared_v1 = mesh_EV.row(shared_edge)(1);

            int prev_flatv0=-1, prev_flatv1=-1;
            int sid0 = -1, sid1 = -1, sid2 = -1;// the leftover index
            int pfsid0 = -1, pfsid1 = -1, pfsid2 = -1;
            // printf("Glo v0 : %d v1 : %d, v2 : %d, shared_v0 : %d, shared_v1 : %d \n", v0,v1,v2,shared_v0,shared_v1);
            getIndices(sid0, sid1, sid2, shared_v0, shared_v1, v0,v1,v2); // this in the world config matching


            if(world2flatVind[shared_v0].size() > 1 or world2flatVind[shared_v1].size() > 1) {
                // printf("The edge between %d %d is a cut edge and thus multiple entries exists choose the best one\n", shared_v0, shared_v1);
                vector<int> flatv0s = world2flatVind[shared_v0]; 
                vector<int> flatv1s = world2flatVind[shared_v1];
                double glob_dist = (mesh.V.row(shared_v0)- mesh.V.row(shared_v1)).norm();
                bool found = false;
                for(auto fvi : flatv0s) {
                    for(auto fvj : flatv1s) {
                        double loc_dist = (sheet_vertices[fvi] - sheet_vertices[fvj]).norm();
                        // printf("Distance diff : %f\n", fabs(glob_dist-loc_dist));
                        if( fabs(loc_dist-glob_dist) < 1e-6 ) {
                            prev_flatv0 = fvi; 
                            prev_flatv1 = fvj; 
                            int result = getIndices2(pfsid0, pfsid1, pfsid2, prev_flatv0, prev_flatv1, pfv0,pfv1,pfv2);
                            if(result != -1) {
                                // printf("New flatten vertices to use are -- %d %d\n", prev_flatv0,prev_flatv1);
                                found = true;
                                break;
                            } else {
                                // printf("The mesh is too regular and is hence giving mixed vertices. Don't worry fix is in place for this.\n");
                            }
                        }
                        if(found)
                            break;
                    }
                }
                if(prev_flatv0 == -1 or prev_flatv1 == -1) {
                    printf("Something went wrong there... DEBUG!!!! \n ");
                    exit(1);
                }
            }
            else { // atleast one entry is guaranteed for a shared edge.. the way mst is unfolded..
                prev_flatv0 = world2flatVind[shared_v0][0];
                prev_flatv1 = world2flatVind[shared_v1][0];
            }

            vector<int> vinds = {v0,v1,v2}; // just to access them easily
            vector<Eigen::Vector3d> vpos_refs = {vpos0_ref, vpos1_ref, vpos2_ref};
            

            // all triangles have same orientation so there should only be 3 cases... 
            // printf("Glo pfv0 : %d pfv1 : %d, pfv2 : %d, prev_flatv0 : %d, prev_flatv1 : %d \n", pfv0,pfv1,pfv2,prev_flatv0,prev_flatv1);
            getIndices(pfsid0, pfsid1, pfsid2, prev_flatv0, prev_flatv1, pfv0,pfv1,pfv2);

            // printf("Ref pfsid0 : %d pfsid1 : %d, pfsid2 : %d, prev_flatv0 : %d, prev_flatv1 : %d \n", pfsid0,pfsid1,pfsid2,prev_flatv0,prev_flatv1);
            // printf("Ref sid0 : %d sid1 : %d, sid2 : %d, prev_flatv0 : %d, prev_flatv1 : %d \n", sid0,sid1,sid2,prev_flatv0,prev_flatv1);

            // First apply translation
            connectTriangle2Previous(sheet_vertices, prev_flatv0, prev_flatv1, sid0, sid1, sid2, vpos_refs);

            bool triangleIntersectionCheck1 = checkTriangleTriangleIntersection2(vpos_refs, sheet_vertices, sheet_indices);
            // printf("Intersection : %d %d Number of badTriangles1 : %d and badTriangles2 : %d\n", triangleIntersectionCheck1, triangleIntersectionCheck2, badTriangles1, badTriangles2);

            // Two of them should be previous vertices only 1 new edge should be added
            // When sharing dofs

            sheet_vertices.push_back(vpos_refs[sid2]);
            int newfv0 = -1, newfv1 = -1, newfv2 = -1; 
            // THIS function ensures that the vertices are in order!!!
            chooseCorrectOrder(newfv0, newfv1, newfv2, sheet_vertices, prev_flatv0, prev_flatv1, fi+2, gl0, gl1, gl2);
            sheet_indices.push_back(Eigen::Vector3i(newfv0, newfv1, newfv2)); // previous 2 and the new one
            // printf("Local Edge lengths  %f %f %f\n", (sheet_vertices[newfv0] - sheet_vertices[newfv1]).norm(), 
                        // (sheet_vertices[newfv1] - sheet_vertices[newfv2]).norm(), 
                        // (sheet_vertices[newfv2] - sheet_vertices[newfv0]).norm()); // flattening length works!!

            // -------- Handle offsets for the flattriangle ---------------
            double dihedralAngle = computeDihedralAngle(mesh.V, mesh_FV, face_id, prev_face_id); // would be helpful while creating the offset surface for the next implementation
            // double offset = dihedralAngle * 180 / M_PI; // DEBUG STATEMENT
            double offset = computeOffset(dihedralAngle, obj_scale, drill_bit_radius);

            // printf("face : %d pFace : %d\n", fi, prev_face_id);
            int commonEdgeId = findCommonFaceEdge(meshFace2Edge[face_id], meshFace2Edge[prev_face_id]);
            mesh_edges_data[commonEdgeId].offset = offset; 
            
            // -------- END Handle offsets for the flattriangle ---------------


            // update world2flatvind map
            if(world2flatVind.find(vinds[sid2]) == world2flatVind.end()) {
                world2flatVind.insert({vinds[sid2], vector<int>{fi+2} });
            } else {
                world2flatVind[vinds[sid2]].push_back(fi+2);
            }

            if(triangleIntersectionCheck1) {
                // printf("MST does not result in an intersection free unfolding need to do something different... \n");
                mstFail = true; 
            }
        }

        else {
            // just push the vertices in the buffer...
            sheet_vertices.push_back(vpos0_ref);
            sheet_vertices.push_back(vpos1_ref);
            sheet_vertices.push_back(vpos2_ref);
            sheet_indices.push_back(Eigen::Vector3i(3*fi, 3*fi+1, 3*fi+2));
            // since it is first time insertion i can just add it via vector addition don't need to check for existence..
            world2flatVind.insert({v0, vector<int>{3*fi+0} });
            world2flatVind.insert({v1, vector<int>{3*fi+1} });
            world2flatVind.insert({v2, vector<int>{3*fi+2} });

        }
        isCreated[face_id] = true;
        // printf("Face created : %d flat id : %ld \n", face_id, sheet_indices.size()-1);
        world2flatFaceMap.insert({face_id, fi}); 
        flatFaceVisitOrder[0].push_back(fi);
        // printf("Face created (W): %d (F): %d\n", face_id, world2flatFaceMap[face_id]);
        // printf("indices used : %d %d %d\n", sheet_indices[fi](0), sheet_indices[fi](1), sheet_indices[fi](2));

        if(fi == bp) {
            break;
        }
    }

    // sanity check
    for (int fi = 0; fi < sheet_indices.size(); fi++) {
        Eigen::Vector3d vpos0, vpos1, vpos2;
        int v0 = sheet_indices[fi][0]; vpos0 = sheet_vertices[v0];
        int v1 = sheet_indices[fi][1]; vpos1 = sheet_vertices[v1];
        int v2 = sheet_indices[fi][2]; vpos2 = sheet_vertices[v2];
        if( not isPositiveOrientation(vpos0 , vpos1 , vpos2 )) {
            printf("SOMETHING WRONG WITH ORIENTATION OF THE TRIANGLES JUST AFTER UNFOLDING BEFORE CUT DEBUG!!!\n");
            exit(1);
        }
    }
    
    // allign the flat faces to map the world faces. Just to avoid the jumbled order
    fixMeshOrder(mesh, sheet_vertices, sheet_indices, world2flatFaceMap, flatFaceVisitOrder); // this will result in worldFlatFacemap being identity
    // mesh Order is fixed so I can directly poll flat faces and world faces!!

    if(mstFail)  {
        printf("The unfolding has intersections resolve them!! \n");
        vector<pair<int, int>> intersectingFaceList; // these are already the indices of triangles in plane
        
        vector<int> face2Patch(sheet_indices.size(), 0); // currently all the faces belong to patch 0 
        if(bool_mesh_cut) {
            applyCutEdges(mesh, sheet_vertices, sheet_indices, hinge_edges, face2Patch, 
            world2flatVind, flatFaceVisitOrder, mesh_edges_data, meshFace2Edge);        
        }
        if(bool_mesh_cut and merge_patches) {
            printf("Trying to merge n : %d patches that I have\n", *max_element(face2Patch.begin(), face2Patch.end()) + 1 );
            mergePatches(mesh, sheet_vertices, sheet_indices, hinge_edges, dual_edges, mesh_edges_data, meshFace2Edge, face2Patch,
                         world2flatVind, flatFaceVisitOrder,
                         obj_scale, drill_bit_radius, min_patches);
        } 

        
        printf("Total number of patches : %ld\n", flatFaceVisitOrder.size());
        for(int pi = 0; pi < flatFaceVisitOrder.size(); pi++) {
            printf("patch index : %d faces : ", pi);
            for(auto ff: flatFaceVisitOrder[pi]) {
                printf("%d ", ff);
            }
            cout << endl;
        }        

        printf("Finish performing basic unfolding of the sheet mesh\n");
    }
    else {
        // correct the flatFace visit order. 
        printf("Finish performing basic unfolding of the sheet mesh\n");
    }

    // before offseting check for orientations and fix wrongly oriented triangles
    correctOrientation(mesh, sheet_vertices, sheet_indices);

    if(with_offset) { 
        // prepare a clean mesh as well which is without any offsets
        Eigen::MatrixXd sheet_CV;
        sheet_CV.resize(sheet_vertices.size(), 3);
        Eigen::MatrixXi sheet_CF;
        sheet_CF.resize(sheet_indices.size(), 3);

        for (size_t fi = 0; fi < sheet_vertices.size(); fi++)
            sheet_CV.row(fi) = sheet_vertices[fi];

        for (size_t ii = 0; ii < sheet_indices.size(); ii++)
            sheet_CF.row(ii) = sheet_indices[ii];

        if (sheet_clean_mesh.V.size() > 0)
            sheet_clean_mesh.clear();
        sheet_clean_mesh.set_mesh(sheet_CV, sheet_CF);
        sheet_clean_mesh.line_width = 5;

        printf("Applying dihedral angle dependent offset to each triangle face\n");

        Eigen::MatrixXi flat_hinge_edges;
        getFlatHinges(flat_hinge_edges, hinge_edges);
        
        std::vector<Eigen::Vector3d> sheet_vertices_offset, sheet_vertices_hinge;
        std::vector<Eigen::Vector3i> sheet_indices_offset(sheet_indices.size(), Eigen::Vector3i(0,0,0)), sheet_indices_hinge; 

        // for each patch add offset surfaces based on the flat face visitOrder and offsetvalues
        applyOffset(mesh, sheet_vertices, sheet_indices, flatFaceVisitOrder, flat_hinge_edges, 
                    sheet_vertices_offset, sheet_indices_offset,
                    sheet_vertices_hinge, sheet_indices_hinge, sheet_qindices_hinge,
                    mesh_edges_data, meshFace2Edge, drill_bit_radius, user_margin, adaptiveScaling, useHalfHinges);

        sheet_vertices = sheet_vertices_offset;
        sheet_indices = sheet_indices_offset;

        int indexing_offset = sheet_vertices.size();
        sheet_vertices.insert(sheet_vertices.end(), sheet_vertices_hinge.begin(), sheet_vertices_hinge.end());
        for(auto &hinge_index: sheet_indices_hinge) {
            hinge_index += Eigen::Vector3i(indexing_offset, indexing_offset, indexing_offset);
        }
        for(auto &quad_index: sheet_qindices_hinge) {
            for(int i = 0; i < 4; i++) {
                quad_index[i] += indexing_offset;
            }
        }

        sheet_indices.insert(sheet_indices.end(), sheet_indices_hinge.begin(), sheet_indices_hinge.end());
        printf("sheet_mesh has %ld vertices\n", sheet_vertices.size());
    }

    // printf("Sheet Vertices : \n");
    // for(int i =0; i<sheet_vertices.size(); i++) {
    //     printf("Vertex : %d values : %f %f %f\n", i, sheet_vertices[i](0), sheet_vertices[i](1), sheet_vertices[i](2));
    // }
    // printf("Sheet indices : \n");
    // for(int i =0; i<sheet_indices.size(); i++) {
        // printf("face : %d indices : %d %d %d\n", i, sheet_indices[i](0), sheet_indices[i](1), sheet_indices[i](2));
    // }

    flatFaceVisitOrder_return = flatFaceVisitOrder;
}

void applyOffsetStep1(const vector<int> &visitOrder, const std::vector<Eigen::Vector3d> &sheet_vertices, 
                    const std::vector<Eigen::Vector3i> &sheet_indices, const Eigen::MatrixXi &flat_hinge_edges, 
                    std::map<int, vector<int>> &face2dualEdge, vector<meshEdge> &mesh_edges_data, 
                    map<int, triplet> &meshFace2Edge, std::vector<Eigen::Vector3d> &sheet_vertices_offset, 
                    std::vector<Eigen::Vector3i> &sheet_indices_offset, map<int, shared_ptr<transNode>> &triangleNodes,
                    float drill_bit_radius) {
    int indexStart;
    vector<bool> isVisited(sheet_indices.size(), false); // number of faces and false // don't worry i can take all the faces // in the plane 

    for(int vi=0; vi <visitOrder.size(); vi++) {
        indexStart = sheet_vertices_offset.size();
        int curr_face_id = visitOrder[vi];
        triplet efaces = meshFace2Edge[curr_face_id];
        vector<double> offset = {mesh_edges_data[get<1>(efaces)].offset, mesh_edges_data[get<2>(efaces)].offset, mesh_edges_data[get<0>(efaces)].offset};

        int v0 = sheet_indices[curr_face_id](0), v1 = sheet_indices[curr_face_id](1), v2 = sheet_indices[curr_face_id](2);
        Eigen::Vector3d vpos0 = sheet_vertices[v0], vpos1 = sheet_vertices[v1], vpos2 = sheet_vertices[v2];

        // change the vertex coordinate according to v0
        performTriangleOffset(vpos0, vpos1, vpos2, offset[0]); // in place change

        // change the vertex coordinate according to v1
        performTriangleOffset(vpos1, vpos2, vpos0, offset[1]);

        // change the vertex coordinate according to v2
        performTriangleOffset(vpos2, vpos0, vpos1, offset[2]);

        // sheet_vertices_offset
        sheet_vertices_offset.push_back(vpos0);
        sheet_vertices_offset.push_back(vpos1);
        sheet_vertices_offset.push_back(vpos2);

        sheet_indices_offset[curr_face_id] = (Eigen::Vector3i(indexStart, indexStart+1, indexStart+2));
        
        if(vi!= 0) { // for the first vertex don't have to do all this hardwork just shrinking the triangle is fine..
            int prev_face_id=-1, flat_dual_edge = -1; // in flat face configuration...
            vector<int> f2dualMap = face2dualEdge[curr_face_id];
            findPreviousFacePatch(prev_face_id, flat_dual_edge, f2dualMap, flat_hinge_edges, curr_face_id, isVisited);            
            // printf("current face : %d and previous face %d connected by flat_dual_edge : %d \n", curr_face_id, prev_face_id, flat_dual_edge); 
            triplet efaces1 = meshFace2Edge[prev_face_id];       
            int commonEdgeId = findCommonFaceEdge(meshFace2Edge[curr_face_id], meshFace2Edge[prev_face_id]);
            if(commonEdgeId == -1) {
                printf("Faces : %d %d does not have a common hinge edge!!!\n", curr_face_id, prev_face_id);
            }

            bool isHalfHinge = mesh_edges_data[commonEdgeId].isHalfHinge; 
            shared_ptr<transNode> node_vi = findCorrectDistanceBetweenTriangle(sheet_indices, 
                    sheet_indices_offset, sheet_vertices_offset, curr_face_id, prev_face_id, drill_bit_radius, isHalfHinge);
            node_vi->parent = triangleNodes[prev_face_id];
            node_vi->translation = node_vi->translation + node_vi->parent->translation;
            node_vi->parent->kids.push_back(node_vi);
            triangleNodes[curr_face_id] = node_vi;            
        }
        else {
            shared_ptr<transNode> node = make_shared<transNode>();
            node->id = vi; 
            node->parent = nullptr; 
            node->translation = Eigen::Vector3d(0,0,0); 
            triangleNodes[curr_face_id] = node; 
        }
        isVisited[curr_face_id] = true;
    }
    
}

void applyOffsetStep2(igl::opengl::ViewerData &mesh, const vector<int> &visitOrder,
        const std::vector<Eigen::Vector3i> &sheet_indices, const Eigen::MatrixXi &flat_hinge_edges, 
        std::map<int, vector<int>> &face2dualEdge, std::vector<Eigen::Vector3d> &sheet_vertices_offset, 
        std::vector<Eigen::Vector3i> &sheet_indices_offset, map<int, shared_ptr<transNode>> &triangleNodes, 
        vector<meshEdge> &mesh_edges_data, map<int, triplet> &meshFace2Edge) {

    bool allowShrinkage = true;
    vector<bool> isVisited2(sheet_indices.size(), false);
    vector<int> created_faces; // in a patch to check for triangle-patch intersection.
    // apply the translation that I have learnt to all the nodes
    Eigen::MatrixXi mesh_EV, mesh_FE, mesh_FV;
    getMeshData(mesh, mesh_EV, mesh_FE, mesh_FV);
    for(int vi=0; vi <visitOrder.size(); vi++) {
        int curr_face_id = visitOrder[vi];
        shared_ptr<transNode> node = triangleNodes[curr_face_id];
        Eigen::Vector3d trans_vec = node->translation;
        int prev_face_id=-1, flat_dual_edge = -1; // in flat face configuration...
        vector<int> f2dualMap = face2dualEdge[curr_face_id];
        
        if(vi!= 0) {
            findPreviousFacePatch(prev_face_id, flat_dual_edge, f2dualMap, flat_hinge_edges, curr_face_id, isVisited2);
            // printf("Current face id : %d previous face id : %d\n", curr_face_id, prev_face_id);
            vector<int> of0vids = {sheet_indices_offset[curr_face_id](0), sheet_indices_offset[curr_face_id](1), sheet_indices_offset[curr_face_id](2)};

            sheet_vertices_offset[of0vids[0]] += trans_vec;
            sheet_vertices_offset[of0vids[1]] += trans_vec;
            sheet_vertices_offset[of0vids[2]] += trans_vec;

            // check if it intersects with any cut-edge shared face? if yes shrink both of them so that they don't
            vector<Eigen::Vector3d> T1 = {sheet_vertices_offset[of0vids[0]], sheet_vertices_offset[of0vids[1]], sheet_vertices_offset[of0vids[2]]};
            int iFaceId = -1, iface_id = -1;
            // check triangle with other triangles in the same patch intersection
            vector<Eigen::Vector3i> patch_faces;
            for (auto fc : created_faces)
                patch_faces.push_back(sheet_indices_offset[fc]);

            bool triangleIntersectionWithCreatedPatch = checkTriangleTriangleIntersection(T1, sheet_vertices_offset, patch_faces, iFaceId);
            if (allowShrinkage and triangleIntersectionWithCreatedPatch) { // shrink the triangles to prevent intersection in flattening by a small amount
                printf("The current face : %d is intersecting with the already layed out patch in face : %d\n", curr_face_id, created_faces[iFaceId]);
                int face0 = curr_face_id, face1 = created_faces[iFaceId];
                triplet e0 =  meshFace2Edge[face0];
                triplet e1 = meshFace2Edge[face1];
                int eans = findCommonFaceEdge(e0, e1);
                if(eans == -1) { // they don't share an edge in the real world. 
                    // only apply it on face0
                    // find the free perimeter edge. 
                    int eoff_i;
                    printf("SPECIAL CASE HANDLING!!!\n");
                    triplet efaces = meshFace2Edge[face0];
                    vector<double> spoffset = {mesh_edges_data[get<1>(efaces)].offset, 
                                    mesh_edges_data[get<2>(efaces)].offset, mesh_edges_data[get<0>(efaces)].offset};

                    printf("offset values : %f %f %f\n", spoffset[0], spoffset[1], spoffset[2]);
                    int csid0 = -1, csid1 = -1, csid2 = -1; // setup the correct indexing
                    if(spoffset[0] < 1e-10) { // looking for correct order of offseting
                        csid0 = 0, csid1 = 1, csid2 = 2;
                        eoff_i = get<0>(meshFace2Edge[face0]);
                    }
                    else if(spoffset[1] < 1e-10) {
                        csid0 = 1, csid1 = 2, csid2 = 0;
                        eoff_i = get<1>(meshFace2Edge[face0]);
                    } 
                    else if(spoffset[2] < 1e-10) {
                        csid0 = 2, csid1 = 0, csid2 = 1;
                        eoff_i = get<2>(meshFace2Edge[face0]);
                    } 
                    else {
                        printf("Somethign went wrong DEBUGGGG!!! in finding the correct perimeter edge\n");
                        exit(1);
                    }
                    printf("Self consistency test\n Edge eoff_i : %d is not an hinge : %d offset value : %f\n", eoff_i, mesh_edges_data[eoff_i].isHinge, mesh_edges_data[eoff_i].offset);

                    Eigen::Vector3i f0is = sheet_indices_offset[face0];
                    double offsetCutEdge = 0.0;
                    double offset_incr = 0.1;
                    int step = 0;
                    bool offsetFound = true;
                    while (triangleIntersectionWithCreatedPatch) {
                        offsetCutEdge += offset_incr; // because at this scale it was already intersecting

                        vector<Eigen::Vector3d> f0vs = {sheet_vertices_offset[f0is[0]], sheet_vertices_offset[f0is[1]], sheet_vertices_offset[f0is[2]]};
                        // change the vertex coordinate according to v0
                        performTriangleOffset(f0vs[csid0], f0vs[csid1], f0vs[csid2], offsetCutEdge); // in place change
                        sheet_vertices_offset[f0is[csid1]] = f0vs[csid1]; sheet_vertices_offset[f0is[csid2]] = f0vs[csid2];

                        T1 = {sheet_vertices_offset[f0is[0]], sheet_vertices_offset[f0is[1]], sheet_vertices_offset[f0is[2]]};
                        int tempIface;
                        triangleIntersectionWithCreatedPatch = checkTriangleTriangleIntersection(T1, sheet_vertices_offset, patch_faces, tempIface);
                        step++;
                        if(step > 10) {
                            offsetFound = false; 
                            break;
                        }
                    }
                    offsetCutEdge += offset_incr; // additional offset for clearance issues!!
                    if (offsetFound)
                        printf("Found intersection free offset in steps : %d final offset_scale value %f\n", step, offsetCutEdge);
                    else {
                        printf("DEBUGG!!!! \nCannot find intersection free offset in steps : %d final offset_scale value %f\n", step, offsetCutEdge);
                        exit(1);
                    }
                    mesh_edges_data[eoff_i].offset = offsetCutEdge; //not a hnge ede but still offset remember!!!
                }
                else {
                    int e0_id = getIndexinTriplet(e0, eans); 
                    int e1_id = getIndexinTriplet(e1, eans); 
                    // self-consisntency test
                    int f0v0, f0v1, f0v2;
                    int f1v0, f1v1, f1v2;
                    f0v0 = mesh_FV.row(face0)(0); f1v0 = mesh_FV.row(face1)(0); 
                    f0v1 = mesh_FV.row(face0)(1); f1v1 = mesh_FV.row(face1)(1); 
                    f0v2 = mesh_FV.row(face0)(2); f1v2 = mesh_FV.row(face1)(2); 
                    int shared_v0 = -1, shared_v1 = -1;
                    int csid0 = -1, csid1 = -1, csid2 = -1, psid0 = -1, psid1 = -1, psid2 = -1;
                    findCommonVertex(shared_v0, shared_v1, mesh_FV.row(face0), mesh_FV.row(face1));
                    getIndices(csid0, csid1, csid2, shared_v0, shared_v1, f0v0, f0v1, f0v2);
                    getIndices(psid0, psid1, psid2, shared_v0, shared_v1, f1v0, f1v1, f1v2);
                    double offsetCutEdge = 0.0;
                    double offset_incr = 0.1;
                    int step = 0;
                    vector<int> f0vids = {sheet_indices_offset[curr_face_id](0), sheet_indices_offset[curr_face_id](1), sheet_indices_offset[curr_face_id](2)};
                    while (triangleIntersectionWithCreatedPatch) {
                        offsetCutEdge += offset_incr; // because at this scale it was already intersecting

                        Eigen::Vector3i f0is = sheet_indices_offset[face0], f1is = sheet_indices_offset[face1];
                        vector<Eigen::Vector3d> f0vs = {sheet_vertices_offset[f0is[0]], sheet_vertices_offset[f0is[1]], sheet_vertices_offset[f0is[2]]};
                        vector<Eigen::Vector3d> f1vs = {sheet_vertices_offset[f1is[0]], sheet_vertices_offset[f1is[1]], sheet_vertices_offset[f1is[2]]};
                        // change the vertex coordinate according to v0
                        performTriangleOffset(f0vs[csid2], f0vs[csid0], f0vs[csid1], offsetCutEdge); // in place change
                        performTriangleOffset(f1vs[psid2], f1vs[psid0], f1vs[psid1], offsetCutEdge); // in place change

                        sheet_vertices_offset[f0is[csid0]] = f0vs[csid0]; sheet_vertices_offset[f0is[csid1]] = f0vs[csid1];
                        sheet_vertices_offset[f1is[psid0]] = f1vs[psid0]; sheet_vertices_offset[f1is[psid1]] = f1vs[psid1];


                        T1 = {sheet_vertices_offset[f0vids[0]], sheet_vertices_offset[f0vids[1]], sheet_vertices_offset[f0vids[2]]};
                        int tempIface;
                        triangleIntersectionWithCreatedPatch = checkTriangleTriangleIntersection(T1, sheet_vertices_offset, patch_faces, tempIface);
                        step++;
                    }
                    offsetCutEdge += offset_incr; // additional offset for clearance issues!!
                    printf("Found intersection free offset in steps : %d final offset_scale value %f\n", step, offsetCutEdge);
                    mesh_edges_data[eans].offset = offsetCutEdge; // not a hnge ede but still offset remember!!!
                }
            }
        }
        isVisited2[curr_face_id] = true;
        created_faces.push_back(curr_face_id);
    }
}

void applyOffsetStep3(const vector<int> &visitOrder, 
        const std::vector<Eigen::Vector3d> &sheet_vertices, const std::vector<Eigen::Vector3i> &sheet_indices,
        const Eigen::MatrixXi &flat_hinge_edges, std::map<int, vector<int>> &face2dualEdge, 
        std::vector<Eigen::Vector3d> &sheet_vertices_offset, std::vector<Eigen::Vector3i> &sheet_indices_offset, 
        std::vector<Eigen::Vector3d> &sheet_vertices_hinge, std::vector<Eigen::Vector3i> &sheet_indices_hinge, 
        std::vector<std::vector<int>> &sheet_qindices_hinge, vector<meshEdge> &mesh_edges_data, 
        map<int, triplet> &meshFace2Edge) {
        
    int hingeIndexStart;
    vector<bool> isVisited3(sheet_indices.size(), false);
    // apply the translation that I have learnt to all the nodes
    for(int vi=0; vi <visitOrder.size(); vi++) {
        hingeIndexStart = sheet_vertices_hinge.size();
        int curr_face_id = visitOrder[vi];
        int prev_face_id = -1, flat_dual_edge = -1; // in flat face configuration...
        vector<int> f2dualMap = face2dualEdge[curr_face_id];

        if(vi!= 0) {
            findPreviousFacePatch(prev_face_id, flat_dual_edge, f2dualMap, flat_hinge_edges, curr_face_id, isVisited3);
            triplet e0 = meshFace2Edge[curr_face_id];
            triplet e1 = meshFace2Edge[prev_face_id];
            int eans = findCommonFaceEdge(e0, e1);
            mesh_edges_data[eans].hingeId = sheet_qindices_hinge.size(); 
            // 1 below what I am adding at this point
            // the triangles have shrunk by now.. Now add a rectangular slab between this triangle and the one that is creating the current triangle
            // the dimension of the slab is equal to the min-edge length that is shared and the hinge_width val
            Eigen::Vector3d shared_hpos0, shared_hpos1, shared_hpos2, shared_hpos3;
            addHinge(sheet_indices, sheet_indices_offset, sheet_vertices_offset, curr_face_id, prev_face_id, shared_hpos0, shared_hpos1, shared_hpos2, shared_hpos3);
            // printf("posss : %d %d %d %d\n", shared_hpos0, shared_hpos1, shared_hpos2, shared_hpos3);
            sheet_vertices_hinge.push_back(shared_hpos0);
            sheet_vertices_hinge.push_back(shared_hpos1);
            sheet_vertices_hinge.push_back(shared_hpos2);
            sheet_vertices_hinge.push_back(shared_hpos3);

            // I changed the ordering such that I can kill two birds with one stone. 
            // Both visualization of hinges in this visualizer and how they are stored in the sheet file. 
            // the first 4 will always reprsenet starting from the current the order of indices. {h0, h2}, h1 and h3 [h1 h2]. 
            // {h0, h2} can interchange to reprsent the order of triangle but will align i.e. the common edge
            // with the current face edge. and [h1 h2] will flip to make sure that triangles are correctly oriented for visualiztion
            // but while storing them in sheet I will just store the first 4, {h0 h2} and flip h1 and h3, and store them as ({h0,h2}, h3, h1)
            if (isPositiveOrientation(sheet_vertices_hinge[hingeIndexStart], sheet_vertices_hinge[hingeIndexStart + 1], sheet_vertices_hinge[hingeIndexStart + 2])) {
                sheet_indices_hinge.push_back(Eigen::Vector3i(hingeIndexStart + 2, hingeIndexStart + 0, hingeIndexStart + 1));
                // sheet_indices_hinge.push_back(Eigen::Vector3i(hingeIndexStart + 0, hingeIndexStart + 1, hingeIndexStart + 2));

                sheet_qindices_hinge.push_back(vector<int>{hingeIndexStart + 2, hingeIndexStart + 0, hingeIndexStart + 1, hingeIndexStart + 3});
            }
            else {
                sheet_indices_hinge.push_back(Eigen::Vector3i(hingeIndexStart + 0, hingeIndexStart + 2, hingeIndexStart + 1));
                // sheet_indices_hinge.push_back(Eigen::Vector3i(hingeIndexStart + 1, hingeIndexStart + 0, hingeIndexStart + 2));

                sheet_qindices_hinge.push_back(vector<int>{hingeIndexStart + 0, hingeIndexStart + 2, hingeIndexStart + 3, hingeIndexStart + 1});
            }
            if (isPositiveOrientation(sheet_vertices_hinge[hingeIndexStart + 1], sheet_vertices_hinge[hingeIndexStart + 2], sheet_vertices_hinge[hingeIndexStart + 3])) {
                sheet_indices_hinge.push_back(Eigen::Vector3i(hingeIndexStart + 3, hingeIndexStart + 1, hingeIndexStart + 2));
                // sheet_indices_hinge.push_back(Eigen::Vector3i(hingeIndexStart + 1, hingeIndexStart + 2, hingeIndexStart + 3));
            }
            else {
                sheet_indices_hinge.push_back(Eigen::Vector3i(hingeIndexStart + 3, hingeIndexStart + 2, hingeIndexStart + 1));
                // sheet_indices_hinge.push_back(Eigen::Vector3i(hingeIndexStart + 2, hingeIndexStart + 1, hingeIndexStart + 3));
            }

        }
        isVisited3[curr_face_id] = true; 
    }
        
}

void trimPerimeterEdges(const vector<int> &visitOrder, std::vector<Eigen::Vector3d> &sheet_vertices_offset, 
            std::vector<Eigen::Vector3i> &sheet_indices_offset, vector<meshEdge> &mesh_edges_data, 
            map<int, triplet> &meshFace2Edge, float user_margin )
{
    if (fabs(user_margin) > 1e-6) { // this should only effect perimeter edges
        // printf("Applying user margin to assist fabrication \n");
        for (int vi = 0; vi < visitOrder.size(); vi++) {
            int curr_face_id = visitOrder[vi];
            // auto edges = meshFace2Edge[curr_face_id];
            triplet efaces = meshFace2Edge[curr_face_id];
            vector<double> offset = {mesh_edges_data[get<1>(efaces)].offset, mesh_edges_data[get<2>(efaces)].offset, mesh_edges_data[get<0>(efaces)].offset};

            vector<bool> is_hinge = {mesh_edges_data[get<1>(efaces)].isHinge, mesh_edges_data[get<2>(efaces)].isHinge, mesh_edges_data[get<0>(efaces)].isHinge};

            // printf("Applying user margin to assist fabrication %d\n", curr_face_id);
            vector<int> f0vids = {sheet_indices_offset[curr_face_id](0), sheet_indices_offset[curr_face_id](1), sheet_indices_offset[curr_face_id](2)};
            Eigen::Vector3d vipos0 = sheet_vertices_offset[f0vids[0]];
            Eigen::Vector3d vipos1 = sheet_vertices_offset[f0vids[1]];
            Eigen::Vector3d vipos2 = sheet_vertices_offset[f0vids[2]];

            if (not is_hinge[0])
                performTriangleOffset(vipos0, vipos1, vipos2, user_margin);

            if (not is_hinge[1])
                performTriangleOffset(vipos1, vipos2, vipos0, user_margin);

            if (not is_hinge[2])
                performTriangleOffset(vipos2, vipos0, vipos1, user_margin);

            sheet_vertices_offset[f0vids[0]] = vipos0;
            sheet_vertices_offset[f0vids[1]] = vipos1;
            sheet_vertices_offset[f0vids[2]] = vipos2;
        }
    }
}

void markHalfHingeEdges(igl::opengl::ViewerData &mesh, const Eigen::MatrixXi &flat_hinge_edges, const vector<int> &visitOrder, 
                std::vector<Eigen::Vector3d> &sheet_vertices_hinge, std::vector<std::vector<int>> &sheet_qindices_hinge,
                vector<meshEdge> &mesh_edges_data, map<int, triplet> &meshFace2Edge, double drill_bit_radius) 
{
        std::map<int, vector<int>> face2dualEdge;
    createFace2dualEdgeMap(face2dualEdge, flat_hinge_edges);

    vector<bool> isVisited(mesh.F.rows(), false); // number of faces and false // don't worry i can take all the faces // in the plane
    
    int totalHalfHingeEdges = 0, totalHinges = 0;

    for(int vi = 0; vi < visitOrder.size(); vi++) {
        int curr_face_id = visitOrder[vi];
        triplet efaces0 = meshFace2Edge[curr_face_id];
        
        if(vi!= 0) { // for the first vertex don't have to do all this hardwork just shrinking the triangle is fine..
            int prev_face_id=-1, flat_dual_edge = -1; // in flat face configuration...
            vector<int> f2dualMap = face2dualEdge[curr_face_id];
            findPreviousFacePatch(prev_face_id, flat_dual_edge, f2dualMap, flat_hinge_edges, curr_face_id, isVisited);                        
            triplet efaces1 = meshFace2Edge[prev_face_id];       
            int commonEdgeId = findCommonFaceEdge(meshFace2Edge[curr_face_id], meshFace2Edge[prev_face_id]);
            int hingeIndex = mesh_edges_data[commonEdgeId].hingeId; 
            vector<int> hinge_indices = sheet_qindices_hinge[hingeIndex]; 
            // the first two indices capture the hinge length
            Eigen::Vector3d hv0 = sheet_vertices_hinge[hinge_indices[0]];
            Eigen::Vector3d hv1 = sheet_vertices_hinge[hinge_indices[1]];
            double hinge_length = (hv0-hv1).norm();
            double safeBendingAngle = getSafeBendingAngle(hinge_length);
            // double safeHalfHingeBendingAngle = getHalfHingeSafeBendingAngle(hinge_length);
            double dihedralAngle = computeDihedralAngle(mesh.V, mesh.F, curr_face_id, prev_face_id); 
            double bendingAngle = (dihedralAngle * 180 / M_PI);
            bool canUseHalfHinge = ((bendingAngle < safeBendingAngle * 0.4)?true:false);
            double safeHingeLength = getSafeHingeLength(bendingAngle);
            // canUseHalfHinge = true; // for quick fix
            mesh_edges_data[commonEdgeId].isHalfHinge = canUseHalfHinge;
            // change the offset to a new offset value
            mesh_edges_data[commonEdgeId].offset = computeOffset(dihedralAngle, 1, drill_bit_radius, canUseHalfHinge);

            // printf("Face f0: %d f1: %d hingeLength: %0.2f safetyAngle; %0.2f bendingAngle: %0.2f bendingThresh : %0.2f Safe hinge length : %0.2f Can use half hinge: %d\n",
                //    curr_face_id, prev_face_id, hinge_length, safeBendingAngle,
                //    bendingAngle, safeBendingAngle * 0.4, safeHingeLength, canUseHalfHinge);
            totalHalfHingeEdges += int(canUseHalfHinge);
            totalHinges += 1;
        }

        isVisited[curr_face_id] = true;
    }
    printf("total Half hinge edges: %d\n\n", totalHalfHingeEdges);
    printf("total hinge edges: %d\n\n", totalHinges);
}

double getAdaptiveScaling(igl::opengl::ViewerData &mesh, const Eigen::MatrixXi &flat_hinge_edges, const vector<int> &visitOrder, 
                std::vector<Eigen::Vector3d> &sheet_vertices_hinge, std::vector<std::vector<int>> &sheet_qindices_hinge,
                vector<meshEdge> &mesh_edges_data, map<int, triplet> &meshFace2Edge) 
{
    std::map<int, vector<int>> face2dualEdge;
    createFace2dualEdgeMap(face2dualEdge, flat_hinge_edges);

    vector<bool> isVisited(mesh.F.rows(), false); // number of faces and false // don't worry i can take all the faces // in the plane 

    double minimumScalingSize = 1.0; 
    double maximumScalingSize = 0.0; 
    int maximumScalingPair0 = -1, maximumScalingPair1 = -1;

    for(int vi = 0; vi < visitOrder.size(); vi++) {
        int curr_face_id = visitOrder[vi];
        triplet efaces0 = meshFace2Edge[curr_face_id];
        
        if(vi!= 0) { // for the first vertex don't have to do all this hardwork just shrinking the triangle is fine..
            int prev_face_id=-1, flat_dual_edge = -1; // in flat face configuration...
            vector<int> f2dualMap = face2dualEdge[curr_face_id];
            findPreviousFacePatch(prev_face_id, flat_dual_edge, f2dualMap, flat_hinge_edges, curr_face_id, isVisited);                        
            triplet efaces1 = meshFace2Edge[prev_face_id];       
            int commonEdgeId = findCommonFaceEdge(meshFace2Edge[curr_face_id], meshFace2Edge[prev_face_id]);
            int hingeIndex = mesh_edges_data[commonEdgeId].hingeId; 
            vector<int> hinge_indices = sheet_qindices_hinge[hingeIndex]; 
            // the first two indices capture the hinge length
            Eigen::Vector3d hv0 = sheet_vertices_hinge[hinge_indices[0]];
            Eigen::Vector3d hv1 = sheet_vertices_hinge[hinge_indices[1]];
            double hinge_length = (hv0-hv1).norm();
            double safeBendingAngle = getSafeBendingAngle(hinge_length);
            double bendingAngle = computeDihedralAngle(mesh.V, mesh.F, curr_face_id, prev_face_id); 
            bendingAngle = (bendingAngle * 180 / M_PI);
            double safeHingeLength = getSafeHingeLength(bendingAngle); 
            double ratio = safeHingeLength / hinge_length; 
            minimumScalingSize = min(ratio, minimumScalingSize); 
            // maximumScalingSize = max(ratio, maximumScalingSize); 
            if(ratio > maximumScalingSize) {
                maximumScalingSize = ratio; 
                maximumScalingPair0 = curr_face_id;
                maximumScalingPair1 = prev_face_id;
            }
        }
        isVisited[curr_face_id] = true;
    }

    printf("Maximum pair : %d %d Maximum scaling size : %0.6f Minimum scaling size : %0.6f\n", maximumScalingPair0, maximumScalingPair1,
                    maximumScalingSize, minimumScalingSize); 
    return maximumScalingSize; 
}

void applyOffset(igl::opengl::ViewerData &mesh, std::vector<Eigen::Vector3d> &sheet_vertices, 
                 const std::vector<Eigen::Vector3i> &sheet_indices, const vector<vector<int>> &flatFaceVisitOrder, 
                 const Eigen::MatrixXi &flat_hinge_edges, std::vector<Eigen::Vector3d> &sheet_vertices_offset, 
                 std::vector<Eigen::Vector3i> &sheet_indices_offset, std::vector<Eigen::Vector3d> &sheet_vertices_hinge, 
                 std::vector<Eigen::Vector3i> &sheet_indices_hinge, std::vector<std::vector<int>> &sheet_qindices_hinge,
                 vector<meshEdge> &mesh_edges_data, map<int, triplet> &meshFace2Edge, 
                 double drill_bit_radius, double user_margin, bool adaptiveScaling, bool useHalfHinges) {

    std::map<int, vector<int>> face2dualEdge;
    createFace2dualEdgeMap(face2dualEdge, flat_hinge_edges);
    map<int, shared_ptr<transNode>> triangleNodes;

    auto clearOffsetData = [&]() {
        triangleNodes.clear();
        sheet_vertices_offset.clear(); 
        for(int i = 0; i < sheet_indices_offset.size(); i++) { // cannot just clear it as i access by index in step1
            sheet_indices_offset[i] = Eigen::Vector3i(0,0,0);
        }
        sheet_vertices_hinge.clear(); sheet_indices_hinge.clear();
        sheet_qindices_hinge.clear();
    };

    auto applyCombinedOffsetSteps = [&](const vector<int> &visitOrder) {
        // computing translation phase
        applyOffsetStep1(visitOrder, sheet_vertices, sheet_indices, flat_hinge_edges, 
                        face2dualEdge, mesh_edges_data, meshFace2Edge, sheet_vertices_offset, sheet_indices_offset, triangleNodes, drill_bit_radius);
        
        // apply translation phase
        applyOffsetStep2(mesh, visitOrder,sheet_indices, flat_hinge_edges, face2dualEdge, 
                        sheet_vertices_offset, sheet_indices_offset, triangleNodes, mesh_edges_data, meshFace2Edge);

        // adding hinges phase
        applyOffsetStep3(visitOrder, sheet_vertices, sheet_indices, flat_hinge_edges, 
                        face2dualEdge, sheet_vertices_offset, sheet_indices_offset, 
                        sheet_vertices_hinge, sheet_indices_hinge, 
                        sheet_qindices_hinge,  mesh_edges_data,
                        meshFace2Edge);

    };
    
    clearOffsetData();
    for(int pi = 0; pi < flatFaceVisitOrder.size(); pi++) { // for each patch
        vector<int> visitOrder = flatFaceVisitOrder[pi];
        applyCombinedOffsetSteps(visitOrder);
    }
    double scalingFactor = 0; 
    for(int pi = 0; pi < flatFaceVisitOrder.size(); pi++) { // for each patch
        vector<int> visitOrder = flatFaceVisitOrder[pi];
        double scalingFactorPatch = getAdaptiveScaling(mesh, flat_hinge_edges, visitOrder, 
                                    sheet_vertices_hinge, sheet_qindices_hinge, mesh_edges_data, meshFace2Edge); 
        scalingFactor = max(scalingFactor, scalingFactorPatch);
    }

    if(adaptiveScaling or scalingFactor > 1) { // either user asked for adaptive scaling or the input mesh would lead to violation constraints
        
        printf("performing adaptive scaling. Scaling factor : %f\n", scalingFactor);

        double globalScalingFactor = scalingFactor; 
        int adapt_iter = 0; 
        while(fabs(scalingFactor-1) > 1e-2 and adapt_iter < 20) {
            // scale up mesh and unfolded mesh aka sheet_vertices. 
            for(int i = 0; i < mesh.V.rows(); i++) {
                mesh.V.row(i) = mesh.V.row(i) * scalingFactor; 
            }
            for(int i = 0; i < sheet_vertices.size(); i++) {
                // sheet_vertices[i] = Eigen::Vector3d(sheet_vertices[i][0]*scalingFactor, ); 
                sheet_vertices[i] = sheet_vertices[i] * scalingFactor; 
            }    
            // and perform the applyCombinedOffsetSteps method again. 

            clearOffsetData();
            for(int pi = 0; pi < flatFaceVisitOrder.size(); pi++) { // for each patch
                vector<int> visitOrder = flatFaceVisitOrder[pi];
                applyCombinedOffsetSteps(visitOrder);
            }
            
            // reset scaling factor and compute new one
            scalingFactor = 0; 
            for(int pi = 0; pi < flatFaceVisitOrder.size(); pi++) { // for each patch
                vector<int> visitOrder = flatFaceVisitOrder[pi];
                double scalingFactorPatch = getAdaptiveScaling(mesh, flat_hinge_edges, visitOrder, sheet_vertices_hinge, sheet_qindices_hinge, mesh_edges_data, meshFace2Edge); 
                scalingFactor = max(scalingFactor, scalingFactorPatch);
            }
            globalScalingFactor = globalScalingFactor * scalingFactor; 
            printf("Global scaling factor : %0.6f\n", globalScalingFactor);
            adapt_iter++;
        }
        printf("Global scaling factor : %0.6f\n", globalScalingFactor); 
    }

    if(useHalfHinges) {

        printf("Using half hinges\n");

        // at this stage I can now mark edges that can be further trimmed by adding half-hinge
        for(int pi = 0; pi < flatFaceVisitOrder.size(); pi++) { // for each patch
            vector<int> visitOrder = flatFaceVisitOrder[pi];
            markHalfHingeEdges(mesh, flat_hinge_edges, visitOrder, 
                    sheet_vertices_hinge, sheet_qindices_hinge, mesh_edges_data, meshFace2Edge, drill_bit_radius);
        }

        // for one last time
        // computing translation phase
        clearOffsetData();
        for(int pi = 0; pi < flatFaceVisitOrder.size(); pi++) { // for each patch
            vector<int> visitOrder = flatFaceVisitOrder[pi];
            applyCombinedOffsetSteps(visitOrder);
        }
    }


    // shave off perimeter edges...
    for(int pi = 0; pi < flatFaceVisitOrder.size(); pi++) { // for each patch
        vector<int> visitOrder = flatFaceVisitOrder[pi];
        trimPerimeterEdges(visitOrder, sheet_vertices_offset, sheet_indices_offset, mesh_edges_data, meshFace2Edge, user_margin);
    }
}

void applyCutEdges(igl::opengl::ViewerData &mesh, std::vector<Eigen::Vector3d> &sheet_vertices,
                   std::vector<Eigen::Vector3i> &sheet_indices, Eigen::MatrixXi &hinge_edges, vector<int> &face2Patch,
                   map<int, vector<int>> &world2flatVind, vector<vector<int>> &flatFaceVisitOrder, 
                   vector<meshEdge> &mesh_edges_data, map<int, triplet> &meshFace2Edge)
{

    Eigen::MatrixXi mesh_EV, mesh_FE, mesh_FV;
    getMeshData(mesh, mesh_EV, mesh_FE, mesh_FV);


    printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\nApplying the cut edge method ----- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n-----!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

    Eigen::MatrixXi flat_hinge_edges;
    getFlatHinges(flat_hinge_edges, hinge_edges);

    std::map<int, vector<int>> flatFace2dualEdge;
    createFace2dualEdgeMap(flatFace2dualEdge, flat_hinge_edges);

    // printf("Sheet indinces: \n");
    // for(int si = 0; si < sheet_indices.size(); si++) {
    //     printf("face : %d : %d %d %d\n", si, sheet_indices[si](0), sheet_indices[si](1), sheet_indices[si](2));
    // }


    int new_vid = sheet_vertices.size();
    int new_patch = 1; // curretnly new_patch = 1 all other are zero waiting to be assigned to some patches

    int ci = 0;
    printf("HERE!!!\n");
    int numOverlappingTriangles = countTotalIntersectingTriangles(sheet_vertices, sheet_indices, face2Patch);
    printf("Start apply cut edge step : %d, now Number of overlapping triangles are --> %d\n \n", ci, numOverlappingTriangles);
    // the new_cut_edges are in world space


    while(numOverlappingTriangles > 0) {
    // while(ci < new_cut_edges.rows() and numOverlappingTriangles > 0) {
    // for(int ci=0; ci<new_cut_edges.rows(); ci++) { // if you uncomment it then comment the ci that is at the end of this loop
        // int f0 = new_cut_edges.row(ci)(0); 
        // int f1 = new_cut_edges.row(ci)(1);

        map<pair<int, int>, int> edgeToIndex; // takes in the pair for hingeEdge and stores its index.. the pair is generated with lexiographical ordering
        // this can also change as higne_edges are updated after a sucucesfull cut
        for(int hi=0; hi<hinge_edges.rows(); hi++) {
            int f0 = hinge_edges.row(hi)(0);
            int f1 = hinge_edges.row(hi)(1);
            if(f0<f1) {
                edgeToIndex.insert({make_pair(f0,f1), hi});
            }
            else {
                edgeToIndex.insert({make_pair(f1,f0), hi});
            }
            // printf("hi : %d f0: %d f1: %d\n", hi, f0, f1);
        }
        vector<pair<int, int>> intersectingFaceList;
        findAllIntersectingFaces(sheet_vertices, sheet_indices, intersectingFaceList); 
        printf("Number of overlapping faces : %ld\n", intersectingFaceList.size());
        // printf("These are the faces in flat domain that are overlapping...: --->\n");
        // for (int ci = 0; ci < intersectingFaceList.size(); ci++) {
        //     printf("ci : %d f0 : %d f1 : %d\n", ci, intersectingFaceList[ci].first, intersectingFaceList[ci].second);
        // }

        vector<vector<int>> Intersectingpaths;
        findAllPaths(hinge_edges, intersectingFaceList, Intersectingpaths); // Intersectingpaths are in world space
    
        int best_cut_edge = findBestCutEdge(hinge_edges, edgeToIndex, Intersectingpaths);
        // the termination condition is hiddein finding the best cut edge. This function has to successed

        int f0 = hinge_edges.row(best_cut_edge)(0); 
        int f1 = hinge_edges.row(best_cut_edge)(1);
        // after modifying the code flat faces and world faces are the same.
        int ff0 = f0;
        int ff1 = f1;

        if(face2Patch[ff0] != face2Patch[ff1]) { 
            printf("Then why was a cut edge suggested between them!!! DEBUG!!!\n");
            exit(1);
            continue; // they are already cut out nothing needs to be done
        }

        // before changing anything remove them from the offset list... 

        int f0sid2 = -1, f1sid2 = -1;
        getEndIndices(sheet_indices[ff0], sheet_indices[ff1], f0sid2, f1sid2);

        if(f0sid2 == -1 or f1sid2 == -1) {
            printf("Something went wrong in finding right indices DEBUG\n"); 
            exit(1);
        }

        int commonEdgeId = findCommonFaceEdge(meshFace2Edge[f0], meshFace2Edge[f1]);
        mesh_edges_data[commonEdgeId].offset = 0; 
        mesh_edges_data[commonEdgeId].isHinge = false;

        // they are in the same patch.. 
        // for now just assign them new patches...
        // copy the vertices in the new as new vertices and add new indices.. 
        // propogate the change to all the 

        // pick f1 arbitarily and propogate the change through there..
        int flatf1 = ff1;
        int shared_v0, shared_v1;
        int shared_fv0, shared_fv1, shared_fv2;
        int shared_edge; 

        findCommonEdge(hinge_edges, best_cut_edge, mesh_FE, shared_edge);
        shared_v0 = mesh_EV.row(shared_edge)(0);
        shared_v1 = mesh_EV.row(shared_edge)(1);
        // printf("Face f0 : %d and f1 : %d are not seprated thus have to apply cut edge at : %d %d\n", ff0, ff1, shared_v0, shared_v1);

        // printf("World shared faces are : %d %d\n", f0, f1);
        // printf("World shared vertices are : %d %d\n", shared_v0, shared_v1);

        // printf("Flat shared faces are : %d %d\n", ff0, ff1);
        shared_fv0 = intersect(sheet_indices[ff1] ,world2flatVind[shared_v0]);
        shared_fv1 = intersect(sheet_indices[ff1] ,world2flatVind[shared_v1]);
        shared_fv2 = subtract(sheet_indices[ff1], vector<int>{shared_fv0, shared_fv1});

    
        vector<int> f1vinds = {sheet_indices[ff1](0), sheet_indices[ff1](1), sheet_indices[ff1](2)};
        // printf("f1vinds %d %d %d\n", f1vinds[0], f1vinds[1], f1vinds[2]);
        int tf1sid0 = -1, tf1sid1 = -1, tf1sid2 = -1;
        getIndices(tf1sid0, tf1sid1, f1sid2, shared_fv0, shared_fv1, sheet_indices[ff1](0),sheet_indices[ff1](1),sheet_indices[ff1](2));
        if(tf1sid0 == -1 or tf1sid1 == -1  or f1sid2 == -1) {
            printf("Something went wrong... \n DEBUG!!!");
            exit(1);
        }

        // printf("Flat shared vertices are : %d %d %d\n", shared_fv0, shared_fv1, shared_fv2);

        // duplicate these sheet_vertices twice
        sheet_vertices.push_back(sheet_vertices[shared_fv0]); new_vid++;
        sheet_vertices.push_back(sheet_vertices[shared_fv1]); new_vid++;

        int new_shared_fv0 = new_vid - 2;
        int new_shared_fv1 = new_vid - 1;

        f1vinds[tf1sid0] = new_shared_fv0;
        f1vinds[tf1sid1] = new_shared_fv1;
        
        Eigen::Vector3d vpos0, vpos1, vpos2;
        int f1v0 = mesh_FV.row(ff1)(0); vpos0 = mesh.V.row(f1v0);
        int f1v1 = mesh_FV.row(ff1)(1); vpos1 = mesh.V.row(f1v1);
        int f1v2 = mesh_FV.row(ff1)(2); vpos2 = mesh.V.row(f1v2);
        double gl0 = (vpos1-vpos0).norm(); double gl1 = (vpos2-vpos1).norm(); double gl2 = (vpos0-vpos2).norm();

        // printf("original indices : %d %d %d\n\n", sheet_indices[ff1][0], sheet_indices[ff1][1], sheet_indices[ff1][2]);
        // printf("before f1vinds : %d %d %d\n", f1vinds[0], f1vinds[1], f1vinds[2]);
        int newfv0 = -1, newfv1 = -1, newfv2 = -1;

        if(not isPositiveOrientation(sheet_vertices[sheet_indices[ff1][0]], sheet_vertices[sheet_indices[ff1][1]], sheet_vertices[sheet_indices[ff1][2]])) {
            printf("SOMETHING IS WRONG IN FACE : %d\n", ff1);
        }


        if(not isPositiveOrientation(sheet_vertices[f1vinds[0]], sheet_vertices[f1vinds[1]], sheet_vertices[f1vinds[2]])) {
            printf("%f %f | %f %f | %f %f\n", sheet_vertices[f1vinds[0]][0], sheet_vertices[f1vinds[0]][1], sheet_vertices[f1vinds[1]][0], sheet_vertices[f1vinds[1]][1], sheet_vertices[f1vinds[2]][0], sheet_vertices[f1vinds[2]][1]);
            printf("%f %f | %f %f | %f %f\n", sheet_vertices[sheet_indices[ff1][0]][0], sheet_vertices[sheet_indices[ff1][0]][1], sheet_vertices[sheet_indices[ff1][1]][0], sheet_vertices[sheet_indices[ff1][1]][1], sheet_vertices[sheet_indices[ff1][2]][0], sheet_vertices[sheet_indices[ff1][2]][1]);
            // swap(f1vinds[0], f1vinds[1]);
            exit(1);
        }    
        // printf("after f1vinds : %d %d %d\n\n", f1vinds[0], f1vinds[1], f1vinds[2]);
        chooseCorrectOrder(newfv0, newfv1, newfv2, sheet_vertices, f1vinds[0], f1vinds[1], f1vinds[2], gl0, gl1, gl2);
        sheet_indices[ff1] = Eigen::Vector3i(newfv0, newfv1, newfv2);
        // printf("Updated Flat shared vertices for f1 are : %d %d %d\n", newfv0, newfv1, newfv2); // only f1 has its index updated.. no change for f0
        // sheet_indices[ff1] = Eigen::Vector3i(f1vinds[0], f1vinds[1], f1vinds[2]);
        // printf("Updated Flat shared vertices for f1 are : %d %d %d\n", new_shared_fv0, new_shared_fv1, shared_fv2); // only f1 has its index updated.. no change for f0

        emplaceIndex(world2flatVind, shared_v0, shared_fv0, new_shared_fv0); // need to add a new member for global to flat vertex correspondance
        emplaceIndex(world2flatVind, shared_v1, shared_fv1, new_shared_fv1);

        // will not need this as the order is just from before...

        // printf("UPDATED Sheet indices: \n");
        // for(int si = 0; si < sheet_indices.size(); si++) {
        //     printf("face : %d : %d %d %d\n", si, sheet_indices[si](0), sheet_indices[si](1), sheet_indices[si](2));
        // }

        // separate the patches in the plane and assign them new patch index
        // start a dfs from ff0 and ff1 assign them different patches..
        // f0 gets the original one for f0 and f1 gets the new available patch number ...

        vector<int> face_stack_f0, face_stack_f1; // in flat world
        vector<bool> visited(face2Patch.size(), false); // number of faces...  
        face_stack_f0.push_back(ff0); // do these operations on flattened faces...
        face_stack_f1.push_back(ff1);

        int old_patch = face2Patch[ff0];
        int new_patch0 = old_patch;
        int new_patch1 = new_patch;
        separatePatchDfs(face_stack_f0, visited, flatFace2dualEdge, flat_hinge_edges, ff1, old_patch, new_patch0, face2Patch);
        separatePatchDfs(face_stack_f1, visited, flatFace2dualEdge, flat_hinge_edges, ff0, old_patch, new_patch1, face2Patch);

        printf("All the faces visited in stack f0 --> \n");
        for(auto pfi : face_stack_f0) {
            printf("%d ",pfi);
        } 
        cout << endl;

        printf("All the faces visited in stack f1 --> \n");
        for(auto pfi : face_stack_f1) {
            replace(sheet_indices[pfi], shared_fv0, new_shared_fv0);
            replace(sheet_indices[pfi], shared_fv1, new_shared_fv1);
            printf("%d ",pfi);
        } 
        cout << endl;



        new_patch++;


        // --------------------------------------------------------------------------//
        // ------------------- Separate the patches -------------------------------//
        // --------------------------------------------------------------------------//

        

        vector<int> patch0_indices, patch1_indices;
        for(int fi = 0; fi < face2Patch.size(); fi++) {
            int v0 = sheet_indices[fi](0);
            int v1 = sheet_indices[fi](1);
            int v2 = sheet_indices[fi](2);
            if(face2Patch[fi] == new_patch1) { // all the faces which have this newer patch move them out of the bounds of the previous 
                // make this offset a function of bounding box for each box or do it as a post process..
                if(getIndex(patch1_indices, v0) == -1) // only add it if it does not exist!!!!
                    patch1_indices.push_back(v0);
                
                if(getIndex(patch1_indices, v1) == -1)
                    patch1_indices.push_back(v1);

                if(getIndex(patch1_indices, v2) == -1)
                    patch1_indices.push_back(v2);
            }
            if(face2Patch[fi] == new_patch0) { // all the faces which have this newer patch move them out of the bounds of the previous 
                // make this offset a function of bounding box for each box or do it as a post process..
                if(getIndex(patch0_indices, v0) == -1) // only add it if it does not exist!!!!
                    patch0_indices.push_back(v0);
                
                if(getIndex(patch0_indices, v1) == -1)
                    patch0_indices.push_back(v1);

                if(getIndex(patch0_indices, v2) == -1)
                    patch0_indices.push_back(v2);
            }
        }
        Eigen::Vector3d trans_offset = Eigen::Vector3d(trans_pos,0,0);
        for(auto pi : patch0_indices) {
            // printf("moving vertex %d \n", pi);
            sheet_vertices[pi] += trans_offset*(new_patch0) - trans_offset*(old_patch); // so as to separate the different patches from each other...
        }
        for(auto pi : patch1_indices) {
            // printf("moving vertex %d \n", pi);
            sheet_vertices[pi] += trans_offset*(new_patch1) - trans_offset*(old_patch); // so as to separate the different patches from each other...
        }

        // Patch separation done!!!
        // The cut is succesfull remove the cut-edge from flat_hinge_edges!! 
        // and the corresponding edge from the hinge_edges and update both flatFace2dualEdge and Face2dualEdge functions...
        int index2rem = -1;
        for(int fhi = 0; fhi < flat_hinge_edges.rows();  fhi++) {
            if(flat_hinge_edges.row(fhi)(0) == ff0 and flat_hinge_edges.row(fhi)(1) == ff1 or 
                flat_hinge_edges.row(fhi)(1) == ff0 and flat_hinge_edges.row(fhi)(0) == ff1 ) {
                    index2rem = fhi;

                    // printf("Check %d %d %d %d\n", hinge_edges.row(fhi)(0), hinge_edges.row(fhi)(1), f0, f1);
                    printf("removing edge : %d fhi which disconnects %d  and %d\n", fhi,  ff0, ff1);
                    break;
                }
        } 
        if(index2rem == -1) {
            printf("DEBUG!!! something wrong with index2rem\n");
            exit(1);
        }

        removeRow(flat_hinge_edges, index2rem);
        removeRow(hinge_edges, index2rem);
        flatFace2dualEdge.clear(); // redo the mapping ..
        createFace2dualEdgeMap(flatFace2dualEdge, flat_hinge_edges);

        // check intersection within a patch.. 
        // if yes try to unfold into a different way if the intersection
        std::vector<Eigen::Vector3i> sheet_indices0, sheet_indices1;
        std::vector<int> visit_orderf0, visit_orderf1; 
        for (auto f0_i : face_stack_f0) {
            visit_orderf0.push_back(f0_i);
            sheet_indices0.push_back(sheet_indices[f0_i]);
        }
        
        for (auto f1_i : face_stack_f1) {
            visit_orderf1.push_back(f1_i);
            sheet_indices0.push_back(sheet_indices[f1_i]);

        }
        // since the patch would be split into two so add a new patch and assign things based on the patch indices
        flatFaceVisitOrder.push_back(vector<int>{});
        flatFaceVisitOrder[new_patch0] = face_stack_f0;
        flatFaceVisitOrder[new_patch1] = face_stack_f1;

        numOverlappingTriangles = countTotalIntersectingTriangles(sheet_vertices, sheet_indices, face2Patch);
        printf("Done apply cut edge step : %d, now Number of overlapping triangles are --> %d\n \n", ci, numOverlappingTriangles);
        ci++; // try the next cut edge...
        // if(ci == 2) { // stop after first cut.... might have to debug a lot for next steps...
        //     break; // REMOVE this line just for checking for 1 CE
        // }
    }

    if(numOverlappingTriangles > 0) {
        printf("Still cannot separate the two patches...\n");
    }

    // for(int fi=0; fi<face2Patch.size(); fi++) {
    //     printf("Face : %d is in patch number : %d\n", fi, face2Patch[fi]);
    // }
}

void separatePatchDfs(vector<int> &face_stack, vector<bool> &visited, std::map<int, vector<int>> &face2dualEdge, const Eigen::MatrixXi &hinge_edges,
                int f1, int prev_patch, int new_patch, vector<int> &face2Patch) {
    // assign patch with a dfs approach
    // https://stackoverflow.com/questions/62754365/how-can-i-print-the-path-between-two-nodes-of-a-tree-graph-using-stack-data-st
    int fi = face_stack.back();
    // printf("Visiting face : %d\n", fi);

    // only change the patches if their original patch number was the same...
    if (visited[fi] or fi == f1 )
        return;

    // if(face2Patch[fi]!= prev_patch) // it is already different from the old patch then i can just gracefully exit.. 
    //     return;

    visited[fi] = true;
    face2Patch[fi] = new_patch;
    // printf("Assigning face : %d a new patch %d\n", fi, new_patch);

    // for all the children of this face via hinge edges add them to the face_stack one by one and start the stack..
    // if they succed then the stack would end else copy new child.. one of the child will succeed!!              
    vector<int> face_edges = face2dualEdge[fi];
    for(auto ei : face_edges) {

        if(visited[hinge_edges.row(ei)(0)] and visited[hinge_edges.row(ei)(1)]) {
            continue; // reached a dead end...
        }

        int fnext = (visited[hinge_edges.row(ei)(0)]) ? hinge_edges.row(ei)(1) :  hinge_edges.row(ei)(0);
        if(fnext == f1) {
            continue;
        }
        // printf("Next face options are %d %d chosen : %d\n", hinge_edges.row(ei)(0), hinge_edges.row(ei)(1), fnext);
        face_stack.push_back(fnext);
        separatePatchDfs(face_stack, visited, face2dualEdge, hinge_edges, f1, prev_patch, new_patch, face2Patch);
    }
}

int findBestCutEdge(const Eigen::MatrixXi &hinge_edges, map<pair<int, int>, int> edgeToIndex, const vector<vector<int>> &paths) {
    // paths are in world space... 
    vector<int> edge_weight(hinge_edges.rows()); // counter for how many times an edge is accounted for in a path
    // need a map from faceIndex to hinge edge index 

    // we now have the pair just iterate over all the paths...
    map<int, vector<int>> cutEdges2path;
    for(int pi = 0; pi < paths.size(); pi++) {
        for(int fi=0; fi < paths[pi].size()-1; fi++) {
            int f0 = paths[pi][fi];
            int f1 = paths[pi][fi+1];

            int edge_index = -1;
            pair<int, int> face_pair;
            if(f0 < f1)
                face_pair = make_pair(f0,f1);
            else
                face_pair = make_pair(f1,f0);

            if(edgeToIndex.find(face_pair) == edgeToIndex.end()) {
                printf("Something went wrong in finding key weights %d %d DEBUG\n", face_pair.first, face_pair.second); 
                // exit(1);
            }
            edge_index = edgeToIndex[face_pair];
            if(cutEdges2path.find(edge_index) == cutEdges2path.end())
                cutEdges2path.insert({edge_index, vector<int>({pi})});
            else
                cutEdges2path[edge_index].push_back(pi);

            edge_weight[edge_index]++;
            // printf("faces : %d %d Edge index : %d weight : %d\n", f0 ,f1, edge_index, edge_weight[edge_index]);
            if(edge_index == -1) {
                printf("Something went wrong in assigning weights DEBUG\n"); 
                // exit(1);
            }
        }
    }

    priority_queue<pair<int,int>, vector<pair<int,int>>, queueComp2> edge_weight_pq;
    for (int hi = 0; hi < hinge_edges.rows(); hi++) {
        if(edge_weight[hi] > 0) {
            // printf("In world coordinates.. Dual Edge with %d %d has weight : %d\n", hinge_edges.row(hi)(0), hinge_edges.row(hi)(1), edge_weight[hi]);
            edge_weight_pq.push(make_pair(hi, edge_weight[hi]));
        }
    }    

    int numPaths2Resolve = paths.size();
    vector<bool> pathsToRemoveList(numPaths2Resolve, false);
    vector<int> cut_edge_vector;
    while (edge_weight_pq.size() > 0) {
        pair<int,int> hingeWeightpair = edge_weight_pq.top();
        int hi = hingeWeightpair.first;

        int fhi0 = min(hinge_edges.row(hi)(0), hinge_edges.row(hi)(1)); // lexiographical order..
        int fhi1 = max(hinge_edges.row(hi)(0), hinge_edges.row(hi)(1));

        // new method probably bug free.. 
        int paths2resolvebefore = numPaths2Resolve;
        for(auto pi: cutEdges2path[hi]) {
            if(not pathsToRemoveList[pi]) { // if the path is not already removed then remove the path
                numPaths2Resolve-- ;
                pathsToRemoveList[pi] = true;
                // printf("Cut edge hi : %d between faces : %d %d is useful as it helps in resolving path pi : %d which had conflicts between %d %d\n", 
                //         hi, fhi0, fhi1, pi, paths[pi][0], paths[pi].back());
            }
        }
        int paths2resolveafter = numPaths2Resolve;
        if(paths2resolveafter < paths2resolvebefore) {// if the cut-edge actually helped in reducing the number of unresolved paths then only add it to the cut-vector list.
            printf("Returning the best cut edge : %d %d more faces to resolve\n", hi, numPaths2Resolve);
            return hi;
        }
        edge_weight_pq.pop();    
    }
    printf("Something went wrong could not find a good cut edge.. DEBUG!!!!!\n");
    exit(1);
}

void minCutEdgeAddition(const Eigen::MatrixXi &hinge_edges, const vector<vector<int>> &paths, Eigen::MatrixXi &new_cut_edges, map<int, int> world2flatFaceMap) {

    // paths are in world space... 
    vector<int> edge_weight(hinge_edges.rows()); // counter for how many times an edge is accounted for in a path
    // need a map from faceIndex to hinge edge index 

    map<pair<int, int>, int> edgeToIndex; // takes in the pair for hingeEdge and stores its index.. the pair is generated with lexiographical ordering

    for(int hi=0; hi<hinge_edges.rows(); hi++) {
        int f0 = hinge_edges.row(hi)(0);
        int f1 = hinge_edges.row(hi)(1);

        if(f0<f1) {
            edgeToIndex.insert({make_pair(f0,f1), hi});
            // printf("hinge edge assignments .. %d %d \n", f0, f1);
        }
        else {
            edgeToIndex.insert({make_pair(f1,f0), hi});
            // printf("hinge edge assignments .. %d %d \n", f1, f0);
        }
    }

    // for(auto kv : edgeToIndex) {
    //     printf("key : (%d %d) value : %d\n", kv.first.first, kv.first.second, kv.second);
    // }

    // we now have the pair just iterate over all the paths...
    map<int, vector<int>> cutEdges2path;

    for(int pi = 0; pi < paths.size(); pi++) {
        for(int fi=0; fi < paths[pi].size()-1; fi++) {
            int f0 = paths[pi][fi];
            int f1 = paths[pi][fi+1];

            int edge_index = -1;
            pair<int, int> face_pair;
            if(f0 < f1)
                face_pair = make_pair(f0,f1);
            else
                face_pair = make_pair(f1,f0);

            if(edgeToIndex.find(face_pair) == edgeToIndex.end()) {
                printf("Something went wrong in finding key weights %d %d DEBUG\n", face_pair.first, face_pair.second); 
                // exit(1);
            }
            edge_index = edgeToIndex[face_pair];
            if(cutEdges2path.find(edge_index) == cutEdges2path.end())
                cutEdges2path.insert({edge_index, vector<int>({pi})});
            else
                cutEdges2path[edge_index].push_back(pi);

            edge_weight[edge_index]++;
            // printf("faces : %d %d Edge index : %d weight : %d\n", f0 ,f1, edge_index, edge_weight[edge_index]);
            if(edge_index == -1) {
                printf("Something went wrong in assigning weights DEBUG\n"); 
                // exit(1);
            }
        }
    }

    priority_queue<pair<int,int>, vector<pair<int,int>>, queueComp2> edge_weight_pq;
    for (int hi = 0; hi < hinge_edges.rows(); hi++) {
        // printf("In world coordinates.. Dual Edge with %d %d has weight : %d\n", hinge_edges.row(hi)(0), hinge_edges.row(hi)(1), edge_weight[hi]);
        edge_weight_pq.push(make_pair(hi, edge_weight[hi]));
    }    

    //  start from here.. chose the edges based on priority queue assignment

    // while(edge_weight_pq.size() > 0) {
    //     pair<int,int> hingeWeightpair = edge_weight_pq.top();
    //     int hi0 = hinge_edges.row(hingeWeightpair.first)(0);
    //     int hi1 = hinge_edges.row(hingeWeightpair.first)(1);
    //     int wi  = hingeWeightpair.second;
    //     printf("Edge index between faces %d %d and edge weight : %d\n", hi0, hi1, wi);
    //     edge_weight_pq.pop();
    // }

    int numPaths2Resolve = paths.size();
    vector<bool> pathsToRemoveList(numPaths2Resolve, false);
    vector<int> cut_edge_vector;
    while (numPaths2Resolve > 0 and edge_weight_pq.size() > 0) {
        pair<int,int> hingeWeightpair = edge_weight_pq.top();
        int hi = hingeWeightpair.first;

        int fhi0 = min(hinge_edges.row(hi)(0), hinge_edges.row(hi)(1)); // lexiographical order..
        int fhi1 = max(hinge_edges.row(hi)(0), hinge_edges.row(hi)(1));

        // new method probably bug free.. 
        int paths2resolvebefore = numPaths2Resolve;
        for(auto pi: cutEdges2path[hi]) {
            if(not pathsToRemoveList[pi]) { // if the path is not already removed then remove the path
                numPaths2Resolve-- ;
                pathsToRemoveList[pi] = true;
                // printf("Cut edge hi : %d between faces : %d %d is useful as it helps in resolving path pi : %d which had conflicts between %d %d\n", 
                //         hi, world2flatFaceMap[fhi0], world2flatFaceMap[fhi1], pi, world2flatFaceMap[paths[pi][0]], world2flatFaceMap[paths[pi].back()]);
            }
        }
        int paths2resolveafter = numPaths2Resolve;
        if(paths2resolveafter < paths2resolvebefore) {// if the cut-edge actually helped in reducing the number of unresolved paths then only add it to the cut-vector list.
            cut_edge_vector.push_back(hi);
            printf("%d more faces to resolve\n", numPaths2Resolve);
        }
            


        // old method has bugs in it..
        // // for all the paths that need to be resolved add this cut edge and remove those paths
        // int fhi0 = min(hinge_edges.row(hi)(0), hinge_edges.row(hi)(1)); // lexiographical order..
        // int fhi1 = max(hinge_edges.row(hi)(0), hinge_edges.row(hi)(1));

        // printf("Currently adding Edge index %d between flat faces %d %d and edge weight : %d\n", hi, world2flatFaceMap[fhi0], world2flatFaceMap[fhi1], hingeWeightpair.second);
        // for(int pi = 0; pi < paths.size(); pi++) {
        //     if(pathsToRemoveList[pi])
        //         continue;
        //     for(int pij = 0; pij < paths[pi].size()-1; pij++) {
        //         int entry0 = min(paths[pi][pij], paths[pi][pij+1]);
        //         int entry1 = max(paths[pi][pij], paths[pi][pij+1]) ;

        //         if((fhi0 == entry0) and (fhi1 == entry1)) {
        //             pathsToRemoveList[pi] = true;
        //             numPaths2Resolve--;
        //             printf("%d more faces to cut\n", numPaths2Resolve);
        //             break;
        //         }
        //     }
        // }


        edge_weight_pq.pop();
    }

    if(edge_weight_pq.size() == 0 and numPaths2Resolve > 0) {
        printf("Something went wrong in finding the cut edges DEBUG!!\n");
    }

    new_cut_edges.resize(cut_edge_vector.size(), 2);
    for(int ci=0; ci<new_cut_edges.rows(); ci++) {
        new_cut_edges.row(ci) = hinge_edges.row(cut_edge_vector[ci]);
        // printf("cut indices : %d %d \n", new_cut_edges.row(ci)(0), new_cut_edges.row(ci)(1));
    }
    // alright this functions works for general cases!!! move onto next one!!
}

void findAllPaths(const Eigen::MatrixXi &hinge_edges, const vector<pair<int, int>> &intersectingFaceList, vector<vector<int>> &paths) {
    // paths are in world coordinate space

    paths.resize(intersectingFaceList.size());

    std::map<int, vector<int>> face2dualEdge;
    createFace2dualEdgeMap(face2dualEdge, hinge_edges);

    // printf("Number of paths left --> %ld\n", paths.size());
    for(int pi = 0; pi < paths.size(); pi++) {
        int f0 = intersectingFaceList[pi].first;
        int f1 = intersectingFaceList[pi].second;
        // init lsearch and rsearch to the two ends of tree that f0 can spawn from
        // printf("Hunting for f0 : %d\n", f0);


        vector<int> face_stack;
        vector<bool> visited(face2dualEdge.size(), false); // number of faces...  
        face_stack.push_back(f0);

        bool foundPath = pathSearchDFS(face_stack, visited, face2dualEdge, hinge_edges, f1);
        if(foundPath) {
            // printf("Found a path from %d to %d  \n", f0, f1);
            for (size_t fi = 0; fi < face_stack.size(); fi++) {
                printf("%d ", face_stack[fi]);
            }
            cout << endl;
            paths[pi] = face_stack;
        }
        else {
            printf("Cannot find a path between %d to %d DEBUG!!!\n", f0, f1);
            printf("The problem usually arises that additional patches might be far apart check if INCREASING the patch distance helps? \n");
            printf("The path from %d to %d is : ", f0, f1);
            for (size_t fi = 0; fi < face_stack.size(); fi++) {
                printf("%d ", face_stack[fi]);
            }
            cout << endl;
            exit(1);
        }
    }
}

bool pathSearchDFS(vector<int> &face_stack, vector<bool> &visited, std::map<int, vector<int>> &face2dualEdge, const Eigen::MatrixXi &hinge_edges, int f1) {
        // https://stackoverflow.com/questions/62754365/how-can-i-print-the-path-between-two-nodes-of-a-tree-graph-using-stack-data-st
        int fi = face_stack.back();
        // printf("Visiting face : %d\n", fi);

        if (visited[fi])
            return false;

        visited[fi] = true;
        if(fi == f1) {
            // faceStack becomes the path
            // printf("Found the destination returning....\n");
            return true;
        }

        // for all the children of this face via hinge edges add them to the face_stack one by one and start the stack..
        // if they succed then the stack would end else copy new child.. one of the child will succeed!!              
        vector<int> face_edges = face2dualEdge[fi];
        for(int di = 0; di < face_edges.size(); di++) {

            int ei = face_edges[di];

            if(visited[hinge_edges.row(ei)(0)] and visited[hinge_edges.row(ei)(1)]) {
                continue; // reached a dead end...
            }

            int fnext = (visited[hinge_edges.row(ei)(0)]) ? hinge_edges.row(ei)(1) :  hinge_edges.row(ei)(0);
            // printf("Next face options are %d %d chosen : %d\n", hinge_edges.row(ei)(0), hinge_edges.row(ei)(1), fnext);
            face_stack.push_back(fnext);
            bool found_path = pathSearchDFS(face_stack, visited, face2dualEdge, hinge_edges, f1);
            if(found_path)
                return true;
            
        }
        face_stack.pop_back(); // this pops out fi.. fnext gets popped out in its own stack termination..
        return false;
    return true;
}

void findAllIntersectingFaces(const vector<Eigen::Vector3d> &sheet_vertices, const vector<Eigen::Vector3i> &sheet_indices, 
                                vector<pair<int, int>> &intersectingFaceList) {
    for (size_t fi = 0; fi < sheet_indices.size(); fi++) {
        int fi0 = sheet_indices[fi](0);
        int fi1 = sheet_indices[fi](1);
        int fi2 = sheet_indices[fi](2);
        vector<Eigen::Vector3d>  Ti = {sheet_vertices[fi0], sheet_vertices[fi1], sheet_vertices[fi2]};
        for (size_t fj = fi+1; fj < sheet_indices.size(); fj++) {            
            int fj0 = sheet_indices[fj](0);
            int fj1 = sheet_indices[fj](1);
            int fj2 = sheet_indices[fj](2);
            vector<Eigen::Vector3d>  Tj = {sheet_vertices[fj0], sheet_vertices[fj1], sheet_vertices[fj2]};
            if(checkTriangleTriangleIntersection2(Ti, Tj)) {
                // printf("cut faces : %ld %ld\n", fi, fj);
                intersectingFaceList.push_back(make_pair(fi, fj));
            }
        }
    }
    
}

void findPreviousFacePatch(int &prev_face_id, int &dual_edge, vector<int> f2dualMap, const Eigen::MatrixXi &hinge_edges, int face_id, const vector<bool> &isVisited) {
    for (size_t dei = 0; dei < f2dualMap.size(); dei++) {
        int hi = f2dualMap[dei];
        int f0 = hinge_edges.row(hi)(0);
        int f1 = hinge_edges.row(hi)(1);
        int prev_face;
        (f0==face_id ? prev_face = f1: prev_face = f0); // push the one not visited node in
        // printf("Checking Hinge edge : %d f0 : %d f1 : %d\n", hi, f0, f1);
        if(isVisited[prev_face]) {
            prev_face_id = prev_face;
            dual_edge = hi;
            // printf("Hinge edge : %d f0 : %d f1 : %d\n", hi, f0, f1);
            break;
        }
    }
    if(prev_face_id == -1) {
        printf("Something went wrong debug PREVIOUS PATCH NOT FOUND\n");
        exit(1);
    }

}

void mergePatches(igl::opengl::ViewerData &mesh, std::vector<Eigen::Vector3d> &sheet_vertices,
                  std::vector<Eigen::Vector3i> &sheet_indices, Eigen::MatrixXi &hinge_edges, const Eigen::MatrixXi &dual_edges, vector<meshEdge> &mesh_edges_data,
                  map<int, triplet> &meshFace2Edge, vector<int> &face2Patch, map<int, vector<int>> &world2flatVind,
                  vector<vector<int>> &flatFaceVisitOrder, double obj_scale, double drill_bit_radius, int min_patches) {

    map<int, int> flat2WorldVind;
    for(auto &kv:world2flatVind) {
        for(auto fv: kv.second) {
            flat2WorldVind.insert({fv, kv.first});
            // printf("%d : %d\n", fv, kv.first);
        }
    }

    std::map<int, vector<int>> face2dualEdge;
    createFace2dualEdgeMap(face2dualEdge, dual_edges);

    // map that takes the flat face and tells which patch does it belong to. 
    // face2Patch This map should change with time
    updateFace2Patch(flatFaceVisitOrder, face2Patch);
    
    // attach patches from smallest to the largest.. 
    // store patch size to their patches as a priority list and then draw from it and merge the patches.. 
    // a priority queue in which first index is the patch id and second index is patch's size
    // priority_queue<pair<int,int>, vector<pair<int,int>>, queueComp2> patchSize2PatchIndex; // sorts them by small patches first merge
    priority_queue<pair<int,int>, vector<pair<int,int>>, queueComp3> patchSize2PatchIndex; // sorts them by large patches first merge
    createPriorityQueue(patchSize2PatchIndex, flatFaceVisitOrder);

    int patch_count = 0;
    while(flatFaceVisitOrder.size() > min_patches and patchSize2PatchIndex.size() > 0) {
        pair<int,int> patchIndexSizePair = patchSize2PatchIndex.top();
        int flat_curr_patch_index = patchIndexSizePair.first;
        int patch_size = patchIndexSizePair.second;
        
        printf("\n \nCurrently Patches merged : %d Patch pi : %d has faces : and size : %d ", patch_count, flat_curr_patch_index, patch_size);
        for(int fi = 0; fi < patch_size; fi++) {
            printf("%d ", flatFaceVisitOrder[flat_curr_patch_index][fi]);
        }  printf("\n");

        patchSize2PatchIndex.pop();


        // for now break after first patch and just try to merge that patch
        priority_queue<pair<pair<int, int>, int>, vector<pair<pair<int, int>, int>>, queueComp4> face2SizeHelper; // higher one first selects the best face possible
        for(int pfi = 0; pfi < patch_size; pfi++) {
            // Step 1 : for each face look for its corresponding face in the world configuration
            // Step 2 : then figure out all the possible edges that we can connect it with. 
            // Step 3 : amongst all these edges which edges allow the face to connect it to a new patch 
            // Step 4 : sort all the hinge edges in a priority queue that allows it to merge with a larger patch
            // Step 5 : next is to check if this new patch addition is intersection free or not? 
            // Step 6 : if not look for a next patch. 

            // Step 1 find its corresponding face in the world
            int flat_face_curr = flatFaceVisitOrder[flat_curr_patch_index][pfi];
            int world_fi = flat_face_curr; 
            
            // Step 2 all the dual-edges for this face in the world
            vector<int> face_edges = face2dualEdge[world_fi];
            // priority_queue<pair<int,int>, vector<pair<int,int>>, queueComp2> face2SizeHelper; // higher one first selects the best face possible 
            
            // need a priority queue because the best one might not be the best to connect because of intersections. 
            for(auto ei : face_edges) {
                // check which of the index is face0 the other one is the correct one. 
                int world_fnext = (dual_edges.row(ei)(0) == world_fi) ? dual_edges.row(ei)(1) : dual_edges.row(ei)(0);
                int flat_fnext = world_fnext;
                int flat_next_patch_index = face2Patch[flat_fnext];

                if(flat_next_patch_index == flat_curr_patch_index) // they are already in the same patch!!
                    continue;

                int next_patch_size = flatFaceVisitOrder[flat_next_patch_index].size();

                face2SizeHelper.push(make_pair(make_pair(flat_face_curr, flat_fnext), flatFaceVisitOrder[flat_next_patch_index].size()));
                // printf("For face : flat : %d world : %d edge index : %d its next neighbour is : flat : %d  world : %d next faces patch : %d and size : %d \n", 
                //         flat_face_curr, world_fi, ei, flat_fnext, world_fnext, flat_next_patch_index, next_patch_size);
                
            }
            // cout << endl;           
        }

        // Step 4 : sort all the hinge edges in a priority queue that allows it to merge with a larger patch and start by drawing smallest patch first
        // face2SizeHelper just contains all the possible face->next patch face pair that can be merged.
        while (flatFaceVisitOrder.size() > min_patches and face2SizeHelper.size() > 0) {
            pair<pair<int, int>, int> next_face_size_pair = face2SizeHelper.top();
            int flat_face_curr = next_face_size_pair.first.first;
            int flat_face_next = next_face_size_pair.first.second;
            int next_flat_patch_index = face2Patch[flat_face_next];
            int flat_curr_patch_index = face2Patch[flat_face_curr];
            if(next_flat_patch_index == flat_curr_patch_index) { // they must have been resolved by some other faces in the same patch
                face2SizeHelper.pop();
                continue; 
            }
            
            int world_fnext = flat_face_next;
            int world_fi = flat_face_curr;
            vector<int> face_edges = face2dualEdge[world_fi];
            // printf("\nFace patch and its size : %d %d %d \n", flat_face_next, next_flat_curr_patch_index, next_face_size_pair.second);

            // Step 3: Check if we can connect to this patch
            // printf("Atempting to merge patches : %d %d using faces : %d %d\n", flat_curr_patch_index, next_flat_curr_patch_index, flat_face_curr, flat_face_next);
            bool canMerge = canIMergePatches(mesh, dual_edges, sheet_vertices, sheet_indices, flatFaceVisitOrder[flat_curr_patch_index], flatFaceVisitOrder[next_flat_patch_index],
                                             world2flatVind, flat2WorldVind, face_edges, flat_face_curr, flat_face_next);
            if(canMerge) {
                Eigen::MatrixXi flat_hinge_edges;
                getFlatHinges(flat_hinge_edges, hinge_edges);
                merge2Patches(flatFaceVisitOrder, flat_curr_patch_index, next_flat_patch_index, flat_face_curr, flat_face_next, flat_hinge_edges);

                // update patchSize2PatchIndex and face2patch data structures
                createPriorityQueue(patchSize2PatchIndex, flatFaceVisitOrder);
                updateFace2Patch(flatFaceVisitOrder, face2Patch);

                // update hinge edges
                Eigen::Vector2i face_vec = Eigen::Vector2i(world_fi, world_fnext);
                hinge_edges.conservativeResize(hinge_edges.rows()+1, hinge_edges.cols());
                hinge_edges.row(hinge_edges.rows()-1) = face_vec;
                printf("Adding faces to hinge edge : world : %d %d flat : %d %d\n", world_fi, world_fnext, world_fi, world_fnext);

                // TODO update the value of flatFaceoffset and flatFaceConvave
                double dihedralAngle = computeDihedralAngle(mesh.V, mesh.F, world_fi, world_fnext); 
                double offset = computeOffset(dihedralAngle, 1, drill_bit_radius);
                
                int pfsid2 = -1, cfsid2 = -1, shared_v0 = -1, shared_v1 = -1; 
                getMergeIndices(sheet_indices, flat2WorldVind, flat_face_curr, flat_face_next, pfsid2 , cfsid2 , shared_v0 , shared_v1 );

                int commonEdgeId = findCommonFaceEdge(meshFace2Edge[flat_face_curr], meshFace2Edge[flat_face_next]);
                mesh_edges_data[commonEdgeId].offset = offset;
                mesh_edges_data[commonEdgeId].isHinge = true; 

                // Our job is done here get out of the loop 
                // break;
                patch_count++; // succesfully merged a patch
            }
            face2SizeHelper.pop(); // otherwise just pop this entry
        }

        // if(patch_count > -1)
        //     break;
        
        // patch_count++;
    }

    // for(int pi = 0; pi < flatFaceVisitOrder.size(); pi++){
    //     for(auto fi : flatFaceVisitOrder[pi]) {
    //         printf("Face : %d has patch : %d\n", fi, pi);
    //         face2Patch[fi] = pi;
    //     }
    // }

    if(flatFaceVisitOrder.size() == 1) { // YAY I have succesfully merged all the patches
        // center the patches
        Eigen::Vector3d com = Eigen::Vector3d(0,0,0);
        for(int si = 0; si < sheet_vertices.size(); si++) {
            com += sheet_vertices[si];
        }
        com /= sheet_vertices.size();
        for(int si = 0; si < sheet_vertices.size(); si++) {
            sheet_vertices[si] -= com;
        }
    }
    else { // more than 1 patch

        for(int pi = 0; pi < flatFaceVisitOrder.size(); pi++) {
            // printf("Centering patch : pi : %d\n", pi);
            // printf("faces : ");
            Eigen::Vector3d com = Eigen::Vector3d(0,0,0);
                
            vector<int> patch_vertex_indices;
            for(auto fi : flatFaceVisitOrder[pi]) {
                int p0v0 = sheet_indices[fi](0);
                int p0v1 = sheet_indices[fi](1);
                int p0v2 = sheet_indices[fi](2);
                if(find(patch_vertex_indices.begin(), patch_vertex_indices.end(), p0v0) == patch_vertex_indices.end() ) {
                    patch_vertex_indices.push_back(p0v0);
                }
                if(find(patch_vertex_indices.begin(), patch_vertex_indices.end(), p0v1) == patch_vertex_indices.end() ) {
                    patch_vertex_indices.push_back(p0v1);
                }
                if(find(patch_vertex_indices.begin(), patch_vertex_indices.end(), p0v2) == patch_vertex_indices.end() ) {
                    patch_vertex_indices.push_back(p0v2);
                }                
            }

            for(auto vi : patch_vertex_indices) {
                // printf("%d ", vi);
                com += sheet_vertices[vi];
            }
            // printf("\n");
            com /= patch_vertex_indices.size();
            for(auto vi : patch_vertex_indices) {
                sheet_vertices[vi] -= com + pi*trans_pos*Eigen::Vector3d(1,0,0);
            }
        }
    }
}

bool canIMergePatches( igl::opengl::ViewerData &mesh, const Eigen::MatrixXi &dual_edges, vector<Eigen::Vector3d> &sheet_vertices, 
                       vector<Eigen::Vector3i> &sheet_indices, vector<int> patch_0, vector<int> patch_1, map<int, vector<int>> &world2flatVind, 
                       map<int, int> &flat2WorldVind, vector<int> face_edges, int face_0, int face_1) {

    // check if patch0 can be merge with patch1 via connection face0 and face1 
    
    // first connect the two patches temporarilly and check if this would result in an intersection or not... 
    vector<Eigen::Vector3d> temp_sheet_vertices = sheet_vertices;
    vector<Eigen::Vector3i> temp_sheet_indices = sheet_indices;

    int cfv0 = temp_sheet_indices[face_0](0); 
    int cfv1 = temp_sheet_indices[face_0](1);
    int cfv2 = temp_sheet_indices[face_0](2);

    int pfv0 = temp_sheet_indices[face_1](0); // these vertices should be in flat vertex format so that I can index it using them
    int pfv1 = temp_sheet_indices[face_1](1);
    int pfv2 = temp_sheet_indices[face_1](2);

    int cv0 = flat2WorldVind[cfv0];
    int cv1 = flat2WorldVind[cfv1];
    int cv2 = flat2WorldVind[cfv2];
    
    int pv0 = flat2WorldVind[pfv0];
    int pv1 = flat2WorldVind[pfv1];
    int pv2 = flat2WorldVind[pfv2];

    int shared_v0 = -1, shared_v1 = -1;

    vector<int> cfv = {cfv0 ,cfv1 ,cfv2};
    vector<int> pfv = {pfv0 ,pfv1 ,pfv2};

    vector<int> cv = {cv0 ,cv1 ,cv2};
    vector<int> pv = {pv0 ,pv1 ,pv2};
    intersect2(cv, pv, shared_v0, shared_v1);

    // printf("debug : %d %d %d %d %d %d, %d %d %d %d %d %d %d %d\n", cfv0 ,cfv1 ,cfv2 ,pfv0 ,pfv1 ,pfv2 ,cv0 ,cv1 ,cv2 ,pv0 ,pv1 ,pv2, shared_v0,shared_v1);

    int csid0 = -1, csid1 = -1, csid2 = -1;
    int psid0 = -1, psid1 = -1, psid2 = -1;

    getIndices(csid0, csid1, csid2, shared_v0, shared_v1, cv0 ,cv1 ,cv2);
    getIndices(psid0, psid1, psid2, shared_v0, shared_v1, pv0 ,pv1 ,pv2);

    // find transformation that takes the the first patch and puts it on top of the other patch
    // i have to move the current triangle to the previous triangle
    Eigen::Vector3d trans = temp_sheet_vertices[pfv[psid0]] - temp_sheet_vertices[cfv[csid0]];

    Eigen::Vector3d e0_ref = temp_sheet_vertices[pfv[psid1]] - temp_sheet_vertices[pfv[psid0]];
    Eigen::Vector3d e1_ref = temp_sheet_vertices[cfv[csid1]] - temp_sheet_vertices[cfv[csid0]]; // from previous step e0 from prev step


    double cosTheta = e0_ref.dot(e1_ref) / (e0_ref.norm()*e1_ref.norm());
    double sinTheta = (e1_ref.cross(e0_ref)).norm() / (e1_ref.norm()*e0_ref.norm());
    if(e1_ref.cross(e0_ref).dot(Eigen::Vector3d(0,0,1)) < 0)
        sinTheta *= -1;

    Eigen::Matrix3d rotMat;
    rotMat << cosTheta , -sinTheta, 0,
                sinTheta, cosTheta,  0,
                0, 0, 1;

    // now apply translation and rotation to all the faces in the patch0 
    // but have to traverse them face by face.. 
    // unfold the first face and check if the rotation needed has to be positive or negative

    bool applyMirror = false;

    // check with a dummy variable if a transformation is possible or not
    Eigen::Vector3d temp_v2 = temp_sheet_vertices[cfv[csid2]] + trans;
    temp_v2 = temp_sheet_vertices[pfv[psid0]] + rotMat * (temp_v2 - temp_sheet_vertices[pfv[psid0]]);

    vector<Eigen::Vector3d>  temp_triangle = {temp_sheet_vertices[pfv[psid0]], temp_sheet_vertices[pfv[psid1]], temp_v2}; // create a temp triangle and check with that 
    // TODO maybe it would be wise to not check with all the triangles but just with the triangles of the two patches. this way i can get rid of th dependence based on translation of patches

    Eigen::Vector3d mirror_line;
    if(checkTriangleTriangleIntersection2(temp_triangle, temp_sheet_vertices, temp_sheet_indices)) {
        // printf("Original unfolding had intersection so had to apply mirror transformation\n");
            
        mirror_line = temp_sheet_vertices[pfv[psid1]] - temp_sheet_vertices[pfv[psid0]]; // refresh these values since rotation is done
        Eigen::Vector3d e2_ref = temp_sheet_vertices[cfv[csid2]] - temp_sheet_vertices[pfv[psid0]]; // take origin as the previous as i have not transformed the current
        Eigen::Vector3d etemp = e2_ref;
        Eigen::Vector3d enorm = mirror_line.normalized();
        temp_v2 = temp_v2 -2*(etemp - etemp.dot(enorm)*enorm); 

        if(checkVertexPolygonIntersection(temp_v2, temp_sheet_vertices, temp_sheet_indices)) {
            printf("Cannot unfold even the first triangle without intersection so the patch cannot be merged\n");
            return false;
        }
        // printf("Flag mirror transformation activated\n");
        applyMirror = true;
    }


    vector<int> patch_verex_indices;
    for(auto fi : patch_0) {
        int p0v0 = temp_sheet_indices[fi](0);
        int p0v1 = temp_sheet_indices[fi](1);
        int p0v2 = temp_sheet_indices[fi](2);
        if(find(patch_verex_indices.begin(), patch_verex_indices.end(), p0v0) == patch_verex_indices.end() ) {
            patch_verex_indices.push_back(p0v0);
        }
        if(find(patch_verex_indices.begin(), patch_verex_indices.end(), p0v1) == patch_verex_indices.end() ) {
            patch_verex_indices.push_back(p0v1);
        }
        if(find(patch_verex_indices.begin(), patch_verex_indices.end(), p0v2) == patch_verex_indices.end() ) {
            patch_verex_indices.push_back(p0v2);
        }
    }

    // translate and rotate
    mirror_line = temp_sheet_vertices[pfv[psid1]] - temp_sheet_vertices[pfv[psid0]]; // This forms the mirror line
    for(auto vis : patch_verex_indices) {
        // printf("moving vertex %d \n", vis);
        temp_sheet_vertices[vis] += trans;
        temp_sheet_vertices[vis] = temp_sheet_vertices[pfv[psid0]] + rotMat * (temp_sheet_vertices[vis] - temp_sheet_vertices[pfv[psid0]]);

        if(applyMirror) {
            // printf("Applying mirror  transformation\n");
            Eigen::Vector3d e2_ref = temp_sheet_vertices[vis] -temp_sheet_vertices[pfv[psid0]]; // refresh these values since rotation is done
            Eigen::Vector3d etemp = e2_ref;
            Eigen::Vector3d enorm = mirror_line.normalized();
            temp_sheet_vertices[vis] = temp_sheet_vertices[vis] -2*(etemp - etemp.dot(enorm)*enorm); 
        }


    }

    if((temp_sheet_vertices[pfv[psid1]] - temp_sheet_vertices[cfv[csid1]]).norm() > 1e-6) {
        printf("Things which should have been merged are not lying on top of each other. wierd DEBUG!!!\n");
        printf("Debug inside failed to combine : %d %d %d %d %d %d\n %d %d %d %d %d %d %d %d\n", cfv0 ,cfv1 ,cfv2 ,pfv0 ,pfv1 ,pfv2 ,cv0 ,cv1 ,cv2 ,pv0 ,pv1 ,pv2, shared_v0,shared_v1);
        printvec(e0_ref, "e0_ref");
        printvec(e1_ref, "e1_ref");
        printvec(mirror_line, "mirror_line");
        printf("Theta = %f \n", acos(cosTheta) * 180 / M_PI);
        printf("pfv[psid1], cfv[csid1] : %d %d\n", pfv[psid1], cfv[csid1]);
        printvec(temp_sheet_vertices[pfv[psid1]], "temp_sheet_vertices[pfv[psid1]]");
        printvec(temp_sheet_vertices[cfv[csid1]], "temp_sheet_vertices[cfv[csid1]]]");
        exit(1);
    }

    if(not checkTriangleTriangleIntersection(temp_sheet_vertices, temp_sheet_indices) ) {
        printf("Can succesfully merge the two patches : using faces : %d %d !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", face_0, face_1);      

        // merge patches will happen globally... outside this function scope

        // removing entry from world2flatVind map and flat2WorldVind map
        removeEntry(world2flatVind, shared_v0, cfv[csid0]); // need to add a new member for global to flat vertex correspondance
        removeEntry(world2flatVind, shared_v1, cfv[csid1]);
        flat2WorldVind.erase(cfv[csid0]);
        flat2WorldVind.erase(cfv[csid1]);

        // Replace sheet_indices not only in face_0 but all the faces in this patch
        for(auto pi: patch_0) {
            replace(temp_sheet_indices[pi], cfv[csid0], pfv[psid0]);
            replace(temp_sheet_indices[pi], cfv[csid1], pfv[psid1]);
        }

        sheet_vertices = temp_sheet_vertices;
        sheet_indices = temp_sheet_indices;
        return true;
    }
    return false;
}

void getMergeIndices(std::vector<Eigen::Vector3i> & sheet_indices, map<int, int> &flat2WorldVind, int face_0, int face_1, int &pfsid2 , int &cfsid2 , int &shared_v0 , int &shared_v1 ) {
    int cfv0 = sheet_indices[face_0](0); 
    int cfv1 = sheet_indices[face_0](1);
    int cfv2 = sheet_indices[face_0](2);

    int pfv0 = sheet_indices[face_1](0); // these vertices should be in flat vertex format so that I can index it using them
    int pfv1 = sheet_indices[face_1](1);
    int pfv2 = sheet_indices[face_1](2);

    int cv0 = flat2WorldVind[cfv0];
    int cv1 = flat2WorldVind[cfv1];
    int cv2 = flat2WorldVind[cfv2];
    
    int pv0 = flat2WorldVind[pfv0];
    int pv1 = flat2WorldVind[pfv1];
    int pv2 = flat2WorldVind[pfv2];

    vector<int> cfv = {cfv0 ,cfv1 ,cfv2};
    vector<int> pfv = {pfv0 ,pfv1 ,pfv2};

    vector<int> cv = {cv0 ,cv1 ,cv2};
    vector<int> pv = {pv0 ,pv1 ,pv2};
    intersect2(cv, pv, shared_v0, shared_v1);

    // printf("Debug inside getMergeIndices : %d %d %d %d %d %d\n %d %d %d %d %d %d %d %d\n", cfv0 ,cfv1 ,cfv2 ,pfv0 ,pfv1 ,pfv2 ,cv0 ,cv1 ,cv2 ,pv0 ,pv1 ,pv2, shared_v0,shared_v1);

    int csid0 = -1, csid1 = -1, csid2 = -1;
    int psid0 = -1, psid1 = -1, psid2 = -1;

    getIndices(csid0, csid1, csid2, shared_v0, shared_v1, cv0 ,cv1 ,cv2);
    getIndices(psid0, psid1, psid2, shared_v0, shared_v1, pv0 ,pv1 ,pv2);    

    pfsid2 = psid2;
    cfsid2 = csid2;

    if(pfsid2 == -1 or cfsid2 == -1 or shared_v0 == -1 or shared_v1 == -1) {
        printf("something went wrong in finding merge helping indices debug\n ");
    }
}


void merge2Patches(vector<vector<int>> &flatFaceVisitOrder, int flat_curr_patch_index, int next_flat_curr_patch_index, int flat_face_curr, int flat_face_next, const Eigen::MatrixXi &flat_hinge_edges) {
    // merge the "flat_curr_patch_index" patch with "next_flat_curr_patch_index" patch
    // this in theory removes the patch with "flat_curr_patch_index" and puts its order at the point where flat_face_next existed.
    // fill the faces to end with "flat_curr_patch_index" patch and then fill remaining of the "next_flat_curr_patch_index "patch
    // instead of starting from its usual starting face it may start from the middle and then branch off from there. 

    vector<int> f0_faces = flatFaceVisitOrder[flat_curr_patch_index];
    vector<int> f1_faces = flatFaceVisitOrder[next_flat_curr_patch_index];


    auto it0 = find(f0_faces.begin(), f0_faces.end(), flat_face_curr);
    auto it1 = find(f1_faces.begin(), f1_faces.end(), flat_face_next);
    int if0 = -1;  // f0_faces.index(flat_face_curr); // index_of_flat_face_curr
    int if1 = -1;  // f1_faces.index(flat_face_next); // index_of_flat_face_curr
    if (it0 != f0_faces.end())
        if0 = it0 - f0_faces.begin();
    else {
        printf("something 1 went wrong in merge2patches function debug!! \n");
        exit(1);
    }

    if (it1 != f1_faces.end())
        if1 = it1 - f1_faces.begin();
    else {
        printf("something 2 went wrong in merge2patches function debug!! \n");
        exit(1);
    }

    // printf("DEBUG merge2patches : current_patch, next_patch, current_face, next face : if0 =  %d %d %d %d %d\n",flat_curr_patch_index,next_flat_curr_patch_index,flat_face_curr,flat_face_next, if0 );

    // fnew 
    vector<int> fnew;

    for(int i = 0; i < if1; i++) {
        fnew.push_back(f1_faces[i]);
    }
    
    fnew.push_back(f1_faces[if1]);

    // fnew.push_back(f0_faces[if0]);
    // unravel tree of patch flat_curr_patch_index using edge-connections
    std::map<int, vector<int>> flatFace2dualEdge;
    vector<bool> isVisited;
    isVisited.resize(f0_faces.size(), false);

    // add the remaining faces using tree like unravelling
    createFace2dualEdgeMap(flatFace2dualEdge, flat_hinge_edges);
    // perform dfs here

    map<int, int> face2index;
    for(int i = 0; i < f0_faces.size(); i++) {
        face2index.insert({f0_faces[i], i});
    }


    // printf("faces unravelled : ");
    stack<int> face_stack;
    int face_id = f0_faces[if0];
    face_stack.push(face_id);
    while(!face_stack.empty()) {
        face_id = face_stack.top();
        face_stack.pop();
        vector<int> f2dualMap = flatFace2dualEdge[face_id];

        if(!isVisited[face2index[face_id]]) {
            fnew.push_back(face_id);
            isVisited[face2index[face_id]] = true;
            // printf("%d ", face_id);
        }
        for(auto di: f2dualMap) {
            int f0 = flat_hinge_edges.row(di)(0);
            int f1 = flat_hinge_edges.row(di)(1);

            if(!isVisited[face2index[f0]])
                face_stack.push(f0);
            
            if(!isVisited[face2index[f1]])
                face_stack.push(f1);

        }

    } printf("\n");

    // now remaining f1_faces
    for(int i = if1+1; i < f1_faces.size(); i++) {
        fnew.push_back(f1_faces[i]);
    }

    flatFaceVisitOrder[next_flat_curr_patch_index] = fnew;
    flatFaceVisitOrder.erase(flatFaceVisitOrder.begin() + flat_curr_patch_index); // remove by index

    // for(int pi = 0; pi < flatFaceVisitOrder.size(); pi++) {
    //     printf("patch index : %d faces : ", pi);
    //     for(auto ff: flatFaceVisitOrder[pi]) {
    //         printf("%d ", ff);
    //     }
    //     cout << endl;
    // }
}

void createPriorityQueue(priority_queue<pair<int,int>, vector<pair<int,int>>, queueComp3> &patchSize2PatchIndex, const vector<vector<int>> &flatFaceVisitOrder) {
// void createPriorityQueue(priority_queue<pair<int,int>, vector<pair<int,int>>, queueComp2> &patchSize2PatchIndex, const vector<vector<int>> &flatFaceVisitOrder) {

    // first clear the queue if its not empty
    while(patchSize2PatchIndex.size() > 0)
        patchSize2PatchIndex.pop();
        
    for(int pi = 0; pi < flatFaceVisitOrder.size(); pi++){
        // printf("Patch pi : %d has faces : ", pi);
        // for(int fi = 0; fi < flatFaceVisitOrder[pi].size(); fi++) {
            // printf("%d ", flatFaceVisitOrder[pi][fi]);
        // }
        patchSize2PatchIndex.push(make_pair(pi, flatFaceVisitOrder[pi].size()));
        // printf("\n");
    }

}

void updateFace2Patch(const vector<vector<int>> &flatFaceVisitOrder, vector<int> &face2Patch) {
    for(int pi = 0; pi < flatFaceVisitOrder.size(); pi++){
        for(auto fi : flatFaceVisitOrder[pi]) {
            // printf("Face : %d has patch : %d\n", fi, pi);
            face2Patch[fi] = pi;
        }
    }    
}

void getFlatHinges(Eigen::MatrixXi &flat_hinge_edges, const Eigen::MatrixXi &hinge_edges) {
    flat_hinge_edges.resize(hinge_edges.rows(), 2);
    for(int fhi = 0; fhi < flat_hinge_edges.rows(); fhi++) {
        if(hinge_edges.row(fhi)(0) == -1 or hinge_edges.row(fhi)(1) == -1) {
            printf("DEBUG : %d %d\n", hinge_edges.row(fhi)(0),hinge_edges.row(fhi)(1));
            continue; // safeguard against spurious things
        }
        flat_hinge_edges.row(fhi) = Eigen::Vector2i(hinge_edges.row(fhi)(0), hinge_edges.row(fhi)(1));
        // printf("flat faces and dual edges %d %d\n", hinge_edges.row(fhi)(0), hinge_edges.row(fhi)(1));
    }    
}

void correctOrientation(igl::opengl::ViewerData &mesh, std::vector<Eigen::Vector3d> &sheet_vertices, std::vector<Eigen::Vector3i> &sheet_indices) {
    // printf("\n\nSanity check\n");
    std::set<int> verticesTobeFlipped;
    for (int fi = 0; fi < mesh.F.rows(); fi++)
    {
        int ffi = fi;
        Eigen::Vector3d wv0, wv1, wv2, fv0, fv1, fv2;
        wv0 = mesh.V.row(mesh.F(fi, 0));
        wv1 = mesh.V.row(mesh.F(fi, 1));
        wv2 = mesh.V.row(mesh.F(fi, 2));
        fv0 = sheet_vertices[(sheet_indices[ffi](0))];
        fv1 = sheet_vertices[(sheet_indices[ffi](1))];
        fv2 = sheet_vertices[(sheet_indices[ffi](2))];
        Eigen::Vector3d zvec = Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d worldLengths1 = Eigen::Vector3d((wv1 - wv0).norm(), (wv2 - wv1).norm(), (wv0 - wv2).norm());
        Eigen::Vector3d worldLengths2 = Eigen::Vector3d((wv2 - wv1).norm(), (wv0 - wv2).norm(), (wv1 - wv0).norm());
        Eigen::Vector3d worldLengths3 = Eigen::Vector3d((wv0 - wv2).norm(), (wv1 - wv0).norm(), (wv2 - wv1).norm());
        Eigen::Vector3d flatLengths = Eigen::Vector3d((fv1 - fv0).norm(), (fv2 - fv1).norm(), (fv0 - fv2).norm());

        // printf("fi: %d  length: %f %f %f\n", fi, worldLengths1[0], worldLengths1[1], worldLengths1[2]);
        // printf("ffi: %d length: %f %f %f\n\n", ffi, flatLengths[0], flatLengths[1], flatLengths[2]);
        // FIX HERE!!!!!!!!!!!!!!!!!!!!!!!!!

        double eps = 1e-8;

        if ((worldLengths1 - flatLengths).norm() < eps or (worldLengths2 - flatLengths).norm() < eps or (worldLengths3 - flatLengths).norm() < eps) {
            // printf("fi : %d is consistent\n", fi);
            continue;
        }
        else {
            printf("fi : %d ffi : %d is not consistent\n", fi, ffi);
            // printf("fi : %d edge lengths world : %f %f %f flat : %f %f %f \n", fi, (wv1 - wv0).norm(),
                //    (wv2 - wv1).norm(), (wv0 - wv2).norm(), (fv1 - fv0).norm(), (fv2 - fv1).norm(), (fv0 - fv2).norm());
            // printf("fi : %d %d %d %d inconsistent", fi, sheet_indices[ffi][0], sheet_indices[ffi][1], sheet_indices[ffi][2]);
            sheet_indices[ffi] = Eigen::Vector3i(sheet_indices[ffi][0], sheet_indices[ffi][2], sheet_indices[ffi][1]); // and now change the order
            // printf("fi : %d %d %d %d should be consistent now\n", fi, sheet_indices[ffi][0], sheet_indices[ffi][1], sheet_indices[ffi][2]);
            if (verticesTobeFlipped.find(sheet_indices[ffi][0]) == verticesTobeFlipped.end())
                verticesTobeFlipped.insert(sheet_indices[ffi][0]);
            if (verticesTobeFlipped.find(sheet_indices[ffi][1]) == verticesTobeFlipped.end())
                verticesTobeFlipped.insert(sheet_indices[ffi][1]);
            if (verticesTobeFlipped.find(sheet_indices[ffi][2]) == verticesTobeFlipped.end())
                verticesTobeFlipped.insert(sheet_indices[ffi][2]);
        }
        // printf("fi : %d orientation world : %f flat : %f\n", fi, zvec.dot((wv1-wv0).cross(wv2-wv0)), zvec.dot((fv1-fv0).cross(fv2-fv0)));
    }
    for(auto vi: verticesTobeFlipped) {
        Eigen::Vector3d pos = sheet_vertices[vi];
        sheet_vertices[vi] = Eigen::Vector3d(pos(0), -pos(1), pos(2));
    }
}