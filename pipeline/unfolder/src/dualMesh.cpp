#include "common.h"

void dualMesh(const igl::opengl::ViewerData &mesh, igl::opengl::ViewerData &dual_mesh, Eigen::MatrixXi &dual_edges, Eigen::MatrixXi &visual_dual_edges, Eigen::MatrixXi &boundary_edges)
// igl::opengl::ViewerData dualMesh(igl::opengl::ViewerData mesh)
{
    igl::opengl::ViewerData outMesh;
    Eigen::MatrixXd V, Vd;
    Eigen::MatrixXi F;
    V = mesh.V;
    F = mesh.F;
    // Compute face barycenter
    Eigen::MatrixXd BC;
    igl::barycenter(V, F, BC);

    // mesh.add_edges
    Eigen::MatrixXi Ed;
    Eigen::MatrixXi EV, FE, EF;
    igl::edge_topology(V, F, EV, FE, EF);
    Eigen::MatrixXd VatEc; // vertices at the mid-point of edges.

    Eigen::MatrixXd C;
    VatEc.resize(EV.rows(), 3);
    for (size_t ei = 0; ei < EV.rows(); ei++)
    {
        Eigen::Vector3d v0 = V.row(EV(ei, 0));
        Eigen::Vector3d v1 = V.row(EV(ei, 1));
        VatEc.row(ei) = (v0 + v1) / 2.0;
        // C.row(ei) = red_color;
    }

    // Add the barycenters to the vertices
    Vd.resize(V.rows() + F.rows() + EV.rows(), 3); // one for each previous vertex, one for each face at the barycenter and one for each edge midpoint
    Vd.block(0, 0, V.rows(), 3) = V;            // add first the barycenter of the points
    Vd.block(V.rows(), 0, F.rows(), 3) = BC;            // add first the barycenter of the points
    Vd.block(V.rows() + F.rows(), 0, EV.rows(), 3) = VatEc; // now add the edge mid-points

    int inner_edges = 0;
    int count_boundary_edges = 0; 
    for (size_t ei = 0; ei < EV.rows(); ++ei) {
        int f0 = EF(ei, 0);
        int f1 = EF(ei, 1);

        if(f0 == 3 and f1 == 37) {
            int f0e0 = FE.row(f0)[0], f0e1 = FE.row(f0)[1], f0e2 = FE.row(f0)[2];
            int f1e0 = FE.row(f1)[0], f1e1 = FE.row(f1)[1], f1e2 = FE.row(f1)[2];
            printf("\n");
        }

        if(f0 == -1 or f1 == -1) {
            count_boundary_edges++; 
        }
        else {
            inner_edges++;
        }
    }

    // Each face is split four ways
    C.resize(inner_edges * 2, 3);
    Ed.resize(inner_edges * 2, 2); // per face there are 3 different edges joining barycenter to the mid-point
    dual_edges.resize(inner_edges, 2); // dual edges are same as actual edges. 1-1 correspondance. ADDENDUM NOT TRUE FOR OPEN MESHES
    boundary_edges.resize(count_boundary_edges, 3); // dual edges are same as actual edges. 1-1 correspondance. ADDENDUM NOT TRUE FOR OPEN MESHES
    
    uint Edi = 0; // count for dual edges
    uint Ebi = 0; // count for boundary edges
    for (int ei = 0; ei < EF.rows(); ++ei) {
        // Edge to face mapping
        int f0 = EF(ei, 0);
        int f1 = EF(ei, 1);

        if(f0 == -1 or f1 == -1) {
            int v0 = EV.row(ei)[0], v1 = EV.row(ei)[1]; // nice thing that it stored v0 and v1 in the lexigraphic order
            // printf("ei : %d f0 : %d f1 : %d v0 : %d v1 : %d\n", ei, f0, f1, v0, v1);
            boundary_edges.row(Ebi) = (f0 == -1) ? Eigen::Vector3i(f1, v0, v1) : Eigen::Vector3i(f0, v0, v1); // store the face and the vertices that forms part of it!!
            Ebi++ ;
            continue;
        }

        // printf("ei : %d f0 : %d f1 : %d v0 : %d v1 : %d\n", ei, f0, f1, EV.row(ei)[0], EV.row(ei)[1]);
        dual_edges.row(int(Edi/2)) = Eigen::Vector2i(f0, f1); // basically matches the edge that we need to have in the unfolded version
        // printf("0 dual edges : %ld %d %d\n", ei, dual_edges.row(ei)(0), dual_edges.row(ei)(1));

        // Edge is from barycenter to mid point and then from mid point to barycenter.
        Ed.row(Edi) = Eigen::Vector2i(V.rows()+f0, V.rows()+F.rows() + ei);
        C.row(Edi) = green_color; // update color
        Edi++; // Vertex corresponding to BC[f0] and VatEc[ei]
        
        Ed.row(Edi) = Eigen::Vector2i(V.rows()+F.rows() + ei, V.rows()+f1);
        C.row(Edi) = green_color; 
        Edi++; 
    }

    dual_mesh.set_mesh(Vd, F); // this step is fine since i am storing the same vertices at the start of the vertex list
    dual_mesh.set_edges(Vd, Ed, C); // aditional edges for showing the dual edges part on top of the original mesh
    visual_dual_edges = Ed;
    printf("Generated dual mesh, vertex size %ld %ld dual edge size : %ld visual dual edge size %ld \n", dual_mesh.V.size(), V.size(), dual_edges.rows(), visual_dual_edges.rows());
}

// https://github.com/libigl/libigl/blob/main/include/igl/facet_components.h would come in handy for computing the connected components


void setDualMeshColor(igl::opengl::ViewerData &dual_mesh, const Eigen::MatrixXi &visual_dual_edges, Eigen::MatrixXd MST_color) {
    dual_mesh.clear_edges();
    Eigen::MatrixXd color;
    if(MST_color.size() == 0) {
        color.resize(visual_dual_edges.rows(), 3);
        for (size_t i = 0; i < visual_dual_edges.rows(); i++) {
            color.row(i) = green_color;
        }
    }
    else {
        color = MST_color;
    }
    dual_mesh.set_edges(dual_mesh.V, visual_dual_edges, color);
}