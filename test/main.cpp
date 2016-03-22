#include <igl/circulation.h>
#include "collapse_edge.h"
#include <igl/edge_flaps.h>
#include <igl/read_triangle_mesh.h>
#include <igl/viewer/Viewer.h>
#include <Eigen/Core>
#include <iostream>
#include <set>
#include <vector>
#include <igl/circulation.h>

using namespace std;
using namespace Eigen;
using namespace igl;

struct MeshModification {
  int vi1;
  int vi2;
  RowVectorXd v1;
  RowVectorXd v2;
  std::vector<int> faceInd;
  MatrixXi faces;
  MeshModification(int vi1, int vi2, RowVectorXd v1, RowVectorXd v2,
		   std::vector<int> faceInd, MatrixXi faces)
    : vi1(vi1), vi2(vi2), v1(v1), v2(v2),faceInd(faceInd), faces(faces) {}
};


int main(int argc, char *argv[]) {
    cout << "Usage: ./703_Decimation_bin [filename.(off|obj|ply)]" << endl;
    cout << "  [space]  toggle animation." << endl;
    cout << "  'r'  reset." << endl;
    // Load a closed manifold mesh
    string filename("fertility.off");
    if (argc >= 2) {
        filename = argv[1];
    }
    MatrixXd V, OV;
    MatrixXi F, OF;
    read_triangle_mesh(filename, OV, OF);

    igl::viewer::Viewer viewer;

    // Prepare array-based edge data structures and priority queue
    VectorXi EMAP;
    MatrixXi E, EF, EI;
    typedef std::set<std::pair<double, int>> PriorityQueue;
    PriorityQueue Q;
    std::vector<PriorityQueue::iterator> Qit;
    // If an edge were collapsed, we'd collapse it to these points:
    MatrixXd C;
    int num_collapsed;
    std::vector<MeshModification> mods;

    // Function for computing cost of collapsing edge (lenght) and placement
    // (midpoint)
    const auto &shortest_edge_and_midpoint = [](
        const int e, const Eigen::MatrixXd &V, const Eigen::MatrixXi & /*F*/,
        const Eigen::MatrixXi &E, const Eigen::VectorXi & /*EMAP*/,
        const Eigen::MatrixXi & /*EF*/, const Eigen::MatrixXi & /*EI*/,
        double &cost, RowVectorXd &p) {
        cost = (V.row(E(e, 0)) - V.row(E(e, 1))).norm();
        p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
    };

    // Function to reset original mesh and data structures
    const auto &reset = [&]() {
        mods.clear();
        F = OF;
        V = OV;
        edge_flaps(F, E, EMAP, EF, EI);
        Qit.resize(E.rows());

        C.resize(E.rows(), V.cols());
        VectorXd costs(E.rows());
        for (int e = 0; e < E.rows(); e++) {
            double cost = e;
            RowVectorXd p(1, 3);
            shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
            C.row(e) = p;
            Qit[e] = Q.insert(std::pair<double, int>(cost, e)).first;
        }
        num_collapsed = 0;
        viewer.data.clear();
        viewer.data.set_mesh(V, F);
        viewer.data.set_face_based(true);
    };

    const auto &collapse_edges = [&](igl::viewer::Viewer &viewer) -> bool {
        // If animating then collapse 10% of edges
        if (viewer.core.is_animating && !Q.empty()) {
            bool something_collapsed = false;
            // collapse edge
            const int max_iter = 1;
            MatrixXd OOV = V;
            MatrixXi OOF = F;
            MatrixXi OOE = E;
            MatrixXi OEF = EF;
            MatrixXi OEI = EI;
            VectorXi OEMAP = EMAP;
            for (int j = 0; j < max_iter; j++) {
	      int e, e1, e2, f1, f2, f3;
	      std::vector<int> faceInd;
	      
                if (!collapse_edge(shortest_edge_and_midpoint, V, F, E, EMAP,
                                   EF, EI, Q, Qit, C, e, e1, e2, f1, f2, faceInd)) {
                    break;
                }
		
		MatrixXi faces(faceInd.size() + 2, 3);
		faceInd.push_back(f1);
		faceInd.push_back(f2);
		for(int i = 0; i < faceInd.size(); i++) {
		  faces.row(i) = OOF.row(faceInd[i]);
		  cout << "ffF" << faces.row(i) << endl;
		}


		
                mods.push_back(
                    MeshModification(OOE(e, 0), OOE(e, 1), OOV.row(OOE(e, 0)),
                                     OOV.row(OOE(e, 1)), faceInd, faces));
                something_collapsed = true;
                num_collapsed++;
            }
            if (something_collapsed) {
                viewer.data.clear();
                viewer.data.set_mesh(V, F);
                viewer.data.set_face_based(true);
            }
        }
        return false;
    };

    const auto &uncollapse_edges = [&](igl::viewer::Viewer &viewer) -> bool {
        if (viewer.core.is_animating && !mods.empty()) {            
            
            
            MeshModification mod = mods.back();
            mods.pop_back();
            V.row(mod.vi1) = mod.v1;
            V.row(mod.vi2) = mod.v2;

	    for(int i = 0; i < mod.faceInd.size(); i++) {
	      F.row(mod.faceInd[i]) = mod.faces.row(i);
	    }
            
         
    
            viewer.data.clear();
            viewer.data.set_mesh(V, F);
            viewer.data.set_face_based(true);



                
        }
    };

    const auto &key_down = [&](igl::viewer::Viewer &viewer, unsigned char key,
                               int mod) -> bool {
        switch (key) {
        case ' ':
            viewer.core.is_animating ^= 1;
            break;
        case 'R':
        case 'r':
            reset();
            break;
        case '1':
            collapse_edges(viewer);
            break;
        case '2':
            uncollapse_edges(viewer);
            break;
        default:
            return false;
        }
        return true;
    };

    reset();
    viewer.core.is_animating = true;
    viewer.callback_key_down = key_down;
    // viewer.callback_pre_draw = pre_draw;
    return viewer.launch();
}
