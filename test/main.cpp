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
    int fi1;
    int fi2;
    int fi3;
    RowVectorXi f1;
    RowVectorXi f2;
    RowVectorXi f3;
    std::vector<int> edgeVec;
    std::vector<int> neighbours;
    int eflip;
    MeshModification(int vi1, int vi2, RowVectorXd v1, RowVectorXd v2, int fi1,
                     int fi2, int fi3, RowVectorXi f1, RowVectorXi f2,
                     RowVectorXi f3, std::vector<int> edgeVec, std::vector<int> neighbours, int eflip)
        : vi1(vi1), vi2(vi2), v1(v1), v2(v2), fi1(fi1), fi2(fi2), fi3(fi3),
          f1(f1), f2(f2), f3(f3), edgeVec(edgeVec), neighbours(neighbours), eflip(eflip) {}
};

inline int Factorial(int x) {
  return (x == 1 ? x : x * Factorial(x - 1));
}

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
            cout << "V" << V << endl;
            cout << "F" << F << endl;
            //cout << "E" << E << endl;
            for (int j = 0; j < max_iter; j++) {
                int e, e1, e2, f1, f2, f3;
                
                
                if (!collapse_edge(shortest_edge_and_midpoint, V, F, E, EMAP,
                                   EF, EI, Q, Qit, C, e, e1, e2, f1, f2, f3)) {
                    break;
                }




                const int eflip = OOE(e, 0) > OOE(e, 1);
                const std::vector<int> neigh = circulation(e, !eflip, OOF, OOE, OEMAP, OEF, OEI);

                std::set<int> circSet;
                for (int k =0; k < neigh.size(); k++){
                    std::list<int> circNode;
                    for (int l =0; l < 3; l++) {
                        (OOF.row(neigh[k])[l] != OOE(e, !eflip)) ? circNode.push_back(neigh[k])[l];    
                    }
                    
                                
                            
                            
                            

                            
                    }

                }
                


                std::set<int> faceSet;


                
                for (int k =0; k < neigh.size(); k++){
                    // cout << neigh[k] << "sadfs" << endl;
                    if (OOF.row(neigh[k])[0] != OOE(e, !eflip)) {
                        faceSet.insert(OOF.row(neigh[k])[0]);
                    }
                    if (OOF.row(neigh[k])[1] != OOE(e, !eflip)) {
                        faceSet.insert(OOF.row(neigh[k])[1]);
                    }
                    if (OOF.row(neigh[k])[2] != OOE(e, !eflip)) {
                        faceSet.insert(OOF.row(neigh[k])[2]);
                    }
                }
                std::vector<int> v(faceSet.begin(), faceSet.end());
               


                
                //cout << e << ", " << e1 << ", " << e2 << ", " << f1 << ", " << f2 << "," << EF << "," << EI << ","  << "," << "," << e << "," << e1 << "," << e2 << "," << f1 << ","<< f2 << ","<< f3 << endl;
                for ( int fuh =0; fuh < neigh.size(); fuh++){

                    cout << "sf" << neigh[fuh] << endl;
                }
                cout << "eflip " << OOE(e, !eflip) << endl;
                
                
                mods.push_back(
                    MeshModification(OOE(e, 0), OOE(e, 1), OOV.row(OOE(e, 0)),
                                     OOV.row(OOE(e, 1)), f1, f2, f3,
                                     OOF.row(f1), OOF.row(f2), OOF.row(f3), v, neigh, OOE(e, !eflip)));
                something_collapsed = true;
                num_collapsed++;
            }
            cout << "V" << V << endl;
            cout << "F" << F << endl;
            //cout << "E" << E << endl;

            // ZEROD: F,E,EF,EI
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
            //F.row(mod.fi1) = mod.f1;
            // F.row(mod.fi2) = mod.f2;
            // F.row(mod.fi3) = mod.f3;
            V.row(mod.vi1) = mod.v1;
            V.row(mod.vi2) = mod.v2;
            std::vector<int> nodes = mod.edgeVec;
            std::vector<int> neighbours = mod.neighbours;
            
            
            cout << "neefsg " << neighbours.size() << endl;
            
            
            
            int newFaces[neighbours.size()][3];// Factorial(nodes.size())/(2
                                               // *
                                               // Factorial(nodes.size()-2)));
            cout << "fact" << Factorial(nodes.size())/(2 * Factorial(nodes.size()-2)) << endl;
        
            std::string bitmask(2, 1);
            bitmask.resize(nodes.size(), 0);
            int j = 0;
                        

            
            do {
                newFaces[j][0] = mod.eflip;
                int temp = 1;
                for (int i = 0; i < nodes.size(); ++i)
                {
                    if (bitmask[i]) {
                        std::cout << " " << nodes[i];
                        // newFaces[j][temp] = nodes[i];
                        ++temp;                        
                    }
                    
                }
                std::cout << std::endl;
                ++j;
            } while (std::prev_permutation(bitmask.begin(), bitmask.end()));


            for (int i = 0; i < neighbours.size(); ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    std::cout << newFaces[i][j] << ' ';
                    F.row(neighbours[i])[j] = newFaces[i][j];
                    
                }
                std::cout << std::endl;
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
