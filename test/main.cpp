#include "collapse_edge.h"
#include <igl/edge_flaps.h>
#include <igl/read_triangle_mesh.h>
#include <igl/viewer/Viewer.h>
#include <Eigen/Core>
#include <iostream>
#include <set>
#include <vector>
#include <igl/per_face_normals.h>
#include <GLFW/glfw3.h>
#include <stb_image_write.h>
#include <igl/circulation.h>

using namespace std;
using namespace Eigen;
using namespace igl;
MatrixXd V, OV;
MatrixXi F, OF;
MatrixXd normals;


struct MeshModification {
    std::vector<int> vertInd;
    MatrixXd verts;
    std::vector<int> faceInd;
    MatrixXi faces;
    MeshModification(std::vector<int> vertInd, MatrixXd verts,
                     std::vector<int> faceInd, MatrixXi faces)
        : vertInd(vertInd), verts(verts), faceInd(faceInd), faces(faces) {}
};

void save_screenshot(viewer::Viewer &viewer, char *filename) {
    int width, height;
    glfwGetWindowSize(viewer.window, &width, &height);
    char *pixels = new char[3 * width * height];
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);
    stbi_write_png(filename, width, height, 3, pixels, 3 * width);
    delete[] pixels;
}

void shortest_edge_and_midpoint(const int e, const Eigen::MatrixXd &V,
                                const Eigen::MatrixXi &F,
                                const Eigen::MatrixXi &E,
                                const Eigen::VectorXi &EMAP,
                                const Eigen::MatrixXi &EF,
                                const Eigen::MatrixXi &EI, double &cost,
                                RowVectorXd &p) {
    // manhattan
    //cost = (V.row(E(e, 0)) - V.row(E(e, 1))).cwiseAbs().sum();
    // euclidean
    // cost = (V.row(E(e, 0)) - V.row(E(e, 1))).norm();
    //p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
    // vectorsum
    // const int eflip = E(e, 0) > E(e, 1);
    // const std::vector<int> nV2Fd = circulation(e, !eflip, F, E, EMAP, EF, EI);
    // p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
    // Eigen::RowVectorXd pointy(3);
    // pointy.setZero();
    // std::set<int> newEdges;
    // for( int i = 0; i < nV2Fd.size(); i++) {
    //   for( int j = 0; j < 3; j++) {
    // 	int curVert = F.row(nV2Fd[i])[j];
    // 	if( curVert != E(e, 0) || curVert != E(e, 1)){
    // 	  if(newEdges.insert(curVert).second){
    // 	    pointy = (V.row(curVert) - p) + pointy;
    // 	  }
    // 	}
    //   }
    // }
    // cost = (pointy).norm();

       // compute normals


    const int eflip = E(e, 0) > E(e, 1);
    const std::vector<int> nV2Fd = circulation(e, !eflip, F, E, EMAP, EF, EI);
    p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
    Eigen::RowVectorXd pointy(3);
    pointy.setZero();
    std::set<int> newEdges;
    for( int i = 0; i < nV2Fd.size(); i++) {

      pointy = normals.row(nV2Fd[i]) + pointy;
    }

    cost = 1/((pointy).norm());

}

int main(int argc, char *argv[]) {
    cout << "Usage: " << argv[0] << " [filename.(off|obj|ply)]" << endl;
    cout << "  [space]  toggle animation." << endl;
    cout << "  'r'  reset." << endl;
    cout << "  '1'  edge collapse." << endl;
    cout << "  '2'  vertex split." << endl;
    // Load a closed manifold mesh
    string filename("fertility.off");
    if (argc >= 2) {
        filename = argv[1];
    }

    read_triangle_mesh(filename, OV, OF);

    // compute normals
    per_face_normals(OV, OF, normals);

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
    std::vector<int> iters;


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
    int decimationsTotal = 0;
    const auto &collapse_edges = [&](igl::viewer::Viewer &viewer) -> bool {
        // If animating then collapse 10% of edges
        if (viewer.core.is_animating && !Q.empty()) {
            bool something_collapsed = false;
            // collapse edge
            const int max_iter = std::ceil(1.0 * Q.size());

            MatrixXd OOV = V;
            MatrixXi OOF = F;
            MatrixXi OOE = E;
            MatrixXi OEF = EF;
            MatrixXi OEI = EI;
            VectorXi OEMAP = EMAP;
            num_collapsed = 0;
            for (int j = 0; j < max_iter; j++) {
                int e, e1, e2, f1, f2;
                std::vector<int> faceInd, vertInd;

                if (!collapse_edge(shortest_edge_and_midpoint, V, F, E, EMAP,
                                   EF, EI, Q, Qit, C, e, e1, e2, f1, f2,
                                   faceInd)) {
                    break;
                }

                decimationsTotal++;

                MatrixXi faces(faceInd.size() + 2, 3);
                faceInd.push_back(f1);
                faceInd.push_back(f2);
                for (int i = 0; i < faceInd.size(); i++) {
                    faces.row(i) = OOF.row(faceInd[i]);
                    // cout << "ffF" << faces.row(i) << endl;
                }

                MatrixXd verts(2, 3);
                vertInd.push_back(OOE(e, 0));
                vertInd.push_back(OOE(e, 1));
                for (int i = 0; i < vertInd.size(); i++) {
                    verts.row(i) = OOV.row(vertInd[i]);
                }

                mods.push_back(
                    MeshModification(vertInd, verts, faceInd, faces));
                something_collapsed = true;
                num_collapsed++;
            }
            if (something_collapsed) {
                iters.push_back(num_collapsed);
                viewer.data.clear();
                viewer.data.set_mesh(V, F);
                viewer.data.set_face_based(true);
            }
        }
        cout << "Collapsed an Edge\n" << "Decimations: " << decimationsTotal << "\n";
        return false;
    };

    const auto &uncollapse_edges = [&](igl::viewer::Viewer &viewer) -> bool {
        if (viewer.core.is_animating && !mods.empty() && !iters.empty()) {

            int max_iter = iters.back();
            iters.pop_back();

            for (int i = 0; i < max_iter; i++) {
                MeshModification mod = mods.back();
                mods.pop_back();
                decimationsTotal--;

                for (int i = 0; i < mod.vertInd.size(); i++) {
                    V.row(mod.vertInd[i]) = mod.verts.row(i);
                }

                for (int i = 0; i < mod.faceInd.size(); i++) {
                    F.row(mod.faceInd[i]) = mod.faces.row(i);
                }
            }

            viewer.data.clear();
            viewer.data.set_mesh(V, F);
            viewer.data.set_face_based(true);
            cout << "Unollapsed an Edge\n" << "Decimations: " << decimationsTotal << "\n";
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
        case '3':
            reset();
            viewer.draw();
            save_screenshot(viewer, "before.png");
            char fn[100];
            char command[512];
            for (int i = 0; i < 100; i++) {
                collapse_edges(viewer);
                viewer.draw();
                sprintf(fn, "after%03d.png", i);
                save_screenshot(viewer, fn);
                sprintf(command, "composite before.png after%03d.png -compose difference "
                                 "diff%03d.png ",
                        i, i);
                system(command);
                sprintf(command, "composite after%03d.png after%03d.png -compose difference "
                                 "delta%03d.png ",
                        i, i - 1, i);
                system(command);
                cout << "Step " << i << " / 100" << endl;
            }
            break;
        case 'S':
        case 's':
            save_screenshot(viewer, "screen.png");
            cout << "saved screen to screen.png" << endl;
            break;
        default:
            return false;
        }
        return true;
    };

    reset();
    viewer.core.is_animating = true;
    viewer.callback_key_pressed = key_down;
    viewer.core.show_lines = false;
    return viewer.launch();
}
