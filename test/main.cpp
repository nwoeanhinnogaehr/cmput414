#include "collapse_edge.h"
#include <igl/edge_flaps.h>
#include <igl/read_triangle_mesh.h>
#include <igl/viewer/Viewer.h>
#include <Eigen/Core>
#include <iostream>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>
#include <igl/per_face_normals.h>
#include <GLFW/glfw3.h>
#include <stb_image_write.h>
#include <igl/circulation.h>
#include <igl/jet.h>
#include <igl/signed_distance.h>
#include <fstream>

using namespace std;
using namespace Eigen;

// V is current vertices, OV is originally loaded vertices
MatrixXd V, OV;
// F is current faces, OF is originally loaded faces
MatrixXi F, OF;
MatrixXd normals;
MatrixXd colors;

igl::viewer::Viewer viewer;

enum {
    COST_VISUALIZATION,
    SOLID,
    MAX_COLOR_MODE,
} color_mode = SOLID;

// A mesh modification represents a single edge collapse operation in a
// reversible format.
struct MeshModification {
    std::vector<int> vertInd;
    MatrixXd verts;
    std::vector<int> faceInd;
    MatrixXi faces;
    MeshModification(std::vector<int> vertInd, MatrixXd verts,
                     std::vector<int> faceInd, MatrixXi faces)
        : vertInd(vertInd), verts(verts), faceInd(faceInd), faces(faces) {}
};

void save_screenshot(igl::viewer::Viewer &viewer, char *filename) {
    int width, height;
    glfwGetWindowSize(viewer.window, &width, &height);
    char *pixels = new char[3 * width * height];
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);
    stbi_write_png(filename, width, height, 3, pixels, 3 * width);
    delete[] pixels;
}

// The cost functions are defined below.
// The input is e, the index of the edge to collapse.
// They have 2 outputs, cost and p.
// Edges that produce lower cost will be collapsed first.
// p is the midpoint to collapse both vertices of the edge to.

void shortest_edge_and_midpoint1(const int e, const Eigen::MatrixXd &V,
                                 const Eigen::MatrixXi &F,
                                 const Eigen::MatrixXi &E,
                                 const Eigen::VectorXi &EMAP,
                                 const Eigen::MatrixXi &EF,
                                 const Eigen::MatrixXi &EI, double &cost,
                                 RowVectorXd &p) {
    // vectorsum
    const int eflip = E(e, 0) > E(e, 1);
    const std::vector<int> nV2Fd = igl::circulation(e, !eflip, F, E, EMAP, EF, EI);
    p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
    Eigen::RowVectorXd pointy(3);
    pointy.setZero();
    std::set<int> newEdges;
    for (int i = 0; i < nV2Fd.size(); i++) {
        for (int j = 0; j < 3; j++) {
            int curVert = F.row(nV2Fd[i])[j];
            if (curVert != E(e, 0) || curVert != E(e, 1)) {
                if (newEdges.insert(curVert).second) {
                    pointy = (V.row(curVert) - p) + pointy;
                }
            }
        }
    }
    cost = (pointy).norm();
}

void shortest_edge_and_midpoint2(const int e, const Eigen::MatrixXd &V,
                                 const Eigen::MatrixXi &F,
                                 const Eigen::MatrixXi &E,
                                 const Eigen::VectorXi &EMAP,
                                 const Eigen::MatrixXi &EF,
                                 const Eigen::MatrixXi &EI, double &cost,
                                 RowVectorXd &p) {
    // use normals
    const int eflip = E(e, 0) > E(e, 1);
    const std::vector<int> nV2Fd = igl::circulation(e, !eflip, F, E, EMAP, EF, EI);
    p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
    Eigen::RowVectorXd pointy(3);
    pointy.setZero();
    // std::set<int> newEdges;
    for (int i = 0; i < nV2Fd.size(); i++) {
        pointy = normals.row(nV2Fd[i]) + pointy;
    }

    cost = 1 / ((pointy).norm());
}

void shortest_edge_and_midpoint3(const int e, const Eigen::MatrixXd &V,
                                 const Eigen::MatrixXi &F,
                                 const Eigen::MatrixXi &E,
                                 const Eigen::VectorXi &EMAP,
                                 const Eigen::MatrixXi &EF,
                                 const Eigen::MatrixXi &EI, double &cost,
                                 RowVectorXd &p) {
    // manhattan
    cost = (V.row(E(e, 0)) - V.row(E(e, 1))).cwiseAbs().sum();
    p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
}

void shortest_edge_and_midpoint4(const int e, const Eigen::MatrixXd &V,
                                 const Eigen::MatrixXi &F,
                                 const Eigen::MatrixXi &E,
                                 const Eigen::VectorXi &EMAP,
                                 const Eigen::MatrixXi &EF,
                                 const Eigen::MatrixXi &EI, double &cost,
                                 RowVectorXd &p) {
    // euclidean
    cost = (V.row(E(e, 0)) - V.row(E(e, 1))).norm();
    p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
}

void shortest_edge_and_midpoint5(const int e, const Eigen::MatrixXd &V,
                                 const Eigen::MatrixXi &F,
                                 const Eigen::MatrixXi &E,
                                 const Eigen::VectorXi &EMAP,
                                 const Eigen::MatrixXi &EF,
                                 const Eigen::MatrixXi &EI, double &cost,
                                 RowVectorXd &p) {
    // circulation angle sum
    p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
    const vector<int> c1 = igl::circulation(e, false, F, E, EMAP, EF, EI);
    const vector<int> c2 = igl::circulation(e, true, F, E, EMAP, EF, EI);
    unordered_set<int> circ;
    circ.insert(c1.begin(), c1.end());
    circ.insert(c2.begin(), c2.end());
    cost = 0.0;
    for (int face : circ) {
        for (int j = 0; j < 3; j++) {
            int edge = EMAP(face + j * F.rows());
            Vector3d eye = viewer.core.camera_eye.cast<double>();
            eye.normalize();
            cost +=
                max(acos(normals.row(EF(edge, 0)).dot(normals.row(EF(edge, 1))))/3.14159/2.0,
                0.5 *(2.0 - abs(eye.dot(normals.row(EF(edge, 0)))) - abs(eye.dot(normals.row(EF(edge, 1))))));

        }
    }
}

void shortest_edge_and_midpoint6(const int e, const Eigen::MatrixXd &V,
                                 const Eigen::MatrixXi &F,
                                 const Eigen::MatrixXi &E,
                                 const Eigen::VectorXi &EMAP,
                                 const Eigen::MatrixXi &EF,
                                 const Eigen::MatrixXi &EI, double &cost,
                                 RowVectorXd &p) {
    // circulation angle sum
    p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
    const vector<int> c1 = igl::circulation(e, false, F, E, EMAP, EF, EI);
    const vector<int> c2 = igl::circulation(e, true, F, E, EMAP, EF, EI);
    unordered_set<int> circ;
    circ.insert(c1.begin(), c1.end());
    circ.insert(c2.begin(), c2.end());
    cost = 0.0;
    for (int face : circ) {
        for (int j = 0; j < 3; j++) {
            int edge = EMAP(face + j * F.rows());
            cost +=
                acos(normals.row(EF(edge, 0)).dot(normals.row(EF(edge, 1))));

        }
    }
}

void shortest_edge_and_midpoint7(const int e, const Eigen::MatrixXd &V,
                                 const Eigen::MatrixXi &F,
                                 const Eigen::MatrixXi &E,
                                 const Eigen::VectorXi &EMAP,
                                 const Eigen::MatrixXi &EF,
                                 const Eigen::MatrixXi &EI, double &cost,
                                 RowVectorXd &p) {
    // DFS angle sum
    p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
    int MAX_ITER = 15;
    cost = 0.0;
    set<int> visited;
    int n_sum = 0;
    for (int j = 0; j < 2; j++) {
        int face = EF(e, j);
        for (int i = 0; i < MAX_ITER; i++) {
            double max_angle = 0.0;
            int max_k = -1;
            for (int k = 0; k < 3; k++) {
                int edge = EMAP(face + k * F.rows());
                if (visited.find(edge) != visited.end())
                    continue;
                double angle = acos(
                    normals.row(EF(edge, 0)).dot(normals.row(EF(edge, 1))));
                if (angle > max_angle) {
                    max_angle = angle;
                    max_k = k;
                }
            }
            if (max_k == -1)
                break;
            n_sum++;
            cost += max_angle / n_sum;
            int edge = EMAP(face + max_k * F.rows());
            visited.insert(edge);
            if (EF(edge, 0) == face) {
                face = EF(edge, 1);
            } else if (EF(edge, 1) == face) {
                face = EF(edge, 0);
            } else {
                // shouldn't happen
                assert(false);
            }
        }
    }
    cost /= n_sum;
}

double generate_distance_field() {
    VectorXd S;
    VectorXi I;
    MatrixXd C, N;
    igl::signed_distance(OV, V, F, igl::SIGNED_DISTANCE_TYPE_PSEUDONORMAL, S, I, C, N);
    double sum;
    for (int i = 0; i < S.size(); i++) {
        sum += abs(S(i)); //ahhahahha just discard the sign anyways
    }
    return sum;
}

auto shortest_edge_and_midpoint = shortest_edge_and_midpoint1;

int cost_function_n = 0;
auto cost_functions = {shortest_edge_and_midpoint1, shortest_edge_and_midpoint2,
                       shortest_edge_and_midpoint3, shortest_edge_and_midpoint4,
                       shortest_edge_and_midpoint5, shortest_edge_and_midpoint6,
                       shortest_edge_and_midpoint7};

int main(int argc, char *argv[]) {
    cout << "Usage: " << argv[0] << " [FILENAME].[off|obj|ply] [1-7] [sl]"
         << endl;
    cout << "where 1-7 is the cost function to use" << endl;
    cout << "      s = save images at all decimation steps" << endl;
    cout << "      l = disable lighting" << endl;
    cout << endl;
    cout << "Keybindings:" << endl;
    cout << "  [space]  toggle animation." << endl;
    cout << "  'r'  reset." << endl;
    cout << "  '1'  edge collapse." << endl;
    cout << "  '2'  vertex split." << endl;
    cout << "  's'  save screenshot." << endl;
    cout << "  'c'  switch color mode." << endl;
    cout << "  'f'  cycle cost function." << endl;
    cout << endl;
    // Load a closed manifold mesh
    string filename;
    if (argc >= 2) {
        filename = argv[1];
    } else {
        return 0;
    }
    if (argc >= 3) {
        int idx = stoi(argv[2]) - 1;
        cost_function_n = idx;
        if (idx >= 0 && idx < cost_functions.size())
            shortest_edge_and_midpoint = *(cost_functions.begin() + idx);
    }

    if (!igl::read_triangle_mesh(filename, OV, OF)) {
        cout << "could not read mesh from \"" << filename << "\"" << endl;
        return 1;
    }

    // compute normals
    igl::per_face_normals(OV, OF, normals);

    // Prepare array-based edge data structures and priority queue
    // EMAP is a map from faces to edges.
    // Index into it like EMAP(face + i*F.rows()) where i is an edge index
    // between 0 and 2 corresponding to the three edges of a triangle.
    VectorXi EMAP;

    // E is a map from edges to vertices. Given some edge index e,
    // E(e, 0) and E(e, 1) are the two vertices that the edge is composed of.
    MatrixXi E;

    // EF is a map from edges to faces. For some edge index e,
    // EF(e, 0) and E(e, 1) are the two faces that contain the edge e.
    MatrixXi EF;

    // EI is a map from edges to face corners. For some edge index e,
    // EI(e, 0) is the index i such that EMAP(EF(e, 0) + i*F.rows()) == e and
    // EI(e, 1) is the index i such that EMAP(EF(e, 1) + i*F.rows()) == e.
    MatrixXi EI;

    typedef std::set<std::pair<double, int>> PriorityQueue;
    // Q stores the list of possible edge collapses and their costs
    PriorityQueue Q;
    std::vector<PriorityQueue::iterator> Qit;
    // If an edge were collapsed, we'd collapse it to these points:
    MatrixXd C;

    // Keep some info on edge collapses for reversal and debug reasons
    int num_collapsed;
    std::vector<MeshModification> mods;
    std::vector<int> iters;
    int total_decimations = 0;

    const auto &reset_view = [&]() {
        viewer.data.clear();
        viewer.data.set_mesh(V, F);
        switch (color_mode) {
        case COST_VISUALIZATION:
            viewer.data.set_colors(colors);
            break;
        case SOLID:
            viewer.data.set_colors(RowVector3d(1.0, 1.0, 1.0));
            break;
        }
        viewer.data.set_face_based(false);
    };

    // Function to reset original mesh and data structures
    const auto &reset = [&]() {
        total_decimations = 0;
        mods.clear();
        iters.clear();
        F = OF;
        V = OV;
        igl::edge_flaps(F, E, EMAP, EF, EI);
        Qit.resize(E.rows());

        C.resize(E.rows(), V.cols());
        colors.resize(V.rows(), 3);
        colors.setZero();
        VectorXd costs(V.rows());
        costs.setZero();
        for (int e = 0; e < E.rows(); e++) {
            double cost = e;
            RowVectorXd p(1, 3);

            shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
            C.row(e) = p;
            Qit[e] = Q.insert(std::pair<double, int>(cost, e)).first;
            costs(E(e, 0)) += cost;
            costs(E(e, 1)) += cost;
        }
        igl::jet(costs, true, colors);
        num_collapsed = 0;
        reset_view();
    };

    const auto &collapse_edges = [&](igl::viewer::Viewer &viewer) -> bool {
        // If animating then collapse 10% of edges
        if (viewer.core.is_animating && !Q.empty()) {
            bool something_collapsed = false;
            // collapse edge
            const int num_iters = 50;

            // Store the state from before the collapse so that it can be
            // reversed later.
            MatrixXd prev_V = V;
            MatrixXi prev_F = F;
            MatrixXi prev_E = E;
            num_collapsed = 0;

            int total_failures = 0; // If a certain number of failures have
                                // occurred, we exit an infinte fail loop.

            for (int j = 0; j < num_iters; j++) {
                int e, e1, e2, f1, f2;
                std::vector<int> faceInd, vertInd;

                if (Q.empty())
                    break;

                if (!collapse_edge(shortest_edge_and_midpoint, V, F, E, EMAP,
                                   EF, EI, Q, Qit, C, e, e1, e2, f1, f2,
                                   faceInd)) {
                    total_failures++;
                    j--;
                    if (total_failures > 1000) {
                        break;
                    }
                    continue;
                } else {
                    total_decimations++;
                    num_collapsed++;
                }

                MatrixXi faces(faceInd.size() + 2, 3);
                faceInd.push_back(f1);
                faceInd.push_back(f2);
                for (int i = 0; i < faceInd.size(); i++) {
                    faces.row(i) = prev_F.row(faceInd[i]);
                    // cout << "ffF" << faces.row(i) << endl;
                }

                MatrixXd verts(2, 3);
                vertInd.push_back(prev_E(e, 0));
                vertInd.push_back(prev_E(e, 1));
                for (int i = 0; i < vertInd.size(); i++) {
                    verts.row(i) = prev_V.row(vertInd[i]);
                }

                mods.push_back(
                    MeshModification(vertInd, verts, faceInd, faces));
                something_collapsed = true;
            }
            if (something_collapsed) {
                iters.push_back(num_collapsed);
                reset_view();
            }
        }
        cout << "Collapsed an Edge\n"
             << "Decimations: " << total_decimations << "\n";
        return false;
    };

    const auto &uncollapse_edges = [&](igl::viewer::Viewer &viewer) -> bool {
        if (viewer.core.is_animating && !mods.empty() && !iters.empty()) {

            int max_iter = iters.back();
            iters.pop_back();

            for (int i = 0; i < max_iter; i++) {
                MeshModification mod = mods.back();
                mods.pop_back();
                total_decimations--;

                for (int i = 0; i < mod.vertInd.size(); i++) {
                    V.row(mod.vertInd[i]) = mod.verts.row(i);
                }

                for (int i = 0; i < mod.faceInd.size(); i++) {
                    F.row(mod.faceInd[i]) = mod.faces.row(i);
                }
            }

            reset_view();
            cout << "Uncollapsed an Edge\n"
                 << "Decimations: " << total_decimations << "\n";
        }
    };

    const auto &save_images = [&]() -> bool {
        reset();
        viewer.draw();

        save_screenshot(viewer, "images/before.png");
        char fn[100];
        char command[512];
        ofstream distfile("surface_distances", ofstream::trunc);
        for (int i = 0; i <= 100; i++) {
            collapse_edges(viewer);
            distfile << generate_distance_field() << endl;
            viewer.draw();
            sprintf(fn, "images/after%03d.png", i);
            save_screenshot(viewer, fn);
            sprintf(command, "composite images/before.png "
                             "images/after%03d.png -compose difference "
                             "images/diff%03d.png ",
                    i, i);
            system(command);
            sprintf(command, "composite images/after%03d.png "
                             "images/after%03d.png -compose difference "
                             "images/delta%03d.png ",
                    i, i - 1, i);
            system(command);
            cout << "Step " << i << " / 100" << endl;
        }
        distfile.close();
        exit(EXIT_SUCCESS);

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
            save_images();
            break;
        case 'S':
        case 's':
            save_screenshot(viewer, "images/screen.png");
            cout << "saved screen to images/screen.png" << endl;
            break;
        case 'C':
        case 'c':
            ((int &)color_mode)++;
            ((int &)color_mode) %= MAX_COLOR_MODE;
            reset_view();
            break;
        case 'F':
        case 'f':
            cost_function_n++;
            cost_function_n %= cost_functions.size();
            shortest_edge_and_midpoint =
                *(cost_functions.begin() + cost_function_n);
            reset();
            break;
        case 'g':
        case 'G':
            cout << generate_distance_field() << endl;
            break;
        default:
            return false;
        }
        return true;
    };

    const auto &s_option = [&](igl::viewer::Viewer &viewer) -> bool {
        if (argc >= 4) {
            for (char c : string(argv[3])) {
                switch (c) {
                case 's':
                    save_images();
                    break;
                case 'l':
                    viewer.core.shininess = 1.0;
                    viewer.core.lighting_factor = 0.0;
                    break;
                }
            }
        }
    };

    reset();
    viewer.core.is_animating = true;
    viewer.callback_key_pressed = key_down;
    viewer.callback_init = s_option;
    viewer.core.show_lines = false;
    viewer.core.camera_zoom = 2.0;
    return viewer.launch();
}
