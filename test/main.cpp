#include <igl/readOFF.h>
#include <igl/viewer/Viewer.h>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

bool key_down(igl::viewer::Viewer &viewer, unsigned char key, int modifier) {
  std::cout << "key press: '" << key << "'" << std::endl;
  if (key == 'N') {
    // TODO increase quality
  } else if (key == 'M') {
    // TODO decrease quality
  }
  return false;
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cout << "usage: " << argv[0] << " mesh" << std::endl;
    return 1;
  }
  char *filename = argv[1];

  std::cout << "n - increase quality" << std::endl
            << "m - decrease quality" << std::endl;

  // Load a mesh in OFF format
  igl::readOFF(filename, V, F);

  // Plot the mesh
  igl::viewer::Viewer viewer;
  viewer.callback_key_down = &key_down;
  viewer.data.set_mesh(V, F);
  viewer.launch();
}
