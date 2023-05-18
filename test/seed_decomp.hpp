#include <decomp_util/seed_decomp.h>
#include <decomp_geometry/geometric_utils.h>

#include <fstream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <vector>
using std::vector;

std::vector<vector<double>>  seed_decomp(float x, float y, std::vector<vector<double>> obs_input) {
  // Obstacles
  vec_Vec2f obs;
  for (unsigned int i = 0; i < obs_input.size(); ++i){
      obs.push_back(Vec2f(obs_input[i][0], obs_input[i][1]));
  }

  // Seed
  const Vec2f pos(x, y);

  // Initialize SeedDecomp2D
  SeedDecomp2D decomp(pos);
  decomp.set_obs(obs);
  decomp.set_local_bbox(Vec2f(1, 1));
  decomp.dilate(.05);
  // Get the result
  auto poly = decomp.get_polyhedron();
  // Convert to linear constraints
  std::vector<vector<double>>  Ab_mat;
  const Vecf<2>& pt = {x, y};
  LinearConstraint2D cs(pt, poly.hyperplanes());
  for (int i = 0; i < cs.A_.rows(); ++i) {
    Ab_mat.push_back(std::vector<double>());
    for (int j = 0; j < cs.A_.cols(); ++j) {
        Ab_mat[i].push_back(cs.A_(i, j));
    }
    Ab_mat[i].push_back(cs.b_(i));
  }

  return Ab_mat;
}