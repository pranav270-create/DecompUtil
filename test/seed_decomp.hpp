#include <decomp_util/seed_decomp.h>
#include <decomp_geometry/geometric_utils.h>

#include <fstream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <vector>
using std::vector;

class DecompRet {
  public:
    //Null constructor
    DecompRet() {}

    DecompRet(std::vector<vector<double>> &Ab, double &area, std::vector<vector<double>> &vertices) {
      Ab_ = Ab;
      area_ = area;
      vertices_ = vertices;
    }

    //Import obstacle points
    void set_Ab(std::vector<vector<double>> &Ab) {
      Ab_ = Ab;
    }

    void set_area(double &area) {
      area_ = area;
    }

    void set_vertices(std::vector<vector<double>> &vertices) {
      vertices_ = vertices;
    }

    std::vector<vector<double>> Ab_;
    double area_;
    std::vector<vector<double>> vertices_;

    ///Get Ab
    std::vector<vector<double>> get_Ab() const { return Ab_; }

    ///Get vertices
    std::vector<vector<double>> get_vertices() const { return vertices_; }

    ///Get area
    double get_area() const { return area_; }
};

DecompRet seed_decomp(float x, float y, std::vector<vector<double>> obs_input) {
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
  decomp.dilate(.01);
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
  const auto vertices = cal_vertices(poly);
  std::vector<vector<double>> vertices_mat;
  double area = 0;
  for (size_t i = 0; i < vertices.size(); i++){
    int j = (i + 1) % vertices.size();
    vertices_mat.push_back(std::vector<double>());
    vertices_mat[i].push_back(vertices[i](0));
    vertices_mat[i].push_back(vertices[i](1));
    // x1, y1 = i(0) i(1)
    // x2, y2 = j(0) j(1)
    area += (vertices[i](0) * vertices[j](1) - vertices[j](0) * vertices[i](1));
  }
  area = fabs(area) / 2;
  DecompRet decomp_ret(Ab_mat, area, vertices_mat);
  return decomp_ret;
}