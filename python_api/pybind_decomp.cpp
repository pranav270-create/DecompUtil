#include "../test/seed_decomp.hpp"
#include "../pybind11/include/pybind11/pybind11.h"
// to convert C++ STL containers to python list, see T800.GetData()
#include "../pybind11/include/pybind11/stl.h" 

namespace py = pybind11;
using namespace std;

PYBIND11_MODULE(py_seed_decomp, m) {
    m.doc() = "pybind seed decomp"; // optional module docstring

    py::class_<DecompRet>(m, "DecompRet")
        .def(py::init())
        .def("getAb", &DecompRet::get_Ab)
        .def("getArea", &DecompRet::get_area)
        .def("getVertices", &DecompRet::get_vertices);

    m.def("seed_decomp", &seed_decomp, "A function which finds the convex safe set in a point cloud",
      py::arg("x"), py::arg("y"), py::arg("obs_input"), py::arg("local_bbox"), py::return_value_policy::reference);
}
