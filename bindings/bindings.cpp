#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "QuaternionLib/quaternion.h"
#include "QuaternionLib/stateful_converter.h"

namespace py = pybind11;
using namespace QuatLib;

using BScalar = double;

void register_quaternionlib(py::module_ &m) {
    m.doc() = "Continuity-Aware Quaternion to Euler conversion library";

    py::enum_<RotationOrder>(m, "RotationOrder")
        .value("XYX", RotationOrder::XYX)
        .value("YZY", RotationOrder::YZY)
        .value("ZXZ", RotationOrder::ZXZ)
        .value("XZX", RotationOrder::XZX)
        .value("YXY", RotationOrder::YXY)
        .value("ZYZ", RotationOrder::ZYZ)
        .value("XYZ", RotationOrder::XYZ)
        .value("YZX", RotationOrder::YZX)
        .value("ZXY", RotationOrder::ZXY)
        .value("XZY", RotationOrder::XZY)
        .value("YXZ", RotationOrder::YXZ)
        .value("ZYX", RotationOrder::ZYX)
        .export_values();

    py::class_<EulerAngles<BScalar>>(m, "EulerAngles")
        .def_property_readonly("angles", [](const EulerAngles<BScalar>& self) -> Eigen::Vector3d {
            return self.angles;
        }, "The Euler angles [Alpha, Beta, Gamma] in radians");

    py::class_<StatefulConverter<BScalar>>(m, "Converter")
        .def(py::init<RotationOrder, size_t>(),
             py::arg("order"), py::arg("history_size") = 5,
             "Initialize the converter with a rotation order and history buffer size.")

        // Updated update method with optional dt
        .def("update", [](StatefulConverter<BScalar>& self,
                          py::array_t<double, py::array::c_style | py::array::forcecast> input_array,
                          double dt) {

             py::buffer_info buf = input_array.request();

             if (buf.ndim != 1 || buf.size != 4) {
                 throw std::runtime_error("Input quaternion must be a 1D array of 4 elements [x, y, z, w]");
             }

             double* ptr = static_cast<double*>(buf.ptr);
             Eigen::Quaternion<BScalar> q;
             q.coeffs()[0] = ptr[0];
             q.coeffs()[1] = ptr[1];
             q.coeffs()[2] = ptr[2];
             q.coeffs()[3] = ptr[3];

             if (q.norm() > 1e-6) q.normalize();
             else q.setIdentity();

             return self.update(q, dt);
        },
        py::arg("coeffs"), py::arg("dt") = 1.0,
        "Convert quaternion coefficients [x, y, z, w]. Optional dt (seconds) for variable frame rates.")

        .def("reset", &StatefulConverter<BScalar>::reset, "Clear the history buffer.")
        .def("set_order", &StatefulConverter<BScalar>::set_order, py::arg("new_order"),
             "Change the rotation order (resets history).");
}

PYBIND11_MODULE(quaternionlib, m) {
    register_quaternionlib(m);
}