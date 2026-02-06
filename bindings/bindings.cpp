#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>   // Automatic conversion Eigen <-> NumPy
#include <pybind11/stl.h>     // Automatic conversion std::vector <-> List
#include <pybind11/numpy.h>   // For py::array_t

#include "QuaternionLib/quaternion.h"
#include "QuaternionLib/stateful_converter.h"

namespace py = pybind11;
using namespace QuatLib;

// We bind the 'double' specialization for Python
using BScalar = double;

// Renamed function to avoid potential conflict with Pybind macros
void register_quaternionlib(py::module_ &m) {
    m.doc() = "Continuity-Aware Quaternion to Euler conversion library";

    // 1. Bind the RotationOrder Enum
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

    // 2. Bind the EulerAngles Struct
    py::class_<EulerAngles<BScalar>>(m, "EulerAngles")
        // Use def_property_readonly with a lambda to force a copy of the Eigen vector
        // back to Python.
        .def_property_readonly("angles", [](const EulerAngles<BScalar>& self) -> Eigen::Vector3d {
            return self.angles;
        }, "The Euler angles [Alpha, Beta, Gamma] in radians");

    // 3. Bind the StatefulConverter
    py::class_<StatefulConverter<BScalar>>(m, "Converter")
        .def(py::init<RotationOrder, size_t>(),
             py::arg("order"), py::arg("history_size") = 5,
             "Initialize the converter with a rotation order and history buffer size.")

        // Expose update taking numpy array explicitly
        // SAFETY FIX: We use py::array::c_style | py::array::forcecast to ensure the
        // underlying buffer is contiguous and of type double. This prevents crashes
        // if the user passes a non-contiguous slice (e.g., arr[::2]).
        .def("update", [](StatefulConverter<BScalar>& self, py::array_t<double, py::array::c_style | py::array::forcecast> input_array) {
             py::buffer_info buf = input_array.request();

             if (buf.ndim != 1 || buf.size != 4) {
                 throw std::runtime_error("Input quaternion must be a 1D array of 4 elements [x, y, z, w]");
             }

             // Manual copy to avoid Eigen alignment crashes with pybind buffers.
             // Accessing 'ptr' is now safe because we enforced contiguous memory above.
             double* ptr = static_cast<double*>(buf.ptr);
             Eigen::Quaternion<BScalar> q;
             // Eigen stores coeffs as (x, y, z, w)
             q.coeffs()[0] = ptr[0]; // x
             q.coeffs()[1] = ptr[1]; // y
             q.coeffs()[2] = ptr[2]; // z
             q.coeffs()[3] = ptr[3]; // w

             // Normalize safely
             // Using stableNorm to avoid overflow/underflow issues, though rare for quats
             if (q.norm() > 1e-6) {
                 q.normalize();
             } else {
                 q.setIdentity();
             }
             return self.update(q);
        },
        py::arg("coeffs"),
        "Convert quaternion coefficients [x, y, z, w].")

        .def("reset", &StatefulConverter<BScalar>::reset, "Clear the history buffer.")

        .def("set_order", &StatefulConverter<BScalar>::set_order, py::arg("new_order"),
             "Change the rotation order (resets history).");
}

// Ensure the module name matches CMake output name (quaternionlib)
PYBIND11_MODULE(quaternionlib, m) {
    register_quaternionlib(m);
}