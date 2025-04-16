#include <LinearTireConfig.h>
#include <pybind11/embed.h>
namespace py = pybind11;

void LinearTireConfig::load_config(std::string racecar_version) {
    py::scoped_interpreter guard{}; // Start the Python interpreter

    py::module car_config_module = py::module::import("pbl_config");  // Import your Python module
    py::object load_linear_tire_config_ros = car_config_module.attr("load_linear_tire_config_ros");

    py::dict config_dict = load_linear_tire_config_ros(racecar_version, this->floor);  // Call Python function and get the result

    C_Sf = config_dict["C_Sf"].cast<double>();
    C_Sr = config_dict["C_Sr"].cast<double>();
    floor = config_dict["floor"].cast<std::string>();
}
