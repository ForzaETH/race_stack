#include <PacejkaTireConfig.h>
#include <pybind11/embed.h>
namespace py = pybind11;

void PacejkaTireConfig::load_config(std::string racecar_version) {
    py::scoped_interpreter guard{}; // Start the Python interpreter

    py::module car_config_module = py::module::import("pbl_config.PacejkaTireConfig");  // Import your Python module
    py::object load_pacejka_tire_config_ros = car_config_module.attr("load_pacejka_tire_config_ros");

    py::dict config_dict = load_pacejka_tire_config_ros(racecar_version, this->floor);  // Call Python function and get the result
    friction_coeff = config_dict["friction_coeff"].cast<double>();
    Bf = config_dict["Bf"].cast<double>();
    Cf = config_dict["Cf"].cast<double>();
    Df = config_dict["Df"].cast<double>();
    Ef = config_dict["Er"].cast<double>();
    Br = config_dict["Br"].cast<double>();
    Cr = config_dict["Cr"].cast<double>();
    Dr = config_dict["Dr"].cast<double>();
    Er = config_dict["Er"].cast<double>();
    floor = config_dict["floor"].cast<std::string>();
}
