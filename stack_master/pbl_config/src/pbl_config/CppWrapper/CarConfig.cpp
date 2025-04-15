#include <CarConfig.h>
#include <pybind11/embed.h>
namespace py = pybind11;

void CarConfig::load_config(std::string racecar_version) {
    py::scoped_interpreter guard{}; // Start the Python interpreter

    py::module car_config_module = py::module::import("pbl_config");  // Import your Python module
    py::object load_car_config_ros = car_config_module.attr("load_car_config_ros");

    py::dict config_dict = load_car_config_ros(racecar_version);  // Call Python function and get the result
    m = config_dict["m"].cast<double>();
    Iz = config_dict["Iz"].cast<double>();
    lf = config_dict["lf"].cast<double>();
    lr = config_dict["lr"].cast<double>();
    h_cg = config_dict["h_cg"].cast<double>();
    a_max = config_dict["a_max"].cast<double>();
    a_min = config_dict["a_min"].cast<double>();
    C_0d = config_dict["C_0d"].cast<double>();
    C_d = config_dict["C_d"].cast<double>();
    C_acc = config_dict["C_acc"].cast<double>();
    C_dec = config_dict["C_dec"].cast<double>();
    C_r = config_dict["C_R"].cast<double>();
    C_0v = config_dict["C_0v"].cast<double>();
    C_v = config_dict["C_v"].cast<double>();
    tau_steer = config_dict["tau_steer"].cast<double>();
    max_steering_angle = config_dict["max_steering_angle"].cast<double>();
    racecar_version = config_dict["racecar_version"].cast<std::string>();
}
