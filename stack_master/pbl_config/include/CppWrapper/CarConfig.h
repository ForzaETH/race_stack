#ifndef CAR_CONFIG_LOADER_H
#define CAR_CONFIG_LOADER_H
#include <Config.h>


class CarConfig : Config
{
private:
public:
    CarConfig(std::string racecar_version) {
        load_config(racecar_version);
    };
    double m;
    double Iz;
    double lf;
    double lr;
    double h_cg;
    double a_max;
    double a_min;
    double C_0d;
    double C_d;
    double C_acc;
    double C_dec;
    double C_r;
    double C_0v;
    double C_v;
    double tau_steer;
    double max_steering_angle;
    std::string racecar_version;
    void load_config(std::string racecar_version) override;
};


#endif  // CAR_CONFIG_LOADER_H