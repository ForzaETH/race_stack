#ifndef PACEJKA_CONFIG_LOADER_H
#define PACEJKA_CONFIG_LOADER_H
#include <Config.h>

class PacejkaTireConfig : Config
{
private:
public:
    PacejkaTireConfig(std::string racecar_version, std::string floor) {
        this->floor = floor;
        load_config(racecar_version);
    };
    double friction_coeff;
    double Bf;
    double Cf;
    double Df;
    double Ef;
    double Br;
    double Cr;
    double Dr;
    double Er;
    std::string floor;
    void load_config(std::string racecar_version) override;
};


#endif  // CAR_CONFIG_LOADER_H