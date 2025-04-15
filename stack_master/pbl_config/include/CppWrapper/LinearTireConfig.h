#ifndef LINEAR_CONFIG_LOADER_H
#define LINEAR_CONFIG_LOADER_H
#include <Config.h>
#include <string>

class LinearTireConfig : Config
{
private:
public:
    LinearTireConfig(std::string racecar_version, std::string floor) : Config(){
        this->floor = floor;
        load_config(racecar_version);
    };
    double C_Sf;
    double C_Sr;
    std::string floor;

    void load_config(std::string racecar_version) override;
};


#endif  // CAR_CONFIG_LOADER_H