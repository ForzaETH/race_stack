#pragma once 

namespace racecar_simulator {

class Precompute {

public:

    // pi
    static constexpr double PI = 3.1415;


    static std::vector<double> get_car_distances(
            int scan_beams, 
            double wheelbase, 
            double width, 
            double scan_distance_to_base_link, 
            double angle_min, 
            double scan_ang_incr);

    static std::vector<double> get_cosines(
            int scan_beams, 
            double angle_min, 
            double scan_ang_incr);

};

}
