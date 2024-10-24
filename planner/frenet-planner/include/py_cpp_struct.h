#ifndef FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
#define FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
#include <stddef.h>

const size_t MAX_PATH_LENGTH = 100;

namespace frenet_planner{

struct FrenetInitialConditions {
    double s0;          // current track position
    double c_speed;     // current speed
    double c_s_dd;      // current longitudonal acceleration
    double c_d;         // current lateral position
    double c_d_d;       // current lateral speed
    double c_d_dd;      // current lateral acceleration
    double target_speed;// the speed it should have at the end of the trajectory
};

    // double *wx;         // array of waypoint x positions
    // double *wy;         // array of waypoint y positions
    // int nw;             // number of waypoints
    // double *o_llx;      // obstacle top left x position
    // double *o_lly;      // obstacle top left y position
    // double *o_urx;      // obstacle bottom right x position
    // double *o_ury;      // obstacle bttom right y position
    // int no;             // number of obstacles

struct FrenetReturnValues {
    int success;
    size_t path_length;
    double x_path[MAX_PATH_LENGTH];
    double y_path[MAX_PATH_LENGTH];
    double speeds[MAX_PATH_LENGTH];
    double ix[MAX_PATH_LENGTH];
    double iy[MAX_PATH_LENGTH];
    double iyaw[MAX_PATH_LENGTH];
    double d[MAX_PATH_LENGTH];
    double s[MAX_PATH_LENGTH];
    double speeds_x[MAX_PATH_LENGTH];
    double speeds_y[MAX_PATH_LENGTH];
    double params[MAX_PATH_LENGTH];
    double costs[MAX_PATH_LENGTH];
};

struct FrenetHyperparameters {
    double max_speed;
    double min_speed;
    double max_accel;
    double max_curvature;
    double max_road_width_l;
    double max_road_width_r;
    double d_road_w;
    double dt;
    double maxt;
    double mint;
    double d_sample_t;
    double init_d_t_s;
    double d_t_s_ot;
    double d_t_s;
    double n_s_sample;
    double obstacle_clearance;
    double track_clearance;
    double k_lat_dev;
    double k_lat_v;
    double k_lat_a;
    double k_lat_j;
    double k_lon_dev;
    double k_track_bounds;
    double k_obstacles;
    double k_temporal;
    double k_lat;
    double k_lon;
    double k_time; 
    double global_path_length;
    int num_threads;
    bool ic_avoid_obstacles;
};
} // end of namespace frenet_planner
#endif //FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H