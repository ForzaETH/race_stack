#ifndef FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_H
#define FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_H

#include <Eigen/LU>
#include <vector>

// 1-dimensional cubic spline class.
// For technical details see: http://mathworld.wolfram.com/CubicSpline.html
namespace frenet_planner{
class CubicSpline1D {
public:
    int nx;
    CubicSpline1D();
    CubicSpline1D(const std::vector<double>& v1, const std::vector<double>& v2,
            bool closed_contour);
    double calc_der0(double t);
    double calc_der1(double t);
    double calc_der2(double t);
private:
    std::vector<double> a, b, c, d, w, x, y;
    int search_index(double t);
    double handle_wrapping(double t);
    void matrix_a(std::vector<double>& deltas, Eigen::MatrixXd& result);
    void vector_b(std::vector<double>& deltas, Eigen::VectorXd& result);
    bool is_closed_contour_;
};
} // end of namespace
#endif //FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_H
