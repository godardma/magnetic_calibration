//
// Created by lemezoth on 16/03/24.
//

#ifndef BUILD_MAGNETIC_PAVING_H
#define BUILD_MAGNETIC_PAVING_H

#include <vector>
#include <array>

using namespace std;

class MagneticPaving {

public:
    MagneticPaving(const std::vector<std::array<double, 3>> &magnetometer_data,
                   const unsigned long &bisect_limit_nb_data,
                   const double &bisect_limit_width);

    void process_data(std::vector<std::array<double, 3>> &regularized_data);

private:
    std::vector<std::array<double, 3>> magnetometer_data_;
    std::array<std::pair<double, double>, 3> bounds_;

    double bisect_limit_width_ = 1.0;
    unsigned long bisect_limit_nb_data_ = 30;
    bool flag_get_all_points_ = false;

    vector<MagneticPaving*> children_;

    double get_maximum_width();

    double get_width(const long &axis);

    int get_axis_to_bisect();

public:
    double cout_bounds();
};


#endif //BUILD_MAGNETIC_PAVING_H
