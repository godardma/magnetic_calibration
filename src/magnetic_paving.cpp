//
// Created by lemezoth on 16/03/24.
//

#include "magnetic_calibration/magnetic_paving.h"
#include <algorithm>
#include <iostream>

MagneticPaving::MagneticPaving(const std::vector<std::array<double, 3>> &magnetometer_data, const unsigned long &bisect_limit_nb_data,
                               const double &bisect_limit_width) {

    magnetometer_data_ = magnetometer_data;
    bisect_limit_nb_data_ = bisect_limit_nb_data;
    bisect_limit_width_ = bisect_limit_width;

    // Compute min, max of all bounds
    // Find the minimum and maximum element
    for (size_t dim = 0; dim < 3; ++dim) {
        // Find the min-max element for the current dimension
        auto min_max = std::minmax_element(magnetometer_data_.begin(), magnetometer_data_.end(), [dim](const auto& a, const auto& b) {
            return a[dim] < b[dim];
        });

        // Store the min-max pair for the current dimension
        bounds_[dim] = std::make_pair((*min_max.first)[dim], (*min_max.second)[dim]);
    }
}

double MagneticPaving::cout_bounds() {
    for (size_t dim = 0; dim < 3; ++dim) {
        std::cout << "dim: " << dim << " min: " << bounds_[dim].first << " max: " << bounds_[dim].second << std::endl;
    }
}

double MagneticPaving::get_maximum_width() {
    // Find the maximum width
    double max_width = 0.0;
    for (size_t dim = 0; dim < 3; ++dim) {
        max_width = std::max(max_width, bounds_[dim].second - bounds_[dim].first);
    }
    return max_width;
}

double MagneticPaving::get_width(const long &axis) {
    return bounds_[axis].second - bounds_[axis].first;
}

int MagneticPaving::get_axis_to_bisect() {
    // Find the axis with the maximum width
    double max_width = 0.0;
    int axis = -1;
    for (size_t dim = 0; dim < 3; ++dim) {
        double width = get_width(dim);
        if (width > max_width) {
            max_width = width;
            axis = dim;
        }
    }
    return axis;
}

void MagneticPaving::process_data(std::vector<std::array<double, 3>> &regularized_data) {

    // Test if limits are reached
    if (magnetometer_data_.size() <= bisect_limit_nb_data_){
        if(flag_get_all_points_) {
            for (auto &i: magnetometer_data_)
                regularized_data.push_back(i);
        }
        else {
            array<double, 3> mean = {0.0, 0.0, 0.0};
            for (auto &i: magnetometer_data_)
                for (size_t dim = 0; dim < 3; ++dim)
                    mean[dim] += i[dim];
            for (size_t dim = 0; dim < 3; ++dim)
                mean[dim] /= (double) magnetometer_data_.size();

            // Add the mean to the regularized data
            regularized_data.push_back(mean);
        }

        return;
    }
    else if(get_maximum_width() <= bisect_limit_width_){

        if(!magnetometer_data_.empty()) {
            // Compute the mean of the data
            array<double, 3> mean = {0.0, 0.0, 0.0};
            for (auto &i: magnetometer_data_)
                for (size_t dim = 0; dim < 3; ++dim)
                    mean[dim] += i[dim];
            for (size_t dim = 0; dim < 3; ++dim)
                mean[dim] /= (double)magnetometer_data_.size();

            // Add the mean to the regularized data
            regularized_data.push_back(mean);
        }
        return;
    }

    // Bisect the data
    int axis = get_axis_to_bisect();
    // Separate data in two parts
    vector<array<double, 3>> data1, data2;
    double width = get_width(axis);
    for (auto &i: magnetometer_data_){
        if (i[axis] < width / 2.0 + bounds_[axis].first)
            data1.push_back(i);
        else
            data2.push_back(i);
    }

    // Create children
    auto *child1 = new MagneticPaving(data1, bisect_limit_nb_data_, bisect_limit_width_);
    auto *child2 = new MagneticPaving(data2, bisect_limit_nb_data_, bisect_limit_width_);
    children_.push_back(child1);
    children_.push_back(child2);

    // Process children
    child1->process_data(regularized_data);
    child2->process_data(regularized_data);

    // Delete children
    delete child1;
    delete child2;
}
