//
// Created by lemezoth on 29/02/24.
// Updated by godardma on 04/04/24.
//

#ifndef BUILD_MAGNETIC_CALIBRATION_NODE_H
#define BUILD_MAGNETIC_CALIBRATION_NODE_H

#include <memory>

#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

#include <vtkSmartPointer.h>
#include <vtkActor.h>

class MagneticCalibration : public std::enable_shared_from_this<MagneticCalibration>{
public:


    double bisect_limit_width_ = 1.0;
    int bisect_limit_nb_data_ = 30;

    std::vector<std::array<double, 3>> magnetometer_data_;
    std::vector<std::array<double, 3>> magnetometer_data_regularized_;
    std::vector<std::array<double, 3>> magnetometer_data_corrected_;

    Vector3d center_;
    Vector3d evals_;
    Vector3d radii_;
    Matrix3d evecs_;
    Matrix4d R_;
    Matrix3d TR_;
    double r_;
    double r_corrected_;
    Vector3d ea_;


    void generate_data();

    void create_data_from_file(string file_name);

    void regularize_data(int bisect_limit_nb_data=30,double bisect_limit_width=1.0);

    void compute_ellipsoid();

    void correct_data();

    void check_correction();

    void view_data(float point_size=1.0,float axis_length=100.0);

    vtkSmartPointer<vtkActor> generate_point_cloud(std::vector<std::array<double, 3>> &pts_data, const string &color, const double &radius = 0.1);

    vtkSmartPointer<vtkActor> generate_ellipsoid(const string &color);
    vtkSmartPointer<vtkActor> generate_unit_sphere(const string &color);
};


#endif //BUILD_MAGNETIC_CALIBRATION_NODE_H
