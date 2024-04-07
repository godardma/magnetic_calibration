#include "magnetic_calibration/magnetic_calibration.h"

int main(int argc, char *argv[]) {
    MagneticCalibration magnetic_calibration;


    // Generate default data
    // magnetic_calibration.generate_data();

    // Other option :
    // Fill magnetic_calibration.magnetometer_data_ with your own data
    
    // Other option :
    // Fill magnetic_calibration.magnetometer_data_ with data from a file
    magnetic_calibration.create_data_from_file("test_set.txt");

    // Regularize dataset
    magnetic_calibration.regularize_data(5,0.2);

    magnetic_calibration.compute_ellipsoid();
    magnetic_calibration.correct_data();
    magnetic_calibration.check_correction();
    magnetic_calibration.view_data(0.03,10.0);


    return 0;
}