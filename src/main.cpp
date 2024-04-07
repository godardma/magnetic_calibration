#include "magnetic_calibration/magnetic_calibration.h"

int main(int argc, char *argv[]) {
    MagneticCalibration magnetic_calibration;

    // Generate default data
    magnetic_calibration.generate_data();

    // Other option :
    // fill magnetic_calibration.magnetometer_data_ with your own data
    // then regularize it :
    
    // magnetic_calibration.regularize_data();

    magnetic_calibration.compute_ellipsoid();
    magnetic_calibration.correct_data();
    magnetic_calibration.check_correction();
    magnetic_calibration.view_data(0.3,10.0);


    return 0;
}