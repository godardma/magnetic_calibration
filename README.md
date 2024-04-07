# Magnetic calibration

Extracted from https://gitlab.ensta-bretagne.fr/lemezoth/voiture2A-ros

## Autors :

:teacher: Thomas LE MEZO <thomas.lemezo@ensta-bretagne.org> (Lab-STICC)

:teacher: Maël GODARD <mael.godard@ensta-bretagne.org> (Lab-STICC)

## Git Structure :

* :file_folder: [/include](include) : **folder containing headers files**
* :file_folder: [/src](src) : **folder containing headers files**
* :spiral_notepad: [CMakeLists.txt](CMakeLists.txt)    **CMakeLists to compile**
* :spiral_notepad: [package.xml](package.xml)
* :spiral_notepad: [README.md](README.md)

## Technologies :

* C++
* Eigen
* Vtk 9.0
* CMake


## Building the package for the first time

* Clone the repository :
```bash
# for ssh clone
git clone git@github.com:godardma/magnetic_calibration.git
# for https clone
git clone https://github.com/godardma/magnetic_calibration.git
```
* Open a terminal in the created folder
* Setup the build folder :
```bash
mkdir build && cd build && cmake ..
```


## Launching :
From a terminal in the build folder :
```bash
make && ./mag_calibration
```

## Using the package
An exemple main file can be found [here](src/main.cpp)

The main steps are :
* Create an instance of the object MagneticCalibration
* Fill the field "magnetometer_data_regularized_" :
    * Fill the field "magnetometer_data_" and regularize the dataset with the function "regularize_data"
    * Use the function "generate_data" to use the the default dataset
* Perform the ellipsoid fit with the "compute_ellipsoid" function to get its center and the transformation matrix (noted TR) between the ellipsoid and a sphere

Other useful functions provided are :
* correct_data: Applies the correction to "magnetometer_data_" and stores the result in "magnetometer_data_corrected_"
* check_correction: Performs an ellipsoid fit on "magnetometer_data_corrected_" to check if it matches a sphere as it should
* view_data: to visualize the data, arguments are point size and the range of the displayed axis
    * In **black** the raw data, stored magnetometer_data_
    * In **red** the regularized data, stored magnetometer_data_regularized
    * In **green** the corrected data, stored magnetometer_data_corrected, and the corresponding sphere



