# Magnetic calibration

C++ package for magnetic calibration

## Autors :

:teacher: Thomas LE MEZO <thomas.lemezo@ensta-bretagne.org> (ENSTA Bretagne, Lab-STICC)

:teacher: Maël GODARD <mael.godard@ensta-bretagne.org> (ENSTA Bretagne, Lab-STICC)

## Git Structure :

* :file_folder: [/dataset](dataset) : **folder containing the input datasets**
* :file_folder: [/include](include) : **folder containing header files**
* :file_folder: [/src](src) : **folder containing source files**
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
    * Use the function "generate_data" to use the the default dataset, no need to regularize it
    * Fill the field "magnetometer_data_" by hand and regularize the dataset
    * Fill the field "magnetometer_data_" from a file using the function "create_data_from_file" and regularize the dataset
    
    The regularization is done by calling the function "regularize_data", arguments are the limit of point in a box and its limit size 
* Perform the ellipsoid fit with the "compute_ellipsoid" function to get its center and the transformation matrix (noted TR) between the ellipsoid and a sphere

Other useful functions provided are :
* correct_data: Applies the correction to "magnetometer_data_" and stores the result in "magnetometer_data_corrected_"
* check_correction: Performs an ellipsoid fit on "magnetometer_data_corrected_" to check if it matches a sphere as it should
* view_data: to visualize the data, arguments are point size and the range of the displayed axis
    * In **black** the raw data, stored magnetometer_data_
    * In **red** the regularized data, stored magnetometer_data_regularized
    * In **green** the corrected data, stored magnetometer_data_corrected, and the corresponding sphere
 
## Notes for users
The package seems to struggle to perform a proper calibration when big values are passed as inputs. We highly suggest to keep the values in the range [-1000,1000] by scaling your inputs if necessary to get a proper result. 

## License :
This package is under [GNU General Public License](https://www.gnu.org/licenses/gpl-3.0.html)

## Credits :
The credits belong to Thomas LE MEZO (ENSTA Bretagne, Lab-STICC) and Maël GODARD (ENSTA Bretagne Lab-STICC)


