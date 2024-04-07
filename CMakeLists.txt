cmake_minimum_required(VERSION 3.8)
project(magnetic_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(CMAKE_CXX_STANDARD 20)

### find dependencies
find_package(ament_cmake REQUIRED)

## Magnetic calibration

find_package(VTK 9.0 COMPONENTS
        CommonCore
        CommonDataModel
        FiltersSources
        InteractionStyle
        RenderingCore
        RenderingOpenGL2

        ChartsCore
        CommonColor
        RenderingContext2D
        RenderingContextOpenGL2
        RenderingFreeType
        RenderingGL2PSOpenGL2
        ViewsContext2D

        CommonTransforms
        RenderingAnnotation
        CommonComputationalGeometry
        REQUIRED)

add_executable(mag_calibration
        src/magnetic_calibration.cpp
        src/magnetic_paving.cpp
)

target_link_libraries(mag_calibration
        "${cpp_typesupport_target}"
        ${VTK_LIBRARIES}
)
target_include_directories(mag_calibration PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${VTK_INCLUDE_DIRS}
)

vtk_module_autoinit(
        TARGETS mag_calibration
        MODULES ${VTK_LIBRARIES}
)

## --- Install ---
install(TARGETS mag_calibration
        DESTINATION lib/${PROJECT_NAME}
)


ament_package()
