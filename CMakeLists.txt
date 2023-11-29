cmake_minimum_required(VERSION 3.27)
project(VehicleCounter)

find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED)
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )
link_directories("lib/")
include_directories("include/")

add_executable(VehicleCounter
        main.cpp
        utils.cpp
        objdetector/detector.cpp
        main.cpp
        visualization/visualize.h
        vehiclelogger.cpp
        vehiclelogger.h
        vehicle.cpp
        vehicle.h)


set(CMAKE_CXX_STANDARD 20)

target_include_directories(VehicleCounter PRIVATE "include/")

target_compile_features(VehicleCounter PRIVATE cxx_std_20)
target_link_libraries(VehicleCounter
        "libonnxruntime.so"
        "${OpenCV_LIBS}"
        "TBB::tbb")

configure_file(models/yolov5m.onnx yolov5m.onnx COPYONLY)
configure_file(models/coco.names coco.names COPYONLY)
configure_file(models/vitTracker.onnx vitTracker.onnx COPYONLY)
