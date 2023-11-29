#ifndef VEHICLELOGGER_H
#define VEHICLELOGGER_H

#include <atomic>
#include <list>
#include <opencv2/opencv.hpp>
#include "objdetector/detector.h"
#include "vehicle.h"



class VehicleLogger {
    public:
    VehicleLogger();
    void update(const cv::Mat& frame);
    unsigned long long get_vehicle_count();

    std::list<Vehicle> active_vehicles_;

    private:
    std::list<cv::Rect> detect(const cv::Mat& frame);
    //increase frame_count_ without overflow
    void increase_frame_count(unsigned long long add = 1);
    void add_vehicles(const cv::Mat& frame, std::list<cv::Rect>& list);
    bool detection_vehicle_update(const cv::Mat& frame, cv::Rect& detection);
    void run_tracking(const cv::Mat& frame);
    void run_detect_merge(const cv::Mat& frame);
    void eliminate_multiples();

    unsigned long long count_;
    unsigned long long frame_count_;

    YOLODetector detector_;


};



#endif //VEHICLELOGGER_H
