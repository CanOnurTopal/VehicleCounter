#include "vehiclelogger.h"
#include <limits>
#include <algorithm>
#include <iterator>
#include "oneapi/tbb.h"



VehicleLogger::VehicleLogger(): count_(0), frame_count_(0) {
}


void VehicleLogger::update(const cv::Mat& frame) {
    constexpr long long unsigned RUN_TRACKER_EVERY = 3;
    constexpr long long unsigned RUN_DETECT_EVERY = 3;



    if (frame_count_ != 0 && frame_count_ % RUN_TRACKER_EVERY == 0) {
        this->run_tracking(frame);
    }
    if(frame_count_ == 0 || frame_count_ % RUN_DETECT_EVERY == 0) {
        this->run_detect_merge(frame);
    }
    /*
    auto iter = active_vehicles_.begin();
    while(iter != active_vehicles_.end()){
        auto last_frame = iter->last_updated_frame();
        if(last_frame!= 0 && frame_count_ - last_frame >= 4) {
            iter = active_vehicles_.erase(iter);
            continue;
        }
        ++iter;
    }
     */

    frame_count_++;
}


void VehicleLogger::add_vehicles(const cv::Mat& frame, std::list<cv::Rect>& list) {

    for (auto iter = list.begin(); iter != list.end(); ++iter) {
            active_vehicles_.emplace_back(frame, *iter);
            count_++;
        }
}

std::list<cv::Rect> VehicleLogger::detect(const cv::Mat& frame) {
    constexpr float confThreshold = 0.3f;
    constexpr float iouThreshold = 0.4f;
    cv::Mat tmp;
    frame.copyTo(tmp);
    return this->detector_.detect(tmp, confThreshold, iouThreshold);
}

void VehicleLogger::increase_frame_count(unsigned long long add) {
    constexpr unsigned long long MAX_VAL = std::numeric_limits<unsigned long long>::max();
    if (MAX_VAL - add < frame_count_) {
        frame_count_ = add - (MAX_VAL - frame_count_);
    } else frame_count_ += add;
}
bool VehicleLogger::detection_vehicle_update(const cv::Mat& frame, cv::Rect& detection) {
    for (Vehicle& vehicle: active_vehicles_){
        if (vehicle.update_if_same(frame, detection, frame_count_)) return true;
    }
    return false;
}

unsigned long long VehicleLogger::get_vehicle_count() {
    return count_;
}
void VehicleLogger::run_tracking(const cv::Mat& frame) {
    std::vector<std::list<Vehicle>::iterator> iters;
    for (auto it = active_vehicles_.begin(); it != active_vehicles_.end(); ++it) {
        iters.push_back(it);
    }
    std::vector<std::list<Vehicle>::iterator> track_failed;
    oneapi::tbb::parallel_for(size_t(0), iters.size(),
        [&iters, frame, &track_failed](size_t i) {
            if(!iters[i]->track(frame)) {
                track_failed.push_back(iters[i]);
            }
        });
    if (!track_failed.empty()) {
        run_detect_merge(frame);
    }
    for (auto iter: track_failed) {
        if (iter->last_updated_frame() != frame_count_) {
            active_vehicles_.erase(iter);
        }
    }

}

void VehicleLogger::run_detect_merge(const cv::Mat& frame) {
    std::list<cv::Rect> vehicles = this->detect(frame);
    std::vector<std::list<cv::Rect>::iterator> iters;
    for (auto it = vehicles.begin(); it != vehicles.end(); ++it) {
        iters.push_back(it);
    }
    oneapi::tbb::parallel_for(size_t(0), iters.size(),
        [&iters, frame, &vehicles, this](size_t i) {
            auto& iter = iters[i];
            if(this->detection_vehicle_update(frame, *iter)) {
                vehicles.erase(iter);
            }
        });
    add_vehicles(frame, vehicles);
}

