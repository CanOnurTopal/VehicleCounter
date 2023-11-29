#include "vehiclelogger.h"
#include <limits>
#include <algorithm>
#include <iterator>
#include "tbb/tbb.h"

VehicleLogger::VehicleLogger(): count_(0), frame_count_(0) {
}

void VehicleLogger::update(cv::Mat& frame) {
    constexpr long long unsigned RUN_TRACKER_EVERY = 2;
    constexpr long long unsigned RUN_DETECT_EVERY = 2;

    if(frame_count_ == 0 || frame_count_ % RUN_DETECT_EVERY == 0) {
        this->run_detect_merge(frame);
    }

    if (frame_count_ != 0 && frame_count_ % RUN_TRACKER_EVERY == 0) {
        this->run_tracking(frame);
    }

    frame_count_++;
}


void VehicleLogger::add_vehicles(cv::Mat& frame, std::list<cv::Rect>& list) {

    for (auto iter = list.begin(); iter != list.end(); ++iter) {
        std::atomic<bool> is_unique(true);
        tbb::parallel_for(size_t(0), active_vehicles_.size(),
            [&is_unique, &frame, &iter, this](size_t i) {
                auto av_iter = this->active_vehicles_.begin();
                std::advance(av_iter, i);
                if (!is_unique.load()) return;
                if(av_iter->update_if_same(frame, *iter, frame_count_, true)) {
                    is_unique.store(false);
                }
            });
        if(is_unique.load()) {
            active_vehicles_.emplace_back(frame, *iter);
            count_++;
        }
    }
}

std::list<cv::Rect> VehicleLogger::detect(cv::Mat& frame) {
    constexpr float confThreshold = 0.3f;
    constexpr float iouThreshold = 0.4f;
    return this->detector_.detect(frame, confThreshold, iouThreshold);
}

void VehicleLogger::increase_frame_count(unsigned long long add) {
    constexpr unsigned long long MAX_VAL = std::numeric_limits<unsigned long long>::max();
    if (MAX_VAL - add < frame_count_) {
        frame_count_ = add - (MAX_VAL - frame_count_);
    } else frame_count_ += add;
}
bool VehicleLogger::detection_vehicle_update(cv::Mat& frame, cv::Rect& detection) {
    for (Vehicle& vehicle: active_vehicles_){
        if (vehicle.update_if_same(frame, detection, frame_count_)) return true;
    }
    return false;
}

unsigned long long VehicleLogger::get_vehicle_count() {
    return count_;
}
void VehicleLogger::run_tracking(cv::Mat& frame) {
    std::vector<std::list<Vehicle>::iterator> iters;
    for (auto it = active_vehicles_.begin(); it != active_vehicles_.end(); ++it) {
        iters.push_back(it);
    }
    std::vector<std::list<Vehicle>::iterator> track_failed;
    tbb::parallel_for(size_t(0), iters.size(),
        [&iters, &frame, &track_failed](size_t i) {
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

void VehicleLogger::run_detect_merge(cv::Mat& frame) {
    std::list<cv::Rect> vehicles = this->detect(frame);
    std::vector<std::list<cv::Rect>::iterator> iters;
    for (auto it = vehicles.begin(); it != vehicles.end(); ++it) {
        iters.push_back(it);
    }
    tbb::parallel_for(size_t(0), iters.size(),
        [&iters, &frame, &vehicles, this](size_t i) {
            auto& iter = iters[i];
            if(this->detection_vehicle_update(frame, *iter)) {
                vehicles.erase(iter);
            }
        });
    add_vehicles(frame, vehicles);
}

