#include "vehicle.h"
#include <opencv2/tracking.hpp>
#include <iostream>

Vehicle::Vehicle(const cv::Mat& frame, const cv::Rect& loc): curr_loc_(loc), last_update_(0) {
    this->init_tracker(frame, loc);
}

bool Vehicle::track(const cv::Mat& frame) {
    /*
    constexpr float TRACKING_SCORE_THRESHOLD = 0.6f;
    try { //try block to Circumvent OPENCV BUG with dnn trackers.
        tracker_ptr_->update(frame, curr_loc_);
        return tracker_ptr_->getTrackingScore() > TRACKING_SCORE_THRESHOLD;
    }
    catch (cv::Exception e){
        return false;
    }
     */
    return tracker_ptr_->update(frame, curr_loc_);


}

bool Vehicle::update_if_same(const cv::Mat& frame, cv::Rect& new_loc, long long unsigned frame_count, bool reject_if_smaller) {
    constexpr float EQUALITY_THRESHOLD = 0.6f;
    if (rect_overlap_(curr_loc_, new_loc) > EQUALITY_THRESHOLD) {
        if (!(new_loc.x && 0 <= new_loc.width && new_loc.x + new_loc.width <= frame.cols && 0 <= new_loc.y && 0 <= new_loc.height && new_loc.y + new_loc.height <= frame.rows)) return true; //PREVENT OPENCV BUG
        if (!(reject_if_smaller && curr_loc_.area() > new_loc.area())) {
            curr_loc_ = new_loc;
            this->init_tracker(frame, new_loc);
            last_update_ = frame_count;
        }
        return true;
    }else return false;
}

float Vehicle::rect_overlap_(cv::Rect& rectA, cv::Rect& rectB) {
    float rect_intersection = (rectA & rectB).area();
    float smaller_area = std::max(rectA.area(), rectB.area());
    return rect_intersection / smaller_area;
}
cv::Rect& Vehicle::get_loc() {
    return curr_loc_;
}

void Vehicle::init_tracker(const cv::Mat& frame, const cv::Rect& loc) {
    CVTracker::Params params;
    tracker_ptr_ = CVTracker::create(params);
    tracker_ptr_->init(frame, loc);
}

unsigned long long Vehicle::last_updated_frame() {
    return last_update_;
}

