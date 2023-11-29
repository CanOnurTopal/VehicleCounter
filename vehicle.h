#ifndef VEHICLE_H
#define VEHICLE_H

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracking.hpp>

namespace cv::tracking {
}

class Vehicle {
    public:
    Vehicle(const cv::Mat& frame, const cv::Rect& loc);
    bool track(const cv::Mat& frame);
    bool update_if_same(const cv::Mat& frame, cv::Rect& new_loc, long long unsigned frame_count, bool reject_if_smaller = false);
    cv::Rect& get_loc();
    long long unsigned last_updated_frame();

    private:
    using CVTracker = cv::tracking::TrackerKCF;
    float rect_overlap_(cv::Rect& rectA, cv::Rect& rectB);
    void init_tracker(const cv::Mat& frame, const cv::Rect& loc);
    cv::Ptr<CVTracker> tracker_ptr_;
    cv::Rect curr_loc_;
    long long unsigned last_update_;


};



#endif //VEHICLE_H
