#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include "utils.h"
#include "objdetector/detector.h"
#include <memory>
#include "vehiclelogger.h"
#include <chrono>

int main(int argc, char* argv[])
{

    std::string HIGHWAY_VIDEO_PATH = "highway.ts";
    YOLODetector detector {nullptr};
    const std::string classNamesPath = "coco.names";
    const std::vector<std::string> classNames = utils::loadNames(classNamesPath);
    const std::string modelPath = "yolov5m.onnx";
    const float confThreshold = 0.3f;
    const float iouThreshold = 0.4f;
    const bool isGPU = true;
    try
    {
        detector = YOLODetector();
        std::cout << "Model was initialized." << std::endl;

    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }

    cv::VideoCapture cap(HIGHWAY_VIDEO_PATH);
    // cap is the object of class video capture that tries to capture Bumpy.mp4
    if ( !cap.isOpened() )  // isOpened() returns true if capturing has been initialized.
    {
        std::cout << "Cannot open the video file. \n";
        return -1;
    }

    double fps = cap.get(cv::CAP_PROP_FPS); //get the frames per seconds of the video
    // The function get is used to derive a property from the element.
    // Example:
    // CV_CAP_PROP_POS_MSEC :  Current Video capture timestamp.
    // CV_CAP_PROP_POS_FRAMES : Index of the next frame.

    cv::namedWindow("A_good_name",cv::WINDOW_KEEPRATIO); //create a window called "MyVideo"
    // first argument: name of the window.
    // second argument: flag- types:
    // WINDOW_NORMAL : The user can resize the window.
    // WINDOW_AUTOSIZE : The window size is automatically adjusted to fit the displayed image() ), and you cannot change the window size manually.
    // WINDOW_OPENGL : The window will be created with OpenGL support.

    cv::Mat frame;
    // Mat object is a basic image container. frame is an object of Mat.

    if (!cap.read(frame)) // if not success, break loop
        // read() decodes and captures the next frame.
    {
        std::cout<<"\n Cannot read the video file. \n";
    }
    //tracker->init(frame,result[0]);
    VehicleLogger vlogger;
    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;
    double frame_count = 0;
    double avg_time = 0;
    while(true)
    {
        if (!cap.read(frame)) // if not success, break loop
            // read() decodes and captures the next frame.
        {
            std::cout<<"\n Cannot read the video file. \n";
            break;
        }
        auto t1 = high_resolution_clock::now();
        vlogger.update(frame);
        auto t2 = high_resolution_clock::now();
        duration<double, std::milli> ms_double = t2 - t1;
        double timetaken_ms = ms_double.count();
        avg_time = (avg_time * frame_count + timetaken_ms) / (frame_count + 1);

        auto av = vlogger.active_vehicles_;
        for (auto it = av.begin(); it != av.end(); ++it){
            cv::Point p1(it->get_loc().x, it->get_loc().y);
            cv::Point p2(it->get_loc().x + it->get_loc().width, it->get_loc().y + it->get_loc().height);
            cv::rectangle(frame, p1, p2, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
        }
        cv::putText(frame, //target image
            std::string("Avg Time: ") + std::to_string(avg_time), //text
            cv::Point(30, 100), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            3.0,
            CV_RGB(118, 185, 0), //font color
            4);
        cv::putText(frame, //target image
            std::string("Visable Vehicles: ") + std::to_string(vlogger.active_vehicles_.size()), //text
            cv::Point(30, 200), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            3.0,
            CV_RGB(118, 185, 0), //font color
            4);
        cv::putText(frame, //target image
            std::string("Vehicle Count: ") + std::to_string(vlogger.get_vehicle_count()), //text
            cv::Point(30, 300), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            3.0,
            CV_RGB(118, 185, 0), //font color
            4);
        imshow("A_good_name", frame);
        // first argument: name of the window.
        // second argument: image to be shown(Mat object).

        if(cv::waitKey(30) == 27) // Wait for 'esc' key press to exit
        {
            break;
        }
        frame_count++;
    }
    std::cout << "Car Count: " << vlogger.get_vehicle_count();
    return 0;
}