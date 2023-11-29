#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include "utils.h"
#include "objdetector/detector.h"
#include <memory>
#include "vehiclelogger.h"
#include <fstream>
#include <chrono>

int main(int argc, char* argv[])
{
    if (argc != 3) {
        std::cout << "Please state your arguments as: program inputvideo.avi results.txt";
        return 0;
    }
    cv::VideoCapture cap(argv[1]);
    // cap is the object of class video capture that tries to capture Bumpy.mp4
    if ( !cap.isOpened() )  // isOpened() returns true if capturing has been initialized.
    {
        std::cout << "Cannot open the video file. \n";
        return -1;
    }

    cv::Mat frame;
    // Mat object is a basic image container. frame is an object of Mat.

    if (!cap.read(frame)) // if not success, break loop
        // read() decodes and captures the next frame.
    {
        std::cout<<"\n Cannot read the video file. \n";
    }
    VehicleLogger vlogger;
    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;
    double frame_count = 0;
    double avg_time = 0;
    while(cap.read(frame))
    {
        auto t1 = high_resolution_clock::now();
        vlogger.update(frame);
        auto t2 = high_resolution_clock::now();
        duration<double, std::milli> ms_double = t2 - t1;
        double timetaken_ms = ms_double.count();
        avg_time = (avg_time * frame_count + timetaken_ms) / (frame_count + 1);
        frame_count++;
    }
    std::ofstream ResultsFile(argv[2]);
    ResultsFile << "Vehicle Count:\n";
    ResultsFile << std::to_string(vlogger.get_vehicle_count()) << "\n"; 
    ResultsFile << "Avg Calculation Speed in ms:\n";
    ResultsFile << std::to_string(avg_time);
    ResultsFile.close();

    std::cout << "Car Count: " << vlogger.get_vehicle_count();
    return 0;
}
