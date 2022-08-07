#ifndef FILTER_H
#define FILTER_H

#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"

class Filter{

public:

    Filter(cv::Mat &map);

    void drawParticles(cv::Mat &map);

    void initializeParticles(int num_particles);

    void prediction();
    void update();

private:

    std::vector<cv::Point> particles_;
    int num_particles_;

    cv::Mat map_;

};

#endif