#ifndef FILTER_H
#define FILTER_H

#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include "opencv2/opencv.hpp"

class Filter{

public:

    Filter(cv::Mat &map);

    void drawParticles(cv::Mat &map);

    void initializeParticles(int num_particles);

    void prediction(short int direction);
    void update(double range, double bearing);

private:

    std::vector<cv::Point> particles_;
    std::vector<double> weights_;
    int num_particles_;

    cv::Mat map_;

    double sum_weights_; //necessary for the normalization

    bool is_collided(const cv::Point next_pose, int direction);
    cv::Point getRange(cv::Point particle, int radius, double &range);
    double getBearing(cv::Point nearest_point, cv::Point particle);

    void uniformSampling();

};

#endif