#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <string>
#include <cmath>

#include <Eigen/Dense>

#include "opencv2/opencv.hpp"


class Robot
{
public:

    

    Robot( cv::Mat &map, cv::Point initial_pose, int robot_size);
    ~Robot();


    void setPose(cv::Point pose);
    cv::Point getPose();

    void move(int k);
    void drawRobot(cv::Mat &map, short int chanel_color);
    cv::Point getSensedPoint(int radius, double &range);
    double getBearing(cv::Point nearest_point);

private:

    int robot_size_;

    cv::Point robot_pose_;
    cv::Mat map_;
    

    bool is_collided(const cv::Point next_pose, const int direction);
};


#endif