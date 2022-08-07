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

    

    Robot(cv::Point initial_pose, cv::Mat &map, int robot_size);
    ~Robot();


    void setPose(cv::Point pose);
    cv::Point getPose();

    void move(int k);
    void drawRobot(cv::Mat &map, short int chanel_color);
    cv::Point getSensedPoint(int direction, int radius);

private:

    int robot_size_;

    cv::Point robot_pose_;
    cv::Mat map_;
    

    bool is_collided(const cv::Point next_pose, const int direction);
};


#endif