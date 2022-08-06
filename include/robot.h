#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "opencv2/opencv.hpp"

class Robot
{
public:

    

    Robot(Eigen::Vector2i initial_pose, cv::Mat &map, int robot_size);
    ~Robot();


    void setPose(Eigen::Vector2i pose);
    Eigen::Vector2i getPose();

    void move(int k);
    

private:

    int robot_size_;
    Eigen::Vector2i pose_;
    cv::Mat map_;

    

    void drawRobot(short int chanel_color);
    bool is_collided(const Eigen::Vector2i next_pose, const int direction);
};


#endif