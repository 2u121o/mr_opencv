#include <iostream>
#include <cstdlib>

#include <Eigen/Dense>

#include "opencv2/opencv.hpp"

#include "robot.h"

int main(){
    std::cout << "Particle Filter Example" << std::endl;

    cv::Mat map = cv::imread("/home/dario/Workspace/particle_filter/map1.png");
    int x_max = map.cols;
    int y_max = map.rows;

    const int thickness = 1;

    cv::Point initial_pose;
    int x = 50;
    initial_pose.x = x%x_max; //to generate a number between 0 and x_max
    int y = 51;
    initial_pose.y = y%y_max; //to generate a number between 0 and x_max

    Robot robot(initial_pose, map, 3);
    int k = 0;
    while(1){
        map = cv::imread("/home/dario/Workspace/particle_filter/map1.png");
        
        

        if(k==27 || k==-1) return 0; //pessed esc or x of the window
        if(k!=0){
            robot.move(k);
            robot.drawRobot(map, 60);
            cv::Point current_pose = robot.getPose();

            int radius = 50;
            cv::Scalar circle_color(0, 0, 255);
            cv::circle(map, current_pose,radius, circle_color, thickness);

            cv::Point meas_point =  robot.getSensedPoint(k, radius);

            if(meas_point.x != -1){
                cv::Scalar line_color(255, 0, 0);
                line(map, current_pose, meas_point, line_color, thickness, cv::LINE_8);
            }
        }

        cv::imshow("Map", map);
        k = cv::waitKey(0);
        
    }

    
       

    // }

    // for(int y=0;y<grHistogram.rows;y++)
    // {
    //     for(int x=0;x<grHistogram.cols;x++)
    //     {
    //         //cv::Vec3b color = grHistogram.at<cv::Vec3b>(cv::Point(x,y));
    //         cv::Vec3b & color = grHistogram.at<cv::Vec3b>(y,x);
    //         if(x>0 && x<550){
    //             if(y>0 && y<550){
    //                 color[0] = 50;
    //                 color[1] = 50;
    //                 color[2] = 10;
    //             }
    //         }

    //         grHistogram.at<cv::Vec3b>(cv::Point(x,y)) = color;
    //         cv::imshow("Map", grHistogram);
    //         cv::waitKey(1);
    //     }
    // }
    
	 
    return 0;
}