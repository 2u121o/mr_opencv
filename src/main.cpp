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

    Eigen::Vector2i initial_pose;
    int x = rand();
    initial_pose[0] = x%x_max; //to generate a number between 0 and x_max
    int y = rand();
    initial_pose[1] = y%y_max; //to generate a number between 0 and x_max

    Robot robot(initial_pose, map, 3);

    while(1){

        cv::imshow("Map", map);
        int k = cv::waitKey(0);

        if(k==27 || k==-1) return 0; //pessed esc or x of the window
        robot.move(k);

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