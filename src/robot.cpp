#include "robot.h"


Robot::Robot( cv::Mat &map, cv::Point initial_pose, int robot_size): robot_pose_(initial_pose), map_(map), robot_size_(robot_size)
{
    std::cout << "[Robot] robot initialized" << std::endl;

}

void Robot::drawRobot(cv::Mat &map, short int chanel_color){

    for(int x_idx=-robot_size_; x_idx<robot_size_+1; x_idx++){
        for(int y_idx=-robot_size_; y_idx<robot_size_+1; y_idx++){
            cv::Vec3b &color = map.at<cv::Vec3b>(robot_pose_.y-y_idx, robot_pose_.x-x_idx);
            color[0] = chanel_color;
            color[1] = chanel_color;
            color[2] = chanel_color;
        }
    }

   
}


cv::Point Robot::getSensedPoint(int radius, double &range){

    cv::Point sensed_point(-1, -1);
    double min_distance = 10000;

    
    //with this first if it avoid core dump in case the robot reach the upper
    //part of the window, in this way the radius is adapted
    int radius_y = robot_pose_.y<=radius ? robot_pose_.y:radius;
    
    for(int y=-radius_y; y<radius; y++){
        for(int x=-radius; x<radius; x++){
            cv::Vec3b pixel_color = map_.at<cv::Vec3b>(robot_pose_.y+y, robot_pose_.x+x);
            if(pixel_color[0]+pixel_color[1]+pixel_color[2] == 0){
                double tmp_distance = std::sqrt(std::pow(x,2)+std::pow(y,2));
                if(min_distance>tmp_distance){
                    sensed_point.x = robot_pose_.x+x;
                    sensed_point.y = robot_pose_.y+y;
                    min_distance = tmp_distance;
                }
            }
        }
    }
    range = min_distance;
    return sensed_point;
}

double Robot::getBearing(cv::Point nearest_point){

    //nearest point in the robot RF
    cv::Point nearest_point_in_robot = nearest_point - robot_pose_;
    return std::atan2(nearest_point_in_robot.y, nearest_point_in_robot.x);

}


void Robot::move(int k){

    //drawRobot(255);

    cv::Point tmp_pose = robot_pose_;
    switch (k)
        {
        case 81: //left
             tmp_pose.x -=1;
            if(!is_collided(tmp_pose, k))
                 robot_pose_.x -= 1;
            break;
        case 82: //up
            tmp_pose.y -=1;
            if(!is_collided(tmp_pose, k))
                robot_pose_.y -= 1;
            break;
        case 83: //right
            tmp_pose.x +=1;
            if(!is_collided(tmp_pose, k))
                robot_pose_.x += 1;
            break;
        case 84: //down
            tmp_pose.y +=1;
            if(!is_collided(tmp_pose, k))
                 robot_pose_.y += 1;
            break;
        default:
            break;
        }
        //drawRobot(0);
      
        
}

bool Robot::is_collided(const cv::Point next_pose, const int direction){

    if(direction == 81){
        cv::Vec3b color = map_.at<cv::Vec3b>(next_pose.y, next_pose.x-robot_size_-1);
        if(color[0]+color[1]+color[2] == 0)
            return true;
    }
    else if(direction == 82){
        cv::Vec3b color = map_.at<cv::Vec3b>(next_pose.y-robot_size_-1, next_pose.x);
        if(color[0]+color[1]+color[2] == 0)
            return true;
    }
    else if(direction == 83){
        cv::Vec3b color = map_.at<cv::Vec3b>(next_pose.y, next_pose.x+robot_size_+1);
        if(color[0]+color[1]+color[2] == 0)
            return true;
    }
    else if(direction == 84){
         cv::Vec3b color = map_.at<cv::Vec3b>(next_pose.y+robot_size_+1, next_pose.x);
        if(color[0]+color[1]+color[2] == 0)
            return true;
    }

    return false;

}

    

void Robot::setPose(cv::Point pose){
    robot_pose_ = pose;
}
 
cv::Point Robot::getPose(){
    return robot_pose_;
}

Robot::~Robot()
{
}