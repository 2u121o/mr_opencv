#include "robot.h"


Robot::Robot(Eigen::Vector2i initial_pose, cv::Mat &map, int robot_size): pose_(initial_pose), map_(map), robot_size_(robot_size)
{
    std::cout << "[Robot] robot initialized" << std::endl;
    
}

void Robot::drawRobot(short int chanel_color){

    for(int x_idx=-robot_size_; x_idx<robot_size_+1; x_idx++){
        for(int y_idx=-robot_size_; y_idx<robot_size_+1; y_idx++){
            cv::Vec3b &color = map_.at<cv::Vec3b>(pose_[1]-y_idx, pose_[0]-x_idx);
            color[0] = chanel_color;
            color[1] = chanel_color;
            color[2] = chanel_color;
        }
    }

}

void Robot::move(int k){

    drawRobot(255);

    Eigen::Vector2i tmp_pose = pose_;
    switch (k)
        {
        case 81: //left
            if(!is_collided(tmp_pose, k))
                 pose_[0] -= 1;
            break;
        case 82: //up
            if(!is_collided(tmp_pose, k))
                pose_[1] -= 1;
            break;
        case 83: //right
            tmp_pose[0] +=1;
            if(!is_collided(tmp_pose, k))
                pose_[0] += 1;
            break;
        case 84: //down
            if(!is_collided(tmp_pose, k))
                 pose_[1] += 1;
            break;
        default:
            break;
        }
        drawRobot(0);
}

bool Robot::is_collided(const Eigen::Vector2i next_pose, const int direction){

    if(direction == 81){
        cv::Vec3b color = map_.at<cv::Vec3b>(next_pose[1], next_pose[0]-robot_size_-2);
        if(color[0] == 0)
            return true;
    }
    else if(direction == 82){
        cv::Vec3b color = map_.at<cv::Vec3b>(next_pose[1]-robot_size_-2, next_pose[0]);
        if(color[0] == 0)
            return true;
    }
    else if(direction == 83){
        cv::Vec3b color = map_.at<cv::Vec3b>(next_pose[1], next_pose[0]+robot_size_+1);
        if(color[0] == 0)
            return true;
    }
    else if(direction == 84){
         cv::Vec3b color = map_.at<cv::Vec3b>(next_pose[1]+robot_size_+2, next_pose[0]);
        if(color[0] == 0)
            return true;
    }

    return false;

}

    

void Robot::setPose(Eigen::Vector2i pose){
    pose_ = pose;
}
 
Eigen::Vector2i Robot::getPose(){
    return pose_;
}

Robot::~Robot()
{
}