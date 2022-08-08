#include "filter.h"

Filter::Filter(cv::Mat &map): map_(map){

    std::cout << "[Filter] Filter initialized" << std::endl;

}

void Filter::initializeParticles(int num_particles){

    num_particles_ = num_particles;
    particles_.resize(num_particles_);

    int x_max = map_.cols;
    int y_max = map_.rows;
    
    for(int i=0; i<num_particles_; i++){
        int x = std::rand()%x_max;
        int y = std::rand()%y_max;
         
        cv::Point particle_tmp(x,y);

        //in case the particle is on some edge i do not consider
        //it and i recompute a new particle obviously im considering
        //just the center of the particle, it may be that i small part
        //is on soe edge but doesnt matter it is much more efficient
        //rater then check the entire particle 
        cv::Vec3b color = map_.at<cv::Vec3b>(y, x);
        if(color[0]+color[1]+color[2] == 0){
            i--;
        }
        else{
            particles_.at(i) = particle_tmp;
        }
    }
}

void Filter::prediction(short int direction){

     for(int i=0; i<num_particles_; i++){

        cv::Point tmp_particle = particles_.at(i);
        switch (direction)
        {
        case 81:
            tmp_particle.x -= 1;
            if(!is_collided(tmp_particle, direction)){
                particles_.at(i) = tmp_particle;
            }
            break;
        case 82:
            tmp_particle.y -= 1;
            if(!is_collided(tmp_particle, direction)){
                particles_.at(i) = tmp_particle;
            }
            break;
        case 83:
            tmp_particle.x += 1;
            if(!is_collided(tmp_particle, direction)){
                particles_.at(i) = tmp_particle;
            }
            break;
        case 84:
            tmp_particle.y += 1;
            if(!is_collided(tmp_particle, direction)){
                particles_.at(i) = tmp_particle;
            }
            break;
        default:
            break;
        }
        
     }

}

bool Filter::is_collided(const cv::Point next_pose, int direction){
    int particle_dimension = 1;
    if(direction == 81){
        cv::Vec3b color = map_.at<cv::Vec3b>(next_pose.y, next_pose.x-particle_dimension);
        if(color[0]+color[1]+color[2] == 0)
            return true;
    }
    else if(direction == 82){
        cv::Vec3b color = map_.at<cv::Vec3b>(next_pose.y-particle_dimension, next_pose.x);
        if(color[0]+color[1]+color[2] == 0)
            return true;
    }
    else if(direction == 83){
        cv::Vec3b color = map_.at<cv::Vec3b>(next_pose.y, next_pose.x+particle_dimension);
        if(color[0]+color[1]+color[2] == 0)
            return true;
    }
    else if(direction == 84){
         cv::Vec3b color = map_.at<cv::Vec3b>(next_pose.y+particle_dimension, next_pose.x);
        if(color[0]+color[1]+color[2] == 0)
            return true;
    }

    return false;

}

void Filter::update(double range, double bearing, std::vector<double> weights){

    Eigen::Vector2d z;
    z << range, bearing;

    weights.resize(num_particles_);
    for(int i=0; i<num_particles_; i++){

        double range_part;
        cv::Point particle = particles_.at(i);
        cv::Point nearest_point = getRange(particle, 30, range_part);
        //int bearing_part = getBearing(nearest_point, particle);

        // Eigen::Vector2d h;
        // h << range_part, bearing_part;

        // double w = (h-z).transpose()*(h-z);
        // weights[i] = w;
    }

}

 cv::Point Filter::getRange(cv::Point particle, int radius, double &range){

    cv::Point nearest_point(-10000, -10000);
    double min_distance = 10000;

    
    //with this first if it avoid core dump in case the robot reach the upper
    //part of the window, in this way the radius is adapted
    int radius_y = particle.y<=radius ? particle.y:radius;
    
    for(int y=-radius_y; y<radius; y++){
        for(int x=-radius; x<radius; x++){
            cv::Vec3b pixel_color = map_.at<cv::Vec3b>(particle.y+y, particle.x+x);
            if(pixel_color[0]+pixel_color[1]+pixel_color[2] == 0){
                double tmp_distance = std::sqrt(std::pow(x,2)+std::pow(y,2));
                if(min_distance>tmp_distance){
                    nearest_point.x = particle.x+x;
                    nearest_point.y = particle.y+y;
                    min_distance = tmp_distance;
                }
            }
        }
    }
    range = min_distance;
    return nearest_point;
}

double Filter::getBearing(cv::Point nearest_point, cv::Point particle){

    //nearest point in the robot RF
    cv::Point nearest_point_in_robot = nearest_point - particle;
    return std::atan2(nearest_point_in_robot.y, nearest_point_in_robot.x);

}

void Filter::drawParticles(cv::Mat &map){

    for(int i=0; i<num_particles_; i++){
        cv::Point particle = particles_.at(i);
        for(int x_idx=-4; x_idx<4; x_idx++){
                for(int y_idx=-4; y_idx<4; y_idx++){ 
                    cv::Vec3b &color = map.at<cv::Vec3b>(particle.y, particle.x);
                    color[0] = 255;
                    color[1] = 0;
                    color[2] = 0;
                }
            }
    }
}
