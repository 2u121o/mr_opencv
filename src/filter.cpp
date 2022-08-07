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
        
        particles_.at(i) = particle_tmp;

    }
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
