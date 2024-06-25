#pragma once

#include <Eigen/Dense>
#include <vector>

class Detection2D
{
public:
    Detection2D(Eigen::MatrixXf cluster_matrix);
    ~Detection2D();

    float width_;
    float length_;
    std::vector<float> position_;
    float theta_;
    std::vector<std::pair<float, float>> corner_list_;
    
private:
    static float min_dist_of_closeness_criteria;
    static float d_theta_deg_for_search;

    std::pair<float,float> calc_cross_point(const float a0, const float b0, const float c0, const float a1, const float b1, const float c1);

    static float calc_closeness_criterion(Eigen::MatrixXf& c);

    void rectangle_search(Eigen::MatrixXf& cluster_matrix);
};
