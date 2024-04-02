#ifndef DETECTION_2D_H
#define DETECTION_2D_H

#include <Eigen/Dense>
#include <vector>

class Detection2D
{
public:
    Detection2D(Eigen::MatrixXf cluster_matrix);
    ~Detection2D();
    
    float getWidth() const;
    float getLength() const;
    Eigen::Vector2f getPosition() const;
    Eigen::Matrix3f getRotation() const;
    std::vector<std::pair<float, float>> getCorners() const;

private:
    static float min_dist_of_closeness_criteria;
    static float d_theta_deg_for_search;

    float width_;
    float length_;
    Eigen::Vector2f position_;
    Eigen::Matrix3f rotation_;
    std::vector<std::pair<float, float>> corner_list_;

    std::pair<float,float> calc_cross_point(const float a0, const float b0, const float c0, const float a1, const float b1, const float c1);

    static float calc_closeness_criterion(Eigen::MatrixXf& c);

    void rectangle_search(Eigen::MatrixXf& cluster_matrix);
};


#endif //DETECTION_2D_H