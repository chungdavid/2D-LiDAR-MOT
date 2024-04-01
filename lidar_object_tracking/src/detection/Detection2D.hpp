#ifndef DETECTION_2D_H
#define DETECTION_2D_H

#include <array>
#include <Eigen/Dense>

class Detection2D
{
public:
    Detection2D(Eigen::MatrixXf cluster_matrix);
    ~Detection2D();

    std::array<float, 4> getRectPointsX() const;
    std::array<float, 4> getRectPointsY() const;
    
    float getWidth() const;
    float getLength() const;
    Eigen::Vector2f getPosition() const;

private:
    static float min_dist_of_closeness_criteria;
    static float d_theta_deg_for_search;

    std::array<float, 3> a_;
    std::array<float, 3> b_;
    std::array<float, 3> c_;

    std::array<float, 4> rect_c_x_;
    std::array<float, 4> rect_c_y_;

    float width_;
    float length_;
    Eigen::Vector2f position_;

    void calc_rect_contour();

    static void calc_cross_point(const float a0, const float a1, const float b0, const float b1, const float c0,
                                const float c1, float& x, float& y);

    static float calc_closeness_criterion(Eigen::MatrixXf& c);

    void rectangle_search(Eigen::MatrixXf& cluster_matrix);
};


#endif //DETECTION_2D_H