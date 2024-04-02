#include "Detection2D.hpp"
#include <cfloat>

float Detection2D::min_dist_of_closeness_criteria = 0.01F;
float Detection2D::d_theta_deg_for_search = 1.0F;

Detection2D::Detection2D(Eigen::MatrixXf cluster_matrix) {
    rectangle_search(cluster_matrix);
}

Detection2D::~Detection2D() { }

float Detection2D::calc_closeness_criterion(Eigen::MatrixXf& c) {
    Eigen::VectorXf c1 = c.row(0);
    Eigen::VectorXf c2 = c.row(1);
    int num_cols = c.cols();

    float c1_min = c1.minCoeff();
    float c1_max = c1.maxCoeff();
    float c2_min = c2.minCoeff();
    float c2_max = c2.maxCoeff();

    Eigen::VectorXf c1_max_vec = Eigen::VectorXf::Constant(num_cols, c1_max);
    Eigen::VectorXf c1_min_vec = Eigen::VectorXf::Constant(num_cols, c1_min);
    Eigen::VectorXf d1 = (c1_max_vec - c1).cwiseMin(c1 - c1_min_vec);

    Eigen::VectorXf c2_max_vec = Eigen::VectorXf::Constant(num_cols, c2_max);
    Eigen::VectorXf c2_min_vec = Eigen::VectorXf::Constant(num_cols, c2_min);
    Eigen::VectorXf d2 = (c2_max_vec - c2).cwiseMin(c2 - c2_min_vec);

    Eigen::VectorXf d = d1.cwiseMin(d2);
    d = d.cwiseMax(min_dist_of_closeness_criteria);
    d = d.cwiseInverse();

    float beta = d.sum();
    return beta;
}

void Detection2D::rectangle_search(Eigen::MatrixXf& cluster_matrix) {
    float d_theta = d_theta_deg_for_search * (M_PI / 180.0F);
    float min_cost[2] = {-FLT_MAX, 0.0F}; //{cost, theta}

    for(float theta = 0.0F; theta < M_PI / 2.0F; theta += d_theta) {
        Eigen::Matrix2f e;
        e << cosf(theta), sinf(theta), -sinf(theta), cosf(theta);
        Eigen::MatrixXf c  = e * cluster_matrix; //c1 and c2 {c1,c2}
        //c1 is the values of r that passes through each point @ current theta
        //c2 is also the values of r that pass through each point, but is a line perpendicular to c1

        float cost = calc_closeness_criterion(c);

        if (min_cost[0] < cost) {
            min_cost[0] = cost;
            min_cost[1] = theta;
        }
    }

    //after we find the best value for theta, do the transformation on each point
    //c1_s is the value of r that will touch each point @ curr theta
    //c2_s is the same but for perpendicular line
    float cos_s = cosf(min_cost[1]);
    float sin_s = sinf(min_cost[1]);
    Eigen::Matrix2f min_trans;
    min_trans << cos_s, sin_s, -sin_s, cos_s;

    Eigen::MatrixXf c_s = min_trans * cluster_matrix;
    
    float c1_s_min = c_s.row(0).minCoeff();
    float c2_s_min = c_s.row(1).minCoeff();
    float c1_s_max = c_s.row(0).maxCoeff();
    float c2_s_max = c_s.row(1).maxCoeff();

    //get center point
    Eigen::Vector2f center_point; //(column vector)
    center_point << (c1_s_max + c1_s_min)/2.0F, (c2_s_max + c2_s_min)/2.0F;
    center_point = min_trans.inverse() * center_point; // transform center point to vehicle coordinate
    
    width_ = c2_s_max - c2_s_min;
    length_ = c1_s_max - c1_s_min;
    position_ << center_point(0), center_point(1);
    rotation_ << cosf(-min_cost[1]), sinf(-min_cost[1]), 0,
                -sinf(-min_cost[1]), cosf(-min_cost[1]), 0,
                0, 0, 1;

    float a1 = cos_s;
    float b1 = sin_s;
    float c1 = c1_s_min;
    float a2 = -sin_s;
    float b2 = cos_s;
    float c2 = c2_s_min;
    float a3 = cos_s;
    float b3 = sin_s;
    float c3 = c1_s_max;
    float a4 = -sin_s;
    float b4 = cos_s;
    float c4 = c2_s_max;

    std::vector<std::pair<float,float>> corners;
    corners.push_back(calc_cross_point(a2, b2, c2, a3, b3, c3));
    corners.push_back(calc_cross_point(a1, b1, c1, a2, b2, c2));
    corners.push_back(calc_cross_point(a1, b1, c1, a4, b4, c4));
    corners.push_back(calc_cross_point(a4, b4, c4, a3, b3, c3));
    corner_list_ = corners;
}

std::pair<float,float> Detection2D::calc_cross_point(const float a0, const float b0, const float c0, const float a1, const float b1, const float c1) {
    float x = (b0 * (-c1) - b1 * (-c0)) / (a0 * b1 - a1 * b0);
    float y = (a1 * (-c0) - a0 * (-c1)) / (a0 * b1 - a1 * b0);
    return std::pair<float,float> (x, y);
}

float Detection2D::getWidth() const {
    return width_;
}
float Detection2D::getLength() const {
    return length_;
}
Eigen::Vector2f Detection2D::getPosition() const {
    return position_;
}
Eigen::Matrix3f Detection2D::getRotation() const {
    return rotation_;
}
std::vector<std::pair<float,float>> Detection2D::getCorners() const {
    return corner_list_;
}