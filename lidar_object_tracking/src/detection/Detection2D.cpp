#include "Detection2D.hpp"
#include <cfloat>

float Detection2D::min_dist_of_closeness_criteria = 0.01F;
float Detection2D::d_theta_deg_for_search = 1.0F;

Detection2D::Detection2D(Eigen::MatrixXf cluster_matrix) {
    rectangle_search(cluster_matrix);
    calc_rect_contour();
}

Detection2D::~Detection2D() { }

float Detection2D::calc_closeness_criterion(Eigen::MatrixXf& c) {
    Eigen::VectorXf c1 = c.row(0);
    Eigen::VectorXf c2 = c.row(1);
    int num_cols = c.cols();

    // std::cout<<"c matrix: \n"<<c<<std::endl;
    // std::cout<<"c1 matrix: \n"<<c1<<std::endl;
    // std::cout<<"c2 matrix: \n"<<c2<<std::endl;

    float c1_min = c1.minCoeff();
    float c1_max = c1.maxCoeff();
    float c2_min = c2.minCoeff();
    float c2_max = c2.maxCoeff();

    // std::cout<<"c1_min: \n"<<c1_min<<std::endl;
    // std::cout<<"c1_max: \n"<<c1_max<<std::endl;
    // std::cout<<"c2_min: \n"<<c2_min<<std::endl;
    // std::cout<<"c2_max: \n"<<c2_max<<std::endl;

    Eigen::VectorXf c1_max_vec = Eigen::VectorXf::Constant(num_cols, c1_max);
    Eigen::VectorXf c1_min_vec = Eigen::VectorXf::Constant(num_cols, c1_min);
    Eigen::VectorXf d1 = (c1_max_vec - c1).cwiseMin(c1 - c1_min_vec);

    // std::cout<<"d1 matrix: \n"<<d1<<std::endl;

    Eigen::VectorXf c2_max_vec = Eigen::VectorXf::Constant(num_cols, c2_max);
    Eigen::VectorXf c2_min_vec = Eigen::VectorXf::Constant(num_cols, c2_min);
    Eigen::VectorXf d2 = (c2_max_vec - c2).cwiseMin(c2 - c2_min_vec);

    // std::cout<<"d2 matrix: \n"<<d2<<std::endl;

    Eigen::VectorXf d = d1.cwiseMin(d2);
    d = d.cwiseMax(min_dist_of_closeness_criteria);
    d = d.cwiseInverse();

    // std::cout<<"d matrix: \n"<<d<<std::endl;

    float beta = d.sum();
    return beta;
}

void Detection2D::rectangle_search(Eigen::MatrixXf& cluster_matrix) {
    float d_theta = d_theta_deg_for_search * (M_PI / 180.0F);
    float min_cost[2] = {-FLT_MAX, 0.0F}; //{cost, theta}

    // std::cout<<"Cluster matrix: "<<cluster_matrix<<std::endl;
    // std::cout<<"Num rows: "<<cluster_matrix.rows()<<std::endl;
    // std::cout<<"Num cols: "<<cluster_matrix.cols()<<std::endl;

    for(float theta = 0.0F; theta < M_PI / 2.0F; theta += d_theta) {
        Eigen::Matrix2f e;
        float sin_theta = sinf(theta);
        float cos_theta = cosf(theta);
        e << cos_theta, sin_theta, -sin_theta, cos_theta;
        Eigen::MatrixXf c  = e * cluster_matrix; //c1 and c2

        // std::cout<<"C1C2 Matrix: "<<c<<std::endl;
        // std::cout<<"Num C1C2 rows: "<<c.rows()<<std::endl;
        // std::cout<<"Num C1C2 cols: "<<c.cols()<<std::endl;

        float cost = calc_closeness_criterion(c);
        // break;
        // std::cout<<"Current theta value: "<<theta<<"\n";
        // std::cout<<"Current cost value: "<<cost<<"\n";
        //current issue: cost always returns infinity so it sets the theta value to 0 everytime

        if (min_cost[0] < cost) {
            min_cost[0] = cost;
            min_cost[1] = theta;
            // std::cout<<"Theta value at newest cost: "<<theta<<"\n";;
        }
    }

    //find best rectangle
    Eigen::Matrix2f min_trans;
    float sin_s = sinf(min_cost[1]);
    float cos_s = cosf(min_cost[1]);
    min_trans << cos_s, sin_s, -sin_s, cos_s;
    Eigen::MatrixXf c_s = min_trans * cluster_matrix;
    
    float c1_s_min = c_s.row(0).minCoeff();
    float c1_s_max = c_s.row(0).maxCoeff();
    float c2_s_min = c_s.row(1).minCoeff();
    float c2_s_max = c_s.row(1).maxCoeff();

    a_[0] = cos_s;
    b_[0] = sin_s;
    c_[0] = c1_s_min;

    a_[1] = -sin_s;
    b_[1] = cos_s;
    c_[1] = c2_s_min;

    a_[2] = cos_s;
    b_[2] = sin_s;
    c_[2] = c1_s_max;

    a_[3] = -sin_s;
    b_[3] = cos_s;
    c_[3] = c2_s_max;
}

void Detection2D::calc_cross_point(const float a0, const float a1, const float b0, const float b1, const float c0, const float c1, float& x, float& y) {
    x = (b0 * (-c1) - b1 * (-c0)) / (a0 * b1 - a1 * b0);
    y = (a1 * (-c0) - a0 * (-c1)) / (a0 * b1 - a1 * b0);
}

void Detection2D::calc_rect_contour() {
    // std::cout<<"values for a:"<<a_[0]<<" "<<a_[1]<<" "<<a_[2]<<" "<<a_[3]<<"\n";
    // std::cout<<"values for b:"<<b_[0]<<" "<<b_[1]<<" "<<b_[2]<<" "<<b_[3]<<"\n";
    // std::cout<<"values for c:"<<c_[0]<<" "<<c_[1]<<" "<<c_[2]<<" "<<c_[3]<<"\n";

    float top_left_x = 0.0, top_left_y = 0.0;
    calc_cross_point(a_[0], a_[1], b_[0], b_[1], c_[0], c_[1], top_left_x, top_left_y);
    rect_c_x_[0] = top_left_x;
    rect_c_y_[0] = top_left_y;

    float top_right_x = 0.0, top_right_y = 0.0;
    calc_cross_point(a_[1], a_[2], b_[1], b_[2], c_[1], c_[2], top_right_x, top_right_y);
    rect_c_x_[1] = top_right_x;
    rect_c_y_[1] = top_right_y;

    float bottom_left_x = 0.0, bottom_left_y = 0.0;
    calc_cross_point(a_[2], a_[3], b_[2], b_[3], c_[2], c_[3], bottom_left_x, bottom_left_y);
    rect_c_x_[2] = bottom_left_x;
    rect_c_y_[2] = bottom_left_y;

    float bottom_right_x = 0.0, bottom_right_y = 0.0;
    calc_cross_point(a_[3], a_[0], b_[3], b_[0], c_[3], c_[0], bottom_right_x, bottom_right_y);
    rect_c_x_[3] = bottom_right_x;
    rect_c_y_[3] = bottom_right_y;
}

std::array<float, 4> Detection2D::getRectPointsX() const {
    return rect_c_x_;
}

std::array<float, 4> Detection2D::getRectPointsY() const {
    return rect_c_y_;
}