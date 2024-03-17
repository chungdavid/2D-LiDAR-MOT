#include "perception/LShapeFitting.hpp"
#include <cfloat>
#include <iostream>

LShapeFitting::LShapeFitting() {
    min_dist_of_closeness_criteria = 0.01F;
    d_theta_deg_for_search = 1.0F;
}

LShapeFitting::~LShapeFitting() { }

void LShapeFitting::fitRect(Eigen::MatrixXf& cluster_matrix, RectangleData& rect) {
    rectangle_search(cluster_matrix, rect);
}

void LShapeFitting::rectangle_search(Eigen::MatrixXf& cluster_matrix, RectangleData& rect) {
    float d_theta = d_theta_deg_for_search * (M_PI / 180.0F);
    float min_cost[2] = {-FLT_MAX, 0.0F}; //{cost, theta}

    std::cout<<"Cluster matrix: "<<cluster_matrix<<std::endl;
    std::cout<<"Num rows: "<<cluster_matrix.rows()<<std::endl;
    std::cout<<"Num cols: "<<cluster_matrix.cols()<<std::endl;

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

    rect.setRectData(sin_s, cos_s, c1_s_min, c1_s_max, c2_s_min, c2_s_max);
    rect.calcRectContour();
}

float LShapeFitting::calc_closeness_criterion(Eigen::MatrixXf& c) {
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