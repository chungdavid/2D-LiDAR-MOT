#ifndef L_SHAPE_FITTING_H
#define L_SHAPE_FITTING_H

#include "perception/RectangleData.hpp"
#include <Eigen/Dense>

class LShapeFitting
{
private:
    float min_dist_of_closeness_criteria;
    float d_theta_deg_for_search;

    float calc_closeness_criterion(Eigen::MatrixXf& c);

    void rectangle_search(Eigen::MatrixXf& cluster_matrix, RectangleData& rect);

public:
    LShapeFitting(); //constructor

    ~LShapeFitting(); //destructor

    void fitRect(Eigen::MatrixXf& cluster_matrix, RectangleData& rect);
};

#endif //L_SHAPE_FITTING_H