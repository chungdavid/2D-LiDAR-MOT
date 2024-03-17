#include "perception/RectangleData.hpp"
#include<iostream>

RectangleData::RectangleData() { }
RectangleData::~RectangleData() { }

void RectangleData::calc_cross_point(const float a0, const float a1, const float b0, const float b1, const float c0,
                                    const float c1, float& x, float& y)
{
    x = (b0 * (-c1) - b1 * (-c0)) / (a0 * b1 - a1 * b0);
    y = (a1 * (-c0) - a0 * (-c1)) / (a0 * b1 - a1 * b0);
}

void RectangleData::calcRectContour() {
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

void RectangleData::setRectData(const float sin_theta, const float cos_theta, const float c1_min, const float c1_max, const float c2_min,
                    const float c2_max)
{
    // std::cout<<"setRectData():"<<sin_theta<<" "<<cos_theta<<" "<<c1_min<<" "<<c1_max<<" "<<c2_min<<" "<<c2_max<<"\n";

    a_[0] = cos_theta;
    b_[0] = sin_theta;
    c_[0] = c1_min;

    a_[1] = -sin_theta;
    b_[1] = cos_theta;
    c_[1] = c2_min;

    a_[2] = cos_theta;
    b_[2] = sin_theta;
    c_[2] = c1_max;

    a_[3] = -sin_theta;
    b_[3] = cos_theta;
    c_[3] = c2_max;
}

std::array<float, 4> RectangleData::getRectPointsX() const {
    return rect_c_x_;
}

std::array<float, 4> RectangleData::getRectPointsY() const {
    return rect_c_y_;
}