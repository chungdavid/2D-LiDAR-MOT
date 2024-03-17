#ifndef RECTANGLE_DATA_H
#define RECTANGLE_DATA_H

#include <array>

class RectangleData
{
private:
    std::array<float, 3> a_;
    std::array<float, 3> b_;
    std::array<float, 3> c_;

    std::array<float, 4> rect_c_x_;
    std::array<float, 4> rect_c_y_;

    static void calc_cross_point(const float a0, const float a1, const float b0, const float b1, const float c0,
                                    const float c1, float& x, float& y);

public:
    RectangleData();

    ~RectangleData();
    
    void calcRectContour();

    void setRectData(const float sin_theta, const float cos_theta, const float c1_min, const float c1_max, const float c2_min,
                    const float c2_max);

    std::array<float, 4> getRectPointsX() const;
    std::array<float, 4> getRectPointsY() const;
};

#endif //RECTANGLE_DATA_H