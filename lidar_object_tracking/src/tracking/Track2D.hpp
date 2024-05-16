#ifndef TRACK_2D_H
#define TRACK_2D_H

#include <vector>
#include "kalman-cpp/kalman.hpp"

class Track2D
{
public:
    Track2D(unsigned int id,
            double p_x,
            double p_y,
            double theta,
            double l,
            double w,
            double dt);
    ~Track2D();

    // information about the detection
    unsigned int track_id_;
    std::vector<double> position_;
    std::vector<double> velocity_;
    double theta_;
    double width_;
    double length_;

    void updateDetectionHistory();
    bool isDeletable();
    bool isConfirmed();
    void kalmanPredict();
    void kalmanUpdate(double p_x_new, double p_y_new, double theta_new, double l_new, double w_new);

private:
    KalmanFilter kalman_filter_;

    bool coasted_;
    bool confirmed_;
    bool deletable_;

    bool detection_history_[4];
    int history_i_ = 0; //index for detection_history_
};

#endif //TRACK_2D_H