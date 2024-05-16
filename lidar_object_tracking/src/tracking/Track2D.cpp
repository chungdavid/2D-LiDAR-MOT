#include "Track2D.hpp"

Track2D::Track2D(unsigned int id,
                double p_x,
                double p_y,
                double theta,
                double l,
                double w,
                double dt) {
    track_id_ = id;
    position_ = {p_x, p_y};
    velocity_ = {0, 0};
    length_ = l;
    width_ = w;
    theta_ = theta;

    coasted_ = false;
    confirmed_ = false;
    deletable_ = false;

    int num_states = 4; //(x, y, vx, vy)
    int num_measurements = 2; //(x, y)

    //constant matricies
    Eigen::MatrixXd A(num_states, num_states); //A - System dynamics matrix
    Eigen::MatrixXd H(num_measurements, num_states); //H - Observation matrix. Multiply a state vector by H to translate it to a measurement vector.
    Eigen::MatrixXd Q(num_states, num_states); //Q - Process noise covariance
    Eigen::MatrixXd R(num_measurements, num_measurements); //R - Measurement noise covariance

    A << 1, 0, dt, 0, 
         0, 1, 0, dt, 
         0, 0, 1, 0, 
         0, 0, 0, 1, 
    
    H << 1, 0, 0, 0, 
         0, 1, 0, 0, 

    //TODO: Tune Q and R matrices
    Q << 2, 0, 0, 0,
         0, 2, 0, 0,
         0, 0, 12, 0,
         0, 0, 0, 12;

    R.setIdentity();
    
    //initial estimates
    Eigen::MatrixXd P0(num_states, num_states); //P0 - Initial estimate error covariance
    Eigen::VectorXd initial_state(num_states);
    P0.setIdentity();
    initial_state << p_x, p_y, 0, 0;

    kalman_filter_ = KalmanFilter(dt, A, H, Q, R, P0);
    kalman_filter_.init(0, initial_state);
}

Track2D::~Track2D() { }

void Track2D::kalmanPredict() {
    kalman_filter_.predict();
    Eigen::VectorXd predicted_state = kalman_filter_.predictedState();
    position_[0] = predicted_state(0);
    position_[1] = predicted_state(1);
    velocity_[0] = predicted_state(2);
    velocity_[1] = predicted_state(3);
    coasted_ = true;
}

void Track2D::kalmanUpdate(double p_x_new,
                double p_y_new,
                double theta_new,
                double l_new,
                double w_new) {
    //TODO implement a filter for length, width, and theta
    length_ = l_new;
    width_ = w_new;
    theta_ = theta_new;

    Eigen::VectorXd measured_state(2);
    measured_state << p_x_new, p_y_new;
    kalman_filter_.update(measured_state);

    Eigen::VectorXd updated_state = kalman_filter_.state();
    position_[0] = updated_state(0);
    position_[1] = updated_state(1);
    velocity_[0] = updated_state(2);
    velocity_[1] = updated_state(3);
    coasted_ = false;
}

void Track2D::updateDetectionHistory() {
    detection_history_[history_i_] = !coasted_;
    history_i_ = (history_i_ + 1) % 4; 
    
    //count the number of times a detection was associated with the track
    int detection_count = 0;
    for(bool h : detection_history_) {
        if(h) {
            ++detection_count;
        }
    }

    if(detection_count == 4) { 
        confirmed_ = true;
    } else if(detection_count == 0) {
        deletable_ = true;
    }
}

bool Track2D::isDeletable() {
    return deletable_;
}

bool Track2D::isConfirmed() {
    return confirmed_;
}