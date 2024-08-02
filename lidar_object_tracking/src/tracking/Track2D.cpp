#include "lidar_object_tracking/tracking/Track2D.hpp"

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

    // init kalman filter
    int num_states = 4; //(x, y, vx, vy)
    int num_measurements = 2; //(x, y)

    // constant matricies
    Eigen::MatrixXd A(num_states, num_states); //A - System dynamics matrix
    Eigen::MatrixXd H(num_measurements, num_states); //H - Observation matrix. Multiply a state vector by H to translate it to a measurement vector.
    Eigen::MatrixXd Q(num_states, num_states); //Q - Process noise covariance
    Eigen::MatrixXd R(num_measurements, num_measurements); //R - Measurement noise covariance

    A << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;
    
    H << 1, 0, 0, 0,
         0, 1, 0, 0;

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

    // init kalman filter for shape
    num_states = 2; //(l, w)
    num_measurements = 2; //(l, w)

    // constant matricies
    Eigen::MatrixXd As(num_states, num_states); //A - System dynamics matrix
    Eigen::MatrixXd Hs(num_measurements, num_states); //H - Observation matrix. Multiply a state vector by H to translate it to a measurement vector.
    Eigen::MatrixXd Qs(num_states, num_states); //Q - Process noise covariance
    Eigen::MatrixXd Rs(num_measurements, num_measurements); //R - Measurement noise covariance

    As << 1, 0,
         0, 1;
    
    Hs << 1, 0,
         0, 1;

    //TODO: Tune Q and R matrices
    Qs << 0.04, 0,
         0, 0.04;

    Rs.setIdentity();
    
    //initial estimates
    Eigen::MatrixXd P0s(num_states, num_states); //P0 - Initial estimate error covariance
    Eigen::VectorXd initial_shape(num_states);
    P0s.setIdentity();
    initial_shape << l, w;

    kalman_filter_shape_ = KalmanFilter(dt, As, Hs, Qs, Rs, P0s);
    kalman_filter_shape_.init(0, initial_shape);
}

Track2D::~Track2D() { }

void Track2D::kalmanPredict() {
    kalman_filter_.predict();
    kalman_filter_shape_.predict();
    Eigen::VectorXd predicted_state = kalman_filter_.predictedState();
    Eigen::VectorXd predicted_shape = kalman_filter_shape_.predictedState();
    position_[0] = predicted_state(0);
    position_[1] = predicted_state(1);
    velocity_[0] = predicted_state(2);
    velocity_[1] = predicted_state(3);
    length_ = predicted_shape(0);
    width_ = predicted_shape(1);
    coasted_ = true;
}

void Track2D::kalmanUpdate(double p_x_new,
                double p_y_new,
                double theta_new,
                double l_new,
                double w_new) {
    // The l-shape fitting algorithm only outputs 0 <= theta <= 90deg for orientation
    // As a result, the length and width of a detection will sometimes switch to keep the orientation in that range 
    // The downside is that the theta doesn't really tell us anything meaningful about orientation of the object
    // To address this challenge properly I would have to make changes to the lidar_object_detection & lidar_object_tracking pipelines

    // For now, we just identify when the length and width were swapped. If they are, switch them and adjust the theta accordingly
    // Not ideal but it works for the time being
    // Here, we determine if the lengths & widths of the incoming detections were swapped by calculating the cost 
    // of swapping them vs not swapping them before updating the side lengths. The cost is the product of the % change in the side lengths
    double cost_swapped = std::abs( (l_new - width_) / width_ * (w_new - length_) / length_ );
    double cost_not_swapped = std::abs( (w_new - width_) / width_ * (l_new - length_) / length_ );

    double cost_diff = std::abs(cost_swapped - cost_not_swapped);

    //there is a significant difference in cost...
    if(cost_diff > 0.05) { 
        // swap if the cost is lower
        if(cost_swapped < cost_not_swapped) {
            std::swap(l_new, w_new);
            theta_new += M_PI / 2;
        }

        if(l_new < length_) {
            // double diff = length_ - l_new;
            // p_x_new += diff * cos(theta_new);
            // p_y_new += diff * sin(theta_new);
            l_new = length_;
        }

        if(w_new < width_) {
            // double diff = width_ - w_new;
            // p_y_new += diff * cos(theta_new);
            // p_x_new += diff * sin(theta_new);
            w_new = width_;
        }
    } else { // if there is no significant difference in cost, don't update the lengths and widths
        l_new = length_;
        w_new = width_;
    }

    theta_ = theta_new;
    
    // Update pos and vel kalman filter
    Eigen::VectorXd measured_state(2);
    measured_state << p_x_new, p_y_new;
    kalman_filter_.update(measured_state);

    Eigen::VectorXd updated_state = kalman_filter_.state();
    position_[0] = updated_state(0);
    position_[1] = updated_state(1);
    velocity_[0] = updated_state(2);
    velocity_[1] = updated_state(3);
    coasted_ = false;

    // Update shape kalman filter
    Eigen::VectorXd measured_shape(2);
    measured_shape << l_new, w_new;
    kalman_filter_shape_.R<< pow(l_new,-2), 0,
                            0, pow(w_new,-2);
    kalman_filter_shape_.update(measured_shape);

    Eigen::VectorXd updated_shape = kalman_filter_shape_.state();
    length_ = updated_shape(0);
    width_ = updated_shape(1);
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