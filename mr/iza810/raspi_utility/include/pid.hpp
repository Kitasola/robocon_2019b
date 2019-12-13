#ifndef ARRC_PID_HPP
#define ARRC_PID_HPP
#include "time.hpp"

namespace arrc_raspi {
class PidVelocity {
public:
  PidVelocity(double p, double i, double d, double max) {
    setParam(p, i, d, max);
    delta_t_.reset();
  }
  void setParam(double p, double i, double d, double max) {
    p_gain_ = p;
    i_gain_ = i;
    d_gain_ = d;
    max_control_ = max;
  }
  double control(double goal, double out) { return control(goal - out); }
  double control(double error) {
    delta_t_.update();
    delta_t_.reset();
    prev_prev_error_ = prev_error_;
    prev_error_ = current_error_;
    current_error_ = error;
    p_item_ = p_gain_ * (current_error_ - prev_error_);
    i_item_ = i_gain_ * current_error_;
    d_item_ =
        d_gain_ *
        ((current_error_ - prev_error_) - (prev_error_ - prev_prev_error_)) /
        delta_t_.read();
    control_ = p_item_ + i_item_ + d_item_;
    if (control_ > max_control_) {
      control_ = max_control_;
    } else if (control_ < -max_control_) {
      control_ = -max_control_;
    }
    return control_;
  }

private:
  double p_gain_, i_gain_, d_gain_;
  double p_item_ = 0, i_item_ = 0, d_item_ = 0;
  Timer delta_t_;
  double control_ = 0, max_control_ = 0;
  double current_error_ = 0, prev_error_ = 0, prev_prev_error_ = 0;
};

class PidPosition {
public:
  PidPosition(double p, double i, double d, double max) {
    setParam(p, i, d, max);
    delta_t_.reset();
  }
  void setParam(double p, double i, double d, double max) {
    p_gain_ = p;
    i_gain_ = i;
    d_gain_ = d;
    max_control_ = max;
  }
  double control(double goal, double out) { return control(goal - out); }
  double control(double error) {
    delta_t_.update();
    delta_t_.reset();
    prev_error_ = current_error_;
    current_error_ = error;
    p_item_ = p_gain_ * current_error_;
    i_item_ += i_gain_ * current_error_ * delta_t_.read();
    if (i_item_ > max_control_) {
      i_item_ = max_control_;
    } else if (i_item_ < -max_control_) {
      i_item_ = -max_control_;
    }
    d_item_ = d_gain_ * (current_error_ - prev_error_) / delta_t_.read();
    control_ = p_item_ + i_item_ + d_item_;
    if (control_ > max_control_) {
      control_ = max_control_;
    } else if (control_ < -max_control_) {
      control_ = -max_control_;
    }
    return control_;
  }
  void reset() { i_gain_ = 0; }

private:
  double p_gain_, i_gain_, d_gain_;
  double p_item_ = 0, i_item_ = 0, d_item_ = 0;
  Timer delta_t_;
  double control_ = 0, max_control_ = 0;
  double current_error_ = 0, prev_error_ = 0;
};
} // namespace arrc_raspi

#endif
