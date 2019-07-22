#ifndef ARRC_PID_HPP
#define ARRC_PID_HPP
#include <mbed.h>

namespace arrc {
class PidVelocity {
public:
  PidVelocity(double p, double i, double d) {
    setParam(p, i, d);
    time_ = new Timer;
    time_->start();
  }
  void setParam(double p, double i, double d) {
    p_gain_ = p;
    i_gain_ = i;
    d_gain_ = d;
  }
  double control(double goal, double out) {
    delta_t_ = time_->read();
    time_->reset();
    prev_prev_error_ = prev_error_;
    prev_error_ = current_error_;
    current_error_ = goal - out;
    double p_item = p_gain_ * (current_error_ - prev_error_);
    double i_item = i_gain_ * current_error_ * delta_t_;
    double d_item =
        d_gain_ *
        ((current_error_ - prev_error_) - (prev_error_ - prev_prev_error_)) /
        delta_t_;
    control_ += p_item + i_item + d_item;
    return control_;
  }
  ~PidVelocity() { delete time_; }

private:
  double p_gain_, i_gain_, d_gain_;
  Timer *time_;
  double delta_t_;
  double control_ = 0;
  double current_error_ = 0, prev_error_ = 0, prev_prev_error_ = 0;
};
} // namespace arrc

#endif
