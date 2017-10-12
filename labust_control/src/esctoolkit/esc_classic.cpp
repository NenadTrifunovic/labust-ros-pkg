/*
 * esc_classic.cpp
 *
 *  Created on: Dec 15, 2014
 *      Author: Filip Mandic
 */

#include <labust/control/esc/EscClassic.hpp>

#include <ros/ros.h>

namespace labust
{
namespace control
{
namespace esc
{
typedef EscClassic Base;

Base::EscClassic(int ctrlNum, numericprecission Ts)
  : EscPerturbationBase<double>(ctrlNum, Ts), state_(vector::Zero(controlNum))
{
  lpf_out_old_.resize(controlNum);
  signal_demodulated_old_.resize(controlNum);
  comp_old_.resize(controlNum);
  corr_signal_.resize(controlNum);
  control_ref_.resize(controlNum);
  sin_amp_.resize(controlNum);
  sin_demodulation_amp_.resize(controlNum);
  sin_freq_.resize(controlNum);
  gain_.resize(controlNum);
  control_.resize(controlNum);
  high_pass_pole_ = 0;
  low_pass_pole_.resize(controlNum);
  comp_pole_.resize(controlNum);
  comp_zero_.resize(controlNum);
  lpf_out_old_.setZero();
  hpf_out_old_ = 0;
  control_ref_ = state_;
  lpf_out_old_.setZero();
  signal_demodulated_old_.setZero();
  comp_old_.setZero();
  corr_signal_.setZero();

  phase_shift_.resize(controlNum);
  phase_shift_[0] = 0;
  for (size_t i = 1; i < controlNum; i++)
  {
    phase_shift_[i] = i * M_PI / ((double)controlNum);
  }
  state_initialized_ = true;
}

Base::~EscClassic()
{
}

void Base::initController(double sin_amp, double sin_demodulation_amp,
                          double sin_freq, double corr_gain,
                          double high_pass_pole, double low_pass_pole,
                          double comp_zero, double comp_pole, double Ts)
{
  sin_amp_.setConstant(sin_amp);
  sin_demodulation_amp_.setConstant(sin_demodulation_amp);
  sin_freq_.setConstant(sin_freq);
  gain_.setConstant(corr_gain);
  high_pass_pole_ = high_pass_pole;
  low_pass_pole_.setConstant(low_pass_pole);
  comp_pole_.setConstant(comp_pole);
  comp_zero_.setConstant(comp_zero);
  Ts_ = Ts;
  if (initialized_ == false)
  {
    obj_val_old_ = 0;
    cycle_count_ = 0;
    hpf_out_old_ = 0;
    lpf_out_old_.setZero();
    state_initialized_ = false;
    old_vals_initialized_ = false;
    initialized_ = true;
  }
}
Base::numericprecission Base::preFiltering(numericprecission cost_signal)
{
  numericprecission filtered_cost;
  if (high_pass_pole_ == 0)
    filtered_cost = cost_signal;
  else
  {
    bool alternative_version(true);
    if (!alternative_version)
    {
      filtered_cost = (-(Ts_ * high_pass_pole_ - 2L) * pre_filter_output_old_ +
                       2L * cost_signal - 2L * pre_filter_input_old_) /
                      (2L + high_pass_pole_ * Ts_);
      std::cerr << "Using standard prefilter filter. high-pass pole: "
                << high_pass_pole_ << ", cost_signal: " << cost_signal
                << ", filtered_cost: " << filtered_cost << std::endl;
    }
    else
    {
      pre_filter_output_old_ = -pre_filter_output_old_ + pre_filter_input_old_;
      filtered_cost =
          cost_signal -
          ((long double)((long double)(2.0L - high_pass_pole_ * Ts_) *
                             pre_filter_output_old_ +
                         (long double)high_pass_pole_ * Ts_ * cost_signal +
                         (long double)high_pass_pole_ * Ts_ *
                             pre_filter_input_old_) /
           (long double)(2.0L + high_pass_pole_ * Ts_));
      std::cerr << "Using alternative  prefilter filter. high-pass pole: "
                << high_pass_pole_ << ", cost_signal: " << cost_signal
                << ", filtered_cost: " << filtered_cost << std::endl;
      std::cerr << ", pre_filter_output_old_: " << pre_filter_output_old_
                << ", pre_filter_input_old_: " << pre_filter_input_old_
                << ", Ts_" << Ts_ << std::endl;
    }
  }
  return filtered_cost;
}

Base::vector Base::gradientEstimation(numericprecission cost_signal_filtered,
                                      vector additional_input)
{
  vector signal_demodulated(controlNum);

  for (size_t i = 0; i < controlNum; i++)
  {
    signal_demodulated(i) =
        cost_signal_filtered * sin_demodulation_amp_(i) *
        std::sin(double(cycle_count_ * Ts_ * sin_freq_(i) + phase_shift_(i)));
  }
  return signal_demodulated;
}

Base::vector Base::postFiltering(vector estimated_gradient)
{
  vector lpf_out(controlNum);
  vector comp_out(controlNum);

  for (size_t i = 0; i < controlNum; i++)
  {
    if (low_pass_pole_[i] == 0)
      lpf_out[i] = estimated_gradient[i];
    else
      lpf_out[i] = ((2.0 - low_pass_pole_[i] * Ts_) * lpf_out_old_[i] +
                    low_pass_pole_[i] * Ts_ * estimated_gradient[i] +
                    low_pass_pole_[i] * Ts_ * estimated_gradient_old_[i]) /
                   (2.0 + low_pass_pole_[i] * Ts_);

    if (comp_pole_[i] == 0)
      comp_out[i] = lpf_out[i];
    else
      comp_out[i] = ((2.0 + Ts_ * comp_zero_[i]) * lpf_out[i] +
                     (Ts_ * comp_zero_[i] - 2.0) * lpf_out_old_[i] -
                     (Ts_ * comp_pole_[i] - 2.0) * comp_old_[i]) /
                    (2.0 + Ts_ * comp_pole_[i]);

    lpf_out_old_[i] = lpf_out[i];
    comp_old_[i] = comp_out[i];
  }

  return comp_out;
}

Base::vector Base::controllerGain(vector postFiltered)
{
  control_ = gain_.cwiseProduct(postFiltered);

  return control_;
}

Base::vector Base::superimposePerturbation(Base::vector control)
{
  for (size_t i = 0; i < controlNum; i++)
  {
    if (!old_vals_initialized_)
      control_ref_[i] =
          sin_amp_[i] * std::sin(double(cycle_count_ * Ts_ * sin_freq_[i] +
                                        phase_shift_[i] + M_PI / 2));
    else
      control_ref_[i] =
          control[i] +
          sin_amp_[i] * std::sin(double(cycle_count_ * Ts_ * sin_freq_[i] +
                                        phase_shift_[i] + M_PI / 2));
  }

  return control_ref_;
}
}
}
}
