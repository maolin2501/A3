/**
 * @file s_curve_generator.cpp
 * @brief Implementation of S-curve trajectory generator
 */

#include "rs_a3_hardware/s_curve_generator.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace rs_a3_hardware
{

SCurveGenerator::SCurveGenerator(double max_velocity, double max_acceleration, double max_jerk)
  : max_velocity_(max_velocity)
  , max_acceleration_(max_acceleration)
  , max_jerk_(max_jerk)
  , position_gain_(10.0)  // Position tracking gain
{
  reset();
}

void SCurveGenerator::setConstraints(double max_velocity, double max_acceleration, double max_jerk)
{
  max_velocity_ = std::abs(max_velocity);
  max_acceleration_ = std::abs(max_acceleration);
  max_jerk_ = std::abs(max_jerk);
}

void SCurveGenerator::getConstraints(double& max_velocity, double& max_acceleration, double& max_jerk) const
{
  max_velocity = max_velocity_;
  max_acceleration = max_acceleration_;
  max_jerk = max_jerk_;
}

void SCurveGenerator::initialize(double position, double velocity, double acceleration)
{
  state_.position = position;
  state_.velocity = velocity;
  state_.acceleration = acceleration;
  state_.jerk = 0.0;
  state_.target_position = position;
  state_.is_moving = false;
}

void SCurveGenerator::reset()
{
  state_ = SCurveState();
}

void SCurveGenerator::setTarget(double target_position)
{
  state_.target_position = target_position;
}

double SCurveGenerator::update(double dt)
{
  if (dt <= 0.0 || dt > 0.1) {
    return state_.position;
  }
  
  // Calculate position error
  double position_error = state_.target_position - state_.position;
  
  // Check if we're close enough to target
  if (std::abs(position_error) < POSITION_THRESHOLD && 
      std::abs(state_.velocity) < VELOCITY_THRESHOLD) {
    state_.velocity = 0.0;
    state_.acceleration = 0.0;
    state_.jerk = 0.0;
    state_.is_moving = false;
    state_.position = state_.target_position;
    return state_.position;
  }
  
  state_.is_moving = true;
  
  // ==================== S-Curve Real-time Algorithm ====================
  // This implements a real-time S-curve controller using jerk-limited control
  
  // Step 1: Calculate desired velocity based on position error
  // Use proportional control with velocity limiting
  double desired_velocity = position_gain_ * position_error;
  desired_velocity = clamp(desired_velocity, -max_velocity_, max_velocity_);
  
  // Step 2: Calculate stopping distance at current velocity
  // Consider S-curve deceleration profile
  double stopping_distance = calculateStoppingDistance(state_.velocity, state_.acceleration);
  
  // Step 3: Determine if we need to start decelerating
  double direction = sign(position_error);
  double velocity_direction = sign(state_.velocity);
  
  // If moving away from target or need to decelerate
  bool need_decelerate = false;
  if (velocity_direction != 0 && velocity_direction != direction) {
    // Moving in wrong direction - must decelerate
    need_decelerate = true;
  } else if (std::abs(position_error) <= std::abs(stopping_distance) * 1.1) {
    // Getting close - start decelerating (with 10% margin)
    need_decelerate = true;
  }
  
  // Step 4: Calculate desired acceleration
  double desired_acceleration;
  if (need_decelerate) {
    // Decelerate: acceleration should oppose velocity
    if (std::abs(state_.velocity) < VELOCITY_THRESHOLD) {
      desired_acceleration = 0.0;
    } else {
      desired_acceleration = -sign(state_.velocity) * max_acceleration_;
    }
  } else {
    // Accelerate toward target
    double velocity_error = desired_velocity - state_.velocity;
    desired_acceleration = clamp(velocity_error / dt, -max_acceleration_, max_acceleration_);
  }
  
  // Step 5: Apply jerk limiting (key to S-curve smoothness)
  double new_acceleration = computeJerkLimitedAcceleration(
    state_.acceleration, desired_acceleration, dt);
  
  // Step 6: Update jerk
  state_.jerk = (new_acceleration - state_.acceleration) / dt;
  
  // Step 7: Update acceleration
  state_.acceleration = new_acceleration;
  
  // Step 8: Update velocity with acceleration limiting
  double new_velocity = state_.velocity + state_.acceleration * dt;
  new_velocity = clamp(new_velocity, -max_velocity_, max_velocity_);
  state_.velocity = new_velocity;
  
  // Step 9: Update position
  // Use trapezoidal integration for better accuracy
  double avg_velocity = (state_.velocity + new_velocity) * 0.5;
  state_.position += avg_velocity * dt;
  
  // Step 10: Clamp to target if very close and velocity is small
  if (std::abs(state_.target_position - state_.position) < POSITION_THRESHOLD * 10 &&
      std::abs(state_.velocity) < VELOCITY_THRESHOLD * 10) {
    state_.position = state_.target_position;
  }
  
  return state_.position;
}

double SCurveGenerator::calculateStoppingTime(double velocity, double acceleration) const
{
  // Time to reduce acceleration to zero
  double t_jerk = std::abs(acceleration) / max_jerk_;
  
  // Velocity change during jerk phase
  double v_change_jerk = 0.5 * std::abs(acceleration) * t_jerk;
  
  // Remaining velocity after jerk phase
  double v_remaining = std::abs(velocity) - v_change_jerk;
  
  if (v_remaining <= 0) {
    // Can stop during jerk phase only
    return std::sqrt(2.0 * std::abs(velocity) / max_jerk_);
  }
  
  // Need constant deceleration phase
  // Time at constant deceleration
  double t_const = v_remaining / max_acceleration_;
  
  // Final jerk phase to bring acceleration to zero
  double t_jerk_final = max_acceleration_ / max_jerk_;
  
  return t_jerk + t_const + t_jerk_final;
}

double SCurveGenerator::calculateStoppingDistance(double velocity, double acceleration) const
{
  double v = std::abs(velocity);
  double a = std::abs(acceleration);
  
  if (v < VELOCITY_THRESHOLD) {
    return 0.0;
  }
  
  // Simplified S-curve stopping distance calculation
  // Full derivation considers all 7 segments, but for real-time control
  // we use an approximation that's sufficient for smooth deceleration
  
  // Time to reduce acceleration to zero (jerk phase 1)
  double t1 = a / max_jerk_;
  double d1 = v * t1 + 0.5 * a * t1 * t1 - (1.0/6.0) * max_jerk_ * t1 * t1 * t1;
  double v1 = v + a * t1 - 0.5 * max_jerk_ * t1 * t1;
  
  if (v1 <= 0) {
    // Stops during first jerk phase
    return std::abs(d1);
  }
  
  // Constant deceleration phase
  double t2 = (v1 - max_acceleration_ * max_acceleration_ / (2.0 * max_jerk_)) / max_acceleration_;
  if (t2 < 0) t2 = 0;
  double d2 = v1 * t2 - 0.5 * max_acceleration_ * t2 * t2;
  double v2 = v1 - max_acceleration_ * t2;
  
  // Final jerk phase
  double t3 = max_acceleration_ / max_jerk_;
  double d3 = v2 * t3 - 0.5 * max_acceleration_ * t3 * t3 + (1.0/6.0) * max_jerk_ * t3 * t3 * t3;
  
  double total_distance = d1 + d2 + d3;
  
  // Add safety margin
  return std::abs(total_distance) * 1.2;
}

double SCurveGenerator::computeJerkLimitedAcceleration(double current_acc, double desired_acc, double dt) const
{
  double acc_change = desired_acc - current_acc;
  double max_acc_change = max_jerk_ * dt;
  
  // Clamp acceleration change by jerk limit
  acc_change = clamp(acc_change, -max_acc_change, max_acc_change);
  
  double new_acc = current_acc + acc_change;
  
  // Also clamp the acceleration itself
  new_acc = clamp(new_acc, -max_acceleration_, max_acceleration_);
  
  return new_acc;
}

// ==================== Trajectory Mode Implementation ====================

SCurveProfile SCurveGenerator::calculateProfile(double start_position, double end_position,
                                                 double start_velocity, double end_velocity [[maybe_unused]])
{
  SCurveProfile profile;
  profile.j_max = max_jerk_;
  profile.a_max = max_acceleration_;
  profile.v_max = max_velocity_;
  profile.p0 = start_position;
  profile.v0 = start_velocity;
  profile.a0 = 0.0;  // Assume starting from zero acceleration
  
  double displacement = end_position - start_position;
  profile.distance = std::abs(displacement);
  profile.direction = (displacement >= 0) ? 1.0 : -1.0;
  
  if (profile.distance < EPSILON) {
    // No movement needed
    profile.total_time = 0.0;
    return profile;
  }
  
  // Calculate time to reach max acceleration (jerk phase)
  double t_j = profile.a_max / profile.j_max;
  
  // Velocity gained during one jerk phase
  double v_j = 0.5 * profile.j_max * t_j * t_j;
  
  // Distance covered during acceleration phase (including both jerk phases)
  // Assuming we reach max acceleration
  double d_acc = v_j * t_j + profile.a_max * t_j * t_j + v_j;
  
  // Check if we can reach max velocity
  double v_acc = 2 * v_j;  // Velocity gained during full acceleration phase
  
  if (v_acc >= profile.v_max) {
    // Cannot reach max acceleration - triangular profile
    calculateShortProfile(profile);
  } else {
    // Check if we reach max velocity before halfway
    double d_to_vmax = d_acc + (profile.v_max - v_acc) * (profile.v_max - v_acc) / (2.0 * profile.a_max);
    
    if (2.0 * d_to_vmax > profile.distance) {
      // Cannot reach max velocity - trapezoidal acceleration profile
      calculateShortProfile(profile);
    } else {
      // Can reach max velocity - full S-curve
      calculateLongProfile(profile);
    }
  }
  
  return profile;
}

void SCurveGenerator::calculateShortProfile(SCurveProfile& profile) const
{
  // For short distances, we may not reach max velocity or even max acceleration
  // Use a symmetric profile: t1 = t3 = t5 = t7, t2 = t6, t4 = 0
  
  double j = profile.j_max;
  double a = profile.a_max;
  // double v = profile.v_max;  // Not used in short profile (won't reach max velocity)
  double d = profile.distance;
  
  // Time to reach max acceleration
  double t_j = a / j;
  
  // Check if we reach max acceleration
  // Distance for pure jerk profile (t1 = t3, no constant acceleration)
  double d_jerk_only = j * t_j * t_j * t_j / 3.0;
  
  if (d < 2.0 * d_jerk_only) {
    // Very short distance - pure jerk profile
    double t1 = std::cbrt(d * 1.5 / j);
    profile.t1 = t1;
    profile.t2 = 0.0;
    profile.t3 = t1;
    profile.t4 = 0.0;
    profile.t5 = t1;
    profile.t6 = 0.0;
    profile.t7 = t1;
    profile.v_cruise = j * t1 * t1;  // Peak velocity
    profile.a_limit = j * t1;  // Peak acceleration
  } else {
    // Need constant acceleration phase
    // Solve for t1 and t2 given distance constraint
    // d = 2 * (v_j * t_j + 0.5 * a * t_j^2 + a * t_a * t_j + 0.5 * a * t_a^2)
    
    profile.t1 = t_j;
    profile.t3 = t_j;
    profile.t5 = t_j;
    profile.t7 = t_j;
    
    // Velocity at end of t1
    double v1 = 0.5 * j * t_j * t_j;
    
    // Distance covered during jerk phases (4 of them)
    double d_jerk = 4.0 * (v1 * t_j / 2.0 + j * t_j * t_j * t_j / 6.0);
    
    // Remaining distance for constant acceleration phases
    double d_const = d - d_jerk;
    
    // Solve for t2 (constant acceleration time)
    // Using quadratic formula
    double A = a;
    double B = 2.0 * v1 + a * t_j;
    double C = -d_const / 2.0;
    
    double discriminant = B * B - 4.0 * A * C;
    if (discriminant < 0) discriminant = 0;
    
    double t2 = (-B + std::sqrt(discriminant)) / (2.0 * A);
    if (t2 < 0) t2 = 0;
    
    profile.t2 = t2;
    profile.t6 = t2;
    profile.t4 = 0.0;  // No cruise phase
    
    profile.v_cruise = v1 + a * (t_j + t2);
    profile.a_limit = a;
  }
  
  profile.total_time = profile.t1 + profile.t2 + profile.t3 + profile.t4 +
                       profile.t5 + profile.t6 + profile.t7;
}

void SCurveGenerator::calculateLongProfile(SCurveProfile& profile) const
{
  // Full 7-segment S-curve with cruise phase
  double j = profile.j_max;
  double a = profile.a_max;
  double v = profile.v_max;
  double d = profile.distance;
  
  // Time for jerk phases
  double t_j = a / j;
  
  // Velocity gained during acceleration jerk phases
  double v_j = 0.5 * j * t_j * t_j;
  
  // Time for constant acceleration to reach max velocity
  double t_a = (v - 2.0 * v_j) / a;
  if (t_a < 0) t_a = 0;
  
  profile.t1 = t_j;
  profile.t2 = t_a;
  profile.t3 = t_j;
  profile.t5 = t_j;
  profile.t6 = t_a;
  profile.t7 = t_j;
  
  // Distance covered during acceleration phase (t1 + t2 + t3)
  double d_acc = v_j * t_j + 0.5 * a * t_j * t_j +  // t1
                 (v_j + 0.5 * a * t_j) * t_a + 0.5 * a * t_a * t_a +  // t2
                 (v_j + a * t_j + a * t_a) * t_j + 0.5 * a * t_j * t_j - j * t_j * t_j * t_j / 6.0;  // t3
  
  // Distance covered during deceleration is symmetric
  double d_dec = d_acc;
  
  // Distance for cruise phase
  double d_cruise = d - d_acc - d_dec;
  if (d_cruise < 0) d_cruise = 0;
  
  // Time for cruise phase
  profile.t4 = d_cruise / v;
  
  profile.v_cruise = v;
  profile.a_limit = a;
  
  profile.total_time = profile.t1 + profile.t2 + profile.t3 + profile.t4 +
                       profile.t5 + profile.t6 + profile.t7;
}

double SCurveGenerator::computeSegmentJerk(const SCurveProfile& profile, double t) const
{
  double j = profile.j_max * profile.direction;
  
  // Determine which segment we're in
  double t_end1 = profile.t1;
  double t_end2 = t_end1 + profile.t2;
  double t_end3 = t_end2 + profile.t3;
  double t_end4 = t_end3 + profile.t4;
  double t_end5 = t_end4 + profile.t5;
  double t_end6 = t_end5 + profile.t6;
  // double t_end7 = t_end6 + profile.t7;  // = total_time
  
  if (t < t_end1) {
    return j;  // Segment 1: positive jerk (accelerating)
  } else if (t < t_end2) {
    return 0.0;  // Segment 2: zero jerk (constant acceleration)
  } else if (t < t_end3) {
    return -j;  // Segment 3: negative jerk (reducing acceleration)
  } else if (t < t_end4) {
    return 0.0;  // Segment 4: zero jerk (cruise)
  } else if (t < t_end5) {
    return -j;  // Segment 5: negative jerk (starting deceleration)
  } else if (t < t_end6) {
    return 0.0;  // Segment 6: zero jerk (constant deceleration)
  } else {
    return j;  // Segment 7: positive jerk (ending deceleration)
  }
}

double SCurveGenerator::getPositionAtTime(const SCurveProfile& profile, double t) const
{
  if (t <= 0) return profile.p0;
  if (t >= profile.total_time) return profile.p0 + profile.distance * profile.direction;
  
  double j = profile.j_max * profile.direction;
  double p = profile.p0;
  double v = profile.v0;
  double a = profile.a0;
  
  // Time boundaries
  double t_end1 = profile.t1;
  double t_end2 = t_end1 + profile.t2;
  double t_end3 = t_end2 + profile.t3;
  double t_end4 = t_end3 + profile.t4;
  double t_end5 = t_end4 + profile.t5;
  double t_end6 = t_end5 + profile.t6;
  
  // Process each segment
  auto processSegment = [&](double dt, double jerk) {
    p += v * dt + 0.5 * a * dt * dt + (1.0/6.0) * jerk * dt * dt * dt;
    v += a * dt + 0.5 * jerk * dt * dt;
    a += jerk * dt;
  };
  
  // Segment 1
  if (t <= t_end1) {
    processSegment(t, j);
    return p;
  }
  processSegment(profile.t1, j);
  
  // Segment 2
  if (t <= t_end2) {
    processSegment(t - t_end1, 0.0);
    return p;
  }
  processSegment(profile.t2, 0.0);
  
  // Segment 3
  if (t <= t_end3) {
    processSegment(t - t_end2, -j);
    return p;
  }
  processSegment(profile.t3, -j);
  
  // Segment 4 (cruise)
  if (t <= t_end4) {
    processSegment(t - t_end3, 0.0);
    return p;
  }
  processSegment(profile.t4, 0.0);
  
  // Segment 5
  if (t <= t_end5) {
    processSegment(t - t_end4, -j);
    return p;
  }
  processSegment(profile.t5, -j);
  
  // Segment 6
  if (t <= t_end6) {
    processSegment(t - t_end5, 0.0);
    return p;
  }
  processSegment(profile.t6, 0.0);
  
  // Segment 7
  processSegment(t - t_end6, j);
  return p;
}

double SCurveGenerator::getVelocityAtTime(const SCurveProfile& profile, double t) const
{
  if (t <= 0) return profile.v0;
  if (t >= profile.total_time) return 0.0;  // Assumes ending at rest
  
  double j = profile.j_max * profile.direction;
  double v = profile.v0;
  double a = profile.a0;
  
  // Time boundaries
  double t_end1 = profile.t1;
  double t_end2 = t_end1 + profile.t2;
  double t_end3 = t_end2 + profile.t3;
  double t_end4 = t_end3 + profile.t4;
  double t_end5 = t_end4 + profile.t5;
  double t_end6 = t_end5 + profile.t6;
  
  auto processSegment = [&](double dt, double jerk) {
    v += a * dt + 0.5 * jerk * dt * dt;
    a += jerk * dt;
  };
  
  if (t <= t_end1) {
    v += a * t + 0.5 * j * t * t;
    return v;
  }
  processSegment(profile.t1, j);
  
  if (t <= t_end2) {
    v += a * (t - t_end1);
    return v;
  }
  processSegment(profile.t2, 0.0);
  
  if (t <= t_end3) {
    double dt = t - t_end2;
    v += a * dt + 0.5 * (-j) * dt * dt;
    return v;
  }
  processSegment(profile.t3, -j);
  
  if (t <= t_end4) {
    v += a * (t - t_end3);
    return v;
  }
  processSegment(profile.t4, 0.0);
  
  if (t <= t_end5) {
    double dt = t - t_end4;
    v += a * dt + 0.5 * (-j) * dt * dt;
    return v;
  }
  processSegment(profile.t5, -j);
  
  if (t <= t_end6) {
    v += a * (t - t_end5);
    return v;
  }
  processSegment(profile.t6, 0.0);
  
  double dt = t - t_end6;
  v += a * dt + 0.5 * j * dt * dt;
  return v;
}

double SCurveGenerator::getAccelerationAtTime(const SCurveProfile& profile, double t) const
{
  if (t <= 0 || t >= profile.total_time) return 0.0;
  
  double j = profile.j_max * profile.direction;
  double a = profile.a0;
  
  // Time boundaries
  double t_end1 = profile.t1;
  double t_end2 = t_end1 + profile.t2;
  double t_end3 = t_end2 + profile.t3;
  double t_end4 = t_end3 + profile.t4;
  double t_end5 = t_end4 + profile.t5;
  double t_end6 = t_end5 + profile.t6;
  
  if (t <= t_end1) {
    return a + j * t;
  }
  a += j * profile.t1;
  
  if (t <= t_end2) {
    return a;
  }
  
  if (t <= t_end3) {
    return a + (-j) * (t - t_end2);
  }
  a += (-j) * profile.t3;
  
  if (t <= t_end4) {
    return a;  // Should be ~0
  }
  
  if (t <= t_end5) {
    return a + (-j) * (t - t_end4);
  }
  a += (-j) * profile.t5;
  
  if (t <= t_end6) {
    return a;
  }
  
  return a + j * (t - t_end6);
}

void SCurveGenerator::generateTrajectory(const SCurveProfile& profile, double dt,
                                          std::vector<double>& positions,
                                          std::vector<double>& velocities,
                                          std::vector<double>& accelerations) const
{
  positions.clear();
  velocities.clear();
  accelerations.clear();
  
  if (profile.total_time <= 0 || dt <= 0) {
    positions.push_back(profile.p0);
    velocities.push_back(profile.v0);
    accelerations.push_back(profile.a0);
    return;
  }
  
  int num_points = static_cast<int>(std::ceil(profile.total_time / dt)) + 1;
  positions.reserve(num_points);
  velocities.reserve(num_points);
  accelerations.reserve(num_points);
  
  for (double t = 0; t <= profile.total_time; t += dt) {
    positions.push_back(getPositionAtTime(profile, t));
    velocities.push_back(getVelocityAtTime(profile, t));
    accelerations.push_back(getAccelerationAtTime(profile, t));
  }
  
  // Ensure we have the final point
  if (positions.empty() || 
      std::abs(positions.back() - (profile.p0 + profile.distance * profile.direction)) > EPSILON) {
    positions.push_back(profile.p0 + profile.distance * profile.direction);
    velocities.push_back(0.0);
    accelerations.push_back(0.0);
  }
}

}  // namespace rs_a3_hardware
