#pragma once

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <graph_msf/core/optimizer/OptimizerBase.h>
#include <graph_msf/factors/non_expression/ArticulatedTurnFactors.h>
#include <graph_msf/factors/non_expression/BaseVerticalGyroFactor.h>
#include <graph_msf/interface/ArticulatedTurnRuntime.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <mutex>
#include <optional>
#include <set>

namespace graph_msf {

// GraphManager serializes append/optimization with its lifecycle mutex. This
// object's mutex also protects asynchronous raw inputs and public snapshots.
// Staged keys are never reused after a failed optimizer transaction: a failure
// latches invalid until the estimator is restarted.
class ArticulatedTurnGraph {
public:
  explicit ArticulatedTurnGraph(const ArticulatedTurnConfig &config)
      : config_(config), R_IC_(config.R_imu_cabin) {
    const double positive[] = {config.initial_turn_sigma_rad,
                               config.leg_bias_prior_sigma_rad_s,
                               config.leg_bias_random_walk_rad_s_sqrt_s,
                               config.parked_heading_sigma_rad,
                               config.minimum_interval_sigma_rad,
                               config.maximum_sample_age_s,
                               config.maximum_interval_s,
                               config.maximum_output_age_s,
                               config.maximum_turn_sigma_rad,
                               config.missing_rate_sigma_rad_s,
                               config.maximum_base_angular_rate_rad_s};
    for (double value : positive)
      if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument("Articulated turn configuration requires "
                                    "positive finite limits/noises");
    if (!config.R_imu_cabin.allFinite() ||
        (config.R_imu_cabin.transpose() * config.R_imu_cabin -
         Eigen::Matrix3d::Identity())
                .norm() > 1e-6 ||
        std::abs(config.R_imu_cabin.determinant() - 1.0) > 1e-6 ||
        config.minimum_leg_count < 1 || config.maximum_queued_samples < 2)
      throw std::invalid_argument(
          "Invalid articulated turn extrinsic or input limits");
    state_.enabled = config.enable;
    state_.reason =
        config.enable
            ? "waiting for initial reference and independent BASE gyros"
            : "disabled";
  }

  bool addSample(const ArticulatedTurnSample &sample) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!config_.enable || state_.fault_latched ||
        !std::isfinite(sample.stamp_sec) ||
        (initialized_ &&
         sample.stamp_sec < last_time_ - config_.maximum_sample_age_s - kArticulatedTimestampToleranceSec))
      return false;
    std::set<uint32_t> ids;
    for (const auto &leg : sample.legs) {
      if (!ids.insert(leg.sensor_id).second || !std::isfinite(leg.stamp_sec) ||
          leg.stamp_sec > sample.stamp_sec + kArticulatedTimestampToleranceSec ||
          sample.stamp_sec - leg.stamp_sec > config_.maximum_sample_age_s + kArticulatedTimestampToleranceSec ||
          !std::isfinite(leg.vertical_rate_rad_s) ||
          std::abs(leg.vertical_rate_rad_s) >
              config_.maximum_base_angular_rate_rad_s ||
          !leg.gravity_sensor.allFinite() ||
          std::abs(leg.gravity_sensor.norm() - 1.0) > 0.02 ||
          !std::isfinite(leg.sigma_vertical_rate_rad_s) ||
          leg.sigma_vertical_rate_rad_s <= 0.0)
        return false;
    }
    if (samples_.size() >= config_.maximum_queued_samples) {
      faultUnlocked_("independent BASE input queue overflow");
      return false;
    }
    return samples_.emplace(sample.stamp_sec, sample).second;
  }

  ArticulatedTurnState state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }
  void fault(const std::string &reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    faultUnlocked_(reason);
  }

  // All arguments are the SAME atomic X/V/B graph transaction. Output values
  // are inserted with X; no factor can refer to a Q that was not staged with
  // it.
  void append(uint64_t key, double time, const gtsam::Pose3 &pose,
              gtsam::NonlinearFactorGraph &graph, gtsam::Values &values,
              std::map<gtsam::Key, double> &timestamps) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!config_.enable || state_.fault_latched)
      return;
    if (!std::isfinite(time) || (initialized_ && time <= last_time_)) {
      faultUnlocked_("non-monotonic articulated graph timestamp");
      return;
    }
    auto latest = samples_.upper_bound(time + kArticulatedTimestampToleranceSec);
    const ArticulatedTurnSample *sample =
        latest == samples_.begin() ? nullptr : &std::prev(latest)->second;
    const bool fresh =
        sample && time - sample->stamp_sec <= config_.maximum_sample_age_s + kArticulatedTimestampToleranceSec;
    const auto X = [](uint64_t k) { return gtsam::Symbol('x', k); };
    const auto Q = [](uint64_t k) { return gtsam::Symbol('q', k); };
    double q = last_q_;
    bool interval_valid = false;
    bool parked = false;
    int count = 0;
    if (!initialized_) {
      if (!fresh || !std::isfinite(sample->initial_turn_reference_rad) ||
          sample->legs.size() < static_cast<size_t>(config_.minimum_leg_count))
        return;
      q = sample->initial_turn_reference_rad;
      graph.emplace_shared<gtsam::PriorFactor<double>>(
          Q(key), q,
          gtsam::noiseModel::Isotropic::Sigma(1,
                                              config_.initial_turn_sigma_rad));
      initialized_ = true;
      parked = sample->parked;
    } else {
      const double dt = time - last_time_;
      auto intervals = integrate_(last_time_, time, parked);
      interval_valid =
          dt <= config_.maximum_interval_s + kArticulatedTimestampToleranceSec &&
          intervals.size() >= static_cast<size_t>(config_.minimum_leg_count);
      if (interval_valid) {
        // This is an initial guess only. Cabin information enters the existing
        // IMU likelihood exactly once; the independent leg factor couples Q/X.
        double corrected_angle = 0.0;
        for (const auto &item : intervals) {
          auto old = biases_.find(item.first);
          corrected_angle +=
              item.second.z - (old == biases_.end()
                                   ? 0.0
                                   : item.second.A.dot(old->second.mean));
        }
        corrected_angle /= intervals.size();
        for (int iteration = 0; iteration < 5; ++iteration) {
          const auto Bi = articulatedBaseRotation(last_pose_, last_q_, R_IC_);
          gtsam::Matrix31 Hq;
          const auto Bj =
              articulatedBaseRotation(pose, q, R_IC_, OptionalNone, Hq);
          gtsam::Matrix33 Hlog;
          const auto delta = gtsam::Rot3::Logmap(Bj * Bi.inverse(), Hlog);
          const double derivative = (Hlog.row(2) * Bi.matrix() * Hq)(0);
          if (std::abs(derivative) < 0.1) {
            interval_valid = false;
            break;
          }
          q -= (delta.z() - corrected_angle) / derivative;
        }
      }
      if (interval_valid) {
        for (const auto &item : intervals) {
          const uint32_t sensor = item.first;
          const auto &integral = item.second;
          auto old = biases_.find(sensor);
          const gtsam::Key bias_key = gtsam::Symbol('u', ++bias_counter_);
          const gtsam::Vector3 mean = old == biases_.end()
                                          ? gtsam::Vector3::Zero().eval()
                                          : old->second.mean;
          values.insert(bias_key, mean);
          if (old == biases_.end()) {
            graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
                bias_key, gtsam::Vector3::Zero().eval(),
                gtsam::noiseModel::Isotropic::Sigma(
                    3, config_.leg_bias_prior_sigma_rad_s));
          } else {
            graph.emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(
                old->second.key, bias_key, gtsam::Vector3::Zero().eval(),
                gtsam::noiseModel::Isotropic::Sigma(
                    3, config_.leg_bias_random_walk_rad_s_sqrt_s *
                           std::sqrt(time - old->second.time)));
          }
          const double commutator_bound =
              0.5 * std::pow(config_.maximum_base_angular_rate_rad_s * dt, 2);
          const double sigma =
              std::sqrt(integral.variance +
                        std::pow(config_.minimum_interval_sigma_rad, 2) +
                        commutator_bound * commutator_bound);
          graph.emplace_shared<BaseVerticalGyroFactor>(
              X(last_key_), Q(last_key_), X(key), Q(key), bias_key, integral.z,
              integral.A, R_IC_, gtsam::noiseModel::Isotropic::Sigma(1, sigma));
          biases_[sensor] = Bias{bias_key, time, mean, integral.last_source};
          ++count;
        }
      } else {
        parked = false;
        // Explicit missing-input process uncertainty; no encoder reanchor and
        // no confidence-preserving hold. Validity stays false for this node.
        q = last_q_;
        graph.emplace_shared<gtsam::BetweenFactor<double>>(
            Q(last_key_), Q(key), 0.0,
            gtsam::noiseModel::Isotropic::Sigma(
                1, config_.missing_rate_sigma_rad_s * dt));
      }
    }
    parked = parked && fresh && sample->parked;
    values.insert(Q(key), q);
    timestamps[Q(key)] = time;
    if (parked) {
      if (!heading_key_) {
        heading_key_ = gtsam::Symbol('h', ++heading_counter_);
        values.insert(*heading_key_,
                      articulatedBaseRotation(pose, q, R_IC_).yaw());
      }
      graph.emplace_shared<ParkedBaseHeadingFactor>(
          X(key), Q(key), *heading_key_, R_IC_,
          gtsam::noiseModel::Isotropic::Sigma(
              1, config_.parked_heading_sigma_rad));
      timestamps[*heading_key_] = time; // retain one uncertain shared heading
    } else
      heading_key_.reset();
    for (const auto &item : biases_)
      timestamps[item.second.key] = time;
    last_key_ = key;
    last_time_ = time;
    last_q_ = q;
    last_pose_ = pose;
    pending_valid_ = interval_valid;
    pending_parked_ = parked;
    pending_leg_count_ = count;
    // Keep the last sample before the graph boundary for causal ZOH
    // integration.
    auto keep = samples_.upper_bound(time);
    if (keep != samples_.begin())
      samples_.erase(samples_.begin(), std::prev(keep));
  }

  void optimized(OptimizerBase &optimizer, uint64_t key) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || state_.fault_latched || key != last_key_)
      return;
    try {
      ArticulatedTurnState candidate;
      candidate.enabled = true;
      candidate.initialized = true;
      candidate.graph_key = key;
      candidate.stamp_sec = last_time_;
      candidate.pose_world_imu =
          optimizer.calculateEstimatedPose3(gtsam::Symbol('x', key));
      candidate.position_rad =
          optimizer.calculateEstimatedScalar(gtsam::Symbol('q', key));
      const auto covariance = optimizer.calculateJointCovarianceAtKeys(
          gtsam::Symbol('x', key), gtsam::Symbol('q', key));
      if (covariance.rows() != 7 || covariance.cols() != 7 ||
          !covariance.allFinite() || !std::isfinite(candidate.position_rad))
        throw std::runtime_error(
            "invalid articulated posterior dimensions/value");
      candidate.joint_covariance = 0.5 * (covariance + covariance.transpose());
      if (Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 7, 7>>(
              candidate.joint_covariance)
              .eigenvalues()
              .minCoeff() < -1e-10)
        throw std::runtime_error(
            "articulated posterior is not positive semidefinite");
      for (auto &item : biases_) {
        item.second.mean = optimizer.calculateEstimatedPoint3(item.second.key);
        candidate.leg_bias_sensor_rad_s[item.first] = item.second.mean;
        const auto bias_covariance =
            optimizer.calculateMarginalCovarianceMatrixAtKey(item.second.key);
        if (bias_covariance.rows() != 3 || !bias_covariance.allFinite())
          throw std::runtime_error("invalid leg bias covariance");
        // Inactive sensor states are retained but have no new graph interval.
        // Forecast their random walk to this snapshot time for the predictor.
        candidate.leg_bias_covariance_sensor_rad2_s2[item.first] =
            bias_covariance +
            gtsam::Matrix33::Identity() *
                std::pow(config_.leg_bias_random_walk_rad_s_sqrt_s, 2) *
                std::max(0.0, last_time_ - item.second.time);
      }
      candidate.parked = pending_parked_;
      candidate.fused_leg_count = pending_leg_count_;
      candidate.valid =
          pending_valid_ && candidate.joint_covariance(6, 6) <=
                                std::pow(config_.maximum_turn_sigma_rad, 2);
      candidate.reason =
          !pending_valid_
              ? "independent BASE gyro interval unavailable"
              : (candidate.valid ? "ok"
                                 : "joint angle uncertainty exceeds limit");
      last_q_ = candidate.position_rad;
      last_pose_ = candidate.pose_world_imu;
      state_ = std::move(candidate);
    } catch (const std::exception &e) {
      faultUnlocked_(std::string("articulated posterior extraction failed: ") +
                     e.what());
    }
  }

private:
  struct Bias {
    gtsam::Key key;
    double time;
    gtsam::Vector3 mean;
    double last_source;
  };
  struct Integral {
    double z{0.0};
    gtsam::Vector3 A{gtsam::Vector3::Zero()};
    double variance{0.0};
    double last_source{-INFINITY};
  };
  std::map<uint32_t, Integral> integrate_(double start, double end,
                                          bool &parked) const {
    std::map<uint32_t, Integral> result;
    auto it = samples_.upper_bound(start + kArticulatedTimestampToleranceSec);
    if (it == samples_.begin()) {
      parked = false;
      return result;
    }
    --it;
    for (const auto &leg : it->second.legs)
      result.emplace(leg.sensor_id, Integral{});
    std::map<uint32_t, std::map<double, std::pair<double, double>>>
        source_weights;
    parked = true;
    double t = start;
    while (t < end - 1e-10) {
      const auto next = std::next(it);
      const double until =
          next == samples_.end() ? end : std::min(end, next->first);
      const auto &sample = it->second;
      if (until - sample.stamp_sec > config_.maximum_sample_age_s + kArticulatedTimestampToleranceSec ||
          !sample.parked)
        parked = false;
      for (auto candidate = result.begin(); candidate != result.end();) {
        const auto leg = std::find_if(
            sample.legs.begin(), sample.legs.end(),
            [&](const auto &x) { return x.sensor_id == candidate->first; });
        if (until - sample.stamp_sec > config_.maximum_sample_age_s + kArticulatedTimestampToleranceSec ||
            leg == sample.legs.end() || leg->stamp_sec > t + kArticulatedTimestampToleranceSec ||
            until - leg->stamp_sec > config_.maximum_sample_age_s + kArticulatedTimestampToleranceSec) {
          candidate = result.erase(candidate);
          continue;
        }
        const double dt = until - t;
        candidate->second.z += dt * leg->vertical_rate_rad_s;
        candidate->second.A += dt * leg->gravity_sensor.normalized();
        candidate->second.last_source =
            std::max(candidate->second.last_source, leg->stamp_sec);
        auto &weight = source_weights[candidate->first][leg->stamp_sec];
        weight.first += dt;
        weight.second = leg->sigma_vertical_rate_rad_s;
        ++candidate;
      }
      t = until;
      if (t < end - 1e-10)
        it = next;
    }
    for (auto& item : result) {
      // One canonical raw sample can cover multiple graph intervals. If its
      // nonnegative integration weights are w_i, freshness bounds sum(w_i)
      // by maximum_sample_age_s plus comparison tolerance at both endpoints.
      // Weighted Cauchy gives sigma^2*w*w' <= diag(sigma^2*w_i*reuse_bound).
      // This admits causal held samples without counting them as independent
      // repeated observations and is valid for unequal graph interval lengths.
      for (const auto& w : source_weights.at(item.first)) {
        item.second.variance += w.second.first * std::pow(w.second.second, 2) *
                                (config_.maximum_sample_age_s + 2.0 * kArticulatedTimestampToleranceSec);
      }
    }
    return result;
  }
  void faultUnlocked_(const std::string &reason) {
    state_.valid = false;
    state_.fault_latched = true;
    state_.reason = reason;
  }
  ArticulatedTurnConfig config_;
  gtsam::Rot3 R_IC_;
  mutable std::mutex mutex_;
  std::map<double, ArticulatedTurnSample> samples_;
  std::map<uint32_t, Bias> biases_;
  std::optional<gtsam::Key> heading_key_;
  uint64_t bias_counter_{0}, heading_counter_{0}, last_key_{0};
  bool initialized_{false}, pending_valid_{false}, pending_parked_{false};
  int pending_leg_count_{0};
  double last_time_{0.0}, last_q_{0.0};
  gtsam::Pose3 last_pose_;
  ArticulatedTurnState state_;
};
} // namespace graph_msf
