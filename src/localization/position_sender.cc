#include "position_sender.h"
#include "frc/DataLogManager.h"
#include "src/localization/position.h"

namespace localization {

constexpr auto RadianToDegree(double radian) -> double {
  return radian * (180 / M_PI);
}

static const int kmax_tags = 50;

PositionSender::PositionSender(const std::string& camera_name, bool verbose,
                               bool sim)
    : instance_(nt::NetworkTableInstance::GetDefault()), verbose_(verbose) {
  std::shared_ptr<nt::NetworkTable> table =
      instance_.GetTable("Orin/PoseEstimate/" + camera_name);

  nt::StructTopic<frc::Pose2d> pose_topic =
      table->GetStructTopic<frc::Pose2d>("Pose");
  nt::StructTopic<frc::Pose3d> pose3d_topic =
      table->GetStructTopic<frc::Pose3d>("Pose3d");
  nt::StructArrayTopic<frc::Pose3d> all_estimates_topic =
      table->GetStructArrayTopic<frc::Pose3d>("All Estimates");

  pose_publisher_ = pose_topic.Publish();
  pose3d_publisher_ = pose3d_topic.Publish();
  all_estimates_publisher_ = all_estimates_topic.Publish();

  nt::DoubleTopic latency_topic = table->GetDoubleTopic("Latency");
  latency_publisher_ = latency_topic.Publish();

  nt::DoubleTopic timestamp_topic_ = table->GetDoubleTopic("Timestamp");
  timestamp_publisher_ = timestamp_topic_.Publish();

  nt::IntegerTopic num_tags_topic = table->GetIntegerTopic("NumTags");
  num_tags_publisher_ = num_tags_topic.Publish();

  nt::DoubleArrayTopic tag_estimation_topic =
      table->GetDoubleArrayTopic("TagEstimation");
  tag_estimation_publisher_ = tag_estimation_topic.Publish(
      {.pollStorage = 200, .sendAll = true, .keepDuplicates = true});

  nt::BooleanArrayTopic tag_ids_topic = table->GetBooleanArrayTopic("TagId");
  tag_ids_publisher_ = tag_ids_topic.Publish();

  nt::DoubleTopic varience_topic = table->GetDoubleTopic("Varience");
  varience_publisher_ = varience_topic.Publish();

  nt::DoubleTopic loss_topic = table->GetDoubleTopic("Loss");
  loss_publisher_ = loss_topic.Publish();

  nt::BooleanArrayTopic rejected_tag_ids_topic =
      table->GetBooleanArrayTopic("RejectedTagId");
  rejected_tag_ids_publisher_ = rejected_tag_ids_topic.Publish();

  if (sim) {
    std::error_code ec;
    log_.emplace("/bos/logs/sim.wpilog", ec);
    if (ec) {
      std::cerr << "Failed to open log: " << ec.message() << '\n';
      std::exit(0);
    }
    pose3d_log_.emplace(*log_, "Pose3d");
    all_estimates_log_.emplace(*log_, "AllEstimates");

    latency_log_.emplace(*log_, "Latency");
    timestamp_log_.emplace(*log_, "Timestamp");
    num_tags_log_.emplace(*log_, "NumTags");

    varience_log_.emplace(*log_, "Varience");
    loss_log_.emplace(*log_, "Loss");

    tag_estimation_log_.emplace(*log_, "TagEstimation");
    tag_ids_log_.emplace(*log_, "TagIds");
    rejected_tag_ids_log_.emplace(*log_, "RejectedTagIds");
  }
}

void PositionSender::Send(
    const std::vector<localization::position_estimate_t>& detections,
    double latency,
    const std::optional<std::vector<frc::Pose3d>>& all_estimates) {
  mutex_.lock();
  for (auto& detection : detections) {
    std::array<double, 8> tag_estimation{
        detection.pose.X().value(),
        detection.pose.Y().value(),
        detection.pose.Rotation().Z().value(),
        detection.variance,
        detection.timestamp +
            instance_.GetServerTimeOffset().value_or(0) / 1000000.0,
        static_cast<double>(detection.num_tags),
        latency,
        detection.avg_tag_dist};

    std::array<int, kmax_tags> tags{};
    for (int tag_id : detection.tag_ids) {
      tags[tag_id] = true;
    }

    std::array<int, kmax_tags> rejected_tags{};
    for (int tag_id : detection.rejected_tag_ids) {
      rejected_tags[tag_id] = true;
    }

    pose_publisher_.Set(
        frc::Pose2d(units::meter_t{detection.pose.X().value()},
                    units::meter_t{detection.pose.Y().value()},
                    units::radian_t{detection.pose.Rotation().Z().value()}));
    pose3d_publisher_.Set(detection.pose);

    tag_estimation_publisher_.Set(tag_estimation);

    tag_ids_publisher_.Set(tags);
    rejected_tag_ids_publisher_.Set(rejected_tags);
    varience_publisher_.Set(detection.variance);

    latency_publisher_.Set(latency);
    timestamp_publisher_.Set(detection.timestamp);
    num_tags_publisher_.Set(detection.num_tags);
    loss_publisher_.Set(detection.loss);
    if (all_estimates.has_value()) {
      all_estimates_publisher_.Set(all_estimates.value());
    }

    if (log_) {
      double adjusted_timestamp =
          detection.timestamp +
          instance_.GetServerTimeOffset().value_or(0) / 1e6;
      auto log_time = static_cast<int64_t>(adjusted_timestamp * 1e6);

      pose3d_log_->Append(detection.pose, log_time);

      tag_estimation_log_->Append(tag_estimation, log_time);
      tag_ids_log_->Append(tags, log_time);
      rejected_tag_ids_log_->Append(rejected_tags, log_time);

      varience_log_->Append(detection.variance, log_time);
      latency_log_->Append(latency, log_time);
      timestamp_log_->Append(detection.timestamp, log_time);
      num_tags_log_->Append(detection.num_tags, log_time);
      loss_log_->Append(detection.loss, log_time);

      if (all_estimates.has_value()) {
        all_estimates_log_->Append(all_estimates.value(), log_time);
      }
      log_->Flush();
    }

    if (verbose_) {
      LOG(INFO) << detection;
    }
  }
  mutex_.unlock();
}
}  // namespace localization
