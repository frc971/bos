#include "position_sender.h"
#include "frc/DataLogManager.h"
#include "src/localization/position.h"

namespace localization {

constexpr auto RadianToDegree(double radian) -> double {
  return radian * (180 / M_PI);
}

static const int kmax_tags = 50;

PositionSender::PositionSender(const std::string& camera_name, bool verbose)
    : instance_(nt::NetworkTableInstance::GetDefault()), verbose_(verbose) {
  std::shared_ptr<nt::NetworkTable> table =
      instance_.GetTable("Orin/PoseEstimate/" + camera_name);

  nt::StructTopic<frc::Pose2d> pose_topic =
      table->GetStructTopic<frc::Pose2d>("Pose");
  nt::StructTopic<frc::Pose3d> pose3d_topic =
      table->GetStructTopic<frc::Pose3d>("Pose3d");

  pose_publisher_ = pose_topic.Publish();
  pose3d_publisher_ = pose3d_topic.Publish();

  nt::DoubleTopic latency_topic = table->GetDoubleTopic("Latency");
  latency_publisher_ = latency_topic.Publish();

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

  nt::BooleanArrayTopic rejected_tag_ids_topic =
      table->GetBooleanArrayTopic("RejectedTagId");
  rejected_tag_ids_publisher_ = rejected_tag_ids_topic.Publish();
  nt::DoubleTopic loss_topic = table->GetDoubleTopic("Loss");
  loss_publisher_ = loss_topic.Publish();
}

void PositionSender::Send(
    const std::vector<localization::position_estimate_t>& detections,
    double latency, double loss) {
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

    loss_publisher_.Set(loss);

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
    num_tags_publisher_.Set(detection.num_tags);

    if (verbose_) {
      LOG(INFO) << detection;
    }
  }
  mutex_.unlock();
}
}  // namespace localization
