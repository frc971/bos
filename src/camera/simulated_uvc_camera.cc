#include "src/camera/simulated_uvc_camera.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <thread>

#include "absl/flags/flag.h"
#include "absl/log/log.h"

ABSL_FLAG(std::optional<double>, simulated_uvc_start, std::nullopt,
          "Start timestamp relative to the first simulated UVC frame");
ABSL_FLAG(std::optional<double>, simulated_uvc_end, std::nullopt,
          "End timestamp relative to the first simulated UVC frame");

struct uvc_context {
  camera::test::MockUvcApi* mock;
};
struct uvc_device {
  camera::test::MockUvcApi* mock;
};
struct uvc_device_handle {
  camera::test::MockUvcApi* mock;
};

namespace {

using camera::test::MockUvcApi;

}  // namespace

namespace camera::test::internal {

thread_local MockUvcApi* constructing_mock = nullptr;

}  // namespace camera::test::internal

namespace camera::test {

MockUvcApi::MockUvcApi() {
  using testing::_;
  using testing::Invoke;
  using testing::Return;

  ON_CALL(*this, Init()).WillByDefault(Return(UVC_SUCCESS));
  ON_CALL(*this, FindDevice()).WillByDefault(Return(UVC_SUCCESS));
  ON_CALL(*this, Open()).WillByDefault(Return(UVC_SUCCESS));
  ON_CALL(*this, GetStreamControl(_, _, _, _))
      .WillByDefault(
          Invoke([](uvc_stream_ctrl_t* ctrl, int width, int height, int) {
            *ctrl = {};
            ctrl->dwMaxVideoFrameSize = width * height * 3;
            return UVC_SUCCESS;
          }));
  ON_CALL(*this, StartStreaming(_, _))
      .WillByDefault(Invoke([this](uvc_frame_callback_t* callback, void* user) {
        callback_ = callback;
        callback_user_ = user;
        return UVC_SUCCESS;
      }));
}

auto MockUvcApi::DeliverJpeg(std::vector<unsigned char>& bytes) -> bool {
  return Deliver(bytes.data(), bytes.size());
}

auto MockUvcApi::DeliverEmptyFrame() -> bool {
  unsigned char placeholder = 0;
  return Deliver(&placeholder, 0);
}

auto MockUvcApi::Deliver(void* data, std::size_t size) -> bool {
  if (callback_ == nullptr)
    return false;
  uvc_frame_t frame{};
  frame.data = data;
  frame.data_bytes = size;
  frame.frame_format = UVC_COLOR_FORMAT_MJPEG;
  frame.sequence = ++sequence_;
  callback_(&frame, callback_user_);
  return true;
}

SimulatedUvcCamera::SimulatedUvcCamera(
    const std::filesystem::path& image_folder,
    const camera_constant_t& constants, absl::Status& status,
    double replay_speed, bool fail_to_init)
    : replay_speed_(replay_speed) {
  const auto start = absl::GetFlag(FLAGS_simulated_uvc_start);
  const auto end = absl::GetFlag(FLAGS_simulated_uvc_end);
  if (replay_speed_ <= 0.0)
    throw std::invalid_argument("Replay speed must be positive");
  if (start.has_value() && end.has_value() && end.value() <= start.value())
    throw std::invalid_argument(
        "Replay end time must be greater than start time");
  for (const auto& entry : std::filesystem::directory_iterator(image_folder)) {
    if (entry.is_regular_file() && entry.path().extension() == ".jpg") {
      image_paths_.push_back(entry.path());
    }
  }
  std::sort(image_paths_.begin(), image_paths_.end(),
            [](const auto& lhs, const auto& rhs) {
              return std::stod(lhs.stem().string()) <
                     std::stod(rhs.stem().string());
            });

  if (!image_paths_.empty() && (start.has_value() || end.has_value())) {
    const double offset = std::stod(image_paths_.front().stem().string());
    std::erase_if(image_paths_, [&](const auto& path) {
      const double timestamp = std::stod(path.stem().string()) - offset;
      return (start.has_value() && timestamp < start.value()) ||
             (end.has_value() && timestamp > end.value());
    });
  }

  if (!fail_to_init) {
    internal::constructing_mock = &mock_uvc_;
  }
  camera_ = std::make_unique<UVCCamera>(constants, status);
  internal::constructing_mock = nullptr;
  camera_->frame_index_ = 0;
  camera_->previous_frame_index_ = 0;
}

SimulatedUvcCamera::~SimulatedUvcCamera() = default;

auto SimulatedUvcCamera::GetFrame() -> timestamped_frame_t {
  if (IsDone())
    return {.invalid = true};

  const auto outcome = SampleFrameOutcome();

  if (outcome == FrameOutcome::kDelayed) {
    LOG(INFO) << "Simulated UVC camera " << camera_->GetCameraConstant().name
              << " activated frame delay probability ("
              << failure_delay_.count() << " ms)";
    if (!next_frame_delay_) {
      PauseNextFrame(failure_delay_);
    }
  }

  if (next_frame_delay_) {
    std::this_thread::sleep_for(*next_frame_delay_);
    next_frame_delay_.reset();
  }

  if (outcome == FrameOutcome::kEmpty) {
    LOG(INFO) << "Simulated UVC camera " << camera_->GetCameraConstant().name
              << " activated empty-frame probability";
    if (!InjectEmptyFrame())
      return {.invalid = true};
    return camera_->GetFrame();
  }
  if (outcome == FrameOutcome::kCorrupt) {
    LOG(INFO) << "Simulated UVC camera " << camera_->GetCameraConstant().name
              << " activated corrupt-frame probability";
    if (!InjectCorruptFrame())
      return {.invalid = true};
    PaceReplay();
    return camera_->GetFrame();
  }

  auto bytes = ReadNextJpeg();
  PaceReplay();
  if (!mock_uvc_.DeliverJpeg(bytes))
    return {.invalid = true};
  return camera_->GetFrame();
}

auto SimulatedUvcCamera::Restart() -> void {
  camera_->Restart();
}

auto SimulatedUvcCamera::GetCameraConstant() const -> camera_constant_t {
  return camera_->GetCameraConstant();
}

auto SimulatedUvcCamera::IsDone() -> bool {
  return next_image_ >= image_paths_.size();
}

auto SimulatedUvcCamera::PauseNextFrame(std::chrono::milliseconds delay)
    -> void {
  next_frame_delay_ = delay;
}

auto SimulatedUvcCamera::InjectEmptyFrame() -> bool {
  return mock_uvc_.DeliverEmptyFrame();
}

auto SimulatedUvcCamera::InjectCorruptFrame() -> bool {
  if (IsDone())
    return false;
  auto bytes = ReadNextJpeg();
  bytes.resize(bytes.size() / 2);
  return mock_uvc_.DeliverJpeg(bytes);
}

auto SimulatedUvcCamera::SetFailureProbabilities(
    const FrameFailureProbabilities& probabilities, uint32_t seed) -> void {
  const double failure_probability =
      probabilities.frame_delay + probabilities.empty + probabilities.corrupt;
  const std::array outcome_probabilities = {
      1.0 - failure_probability, probabilities.frame_delay, probabilities.empty,
      probabilities.corrupt};
  const auto is_valid = [](double probability) {
    return std::isfinite(probability) && probability >= 0.0;
  };
  if (!std::all_of(outcome_probabilities.begin(), outcome_probabilities.end(),
                   is_valid) ||
      probabilities.delay < std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("Invalid frame failure probability table");
  }
  failure_delay_ = probabilities.delay;
  frame_outcome_distribution_ = std::discrete_distribution<std::size_t>(
      outcome_probabilities.begin(), outcome_probabilities.end());
  random_engine_.seed(seed);
}

auto SimulatedUvcCamera::SampleFrameOutcome() -> FrameOutcome {
  return static_cast<FrameOutcome>(frame_outcome_distribution_(random_engine_));
}

auto SimulatedUvcCamera::PaceReplay() const -> void {
  if (next_image_ >= image_paths_.size())
    return;
  const double current_timestamp =
      std::stod(image_paths_[next_image_ - 1].stem().string());
  const double next_timestamp =
      std::stod(image_paths_[next_image_].stem().string());
  const double delay_seconds =
      std::max(0.0, next_timestamp - current_timestamp) / replay_speed_;
  std::this_thread::sleep_for(std::chrono::duration<double>(delay_seconds));
}

auto SimulatedUvcCamera::ReadNextJpeg() -> std::vector<unsigned char> {
  std::ifstream input(image_paths_[next_image_++], std::ios::binary);
  if (!input)
    throw std::runtime_error("Unable to read simulated UVC frame");
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

}  // namespace camera::test
