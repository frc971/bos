#include "src/camera/simulated_uvc_camera.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>

namespace camera {
namespace {

auto NumericTimestamp(const std::filesystem::path& path) -> double {
  const std::string stem = path.stem().string();
  std::size_t consumed = 0;
  try {
    const double value = std::stod(stem, &consumed);
    return consumed == stem.size() ? value
                                   : std::numeric_limits<double>::infinity();
  } catch (const std::exception&) {
    return std::numeric_limits<double>::infinity();
  }
}

auto IsJpeg(const std::filesystem::path& path) -> bool {
  std::string extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char character) {
                   return static_cast<char>(std::tolower(character));
                 });
  return extension == ".jpg" || extension == ".jpeg";
}

auto ReadBytes(const std::filesystem::path& path,
               std::vector<unsigned char>* bytes) -> bool {
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input) return false;
  const auto length = input.tellg();
  if (length < 0) return false;
  bytes->resize(static_cast<std::size_t>(length));
  input.seekg(0);
  return bytes->empty() || static_cast<bool>(input.read(
                               reinterpret_cast<char*>(bytes->data()),
                               static_cast<std::streamsize>(bytes->size())));
}

}  // namespace

SimulatedUVCFrameSource::SimulatedUVCFrameSource(
    std::filesystem::path image_directory, absl::Status& status) {
  try {
    for (const auto& entry :
         std::filesystem::directory_iterator(image_directory)) {
      if (entry.is_regular_file() &&
          std::isfinite(NumericTimestamp(entry.path()))) {
        image_paths_.push_back(entry.path());
      }
    }
  } catch (const std::filesystem::filesystem_error& error) {
    status = absl::InvalidArgumentError(error.what());
    return;
  }
  std::sort(image_paths_.begin(), image_paths_.end(),
            [](const auto& left, const auto& right) {
              return NumericTimestamp(left) < NumericTimestamp(right);
            });
  if (image_paths_.empty()) {
    status = absl::NotFoundError("image directory contains no files");
    return;
  }
  status = absl::OkStatus();
}

auto SimulatedUVCFrameSource::empty() const -> bool {
  return cursor_ >= image_paths_.size();
}

auto SimulatedUVCFrameSource::size() const -> std::size_t {
  return image_paths_.size();
}

void SimulatedUVCFrameSource::Rewind() { cursor_ = 0; }

auto SimulatedUVCFrameSource::NextFrame(SimulatedUVCFrameFault fault)
    -> uvc_frame_t {
  if (empty()) return EmptyFrame();
  const auto path = image_paths_[cursor_++];
  const cv::Mat image = cv::imread(path.string(), cv::IMREAD_COLOR);
  encoded_frame_.clear();
  if (IsJpeg(path)) {
    (void)ReadBytes(path, &encoded_frame_);
  } else if (!image.empty()) {
    cv::imencode(".jpg", image, encoded_frame_);
  }
  if (fault == SimulatedUVCFrameFault::kEmpty) return EmptyFrame();
  if (fault == SimulatedUVCFrameFault::kCorruptJpeg &&
      encoded_frame_.size() > 3U) {
    encoded_frame_.resize(3U);
  }
  uvc_frame_t frame{};
  frame.data = encoded_frame_.data();
  frame.data_bytes = encoded_frame_.size();
  frame.width = image.cols;
  frame.height = image.rows;
  frame.frame_format =
      fault == SimulatedUVCFrameFault::kUnsupportedFormat
          ? static_cast<decltype(frame.frame_format)>(255)
          : UVC_COLOR_FORMAT_MJPEG;
  frame.sequence = sequence_++;
  return frame;
}

void SimulatedUVCFrameSource::DeliverNext(uvc_frame_callback_t* callback,
                                          void* callback_user,
                                          SimulatedUVCFrameFault fault) {
  uvc_frame_t frame = NextFrame(fault);
  callback(&frame, callback_user);
}

auto SimulatedUVCFrameSource::EmptyFrame() -> uvc_frame_t {
  encoded_frame_ = {0xff, 0xd8, 0xff};
  uvc_frame_t frame{};
  frame.data = encoded_frame_.data();
  frame.data_bytes = encoded_frame_.size();
  frame.frame_format = UVC_COLOR_FORMAT_MJPEG;
  frame.sequence = sequence_++;
  return frame;
}

namespace {

constexpr std::size_t kTransferBufferCount = 100;
constexpr std::size_t kUvcHeaderSize = 2;

enum class FaultKind : std::uint64_t {
  kContext,
  kDiscovery,
  kOpen,
  kNegotiation,
  kStreamingStart,
  kTimeout,
  kTemporaryStall,
  kPermanentStall,
  kOverflow,
  kTransferError,
  kNoDevice,
  kCorruption,
  kEmpty,
  kOversized,
  kUnsupported,
  kSyntheticInvalid,
  kDeliveryException,
  kFatalAbort,
  kStallDuration,
  kProcessingDuration,
};

auto Mix(std::uint64_t value) -> std::uint64_t {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

auto UnitRandom(std::uint64_t seed, std::uint64_t frame, FaultKind kind,
                std::uint64_t generation) -> double {
  const auto bits =
      Mix(seed ^ Mix(frame) ^ Mix(static_cast<std::uint64_t>(kind)) ^
          Mix(generation));
  return static_cast<double>(bits >> 11U) * (1.0 / 9007199254740992.0);
}

auto IsProbability(double value) -> bool {
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

auto ValidateRange(const SimulatedUVCDurationRange& range,
                   std::string_view name) -> absl::Status {
  if (!std::isfinite(range.min_seconds) || !std::isfinite(range.max_seconds) ||
      range.min_seconds < 0.0 || range.max_seconds < range.min_seconds) {
    return absl::InvalidArgumentError(std::string(name) +
                                      " must be a finite nonnegative range");
  }
  return absl::OkStatus();
}

auto ValidateConfig(const camera_constant_t& camera,
                    const SimulatedUVCCameraConfig& config) -> absl::Status {
  std::error_code path_error;
  const bool is_directory =
      std::filesystem::is_directory(config.image_directory, path_error);
  if (config.image_directory.empty() || path_error || !is_directory) {
    return absl::InvalidArgumentError("image_directory is not a directory");
  }
  if (!std::isfinite(config.replay_speed) || config.replay_speed <= 0.0) {
    return absl::InvalidArgumentError(
        "replay_speed must be finite and positive");
  }
  if (!config.random_seed) {
    return absl::InvalidArgumentError("random_seed is required");
  }
  if ((config.start && !std::isfinite(*config.start)) ||
      (config.end && !std::isfinite(*config.end)) ||
      (config.start && config.end && *config.end <= *config.start)) {
    return absl::InvalidArgumentError("start/end bounds are invalid");
  }
  if (!camera.frame_width || !camera.frame_height || !camera.fps ||
      *camera.frame_width == 0U || *camera.frame_height == 0U ||
      *camera.fps == 0U) {
    return absl::InvalidArgumentError(
        "camera width, height, and fps must be present and positive");
  }
  if (!camera.max_payload_size || !camera.max_frame_size ||
      *camera.max_payload_size <= kUvcHeaderSize ||
      *camera.max_frame_size == 0U) {
    return absl::InvalidArgumentError(
        "maximum payload and frame sizes must be present and positive");
  }

  const auto& init = config.initialization_faults;
  const auto& frame = config.frame_faults;
  const double probabilities[] = {
      init.context,
      init.discovery,
      init.open,
      init.negotiation,
      init.streaming_start,
      frame.timeout,
      frame.temporary_stall,
      frame.permanent_stall,
      frame.overflow,
      frame.transfer_error,
      frame.no_device,
      frame.corruption,
      frame.empty_frame,
      frame.oversized_frame,
      frame.unsupported_format,
      frame.synthetic_invalid_delivery,
      frame.delivery_exception,
      frame.fatal_abort,
  };
  for (double probability : probabilities) {
    if (!IsProbability(probability)) {
      return absl::InvalidArgumentError(
          "all injected-fault probabilities must be in [0, 1]");
    }
  }
  if (auto status = ValidateRange(config.temporary_stall, "temporary_stall");
      !status.ok()) {
    return status;
  }
  return ValidateRange(config.processing_delay, "processing_delay");
}

struct SourceFrame {
  std::filesystem::path path;
  double recorded_time = 0.0;
  std::uint64_t index = 0;
};

auto Enumerate(const SimulatedUVCCameraConfig& config,
               std::vector<SourceFrame>* frames) -> absl::Status {
  std::vector<std::pair<double, std::filesystem::path>> found;
  try {
    for (const auto& entry :
         std::filesystem::directory_iterator(config.image_directory)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      const std::string stem = entry.path().stem().string();
      std::size_t consumed = 0;
      double timestamp = 0.0;
      try {
        timestamp = std::stod(stem, &consumed);
      } catch (const std::exception&) {
        continue;
      }
      if (consumed == stem.size() && std::isfinite(timestamp)) {
        found.emplace_back(timestamp, entry.path());
      }
    }
  } catch (const std::filesystem::filesystem_error& error) {
    return absl::InvalidArgumentError("failed to enumerate image_directory: " +
                                      std::string(error.what()));
  }
  if (found.empty()) {
    return absl::NotFoundError(
        "image_directory has no numeric frame filenames");
  }
  std::sort(found.begin(), found.end(),
            [](const auto& left, const auto& right) {
              if (left.first != right.first) {
                return left.first < right.first;
              }
              return left.second.string() < right.second.string();
            });
  const double origin = found.front().first;
  std::uint64_t index = 0;
  for (const auto& [timestamp, path] : found) {
    const double normalized = timestamp - origin;
    if ((config.start && normalized < *config.start) ||
        (config.end && normalized > *config.end)) {
      ++index;
      continue;
    }
    frames->push_back({path, normalized, index++});
  }
  if (frames->empty()) {
    return absl::NotFoundError("no frames remain inside the requested bounds");
  }
  return absl::OkStatus();
}

auto ReadFile(const std::filesystem::path& path,
              std::vector<unsigned char>* out) -> bool {
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream) {
    return false;
  }
  const auto size = stream.tellg();
  if (size < 0) {
    return false;
  }
  out->resize(static_cast<std::size_t>(size));
  stream.seekg(0);
  return out->empty() || static_cast<bool>(stream.read(
                             reinterpret_cast<char*>(out->data()),
                             static_cast<std::streamsize>(out->size())));
}

}  // namespace

class SimulatedUVCCamera::State {
 public:
  State(camera_constant_t camera, SimulatedUVCCameraConfig config,
        std::vector<SourceFrame> frames)
      : camera_(std::move(camera)),
        config_(std::move(config)),
        frames_(std::move(frames)) {}

  ~State() { Stop(); }

  auto Initialize(std::uint64_t generation) -> absl::Status {
    generation_ = generation;
    const auto& faults = config_.initialization_faults;
    const std::pair<double, FaultKind> stages[] = {
        {faults.context, FaultKind::kContext},
        {faults.discovery, FaultKind::kDiscovery},
        {faults.open, FaultKind::kOpen},
        {faults.negotiation, FaultKind::kNegotiation},
        {faults.streaming_start, FaultKind::kStreamingStart},
    };
    constexpr std::string_view names[] = {"context", "device discovery",
                                          "device open", "negotiation",
                                          "streaming startup"};
    for (std::size_t i = 0; i < std::size(stages); ++i) {
      if (Decision(0, stages[i].second, stages[i].first)) {
        AddFault();
        done_.store(true);
        return absl::AbortedError("simulated UVC failure during " +
                                  std::string(names[i]));
      }
    }
    PrecomputeFrameDecisions();
    stop_.store(false);
    done_.store(false);
    producer_finished_ = false;
    event_finished_ = false;
    callback_finished_ = false;
    permanent_stall_ = false;
    try {
      producer_ = std::thread(&State::ProducerLoop, this);
      event_ = std::thread(&State::EventLoop, this);
      callback_ = std::thread(&State::CallbackLoop, this);
    } catch (const std::system_error& error) {
      Stop();
      done_.store(true);
      return absl::ResourceExhaustedError(
          "failed to create simulated UVC worker threads: " +
          std::string(error.what()));
    }
    return absl::OkStatus();
  }

  void Stop() {
    stop_.store(true);
    cv_.notify_all();
    delivered_cv_.notify_all();
    if (producer_.joinable())
      producer_.join();
    if (event_.joinable())
      event_.join();
    if (callback_.joinable())
      callback_.join();
    std::lock_guard lock(pipeline_mutex_);
    transfers_.clear();
    completed_.reset();
  }

  void Restart() {
    Stop();
    {
      std::lock_guard lock(stats_mutex_);
      ++stats_.restarts;
    }
    const absl::Status ignored = Initialize(generation_ + 1U);
    (void)ignored;
  }

  auto GetFrame() -> timestamped_frame_t {
    std::unique_lock wait_lock(wait_mutex_);
    delivered_cv_.wait(wait_lock, [this] {
      return delivered_sequence_.load() != consumed_sequence_.load() ||
             done_.load() || stop_.load();
    });
    wait_lock.unlock();

    std::lock_guard frame_lock(frame_mutex_);
    const std::uint64_t sequence = delivered_sequence_.load();
    const std::uint64_t consumed = consumed_sequence_.load();
    if (sequence == consumed) {
      return {.timestamp = Now(), .invalid = true};
    }
    if (consumed != 0U && sequence > consumed + 1U) {
      std::lock_guard stats_lock(stats_mutex_);
      stats_.sequence_gaps += sequence - consumed - 1U;
    }
    consumed_sequence_.store(sequence);
    if (delivered_exception_) {
      delivered_exception_ = false;
      throw std::runtime_error("simulated UVC frame delivery exception");
    }
    timestamped_frame_t result;
    delivered_frame_.frame.copyTo(result.frame);
    result.timestamp = delivered_frame_.timestamp;
    result.invalid = delivered_frame_.invalid;
    {
      std::lock_guard stats_lock(stats_mutex_);
      ++stats_.deliveries;
    }
    return result;
  }

  auto IsDone() const -> bool { return done_.load(); }

  auto Statistics() const -> SimulatedUVCCameraStatistics {
    std::lock_guard lock(stats_mutex_);
    return stats_;
  }

 private:
  struct Transfer {
    std::vector<unsigned char> bytes;
    std::uint64_t source_index = 0;
    bool fid = false;
    bool eof = false;
    bool unsupported = false;
    bool synthetic_invalid = false;
    bool delivery_exception = false;
    double processing_delay_seconds = 0.0;
  };

  struct CompletedFrame {
    std::vector<unsigned char> bytes;
    std::uint64_t sequence = 0;
    std::uint64_t source_index = 0;
    bool unsupported = false;
    bool synthetic_invalid = false;
    bool delivery_exception = false;
    double processing_delay_seconds = 0.0;
  };

  struct FrameDecisions {
    bool timeout = false;
    bool temporary_stall = false;
    bool permanent_stall = false;
    bool overflow = false;
    bool transfer_error = false;
    bool no_device = false;
    bool corruption = false;
    bool empty = false;
    bool oversized = false;
    bool unsupported = false;
    bool synthetic_invalid = false;
    bool delivery_exception = false;
    bool fatal_abort = false;
    std::chrono::duration<double> stall_duration{0.0};
    std::chrono::duration<double> processing_duration{0.0};
  };

  auto Decision(std::uint64_t frame, FaultKind kind, double probability) const
      -> bool {
    return UnitRandom(*config_.random_seed, frame, kind, generation_) <
           probability;
  }

  auto Duration(std::uint64_t frame, FaultKind kind,
                const SimulatedUVCDurationRange& range) const
      -> std::chrono::duration<double> {
    const double fraction =
        UnitRandom(*config_.random_seed, frame, kind, generation_);
    return std::chrono::duration<double>(
        range.min_seconds + fraction * (range.max_seconds - range.min_seconds));
  }

  void PrecomputeFrameDecisions() {
    frame_decisions_.clear();
    frame_decisions_.reserve(frames_.size());
    const auto& faults = config_.frame_faults;
    for (const SourceFrame& frame : frames_) {
      frame_decisions_.push_back(FrameDecisions{
          .timeout = Decision(frame.index, FaultKind::kTimeout, faults.timeout),
          .temporary_stall = Decision(frame.index, FaultKind::kTemporaryStall,
                                      faults.temporary_stall),
          .permanent_stall = Decision(frame.index, FaultKind::kPermanentStall,
                                      faults.permanent_stall),
          .overflow =
              Decision(frame.index, FaultKind::kOverflow, faults.overflow),
          .transfer_error = Decision(frame.index, FaultKind::kTransferError,
                                     faults.transfer_error),
          .no_device =
              Decision(frame.index, FaultKind::kNoDevice, faults.no_device),
          .corruption =
              Decision(frame.index, FaultKind::kCorruption, faults.corruption),
          .empty = Decision(frame.index, FaultKind::kEmpty, faults.empty_frame),
          .oversized = Decision(frame.index, FaultKind::kOversized,
                                faults.oversized_frame),
          .unsupported = Decision(frame.index, FaultKind::kUnsupported,
                                  faults.unsupported_format),
          .synthetic_invalid =
              Decision(frame.index, FaultKind::kSyntheticInvalid,
                       faults.synthetic_invalid_delivery),
          .delivery_exception =
              Decision(frame.index, FaultKind::kDeliveryException,
                       faults.delivery_exception),
          .fatal_abort =
              Decision(frame.index, FaultKind::kFatalAbort, faults.fatal_abort),
          .stall_duration = Duration(frame.index, FaultKind::kStallDuration,
                                     config_.temporary_stall),
          .processing_duration =
              Duration(frame.index, FaultKind::kProcessingDuration,
                       config_.processing_delay),
      });
    }
  }

  auto StopAwareWait(std::chrono::duration<double> duration) -> bool {
    std::unique_lock lock(pipeline_mutex_);
    return cv_.wait_for(lock, duration, [this] { return stop_.load(); });
  }

  void AddFault() {
    std::lock_guard lock(stats_mutex_);
    ++stats_.injected_faults;
  }

  void ProducerLoop() {
    double previous_time = cursor_ == 0U ? frames_.front().recorded_time
                                         : frames_[cursor_ - 1U].recorded_time;
    while (!stop_.load() && cursor_ < frames_.size()) {
      const SourceFrame source = frames_[cursor_];
      const FrameDecisions decisions = frame_decisions_[cursor_++];
      const double delta = source.recorded_time - previous_time;
      previous_time = source.recorded_time;
      if (delta > 0.0 && StopAwareWait(std::chrono::duration<double>(
                             delta / config_.replay_speed))) {
        break;
      }
      {
        std::lock_guard lock(stats_mutex_);
        ++stats_.source_frames;
      }
      if (decisions.fatal_abort) {
        AddFault();
        std::abort();
      }
      if (decisions.permanent_stall) {
        AddFault();
        permanent_stall_ = true;
        std::unique_lock lock(pipeline_mutex_);
        cv_.wait(lock, [this] { return stop_.load(); });
        break;
      }
      if (decisions.temporary_stall) {
        AddFault();
        if (StopAwareWait(decisions.stall_duration)) {
          break;
        }
      }
      if (decisions.no_device) {
        AddFault();
        break;
      }
      if (decisions.timeout) {
        AddFault();
        {
          std::lock_guard lock(stats_mutex_);
          ++stats_.retries;
        }
        if (StopAwareWait(std::chrono::duration<double>(1.0 / *camera_.fps))) {
          break;
        }
      }
      if (decisions.overflow || decisions.transfer_error) {
        if (decisions.overflow)
          AddFault();
        if (decisions.transfer_error)
          AddFault();
        std::lock_guard lock(stats_mutex_);
        ++stats_.retired_buffers;
        continue;
      }

      std::vector<unsigned char> bytes;
      if (!ReadFile(source.path, &bytes)) {
        std::lock_guard lock(stats_mutex_);
        ++stats_.retired_buffers;
        continue;
      }
      const bool empty = decisions.empty;
      const bool corrupt = decisions.corruption;
      const bool oversized = decisions.oversized;
      const bool unsupported = decisions.unsupported;
      const bool synthetic = decisions.synthetic_invalid;
      const bool delivery_exception = decisions.delivery_exception;
      for (bool injected : {empty, corrupt, oversized, unsupported, synthetic,
                            delivery_exception}) {
        if (injected)
          AddFault();
      }
      if (empty)
        bytes.clear();
      if (corrupt && !bytes.empty())
        bytes.resize(bytes.size() / 3U);
      if (oversized)
        bytes.resize(static_cast<std::size_t>(*camera_.max_frame_size) + 1U,
                     0xa5U);

      const std::size_t packet_capacity =
          static_cast<std::size_t>(*camera_.max_payload_size) - kUvcHeaderSize;
      const std::size_t packet_count = std::max<std::size_t>(
          1U, (bytes.size() + packet_capacity - 1U) / packet_capacity);
      const bool fid = (source.index & 1U) != 0U;
      for (std::size_t packet = 0; packet < packet_count; ++packet) {
        const std::size_t begin = packet * packet_capacity;
        const std::size_t end = std::min(bytes.size(), begin + packet_capacity);
        Transfer transfer;
        if (begin < end) {
          transfer.bytes.assign(
              bytes.begin() + static_cast<std::ptrdiff_t>(begin),
              bytes.begin() + static_cast<std::ptrdiff_t>(end));
        }
        transfer.source_index = source.index;
        transfer.fid = fid;
        transfer.eof = packet + 1U == packet_count;
        transfer.unsupported = unsupported;
        transfer.synthetic_invalid = synthetic;
        transfer.delivery_exception = delivery_exception;
        transfer.processing_delay_seconds =
            decisions.processing_duration.count();
        std::unique_lock lock(pipeline_mutex_);
        cv_.wait(lock, [this] {
          return stop_.load() || transfers_.size() < kTransferBufferCount;
        });
        if (stop_.load())
          break;
        transfers_.push_back(std::move(transfer));
        {
          std::lock_guard stats_lock(stats_mutex_);
          ++stats_.transfers;
        }
        lock.unlock();
        cv_.notify_all();
      }
    }
    {
      std::lock_guard lock(pipeline_mutex_);
      producer_finished_ = true;
    }
    cv_.notify_all();
  }

  void EventLoop() {
    std::vector<unsigned char> assembly;
    std::optional<bool> assembly_fid;
    bool oversized = false;
    while (true) {
      Transfer transfer;
      {
        std::unique_lock lock(pipeline_mutex_);
        cv_.wait(lock, [this] {
          return stop_.load() || !transfers_.empty() || producer_finished_;
        });
        if (stop_.load())
          break;
        if (transfers_.empty() && producer_finished_)
          break;
        transfer = std::move(transfers_.front());
        transfers_.pop_front();
      }
      cv_.notify_all();
      if (assembly_fid && *assembly_fid != transfer.fid) {
        assembly.clear();
        oversized = false;
      }
      assembly_fid = transfer.fid;
      if (assembly.size() + transfer.bytes.size() > *camera_.max_frame_size) {
        oversized = true;
      } else if (!oversized) {
        assembly.insert(assembly.end(), transfer.bytes.begin(),
                        transfer.bytes.end());
      }
      if (!transfer.eof)
        continue;
      if (!oversized) {
        CompletedFrame completed{assembly,
                                 ++assembled_sequence_,
                                 transfer.source_index,
                                 transfer.unsupported,
                                 transfer.synthetic_invalid,
                                 transfer.delivery_exception,
                                 transfer.processing_delay_seconds};
        std::lock_guard lock(pipeline_mutex_);
        if (completed_ || callback_busy_) {
          std::lock_guard stats_lock(stats_mutex_);
          ++stats_.hold_buffer_overwrites;
        }
        completed_ = std::move(completed);
        {
          std::lock_guard stats_lock(stats_mutex_);
          ++stats_.assembled_frames;
        }
      } else {
        std::lock_guard lock(stats_mutex_);
        ++stats_.retired_buffers;
      }
      assembly.clear();
      assembly_fid.reset();
      oversized = false;
      cv_.notify_all();
    }
    {
      std::lock_guard lock(pipeline_mutex_);
      event_finished_ = true;
    }
    cv_.notify_all();
  }

  void CallbackLoop() {
    while (true) {
      CompletedFrame completed;
      {
        std::unique_lock lock(pipeline_mutex_);
        cv_.wait(lock, [this] {
          return stop_.load() || completed_.has_value() || event_finished_;
        });
        if (stop_.load())
          break;
        if (!completed_ && event_finished_)
          break;
        completed = std::move(*completed_);
        completed_.reset();
        callback_busy_ = true;
      }
      if (StopAwareWait(std::chrono::duration<double>(
              completed.processing_delay_seconds))) {
        FinishCallback();
        break;
      }

      std::unique_lock frame_lock(frame_mutex_, std::try_to_lock);
      if (!frame_lock.owns_lock()) {
        std::lock_guard stats_lock(stats_mutex_);
        ++stats_.callback_drops;
      } else {
        timestamped_frame_t decoded;
        decoded.timestamp = Now();
        decoded.invalid = completed.synthetic_invalid;
        if (!completed.synthetic_invalid && !completed.unsupported &&
            !completed.bytes.empty()) {
          decoded.frame = cv::imdecode(completed.bytes, cv::IMREAD_GRAYSCALE);
        }
        if (completed.synthetic_invalid) {
          decoded.frame.release();
        } else if (completed.unsupported || decoded.frame.empty()) {
          std::lock_guard stats_lock(stats_mutex_);
          ++stats_.decode_failures;
          frame_lock.unlock();
          FinishCallback();
          continue;
        }
        delivered_frame_ = std::move(decoded);
        delivered_exception_ = completed.delivery_exception;
        delivered_sequence_.store(completed.sequence);
        frame_lock.unlock();
        delivered_cv_.notify_all();
      }
      FinishCallback();
    }
    {
      std::lock_guard lock(pipeline_mutex_);
      callback_busy_ = false;
      callback_finished_ = true;
    }
    if (!stop_.load()) {
      {
        std::lock_guard stats_lock(stats_mutex_);
        ++stats_.eof;
      }
      done_.store(true);
    }
    delivered_cv_.notify_all();
  }

  void FinishCallback() {
    {
      std::lock_guard lock(pipeline_mutex_);
      callback_busy_ = false;
    }
    cv_.notify_all();
  }

  static auto Now() -> double {
    return frc::Timer::GetFPGATimestamp().to<double>();
  }

  const camera_constant_t camera_;
  const SimulatedUVCCameraConfig config_;
  const std::vector<SourceFrame> frames_;
  std::vector<FrameDecisions> frame_decisions_;
  std::size_t cursor_ = 0;
  std::uint64_t generation_ = 0;

  mutable std::mutex stats_mutex_;
  SimulatedUVCCameraStatistics stats_;

  std::mutex pipeline_mutex_;
  std::condition_variable cv_;
  std::deque<Transfer> transfers_;
  std::optional<CompletedFrame> completed_;
  bool producer_finished_ = false;
  bool event_finished_ = false;
  bool callback_finished_ = false;
  bool callback_busy_ = false;
  bool permanent_stall_ = false;
  std::uint64_t assembled_sequence_ = 0;

  std::mutex frame_mutex_;
  timestamped_frame_t delivered_frame_;
  bool delivered_exception_ = false;
  std::atomic<std::uint64_t> delivered_sequence_{0};
  std::atomic<std::uint64_t> consumed_sequence_{0};

  std::mutex wait_mutex_;
  std::condition_variable delivered_cv_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> done_{true};
  std::thread producer_;
  std::thread event_;
  std::thread callback_;
};

SimulatedUVCCamera::SimulatedUVCCamera(const camera_constant_t& camera_constant,
                                       SimulatedUVCCameraConfig config,
                                       absl::Status& status)
    : camera_constant_(camera_constant), config_(std::move(config)) {
  status = ValidateConfig(camera_constant_, config_);
  if (!status.ok())
    return;
  std::vector<SourceFrame> frames;
  status = Enumerate(config_, &frames);
  if (!status.ok())
    return;
  state_ =
      std::make_unique<State>(camera_constant_, config_, std::move(frames));
  status = state_->Initialize(0);
}

SimulatedUVCCamera::~SimulatedUVCCamera() = default;

auto SimulatedUVCCamera::GetFrame() -> timestamped_frame_t {
  if (!state_)
    return {.invalid = true};
  return state_->GetFrame();
}

void SimulatedUVCCamera::Restart() {
  if (state_)
    state_->Restart();
}

auto SimulatedUVCCamera::IsDone() -> bool {
  return !state_ || state_->IsDone();
}

auto SimulatedUVCCamera::GetCameraConstant() const -> camera_constant_t {
  return camera_constant_;
}

auto SimulatedUVCCamera::GetStatistics() const -> SimulatedUVCCameraStatistics {
  return state_ ? state_->Statistics() : SimulatedUVCCameraStatistics{};
}

}  // namespace camera
