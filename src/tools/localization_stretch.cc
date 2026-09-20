#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

#include "src/camera/camera.h"
#include "src/localization/opencv_apriltag_detector.h"

ABSL_FLAG(std::string, image_folder, "",
          "Folder containing one sequence of image frames");  // NOLINT
ABSL_FLAG(
    std::string, intrinsics, "constants/gamepiece/intrinsics.json",
    "Camera intrinsics JSON used to initialize the CPU detector");  // NOLINT
ABSL_FLAG(double, max_gap, 3.0,
          "Maximum time in seconds allowed between detections");  // NOLINT
ABSL_FLAG(
    std::size_t, progress_interval, 100,
    "Print progress every N frames; zero disables progress output");  // NOLINT

namespace {

struct FramePath {
  std::filesystem::path path;
  double timestamp;
};

struct Stretch {
  FramePath start;
  FramePath end;
  std::size_t span_frames = 0;
  std::size_t detection_frames = 0;
  bool valid = false;

  [[nodiscard]] auto Duration() const -> double {
    return valid ? end.timestamp - start.timestamp : 0.0;
  }
};

auto Lowercase(std::string value) -> std::string {
  std::ranges::transform(value, value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

auto IsImage(const std::filesystem::path& path) -> bool {
  static constexpr std::string_view kExtensions[] = {
      ".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"};
  const std::string extension = Lowercase(path.extension().string());
  return std::ranges::find(kExtensions, extension) != std::end(kExtensions);
}

auto ParseNumericStem(const std::filesystem::path& path)
    -> std::optional<double> {
  const std::string stem = path.stem().string();
  std::size_t parsed_characters = 0;
  try {
    const double value = std::stod(stem, &parsed_characters);
    if (parsed_characters == stem.size() && std::isfinite(value)) {
      return value;
    }
  } catch (const std::exception&) {
    // Timestamped frame filenames must have numeric stems.
  }
  return std::nullopt;
}

auto FindFrames(const std::filesystem::path& folder) -> std::vector<FramePath> {
  if (!std::filesystem::is_directory(folder)) {
    throw std::runtime_error("image folder is not a directory: " +
                             folder.string());
  }

  std::vector<FramePath> frames;
  for (const auto& entry : std::filesystem::directory_iterator(folder)) {
    if (entry.is_regular_file() && IsImage(entry.path())) {
      const auto timestamp = ParseNumericStem(entry.path());
      if (!timestamp.has_value()) {
        throw std::runtime_error(
            "frame filename stem is not a numeric timestamp: " +
            entry.path().string());
      }
      frames.push_back({.path = entry.path(), .timestamp = timestamp.value()});
    }
  }

  std::ranges::sort(frames, [](const FramePath& left, const FramePath& right) {
    if (left.timestamp != right.timestamp) {
      return left.timestamp < right.timestamp;
    }
    return left.path.filename().string() < right.path.filename().string();
  });
  return frames;
}

auto ReadJson(const std::filesystem::path& path) -> nlohmann::json {
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("could not open intrinsics file: " +
                             path.string());
  }
  nlohmann::json value;
  stream >> value;
  return value;
}

auto IsBetter(const Stretch& candidate, const Stretch& best) -> bool {
  return candidate.valid &&
         (!best.valid || candidate.Duration() > best.Duration() ||
          (candidate.Duration() == best.Duration() &&
           candidate.detection_frames > best.detection_frames));
}

auto PrintFrame(const char* label, const FramePath& frame) -> void {
  std::cout << "  " << label << ": " << frame.path.filename().string()
            << " (timestamp " << std::setprecision(12) << frame.timestamp
            << ")\n";
}

auto Run(const std::filesystem::path& folder,
         const std::filesystem::path& intrinsics_path, double max_gap,
         std::size_t progress_interval) -> int {
  const std::vector<FramePath> frames = FindFrames(folder);
  if (frames.empty()) {
    throw std::runtime_error("no supported image frames found in: " +
                             folder.string());
  }
  if (!std::isfinite(max_gap) || max_gap < 0.0) {
    throw std::runtime_error("max_gap must be a finite, nonnegative duration");
  }

  const nlohmann::json intrinsics = ReadJson(intrinsics_path);
  std::unique_ptr<localization::OpenCVAprilTagDetector> detector;
  cv::Size frame_size;

  Stretch current;
  Stretch best;
  std::size_t pending_gap_frames = 0;
  std::size_t frames_with_tags = 0;
  std::size_t unreadable_frames = 0;
  std::size_t processed_frames = 0;

  const auto finish_current = [&current, &best, &pending_gap_frames]() {
    if (IsBetter(current, best)) {
      best = current;
    }
    current = {};
    pending_gap_frames = 0;
  };

  for (const auto& frame : frames) {
    ++processed_frames;
    cv::Mat image = cv::imread(frame.path.string(), cv::IMREAD_GRAYSCALE);
    bool found_tag = false;

    if (image.empty()) {
      ++unreadable_frames;
      std::cerr << "Warning: could not decode " << frame.path << '\n';
    } else {
      if (!detector) {
        frame_size = image.size();
        detector = std::make_unique<localization::OpenCVAprilTagDetector>(
            static_cast<unsigned int>(image.cols),
            static_cast<unsigned int>(image.rows), intrinsics);
      } else if (image.size() != frame_size) {
        throw std::runtime_error(
            "frame dimensions changed at " + frame.path.string() +
            ": expected " + std::to_string(frame_size.width) + "x" +
            std::to_string(frame_size.height) + ", got " +
            std::to_string(image.cols) + "x" + std::to_string(image.rows));
      }

      if (!image.isContinuous()) {
        image = image.clone();
      }
      camera::timestamped_frame_t timestamped_frame{
          .frame = image, .timestamp = frame.timestamp};
      found_tag = !detector->GetTagDetections(timestamped_frame).empty();
    }

    if (current.valid && frame.timestamp - current.end.timestamp > max_gap) {
      finish_current();
    }

    if (found_tag) {
      ++frames_with_tags;
      if (!current.valid) {
        current = {.start = frame,
                   .end = frame,
                   .span_frames = 1,
                   .detection_frames = 1,
                   .valid = true};
      } else {
        current.end = frame;
        current.span_frames += pending_gap_frames + 1;
        ++current.detection_frames;
      }
      pending_gap_frames = 0;
    } else if (current.valid) {
      ++pending_gap_frames;
    }

    if (progress_interval != 0 && (processed_frames % progress_interval == 0 ||
                                   processed_frames == frames.size())) {
      std::cerr << "Processed " << processed_frames << '/' << frames.size()
                << " frames; tag found in " << frames_with_tags << '\n';
    }
  }
  finish_current();

  std::cout << "Frames scanned: " << frames.size() << '\n'
            << "Frames with tags: " << frames_with_tags << '\n'
            << "Unreadable frames: " << unreadable_frames << '\n'
            << "Allowed detection gap: " << max_gap << " seconds\n";
  if (!best.valid) {
    std::cout << "Longest localization stretch: none\n";
    return 0;
  }

  std::cout << "Longest localization stretch:\n";
  PrintFrame("start", best.start);
  PrintFrame("end", best.end);
  std::cout << "  duration: " << best.Duration() << " seconds\n"
            << "  span frames: " << best.span_frames << '\n'
            << "  frames with tags: " << best.detection_frames << '\n'
            << "  gap frames inside span: "
            << best.span_frames - best.detection_frames << '\n';
  return 0;
}

}  // namespace

auto main(int argc, char* argv[]) -> int {
  absl::ParseCommandLine(argc, argv);
  if (absl::GetFlag(FLAGS_image_folder).empty()) {
    std::cerr
        << "Usage: " << argv[0]
        << " --image_folder=PATH [--intrinsics=PATH] [--max_gap=SECONDS]\n";
    return 2;
  }

  try {
    return Run(absl::GetFlag(FLAGS_image_folder),
               absl::GetFlag(FLAGS_intrinsics), absl::GetFlag(FLAGS_max_gap),
               absl::GetFlag(FLAGS_progress_interval));
  } catch (const std::exception& exception) {
    std::cerr << "localization_stretch: " << exception.what() << '\n';
    return 1;
  }
}
