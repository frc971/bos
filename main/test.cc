#include <frc/apriltag/AprilTagFieldLayout.h>
#include "camera/camera.h"
#include <frc/apriltag/AprilTagFields.h>
#include <thread>
#include "localization/position_sender.h"
#include <fstream>
#include "wpilibc/frc/RuntimeType.h"
#include "apriltag/apriltag.h"
#include <iostream>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <opencv2/highgui.hpp>
#include <sstream>

using json = nlohmann::json;

int main() {
  frc::AprilTagFieldLayout layout(frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeAndyMark));
  std::cout << layout.GetTagPose(7)->Rotation().Z().value();
}

