#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <sstream>
#include <thread>
#include "apriltag/apriltag.h"
#include "camera/camera.h"
#include "localization/position_sender.h"
#include "wpilibc/frc/RuntimeType.h"

using json = nlohmann::json;

int main() {}
