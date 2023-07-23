#include <iostream>
#include <vector>
#include <memory>

using namespace std;

enum class SensorType {
  UNKNOWN_SENSOR_TYPE = -1,
  VELODYNE_64 = 0,
  VELODYNE_32 = 1,
  VELODYNE_16 = 2,
  LDLIDAR_4 = 3,
  LDLIDAR_1 = 4,
  SHORT_RANGE_RADAR = 5,
  LONG_RANGE_RADAR = 6,
  MONOCULAR_CAMERA = 7,
  STEREO_CAMERA = 8,
  ULTRASONIC = 9,
  VELODYNE_128 = 10,
  SENSOR_TYPE_NUM
};

enum class SensorOrientation {
  FRONT = 0,
  LEFT_FORWARD = 1,
  LEFT = 2,
  LEFT_BACKWARD = 3,
  REAR = 4,
  RIGHT_BACKWARD = 5,
  RIGHT = 6,
  RIGHT_FORWARD = 7,
  PANORAMIC = 8
};

int main()
{
    string name = "UNKNONW_SENSOR";
    SensorType type = SensorType::UNKNOWN_SENSOR_TYPE;
    SensorOrientation orientation = SensorOrientation::FRONT;
    std::string frame_id = "UNKNOWN_FRAME_ID";

     cout << static_cast<int>(type) << endl;
}

