// ultrasonic range finder
#ifndef PING_SENSOR_H
#define PING_SENSOR_H

#include <string>
#include <micro_ros_arduino.h>
#include <sensor_msgs/msg/range.h>

 struct PINGSensorConfiguration
 {
    int pingPin;
    float minimumRange;
    float maximumRange;
    double fieldOfView;
    const char* referenceFrameId;
 };


class PINGSensor
{
public:
    PINGSensor(const PINGSensorConfiguration& configuration);
    const PINGSensorConfiguration configuration;
    float getMeasurement() const;
};

void fillMeasurementMessage(const PINGSensor &ultraSonicSensor, sensor_msgs__msg__Range * msg);
#endif