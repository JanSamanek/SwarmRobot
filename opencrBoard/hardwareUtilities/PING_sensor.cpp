#include "PING_sensor.h"
#include "Arduino.h"
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>

PINGSensor::PINGSensor(const PINGSensorConfiguration& configuration)
:
configuration(configuration)
{

}

float PINGSensor::getMeasurement() const
{
    int pingPin = configuration.pingPin;
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin, LOW);

    pinMode(pingPin, INPUT);
    int64_t duration = pulseIn(pingPin, HIGH);
  
    float distance = duration * 0.00034 / 2;
    return distance;
}

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

sensor_msgs__msg__Range generateMeasurementMessage(const PINGSensor &ultraSonicSensor)
{
    struct timespec tv = {0};
    clock_gettime(0, &tv);

    float distance = ultraSonicSensor.getMeasurement();

    sensor_msgs__msg__Range msg;

    msg.header.frame_id = micro_ros_string_utilities_set(msg.header.frame_id, ultraSonicSensor.configuration.referenceFrameId);
    msg.header.stamp.sec = tv.tv_sec;
    msg.header.stamp.nanosec = tv.tv_nsec;
    msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    msg.field_of_view = ultraSonicSensor.configuration.fieldOfView * (M_PI / 180);
    msg.min_range = ultraSonicSensor.configuration.minimumRange;
    msg.max_range = ultraSonicSensor.configuration.maximumRange;
    msg.range = distance;

    return msg;
}
