#include "PING_sensor.h"
#include "Arduino.h"

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
    int64_t duration = pulseIn(pingPin, HIGH, 7500); // TODO: timeout
  
    float distance = duration * 0.00034 / 2;
    return distance;
}

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

void fillMeasurementMessage(const PINGSensor &ultraSonicSensor, sensor_msgs__msg__Range * msg)
{
    struct timespec tv = {0};
    clock_gettime(0, &tv);

    float distance = ultraSonicSensor.getMeasurement();

    msg->header.stamp.sec = tv.tv_sec;
    msg->header.stamp.nanosec = tv.tv_nsec;
    msg->range = distance;
}
