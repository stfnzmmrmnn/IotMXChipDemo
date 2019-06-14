// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license. 

#include "HTS221Sensor.h"
#include "LPS22HBSensor.h"
#include "LSM6DSLSensor.h"
#include "LIS2MDLSensor.h"
#include "IrDASensor.h"
#include "AzureIotHub.h"
#include "Arduino.h"
#include "parson.h"
#include "config.h"
#include "RGB_LED.h"
#include "utility.h"

#define RGB_LED_BRIGHTNESS 32

DevI2C *i2c;
HTS221Sensor *tempHumiditySensor;
LPS22HBSensor *pressureSensor;
LSM6DSLSensor *accelGyroSensor;
LIS2MDLSensor *magnetoSensor;
IRDASensor *irdaSensor;

static RGB_LED rgbLed;
static int interval = INTERVAL;
static float humidity;
static float temperature;
static float pressure;
int *accelerometerAxes;
int *gyroscopeAxes;
int *magnetometerAxes;
float data;

int getInterval()
{
    return interval;
}

void blinkLED()
{
    rgbLed.turnOff();
    rgbLed.setColor(RGB_LED_BRIGHTNESS, 0, 0);
    delay(500);
    rgbLed.turnOff();
}

void blinkSendConfirmation()
{
    rgbLed.turnOff();
    rgbLed.setColor(0, 0, RGB_LED_BRIGHTNESS);
    delay(500);
    rgbLed.turnOff();
}

void parseTwinMessage(DEVICE_TWIN_UPDATE_STATE updateState, const char *message)
{
    JSON_Value *root_value;
    root_value = json_parse_string(message);
    if (json_value_get_type(root_value) != JSONObject)
    {
        if (root_value != NULL)
        {
            json_value_free(root_value);
        }
        LogError("parse %s failed", message);
        return;
    }
    JSON_Object *root_object = json_value_get_object(root_value);

    double val = 0;
    if (updateState == DEVICE_TWIN_UPDATE_COMPLETE)
    {
        JSON_Object *desired_object = json_object_get_object(root_object, "desired");
        if (desired_object != NULL)
        {
            val = json_object_get_number(desired_object, "interval");
        }
    }
    else
    {
        val = json_object_get_number(root_object, "interval");
    }
    if (val > 500)
    {
        interval = (int)val;
        LogInfo(">>>Device twin updated: set interval to %d", interval);
    }
    json_value_free(root_value);
}

void SensorInit()
{
    i2c = new DevI2C(D14, D15);
    tempHumiditySensor = new HTS221Sensor(*i2c);
    tempHumiditySensor->init(NULL);
    pressureSensor = new LPS22HBSensor(*i2c);
    pressureSensor->init(NULL);
    accelGyroSensor = new LSM6DSLSensor(*i2c, D4, D5);
    accelGyroSensor->init(NULL);
    accelGyroSensor->enableAccelerator();
    accelGyroSensor->enableGyroscope();
    //accelGyro->enablePedometer();
    //accelGyro->setPedometerThreshold(LSM6DSL_PEDOMETER_THRESHOLD_MID_LOW);
    magnetoSensor = new LIS2MDLSensor(*i2c);
    magnetoSensor->init(NULL);
    //irdaSensor = new IRDASensor();
    //irdaSensor->init();

    // initialize values
    humidity = -1;
    temperature = -1000;
    pressure = 0;
    int initialGyroscopeAxes[3] = {0};
    gyroscopeAxes = initialGyroscopeAxes;
    int initialAccelerometerAxes[3] = {0};
    accelerometerAxes = initialAccelerometerAxes;
    int initialMagnetometerAxes[3] = {0};
    magnetometerAxes = initialMagnetometerAxes;
}

float readTemperature()
{
    tempHumiditySensor ->reset();
    float temperature = 0;
    tempHumiditySensor->getTemperature(&temperature);
    return temperature;
}

float readHumidity()
{
    tempHumiditySensor->reset();
    float humidity = 0;
    tempHumiditySensor->getHumidity(&humidity);
    return humidity;
}

float readPressure()
{
    float pressure = 0;
    pressureSensor->getPressure(&pressure);
    return pressure;
}

int* readMagnetometer()
{
    static int magnetometerAxes[3] = {0};
    magnetoSensor->getMAxes(magnetometerAxes);
    return magnetometerAxes;
}

int* readAccelerometer()
{
    static int accelerometerAxes[3] = {0};
    accelGyroSensor->getXAxes(accelerometerAxes);
    accelGyroSensor->getXSensitivity(&data);
    return accelerometerAxes;
}

int* readGyroscope()
{
    static int gyroscopeAxes[3] = {0};
    accelGyroSensor->getGAxes(gyroscopeAxes);
    accelGyroSensor->getGSensitivity(&data);
    return gyroscopeAxes;
}

bool* readMessage(int messageId, char *payload, float *temperatureValue, float *humidityValue)
{
    JSON_Value *root_value = json_value_init_object();
    JSON_Object *root_object = json_value_get_object(root_value);
    char *serialized_string = NULL;
    static bool alerts[2]; // = {false};

    // set message ID value
    json_object_set_number(root_object, "messageId", messageId);

    // read sensor data
    float t = readTemperature();
    float h = readHumidity();
    float p = readPressure();
    int *m = readMagnetometer();
    int *a = readAccelerometer();
    int *g = readGyroscope();
    
    // initialize alarm bools
    bool temperatureAlert = false;
    bool motionAlert = false;

    // add temperature values to JSON object
    
    
        temperature = t;
        *temperatureValue = t;
        json_object_set_number(root_object, "temperature", temperature);
    
    // add humidity values to JSON object
    
        humidity = h;
        *humidityValue = h;
        json_object_set_number(root_object, "humidity", humidity);
    
    // add pressure values to JSON object
        
        pressure = p;
        json_object_set_number(root_object, "pressure", pressure);
    

    // add magnetometer values to JSON object
    json_object_set_number(root_object, "magnetometerX", m[0]);
    json_object_set_number(root_object, "magnetometerY", m[1]);
    json_object_set_number(root_object, "magnetometerZ", m[2]);
    
    // add accelerometer values to JSON object
    json_object_set_number(root_object, "accelerometerX", a[0]);
    json_object_set_number(root_object, "accelerometerY", a[1]);
    json_object_set_number(root_object, "accelerometerZ", a[2]);

    // add gyroscope values to JSON object
    json_object_set_number(root_object, "gyroscopeX", g[0]);
    json_object_set_number(root_object, "gyroscopeY", g[1]);
    json_object_set_number(root_object, "gyroscopeZ", g[2]);

    // send alarms
    if(temperature > TEMPERATURE_ALERT)
    {
        temperatureAlert = true;
    }
    if (abs(a[0]) > ACCELEROMETER_ALERT || abs(a[1]) > ACCELEROMETER_ALERT || abs(a[2]) > ACCELEROMETER_ALERT)
    {
        motionAlert = true;
    }

    // save old values
    gyroscopeAxes = g;
    accelerometerAxes = a;
    magnetometerAxes = m;

    // set return array
    alerts[0] = temperatureAlert;
    alerts[1] = motionAlert;

    serialized_string = json_serialize_to_string_pretty(root_value);

    snprintf(payload, MESSAGE_MAX_LEN, "%s", serialized_string);
    json_free_serialized_string(serialized_string);
    json_value_free(root_value);
    return alerts;
}

#if (DEVKIT_SDK_VERSION >= 10602)
void __sys_setup(void)
{
    // Enable Web Server for system configuration when system enter AP mode
    EnableSystemWeb(WEB_SETTING_IOT_DEVICE_CONN_STRING);
}
#endif