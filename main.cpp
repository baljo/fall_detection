/*

Arduino Nicla Sense ME WEB BLE Sense dashboard demo


Hardware required: https://store.arduino.cc/nicla-sense-me

1) Upload this sketch to the Arduino Nano BLE sense board

2) Open the following web page in the Chrome browser:
https://arduino.github.io/ArduinoAI/NiclaSenseME-dashboard/

3) Click on the green button in the web page to connect the browser to the board over BLE


Web dashboard by D. Pajak

Device sketch based on example by Sandeep Mistry and Massimo Banzi
Sketch and web dashboard copy-fixed to be used with the Nicla Sense ME by Pablo Marquínez

*/

//#include <Arduino.h>
#include <Fall_detection_w_Nicla_Sense_ME_inferencing.h>
#include "Arduino_BHY2.h"  //Click here to get the library: http://librarymanager/All#Arduino_BHY2
#include "Nicla_System.h"
#include "ArduinoBLE.h"

#define BLE_SENSE_UUID(val) ("19b10000-" val "-537e-4f6c-d104768a1214")

const int VERSION = 0x00000000;

BLEService service(BLE_SENSE_UUID("0000"));

BLEUnsignedIntCharacteristic versionCharacteristic(BLE_SENSE_UUID("1001"), BLERead | BLENotify);
BLEShortCharacteristic temperatureCharacteristic(BLE_SENSE_UUID("2001"), BLERead | BLENotify);
BLEShortCharacteristic humidityCharacteristic(BLE_SENSE_UUID("3001"), BLERead | BLENotify);
BLEShortCharacteristic pressureCharacteristic(BLE_SENSE_UUID("4001"), BLERead | BLENotify);

BLECharacteristic accelerometerCharacteristic(BLE_SENSE_UUID("5001"), BLERead | BLENotify, 3 * sizeof(float));  // Array of 3x 2 Bytes, XY
BLECharacteristic gyroscopeCharacteristic(BLE_SENSE_UUID("6001"), BLERead | BLENotify, 3 * sizeof(float));      // Array of 3x 2 Bytes, XYZ
BLECharacteristic quaternionCharacteristic(BLE_SENSE_UUID("7001"), BLERead | BLENotify, 4 * sizeof(float));     // Array of 4x 2 Bytes, XYZW

BLECharacteristic rgbLedCharacteristic(BLE_SENSE_UUID("8001"), BLERead | BLEWrite, 3 * sizeof(byte));  // Array of 3 bytes, RGB

BLEShortCharacteristic bsecCharacteristic(BLE_SENSE_UUID("9001"), BLERead | BLENotify);
BLEShortCharacteristic co2Characteristic(BLE_SENSE_UUID("9002"), BLERead | BLENotify);
BLEShortCharacteristic gasCharacteristic(BLE_SENSE_UUID("9003"), BLERead | BLENotify);
BLEShortCharacteristic accCharacteristic(BLE_SENSE_UUID("9004"), BLERead | BLENotify);

// String to calculate the local and device name
String name;
/*
Sensor temperature(SENSOR_ID_TEMP);
Sensor humidity(SENSOR_ID_HUM);
Sensor pressure(SENSOR_ID_BARO);
Sensor gas(SENSOR_ID_GAS);
SensorXYZ gyroscope(SENSOR_ID_GYRO);
// SensorXYZ accelerometer(SENSOR_ID_ACC);
SensorQuaternion quaternion(SENSOR_ID_RV);
SensorBSEC bsec(SENSOR_ID_BSEC);
*/
int connected = 0;
long int updateFreq;


// ***************** FROM ANOMALY PROGRAM *****************

/** Struct to link sensor axis name to sensor value function */
typedef struct {
  const char *name;
  float (*get_value)(void);

} eiSensors;

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2 9.80665f

/** Number sensor axes used */
#define NICLA_N_SENSORS 3


/* Private variables ------------------------------------------------------- */
static const bool debug_nn = false;  // Set this to true to see e.g. features generated from the raw signal

SensorXYZ accel(SENSOR_ID_ACC);
/*
SensorXYZ gyro(SENSOR_ID_GYRO);
  SensorOrientation ori(SENSOR_ID_ORI);
  SensorQuaternion rotation(SENSOR_ID_RV);
  Sensor temp(SENSOR_ID_TEMP);
  Sensor baro(SENSOR_ID_BARO);
  Sensor hum(SENSOR_ID_HUM);
  Sensor gas(SENSOR_ID_GAS);
*/
static bool ei_connect_fusion_list(const char *input_list);
static float get_accX(void) {
  return (accel.x() * 8.0 / 32768.0) * CONVERT_G_TO_MS2;
}
static float get_accY(void) {
  return (accel.y() * 8.0 / 32768.0) * CONVERT_G_TO_MS2;
}
static float get_accZ(void) {
  return (accel.z() * 8.0 / 32768.0) * CONVERT_G_TO_MS2;
}
/*
  static float get_gyrX(void){return (gyro.x() * 8.0 / 32768.0) * CONVERT_G_TO_MS2;}
  static float get_gyrY(void){return (gyro.y() * 8.0 / 32768.0) * CONVERT_G_TO_MS2;}
  static float get_gyrZ(void){return (gyro.z() * 8.0 / 32768.0) * CONVERT_G_TO_MS2;}
  static float get_oriHeading(void){return ori.heading();}
  static float get_oriPitch(void){return ori.pitch();}
  static float get_oriRoll(void){return ori.roll();}
  static float get_rotX(void){return rotation.x();}
  static float get_rotY(void){return rotation.y();}
  static float get_rotZ(void){return rotation.z();}
  static float get_rotW(void){return rotation.w();}
  static float get_temperature(void){return temp.value();}
  static float get_barrometric_pressure(void){return baro.value();}
  static float get_humidity(void){return hum.value();}
  static float get_gas(void){return gas.value();}

*/
static int8_t fusion_sensors[NICLA_N_SENSORS];
static int fusion_ix = 0;
int red_led_time = 0;

/** Used sensors value function connected to label name */
eiSensors nicla_sensors[] = {
  "x", &get_accX,
  "y", &get_accY,
  "z", &get_accZ,
/*      "gyrX", &get_gyrX,
      "gyrY", &get_gyrY,
      "gyrZ", &get_gyrZ,
      "heading", &get_oriHeading,
      "pitch", &get_oriPitch,
      "roll", &get_oriRoll,
      "rotX", &get_rotX,
      "rotY", &get_rotY,
      "rotZ", &get_rotZ,
      "rotW", &get_rotW,
      "temperature", &get_temperature,
      "barometer", &get_barrometric_pressure,
      "humidity", &get_humidity,
      "gas", &get_gas,*/
};

// ********************************************************


void setup() {

  Serial.begin(115200);

  Serial.println("\n ***** Start *****");

  nicla::begin();
  nicla::leds.begin();
  nicla::leds.setColor(green);

  while (!Serial)
      ;
    Serial.println("Edge Impulse Sensor Fusion Inference\r\n");

    /* Connect used sensors */
    if (ei_connect_fusion_list(EI_CLASSIFIER_FUSION_AXES_STRING) == false) {
      ei_printf("ERR: Errors in sensor list detected\r\n");
      return;
    }



 // nicla::enableCharge(200);  // Charges the battery with 100 mA

  //Sensors initialization
 // BHY2.begin(NICLA_STANDALONE);
/*  temperature.begin();
  humidity.begin();
  pressure.begin();
  gyroscope.begin();*/
  //  accelerometer.begin();
 // accel.begin();
/*
  quaternion.begin();
  bsec.begin();
  gas.begin();
*/
  /* Init & start sensors */
    BHY2.begin(NICLA_I2C);
    accel.begin();
  /*
      gyro.begin();
      ori.begin();
      rotation.begin();
      temp.begin();
      baro.begin();
      hum.begin();
      gas.begin();*/



  if (!BLE.begin()) {
    Serial.println("Failled to initialized BLE!");

    while (1)
      ;
    }

  String address = BLE.address();

  Serial.print("address = ");
  Serial.println(address);

  address.toUpperCase();

  name = "NiclaSenseME-";
  name += address[address.length() - 5];
  name += address[address.length() - 4];
  name += address[address.length() - 2];
  name += address[address.length() - 1];

  Serial.print("name = ");
  Serial.println(name);

  BLE.setLocalName(name.c_str());
  BLE.setDeviceName(name.c_str());
  BLE.setAdvertisedService(service);
  Serial.println(String(service));

  // Add all the previously defined Characteristics
  service.addCharacteristic(temperatureCharacteristic);
  service.addCharacteristic(humidityCharacteristic);
  service.addCharacteristic(pressureCharacteristic);
  service.addCharacteristic(versionCharacteristic);
  service.addCharacteristic(accelerometerCharacteristic);
  service.addCharacteristic(gyroscopeCharacteristic);
  service.addCharacteristic(quaternionCharacteristic);
  service.addCharacteristic(bsecCharacteristic);
  service.addCharacteristic(co2Characteristic);
  service.addCharacteristic(gasCharacteristic);
  service.addCharacteristic(rgbLedCharacteristic);
  service.addCharacteristic(accCharacteristic);

  Serial.println(String(humidityCharacteristic));
/*
  // Disconnect event handler
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // Sensors event handlers
  temperatureCharacteristic.setEventHandler(BLERead, onTemperatureCharacteristicRead);
  humidityCharacteristic.setEventHandler(BLERead, onHumidityCharacteristicRead);
  pressureCharacteristic.setEventHandler(BLERead, onPressureCharacteristicRead);
  bsecCharacteristic.setEventHandler(BLERead, onBsecCharacteristicRead);
  co2Characteristic.setEventHandler(BLERead, onCo2CharacteristicRead);
  gasCharacteristic.setEventHandler(BLERead, onGasCharacteristicRead);

  rgbLedCharacteristic.setEventHandler(BLEWritten, onRgbLedCharacteristicWrite);
*/
  versionCharacteristic.setValue(VERSION);

  BLE.addService(service);
  //BLE.setAdvertisingInterval(320);
  BLE.advertise();
  nicla::leds.setColor(blue);
}

uint8_t i = 0;
unsigned long int startedTime;


void loop() {

  if (connected == 0) {
    startedTime = millis();
    connected = 0;
  }

  while (BLE.connected()) {

    static auto printTime = millis();

    BHY2.update();
    nicla::leds.setColor(0, 0, 0);

    updateFreq = 500;  // Default update frequency which will be overridden soon

    if (printTime - startedTime >= 60000)  // First whatever seconds updating more frequently
      updateFreq = 1000;
    if (printTime - startedTime >= 300000)
      updateFreq = 5000;
    if (printTime - startedTime >= 600000)
      updateFreq = 10000;

    if (millis() - printTime >= updateFreq) {
      nicla::leds.setColor(blue);
      printTime = millis();
/*
      float temperatureValue = temperature.value();
      temperatureCharacteristic.writeValue(temperatureValue * 100);
      //Serial.println(temperatureValue);

      uint32_t co2 = bsec.co2_eq();
      co2Characteristic.writeValue(co2);

      float airQuality = float(bsec.iaq());
      bsecCharacteristic.writeValue(airQuality);

      float pressureValue = pressure.value();
      pressureCharacteristic.writeValue((pressureValue - 800) * 100);
      //Serial.println(pressureValue);

      float humidityValue = humidity.value();
      humidityCharacteristic.writeValue(humidityValue);

      unsigned int g = gas.value();
      gasCharacteristic.writeValue(g);
*/

      // **************** FROM ANOMALY PROGRAM *******************


      if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != fusion_ix) {
        ei_printf("ERR: Nicla sensors don't match the sensors required in the model\r\n"
                  "Following sensors are required: %s\r\n",
                  EI_CLASSIFIER_FUSION_AXES_STRING);
        return;
      }

      //ei_printf("Sampling...\r\n");

      // Allocate a buffer here for the values we'll read from the IMU
      float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

      for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) {
        // Determine the next tick (and then sleep later)
        int64_t next_tick = (int64_t)micros() + ((int64_t)EI_CLASSIFIER_INTERVAL_MS * 1000);

        // Update function should be continuously polled
        BHY2.update();

        for (int i = 0; i < fusion_ix; i++) {
          buffer[ix + i] = nicla_sensors[fusion_sensors[i]].get_value();
        }

        int64_t wait_time = next_tick - (int64_t)micros();

        if (wait_time > 0) {
          delayMicroseconds(wait_time);
        }
      }

      // Turn the raw buffer in a signal which we can the classify
      signal_t signal;
      int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
      if (err != 0) {
        ei_printf("ERR:(%d)\r\n", err);
        return;
      }

      // Run the classifier
      ei_impulse_result_t result = { 0 };

      err = run_classifier(&signal, &result, debug_nn);
      if (err != EI_IMPULSE_OK) {
        ei_printf("ERR:(%d)\r\n", err);
        return;
      }

      // print the predictions
      //ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.):\r\n",
      //          result.timing.dsp, result.timing.classification, result.timing.anomaly);
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("%s: %.5f\r\n", result.classification[ix].label, result.classification[ix].value);
      }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
      ei_printf("%.3f\r\n", result.anomaly);
      accCharacteristic.writeValue(result.anomaly);
      //  ei_printf("    anomaly score: %.3f\r\n", result.anomaly);
      if (result.anomaly > 50) {
        for (red_led_time = 0; red_led_time < 10; red_led_time++) {
          nicla::leds.setColor(red);
          delay(100);
          nicla::leds.setColor(off);
          delay(200);
        }
      }
#endif

      // *********************************************************
    }
  }
  nicla::leds.setColor(green);
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  nicla::leds.setColor(red);
}
/*
void onTemperatureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  float temperatureValue = temperature.value();
  //temperatureCharacteristic.writeValue(temperatureValue);
}

void onHumidityCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t humidityValue = humidity.value() + 0.5f;  //since we are truncating the float type to a uint8_t, we want to round it
  humidityCharacteristic.writeValue(humidityValue);
  //Serial.println(humidityValue);
}

void onPressureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  float pressureValue = pressure.value();
  pressureCharacteristic.writeValue(pressureValue);
}

void onBsecCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  float airQuality = float(bsec.iaq());
  bsecCharacteristic.writeValue(airQuality);
}

void onCo2CharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  uint32_t co2 = bsec.co2_eq();
  co2Characteristic.writeValue(co2);
}

void onGasCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  unsigned int g = gas.value();
  gasCharacteristic.writeValue(g);
}
*/
void onRgbLedCharacteristicWrite(BLEDevice central, BLECharacteristic characteristic) {
  byte r = rgbLedCharacteristic[0];
  byte g = rgbLedCharacteristic[1];
  byte b = rgbLedCharacteristic[2];

  nicla::leds.setColor(r, g, b);
}


#if !defined(EI_CLASSIFIER_SENSOR) || (EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_FUSION && EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER)
#error "Invalid model for current sensor"
#endif


/**
     @brief Go through nicla sensor list to find matching axis name

     @param axis_name
     @return int8_t index in nicla sensor list, -1 if axis name is not found
  */
static int8_t ei_find_axis(char *axis_name) {
  int ix;
  for (ix = 0; ix < NICLA_N_SENSORS; ix++) {
    if (strstr(axis_name, nicla_sensors[ix].name)) {
      return ix;
    }
  }
  return -1;
}

/**
     @brief Check if requested input list is valid sensor fusion, create sensor buffer

     @param[in]  input_list      Axes list to sample (ie. "accX + gyrY + magZ")
     @retval  false if invalid sensor_list
  */
static bool ei_connect_fusion_list(const char *input_list) {
  char *buff;
  bool is_fusion = false;

  /* Copy const string in heap mem */
  char *input_string = (char *)ei_malloc(strlen(input_list) + 1);
  if (input_string == NULL) {
    return false;
  }
  memset(input_string, 0, strlen(input_list) + 1);
  strncpy(input_string, input_list, strlen(input_list));

  /* Clear fusion sensor list */
  memset(fusion_sensors, 0, NICLA_N_SENSORS);
  fusion_ix = 0;

  buff = strtok(input_string, "+");

  while (buff != NULL) { /* Run through buffer */
    int8_t found_axis = 0;

    is_fusion = false;
    found_axis = ei_find_axis(buff);

    if (found_axis >= 0) {
      if (fusion_ix < NICLA_N_SENSORS) {
        fusion_sensors[fusion_ix++] = found_axis;
      }
      is_fusion = true;
    }

    buff = strtok(NULL, "+ ");
  }

  ei_free(input_string);

  return is_fusion;
}