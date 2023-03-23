/*
  Arduino LSM6DSOX - Power modes and sample rates tester

  This example sets the various LSM6DSOX power modes and
  sample rates for accelerometer and gyroscope.

  The circuit:
  - Arduino Nano RP2040 Connect

  created 23 Mar 2023
  by Rene van Ee

  This example code is in the public domain.
*/

#include <Arduino_LSM6DSOX.h>
#include <Scheduler.h>

#define INTERVAL_MS 5000

unsigned int samplecounter_XL;
unsigned int samplecounter_G;

String strPowerModeXL[] = {
  "UNDEFINED", "POWER_DOWN", "ULTRA LOW POWER", "LOW POWER/NORMAL", "HIGH PERFORMANCE"
};
String strPowerModeG[] = {
  "UNDEFINED", "POWER_DOWN", "LOW POWER/NORMAL", "HIGH PERFORMANCE"
};

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  Scheduler.startLoop(loop_odr);
}

void loop() {
  if (IMU.accelerationAvailable()) {
    float x, y, z;
    IMU.readAcceleration(x, y, z);

    samplecounter_XL++;
  }

  if (IMU.gyroscopeAvailable()) {
    float x, y, z;
    IMU.readGyroscope(x, y, z);

    samplecounter_G++;
  }

  yield();
}

void testBlock(PowerModeXL powerXL, PowerModeG powerG, float odr_XL, float odr_G) {
  Serial.println("Setting G power mode to " + strPowerModeG[(int)powerG]);
  IMU.setPowerModeG(powerG);

  Serial.println("Setting XL power mode to " + strPowerModeXL[(int)powerXL]);
  IMU.setPowerModeXL(powerXL);

  Serial.println("Setting XL ODR to " + String(odr_XL));
  IMU.setODR_XL(odr_XL);

  if(powerG != PowerModeG::G_POWER_DOWN) {
    Serial.println("Setting G ODR to " + String(odr_G));
    IMU.setODR_G(odr_G);
  }

  Serial.println("Collecting samples for " + String(INTERVAL_MS) + " ms");
  samplecounter_XL = 0;
  samplecounter_G = 0;
  delay(INTERVAL_MS);

  float freq_XL_measured = (1000.0*samplecounter_XL) / INTERVAL_MS;
  float freq_G_measured  = (1000.0*samplecounter_G)  / INTERVAL_MS;

  float freq_XL_specified = IMU.accelerationSampleRate();
  float freq_G_specified = IMU.gyroscopeSampleRate();

  Serial.print("XL sample rate = ");
  Serial.print(freq_XL_specified);
  Serial.print("(specified), ");
  Serial.print(freq_XL_measured);
  Serial.println("(measured)");

  Serial.print("G sample rate = ");
  Serial.print(freq_G_specified);
  Serial.print("(specified), ");
  Serial.print(freq_G_measured);
  Serial.println("(measured)");
  Serial.println("-");

  yield();
}

void loop_odr() {
  testBlock(PowerModeXL::XL_POWER_DOWN,           G_POWER_DOWN, 0, 0);
  testBlock(PowerModeXL::XL_POWER_MODE_LP_NORMAL, PowerModeG::G_POWER_MODE_LP_NORMAL, 1.6, 0);
  testBlock(PowerModeXL::XL_POWER_MODE_ULP,       PowerModeG::G_POWER_DOWN, 1.6, 0);
  testBlock(PowerModeXL::XL_POWER_MODE_LP_NORMAL, PowerModeG::G_POWER_MODE_LP_NORMAL, 12.5, 12.5);
  testBlock(PowerModeXL::XL_POWER_MODE_LP_NORMAL, PowerModeG::G_POWER_MODE_LP_NORMAL, 26, 26);
  testBlock(PowerModeXL::XL_POWER_MODE_LP_NORMAL, PowerModeG::G_POWER_MODE_LP_NORMAL, 52, 52);
  testBlock(PowerModeXL::XL_POWER_MODE_LP_NORMAL, PowerModeG::G_POWER_MODE_LP_NORMAL, 104, 104);
  testBlock(PowerModeXL::XL_POWER_MODE_LP_NORMAL, PowerModeG::G_POWER_MODE_LP_NORMAL, 208, 208);
  testBlock(PowerModeXL::XL_POWER_MODE_HP,        PowerModeG::G_POWER_MODE_HP, 416, 416);
  testBlock(PowerModeXL::XL_POWER_MODE_HP,        PowerModeG::G_POWER_MODE_HP, 833, 833);
}