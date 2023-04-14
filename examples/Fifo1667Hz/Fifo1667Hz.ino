/*
  Arduino LSM6DSOX - Read FIFO

  This example demonstrates high frequency reading of accelerometer, 
  gyroscope, temperature and timestamp values from an LSM6DSOX FIFO.

  The circuit:
  - Arduino Nano RP2040 Connect

  created 21 February 2023
  by Rene van Ee

  This example code is in the public domain.
*/

#include <Arduino_LSM6DSOX.h>
#include <limits>
#include <math.h> // isnan

int lasttime;

uint32_t prev_counter;

unsigned long execution_time_min;
unsigned long execution_time_max;
unsigned long execution_time_sum;
int execution_time_N;

double T_sum;
int T_N;

#define REPORT_INTERVAL       5000
#define ODR_ALL               1667


void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting device...");

  IMU.settings.i2c_frequency = 400000;
  IMU.settings.powerMode_XL = XL_POWER_MODE_HP;
  IMU.settings.ODR_XL = ODR_ALL;
  IMU.settings.powerMode_G = G_POWER_MODE_HP;
  IMU.settings.ODR_G = ODR_ALL;
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  // Allow for some startup time
  delay(10);

  IMU.resetTimestamp();

  IMU.fifo.settings.compression = true;
  IMU.fifo.settings.BDR_XL = ODR_ALL;
  IMU.fifo.settings.BDR_G = ODR_ALL;
  IMU.fifo.settings.timestamp_decimation = 8;
  IMU.fifo.settings.BDR_temperature = 52;
  IMU.fifo.begin();

  // Wait until samples are being produced
  FIFOStatus status;
  do {
    delay(1);
    if(IMU.fifo.readStatus(status) != 1) {
      Serial.println("Error reading from IMU!");
      while (1);
    }
  } while(status.DIFF_FIFO == 0);

  lasttime = millis();

  prev_counter = IMU.fifo.counter_uninitialized;

  execution_time_min = std::numeric_limits<unsigned long>::max();
  execution_time_max = 0;
  execution_time_sum = 0;
  execution_time_N = 0;

  T_sum = 0.0;
  T_N = 0;
}

void loop() {
  Sample sample;
  unsigned long us_before = micros();
  SampleStatus sampleResult = IMU.fifo.getSample(sample);
  unsigned long execution_time = micros() - us_before;

  bool errorOccurred = false;
  switch(sampleResult) {
    // Good results
    case SampleStatus::OK: // Sample successfully retrieved
      // Check if current counter equals previous counter+1
      // (basic sanity check)
      if(prev_counter != IMU.fifo.counter_uninitialized) {
        if(sample.counter != (prev_counter+1))
        {
          Serial.print("Counter error");
          errorOccurred = true;
          break;
        }
      }
      prev_counter = sample.counter;     

      if(execution_time_min > execution_time) execution_time_min = execution_time;
      if(execution_time_max < execution_time) execution_time_max = execution_time;
      execution_time_sum += execution_time;
      execution_time_N++;

      // In some samples T is defined (not isnan)
      if(!isnan(sample.temperature)) {
        T_sum += sample.temperature;
        T_N++;
      }

      /*
      Serial.println("sample retrieved: counter="+String(sample.counter)+" t="+String(sample.timestamp)+" T="+String(sample.temperature)+
        " XL="+String(sample.XL_X)+","+String(sample.XL_Y)+","+String(sample.XL_Z)+"(FS "+String(sample.XL_fullScale)+")"+
        " G="+String(sample.G_X)+","+String(sample.G_Y)+","+String(sample.G_Z)+"(FS "+String(sample.G_fullScale)+")");
        */
      break;
    case SampleStatus::BUFFER_UNDERRUN:
      // No sample available: we're too fast, need to wait (or perform something useful)
      delay(1);
      break;

    // Bad results (various errors)
    case SampleStatus::COMMUNICATION_ERROR:
      Serial.print("Communication error");
      errorOccurred = true;
      break;
    case SampleStatus::PARITY_ERROR:
      Serial.print("Tag parity error");
      errorOccurred = true;
      break;
    case SampleStatus::TAG_NOT_IMPLEMENTED:
      Serial.print("Tag not implemented error");
      errorOccurred = true;
      break;
    case SampleStatus::UNKNOWN_TAG:
      Serial.print("Unknown tag error");
      errorOccurred = true;
      break;
    case SampleStatus::BUFFER_OVERRUN:
      // We're too slow
      Serial.print("Buffer overrun error");
      errorOccurred = true;
      break;
    default:
      break;
  }
  if(errorOccurred) {
    Serial.println(" while reading from IMU!");
    while (1);
  }

  // Reporting
  int currenttime = millis();
  int deltat = currenttime - lasttime;
  if(deltat > REPORT_INTERVAL) {
    lasttime = currenttime;
    double frequency = (double)execution_time_N / (0.001*deltat);
    Serial.println("Sample frequency = " + String(execution_time_N) + 
                   " / "+String(0.001*deltat)+"s = "+String(frequency)+"/s");

    Serial.println("Retrieve sample execution time [us]:");
    Serial.println("\tmin = "+String(execution_time_min));
    Serial.println("\tmax = "+String(execution_time_max));
    Serial.println("\tavg = "+String((double)execution_time_sum / execution_time_N));
    Serial.println("\t%   = "+String((double)execution_time_sum / (10.0*deltat)));

    execution_time_min = std::numeric_limits<unsigned long>::max();
    execution_time_max = 0;
    execution_time_sum = 0;
    execution_time_N = 0;

    Serial.println("Tavg = "+String(T_sum / T_N)+" degC ("+String(T_N)+" samples)");
    Serial.println("---");

    T_sum = 0.0;
    T_N = 0;
  }
}
