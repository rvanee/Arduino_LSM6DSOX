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

double XL_sum[3];
double XL_squared_sum[3];
int XL_N;
double G_sum[3];
double G_squared_sum[3];
int G_N;


#define REPORT_INTERVAL       5000
#define ODR_ALL               1667

double stddev(double squared_sum, double sum, unsigned long int n) {
  return sqrt((squared_sum - (sum*sum/n))/(n-1));
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting device...");

  IMU.settings.i2c_frequency = 400000;
  IMU.settings.powerMode_XL = XL_POWER_MODE_HP;
  IMU.settings.ODR_XL = ODR_ALL;//6667;
  //IMU.settings.cutoff_LPF2_XL = 800;
  IMU.settings.powerMode_G = G_POWER_MODE_HP;
  IMU.settings.ODR_G = ODR_ALL;//6667;
  //IMU.settings.cutoff_LPF1_G = 609;
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  // Allow for some startup time
  delay(10);

  IMU.resetTimestamp();

  IMU.fifo.settings.compression = true;
  IMU.fifo.settings.BDR_XL = ODR_ALL;//1667;
  IMU.fifo.settings.BDR_G = ODR_ALL;//1667;
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

  XL_sum[0] = 0.0;
  XL_sum[1] = 0.0;
  XL_sum[2] = 0.0;
  XL_squared_sum[0] = 0.0;
  XL_squared_sum[1] = 0.0;
  XL_squared_sum[2] = 0.0;
  XL_N = 0;

  G_sum[0] = 0.0;
  G_sum[1] = 0.0;
  G_sum[2] = 0.0;
  G_squared_sum[0] = 0.0;
  G_squared_sum[1] = 0.0;
  G_squared_sum[2] = 0.0;
  G_N = 0;
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

      // Update sum of XL X/Y/Z (squared)
      if(!isnan(sample.XL.XYZ[0])) {
        for(int ixl=0; ixl<3; ixl++) { // Iterate through X/Y/Z
          double XL_XYZ = sample.XL.XYZ[ixl];
          XL_sum[ixl] += XL_XYZ;
          XL_squared_sum[ixl] += XL_XYZ*XL_XYZ;
        }
        XL_N++;
      }
      // Update sum of G X/Y/Z (squared)
      if(!isnan(sample.G.XYZ[0])) {
        for(int ig=0; ig<3; ig++) { // Iterate through X/Y/Z
          double G_XYZ = sample.G.XYZ[ig];
          G_sum[ig] += G_XYZ;
          G_squared_sum[ig] += G_XYZ*G_XYZ;
        }
        G_N++;
      }
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

    // Display timing information + bookkeeping
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

    // Display temperature information + bookkeeping
     Serial.println("Tavg = "+String(T_sum / T_N)+" degC ("+String(T_N)+" samples)");
    T_sum = 0.0;
    T_N = 0;

    // Display XL stddev information + bookkeeping
     for(int ixl=0; ixl < 3; ixl++) {
      double XL_std_mg = 1000*stddev(XL_squared_sum[ixl], XL_sum[ixl], XL_N);
      char xyz = 'X' + ixl;
      Serial.println("XL_std "+String(xyz)+" = "+String(XL_std_mg)+" mg");
      XL_sum[ixl] = 0.0;
      XL_squared_sum[ixl] = 0.0;
    }
    XL_N = 0;

    // Display G stddev information + bookkeeping
     for(int ig=0; ig < 3; ig++) {
      double G_std_mdegs = 1000*stddev(G_squared_sum[ig], G_sum[ig], G_N);
      char xyz = 'X' + ig;
      Serial.println("G_std "+String(xyz)+" = "+String(G_std_mdegs)+" mdeg/s");
      G_sum[ig] = 0.0;
      G_squared_sum[ig] = 0.0;
    }
    G_N = 0;

    Serial.println("---");
  }
}
