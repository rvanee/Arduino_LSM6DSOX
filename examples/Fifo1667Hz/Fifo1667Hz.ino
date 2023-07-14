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


#define REPORT_INTERVAL       60000
#define ODR_ALL               1667


class SimpleStats {
public:
  SimpleStats(String name) {
    this->name = name;
    reset();
  }

  void reset() {
    sum = 0.0;
    sum_squared = 0.0;
    min = std::numeric_limits<double>::max();
    max = 0.0;
    N = 0;
  }

  void add(double value) {
    sum += value;
    sum_squared += value*value;
    if(value < min) min = value;
    if(value > max) max = value;
    N++;
  }

  double mean() { return N>0 ? sum / N : NAN; }
  double std()  { return N>1 ? sqrt((sum_squared - (sum*sum/N))/(N-1)) : NAN; }

  String name;
  double min;
  double max;
  unsigned long N;

private:
  double sum;
  double sum_squared;
};

SimpleStats execution_time = SimpleStats("execution_time");
SimpleStats temperature = SimpleStats("temperature");
SimpleStats XL[3] = {SimpleStats("XL_X"), SimpleStats("XL_Y"), SimpleStats("XL_Z")};
SimpleStats G[3] = {SimpleStats("G_X"), SimpleStats("G_Y"), SimpleStats("G_Z")};
SimpleStats sample_interval = SimpleStats("sample_interval");
SimpleStats a = SimpleStats("a");

int lasttime;
uint32_t prev_counter;
uint64_t prev_micros;

unsigned long XL_invalid;
unsigned long G_invalid;
unsigned long num_errors;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting device...");

  // The LSM6DSOX supports I2c fast mode (400kb/s).
  // You may also try fast mode plus (1000kb/s); 
  // this worked on my 2023 Arduino Nano RP2040. 
  IMU.settings.i2c_frequency = 400000; // or 1000000 (fast mode plus)
  IMU.settings.rad_G = true; // Gyro output in rad/s rather than deg/s
  IMU.settings.powerMode_XL = XL_POWER_MODE_HP;
  IMU.settings.ODR_XL = ODR_ALL;
  //IMU.settings.cutoff_LPF2_XL = 400;
  IMU.settings.fullRange_XL = 2;
  IMU.settings.powerMode_G = G_POWER_MODE_HP;
  IMU.settings.ODR_G = ODR_ALL;
  //IMU.settings.cutoff_LPF1_G = 416;
  IMU.settings.fullRange_G = 125;
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
  IMU.fifo.settings.timestamp_decimation = 0;
  IMU.fifo.settings.BDR_temperature = 52;
  IMU.fifo.begin();
  
  prev_counter = 0;

  XL_invalid = 0;
  G_invalid = 0;
  num_errors = 0;

  lasttime = millis();

  prev_micros = micros();
}

void loop() {
  unsigned long us_before = micros();
  ReadResult readResult = IMU.fifo.fillQueue();
  unsigned long execution_time_us = micros() - us_before;
  execution_time.add(execution_time_us);

  bool errorOccurred = false;
  switch(readResult) {
    case ReadResult::DATA_READ:
      break;
    case ReadResult::NO_DATA_AVAILABLE:
      // No sample available: we're too fast, don't do anything here
      break;
    case ReadResult::QUEUE_FULL:
      // We're too slow in sample extraction from the queue...
      Serial.print("Buffer overrun error");
      errorOccurred = true;
      break;
    case ReadResult::FIFO_OVERFLOW:
      // We're too slow in reading words from the IMU...
      Serial.print("IMU FIFO overrun error");
      errorOccurred = true;
      break;
    case ReadResult::READ_ERROR:
      Serial.print("Communication error");
      errorOccurred = true;
      break;
    case ReadResult::LOGIC_ERROR:
    default:
      Serial.print("Software logic error, this shouldn't happen");
      errorOccurred = true;
  }
  if(errorOccurred) {
    Serial.println(" while reading from IMU!");
    while (1);
  }

  Sample sample;
  while(IMU.fifo.retrieveSample(sample)) {
    if(prev_counter != 0) {
      if(sample.counter != (prev_counter+1))
      {
        Serial.println("Counter error: counter="+String(sample.counter)+", prev="+String(prev_counter));
        num_errors++;
        // Go on, this may happen when problems occur with
        // the tag byte during decode
      }
    }
    prev_counter = sample.counter;

    // Invalid samples
    XL_invalid += !sample.XL.valid;
    G_invalid  += !sample.G.valid;

    // In some samples T is defined (not isnan)
    if(!isnan(sample.temperature)) {
      temperature.add(sample.temperature);
    }

    // Update XL X/Y/Z statistics
    if(!isnan(sample.XL.XYZ[0])) {
      for(int ixl=0; ixl<3; ixl++) { // Iterate through X/Y/Z
        XL[ixl].add((1.0/65536)*sample.XL.XYZ[ixl]);
      }
    }
    // Update G X/Y/Z statistics
    if(!isnan(sample.G.XYZ[0])) {
      for(int ig=0; ig<3; ig++) { // Iterate through X/Y/Z
        G[ig].add((1.0/65536)*sample.G.XYZ[ig]);
      }
    }

    // Update sample interval statistics
    // Exclude first 10% of samples to have the filters settle down
    if(sample.counter > ((REPORT_INTERVAL*ODR_ALL)/10000)) {
      sample_interval.add(sample.timestamp - prev_micros);
    }
    prev_micros = sample.timestamp;
  } // END while(retrieveSample(sample))

  // Reporting
  int currenttime = millis();
  int deltat = currenttime - lasttime;
  if(deltat > REPORT_INTERVAL) {
    IMU.fifo.end(); // Stop fifo queuing: reporting will take some time
    lasttime = currenttime;

    // Display timing information + bookkeeping
    double frequency = XL[0].N / (0.001*deltat);
    Serial.println("Sample frequency = " + String(XL[0].N) + 
                   " / "+String(0.001*deltat)+"s = "+String(frequency)+"/s");

    Serial.println("Sample retrieval execution time [us]:");
    Serial.println("\tmin = "+String(execution_time.min));
    Serial.println("\tmax = "+String(execution_time.max));
    Serial.println("\tavg = "+String(execution_time.mean()));
    Serial.println("\tstd = "+String(execution_time.std()));
    execution_time.reset();

    // Invalid XL/G measurements
    Serial.println("# invalid XL samples = "+String(XL_invalid));
    Serial.println("# invalid G  samples = "+String(G_invalid));
    Serial.println("# errors = "+String(num_errors));
    XL_invalid = 0;
    G_invalid = 0;
    // Do not reset num_errors, in order to increase test coverage

    // Display temperature information + bookkeeping
    Serial.println("Tavg = "+String(temperature.mean())+" (std="+
      String(temperature.std())+") degC ("+String(temperature.N)+" samples)");
    temperature.reset();

    // Display XL std information + bookkeeping
    for(int ixl=0; ixl < 3; ixl++) {
      double XL_std_mg = 1000*XL[ixl].std();
      Serial.println(XL[ixl].name+" std = "+String(XL_std_mg)+" mg");
      XL[ixl].reset();
    }

    // Display G stddev information + bookkeeping
    for(int ig=0; ig < 3; ig++) {
      double G_std_munits = 1000*G[ig].std();
      Serial.print(G[ig].name+" std = "+String(G_std_munits));
      if(IMU.settings.rad_G) {
        Serial.println(" mrad/s");
      } else {
        Serial.println(" mdeg/s");
      }
      G[ig].reset();
    }

    // Display sample interval information
    Serial.println("sample interval = "+String(sample_interval.mean())+" (std="+
      String(sample_interval.std())+") us ("+String(sample_interval.N)+" samples)");
    Serial.println("  min = "+String(sample_interval.min)+", max = "+
      String(sample_interval.max));
    sample_interval.reset();

    Serial.println("---");

    // Restart FIFO
    IMU.fifo.begin();
    prev_counter = 0;
    prev_micros = micros();
  } // END if(deltat > REPORT_INTERVAL)
} // END void loop()
