#include <Adafruit_BNO08x.h>
#include <sh2.h>
#include <sh2_SensorValue.h>
#include <sh2_err.h>
#include <sh2_hal.h>
#include <sh2_util.h>
#include <shtp.h>

//INT Method to Communicate sensors --Calibration
#include <SPI.h>
#include <math.h>

#define BNOs        3           // number of BNO08x breakouts
#define pinRST1     21          // GP21 connects to RST of first BNO085
#define pinRST2     21          // GP27 connects to RST of second BNO085
#define pinRST3     21          // GP14 connects to RST of third BNO085

#define pinINT1     20          // GP20 connects to INT of first BNO085
#define pinINT2     13          // GP26 connects to INT of second BNO085
#define pinINT3     8          // GP15 connects to INT of third BNO085

#define pinCS1      17          // GP17 connects to CS of first BNO085
#define pinCS2      15          // GP22 connects to CS of second BNO085
#define pinCS3      7          // GP28 connects to CS of third BNO085

// Sensor pin definition
const int CS_pins[BNOs] = {pinCS1, pinCS2, pinCS3};
const int INT_pins[BNOs] = {pinINT1, pinINT2, pinINT3};
const int RST_pins[BNOs] = {pinRST1, pinRST2, pinRST3};

// Common SPI pin definitions
#define pinSCL      18          // GP18 - SCL/SCLK
#define pinSDA      16          // GP16 - SDA/MISO
#define pinDI       19          // GP19 - DI/MOSI

#define ALL_REPORTS 0           // 0 request only quaternions, 1 request all reports
#define REPORT_TIME 1000L      // time between sensor reports (increased for debugging)
#define SPI_CLOCK   3000000L    // SPI clock rate (reduced for stability)
#define SERIAL_BAUD 2000000L     // serial port baud rate

// Enable debug mode for troubleshooting
#define DATAOUT     1           // 1 enables data output messages
#define PROCESS_EULER_ANGLES 1  // Set to 0 to disable Euler angle calculation and output

#define SPI_SETTINGS SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3)
#define numberof(x)  (sizeof(x) / sizeof(*(x)))

// Calibration related definitions
#define CALIBRATION_REPORT 0x14    // Calibration status report

#define SENSOR_ACCURACY_UNRELIABLE 0
#define SENSOR_ACCURACY_LOW 1
#define SENSOR_ACCURACY_MEDIUM 2
#define SENSOR_ACCURACY_HIGH 3

// Store calibration status
uint8_t calibration_accuracy[BNOs] = {0};  // Stores calibration accuracy for each sensor

// *******************
// **  Output data  **
// *******************

int16_t iax[BNOs], iay[BNOs], iaz[BNOs];             // accel, integer
int16_t igx[BNOs], igy[BNOs], igz[BNOs];             // gyro, integer
int16_t imx[BNOs], imy[BNOs], imz[BNOs];             // magneto, integer
int16_t ilx[BNOs], ily[BNOs], ilz[BNOs];             // linear accel, integer
int16_t iqw[BNOs], iqx[BNOs], iqy[BNOs], iqz[BNOs];  // quaternion, integer

char obuf[70], *pbuf;               // ensure this output buffer is big enough for your output string!
// unsigned long startTime = 0;        // Used to record the start time of the program

// Helper function for base64 encoding
static void uart_b64(int32_t i)     // output 18-bit integer as compact 3-digit base64
{
  for (int n=12; n >= 0; n-=6)
  {
    uint8_t c = (i >> n) & 63;
    *pbuf++ = (char)(c<26 ? 'A'+c : c<52 ? 'a'-26+c : c<62 ? '0'-52+c : c==62 ? '+' : '/');
  }
}

// Function to convert quaternion to Euler angles
void quaternion_to_euler(int16_t qw, int16_t qx, int16_t qy, int16_t qz, float *roll, float *pitch, float *yaw)
{
  // Convert quaternion to floating point values (normalize)
  float w = qw / 16384.0f;  // Quaternion values are usually in Q14 format (divide by 2^14)
  float x = qx / 16384.0f;
  float y = qy / 16384.0f;
  float z = qz / 16384.0f;
  
  // Calculate roll (x-axis rotation)
  float sinr_cosp = 2 * (w * x + y * z);
  float cosr_cosp = 1 - 2 * (x * x + y * y);
  *roll = atan2(sinr_cosp, cosr_cosp) * 180.0f / M_PI;
  
  // Calculate pitch (y-axis rotation)
  float sinp = 2 * (w * y - z * x);
  if (fabs(sinp) >= 1)
    *pitch = copysign(90.0f, sinp); // Use 90 degrees if out of range
  else
    *pitch = asin(sinp) * 180.0f / M_PI;
    
  // Calculate yaw (z-axis rotation)
  float siny_cosp = 2 * (w * z + x * y);
  float cosy_cosp = 1 - 2 * (y * y + z * z);
  *yaw = atan2(siny_cosp, cosy_cosp) * 180.0f / M_PI;
}

// Export data in CSV format (including Euler angles and calibration status)
static void output_csv_format() {
  // // Get the current time (in seconds, since program startup)
  // float currentTime = (millis() - startTime) / 1000.0f;
  
  // // Start timastamp
  // Serial.print(currentTime, 6);  // 6 decimal places precision
  
  // Data Repprt of each sensor
  for (uint8_t bno = 0; bno < BNOs; bno++) {
    // Quarernion(normalized)
    Serial.print(","); Serial.print(iqw[bno] / 16384.0f, 6);
    Serial.print(","); Serial.print(iqx[bno] / 16384.0f, 6);
    Serial.print(","); Serial.print(iqy[bno] / 16384.0f, 6);
    Serial.print(","); Serial.print(iqz[bno] / 16384.0f, 6);
    
    // Euler angles
    #if PROCESS_EULER_ANGLES
    float roll, pitch, yaw;
    quaternion_to_euler(iqw[bno], iqx[bno], iqy[bno], iqz[bno], &roll, &pitch, &yaw);
    Serial.print(","); Serial.print(roll, 6);
    Serial.print(","); Serial.print(pitch, 6);
    Serial.print(","); Serial.print(yaw, 6);
    #endif
    // Calibration Status
    Serial.print(","); Serial.print(calibration_accuracy[bno]);
    
    #if ALL_REPORTS
    // Accelerometer data
    Serial.print(","); Serial.print(iax[bno] / 256.0f, 6);
    Serial.print(","); Serial.print(iay[bno] / 256.0f, 6);
    Serial.print(","); Serial.print(iaz[bno] / 256.0f, 6);
    
    // Gyroscope data
    Serial.print(","); Serial.print(igx[bno] * 180.0f / (M_PI * 512.0f), 6);
    Serial.print(","); Serial.print(igy[bno] * 180.0f / (M_PI * 512.0f), 6);
    Serial.print(","); Serial.print(igz[bno] * 180.0f / (M_PI * 512.0f), 6);
    
    // Magnetometer data
    Serial.print(","); Serial.print(imx[bno] / 16.0f, 6);
    Serial.print(","); Serial.print(imy[bno] / 16.0f, 6);
    Serial.print(","); Serial.print(imz[bno] / 16.0f, 6);
    
    // Linear acceleration data
    Serial.print(","); Serial.print(ilx[bno] / 256.0f, 6);
    Serial.print(","); Serial.print(ily[bno] / 256.0f, 6);
    Serial.print(","); Serial.print(ilz[bno] / 256.0f, 6);
    #endif
  }
  

  Serial.println();
}

// Print CSV header
static void print_csv_header() {
  // Serial.print("Time(Sec)");
  
  // Add column headers for each sensor
  for (uint8_t bno = 0; bno < BNOs; bno++) {
    // Quaternion title
    Serial.print(",W"); Serial.print(bno);
    Serial.print(",X"); Serial.print(bno);
    Serial.print(",Y"); Serial.print(bno);
    Serial.print(",Z"); Serial.print(bno);
    
    // Euler Angle title
    #if PROCESS_EULER_ANGLES
    Serial.print(",X"); Serial.print(bno);
    Serial.print(",Y"); Serial.print(bno);
    Serial.print(",Z"); Serial.print(bno);
    #endif
    
    // Calibration title
    Serial.print(",Cal"); Serial.print(bno);
    
    #if ALL_REPORTS
    // Accelerometer Title
    Serial.print(",X"); Serial.print(bno);
    Serial.print(",Y"); Serial.print(bno);
    Serial.print(",Z"); Serial.print(bno);
    
    // Gyroscope Title
    Serial.print(",X"); Serial.print(bno);
    Serial.print(",Y"); Serial.print(bno);
    Serial.print(",Z"); Serial.print(bno);
    
    // Magnetometer Title
    Serial.print(",X"); Serial.print(bno);
    Serial.print(",Y"); Serial.print(bno);
    Serial.print(",Z"); Serial.print(bno);
    
    // Linear Acceleration Title
    Serial.print(",X"); Serial.print(bno);
    Serial.print(",Y"); Serial.print(bno);
    Serial.print(",Z"); Serial.print(bno);
    #endif
  }
  
  Serial.println();
}

// ******************************************************************************************************
// **  Receive one SPI byte while simultaneously sending one byte from a queue of pending tx messages  **
// ******************************************************************************************************

#define ACC_REPORT   0x01   // accel report
#define GYRO_REPORT  0x02   // gyro report
#define MAG_REPORT   0x03   // magneto report
#define LAC_REPORT   0x04   // linear accel report
#define QUAT_REPORT  0x05  //#define SENSOR_REPORTID_ROTATION_VECTOR 0x05     #define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08     #define SH2_ARVR_STABILIZED_RV 0x28     #define SH2_GYRO_INTEGRATED_RV 0x2A
#define TIME_REPORT  0xFB   // time report

static const uint8_t req_acc[]  = {21, 0, 2, 0, 0xFD, ACC_REPORT,  0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t req_gyro[] = {21, 0, 2, 0, 0xFD, GYRO_REPORT, 0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t req_mag[]  = {21, 0, 2, 0, 0xFD, MAG_REPORT,  0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t req_lac[]  = {21, 0, 2, 0, 0xFD, LAC_REPORT,  0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t req_quat[] = {21, 0, 2, 0, 0xFD, QUAT_REPORT, 0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};

static const struct TxQueue                 // list of pending tx messages
{
  const uint8_t length;
  const uint8_t *message;
} txqueue[] =
{
 #if ALL_REPORTS
  {sizeof(req_acc),  req_acc},
  {sizeof(req_gyro), req_gyro},
  {sizeof(req_mag),  req_mag},
  {sizeof(req_lac),  req_lac},
 #endif
  {sizeof(req_quat), req_quat},
};

uint8_t txqueue_msg[BNOs];                  // next message to send
uint8_t txqueue_pos[BNOs];                  // next byte to send

static uint8_t recv(uint8_t bno)            // receive one byte while simultaneously sending one byte from current tx message (or zero if no tx message)
{
  uint8_t tx = 0;
  if (txqueue_msg[bno] < numberof(txqueue)) // if more tx messages to send
  {
    if (txqueue_pos[bno] < txqueue[txqueue_msg[bno]].length)  // if current tx message is incomplete
    {
      tx = txqueue[txqueue_msg[bno]].message[txqueue_pos[bno]++];  // byte to send
    }
  }
  uint8_t rx = SPI.transfer(tx);            // send tx, receive rx
  return rx;
}

static void flush(uint8_t bno)              // send all pending tx message bytes, then advance to next tx message
{
  if (txqueue_msg[bno] >= numberof(txqueue)) // if no more tx messages to send
    return;
  if (txqueue_pos[bno])                     // if current tx message is underway
  {
    while (txqueue_pos[bno] < txqueue[txqueue_msg[bno]].length)  // while tx message is incomplete
    {
      uint8_t tx = txqueue[txqueue_msg[bno]].message[txqueue_pos[bno]++];
      SPI.transfer(tx);                     // send tx byte, discard rx byte
    }
    txqueue_msg[bno]++;                     // prepare next tx message
    txqueue_pos[bno] = 0;
  }
}


static void bno_select(uint8_t bno)         // select SPI port of desired BNO
{
  digitalWrite(CS_pins[bno], LOW);
  delayMicroseconds(1);  // Small delay to ensure CS is stable
}

static void bno_deselect(uint8_t bno)       // deselect SPI port of desired BNO
{
  digitalWrite(CS_pins[bno], HIGH);
  delayMicroseconds(1);  // Small delay to ensure CS is stable
}

// ******************************************
// **  Check for and parse sensor reports  **
// ******************************************

// Data Collection Status Variables
static bool header_printed = false;
static uint8_t report_counter[BNOs] = {0};

static void check_report(uint8_t bno)
{
  int16_t length;
  uint8_t channel;
  uint8_t seqnum[BNOs];

  bno_select(bno);
  length = recv(bno);                       // length LSB
  length |= (uint16_t)recv(bno) << 8;       // length MSB
  length &= 0x7FFF;                         // ignore continuation flag
  channel = recv(bno);                      // channel number
  seqnum[bno] = recv(bno);                  // sequence number (ignore)
  length -= 4;                              // done reading SHTP Header

  if (length <= 0 || length > 1000)         // if null/bad/degenerate SHTP header
  {
    flush(bno);
    bno_deselect(bno);
    return;
  }

  while (length)                            // while more reports in cargo
  {
    uint8_t buf[20];                        // report buffer, big enough for longest interesting report
    uint16_t n = 0;                         // index into report buffer

    buf[n++] = recv(bno);                   // first byte of report
    length--;

    // known reports
    if (channel==3 && buf[0]==TIME_REPORT && length >= 5-1)
    {
      for (uint8_t n=1; n<5; n++)           // read remainder of report
      {
        buf[n] = recv(bno);
        length--;
      }
      continue;
    }
    #if ALL_REPORTS
      if (channel==3 && buf[0]==ACC_REPORT && length >= 10-1)
      {
        for (uint8_t n=1; n<10; n++)        // read remainder of report
        {
          buf[n] = recv(bno);
          length--;
        }
        iax[bno] = *(int16_t*)&buf[4];
        iay[bno] = *(int16_t*)&buf[6];
        iaz[bno] = *(int16_t*)&buf[8];
        continue;
      }
      if (channel==3 && buf[0]==GYRO_REPORT && length >= 10-1)
      {
        for (uint8_t n=1; n<10; n++)        // read remainder of report
        {
          buf[n] = recv(bno);
          length--;
        }
        igx[bno] = *(int16_t*)&buf[4];
        igy[bno] = *(int16_t*)&buf[6];
        igz[bno] = *(int16_t*)&buf[8];
        continue;
      }
      if (channel==3 && buf[0]==MAG_REPORT && length >= 10-1)
      {
        for (uint8_t n=1; n<10; n++)        // read remainder of report
        {
          buf[n] = recv(bno);
          length--;
        }
        imx[bno] = *(int16_t*)&buf[4];
        imy[bno] = *(int16_t*)&buf[6];
        imz[bno] = *(int16_t*)&buf[8];
        continue;
      }
      if (channel==3 && buf[0]==LAC_REPORT && length >= 10-1)
      {
        for (uint8_t n=1; n<10; n++)        // read remainder of report
        {
          buf[n] = recv(bno);
          length--;
        }
        ilx[bno] = *(int16_t*)&buf[4];
        ily[bno] = *(int16_t*)&buf[6];
        ilz[bno] = *(int16_t*)&buf[8];
        continue;
      }
    #endif
    if (channel==3 && buf[0]==QUAT_REPORT && length >= 14-1)
    {
      for (uint8_t n=1; n<14; n++)          // read remainder of report
      {
        buf[n] = recv(bno);
        length--;
      }
      iqw[bno] = *(int16_t*)&buf[10];
      iqx[bno] = *(int16_t*)&buf[4];
      iqy[bno] = *(int16_t*)&buf[6];
      iqz[bno] = *(int16_t*)&buf[8];
      
      // Get calibration accuracy status (buf[2] contains calibration accuracy)
      calibration_accuracy[bno] = buf[2];
      
      // Increment the counter only when a quaternion report is received
      report_counter[bno]++;
      
      // Printing header for the first time
      if (!header_printed) {
        print_csv_header();
        header_printed = true;
      }
      
      // Check if all sensors are reporting
      bool all_reported = true;
      for (uint8_t i = 0; i < BNOs; i++) {
        if (report_counter[i] == 0) {
          all_reported = false;
          break;
        }
      }
      
      // f all sensors are reporting, export data in CSV format
      if (all_reported) {
        output_csv_format();
        // reset counter
        for (uint8_t i = 0; i < BNOs; i++) {
          report_counter[i] = 0;
        }
      }
      continue;
    }

    // unwanted reports
    if (channel==0 && buf[0]==0)
    {
      while (length)                        // discard remainder of cargo
      {
        recv(bno);
        length--;
      }
      continue;
    }
    if (channel==1 && buf[0]==1)
    {
      while (length)                        // discard remainder of cargo
      {
        recv(bno);
        length--;
      }
      continue;
    }
    if (channel==2 && buf[0]==0xF1)
    {
      while (length)                        // discard remainder of cargo
      {
        recv(bno);
        length--;
      }
      continue;
    }
    if (channel==2 && buf[0]==0xFC)
    {
      while (length)                        // discard remainder of cargo
      {
        recv(bno);
        length--;
      }
      continue;
    }
    while (length)                          // discard remainder of cargo
    {
      recv(bno);
      length--;
    }
    continue;
  }
  flush(bno);
  bno_deselect(bno);

  return;
}

// **********************
// **  Setup and Loop  **
// **********************

uint8_t volatile int_count[BNOs];
uint8_t          int_counted[BNOs];

// Interrupt service routines for the three BNO085 sensors
void isr0() { int_count[0]++; }
void isr1() { int_count[1]++; }
void isr2() { int_count[2]++; }

void setup()
{
  Serial.begin(SERIAL_BAUD);                // initialize serial
  // Wait for serial port to connect or timeout after 3 seconds
  uint32_t start = millis();
  while (!Serial && (millis() - start < 3000));
  
  // // program start time
  // startTime = millis();

  // Initialize pin modes
  pinMode(pinSCL, OUTPUT);
  pinMode(pinSDA, INPUT);
  pinMode(pinDI, OUTPUT);

  // Initialize CS pins and deselect all BNOs
  for (uint8_t bno=0; bno<BNOs; bno++) {
    pinMode(CS_pins[bno], OUTPUT);
    digitalWrite(CS_pins[bno], HIGH);
  }

  // Setup P0 and P1 for SPI mode if needed
  // The BNO085 datasheet indicates P0 and P1 need to be set to 3V3 for SPI mode
  pinMode(36, OUTPUT);  // 3V3 pin for reference
  digitalWrite(36, HIGH);

  // Initialize SPI
  SPI.begin();
  SPI.beginTransaction(SPI_SETTINGS);

  // Reset all BNO085 sensors
  for (uint8_t bno=0; bno<BNOs; bno++) {
    pinMode(RST_pins[bno], OUTPUT);
    digitalWrite(RST_pins[bno], LOW);
  }
  delay(10);
  for (uint8_t bno=0; bno<BNOs; bno++) {
    digitalWrite(RST_pins[bno], HIGH);
  }
  delay(500);  // Increased reset recovery time
  delay(100);  // Additional stability delay

  // Set up interrupt pins
  pinMode(pinINT1, INPUT_PULLUP);  // Use pullup for stability
  pinMode(pinINT2, INPUT_PULLUP);  // Use pullup for stability
  pinMode(pinINT3, INPUT_PULLUP);  // Use pullup for stability

  attachInterrupt(digitalPinToInterrupt(pinINT1), isr0, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinINT2), isr1, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinINT3), isr2, FALLING);
  
  // Print CSV header
  print_csv_header();
  
  // Initialize the calibration status of all sensors
  for (uint8_t bno = 0; bno < BNOs; bno++) {
    calibration_accuracy[bno] = 0;
  }
}

void loop()
{
  // Check for interrupt-driven report requests
  for (uint8_t bno=0; bno<BNOs; bno++)
  {
    if (int_count[bno] != int_counted[bno]) // if transfer request
    {
      int_counted[bno] = int_count[bno];    // acknowledge all interrupt(s)
      check_report(bno);
    }
  }
}







// // Talks to multiple BNO085 sensors via SPI for Raspberry Pi Pico.
// // Supports 3 BNO085 sensors with a single RESET pin.

// #include <SPI.h>

// #define BNOs        3           // number of BNO08x breakouts
// #define pinRST      21          // GP21 connects to RST of all BNO085 sensors
// #define pinsINT     8,13,20     // GP8,GP13,GP20 connect to INT of BNO0,BNO1,BNO2
// #define pinsCS      7,15,17     // GP7,GP15,GP17 connect to CS of BNO0,BNO1,BNO2

// // SPI pin definitions
// #define pinSCL      18          // GP18 - SCL/SCLK
// #define pinSDA      16          // GP16 - SDA/MISO
// #define pinDI       19          // GP19 - DI/MOSI

// #define ALL_REPORTS 0           // 0 request only quaternions, 1 request all reports
// #define REPORT_TIME 2500L      // time between sensor reports (increased for reliability)
// #define SPI_CLOCK   3000000L    // SPI clock rate (reduced for stability)
// #define SERIAL_BAUD 230400L     // serial port baud rate

// // Enable output mode
// #define DATAOUT     1           // 1 enables data output messages

// #define SPI_SETTINGS SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3)
// #define numberof(x)  (sizeof(x) / sizeof(*(x)))

// // *******************
// // **  Output data  **
// // *******************

// int16_t iax[BNOs], iay[BNOs], iaz[BNOs];             // accel, integer
// int16_t igx[BNOs], igy[BNOs], igz[BNOs];             // gyro, integer
// int16_t imx[BNOs], imy[BNOs], imz[BNOs];             // magneto, integer
// int16_t ilx[BNOs], ily[BNOs], ilz[BNOs];             // linear accel, integer
// int16_t iqw[BNOs], iqx[BNOs], iqy[BNOs], iqz[BNOs];  // quaternion, integer

// char obuf[70], *pbuf;               // ensure this output buffer is big enough for your output string!

// static void uart_b64(int32_t i)     // output 18-bit integer as compact 3-digit base64
// {
//   for (int n=12; n >= 0; n-=6)
//   {
//     uint8_t c = (i >> n) & 63;
//     *pbuf++ = (char)(c<26 ? 'A'+c : c<52 ? 'a'-26+c : c<62 ? '0'-52+c : c==62 ? '+' : '/');
//   }
// }

// #if ALL_REPORTS
//   static void output_data(uint8_t bno)
//   {
//     float kACC = 1/9.80665/256 * 131072/10.0;   // scale units for my project
//     float kGYR =  180/M_PI/512 * 131072/4000.0;
//     float kMAG =       0.01/16 * 131072/1.0;
//     float kLAC = 1/9.80665/256 * 131072/10.0;

//     pbuf = obuf;                        // pointer into output buffer
//     *pbuf++ = 'k';  *pbuf++ = 'q'+bno;  // string header "kq" is BNO0, "kr" is BNO1, "ks" is BNO2, etc
//     uart_b64(kACC*iax[bno]);  uart_b64(-kACC*iay[bno]);  uart_b64(-kACC*iaz[bno]);  // accel,   convert from m/sec/sec*256 to       g*131072/10.0
//     uart_b64(kGYR*igx[bno]);  uart_b64(-kGYR*igy[bno]);  uart_b64(-kGYR*igz[bno]);  // gyro,    convert from   rad/sec*512 to deg/sec*131072/4000.0
//     uart_b64(kMAG*imx[bno]);  uart_b64(-kMAG*imy[bno]);  uart_b64(-kMAG*imz[bno]);  // magneto, convert from        uT*16  to   gauss*131072/1.0
//     uart_b64(kLAC*ilx[bno]);  uart_b64(-kLAC*ily[bno]);  uart_b64(-kLAC*ilz[bno]);  // linacc,  convert from m/sec/sec*256 to       g*131072/10.0
//     uart_b64(iqw[bno]+iqz[bno]); uart_b64(iqx[bno]+iqy[bno]); uart_b64(iqx[bno]-iqy[bno]); uart_b64(iqw[bno]-iqz[bno]);  // quat,    rotate quat to my reference frame, do no conversion
//     uart_b64(0);            // temp,    convert from    degC*128     to  degC*131072/100.0
//     uart_b64(0);            // baro,    convert from hectoPa*1048576 to  mbar*131072/2000.0
//     uart_b64(0xFF);         // status,  four 2-bit codes {sys,gyr,acc,mag}
//     *pbuf++ = 13;           // CR LF
//     *pbuf++ = 10;
//     *pbuf++ = 0;            // terminate string
//     Serial.write(obuf);     // writing one long string is *much* faster than printing individual values
//   }
// #else
//   static void output_data(uint8_t bno)
//   {
//     pbuf = obuf;                        // pointer into output buffer
//     *pbuf++ = 'k';  *pbuf++ = 'q'+bno;  // string header "kq" is BNO0, "kr" is BNO1, "ks" is BNO2, etc
//     // my BNO08x orientation dot is towards left rear, rotate BNO08x quaternion to NED conventions
//     uart_b64(iqw[bno]+iqz[bno]); uart_b64(iqx[bno]+iqy[bno]); uart_b64(iqx[bno]-iqy[bno]); uart_b64(iqw[bno]-iqz[bno]);  // quat
//     *pbuf++ = 13;           // CR LF
//     *pbuf++ = 10;
//     *pbuf++ = 0;            // terminate string
//     Serial.write(obuf);     // writing one long string is *much* faster than printing individual values
//   }
// #endif

// // ******************************************************************************************************
// // **  Receive one SPI byte while simultaneously sending one byte from a queue of pending tx messages  **
// // ******************************************************************************************************

// #define ACC_REPORT   0x01   // accel report, see 6.5.9
// #define GYRO_REPORT  0x02   // gyro report, see 6.5.13
// #define MAG_REPORT   0x03   // magneto report, see 6.5.16
// #define LAC_REPORT   0x04   // linear accel report, see 6.5.10
// #define QUAT_REPORT  0x05   // quaternion report, see 6.5.18
// #define TIME_REPORT  0xFB   // time report, see 7.2.1

// static const uint8_t req_acc[]  = {21, 0, 2, 0, 0xFD, ACC_REPORT,  0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};
// static const uint8_t req_gyro[] = {21, 0, 2, 0, 0xFD, GYRO_REPORT, 0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};
// static const uint8_t req_mag[]  = {21, 0, 2, 0, 0xFD, MAG_REPORT,  0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};
// static const uint8_t req_lac[]  = {21, 0, 2, 0, 0xFD, LAC_REPORT,  0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};
// static const uint8_t req_quat[] = {21, 0, 2, 0, 0xFD, QUAT_REPORT, 0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};

// static const struct TxQueue                 // list of pending tx messages
// {
//   const uint8_t length;
//   const uint8_t *message;
// } txqueue[] =
// {
//  #if ALL_REPORTS
//   {sizeof(req_acc),  req_acc},
//   {sizeof(req_gyro), req_gyro},
//   {sizeof(req_mag),  req_mag},
//   {sizeof(req_lac),  req_lac},
//  #endif
//   {sizeof(req_quat), req_quat},
// };

// uint8_t txqueue_msg[BNOs];                  // next message to send
// uint8_t txqueue_pos[BNOs];                  // next byte to send

// static uint8_t recv(uint8_t bno)            // receive one byte while simultaneously sending one byte from current tx message (or zero if no tx message)
// {
//   uint8_t tx = 0;
//   if (txqueue_msg[bno] < numberof(txqueue)) // if more tx messages to send
//   {
//     if (txqueue_pos[bno] < txqueue[txqueue_msg[bno]].length)  // if current tx message is incomplete
//     {
//       tx = txqueue[txqueue_msg[bno]].message[txqueue_pos[bno]++];  // byte to send
//     }
//   }
//   uint8_t rx = SPI.transfer(tx);            // send tx, receive rx
//   return rx;
// }

// static void flush(uint8_t bno)              // send all pending tx message bytes, then advance to next tx message
// {
//   if (txqueue_msg[bno] >= numberof(txqueue)) // if no more tx messages to send
//     return;
//   if (txqueue_pos[bno])                     // if current tx message is underway
//   {
//     while (txqueue_pos[bno] < txqueue[txqueue_msg[bno]].length)  // while tx message is incomplete
//     {
//       uint8_t tx = txqueue[txqueue_msg[bno]].message[txqueue_pos[bno]++];
//       SPI.transfer(tx);                     // send tx byte, discard rx byte
//     }
//     txqueue_msg[bno]++;                     // prepare next tx message
//     txqueue_pos[bno] = 0;
//   }
// }

// static const decltype(pinRST) CS[]  = {pinsCS};   // array of CS pin numbers for all BNOs
// static const decltype(pinRST) INT[] = {pinsINT};  // array of INT pin numbers for all BNOs

// static void bno_select(uint8_t bno)         // select SPI port of desired BNO
// {
//   digitalWrite(CS[bno], LOW);
//   delayMicroseconds(1);  // Small delay to ensure CS is stable
// }

// static void bno_deselect(uint8_t bno)       // deselect SPI port of desired BNO
// {
//   digitalWrite(CS[bno], HIGH);
//   delayMicroseconds(1);  // Small delay to ensure CS is stable
// }

// // ******************************************
// // **  Check for and parse sensor reports  **
// // ******************************************
// static void check_report(uint8_t bno)
// {
//   int16_t length;
//   uint8_t channel;
//   uint8_t seqnum[BNOs];

//   bno_select(bno);
//   length = recv(bno);                       // length LSB
//   length |= (uint16_t)recv(bno) << 8;       // length MSB
//   length &= 0x7FFF;                         // ignore continuation flag
//   channel = recv(bno);                      // channel number
//   seqnum[bno] = recv(bno);                  // sequence number (ignore)
//   length -= 4;                              // done reading SHTP Header

//   if (length <= 0 || length > 1000)         // if null/bad/degenerate SHTP header
//   {
//     flush(bno);
//     bno_deselect(bno);
//     return;
//   }

//   while (length)                            // while more reports in cargo
//   {
//     uint8_t buf[20];                        // report buffer, big enough for longest interesting report (uninteresting reports will be ignored)
//     uint16_t n = 0;                         // index into report buffer

//     buf[n++] = recv(bno);                   // first byte of report
//     length--;

//     // known reports
//     if (channel==3 && buf[0]==TIME_REPORT && length >= 5-1)
//     {
//       for (uint8_t n=1; n<5; n++)           // read remainder of report
//       {
//         buf[n] = recv(bno);
//         length--;
//       }
//       continue;
//     }
//     #if ALL_REPORTS
//       if (channel==3 && buf[0]==ACC_REPORT && length >= 10-1)
//       {
//         for (uint8_t n=1; n<10; n++)        // read remainder of report
//         {
//           buf[n] = recv(bno);
//           length--;
//         }
//         iax[bno] = *(int16_t*)&buf[4];
//         iay[bno] = *(int16_t*)&buf[6];
//         iaz[bno] = *(int16_t*)&buf[8];
//         continue;
//       }
//       if (channel==3 && buf[0]==GYRO_REPORT && length >= 10-1)
//       {
//         for (uint8_t n=1; n<10; n++)        // read remainder of report
//         {
//           buf[n] = recv(bno);
//           length--;
//         }
//         igx[bno] = *(int16_t*)&buf[4];
//         igy[bno] = *(int16_t*)&buf[6];
//         igz[bno] = *(int16_t*)&buf[8];
//         continue;
//       }
//       if (channel==3 && buf[0]==MAG_REPORT && length >= 10-1)
//       {
//         for (uint8_t n=1; n<10; n++)        // read remainder of report
//         {
//           buf[n] = recv(bno);
//           length--;
//         }
//         imx[bno] = *(int16_t*)&buf[4];
//         imy[bno] = *(int16_t*)&buf[6];
//         imz[bno] = *(int16_t*)&buf[8];
//         continue;
//       }
//       if (channel==3 && buf[0]==LAC_REPORT && length >= 10-1)
//       {
//         for (uint8_t n=1; n<10; n++)        // read remainder of report
//         {
//           buf[n] = recv(bno);
//           length--;
//         }
//         ilx[bno] = *(int16_t*)&buf[4];
//         ily[bno] = *(int16_t*)&buf[6];
//         ilz[bno] = *(int16_t*)&buf[8];
//         continue;
//       }
//     #endif
//     if (channel==3 && buf[0]==QUAT_REPORT && length >= 14-1)
//     {
//       for (uint8_t n=1; n<14; n++)          // read remainder of report
//       {
//         buf[n] = recv(bno);
//         length--;
//       }
//       iqw[bno] = *(int16_t*)&buf[10];
//       iqx[bno] = *(int16_t*)&buf[4];
//       iqy[bno] = *(int16_t*)&buf[6];
//       iqz[bno] = *(int16_t*)&buf[8];
//       if (DATAOUT) {output_data(bno);}      // output data message
//       continue;
//     }

//     // unwanted reports
//     if (channel==0 && buf[0]==0)
//     {
//       while (length)                        // discard remainder of cargo
//       {
//         recv(bno);
//         length--;
//       }
//       continue;
//     }
//     if (channel==1 && buf[0]==1)
//     {
//       while (length)                        // discard remainder of cargo
//       {
//         recv(bno);
//         length--;
//       }
//       continue;
//     }
//     if (channel==2 && buf[0]==0xF1)
//     {
//       while (length)                        // discard remainder of cargo
//       {
//         recv(bno);
//         length--;
//       }
//       continue;
//     }
//     if (channel==2 && buf[0]==0xFC)
//     {
//       while (length)                        // discard remainder of cargo
//       {
//         recv(bno);
//         length--;
//       }
//       continue;
//     }
//     while (length)                          // discard remainder of cargo
//     {
//       recv(bno);
//       length--;
//     }
//     continue;
//   }
//   flush(bno);
//   bno_deselect(bno);

//   return;
// }

// // **********************
// // **  Setup and Loop  **
// // **********************

// uint8_t volatile int_count[BNOs];
// uint8_t          int_counted[BNOs];

// // Interrupt service routines for the three BNO085 sensors
// void isr0() { int_count[0]++; }
// void isr1() { int_count[1]++; }
// void isr2() { int_count[2]++; }

// void setup()
// {
//   Serial.begin(SERIAL_BAUD);                // initialize serial
//   // Wait for serial port to connect or timeout after 3 seconds
//   uint32_t start = millis();
//   while (!Serial && (millis() - start < 3000));
  
//   Serial.println("\nTriple BNO085 Sensor on Pico Running...");
//   Serial.println("Hardware Initialization...");

//   // Initialize pin modes for SPI pins
//   pinMode(pinSCL, OUTPUT);
//   pinMode(pinSDA, INPUT);
//   pinMode(pinDI, OUTPUT);

//   // Initialize CS pins and deselect all BNOs
//   for (uint8_t bno=0; bno<BNOs; bno++) {
//     pinMode(CS[bno], OUTPUT);
//     digitalWrite(CS[bno], HIGH);
//     Serial.print("CS"); Serial.print(bno); Serial.print(" pin = GP"); Serial.println(CS[bno]);
//   }

//   // Initialize SPI
//   SPI.begin();
  
//   // Reset all BNO085 sensors using single reset pin
//   Serial.println("Resetting all BNO085 sensors...");
//   pinMode(pinRST, OUTPUT);
//   digitalWrite(pinRST, LOW);
//   delay(10);  // longer reset pulse
//   digitalWrite(pinRST, HIGH);
//   delay(500);  // Longer reset recovery time
//   Serial.println("Reset complete, waiting for stability...");
//   delay(100);  // Additional stability delay

//   // Set up interrupt pins with pull-ups
//   Serial.println("Setting up interrupts...");
//   pinMode(INT[0], INPUT_PULLUP);
//   pinMode(INT[1], INPUT_PULLUP);
//   pinMode(INT[2], INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(INT[0]), isr0, FALLING);
//   attachInterrupt(digitalPinToInterrupt(INT[1]), isr1, FALLING);
//   attachInterrupt(digitalPinToInterrupt(INT[2]), isr2, FALLING);
//   Serial.print("INT0 pin = GP"); Serial.println(INT[0]);
//   Serial.print("INT1 pin = GP"); Serial.println(INT[1]);
//   Serial.print("INT2 pin = GP"); Serial.println(INT[2]);
  
//   // Initialize each sensor one at a time
//   Serial.println("Initializing sensors...");
//   for (uint8_t bno=0; bno<BNOs; bno++) {
//     Serial.print("Initializing BNO ");
//     Serial.println(bno);
    
//     // Reset the transmit queue for this sensor
//     txqueue_msg[bno] = 0;
//     txqueue_pos[bno] = 0;
    
//     SPI.beginTransaction(SPI_SETTINGS);
    
//     // Send configuration messages
//     for (uint8_t i = 0; i < numberof(txqueue); i++) {
//       bno_select(bno);
//       for (uint8_t j = 0; j < txqueue[i].length; j++) {
//         SPI.transfer(txqueue[i].message[j]);
//       }
//       bno_deselect(bno);
//       delay(50);  // Give sensor time to process each message
//     }
    
//     SPI.endTransaction();
    
//     // Wait a bit between sensors
//     delay(200);
//   }

//   Serial.println("Initialization complete, beginning sensor monitoring...");

// }

// void loop()
// {
//   // Check for interrupt-driven report requests
//   for (uint8_t bno=0; bno<BNOs; bno++)
//   {
//     if (int_count[bno] != int_counted[bno]) // if transfer request
//     {
//       int_counted[bno] = int_count[bno];    // acknowledge all interrupt(s)
//       SPI.beginTransaction(SPI_SETTINGS);
//       check_report(bno);
//       SPI.endTransaction();
//     }
//   }
  
// }


// Talks to multiple BNO085 sensors via SPI for Raspberry Pi Pico.
// Supports 3 BNO085 sensors with a single RESET pin.

// #include <SPI.h>

// #define BNOs        3           // number of BNO08x breakouts
// #define pinRST      21          // GP21 connects to RST of all BNO085 sensors
// #define pinsINT     8,13,20     // GP8,GP13,GP20 connect to INT of BNO0,BNO1,BNO2
// #define pinsCS      7,15,17     // GP7,GP15,GP17 connect to CS of BNO0,BNO1,BNO2

// // SPI pin definitions
// #define pinSCL      18          // GP18 - SCL/SCLK
// #define pinSDA      16          // GP16 - SDA/MISO
// #define pinDI       19          // GP19 - DI/MOSI

// #define ALL_REPORTS 0           // 0 request only quaternions, 1 request all reports
// #define REPORT_TIME 2500L      // time between sensor reports (increased for reliability)
// #define SPI_CLOCK   3000000L    // SPI clock rate (reduced for stability)
// #define SERIAL_BAUD 230400L     // serial port baud rate

// // Enable output mode
// #define DATAOUT     1           // 1 enables data output messages

// // Add synchronized data output
// #define SYNC_OUTPUT 1           // 1 enables synchronized data output for all sensors
// #define SYNC_PERIOD 10         // time between synchronized outputs (milliseconds)

// #define SPI_SETTINGS SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3)
// #define numberof(x)  (sizeof(x) / sizeof(*(x)))

// // *******************
// // **  Output data  **
// // *******************

// int16_t iax[BNOs], iay[BNOs], iaz[BNOs];             // accel, integer
// int16_t igx[BNOs], igy[BNOs], igz[BNOs];             // gyro, integer
// int16_t imx[BNOs], imy[BNOs], imz[BNOs];             // magneto, integer
// int16_t ilx[BNOs], ily[BNOs], ilz[BNOs];             // linear accel, integer
// int16_t iqw[BNOs], iqx[BNOs], iqy[BNOs], iqz[BNOs];  // quaternion, integer

// bool data_ready[BNOs] = {false, false, false};       // flags to track which sensors have new data
// uint32_t last_sync_output = 0;                       // timer for synchronized output

// char obuf[70], *pbuf;               // ensure this output buffer is big enough for your output string!

// static void uart_b64(int32_t i)     // output 18-bit integer as compact 3-digit base64
// {
//   for (int n=12; n >= 0; n-=6)
//   {
//     uint8_t c = (i >> n) & 63;
//     *pbuf++ = (char)(c<26 ? 'A'+c : c<52 ? 'a'-26+c : c<62 ? '0'-52+c : c==62 ? '+' : '/');
//   }
// }

// #if ALL_REPORTS
//   static void output_data(uint8_t bno)
//   {
//     float kACC = 1/9.80665/256 * 131072/10.0;   // scale units for my project
//     float kGYR =  180/M_PI/512 * 131072/4000.0;
//     float kMAG =       0.01/16 * 131072/1.0;
//     float kLAC = 1/9.80665/256 * 131072/10.0;

//     pbuf = obuf;                        // pointer into output buffer
//     *pbuf++ = 'k';  *pbuf++ = 'q'+bno;  // string header "kq" is BNO0, "kr" is BNO1, "ks" is BNO2, etc
//     uart_b64(kACC*iax[bno]);  uart_b64(-kACC*iay[bno]);  uart_b64(-kACC*iaz[bno]);  // accel,   convert from m/sec/sec*256 to       g*131072/10.0
//     uart_b64(kGYR*igx[bno]);  uart_b64(-kGYR*igy[bno]);  uart_b64(-kGYR*igz[bno]);  // gyro,    convert from   rad/sec*512 to deg/sec*131072/4000.0
//     uart_b64(kMAG*imx[bno]);  uart_b64(-kMAG*imy[bno]);  uart_b64(-kMAG*imz[bno]);  // magneto, convert from        uT*16  to   gauss*131072/1.0
//     uart_b64(kLAC*ilx[bno]);  uart_b64(-kLAC*ily[bno]);  uart_b64(-kLAC*ilz[bno]);  // linacc,  convert from m/sec/sec*256 to       g*131072/10.0
//     uart_b64(iqw[bno]+iqz[bno]); uart_b64(iqx[bno]+iqy[bno]); uart_b64(iqx[bno]-iqy[bno]); uart_b64(iqw[bno]-iqz[bno]);  // quat,    rotate quat to my reference frame, do no conversion
//     uart_b64(0);            // temp,    convert from    degC*128     to  degC*131072/100.0
//     uart_b64(0);            // baro,    convert from hectoPa*1048576 to  mbar*131072/2000.0
//     uart_b64(0xFF);         // status,  four 2-bit codes {sys,gyr,acc,mag}
//     *pbuf++ = 13;           // CR LF
//     *pbuf++ = 10;
//     *pbuf++ = 0;            // terminate string
//     Serial.write(obuf);     // writing one long string is *much* faster than printing individual values
//   }
// #else
//   static void output_data(uint8_t bno)
//   {
//     pbuf = obuf;                        // pointer into output buffer
//     *pbuf++ = 'k';  *pbuf++ = 'q'+bno;  // string header "kq" is BNO0, "kr" is BNO1, "ks" is BNO2, etc
//     // my BNO08x orientation dot is towards left rear, rotate BNO08x quaternion to NED conventions
//     uart_b64(iqw[bno]+iqz[bno]); uart_b64(iqx[bno]+iqy[bno]); uart_b64(iqx[bno]-iqy[bno]); uart_b64(iqw[bno]-iqz[bno]);  // quat
//     *pbuf++ = 13;           // CR LF
//     *pbuf++ = 10;
//     *pbuf++ = 0;            // terminate string
//     Serial.write(obuf);     // writing one long string is *much* faster than printing individual values
//   }
// #endif

// // Function to output synchronized data from all sensors
// static void output_synchronized_data() 
// {
//   // First, output timestamp (optional, but helps with parsing)
//   char timestamp[20];
//   sprintf(timestamp, "%lu", millis());
//   Serial.print(timestamp);
//   Serial.print(" -> ");
  
//   // Output data for each sensor in order
//   for (uint8_t bno = 0; bno < BNOs; bno++) {
//     output_data(bno);
//   }
  
//   // Optional: add a separator between complete frames
//   Serial.println("---");  // Frame separator
// }

// // ******************************************************************************************************
// // **  Receive one SPI byte while simultaneously sending one byte from a queue of pending tx messages  **
// // ******************************************************************************************************

// #define ACC_REPORT   0x01   // accel report, see 6.5.9
// #define GYRO_REPORT  0x02   // gyro report, see 6.5.13
// #define MAG_REPORT   0x03   // magneto report, see 6.5.16
// #define LAC_REPORT   0x04   // linear accel report, see 6.5.10
// #define QUAT_REPORT  0x05   // quaternion report, see 6.5.18
// #define TIME_REPORT  0xFB   // time report, see 7.2.1

// static const uint8_t req_acc[]  = {21, 0, 2, 0, 0xFD, ACC_REPORT,  0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};
// static const uint8_t req_gyro[] = {21, 0, 2, 0, 0xFD, GYRO_REPORT, 0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};
// static const uint8_t req_mag[]  = {21, 0, 2, 0, 0xFD, MAG_REPORT,  0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};
// static const uint8_t req_lac[]  = {21, 0, 2, 0, 0xFD, LAC_REPORT,  0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};
// static const uint8_t req_quat[] = {21, 0, 2, 0, 0xFD, QUAT_REPORT, 0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};

// static const struct TxQueue                 // list of pending tx messages
// {
//   const uint8_t length;
//   const uint8_t *message;
// } txqueue[] =
// {
//  #if ALL_REPORTS
//   {sizeof(req_acc),  req_acc},
//   {sizeof(req_gyro), req_gyro},
//   {sizeof(req_mag),  req_mag},
//   {sizeof(req_lac),  req_lac},
//  #endif
//   {sizeof(req_quat), req_quat},
// };

// uint8_t txqueue_msg[BNOs];                  // next message to send
// uint8_t txqueue_pos[BNOs];                  // next byte to send

// static uint8_t recv(uint8_t bno)            // receive one byte while simultaneously sending one byte from current tx message (or zero if no tx message)
// {
//   uint8_t tx = 0;
//   if (txqueue_msg[bno] < numberof(txqueue)) // if more tx messages to send
//   {
//     if (txqueue_pos[bno] < txqueue[txqueue_msg[bno]].length)  // if current tx message is incomplete
//     {
//       tx = txqueue[txqueue_msg[bno]].message[txqueue_pos[bno]++];  // byte to send
//     }
//   }
//   uint8_t rx = SPI.transfer(tx);            // send tx, receive rx
//   return rx;
// }

// static void flush(uint8_t bno)              // send all pending tx message bytes, then advance to next tx message
// {
//   if (txqueue_msg[bno] >= numberof(txqueue)) // if no more tx messages to send
//     return;
//   if (txqueue_pos[bno])                     // if current tx message is underway
//   {
//     while (txqueue_pos[bno] < txqueue[txqueue_msg[bno]].length)  // while tx message is incomplete
//     {
//       uint8_t tx = txqueue[txqueue_msg[bno]].message[txqueue_pos[bno]++];
//       SPI.transfer(tx);                     // send tx byte, discard rx byte
//     }
//     txqueue_msg[bno]++;                     // prepare next tx message
//     txqueue_pos[bno] = 0;
//   }
// }

// static const decltype(pinRST) CS[]  = {pinsCS};   // array of CS pin numbers for all BNOs
// static const decltype(pinRST) INT[] = {pinsINT};  // array of INT pin numbers for all BNOs

// static void bno_select(uint8_t bno)         // select SPI port of desired BNO
// {
//   digitalWrite(CS[bno], LOW);
//   delayMicroseconds(1);  // Small delay to ensure CS is stable
// }

// static void bno_deselect(uint8_t bno)       // deselect SPI port of desired BNO
// {
//   digitalWrite(CS[bno], HIGH);
//   delayMicroseconds(1);  // Small delay to ensure CS is stable
// }

// // ******************************************
// // **  Check for and parse sensor reports  **
// // ******************************************

// static uint32_t poll_last = 0;              // polling timer

// static void check_report(uint8_t bno)
// {
//   int16_t length;
//   uint8_t channel;
//   uint8_t seqnum[BNOs];

//   bno_select(bno);
//   length = recv(bno);                       // length LSB
//   length |= (uint16_t)recv(bno) << 8;       // length MSB
//   length &= 0x7FFF;                         // ignore continuation flag
//   channel = recv(bno);                      // channel number
//   seqnum[bno] = recv(bno);                  // sequence number (ignore)
//   length -= 4;                              // done reading SHTP Header

//   if (length <= 0 || length > 1000)         // if null/bad/degenerate SHTP header
//   {
//     flush(bno);
//     bno_deselect(bno);
//     return;
//   }

//   while (length)                            // while more reports in cargo
//   {
//     uint8_t buf[20];                        // report buffer, big enough for longest interesting report (uninteresting reports will be ignored)
//     uint16_t n = 0;                         // index into report buffer

//     buf[n++] = recv(bno);                   // first byte of report
//     length--;

//     // known reports
//     if (channel==3 && buf[0]==TIME_REPORT && length >= 5-1)
//     {
//       for (uint8_t n=1; n<5; n++)           // read remainder of report
//       {
//         buf[n] = recv(bno);
//         length--;
//       }
//       continue;
//     }
//     #if ALL_REPORTS
//       if (channel==3 && buf[0]==ACC_REPORT && length >= 10-1)
//       {
//         for (uint8_t n=1; n<10; n++)        // read remainder of report
//         {
//           buf[n] = recv(bno);
//           length--;
//         }
//         iax[bno] = *(int16_t*)&buf[4];
//         iay[bno] = *(int16_t*)&buf[6];
//         iaz[bno] = *(int16_t*)&buf[8];
//         continue;
//       }
//       if (channel==3 && buf[0]==GYRO_REPORT && length >= 10-1)
//       {
//         for (uint8_t n=1; n<10; n++)        // read remainder of report
//         {
//           buf[n] = recv(bno);
//           length--;
//         }
//         igx[bno] = *(int16_t*)&buf[4];
//         igy[bno] = *(int16_t*)&buf[6];
//         igz[bno] = *(int16_t*)&buf[8];
//         continue;
//       }
//       if (channel==3 && buf[0]==MAG_REPORT && length >= 10-1)
//       {
//         for (uint8_t n=1; n<10; n++)        // read remainder of report
//         {
//           buf[n] = recv(bno);
//           length--;
//         }
//         imx[bno] = *(int16_t*)&buf[4];
//         imy[bno] = *(int16_t*)&buf[6];
//         imz[bno] = *(int16_t*)&buf[8];
//         continue;
//       }
//       if (channel==3 && buf[0]==LAC_REPORT && length >= 10-1)
//       {
//         for (uint8_t n=1; n<10; n++)        // read remainder of report
//         {
//           buf[n] = recv(bno);
//           length--;
//         }
//         ilx[bno] = *(int16_t*)&buf[4];
//         ily[bno] = *(int16_t*)&buf[6];
//         ilz[bno] = *(int16_t*)&buf[8];
//         continue;
//       }
//     #endif
//     if (channel==3 && buf[0]==QUAT_REPORT && length >= 14-1)
//     {
//       for (uint8_t n=1; n<14; n++)          // read remainder of report
//       {
//         buf[n] = recv(bno);
//         length--;
//       }
//       iqw[bno] = *(int16_t*)&buf[10];
//       iqx[bno] = *(int16_t*)&buf[4];
//       iqy[bno] = *(int16_t*)&buf[6];
//       iqz[bno] = *(int16_t*)&buf[8];
      
//       #if SYNC_OUTPUT
//         // Instead of immediately outputting data, mark this sensor as having new data
//         data_ready[bno] = true;
//       #else
//         // Original behavior - output immediately
//         if (DATAOUT) {output_data(bno);}      // output data message
//       #endif
      
//       continue;
//     }

//     // unwanted reports
//     if (channel==0 && buf[0]==0)
//     {
//       while (length)                        // discard remainder of cargo
//       {
//         recv(bno);
//         length--;
//       }
//       continue;
//     }
//     if (channel==1 && buf[0]==1)
//     {
//       while (length)                        // discard remainder of cargo
//       {
//         recv(bno);
//         length--;
//       }
//       continue;
//     }
//     if (channel==2 && buf[0]==0xF1)
//     {
//       while (length)                        // discard remainder of cargo
//       {
//         recv(bno);
//         length--;
//       }
//       continue;
//     }
//     if (channel==2 && buf[0]==0xFC)
//     {
//       while (length)                        // discard remainder of cargo
//       {
//         recv(bno);
//         length--;
//       }
//       continue;
//     }
//     while (length)                          // discard remainder of cargo
//     {
//       recv(bno);
//       length--;
//     }
//     continue;
//   }
//   flush(bno);
//   bno_deselect(bno);

//   return;
// }

// // **********************
// // **  Setup and Loop  **
// // **********************

// uint8_t volatile int_count[BNOs];
// uint8_t          int_counted[BNOs];

// // Interrupt service routines for the three BNO085 sensors
// void isr0() { int_count[0]++; }
// void isr1() { int_count[1]++; }
// void isr2() { int_count[2]++; }

// void setup()
// {
//   Serial.begin(SERIAL_BAUD);                // initialize serial
//   // Wait for serial port to connect or timeout after 3 seconds
//   uint32_t start = millis();
//   while (!Serial && (millis() - start < 3000));
  
//   Serial.println("\nBNO085 Triple Sensor on Pico Running...");
//   Serial.println("Hardware Initialization...");

//   // Initialize pin modes for SPI pins
//   pinMode(pinSCL, OUTPUT);
//   pinMode(pinSDA, INPUT);
//   pinMode(pinDI, OUTPUT);

//   // Initialize CS pins and deselect all BNOs
//   for (uint8_t bno=0; bno<BNOs; bno++) {
//     pinMode(CS[bno], OUTPUT);
//     digitalWrite(CS[bno], HIGH);
//     Serial.print("CS"); Serial.print(bno); Serial.print(" pin = GP"); Serial.println(CS[bno]);
//   }

//   // Initialize SPI
//   SPI.begin();
  
//   // Reset all BNO085 sensors using single reset pin
//   Serial.println("Resetting all BNO085 sensors...");
//   pinMode(pinRST, OUTPUT);
//   digitalWrite(pinRST, LOW);
//   delay(10);  // longer reset pulse
//   digitalWrite(pinRST, HIGH);
//   delay(500);  // Longer reset recovery time
//   Serial.println("Reset complete, waiting for stability...");
//   delay(100);  // Additional stability delay

//   // Set up interrupt pins with pull-ups
//   Serial.println("Setting up interrupts...");
//   pinMode(INT[0], INPUT_PULLUP);
//   pinMode(INT[1], INPUT_PULLUP);
//   pinMode(INT[2], INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(INT[0]), isr0, FALLING);
//   attachInterrupt(digitalPinToInterrupt(INT[1]), isr1, FALLING);
//   attachInterrupt(digitalPinToInterrupt(INT[2]), isr2, FALLING);
//   Serial.print("INT0 pin = GP"); Serial.println(INT[0]);
//   Serial.print("INT1 pin = GP"); Serial.println(INT[1]);
//   Serial.print("INT2 pin = GP"); Serial.println(INT[2]);
  
//   // Initialize each sensor one at a time
//   Serial.println("Initializing sensors...");
//   for (uint8_t bno=0; bno<BNOs; bno++) {
//     // Serial.print("Initializing BNO ");
//     // Serial.println(bno);
    
//     // Reset the transmit queue for this sensor
//     txqueue_msg[bno] = 0;
//     txqueue_pos[bno] = 0;
    
//     SPI.beginTransaction(SPI_SETTINGS);
    
//     // Send configuration messages
//     for (uint8_t i = 0; i < numberof(txqueue); i++) {
//       bno_select(bno);
//       for (uint8_t j = 0; j < txqueue[i].length; j++) {
//         SPI.transfer(txqueue[i].message[j]);
//       }
//       bno_deselect(bno);
//       delay(50);  // Give sensor time to process each message
//     }
    
//     SPI.endTransaction();
    
//     // Wait a bit between sensors
//     delay(200);
//   }

//   // Initialize data_ready flags
//   for (uint8_t bno=0; bno<BNOs; bno++) {
//     data_ready[bno] = false;
//   }

//   // Serial.println("Initialization complete, beginning sensor monitoring...");
//   // Serial.println("Data will be output in synchronized frames with all sensors.");
//   last_sync_output = millis();
// }

// void loop()
// {
//   // Check for interrupt-driven report requests
//   for (uint8_t bno=0; bno<BNOs; bno++)
//   {
//     if (int_count[bno] != int_counted[bno]) // if transfer request
//     {
//       int_counted[bno] = int_count[bno];    // acknowledge all interrupt(s)
//       SPI.beginTransaction(SPI_SETTINGS);
//       check_report(bno);
//       SPI.endTransaction();
//     }
//   }
  
//   // Check if it's time for a synchronized output
//   #if SYNC_OUTPUT
//   if ((uint32_t)(millis() - last_sync_output) >= SYNC_PERIOD)
//   {
//     last_sync_output = millis();
    
//     // Check if we have data from all sensors
//     bool all_ready = true;
//     for (uint8_t bno=0; bno<BNOs; bno++) {
//       if (!data_ready[bno]) {
//         all_ready = false;
//         break;
//       }
//     }
    
//     // If all sensors have data or it's been too long since last output, output what we have
//     if (all_ready || (uint32_t)(millis() - last_sync_output) >= SYNC_PERIOD*3) {
//       if (DATAOUT) {
//         output_synchronized_data();
//       }
      
//       // Reset data_ready flags
//       for (uint8_t bno=0; bno<BNOs; bno++) {
//         data_ready[bno] = false;
//       }
//     }
//   }
//   #endif
// }