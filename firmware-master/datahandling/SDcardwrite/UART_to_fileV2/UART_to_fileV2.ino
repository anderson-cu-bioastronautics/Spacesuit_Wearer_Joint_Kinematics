// Defines
#define HWSERIAL1 Serial1
#define HWSERIAL2 Serial2
#define HWSERIAL3 Serial3
#define HWSERIAL4 Serial4
#define HWSERIAL5 Serial5
#define HWSERIAL6 Serial6
#define HWSERIAL7 Serial7
#define HWSERIAL8 Serial8
#define BLOCKLENGTH 16200
#define BUFFERSIZE 16384
#define UARTBAUD 115200

// Includes
#include "SdFat.h"
//#include <SPI.h>
//#include <SD.h>

// Active IMUs
bool actIMU1 = true;
bool actIMU2 = false;
bool actIMU3 = false;
bool actIMU4 = false;
bool actIMU5 = false;
bool actIMU6 = false;
bool actIMU7 = false;
bool actIMU8 = false;

// Buffers and storage
SdExFat sd;
//unsigned long Time1 = 0;
//byte Time1_array[4];
//byte Time2_array[4];
//unsigned long Time2 = 0;
char incoming_byte1[BLOCKLENGTH]; 
char incoming_byte2[BLOCKLENGTH]; 
char incoming_byte3[BLOCKLENGTH];
char incoming_byte4[BLOCKLENGTH];
char incoming_byte5[BLOCKLENGTH];
char incoming_byte6[BLOCKLENGTH];
char incoming_byte7[BLOCKLENGTH];
char incoming_byte8[BLOCKLENGTH];
char hwbuffer1[BUFFERSIZE];
char hwbuffer2[BUFFERSIZE];

// Pin assignments
int start_stop_pin = 41; //Switch pin
int TX_IMU1 = 1;
int TX_IMU2 = 8;
int TX_IMU3 = 14;
int TX_IMU4 = 17;
int TX_IMU5 = 20;
int TX_IMU6 = 24;
int TX_IMU7 = 29;
int TX_IMU8 = 35;

// Flags
bool start_stop;
bool flag = false;  
bool fWriteData = false;

// Setup
void setup() {
// Increase UART buffers  
  HWSERIAL1.addMemoryForRead(hwbuffer1, BUFFERSIZE);
  HWSERIAL2.addMemoryForRead(hwbuffer2, BUFFERSIZE);
  
//Set Start-Stop pin
   pinMode(start_stop_pin, INPUT_PULLDOWN);  
// Check if SD card initialized correctly
    if (!sd.begin(SdioConfig(FIFO_SDIO))) { // SdFat.h Ver 2.0.0 Beta
      Serial.println("SD initialization failed!");
    } 
    else {Serial.println("SD initialization OK");
    }
  sd.mkdir("Test-12-7");
  sd.chdir("/Test-12-7");
}

void loop() {
  memset(incoming_byte1, 0, sizeof(incoming_byte1));
  memset(incoming_byte2, 0, sizeof(incoming_byte2));
  start_stop = digitalRead(start_stop_pin);
  
  if (start_stop == HIGH){
    if (flag == false){
    HWSERIAL1.begin(UARTBAUD); // opens serial port, acts as start signal for IMU board
    HWSERIAL2.begin(UARTBAUD);
    HWSERIAL3.begin(UARTBAUD);
    HWSERIAL4.begin(UARTBAUD);
    HWSERIAL5.begin(UARTBAUD);
    HWSERIAL6.begin(UARTBAUD);
    HWSERIAL7.begin(UARTBAUD);
    HWSERIAL8.begin(UARTBAUD);

    HWSERIAL1.flush(); // clear serial ports
    HWSERIAL2.flush();
    HWSERIAL3.flush();
    HWSERIAL4.flush();
    HWSERIAL5.flush();
    HWSERIAL6.flush();
    HWSERIAL7.flush();
    HWSERIAL8.flush();

    flag = true;
    
    }
      // Save data only when we receive data:
    if (HWSERIAL1.available() > 0) {
      // read the incoming byte:
      HWSERIAL1.readBytes(incoming_byte1,BLOCKLENGTH);
      Serial.print(incoming_byte1[1]); 
      fWriteData = true;
      
      }
    if (HWSERIAL2.available() > 0) {
      HWSERIAL2.readBytes(incoming_byte2,BLOCKLENGTH);
      fWriteData = true;
      
      }
    if (fWriteData) {
      if (actIMU1) {
        auto data_file = sd.open("IMU1-Data.bin", FILE_WRITE);
        data_file.write(incoming_byte1,BLOCKLENGTH);
        data_file.close();
      }
      if (actIMU2) {
        auto data_file = sd.open("IMU2-Data.bin", FILE_WRITE);
        data_file.write(incoming_byte2,BLOCKLENGTH);
        data_file.close();
      }
      if (actIMU3) {
        auto data_file = sd.open("IMU3-Data.bin", FILE_WRITE);
        data_file.write(incoming_byte3,BLOCKLENGTH);
        data_file.close();
      }
      if (actIMU4) {
        auto data_file = sd.open("IMU4-Data.bin", FILE_WRITE);
        data_file.write(incoming_byte4,BLOCKLENGTH);
        data_file.close();
      }
      if (actIMU5) {
        auto data_file = sd.open("IMU5-Data.bin", FILE_WRITE);
        data_file.write(incoming_byte5,BLOCKLENGTH);
        data_file.close();
      }
      if (actIMU6) {
        auto data_file = sd.open("IMU6-Data.bin", FILE_WRITE);
        data_file.write(incoming_byte6,BLOCKLENGTH);
        data_file.close();
      }
      if (actIMU7) {
        auto data_file = sd.open("IMU7-Data.bin", FILE_WRITE);
        data_file.write(incoming_byte7,BLOCKLENGTH);
        data_file.close();
      }
      if (actIMU8) {
        auto data_file = sd.open("IMU8-Data.bin", FILE_WRITE);
        data_file.write(incoming_byte8,BLOCKLENGTH);
        data_file.close();
      }
      fWriteData = false;
    }
  }
  
  if (start_stop == LOW){
    if (flag == true){
      HWSERIAL1.end();
      HWSERIAL2.end();
      HWSERIAL3.end();
      HWSERIAL4.end();
      HWSERIAL5.end();
      HWSERIAL6.end();
      HWSERIAL7.end();
      HWSERIAL8.end();
      pinMode(TX_IMU1, OUTPUT);
      pinMode(TX_IMU2, OUTPUT);
      pinMode(TX_IMU3, OUTPUT);
      pinMode(TX_IMU4, OUTPUT);
      pinMode(TX_IMU5, OUTPUT);
      pinMode(TX_IMU6, OUTPUT);
      pinMode(TX_IMU7, OUTPUT);
      pinMode(TX_IMU8, OUTPUT);
      digitalWrite(TX_IMU1, LOW);
      digitalWrite(TX_IMU2, LOW);
      digitalWrite(TX_IMU3, LOW);
      digitalWrite(TX_IMU4, LOW);
      digitalWrite(TX_IMU5, LOW);
      digitalWrite(TX_IMU6, LOW);
      digitalWrite(TX_IMU7, LOW);
      digitalWrite(TX_IMU8, LOW);
      flag = false;
    }
  }
      // This block is for debugging, prints time of each loop in Serial Monitor
      //Serial.print(incoming_byte1[1]); 
      //Serial.print(Time1);
}

      //THIS BLOCK CONVERTS THE RECEIVED DATA TO A STRING IN SCIENTIFIC NOTATION
      /*
    float final_value;
    union u_tag {
      byte b[4];
      float final_val_memory;
      } u;

    u.b[0] = incoming_byte[0];
    u.b[1] = incoming_byte[1];
    u.b[2] = incoming_byte[2];
    u.b[3] = incoming_byte[3];

    final_value = u.final_val_memory;

    // Record received data:
    char sci_not[100];
    sprintf(sci_not, "%e, ", final_value);
    
    */
  
