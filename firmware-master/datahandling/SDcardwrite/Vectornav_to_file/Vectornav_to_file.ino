#define HWSERIAL Serial1
#include "SdFat.h"
//#include <SPI.h>
//#include <SD.h>
SdExFat sd;
unsigned long Time = 0;
byte incoming_byte[4500]; 

void setup() {
HWSERIAL.begin(230400); // opens serial port, sets data rate to 115200 bps

 // Clear serial ports
 Serial.flush(); 
 HWSERIAL.flush();

 // Check if SD card initialized correctly
 if (!sd.begin(SdioConfig(FIFO_SDIO))) { // SdFat.h Ver 2.0.0 Beta
    Serial.println("SD initialization failed!");
  } else {
    Serial.println("SD initialization OK");
  }
  
 // Create file in SD card     
 
 }

void loop() {
memset(incoming_byte, 0, sizeof(incoming_byte));
HWSERIAL.flush();
    // Save data only when we receive data:
  if (HWSERIAL.available() > 0) {
    // read the incoming byte:
    Time = millis();
    HWSERIAL.readBytes(incoming_byte,4500);
    
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
   int i;int j;
   char parsed_data_set[3600];
      for (j = 0; j< 150;j++){
          for(i = 4; i < 28; i++)  
            {parsed_data_set[j*24+(i-4)]=incoming_byte[i+(j*30)];}
      }
    // Write data in binary
    auto data_file = sd.open("Vectornav_800Hz-4-14-Validation-Fullset2.bin", FILE_WRITE);
    
    //byte time_arr[4];
    //time_arr[0] = Time & 0xFF; 
    //time_arr[1] = (Time >> 8) & 0xFF ;
    //time_arr[2] = (Time >> 16) & 0xFF ;
    //time_arr[3] = (Time >> 24) & 0xFF ;
    data_file.write(parsed_data_set,3600);
    data_file.close();
    // This block is for debugging, prints time of each loop in Serial Monitor
    Serial.print(incoming_byte[1]); 
    Serial.print(Time);
    
    }
    
    
}
//    line++;
//    if (line%6==0){
//      data_file.println();
//    }
//  }