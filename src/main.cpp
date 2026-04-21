#include <Arduino.h>

#include <FlexCAN_T4.h>
#include "CRC8.h"
CRC8 crc(0x85, 0x00, 0x00, false, false);
FlexCAN_T4<CAN1, RX_SIZE_512, TX_SIZE_64> can1;
FlexCAN_T4<CAN2, RX_SIZE_512, TX_SIZE_64> can2;
FlexCAN_T4<CAN3, RX_SIZE_512, TX_SIZE_64> can3;
CAN_message_t msg;

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 100;  //the value is a number of milliseconds


  int battVolt1 = 0; 
  int battVolt2 = 0;
  int battVolt = 0;
  int battCur1 = 0;
  int battCur2 = 0;
  int battCur = 0;
  int soc1 = 0;
  int soc2 = 0;
  int soc = 0;
  int dislimit1 = 0;
  int dislimit2 = 0;
  int dislimit = 0;
  int chglimit1 = 0;
  int chglimit2 = 0;
  int chglimit = 0;
  int chargelimit1 = 0;
  int chargelimit2 = 0;
  int chargelimit = 0;
  int GIDS1 = 0;
  int GIDS2 = 0;
  int GIDS = 0;
  int tempSegment1 = 0;
  int tempSegment2 = 0;
  int tempSegment = 0;
  int QCfullCapacity1 = 0;
  int QCfullCapacity2 = 0;
  int QCfullCapacity = 0;
  int QCremainCapacity1 = 0;
  int QCremainCapacity2 = 0;
  int QCremainCapacity = 0;

  int ChgPwrStatMask = 0b00000011;  // Mask to extract the 2 bits of the charge power status from the CAN message
  int ChgPwrStat = 0;  // Charge power status for battery 00b = Reserved 01b = Normal limit PIN 10b = High rate limit PIN 11b = Immediate limit PIN
  int BPCmaxUprateMask = 0b11100000;  // Mask to extract the 3 bits of the max uprate for battery charge power status from the CAN message
  int BPCmaxUprate = 0;  // Max uprate for battery charge power status 

  //0x5C0 values
  int HDhighLowVtime1[4] = {0}; 
  int HDtempWakeup1[4] = {0};
  int HDtemperature1[4] = {0};
  int HDintCurr1[4] = {0};
  int HDirCoef1[4] = {0};
  int HDcellV1[4] = {0};

  int HDhighLowVtime2[4] = {0};
  int HDtempWakeup2[4] = {0}; 
  int HDtemperature2[4] = {0};
  int HDintCurr2[4] = {0};
  int HDirCoef2[4] = {0};
  int HDcellV2[4] = {0};


  // int temperature1 = 0;
  // int temperature2 = 0;
  // int temperature = 0;
  // char battType1[] = "ZE0  ";
  // char battType2[] = "AZE0 ";
  // char battType[] = "ZE1";
  // char battCur1step1[6];
  // char battCur1step2[6];
  // char battCur2step1[6];
  // char battCur2step2[6];
  //char battVoltage1step1[6];
  //char battVoltage2step1[6];
  const float safetyMargin = 0.75;
  const int maxDCFC = 400;    // kW * 0.10 (400 = 40kW)

void setup(void) {
  Serial.begin(115200);
  can1.begin();              // Battery 1
  can1.setClock(CLK_60MHz);
  can1.setBaudRate(500000);  // 500kbps data rate
  can2.begin();              // Battery 2
  can2.setClock(CLK_60MHz);
  can2.setBaudRate(500000);  // 500kbps data rate
  can2.setMaxMB(64);         // Max mailboxs, 1 MailBox per can message
  can2.enableFIFO();         // Preserve order of messages.
  can3.begin();              // ZombieVerter
  can3.setClock(CLK_60MHz);
  can3.setBaudRate(500000);  // 500kbps data rate
  can3.setMaxMB(64);         // Max mailboxs, 1 MailBox per can message
  can3.enableFIFO();         // Preserve order of messages.

 startMillis = millis();  //initial start time
}

void loop() {
  if ( can1.read(msg) ) {   // Can messages from battery 1 are read, processed, and sent to the zombieverter (can3)
    const uint8_t *bytes = msg.buf;
  switch (msg.id) {
    case 0x1DB: {
      battCur1 = uint16_t(bytes[0] << 3) + uint16_t(bytes[1] >> 5);
      battVolt1 = (uint16_t(bytes[2] << 2) + uint16_t(bytes[3] >> 6));   // / 2;
      break;
    }
    case 0x1DC: {
      dislimit1 = (uint16_t(bytes[0] << 2) + uint16_t(bytes[1] >> 6));              // * 0.25;  // Kw discharge limit
      chglimit1 = (uint16_t((bytes[1] & 0x3F) << 4) + uint16_t(bytes[2] >> 4));     // * 0.25;  // Kw regen limit
      chargelimit1 = (uint16_t((bytes[2] & 0x0F) << 6) + uint16_t(bytes[3] >> 2));  // * 0.1;  // Kw charger limit
      ChgPwrStat = (bytes[3] & ChgPwrStatMask);  // Extract charge power status from the CAN message
      BPCmaxUprate = (bytes[4] & BPCmaxUprateMask);  // Extract max uprate for battery charge power status from the CAN message
      break;
    }
    case 0x55B: {
      soc1 = (uint16_t(bytes[0] << 2) + uint16_t(bytes[1] >> 6));                   // * 0.1;
      break;
    }
    case 0x59E: {
      QCfullCapacity1 = (uint16_t(bytes[2] << 4) + uint16_t(bytes[3] >> 4));                   // * 100 Wh
      QCremainCapacity1 = (uint16_t(bytes[3] << 5) + uint16_t(bytes[4] >> 3));
      QCremainCapacity1 &= 0b0000000111111111;  // Mask to ensure only the bits used for QC remaining capacity are included
      break;
    }
    case 0x5BC: {
      GIDS1 = (uint16_t(bytes[0] << 2) + uint16_t(bytes[1] >> 6));                   // GIDS
      tempSegment1 = (uint16_t(bytes[3]));                   // Temp segment for dashboard
      break;
    }
    case 0x5C0: {
      int mux = bytes[0] >> 6;
      HDhighLowVtime1[mux] = (uint16_t(bytes[0] & 0b00001111));
      HDtempWakeup1[mux] = (uint16_t(bytes[1] & 0b11111110));
      HDtemperature1[mux] = uint16_t(bytes[2]);
      HDintCurr1[mux] = uint16_t(bytes[3]);
      HDirCoef1[mux] = (uint16_t(bytes[4] & 0b11111110));
      HDcellV1[mux] = (uint16_t(bytes[5]));
      }
      break;
    }
  }
  
 
  else if ( can2.read(msg) ) {    // Battery 2 messages are read, processed, and sent to the zombieverter (can3)
  const uint8_t *bytes = msg.buf;
  switch (msg.id) {
    case 0x1DB: {
      battCur2 = uint16_t(bytes[0] << 3) + uint16_t(bytes[1] >> 5);
      battCur = battCur1 + battCur2;
      battCur = battCur << 5;                                                    // Shift bits 5 places to the left to stuff into CAN frame
      msg.buf[0] = highByte(battCur);
      msg.buf[1] = bytes[1] & B00011111;                                         // Clear the bits used for battCur 
      msg.buf[1] = msg.buf[1] + lowByte(battCur);                                //and add the low byte of battCur shifted 5 places to the left
      battVolt2 = (uint16_t(bytes[2] << 2) + uint16_t(bytes[3] >> 6));                     // / 2;
      //battVolt = battVolt2;
      for ( uint8_t i = 0; i < 7; i++ ) {
        crc.add(msg.buf[i]);
      }
      msg.buf[7] = crc.calc();
      crc.restart();
    break;
    } 

    case 0x1DC: {
      dislimit2 = (uint16_t(bytes[0] << 2) + uint16_t(bytes[1] >> 6));                    // * 0.25;  // Kw discharge limit
        if (dislimit1 == 0 || dislimit2 == 0) {
          dislimit = 0;
         }
        else {
          dislimit = dislimit1 + dislimit2;
          dislimit = dislimit * safetyMargin;
          dislimit = dislimit << 6;
        }
      chglimit2 = (uint16_t((bytes[1] & 0x3F) << 4) + uint16_t(bytes[2] >> 4));           // * 0.25;  // Kw charge limit
        if (chglimit1 == 0 || chglimit2 == 0) {
          chglimit = 0;
         }
        else {
          chglimit = chglimit1 + chglimit2;
          chglimit = chglimit * safetyMargin;
          chglimit = chglimit << 4;
        }
      chargelimit2 = (uint16_t((bytes[2] & 0x0F) << 6) + uint16_t(bytes[3] >> 2));        // * 0.1;  // Kw charger limit
        if (chargelimit1 == 0 || chargelimit2 == 0) {
          chargelimit = 0;
         }
        else {
          chargelimit = chargelimit1 + chargelimit2;
          chargelimit = chargelimit * safetyMargin;
        }
          if (chargelimit > maxDCFC) {
            chargelimit = maxDCFC;
            }
          chargelimit = chargelimit << 2;
    
      msg.buf[0] = highByte(dislimit);
      msg.buf[1] = lowByte(dislimit) + highByte(chglimit);
      msg.buf[2] = lowByte(chglimit) + highByte(chargelimit);
      if (ChgPwrStat > (msg.buf[3] & ChgPwrStatMask))  {  // If the charge power status from battery 1 is more limiting than that from battery 2, use it in the CAN message to the zombieverter
        msg.buf[3] = ChgPwrStat;  // Add the charge power status from battery 1 to the CAN message
      }
      else {
        msg.buf[3] = bytes[3] & B00000011;
      }
      msg.buf[3] = msg.buf[3] + lowByte(chargelimit);
      if (BPCmaxUprate < (msg.buf[4] & BPCmaxUprateMask))  {  // If the max uprate for battery charge power status from battery 1 is more limiting than that from battery 2, use it in the CAN message to the zombieverter
        msg.buf[4] = (msg.buf[4] & BPCmaxUprateMask); // Clear battery 2 value from the CAN message
        msg.buf[4] += BPCmaxUprate;  // Add the max uprate for battery charge power status from battery 1 to the CAN message
      }
      else {
        msg.buf[4] = bytes[4] & B11100000;
      }
      for ( uint8_t i = 0; i < 7; i++ ) {
        crc.add(msg.buf[i]);
      }
      msg.buf[7] = crc.calc();
      crc.restart();
      break;
    }

    case 0x55B: {
      soc2 = (uint16_t(bytes[0] << 2) + uint16_t(bytes[1] >> 6));                           // * 0.1;
        if (soc1 > 990 || soc2 > 990) {
          soc = 1000;
         }
        else {
          soc = soc1 + soc2;
          soc = soc /2;
        }
      
      soc = soc << 6;
      msg.buf[0] = highByte(soc);
      msg.buf[1] = lowByte(soc);

      for ( uint8_t i = 0; i < 7; i++ ) {
        crc.add(msg.buf[i]);
      }
      msg.buf[7] = crc.calc();
      crc.restart();

      break;
    }
    case 0x59E: {
      QCfullCapacity2 = (uint16_t(bytes[2] << 4) + uint16_t(bytes[3] >> 4));                   // * 100 Wh
      QCremainCapacity2 = (uint16_t(bytes[3] << 5) + uint16_t(bytes[4] >> 3));                   // * 100 Wh
      QCfullCapacity = QCfullCapacity1 + QCfullCapacity2;
      QCremainCapacity = QCremainCapacity1 + QCremainCapacity2;
      QCremainCapacity1 &= 0b0000000111111111; // Mask to ensure only the bits used for QC remaining capacity are included
      
      msg.buf[2] = QCfullCapacity >> 4;
      msg.buf[3] = lowByte(QCfullCapacity << 4) + (QCremainCapacity >> 5);
      msg.buf[4] = lowByte(QCremainCapacity << 3);

      Serial.print("QC full capacity: ");
      Serial.print(QCfullCapacity); 
      Serial.print("00 Wh, QC remaining capacity: ");
      Serial.print(QCremainCapacity);
      Serial.println("00 Wh");
      Serial.print("QC full capacity1: ");
      Serial.print(QCfullCapacity1);  
      Serial.print("00 Wh, QC remaining capacity1: ");
      Serial.print(QCremainCapacity1);
      Serial.println("00 Wh");
      Serial.print("QC full capacity2: ");
      Serial.print(QCfullCapacity2);  
      Serial.print("00 Wh, QC remaining capacity2: ");
      Serial.print(QCremainCapacity2);
      Serial.println("00 Wh");
      break;
    }
     case 0x5BC: {
      GIDS2 = (uint16_t(bytes[0] << 2) + uint16_t(bytes[1] >> 6));                   // GIDS
      GIDS = GIDS1 + GIDS2;
      GIDS = GIDS << 6;
      msg.buf[0] = highByte(GIDS);
      msg.buf[1] = lowByte(GIDS);
      tempSegment2 = (uint16_t(bytes[3]));                   // Temp segment for dashboard
      if (tempSegment1 > tempSegment2) {
        msg.buf[3] = tempSegment1;
        }
      break;
      }

      // Messages with this ID contain data for multiple muxes. The mux is determined by the first 2 bits of the first byte of the CAN message. 
      // Data for each mux is stored in separate arrays for battery 1 and battery 2, and the appropriate values are added to the CAN message sent
      // to the zombieverter based on comparisons between the values from battery 1 and battery 2 for each mux.
      // Fault codes still need to be added to this section.
      case 0x5C0: {  
        int mux = bytes[0] >> 6;
        HDhighLowVtime2[mux] = (uint16_t(bytes[0] & 0b00001111));
        HDtempWakeup2[mux] = (uint16_t(bytes[1] & 0b11111110));
        HDtemperature2[mux] = uint16_t(bytes[2]);       
        HDintCurr2[mux] = uint16_t(bytes[3]);
        HDirCoef2[mux] = (uint16_t(bytes[4] & 0b11111110));
        HDcellV2[mux] = (uint16_t(bytes[5]));
        switch (mux){
          case 1:{
            if (HDhighLowVtime1[mux] > HDhighLowVtime2[mux]) {
              msg.buf[0] &= 0b11110000; // Clear the bits used for HDhighLowVtime
              msg.buf[0] += HDhighLowVtime1[mux]; // Add the HDhighLowVtime from battery 1 to the CAN message
            }
            if (HDtempWakeup1[mux] > HDtempWakeup2[mux]) {
              msg.buf[1] &= 0b00000001; // Clear the bits used for HDtempWakeup
              msg.buf[1] += (HDtempWakeup1[mux]); // Add the HDtempWakeup from battery 1 to the CAN message
            }
            if (HDtemperature1[mux] > HDtemperature2[mux]) {
              msg.buf[2] = HDtemperature1[mux];
            }
            if (HDintCurr1[mux] > HDintCurr2[mux]) {
              msg.buf[3] = HDintCurr1[mux];
            }
            if (HDirCoef1[mux] > HDirCoef2[mux]) {
              msg.buf[4] &= 0b00000001; // Clear the bits used for HDirCoef
              msg.buf[4] += HDirCoef1[mux]; // Add the HDirCoef from battery 1 to the CAN message
            }
            if (HDcellV1[mux] > HDcellV2[mux]) {
              msg.buf[5] = (HDcellV1[mux] << 2);
            }
          break;
          }
          case 2:{
            msg.buf[0] &= 0b11110000; // Clear the bits used for HDhighLowVtime
            msg.buf[0] += ((HDhighLowVtime1[mux] + HDhighLowVtime2[mux]) /2); // Average of HDhighLowVtime from battery 1 and battery 2 added to the CAN message
            
            msg.buf[1] &= 0b00000001; // Clear the bits used for HDtempWakeup
            msg.buf[1] += ((HDtempWakeup1[mux] + HDtempWakeup2[mux]) / 2); // Average of HDtempWakeup from battery 1 and battery 2 added to the CAN message

            msg.buf[2] = ((HDtemperature1[mux] + HDtemperature2[mux]) / 2); // Average of HDtemperature from battery 1 and battery 2 added to the CAN message
            
            msg.buf[3] = ((HDintCurr1[mux] + HDintCurr2[mux]) / 2); // Average of HDintCurr from battery 1 and battery 2 added to the CAN message
            
            msg.buf[4] &= 0b00000001; // Clear the bits used for HDirCoef
            msg.buf[4] += ((HDirCoef1[mux] + HDirCoef2[mux]) / 2 ); // Average of HDirCoef from battery 1 and battery 2 added to the CAN message
            
            msg.buf[5] = (((HDcellV1[mux] + HDcellV2[mux]) / 2) << 2); // Average of HDcellV from battery 1 and battery 2 added to the CAN message
            break;
          }
          case 3:{
            if (HDhighLowVtime1[mux] < HDhighLowVtime2[mux]) {
              msg.buf[0] &= 0b11110000; // Clear the bits used for HDhighLowVtime
              msg.buf[0] += HDhighLowVtime1[mux]; // Add the HDhighLowVtime from battery 1 to the CAN message
            }
            if (HDtempWakeup1[mux] < HDtempWakeup2[mux]) {
              msg.buf[1] &= 0b00000001; // Clear the bits used for HDtempWakeup
              msg.buf[1] += (HDtempWakeup1[mux]); // Add the HDtempWakeup from battery 1 to the CAN message
            }
            if (HDtemperature1[mux] < HDtemperature2[mux]) {
              msg.buf[2] = HDtemperature1[mux];
            }
            if (HDintCurr1[mux] < HDintCurr2[mux]) {
              msg.buf[3] = HDintCurr1[mux];
            }
            if (HDirCoef1[mux] < HDirCoef2[mux]) {
              msg.buf[4] &= 0b00000001; // Clear the bits used for HDirCoef
              msg.buf[4] += HDirCoef1[mux]; // Add the HDirCoef from battery 1 to the CAN message
            }
            if (HDcellV1[mux] < HDcellV2[mux]) {
              msg.buf[5] = (HDcellV1[mux] << 2);
            }
          break;
          }
        }
      }
    }
can3.write(msg);
  } 

  else if ( can3.read(msg) ) {
    can1.write(msg);
    can2.write(msg);
   }
  else { 
     currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= period) {  //test whether the period has elapsed
    startMillis = currentMillis; 
    
    msg.id = 0x7A1;
    msg.buf[0] = highByte(battVolt1);
    msg.buf[1] = lowByte(battVolt1);
    msg.buf[2] = highByte(battCur1);
    msg.buf[3] = lowByte(battCur1);
    msg.buf[4] = highByte(soc1);
    msg.buf[5] = lowByte(soc1);
    can3.write(msg);  

    msg.id = 0x7B1;
    msg.buf[0] = highByte(dislimit1);
    msg.buf[1] = lowByte(dislimit1);
    msg.buf[2] = highByte(chglimit1);
    msg.buf[3] = lowByte(chglimit1);
    msg.buf[4] = highByte(chargelimit1);
    msg.buf[5] = lowByte(chargelimit1);
    can3.write(msg);    

     msg.id = 0x7A2;
    msg.buf[0] = highByte(battVolt2);
    msg.buf[1] = lowByte(battVolt2);
    msg.buf[2] = highByte(battCur2);
    msg.buf[3] = lowByte(battCur2);
    msg.buf[4] = highByte(soc2);
    msg.buf[5] = lowByte(soc2);
    can3.write(msg);  

    msg.id = 0x7B2;
    msg.buf[0] = highByte(dislimit2);
    msg.buf[1] = lowByte(dislimit2);
    msg.buf[2] = highByte(chglimit2);
    msg.buf[3] = lowByte(chglimit2);
    msg.buf[4] = highByte(chargelimit2);
    msg.buf[5] = lowByte(chargelimit2);
    can3.write(msg);  
  }  
  } 
}