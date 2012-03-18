/*

 AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>.
  
 */
/*

 Author: Bill Faulkner
 
 Implements a flexible Flight Data Recorder using the OpeLog
 Serial Logger from Sparkfun. 
 */

#ifndef _AQ_FDR_H_
#define _AQ_FDR_H_

//  This class is used with the OpenLog serial logger from Sparkfun.
//  Data is sent in raw binary form (not ASCII) for speed. A header
//  is written first that describes the structure of each type of
//  log record.
//
//  Basic Structure each record
//
//  Start  Start byte
//  Type   Record Type byte
//  Data   Variable length data
//  Cksum  Checksum byte
//

// OpenLog Interactions
#define  FDR_OL_PROMPT         '>'					
#define  FDR_OL_ESCSEQ         "\x1A\x1A\x1A\r"    // 3 ctrl-Z and a CR
#define  FDR_OL_RESTART        "\rrestart\r"
#define	 FDR_OL_DTR_PIN		

#define  FDR_CMDESC            '~'
#define  FDR_CMDRETRIES        5
#define  FDR_RECORD_SS         0xA3

// FDR Record Types
#define  FDR_REC_HEADER        0
#define  FDR_REC_FLIGHT        1  
#define  FDR_REC_GPS           2
#define  FDR_REC_ITOD          3
#define  FDR_REC_ALTHOLD       4
#define  FDR_REC_VRSWITCH      5
#define  FDR_REC_ALTPID        6
#define  FDR_REC_BARODATA      7
#define  FDR_REC_BAROGND       8



////////////////////////////////////////////////////////////////////////////////
//
//  The logged output is in a binary format.  A key table indiced by FDR record type
//  is used by the decorder to understand how to unpack the record.  The descriptor
//  element of the key contains a string of letters describing the order and type
//  of data elements packed into the record.  The decorder uses these letters to
//  know how big each data element is and how to decode it.  The defined types are:
//
//  T  time, normally a long, the high order byte is chopped & the decoder handles wraparound.
//  l
//  b
//  l
//  f
//  
////////////////////////////////////////////////////////////////////////////////
#define  FDR_REC_NUMTYPES       sizeof(_descriptors)/sizeof(_descriptors[0])

struct FDR_Types {
  uint8_t      type;
                                //TODO:  add descriptive type name string
  uint8_t      descriptor[21];  // null terminated String of decode values, max 20
  uint8_t      labels[100];
} 
_descriptors[] = {
  {FDR_REC_HEADER,  "X",        "X"},                     
  {FDR_REC_FLIGHT,  "Thhaib", "raw Alt,sm Alt,gndZacc,Thr Adj,Mtr Cmd,AH Flg" }, 
  /*
  {FDR_REC_GPS,     "Tgghb",    "lat,long,GPS alt,flag"},
  {FDR_REC_ITOD,    "Tffffa",      "raw Alt, Roll, Pitch, Yaw"},
  {FDR_REC_ALTHOLD, "Thib",       "H Alt,H Thr,AltH f"},
  {FDR_REC_VRSWITCH,"Tv",        "bit flags"},
  {FDR_REC_ALTPID,  "Thhii",    "H Alt,C Alt,Th Adj,Mtr Cmd"},
  {FDR_REC_BARODATA,"Tblf",     "flags,iir_b5,iir_Alt"},
  {FDR_REC_BAROGND, "Tfff",      "GndTemp,GndPress,GndAlt"}
  */
};

#define  OPENFDR_PIN  32

class serialLogger /*public Print*/ {
  
private:
  uint8_t                    _port;
  uint8_t                    _baud;
  uint8_t                    _buffer[250];
  uint8_t                    _inCommandMode;
  HardwareSerial             *_serialPort; 

public:
/*
  using Print::write;  // pull in write(str) and write(buf, size) from Print
*/


  void initialize(uint8_t port, long baud) {

        _buffer[0] = FDR_RECORD_SS;  // Load first buffer char w/ start byte
        _inCommandMode = OFF;

    switch (port) {
      
      case 0: _serialPort = &Serial; break;
    
#if defined(AeroQuadMega_v2) || defined(AeroQuadMega_Wii) || defined(AeroQuadMega_CHR6DM)
      case 1: _serialPort = &Serial1; break;
      case 2: _serialPort = &Serial2; break;
      case 3: _serialPort = &Serial3; break;
#endif

      default: _serialPort = &Serial; break;
    
    } 


    // OpenLog will open and read the file CONFIG.TXT which should contain:
    // "115200,26,3,0", Baud=115200, Esc char is 26=0x1A, Num esc chars=3, 0 is new file mode
    // 
    // Since the quad could have been reset without the OpenLog being restarted
    // make sure we are starting with a fresh file.  
    //
    // FUTURE: Hook up OpenLog's DTR pin to an Output PIN so this code can reset OpenLog

    _serialPort -> begin(baud);
  
//#ifdef  FDR_USE_DTR  
    pinMode (FDR_OL_DTR_PIN, OUTPUT);
    digitalWrite (FDR_OL_DTR_PIN, HIGH);
    delay(22);
//#endif
    

    exitCommandMode();
    
    dumpRecord(FDR_REC_HEADER);


  } //end begin()


  //
  //  writes a single character to the FDR
  //
  /*
  void write (uint8_t c) {
    _serialPort -> write(c);
  }
  */



  //
  //
  //
  //
  void dumpRecord(uint8_t type) {
    uint32_t   cksum=0;
    uint8_t    cksum_b;
    uint16_t         len=0;
    uint8_t    *bptr = &_buffer[1];
    uint8_t    *iptr = &_buffer[0];
    uint8_t    *vptr;
    uint8_t		*dptr;
    uint16_t   i_val;
    /*
    uint32_t   ival;
    float      f_val;
    float fval;
    byte  b_val;
    */

    // if in interactive command mode, dont send records to the log
    if (_inCommandMode == ON) return;

    // build up a string & write it all at once to save function call overhead
    // nested if stmts take less space - deal with that late

//FIX: have FDR_rec_header write to buffer & let action at bottom calc checksum & write to OpenLog
//FIX: move cksum calc to one loop at bottom.    
    switch (type) {

      case (FDR_REC_HEADER):          //header descriptor record
      
        // output: start byte
        _serialPort -> write(_buffer,1);
        cksum = FDR_RECORD_SS;

        // output: type
        _serialPort -> write(type);

        // count length of descriptor & label strings
        for (uint8_t i=0; i < FDR_REC_NUMTYPES; i++) 
          len+= 1+ strlen((const char *)_descriptors[i].descriptor)
                 + strlen((const char *)_descriptors[i].labels) +2;  // 1+ for type byte, +2 for \0 string term.

        // output: length of FDR_REC_HEADER type record
        _serialPort -> write((uint8_t *)&len,2);
        cksum += len;

        // output: write out each descriptor record
        for (uint8_t i=0; i < FDR_REC_NUMTYPES; i++) {
          // type
          _serialPort -> write(_descriptors[i].type);
          cksum += _descriptors[i].type;
          
          // descriptor string
          for(dptr = _descriptors[i].descriptor; *dptr; dptr++) cksum += *dptr;
          _serialPort -> write(_descriptors[i].descriptor, dptr - _descriptors[i].descriptor);
          _serialPort -> write((uint8_t) 0);  //String terminator;
          //TODO: increase length of write by 1 to pick up terminating 0 without extra write.
          
          // labels string
          for(dptr = _descriptors[i].labels; *dptr; dptr++) cksum += *dptr;
          _serialPort -> write(_descriptors[i].labels, dptr - _descriptors[i].labels);
          _serialPort -> write((uint8_t) 0);  //String terminator;
          //TODO: increase length of write by 1 to pick up terminating 0 without extra write.
        }
        
        // finally write chksum (truncated to one byte)
        cksum_b = cksum;  // not sure if we could trust a cast here.  Test later
        _serialPort -> write(cksum_b);
 
      break; // end case 0: 

#define DUMP(X) vptr=(uint8_t *)&X;for(uint8_t i=0;i<sizeof(X);i++)*bptr++=*vptr++; 
#define DUMPA(X,Y) vptr=(uint8_t *)X;for(uint8_t i=0;i<Y;i++)*bptr++=*vptr++;  //addr, size 

      //  {FDR_REC_FLIGHT,  "Thhaib", "raw Alt,sm Alt,gndZacc,Throttle,AH Flg" },
      case (FDR_REC_FLIGHT):
        DUMP(type);                             // Type
        DUMPA(&currentTime,3);             		//T TimeStamp
        i_val=baroRawAltitude*100;				// Raw Alt - send as 2B int instead of float
        DUMP(i_val); //h
        i_val=baroAltitude*100;					// Smoothed Alt
        DUMP(i_val); //h
        i_val = (earthAccel[ZAXIS] * 1000);		// Earth Z Axis Accel
        DUMP(i_val); //a

        DUMP(throttle);               			// Throttal value
        DUMP(motorCommand[0]);                          //i Motor cmd - assuming stable level hover so only dump one
        DUMP(altitudeHoldState);

      break;  // end case 1
/*
      case (FDR_REC_GPS): 
      {
        uint8_t  gpsData;
        DUMP(type);
        DUMPA(&currentTime,3);              //T
        //DUMP(GPS.Time);                       //l Time in ms from start of week
        DUMP(GPS.Lattitude);                  //l Lattitude in degrees
        DUMP(GPS.Longitude);                  //l Longitude in degrees
        DUMPA(&GPS.Altitude,2);               //i Altitude in cm - dont need 4 bytes
        gpsData=(GPS.NumSats) | (GPS.Fix << 7);
        DUMP(gpsData);                        //b Flag indicating GPS had a fix & now many Sats     
      }              
      // throttle, motor out battery 
      break;
      case (FDR_REC_ITOD):
        DUMP(type);
        DUMPA(&currentTime,3); 
        DUMP(altitude.rawAltitude); //f
        f_val = tempFlightAngle.getData(ROLL);
        DUMP(f_val); //f
        f_val = tempFlightAngle.getData(PITCH);
        DUMP(f_val); //f
        f_val = tempFlightAngle.getData(YAW);
        DUMP(f_val); //f
        i_val = (tempFlightAngle.groundZaccel * 1000);
        DUMP(i_val); //a
            break;
            
      case (FDR_REC_ALTHOLD):
        DUMP(type);
        DUMPA(&currentTime,3); 
    
        i_val=holdAltitude*100;
        DUMP(i_val); //h
        //DUMP(holdAltitude);                   //f Altitude to hold
        DUMP(holdThrottle);                   //i Throttle value used for holding - overload with AltHold flag
        DUMP(altitudeHold); 
            break;
            
      case (FDR_REC_VRSWITCH):
        DUMP(type);
        DUMPA(&currentTime,3); 
    
        b_val=auxSwitch.getState();
        b_val |= flightMode << 5;            
        DUMP(b_val); //h
            break;
            
      case (FDR_REC_ALTPID):
        DUMP(type);
        DUMPA(&currentTime,3);
        i_val=holdAltitude*100;
        DUMP(i_val); //h 
        i_val=altitude.getData()*100;
        DUMP(i_val);
        DUMP(throttleAdjust);                 //i Throttle adjustments for alt hold 
        i_val = motors.getMotorCommand(FRONT);
        DUMP(i_val);                          //i Motor cmd - assuming stable level hover so only dump one
            break;
            
       case (FDR_REC_BARODATA):
         DUMP(type);
         DUMPA(&currentTime,3);
         b_val = auxSwitch.getState();
         DUMP(b_val);
         DUMP(altitude.tb5);
         //f_val = altitude.getRawData();
         DUMP(altitude.altitude);
         //DUMP(altitude.groundb5);
         //DUMP(altitude.wotAltitude);
         //i_val = (flightAngle.groundZaccel * 1000);
         //DUMP(i_val); //a
         break;
         
       case (FDR_REC_BAROGND):
         DUMP(type);
         DUMPA(&currentTime,3);
         DUMP(altitude.groundTemperature);
         DUMP(altitude.groundPressure);
         DUMP(altitude.groundAltitude);
         break;
*/
    default:
      break;
    }

    // calculate checksum and write data.  Not for header record.
    if (type != FDR_REC_HEADER) {
      // Calculate & populate checksum
      for (; iptr<bptr; iptr++) cksum += *iptr;
      cksum_b = cksum; //truncate
      *bptr = cksum_b;
      _serialPort -> write(_buffer,((uint16_t) bptr - (uint16_t) &_buffer)+1);
    }
    Serial.println("Dumped record");
  }


  //
  // Invoked from readSerialCommand to enter interactive command mode
  // This allows a user to interact with the Openlog based FDR through
  // the AQ Configurator serial window.  
  //
  // NOTE: There are delays induced entering command mode so this
  //       should NOT be done while in flight.  
  //
  // TODO: Add a check for Armed Motors & reject request if AQ is armed.
  //
  void enterInteractiveMode () {  
    _inCommandMode = ON;
    enterCommandMode(FDR_CMDRETRIES);
	Serial.println("entered interactive command mode");
  } // end enterInteractiveMode()
  
  
  //
  // Send escape sequence to OpenLog to get it to drop
  // into command mode. Returns # tries it took to see cmd prompt
  // from OpenLog or 0 if timeout waiting after sending esc sequence
  //
  uint8_t enterCommandMode (uint8_t numTries) {

    uint8_t           i;
    uint32_t          lastCharTime;
    uint32_t          currCharTime;

    for (i=1; i<=numTries; i++) {
      
      _serialPort -> flush();
      _serialPort -> write(FDR_OL_ESCSEQ);
      //_serialPort -> write("\r"); // in case we were already in cmd mode
      
      lastCharTime = millis();
      while (lastCharTime + 50 > (currCharTime = millis())) {
        while (_serialPort -> available()) {
          if (_serialPort -> read() == FDR_OL_PROMPT) {
            Serial.print(FDR_OL_PROMPT);
            return(i);
          }
          else
            lastCharTime = currCharTime;
        }
      }
      
    } // end for()
    
    return(0);
  } // end enterCommandMode()
 

  
  //
  // Invoked from readSerialCommand to enter interactive command mode
  // 
  void exitCommandMode () {
    
    digitalWrite (OPENFDR_PIN, LOW);
    delay(1);
    digitalWrite (OPENFDR_PIN, HIGH);
    delay(2200);
    
    //_serialPort -> write("\rrestart\r");      // tell OpenLog to restart logging
    //delay(200);                                    // give OpenLog time to reset.
    _inCommandMode = OFF;
    dumpRecord(FDR_REC_HEADER);                    // dump decoding header record into log
    delay(10);                                    // give header time to flush before returning                            

    
  } // end exitCommandMode()



  //
  //  Couples normal Serial0 port to OpenLog Serial port
  //  allowing interaction with OpenLogs command line interface
  //  Note - dont want to do this if we are flying.
  //
  void interactiveCommandMode () {
    uint8_t        c;
    
    if (_inCommandMode == OFF) return;

    // Read user, write to logger
    while (Serial.available()) {
      c=Serial.read();

      // command escape char - exit interactive mode
      if (c == FDR_CMDESC) 
        exitCommandMode();
      else 
        _serialPort -> write(c); 
    }

    // Read logger, write to user   
    while (_serialPort -> available()) {
      c=_serialPort -> read(); 
      Serial.write(c);
    }

  } // end interactiveCommandMode()

};

#endif //#define _AQ_FDR_H_



