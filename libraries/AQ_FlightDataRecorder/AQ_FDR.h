#define	 FDR_OL_DTR_PIN		   32
#define  FDR_CMDDTR			   '@'
#define  FDR_REC_ACCELS		   9
  {FDR_REC_FLIGHT,  "Thhfiib", "raw Alt,sm Alt,earthZ,Thr Adj,Mtr Cmd,AH Flg" }, 
  {FDR_REC_ACCELS,  "Tffffff", "earthX,earthY,earthZ,accX,accY,accZ" }, 
    }
	
	// for some very strange reason OpenLog is not recording data unless we
	// force an escape followed by a reset of Openlog.  To be investiaged
    _serialPort -> begin(baud);

      //  {{FDR_REC_FLIGHT,  "Thhfiib", "raw Alt,sm Alt,earthZ,Thr Adj,Mtr Cmd,AH Flg" }, 
      
      

        DUMP(earthAccel[ZAXIS]);

      
      case (FDR_REC_ACCELS):
        DUMP(type);                             // Type
        DUMPA(&currentTime,3);             		//T TimeStamp
        DUMP(earthAccel[XAXIS]);
		DUMP(earthAccel[YAXIS]);
        DUMP(earthAccel[ZAXIS]);
        DUMP(meterPerSecSec[XAXIS]);
        DUMP(meterPerSecSec[YAXIS]);
        DUMP(meterPerSecSec[ZAXIS]);

      break;  // end case 9
/*      
      case (FDR_REC_TEST): 
		vptr=(uint8_t *)&"ABCDEF Testing all data";for(uint8_t i=0;i<strlen("ABCDEF Testing all data");i++)*bptr++=*vptr++;
		break;
*/
      //_serialPort -> flush();
    digitalWrite (FDR_OL_DTR_PIN, LOW);
    digitalWrite (FDR_OL_DTR_PIN, HIGH);
    delay(20);
    dumpRecord(FDR_REC_HEADER);                    // dump decoding header record into log                           

    
while (_inCommandMode == ON) {
}
uint8_t inCommandMode() {
	return (_inCommandMode);
}

