                                //TODO:  add descriptive type name string
class serialLogger /*public Print*/ {
/*
*/
  void initialize(uint8_t port, long baud) {
        _buffer[0] = FDR_RECORD_SS;  // Load first buffer char w/ start byte
        _inCommandMode = OFF;
//#ifdef  FDR_USE_DTR  
//#endif
    
    dumpRecord(FDR_REC_HEADER);
    Serial.println("Dumped record");