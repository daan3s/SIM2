int readTOF(int numOfIterations){
  int succsesses = 0;
  int failures = 0;
  int data = 0;

  while(succsesses < numOfIterations && failures < numOfIterations){ // will run until it has enough data or failed mesurments
    
    delay(12);
    VL53L0X_RangingMeasurementData_t measure; // takes the actual mesurment
    
  
    TOF.rangingTest(&measure, false); // set it to 'true' to get all debug data in serial! (probably never be used)

    if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 8000 ) {  // filter failures and incorrect data
    //Serial.println(measure.RangeMilliMeter); //prints distance
    data = data + measure.RangeMilliMeter;  
    succsesses++;

    } else {
    //Serial.println(" out of range ");
    failures++;
    }
  }

  if(failures < numOfIterations){
    data = data / numOfIterations; //converts mesurments into an avarige
    return(data);
  }else{
    return (0); //failed scan or out of range mesurments
  }
}