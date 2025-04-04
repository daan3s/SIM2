//work in progress

int sweep(int anglesToScan){ 
  // scans for the diabolo
  //returns the magnitude and angle of the diabolo location

  int anglesDone = 0;
  int distSum = 0; 
  int angleSum = 0;
  int hasMesured = 0;

  for(int i; i > anglesToScan; i++){
    int dist = readTOF(3);

    if (dist =! 0){
      distSum = distSum + dist;
      angleSum = angleSum + magnitudeAngle;
      numOfMesurment++;
    }
    stepperToAngle(magnitudeAngle++);
    delay(10);
  }
  distSum = distSum/numOfMesurment;
  angleSum = angleSum/numOfMesurment;

  return(angleSum,distSum);
}