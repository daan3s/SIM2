

int DataIN(){
  //need to make this not part advance until a command has been given
  Serial.println("input a number pls : ")
  do{
    if(Serial.available()){
      String input = Serial.readStringUntil('\n');  
      input.trim();  

      if(isNumber(input)){
        input.toInt()
        return(input);
      }
    }
  }while(Serial.available()!);
}


bool isNumber(String str) {
    if (str.length() == 0) return false;
    for (unsigned int i = 0; i < str.length(); i++) {
        if (!isDigit(str[i])) {
            return false;
        }
    }
    return true;
  
