

SoftwareSerial BT(10, 11);  // Bluetooth RX, TX


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  BT.begin(9600); 
  delay(200);
  Serial.println("Bluetooth Ready. Enter a number:");
}

void loop() {
  // put your main code here, to run repeatedly:

    if (BT.available()) {
        String input = BT.readStringUntil('\n');  // Read input from Bluetooth
        input.trim();  

  Serial.print("Received Bluetooth Command: ");
        Serial.println(input);
}
