#include <SoftwareSerial.h>

// Connect bluetooth Tx to 17 and bluetooth Rx to 16
// Bluetooth is called by Serial1
String inputString;
long inputTime = 0;

struct signals{
  double velocity;
  double omega;
};

signals input;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600);
  inputString.reserve(200);  
  //ImuSetup();
  VelocitySetup();
  PIDSetup();

}

void loop() {
  // put your main code here, to run repeatedly:      
  
  //ImuLoop();
  PIDLoop();
  printPidIO(true);
  /*if((millis() - inputTime) > 5000 && inputTime != 0){
    setMotorSpeed(0, 0);
    Serial.println("shut off motors");
  }*/
}

// This is called when new data is available from bluetooth device
void serialEvent2(){
  while(Serial2.available()){
    char inChar = (char)Serial2.read(); // read bit by bit and build the string.
    inputString += inChar;
    if(inChar == '\n'){ //if last bit was a newLine, read is compleate.
      int Xindex = inputString.indexOf('X');
      int Yindex = inputString.indexOf('Y');
      input.omega = inputString.substring(Xindex + 1, Yindex).toInt() / 100.0;
      input.velocity = inputString.substring(Yindex + 1).toInt() / 100.0;

      // fulhax för oscars enhetssteg
      /*if(inputString.substring(Yindex + 1).toInt() > 0)
        input.velocity = 1;
      else
        input.velocity = 0;
      input.omega = 0;*/
      
      inputTime = millis();
      //Debug, do not use Serial.print in event function!
      Serial.print("X: ");
      Serial.print(input.omega);
      Serial.print(" Y: ");
      Serial.println(input.velocity);
      
      inputString = ""; // reset inputString for new data
    }
  }
}

