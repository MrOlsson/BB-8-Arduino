#include <SoftwareSerial.h>
#define LEDPORT 14

// Connect bluetooth Tx to 17 and bluetooth Rx to 16
// Bluetooth is called by Serial1
String inputString;
long inputTime = 0;

struct signals{
  double velocity;
  double omega;
  int head;
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
  pinMode(LEDPORT, OUTPUT);
  digitalWrite(LEDPORT,HIGH);
}
int oldTime=0;
short n;
void loop() {
  // put your main code here, to run repeatedly:      
  
  //ImuLoop();
  PIDLoop();
  if(millis()>oldTime+100){
    printPidIO(true);
    oldTime = millis();
    /*
    if(input.head == 1){
      digitalWrite(LEDPORT,HIGH);
    }else if(input.head == -1){
      digitalWrite(LEDPORT,HIGH);
      if(n>5){
        digitalWrite(LEDPORT, LOW);
      }if(n>9){
        n=0;
      }
      n++;
    }else{
      digitalWrite(LEDPORT,LOW);
    }
    */
  }
}

// This is called when new data is available from bluetooth device
void serialEvent2(){
  while(Serial2.available()){
    char inChar = (char)Serial2.read(); // read bit by bit and build the string.
    inputString += inChar;
    if(inChar == '\n'){ //if last bit was a newLine, read is compleate.
      int Xindex = inputString.indexOf('X');
      int Yindex = inputString.indexOf('Y');
      int Hindex = inputString.indexOf('H');
      input.omega = inputString.substring(Xindex + 1, Yindex).toInt() / 100.0;
      input.velocity = inputString.substring(Yindex + 1, Hindex).toInt() / 100.0;
      input.head = inputString.substring(Hindex+1).toInt();
      
      // fulhax för oscars enhetssteg
      /*if(inputString.substring(Yindex + 1).toInt() > 0)
        input.velocity = 1;
      else
        input.velocity = 0;
      input.omega = 0;*/
      
      inputTime = millis();
      //Debug, do not use Serial.print in event function!
      /*
      Serial.print("X: ");
      Serial.print(input.omega);
      Serial.print(" Y: ");
      Serial.println(input.velocity);
      */
      inputString = ""; // reset inputString for new data
    }
  }
}

