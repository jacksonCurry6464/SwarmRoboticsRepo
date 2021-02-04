

int sensorPin1 = A0;    // select the input pin for the potentiometer
int sensorPin2 = A1;    // select the input pin for the potentiometer
int outputPin1 = 8;      // select the pin for the LED
int outputPin2 = 7;      // select the pin for the LED
int sensorValue = 20;  // variable to store the value coming from the sensor
int leftInput;
int rightInput;
int threshHoldValueLeft = 750;
int threshHoldValueRight = 800;

// Ip left right
// .181 600 600
// .99 990 980
// .75 750 750
// .42 450 450
// .205 995 995
void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(outputPin1, OUTPUT);
  pinMode(outputPin2, OUTPUT);
  // turn the left IR pin on
  //digitalWrite(ledPin1, HIGH);
  // turn the right IR pin on
  //digitalWrite(ledPin2, HIGH);
  Serial.begin(115200);
}

void loop() {
  // read the value from the sensor:
  leftInput = analogRead(sensorPin1);
  rightInput = analogRead(sensorPin2);
  //Serial.println(leftInput);
  //Serial.print(' ');
  //Serial.println(rightInput);

  // stop the program for <sensorValue> milliseconds:
  delay(sensorValue);

  if(leftInput > threshHoldValueLeft)
  {
    digitalWrite(outputPin1,HIGH);
  }
  else
  {
    digitalWrite(outputPin1,LOW);
  }


  if(rightInput > threshHoldValueRight)
  {
    digitalWrite(outputPin2,HIGH);
  }
  else
  {
    digitalWrite(outputPin2,LOW);
  }

  
  // stop the program for for <sensorValue> milliseconds:
  delay(sensorValue);
}
