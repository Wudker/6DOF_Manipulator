#include <Servo.h>
Servo Servo1, Servo2, Servo3;
const float armLength = 10.0;
const int servopin1 = 9;
const int servopin2 = 10;
const int servopin3 = 11;
const int potentiometerPinA = A0; 
const int potentiometerPinT = A1; 
float valueX = 0.0;
float valueY = 0.0;
float valueZ = 0.0;
float kappaAngle = 0;
int sensorValue_alpha = 0; 
int angle_pot_alpha = 0;
int sensorValue_teta = 0; 
int angle_pot_teta = 0;



void setup() {
  Servo1.attach(servopin1); //alpha
  Servo2.attach(servopin2); //tetea
  Servo3.attach(servopin3); //kappa
  Serial.begin(9600);
}

void loop() {

  // Odczytanie danych z portu szeregowego
  String input = Serial.readStringUntil('\n');
  // Znalezienie pozycji przecinków
  int firstCommaIndex = input.indexOf(',');
  int secondCommaIndex = input.indexOf(',', firstCommaIndex + 1);
  
  if (firstCommaIndex > 0 && secondCommaIndex > 0) {
    // Podzielenie ciągu na trzy części
    String part1 = input.substring(0, firstCommaIndex);
    String part2 = input.substring(firstCommaIndex + 1, secondCommaIndex);
    String part3 = input.substring(secondCommaIndex + 1);
    
    // Konwersja na liczby float
    valueX = part1.toFloat();
    valueY = part2.toFloat();
    valueZ = part3.toFloat();
    
    // Obliczenia dla płaszczyzny XY
    float baseVectorLength = sqrt(valueX * valueX + valueY * valueY);
    
    if (baseVectorLength / 2 > armLength) {
      Serial.println("Błąd: długość wektora przekracza dwa razy długość ramienia.");
      return;
    }

    kappaAngle = calculateKappaAngle(valueX, valueY);
    
    // Obliczenia dla płaszczyzny ZX
    float vectorLength = sqrt(valueX * valueX + valueZ * valueZ);
    if (vectorLength / 2 > armLength || vectorLength == 0) {
      Serial.println("Błąd: długość wektora w płaszczyźnie ZX przekracza dwa razy długość ramienia.");
      return;
    }

    float alphaAngle=0, thetaAngle=0;
    calculateAngles(vectorLength, armLength, valueX, valueZ, alphaAngle, thetaAngle);
    
    // Wypisz wyniki
    Serial.print("Kappa angle: ");
    Serial.println(kappaAngle);
    Serial.print("Theta angle: ");
    Serial.println(thetaAngle);
    Serial.print("Alpha angle: ");
    Serial.println(alphaAngle);
    
    // Ustalenia kątu alpha
      sensorValue_alpha = analogRead(potentiometerPinA); 
      angle_pot_alpha = map(sensorValue_alpha, 0, 1023, 0, 300)-115; 
      sensorValue_teta = analogRead(potentiometerPinT); 
      angle_pot_teta = map(sensorValue_teta, 0, 1023, 0, 300)-115; 


    float alphaAngle_smooth = (alphaAngle-angle_pot_alpha) / 250;
    float thetaAngle_smooth = (thetaAngle-angle_pot_teta) / 250;
    
    delay(100);
    for (int i = 0; i <= 250; i++) {
      setServoAngle(Servo1, (angle_pot_alpha +(alphaAngle_smooth * i )));
      setServoAngle(Servo2, (thetaAngle_smooth +(alphaAngle_smooth * i )));
    delay(5);
    }

    setServoAngle(Servo3, kappaAngle);
    setServoAngle(Servo1, alphaAngle);
    setServoAngle(Servo2, thetaAngle);

  

  }
}

float calculateKappaAngle(float x, float y) {
    float angle = 0;
    if (x == 0 && y == 0) {
        angle = 0;
    }
    else if (x == 0) {
        angle = (y > 0) ? 90 : 270;
    }
    else if (y == 0) {
        angle = (x > 0) ? 0 : 180;
    }
    else {
        angle = atan2(y, x) * (180.0 / PI);
        if (x > 0 && y > 0) {
            angle = angle;
        }
        if (x < 0 && y > 0) {
            angle = 270-angle;
        }
        if (x < 0 && y < 0) {
            angle =180+(180+angle);
        }
        if (x > 0 && y < 0) {
            angle = 180+(180 + angle);
        }
    }
  return angle;
}

void calculateAngles(float vectorLength, float armLength, float x, float z, float &alphaAngle, float &thetaAngle) {
  float initialAngle = (x == 0) ? 90 : atan2(z, x) * (180.0 / PI);

  thetaAngle = asin((vectorLength / 2) / armLength) * 2 * (180.0 / PI);

  alphaAngle = acos((vectorLength / 2) / armLength) * (180.0 / PI) + initialAngle;

  if (x == 0 && z != 0) {
    alphaAngle = 90;
  } else if (x != 0 && z == 0) {
    alphaAngle = 0;
  }
}

void setServoAngle(Servo &servo, float angle) {
  if (angle >= 0 && angle <= 180) {
    servo.write(angle);
  } else {
    Serial.print("Invalid angle for servo: ");
    Serial.println(angle);
  }
}
