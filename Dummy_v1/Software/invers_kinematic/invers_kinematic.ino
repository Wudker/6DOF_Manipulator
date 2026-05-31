#include <Servo.h>
Servo Servo1, Servo2, Servo3, Servo4, Servo5;
const float armLength = 15.0;
const int servopin1 = 9; //kappa
const int servopin2 = 8;
const int servopin3 = 7;
const int servopin4 = 6;
const int servopin5 = 4;//chwytak
const int potentiometerPinA = A0; 
const int potentiometerPinT = A3; 
const int transistorPin = 10; //bramka tranzystora
float kappaAngle = 0;
float sensorValue_alpha = 0; 
float angle_pot_alpha = 0;
float sensorValue_teta = 0; 
float angle_pot_teta = 0;
float last_kappa=90, last_lambda=90;
float  valueX, valueY, valueZ;
const int czekaj=500;
int first_start=1;

const int recordDurationMs = 5000; // czas nagrywania
const int recordIntervalMs = 5;   // odstęp pomiędzy pomiarami
const int MAX_POSITIONS = 300;     // 5000ms / 5ms = 1000 próbek
int savedPositions[MAX_POSITIONS][3]; // tylko 3 wartości: alpha, theta, kappa
int positionCount = 0;


void setup() {
  pinMode(transistorPin, OUTPUT);
  Servo1.attach(servopin2); //alpha
  Servo2.attach(servopin3); //theta
  Servo3.attach(servopin1); //kappa
  Servo4.attach(servopin4); //lambda
  Servo5.attach(servopin5); //graber
  setServoAngle(Servo3, 90);
  Serial.begin(9600);}
void loop() {
  
  if(first_start==1){
      moveToPosition(0,9,9,0);
      digitalWrite(transistorPin, HIGH);
      first_start=0;
      put();

  }
  String input = Serial.readStringUntil('\n');
    if(input=="a"){

      //jak ostatnia wartosc to 0 to automat jak 180 to wskazanie na przod
         moveToPosition(0,15,5,0);
        moveToPosition(0,15,2,0);
        grab();
        moveToPosition(-15,15,15,180);
        moveToPosition(10,15,5,0);
        moveToPosition(10,15,2,0);   
        put();
        moveToPosition(10,15,5,0);
        moveToPosition(0,15,15,0);
        grab();
        put();

        
        
    }
      if(input=="b"){
        put();
        moveToPosition(0,15,15,0);
    
        }
      if(input=="=o"){
         moveToPosition(0,8,20,180);
    }
      if(input=="g"){
        grab();
    }
      if(input=="p"){
        put();
    } 
    if (input == "m") {
          float valueX,valueY, valueZ,x,y,z;
          while (Serial.available() == 0) {
            // Czekaj na dane
          }
          
          String input1 = Serial.readStringUntil('\n');


          int firstCommaIndex = input1.indexOf(',');
          int secondCommaIndex = input1.indexOf(',', firstCommaIndex + 1);

          if (firstCommaIndex > 0 && secondCommaIndex > 0) {
            // Podzielenie ciągu na trzy części
            String part1 = input1.substring(0, firstCommaIndex);
            String part2 = input1.substring(firstCommaIndex + 1, secondCommaIndex);
            String part3 = input1.substring(secondCommaIndex + 1);

            // Konwersja na liczby float
            valueX = part1.toFloat();
            valueY = part2.toFloat();
            valueZ = part3.toFloat();
            moveToPosition(valueX, valueY, valueZ,0);
          }
        }
  if (input == "rec") {
  recordCurrentPosition();
}
if (input == "play") {
  playbackPositions();
}
if (input == "clear") {
  positionCount = 0;
  Serial.println("Wyczyszczono zapisane pozycje.");
}

  
  }

void recordCurrentPosition() {
  if (positionCount >= MAX_POSITIONS) {
    Serial.println("Brak miejsca na nowe pozycje!");
    return;
  }

  // Odłącz wszystkie 3 serwa, żeby można było je poruszać ręcznie
  Servo1.detach(); // alpha
  Servo2.detach(); // theta
  Servo3.detach(); // kappa

  Serial.println("Nagrywanie rozpoczęte – poruszaj serwami ręcznie...");

  unsigned long startTime = millis();
  while (millis() - startTime < recordDurationMs) {
    if (positionCount >= MAX_POSITIONS) {
      Serial.println("Osiągnięto limit pozycji.");
      break;
    }

    // Odczyty z potencjometrów (theta i kappa)
    int thetaRaw = analogRead(potentiometerPinT); // A3
    int kappaRaw = analogRead(potentiometerPinA); // A0

    int theta = map(thetaRaw, 0, 1023, 0, 180);
    int kappa = map(kappaRaw, 0, 1023, 0, 180);

    // Odczyt alpha – potrzebujemy tymczasowo podłączyć serwo
    Servo1.attach(servopin2);
    int alpha = Servo1.read();
    Servo1.detach();

    // Zapisz próbkę
    savedPositions[positionCount][0] = alpha;
    savedPositions[positionCount][1] = theta;
    savedPositions[positionCount][2] = kappa;
    positionCount++;

    delay(recordIntervalMs);
  }

  Serial.print("Zakończono nagrywanie. Zapisano pozycji: ");
  Serial.println(positionCount);

  // Podłącz ponownie wszystkie 3 serwa
  Servo1.attach(servopin2); // alpha
  Servo2.attach(servopin3); // theta
  Servo3.attach(servopin1); // kappa
}
void playbackPositions() {
  Serial.println("Odtwarzanie zapisanych pozycji:");
  for (int i = 0; i < positionCount; i++) {
    setServoAngle(Servo1, savedPositions[i][0]); // alpha
    setServoAngle(Servo2, savedPositions[i][1]); // theta
    setServoAngle(Servo3, savedPositions[i][2]); // kappa
    delay(recordIntervalMs);
  }
  Serial.println("Odtwarzanie zakonczone:");
}
void grab(){setServoAngle(Servo5, 0);}

void put(){setServoAngle(Servo5, 90);}

void save_Last_kappa_position(float kappaAngle,float &last_kappa){
    last_kappa=kappaAngle;
}
void save_Last_lambda_position(float lambdaAngle,float &last_lambda){
    last_lambda=lambdaAngle;
}

float easing(float t) {
  return 0.5 * (1 - cos(t * PI));  // sinusoidal ease-in-out
}

void moveToPosition(float x, float y, float z, int Pointer) {
  const float maxStepPerServoDeg = 0.8;  // maksymalna zmiana kąta serwa w jednym kroku

  float baseVectorLength = sqrt(x * x + y * y);
  if (baseVectorLength / 2 > armLength) {
    Serial.println("Błąd: długość wektora przekracza dwa razy długość ramienia.");
    return;
  }

  save_Last_kappa_position(kappaAngle, last_kappa);
  kappaAngle = calculateKappaAngle(x, y);

  float alphaAngle = 0, thetaAngle = 0, valueXY, initialAngle, lambdaAngle;
  valueXY = sqrt(x * x + y * y);
  float vectorLength = sqrt(valueXY * valueXY + z * z);
  if (vectorLength / 2 > armLength || vectorLength == 0) {
    Serial.println("Błąd: długość wektora w płaszczyźnie ZX przekracza dwa razy długość ramienia.");
    return;
  }

  x = abs(x);
  z = abs(z);
  calculateAngles(vectorLength, armLength, valueXY, z, alphaAngle, thetaAngle, initialAngle);

  int korekta = (alphaAngle >= 10) ? 10 : 0;
  alphaAngle -= korekta;

  delay(500);
  sensorValue_alpha = analogRead(potentiometerPinA); 
  angle_pot_alpha = map(sensorValue_alpha, 0, 1023, 300, 0) - (122 + korekta);
  sensorValue_teta = analogRead(potentiometerPinT); 
  angle_pot_teta = map(sensorValue_teta, 0, 1023, 0, 300) - 130;

  float last_known_lambda = last_lambda;
  lambdaAngle = (Pointer == 180) ? 0 : 180 - (360 - (alphaAngle + thetaAngle + 90));
  save_Last_lambda_position(lambdaAngle, last_lambda);

  // Zmiany kątowe
  float deltaAlpha = alphaAngle - angle_pot_alpha;
  float deltaTheta = thetaAngle - angle_pot_teta;
  float deltaKappa = kappaAngle - last_kappa;
  float deltaLambda = lambdaAngle - last_known_lambda;

  // Oblicz potrzebną liczbę kroków jako największą zmianę podzieloną przez krok
  float maxDelta = max(
    max(abs(deltaAlpha), abs(deltaTheta)),
    max(abs(deltaKappa), abs(deltaLambda))
  );
  int steps = max(1, ceil(maxDelta / maxStepPerServoDeg));  // co najmniej jeden krok

  for (int i = 1; i <= steps; i++) {
    float t = easing((float)i / steps);
    float alfa = angle_pot_alpha + deltaAlpha * t;
    float teta = angle_pot_teta + deltaTheta * t;
    float kappa = last_kappa + deltaKappa * t;
    float lambda = last_known_lambda + deltaLambda * t;

    setServoAngle(Servo1, alfa);
    setServoAngle(Servo2, teta);
    setServoAngle(Servo3, kappa);
    setServoAngle(Servo4, lambda);
    delay(16);  // płynność
  }

  // Ustaw końcową pozycję
  setServoAngle(Servo1, alphaAngle);
  setServoAngle(Servo2, thetaAngle);
  setServoAngle(Servo3, kappaAngle);
  setServoAngle(Servo4, lambdaAngle);
  delay(czekaj);
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

void calculateAngles(float vectorLength, float armLength, float x, float z, float& alphaAngle, float& thetaAngle, float &initialAngle) {
     initialAngle = (x == 0) ? 90 : atan2(z, x) * (180.0 / PI);

    thetaAngle = asin((vectorLength / 2) / armLength) * 2 * (180.0 / PI);
    alphaAngle = acos((vectorLength / 2) / armLength) * (180.0 / PI) + initialAngle;

    if (x == 0 && z != 0) {
        alphaAngle = 90;
    }
    else if (x != 0 && z == 0) {
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