#include <Arduino.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include <WebServer.h>

// ===== Ustawienia Access Pointa =====
const char* ap_ssid = "Dummy_remote";   // nazwa sieci
const char* ap_pass = "123456789";      // hasło (min. 8 znaków)

// ===== Serwer HTTP =====
WebServer server(80);

// ---------- PARAMETRY ----------
#define NUM_STEPPERS        6
#define STEPS_PER_REV       200
#define MICROSTEPS          16
#define STEP_CM             0.5f     // krok kartezjański na klik
#define STEP_DEG            1.0f     // krok kątowy na klik
#define EPS_STEPS           3        // tolerancja do "celu"
#define Arm_length          18.0f    // długość ramion

// Stałe pomocnicze do kinematyki
#define DEG2RAD (PI/180.0f)
#define RAD2DEG (180.0f/PI)

// Przyjęte wymiary "narzędzia" (offset od środka nadgarstka do końcówki)
#define TOOL_X 0.0f
#define TOOL_Y 0.0f
#define TOOL_Z 0.0f

// Fragment tabeli D–H używany tylko do wyznaczenia R0_3 (orientacja)
// n  θ_off  d    a           α
// 1   0     d1   0           90
// 2   90    0    a2=Arm_len  0
// 3   0     0    0           90
static const float DH_d[3]            = { 10.0f,      0.0f,       0.0f };
static const float DH_a[3]            = { 0.0f,      Arm_length, 0.0f };
static const float DH_alpha_deg[3]    = { 90.0f,     0.0f,       90.0f };
static const float DH_theta_off_deg[3]= { 0.0f,      90.0f,      0.0f };

// UART (pilot)
#define UART_RX 16
#define UART_TX 17
HardwareSerial& LINK = Serial2;
static const uint32_t BAUD = 9600;

// LED
#define LED_PIN 2

// PINY DRV
#define STEP_PIN0 33
#define DIR_PIN0  32
#define STEP_PIN1 22
#define DIR_PIN1  23
#define STEP_PIN2 26
#define DIR_PIN2  25
#define STEP_PIN3 14
#define DIR_PIN3  27
#define STEP_PIN4 18
#define DIR_PIN4  19
#define STEP_PIN5 13
#define DIR_PIN5  21

const int STEP_PINS[NUM_STEPPERS] = { STEP_PIN0, STEP_PIN1, STEP_PIN2, STEP_PIN3, STEP_PIN4, STEP_PIN5 };
const int DIR_PINS [NUM_STEPPERS] = { DIR_PIN0,  DIR_PIN1,  DIR_PIN2,  DIR_PIN3,  DIR_PIN4,  DIR_PIN5  };

// Przełożenia: obr./silnika na obr./przegubu
const float GEAR[NUM_STEPPERS] = { 6.0f, 6.0f, 4.0f, 2.0f, 2.0f, 1.0f };
// Polaryzacje DIR (+1 normalnie, -1 odwróć)
const int   DIR_SIGN[NUM_STEPPERS] = { -1, -1, +1, -1, +1, -1 };

AccelStepper* M[NUM_STEPPERS] = { nullptr };
float MAX_V[NUM_STEPPERS];     // [steps/s]
float MAX_A[NUM_STEPPERS];     // [steps/s^2]

// ===== Zmienne na współrzędne z WWW =====
float X_val = 18.0f;
float Y_val = 0.0f;
float Z_val = 18.0f;
float A_val =270.0f;  // Roll
float K_val = 90.0f;  // Pitch (Pich)
float W_val = 90.0f;  // Yaw

bool newCmd6DOF = false;      // flaga nowej komendy z telefonu
bool cmdF1 = false;
bool cmdF2 = false;
bool cmdF3 = false;
bool cmdF4 = false;

// ---------- STAN / TRYBY ----------
enum { MODE_XYZ = 0, MODE_JOINTS = 1 };
enum { GROUP_13 = 0, GROUP_46 = 1 };
int MODE  = MODE_XYZ;   // domyślnie XYZ
int GROUP = GROUP_13;   // domyślnie 1-3

// Pozycja celu w XYZ (ostatni punkt)
float GOAL_X = 18.0f, GOAL_Y = 0.0f, GOAL_Z = 18.0f;

// Kąty referencyjne dla (18,0,18)
bool  REF_SET = false;
float REF_K = 0.0f, REF_A = 0.0f, REF_B = 0.0f;

// Aktualny "target" kroków dla wszystkich osi
long TARGET_STEPS[NUM_STEPPERS] = {0};

// ---------- PROSTE NARZĘDZIA ----------
static float Klamra_tryg_ochronna(float v){
  if(v<-1.f) return -1.f;
  if(v>1.f)  return  1.f;
  return v;
}

static float Zwin_do_180(float a){
  while(a<=-180.f)a+=360.f;
  while(a>180.f)a-=360.f;
  return a;
}

static void Klamra_max_zasieg(float &x, float &y, float &z){
  const float rMax = 2.0f * Arm_length - 0.5f;
  float r = sqrtf(x*x + y*y + z*z);

  if (r > rMax && r > 0.0f) {
    float s = rMax / r;
    x *= s;
    y *= s;
    z *= s;
  }
}


static long Steps_for_deg(int Gear_i, float deg){
  float turns = deg / 360.0f;
  float motorTurns = turns * GEAR[Gear_i];
  float steps = motorTurns * STEPS_PER_REV * MICROSTEPS;
  return lroundf(steps);
}

static void Invers_kinematic(float x, float y, float z, float &K, float &A, float &B){
  float rxy = hypotf(x, y);
  float d   = hypotf(rxy, z);
  float s   = Klamra_tryg_ochronna(d / (2.0f*Arm_length));
  float elev  = atan2f(z, rxy) * 180.0f/PI;
  float delta = acosf(s)       * 180.0f/PI;
  A = elev + delta;                 // ~90° w (18,0,18)
  B = 2.0f * asinf(s) * 180.0f/PI;  // ~90° w (18,0,18)
  K = atan2f(y, x) * 180.0f/PI;
  if (K < 0) K += 360.0f;
}

// ---------- MACIERZE / RPY / D-H (prosto) ----------

// 4x4 z parametrów D–H (θ,d,a,α) – θ i α w stopniach
static void DH_mat(float theta_deg, float d, float a, float alpha_deg, float T[4][4]){
  float th = theta_deg * DEG2RAD;
  float al = alpha_deg * DEG2RAD;

  float cth = cosf(th);
  float sth = sinf(th);
  float cal = cosf(al);
  float sal = sinf(al);

  T[0][0] = cth;
  T[0][1] = -sth*cal;
  T[0][2] =  sth*sal;
  T[0][3] =  a*cth;

  T[1][0] = sth;
  T[1][1] = cth*cal;
  T[1][2] = -cth*sal;
  T[1][3] =  a*sth;

  T[2][0] = 0.0f;
  T[2][1] = sal;
  T[2][2] = cal;
  T[2][3] = d;

  T[3][0] = 0.0f;
  T[3][1] = 0.0f;
  T[3][2] = 0.0f;
  T[3][3] = 1.0f;
}

// mnożenie macierzy 4x4: C = A*B
static void Mat4_mul(const float A[4][4], const float B[4][4], float C[4][4]){
  for(int i=0;i<4;i++){
    for(int j=0;j<4;j++){
      float s = 0.0f;
      for(int k=0;k<4;k++) s += A[i][k]*B[k][j];
      C[i][j] = s;
    }
  }
}

// wyciągnięcie macierzy 3x3 z 4x4 (górny lewy róg)
static void Mat3_from_4x4(const float T[4][4], float R[3][3]){
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      R[i][j] = T[i][j];
}

// transpozycja 3x3
static void Mat3_transpose(const float A[3][3], float AT[3][3]){
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      AT[i][j] = A[j][i];
}

// mnożenie 3x3: C = A*B
static void Mat3_mul(const float A[3][3], const float B[3][3], float C[3][3]){
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      float s = 0.0f;
      for(int k=0;k<3;k++) s += A[i][k]*B[k][j];
      C[i][j] = s;
    }
  }
}

// RPY (Z–Y–X, yaw–pitch–roll) -> macierz R
static void RPY_to_R(float phi_deg, float theta_deg, float psi_deg, float R[3][3]){
  float phi   = phi_deg   * DEG2RAD; // roll  (X)
  float theta = theta_deg * DEG2RAD; // pitch (Y)
  float psi   = psi_deg   * DEG2RAD; // yaw   (Z)

  float cphi = cosf(phi),  sphi = sinf(phi);
  float cth  = cosf(theta),sth  = sinf(theta);
  float cpsi = cosf(psi),  spsi = sinf(psi);

  R[0][0] =  cpsi*cth;
  R[0][1] =  cpsi*sth*sphi - spsi*cphi;
  R[0][2] =  cpsi*sth*cphi + spsi*sphi;

  R[1][0] =  spsi*cth;
  R[1][1] =  spsi*sth*sphi + cpsi*cphi;
  R[1][2] =  spsi*sth*cphi - cpsi*sphi;

  R[2][0] = -sth;
  R[2][1] =  cth*sphi;
  R[2][2] =  cth*cphi;
}

// R -> RPY (Z–Y–X)
static void RPY_from_R(const float R[3][3], float &phi_deg, float &theta_deg, float &psi_deg){
  float r11 = R[0][0];
  float r21 = R[1][0];
  float r31 = R[2][0];
  float r32 = R[2][1];
  float r33 = R[2][2];

  float theta = -asinf(Klamra_tryg_ochronna(r31));
  float phi   = atan2f(r32, r33);
  float psi   = atan2f(r21, r11);

  phi_deg   = phi   * RAD2DEG;
  theta_deg = theta * RAD2DEG;
  psi_deg   = psi   * RAD2DEG;
}

// R0_3 z kątów K,A,B + tabeli D–H (tylko orientacja)
static void R03_from_KAB(float K_deg, float A_deg, float B_deg, float R0_3[3][3]){
  float T01[4][4], T12[4][4], T23[4][4];
  float T02[4][4], T03[4][4];

  float th1 = K_deg + DH_theta_off_deg[0];
  float th2 = A_deg + DH_theta_off_deg[1];
  float th3 = B_deg + DH_theta_off_deg[2];

  DH_mat(th1, DH_d[0], DH_a[0], DH_alpha_deg[0], T01);
  DH_mat(th2, DH_d[1], DH_a[1], DH_alpha_deg[1], T12);
  DH_mat(th3, DH_d[2], DH_a[2], DH_alpha_deg[2], T23);

  Mat4_mul(T01, T12, T02);
  Mat4_mul(T02, T23, T03);

  Mat3_from_4x4(T03, R0_3);
}

// ---------- STRONA GŁÓWNA ----------
void handleRoot() {
  String page = "";
  page += "<!DOCTYPE html><html><head>";
  page += "<meta charset='UTF-8'>";
  page += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  page += "<title>Dummy remote pilot</title>";
  page += "<style>";
  page += "html { font-family: Helvetica; text-align: center; }";
  page += "body { margin-top: 40px; }";
  page += "label { display: block; margin: 8px; }";
  page += "input { padding: 5px; width: 140px; }";
  page += ".button { background-color: #4CAF50; color: white; border: none;";
  page += "padding: 10px 20px; font-size: 16px; cursor: pointer; margin-top:15px; }";
  page += "</style></head><body>";

  page += "<h1>Dummy remote pilot</h1>";
  page += "<h3>Podaj współrzędne 6DOF</h3>";

  page += "<form action='/set' method='GET'>";
  page += "<label>X    = <input type='number' step='0.01' name='X' value='" + String(X_val, 2) + "'></label>";
  page += "<label>Y    = <input type='number' step='0.01' name='Y' value='" + String(Y_val, 2) + "'></label>";
  page += "<label>Z    = <input type='number' step='0.01' name='Z' value='" + String(Z_val, 2) + "'></label>";
  page += "<label>Roll = <input type='number' step='0.01' name='Roll' value='" + String(A_val, 2) + "'></label>";
  page += "<label>Pich = <input type='number' step='0.01' name='Pich' value='" + String(K_val, 2) + "'></label>";
  page += "<label>Yaw  = <input type='number' step='0.01' name='Yaw' value='" + String(W_val, 2) + "'></label>";

  // Twoje istniejące przyciski
  page += "<input class='button' type='submit' name='action' value='Wyślij'>";
  page += "<input class='button' type='submit' name='action' value='save'>";
  page += "<input class='button' type='submit' name='action' value='play'>";

  // ---- NOWE 4 PRZYCISKI ----
  // Używam <button>, żeby tekst na przycisku mógł być ładny,
  // a value (action) krótkie i stabilne w kodzie.
  page += "<br>";
  page += "<button class='button' type='submit' name='action' value='f1'>Linia prosta</button>";
  page += "<button class='button' type='submit' name='action' value='f2'>Linia krzywa</button>";
  page += "<button class='button' type='submit' name='action' value='f3'>okrag</button>";
  page += "<button class='button' type='submit' name='action' value='f4'>chwyt</button>";

  page += "</form>";

  page += "<h3>Aktualne wartości na ESP32:</h3>";
  page += "X = " + String(X_val, 2) + "<br>";
  page += "Y = " + String(Y_val, 2) + "<br>";
  page += "Z = " + String(Z_val, 2) + "<br>";
  page += "Roll = " + String(A_val, 2) + "<br>";
  page += "Pich = " + String(K_val, 2) + "<br>";
  page += "Yaw = " + String(W_val, 2) + "<br>";

  page += "</body></html>";

  server.send(200, "text/html", page);
}

void handleSet() {
  // Zaczytanie wartości z formularza (tak jak było)
  if (server.hasArg("X"))    X_val = server.arg("X").toFloat();
  if (server.hasArg("Y"))    Y_val = server.arg("Y").toFloat();
  if (server.hasArg("Z"))    Z_val = server.arg("Z").toFloat();
  if (server.hasArg("Roll")) A_val = server.arg("Roll").toFloat();
  if (server.hasArg("Pich")) K_val = server.arg("Pich").toFloat();
  if (server.hasArg("Yaw"))  W_val = server.arg("Yaw").toFloat();

  Serial.println("Nowe współrzędne (z WWW):");
  Serial.print("X=");    Serial.println(X_val);
  Serial.print("Y=");    Serial.println(Y_val);
  Serial.print("Z=");    Serial.println(Z_val);
  Serial.print("Roll="); Serial.println(A_val);
  Serial.print("Pich="); Serial.println(K_val);
  Serial.print("Yaw=");  Serial.println(W_val);

  String action = "Wyślij";
  if (server.hasArg("action")) {
    action = server.arg("action");
  }

  if (action == "save") {
    Serial.println("save z WWW");
    Save();
  }
  else if (action == "play") {
    Serial.println("play z WWW");
    Play();
  }
  // ---- NOWE AKCJE ----
  else if (action == "f1") {
    Serial.println("f1 z WWW");
    cmdF1 = true;   // w dalszej części kodu zrobisz if(cmdF1){...}
  }
  else if (action == "f2") {
    Serial.println("f2 z WWW");
    cmdF2 = true;
  }
  else if (action == "f3") {
    Serial.println("f3 z WWW");
    cmdF3 = true;
  }
  else if (action == "f4") {
    Serial.println("f4 z WWW");
    cmdF4 = true;
  }
  else { // "Wyślij" lub cokolwiek innego
    newCmd6DOF = true;
  }

  handleRoot();
}


// ---------- FUNKCJA 6DOF ----------
static void Move_6DOF(float x, float y, float z,
                      float phi, float theta, float psi)
{
  float x_wc = x - TOOL_X;
  float y_wc = y - TOOL_Y;
  float z_wc = z - TOOL_Z;

  Klamra_max_zasieg(x_wc, y_wc, z_wc);

  float K, A, B;
  Invers_kinematic(x_wc, y_wc, z_wc, K, A, B);

  if (!REF_SET){
    REF_K = K; REF_A = A; REF_B = B;
    REF_SET = true;
  }

  float dK = Zwin_do_180(K - REF_K);
  float dA = Zwin_do_180(A - REF_A);
  float dB = Zwin_do_180(B - REF_B);

  long t[NUM_STEPPERS];
  t[0] = Steps_for_deg(0, dK);
  t[1] = Steps_for_deg(1, dA);
  t[2] = Steps_for_deg(2, dB);

  float R0_3[3][3];
  R03_from_KAB(K, A, B, R0_3);

  float R0_6[3][3];
  RPY_to_R(phi, theta, psi, R0_6);

  float R3_0[3][3];
  float R3_6[3][3];
  Mat3_transpose(R0_3, R3_0);
  Mat3_mul(R3_0, R0_6, R3_6);

  float phi_w, theta_w, psi_w;
  RPY_from_R(R3_6, phi_w, theta_w, psi_w);

  t[3] = Steps_for_deg(3, phi_w);
  t[4] = Steps_for_deg(4, theta_w);
  t[5] = Steps_for_deg(5, psi_w);

  for(int i=0;i<NUM_STEPPERS;i++){
    M[i]->moveTo(t[i]);
    TARGET_STEPS[i] = t[i];
  }

  Same_time();

  GOAL_X = x;
  GOAL_Y = y;
  GOAL_Z = z;
}

// ---------- Ustaw limity prędkości/acc ----------tancie
static void Ustaw_parametry_silnikow(float deg_per_s, float deg_per_s2){
  for(int i=0;i<NUM_STEPPERS;i++){
    float stepsPerDeg = (STEPS_PER_REV * MICROSTEPS * GEAR[i]) / 360.0f;
    MAX_V[i] = stepsPerDeg * deg_per_s;
    MAX_A[i] = stepsPerDeg * deg_per_s2;
    M[i]->setMaxSpeed(MAX_V[i]);
    M[i]->setAcceleration(MAX_A[i]);
  }
}

//skaluje predkosci silników wzgledem najdłużej trasy
static void Same_time(){
  float tMin = 0.0f;
  long d[NUM_STEPPERS];
  for(int i=0;i<NUM_STEPPERS;i++){
    d[i] = abs(M[i]->distanceToGo());
    if (d[i] > 0) {
      float t = d[i] / MAX_V[i];
      if (t > tMin) tMin = t;
    }
  }
  if (tMin <= 0.0f) return;
  for(int i=0;i<NUM_STEPPERS;i++){
    if (d[i] == 0) continue;
    float v = d[i] / tMin; if (v > MAX_V[i]) v = MAX_V[i];
    M[i]->setMaxSpeed(v);
    float a = MAX_A[i] * (v / MAX_V[i]); if (a < 100.0f) a = 100.0f;
    M[i]->setAcceleration(a);
  }
}

// Czy wszystkie osie są "na celu"
static bool Na_celu(){
  for(int i=0;i<NUM_STEPPERS;i++){
    if (abs(M[i]->distanceToGo()) > EPS_STEPS) return false;
  }
  return true;
}

// ---------- QUEUE XYZ (prosto) ----------
#define Queue_limit 256
float QX[Queue_limit], QY[Queue_limit], QZ[Queue_limit];
int QH=0, QT=0, QC=0;
static void Queue_xyz_Clear(){ QH=QT=QC=0; }
static bool Queue_xyz_Push(float x,float y,float z){
  if (QC>=Queue_limit) return false;
  QX[QT]=x; QY[QT]=y; QZ[QT]=z;
  QT=(QT+1)%Queue_limit; QC++; return true;
}
static bool Queue_xyz_pop(float &x,float &y,float &z){
  if (QC==0) return false;
  x=QX[QH]; y=QY[QH]; z=QZ[QH];
  QH=(QH+1)%Queue_limit; QC--; return true;
}
static void Queue_xyz_goal(float &x,float &y,float &z){
  if (QC==0){ x=GOAL_X; y=GOAL_Y; z=GOAL_Z; return; }
  int idx = (QT+Queue_limit-1)%Queue_limit;
  x=QX[idx]; y=QY[idx]; z=QZ[idx];
}

// ---------- QUEUE JOINTS (pełne wektory kroków) ----------
long JQ[Queue_limit][NUM_STEPPERS];
int  JH=0, JT=0, JC=0;
static void Queue_jq_clear(){ JH=JT=JC=0; }
static bool Queue_jq_push(long vec[NUM_STEPPERS]){
  if (JC>=Queue_limit) return false;
  for(int i=0;i<NUM_STEPPERS;i++) JQ[JT][i]=vec[i];
  JT=(JT+1)%Queue_limit; JC++; return true;
}
static bool Queue_jq_pop(long vec[NUM_STEPPERS]){
  if (JC==0) return false;
  for(int i=0;i<NUM_STEPPERS;i++) vec[i]=JQ[JH][i];
  JH=(JH+1)%Queue_limit; JC--; return true;
}
static void Queue_jq_current(long vec[NUM_STEPPERS]){
  if (JC==0){
    for(int i=0;i<NUM_STEPPERS;i++) vec[i]=TARGET_STEPS[i];
    return;
  }
  int idx = (JT+Queue_limit-1)%Queue_limit;
  for(int i=0;i<NUM_STEPPERS;i++) vec[i]=JQ[idx][i];
}

// ---------- PROGRAM (Save/Play) ----------
enum { PROG_EMPTY=0, PROG_XYZ=1, PROG_JOINTS=2 };
#define PROG_LIMIT 256
int   PROG_KIND = PROG_EMPTY;
int   PROG_COUNT = 0;
float PX[PROG_LIMIT], PY[PROG_LIMIT], PZ[PROG_LIMIT];          // XYZ
long  Prog_step[PROG_LIMIT][NUM_STEPPERS];                     // JOINTS

static void prog_clear(){ PROG_KIND=PROG_EMPTY; PROG_COUNT=0; }
static bool prog_add_xyz(float x,float y,float z){
  if (PROG_COUNT>=PROG_LIMIT) return false;
  if (PROG_KIND==PROG_JOINTS){ prog_clear(); }
  PROG_KIND=PROG_XYZ;
  PX[PROG_COUNT]=x; PY[PROG_COUNT]=y; PZ[PROG_COUNT]=z; PROG_COUNT++;
  return true;
}
static bool prog_add_joints(const long vec[NUM_STEPPERS]){
  if (PROG_COUNT>=PROG_LIMIT) return false;
  if (PROG_KIND==PROG_XYZ){ prog_clear(); }
  PROG_KIND=PROG_JOINTS;
  for(int i=0;i<NUM_STEPPERS;i++) Prog_step[PROG_COUNT][i]=vec[i];
  PROG_COUNT++;
  return true;
}

// ---------- LOGIKA XYZ ----------
static void Move_to_xyz(float x,float y,float z){
  Klamra_max_zasieg(x,y,z);
  float K,A,B; Invers_kinematic(x,y,z,K,A,B);
  if (!REF_SET){
    REF_K=K; REF_A=A; REF_B=B; REF_SET=true;
  }

  float dK = Zwin_do_180(K-REF_K);
  float dA = Zwin_do_180(A-REF_A);
  float dB = Zwin_do_180(B-REF_B);

  long t0 = Steps_for_deg(0,dK);
  long t1 = Steps_for_deg(1,dA);
  long t2 = Steps_for_deg(2,dB);

  M[0]->moveTo(t0); TARGET_STEPS[0]=t0;
  M[1]->moveTo(t1); TARGET_STEPS[1]=t1;
  M[2]->moveTo(t2); TARGET_STEPS[2]=t2;

  for(int i=3;i<NUM_STEPPERS;i++){
    M[i]->moveTo(TARGET_STEPS[i]);
  }

  Same_time();
  GOAL_X=x; GOAL_Y=y; GOAL_Z=z;
}

static void Next_xyz(){
  if (!Na_celu()) return;
  float nx,ny,nz;
  if (Queue_xyz_pop(nx,ny,nz)) Move_to_xyz(nx,ny,nz);
}

// ---------- LOGIKA PRZEGUBÓW ----------
static void Move_to_joints(const long vec[NUM_STEPPERS]){
  for(int i=0;i<NUM_STEPPERS;i++){
    M[i]->moveTo(vec[i]);
    TARGET_STEPS[i]=vec[i];
  }
  Same_time();
}

static void Queue_joints_move(int joint, float deltaDeg){
  long base[NUM_STEPPERS]; Queue_jq_current(base);
  base[joint] += Steps_for_deg(joint, deltaDeg);
  if (Queue_jq_push(base) && Na_celu()){
    long first[NUM_STEPPERS];
    if (Queue_jq_pop(first)) Move_to_joints(first);
  }
}

static int map_axis(char axis){
  if (GROUP==GROUP_13){
    if (axis=='X') return 0;
    if (axis=='Y') return 1;
    return 2;
  } else {
    if (axis=='X') return 3;
    if (axis=='Y') return 4;
    return 5;
  }
}

static void Next_joint(){
  if (!Na_celu()) return;
  long nx[NUM_STEPPERS];
  if (Queue_jq_pop(nx)) Move_to_joints(nx);
}

// ---------- KOMENDY WYSOKIEGO POZIOMU ----------
static uint32_t lastStopMs=0; static int stopClicks=0; const uint32_t STOP_MS=1500;

static void Save(){
  bool ok=false;
  if (MODE==MODE_XYZ){
    ok = prog_add_xyz(GOAL_X,GOAL_Y,GOAL_Z);
  } else {
    ok = prog_add_joints(TARGET_STEPS);
  }
  if (ok){
    digitalWrite(LED_PIN,HIGH);
    delay(20);
    digitalWrite(LED_PIN,LOW);
  }
}

static void Play(){
  if (PROG_COUNT==0 || PROG_KIND==PROG_EMPTY) {
    Serial.println(F("PLAY: empty")); return;
  }
  Queue_jq_clear(); Queue_xyz_Clear();
  if (PROG_KIND==PROG_XYZ){
    for(int i=0;i<PROG_COUNT;i++)
      Queue_xyz_Push(PX[i],PY[i],PZ[i]);
    if (Na_celu()) Next_xyz();
    Serial.print(F("PLAY XYZ, queued="));
    Serial.println(QC);
  } else {
    for(int i=0;i<PROG_COUNT;i++) {
      long v[NUM_STEPPERS];
      for(int j=0;j<NUM_STEPPERS;j++) v[j]=Prog_step[i][j];
      Queue_jq_push(v);
    }
    if (Na_celu()){
      long first[NUM_STEPPERS];
      if (Queue_jq_pop(first)) Move_to_joints(first);
    }
    Serial.print(F("PLAY JOINTS, queued="));
    Serial.println(JC);
  }
}

static void Stop(){
  for(int i=0;i<NUM_STEPPERS;i++) M[i]->stop();
  Queue_xyz_Clear(); Queue_jq_clear();
  uint32_t now=millis();
  if (now-lastStopMs<=STOP_MS) stopClicks++; else stopClicks=1;
  lastStopMs=now;
  if (stopClicks>=5){
    prog_clear(); stopClicks=0;
    Serial.println(F("STOP x5: program CLEARED"));
  }
}

static void setModeXYZ(){ MODE=MODE_XYZ; Serial.println(F("MODE=XYZ")); }
static void setModeJoints(){ MODE=MODE_JOINTS; GROUP=GROUP_13; Serial.println(F("MODE=JOINTS, GROUP=1-3")); }
static void setGroup13(){ GROUP=GROUP_13; Serial.println(F("GROUP=1-3")); }
static void setGroup46(){ GROUP=GROUP_46; Serial.println(F("GROUP=4-6")); }

// ---------- INICJALIZACJA ----------
static void initStart(){
  float K,A,B; Invers_kinematic(18.0f,0.0f,18.0f,K,A,B);
  REF_K=K; REF_A=A; REF_B=B; REF_SET=true;
  for(int i=0;i<NUM_STEPPERS;i++) M[i]->setCurrentPosition(0);
  Move_to_xyz(18.0f,0.0f,18.0f);
}
// ---------- Prezentacja ----------

static void Prezentacja_ServiceOnce(){
  static uint32_t lastHttp = 0;
  uint32_t now = millis();

  if (now - lastHttp >= 20) {   // 50 Hz wystarczy do UI
    server.handleClient();
    lastHttp = now;
  }

  for(int i=0;i<NUM_STEPPERS;i++) M[i]->run();
  Next_xyz();
  Next_joint();

}

static void Prezentacja_WaitStill(){
  while(!Na_celu()){
    Prezentacja_ServiceOnce();
  }
}

// zatrzymaj, wyczyść kolejki + program i zsynchronizuj TARGET_STEPS z aktualną pozycją
static void Prezentacja_Begin(){
  for(int i=0;i<NUM_STEPPERS;i++) M[i]->stop();
  Queue_xyz_Clear();
  Queue_jq_clear();
  prog_clear();

  Prezentacja_WaitStill();

  for(int i=0;i<NUM_STEPPERS;i++){
    long cur = M[i]->currentPosition();
    TARGET_STEPS[i] = cur;
    M[i]->moveTo(cur); // żeby Na_celu() pozostało true
  }
}

// dodaj punkt 6DOF do programu JOINTS, używając istniejącego Move_6DOF() "na sucho"
static bool Prezentacja_ProgAdd6DOF(float x,float y,float z, float roll,float pitch,float yaw){
  // backup stanu żeby nie rozjechać logiki ruchu
  long curPos[NUM_STEPPERS];
  for(int i=0;i<NUM_STEPPERS;i++) curPos[i] = M[i]->currentPosition();

  float oldGX = GOAL_X, oldGY = GOAL_Y, oldGZ = GOAL_Z;

  // użyj istniejącej funkcji do policzenia TARGET_STEPS[]
  Move_6DOF(x,y,z, roll,pitch,yaw);

  // skopiuj wynik do programu
  long vec[NUM_STEPPERS];
  for(int i=0;i<NUM_STEPPERS;i++) vec[i] = TARGET_STEPS[i];
  bool ok = prog_add_joints(vec);

  // przywróć pozycje/targety żeby nic nie ruszyło i Na_celu() było true
  for(int i=0;i<NUM_STEPPERS;i++){
    M[i]->setCurrentPosition(curPos[i]);
    M[i]->moveTo(curPos[i]);
    TARGET_STEPS[i] = curPos[i];
  }

  GOAL_X = oldGX; GOAL_Y = oldGY; GOAL_Z = oldGZ;

  return ok;
}

static void Prezentacja_PlayWaitAndClear(){
  Play();
  while (JC > 0 || QC > 0 || !Na_celu()){
    Prezentacja_ServiceOnce();
  }
  Queue_xyz_Clear();
  Queue_jq_clear();
  prog_clear();
}

// ------------------ F1: Linia prosta ------------------
static void Prezentacja_linia_prosto(){
  delay(1000);

  // segment 1:
  Prezentacja_Begin();
  prog_add_joints(TARGET_STEPS); // start = aktualna pozycja (bez szarpnięcia)
  Prezentacja_ProgAdd6DOF(12, 0, 18, 270, 90, 90);
  Prezentacja_ProgAdd6DOF(13, 0, 18, 270, 90, 90);
  Prezentacja_ProgAdd6DOF(14, 0, 18, 270, 90, 90);
  Prezentacja_ProgAdd6DOF(15, 0, 18, 270, 90, 90);    
  Prezentacja_ProgAdd6DOF(16, 0, 18, 270, 90, 90);
  Prezentacja_ProgAdd6DOF(17, 0, 18, 270, 90, 90);
  Prezentacja_ProgAdd6DOF(18, 0, 18, 270, 90, 90);
  Prezentacja_ProgAdd6DOF(19, 0, 18, 270, 90, 90);
  Prezentacja_ProgAdd6DOF(20, 0, 18, 270, 90, 90);
  Prezentacja_ProgAdd6DOF(21, 0, 18, 270, 90, 90);
  Prezentacja_ProgAdd6DOF(22, 0, 18, 270, 90, 90);
  Prezentacja_PlayWaitAndClear();

  delay(1000);

  // segment 2: powrót na (18,0,18)
  Prezentacja_Begin();
  prog_add_joints(TARGET_STEPS);
  Prezentacja_ProgAdd6DOF(18, 0, 18, 270, 90, 90);
  Prezentacja_PlayWaitAndClear();
}

// ------------------ F2: Linia krzywa ------------------
static void Prezentacja_linia_krzywo(){
  delay(1000);

  Prezentacja_Begin();
  prog_add_joints(TARGET_STEPS);

  Prezentacja_ProgAdd6DOF(12, 0, 17, 300, 90, 20);
  Prezentacja_ProgAdd6DOF(14, 2, 15, 310, 50, 20);
  Prezentacja_ProgAdd6DOF(16, 4, 17, 260, 20, 20);
  Prezentacja_ProgAdd6DOF(18, 2, 19, 200, 60,20);
  Prezentacja_ProgAdd6DOF(20, -1, 20, 180, 120, 20);
  Prezentacja_ProgAdd6DOF(22, 0, 18, 270, 90, 90);


  // powrót
  Prezentacja_ProgAdd6DOF(18, 0, 18, 270, 90, 90);

  Prezentacja_PlayWaitAndClear();
}

// ------------------ F3: Okrąg (3 płaszczyzny, limit PROG_LIMIT=128) ------------------
static void Prezentacja_Okrag(){
  delay(500);

  const float cx = 18.0f, cy = 0.0f, cz = 18.0f;
  const float roll = 270.0f, pitch = 90.0f, yaw = 90.0f;
  const float rad = 5.0f;

  // 3 płaszczyzny + powroty do środka muszą się zmieścić w 128:
  // total ≈ 3*PTS + 8  -> PTS=36 daje 116 punktów (bezpiecznie)
  const int PTS = 36*2;

  Prezentacja_Begin();
  prog_add_joints(TARGET_STEPS);

  // do środka
  Prezentacja_ProgAdd6DOF(cx, cy, cz, roll, pitch, yaw);

  // XY (Z stałe)
  for(int i=0;i<=PTS;i++){
    float a = (2.0f * PI) * (float)i / (float)PTS;
    float x = cx + rad * cosf(a);
    float y = cy + rad * sinf(a);
    float z = cz;
    if(!Prezentacja_ProgAdd6DOF(x,y,z, roll,pitch,yaw)) break;
  }
  Prezentacja_ProgAdd6DOF(cx, cy, cz, roll, pitch, yaw);

  // XZ (Y stałe)
  for(int i=0;i<=PTS;i++){
    float a = (2.0f * PI) * (float)i / (float)PTS;
    float x = cx + rad * cosf(a);
    float y = cy;
    float z = cz + rad * sinf(a);
    if(!Prezentacja_ProgAdd6DOF(x,y,z, roll,pitch,yaw)) break;
  }
  Prezentacja_ProgAdd6DOF(cx, cy, cz, roll, pitch, yaw);

  // YZ (X stałe)
  for(int i=0;i<=PTS;i++){
    float a = (2.0f * PI) * (float)i / (float)PTS;
    float x = cx;
    float y = cy + rad * cosf(a);
    float z = cz + rad * sinf(a);
    if(!Prezentacja_ProgAdd6DOF(x,y,z, roll,pitch,yaw)) break;
  }
  Prezentacja_ProgAdd6DOF(cx, cy, cz, roll, pitch, yaw);

  Prezentacja_PlayWaitAndClear();
}

// ------------------ F4: Długi ruch / chwyt ------------------
static void Prezentacja_dlugi_ruch(){
  delay(1000);

  // segment 1
  Prezentacja_Begin();
  prog_add_joints(TARGET_STEPS);
  Prezentacja_ProgAdd6DOF( 8, 15, 8, 220, 200,  75);
  Prezentacja_ProgAdd6DOF( 8, 15, 2, 220, 200, 115);
  Prezentacja_ProgAdd6DOF( 8, 15, 10, 240, 200,  75);
  Prezentacja_PlayWaitAndClear();

  delay(1000);

  // segment 2
  Prezentacja_Begin();
  prog_add_joints(TARGET_STEPS);
  Prezentacja_ProgAdd6DOF(18,  0, 10, 250, 200,  30);
  Prezentacja_ProgAdd6DOF(19, -8, 15, 310,  90, 110);
  Prezentacja_ProgAdd6DOF(21, -8, 15, 310,  90, 110);
  Prezentacja_ProgAdd6DOF(22, -8, 15, 310,  90, 110);
  Prezentacja_ProgAdd6DOF(23, -8, 15, 310,  90, 110);
  Prezentacja_PlayWaitAndClear();

  delay(1000);

  // segment 3: powrót
  Prezentacja_Begin();
  prog_add_joints(TARGET_STEPS);
  Prezentacja_ProgAdd6DOF(18, 0, 18, 270, 90, 90);
  Prezentacja_PlayWaitAndClear();
}



// ---------- SETUP ----------
void setup(){
  Serial.begin(BAUD);          // 9600 na USB
  LINK.begin(BAUD, SERIAL_8N1, UART_RX, UART_TX);  // 9600 na pilota
  LINK.setTimeout(5);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  for(int i=0;i<NUM_STEPPERS;i++){
    M[i] = new AccelStepper(AccelStepper::DRIVER, STEP_PINS[i], DIR_PINS[i]);
    bool inv = (DIR_SIGN[i] < 0);
    M[i]->setPinsInverted(inv, false, false);
  }

  Ustaw_parametry_silnikow(60.0f, 480.0f);
  initStart();
  setModeJoints(); setGroup13();
  Serial.println(F("READY (uplift): XYZ & JOINTS, Save/Play/Stop, pilot + WWW"));

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point uruchomiony. IP: ");
  Serial.println(IP);

  server.on("/", handleRoot);
  server.on("/set", handleSet);

  server.begin();
  Serial.println("Serwer HTTP wystartował");
}


// ---------- LOOP ----------
void loop(){
  server.handleClient();
    if (cmdF1) { cmdF1 = false; Prezentacja_linia_prosto(); }
    if (cmdF2) { cmdF2 = false; Prezentacja_linia_krzywo(); }
    if (cmdF3) { cmdF3 = false; Prezentacja_Okrag(); }
    if (cmdF4) { cmdF4 = false; Prezentacja_dlugi_ruch(); }

   //jeśli przyszła nowa komenda 6DOF z telefonu i ramię stoi -> wykonaj
  if (newCmd6DOF && Na_celu()) {
    Move_6DOF(X_val, Y_val, Z_val, A_val, K_val, W_val);
    newCmd6DOF = false;
  }

  for(int i=0;i<NUM_STEPPERS;i++) M[i]->run();
  Next_xyz();
  Next_joint();

  if (!LINK.available()) return;
  String line = LINK.readStringUntil('\n');
  line.trim();
  if (!line.startsWith("BTN:")) return;
  line.remove(0,4);

  if      (line=="Sterowanie_XYZ")        { setModeXYZ(); }
  else if (line=="Sterowanie_przegubowe") { setModeJoints(); }
  else if (line=="Przegub_13")            { setGroup13(); }
  else if (line=="Przegub_46")            { setGroup46(); }
  else if (line=="Save") { Save(); }
  else if (line=="Play") { Play(); }
  else if (line=="Stop") { Stop(); }
  else {
    if (MODE==MODE_XYZ){
      float bx,by,bz; Queue_xyz_goal(bx,by,bz);
      bool q=false;
      if      (line=="X+") q=Queue_xyz_Push(bx+STEP_CM, by,          bz);
      else if (line=="X-") q=Queue_xyz_Push(bx-STEP_CM, by,          bz);
      else if (line=="Y+") q=Queue_xyz_Push(bx,          by+STEP_CM, bz);
      else if (line=="Y-") q=Queue_xyz_Push(bx,          by-STEP_CM, bz);
      else if (line=="Z+") q=Queue_xyz_Push(bx,          by,          bz+STEP_CM);
      else if (line=="Z-") q=Queue_xyz_Push(bx,          by,          bz-STEP_CM);
      if (q && Na_celu()) Next_xyz();
    } else {
      int jx = map_axis('X');
      int jy = map_axis('Y');
      int jz = map_axis('Z');
      if      (line=="X+") Queue_joints_move(jx,+STEP_DEG);
      else if (line=="X-") Queue_joints_move(jx,-STEP_DEG);
      else if (line=="Y+") Queue_joints_move(jy,+STEP_DEG);
      else if (line=="Y-") Queue_joints_move(jy,-STEP_DEG);
      else if (line=="Z+") Queue_joints_move(jz,+STEP_DEG);
      else if (line=="Z-") Queue_joints_move(jz,-STEP_DEG);
    }
  }

  Serial.print(F("cmd: ")); Serial.print(line);
  Serial.print(F("  mode=")); Serial.print(MODE==MODE_XYZ?"XYZ":"JOINTS");
  Serial.print(F("  grp="));  Serial.print(GROUP==GROUP_13?"1-3":"4-6");
  Serial.print(F("  qXYZ=")); Serial.print(QC);
  Serial.print(F("  qJ="));   Serial.println(JC);
}
