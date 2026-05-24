#include <Arduino.h>

// ===== UART2 (do urządzenia głównego) =====
#define UART_RX 16
#define UART_TX 17
HardwareSerial& LINK = Serial2;

// ===== Piny przycisków =====
const uint8_t BTN_PINS[] = {
  18,19,21,22,23,   // BTN1..BTN5
  32,33,25,26,27,   // BTN6..BTN10
  14,               // BTN11
  13,               // BTN12
  34                // BTN13 (zewn. pull-up do 3V3)
};

// true = INPUT_PULLUP, false = INPUT (dla GPIO34 tylko INPUT)
const bool USE_PULLUP[] = {
  1,1,1,1,1,
  1,1,1,1,1,
  1,
  1,
  0
};

const char* BTN_NAMES[] = {
  "Play","Przegub_46","Przegub_13","Sterowanie_przegubowe","Sterowanie_XYZ",
  "Save","Stop","Y-","X+","Y+",
  "X-","Z+","Z-"
};

// które przyciski mają reagować na przytrzymanie (auto-repeat)
const bool AUTO_REPEAT[] = {
  0,0,0,0,0,   // Play, przeguby, tryby
  0,0,         // Save, Stop
  1,1,1,       // Y-, X+, Y+
  1,1,1        // X-, Z+, Z-
};

static_assert(
  (sizeof(BTN_PINS) == sizeof(USE_PULLUP)) &&
  (sizeof(BTN_PINS)/sizeof(BTN_PINS[0]) == sizeof(BTN_NAMES)/sizeof(BTN_NAMES[0])) &&
  (sizeof(BTN_PINS)/sizeof(BTN_PINS[0]) == sizeof(AUTO_REPEAT)/sizeof(AUTO_REPEAT[0])),
  "Tablice BTN_* musza miec ten sam rozmiar"
);

const size_t N = sizeof(BTN_PINS)/sizeof(BTN_PINS[0]);
const unsigned long DEBOUNCE_MS         = 20;
const unsigned long HOLD_FIRST_DELAY_MS = 400;  // po 0.4 s zaczyna powtarzać
const unsigned long HOLD_REPEAT_MS      = 150;  // co 0.15 s kolejny impuls

// proste stany dla debounce
bool lastStable[20];
bool lastRead[20];
unsigned long lastChangeMs[20];

// czasy powtarzania przy przytrzymaniu
unsigned long nextRepeatMs[20];

void setup() {
  // (opcjonalnie) debug na USB
  Serial.begin(9600);

  // UART2 do komunikacji z kontrolerem głównym
  LINK.begin(9600, SERIAL_8N1, UART_RX, UART_TX);

  for (size_t i=0;i<N;i++) {
    pinMode(BTN_PINS[i], USE_PULLUP[i] ? INPUT_PULLUP : INPUT);
    bool s = digitalRead(BTN_PINS[i]); // HIGH = nie wciśnięty przy pull-up
    lastStable[i]   = s;
    lastRead[i]     = s;
    lastChangeMs[i] = millis();
    nextRepeatMs[i] = 0;
  }

  Serial.println(F("Pilot: READY (UART2 @9600)"));
  LINK.println("HELLO"); // prosty sygnał startu (możesz usunąć)
}

void loop() {
  unsigned long now = millis();

  for (size_t i = 0; i < N; ++i) {
    bool r = digitalRead(BTN_PINS[i]);   // przy INPUT_PULLUP: LOW = wciśnięty

    // wykryj zmianę i zacznij odliczanie debounce
    if (r != lastRead[i]) {
      lastRead[i] = r;
      lastChangeMs[i] = now;
    }

    // po DEBOUNCE_MS uznaj zmianę za stabilną
    if ((now - lastChangeMs[i]) >= DEBOUNCE_MS && r != lastStable[i]) {
      lastStable[i] = r;

      if (r == LOW) {  // zdarzenie: WCIŚNIĘCIE
        LINK.print(F("BTN:"));
        LINK.println(BTN_NAMES[i]);

        Serial.print(F("Wcisniety: "));
        Serial.println(BTN_NAMES[i]);

        // przygotuj auto-repeat jeśli włączony dla danego przycisku
        if (AUTO_REPEAT[i]) {
          nextRepeatMs[i] = now + HOLD_FIRST_DELAY_MS;
        } else {
          nextRepeatMs[i] = 0;
        }
      } else {
        // zdarzenie: PUSZCZENIE
        // zatrzymaj auto-repeat
        nextRepeatMs[i] = 0;

        // jeśli chcesz także zdarzenie PUSZCZENIA, odkomentuj:
        /*
        LINK.print(F("REL:"));
        LINK.println(BTN_NAMES[i]);
        */
      }
    }

    // ===== OBSŁUGA PRZYTRZYMANIA (AUTO-REPEAT) =====
    if (AUTO_REPEAT[i] && lastStable[i] == LOW && nextRepeatMs[i] != 0) {
      // porównanie odporne na overflow millis()
      if ((long)(now - nextRepeatMs[i]) >= 0) {
        // traktujemy przytrzymanie jak kolejne "wciśnięcie" tego samego przycisku
        LINK.print(F("BTN:"));
        LINK.println(BTN_NAMES[i]);

        Serial.print(F("Auto-repeat: "));
        Serial.println(BTN_NAMES[i]);

        nextRepeatMs[i] = now + HOLD_REPEAT_MS;
      }
    }
  }
}
