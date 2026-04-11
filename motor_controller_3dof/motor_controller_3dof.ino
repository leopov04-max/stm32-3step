// ==========================================
// DIEU KHIEN 3 TRUC STEPPER MOTOR (3-DOF)
// Nucleo-F446RE + TMC2208 UART x3
// TMCStepper Library
// ==========================================
// - UART cau hinh driver: dong dien, microstep, StealthChop
// - STEP/DIR dieu khien chuyen dong
// - Bresenham noi suy dong thoi 3 truc
// ==========================================
// LUU Y UART:
//   USART2 (PA2/PA3) da dung cho ST-Link VCP (Serial debug)
//   Truc X --> USART1 : TX=PA9(D8),  RX=PA10(D2)
//   Truc Y --> USART3 : TX=PB10(D6), RX=PC11(CN10-34)
//   Truc Z --> USART6 : TX=PC6(CN10-4), RX=PC7(D9)
// ==========================================

#include <TMCStepper.h>

#define NUM_AXES 3

// ==========================================
// CAU HINH CHAN KET NOI (Nucleo-F446RE)
// ==========================================

// --- Truc X ---
#define X_DIR_PIN   PA4    // A2
#define X_STEP_PIN  PA1    // A1
#define X_EN_PIN    PA6    // D12
#define X_SW_PIN    PA7    // D11
// PA9(D8)  -> 1kOhm -> PDN_UART driver X
// PA10(D2) -> truc tiep  -> PDN_UART driver X
HardwareSerial SerialX(PA10, PA9);   // RX, TX  [USART1]

// --- Truc Y ---
// LUU Y: PB3(D3)/PB4(D5) la chan JTAG, PB10(D6) la USART3_TX -> KHONG dung!
#define Y_DIR_PIN   PA0    // A0
#define Y_STEP_PIN  PC1    // A4 (CN8 pin5)
#define Y_EN_PIN    PB6    // D10
#define Y_SW_PIN    PB0    // A3
// PC10(CN7 pin1) -> 1kOhm -> PDN_UART driver Y
// PC11(CN7 pin3) -> truc tiep -> PDN_UART driver Y
HardwareSerial SerialY(PC11, PC10);  // RX, TX  [USART3]

// --- Truc Z ---
#define Z_DIR_PIN   PC0    // A5 (CN8 pin6)
#define Z_STEP_PIN  PA8    // D7
#define Z_EN_PIN    PB9    // D14
#define Z_SW_PIN    PC9    // CN5 pin 10 (Morpho) - doi tu PB8(D15) vi I2C1_SCL conflict
// PC6(CN10-4) -> 1kOhm -> PDN_UART driver Z
// PC7(D9)     -> truc tiep -> PDN_UART driver Z
HardwareSerial SerialZ(PC7, PC6);    // RX, TX  [USART6]

// ==========================================
// THONG SO DRIVER (ap dung cho ca 3 truc)
// ==========================================
#define R_SENSE          0.11f
#define RUN_CURRENT_MA   800    // Dong dien chay (mA)
#define HOLD_CURRENT_MA  400    // Dong dien giu (mA)
#define MICROSTEPS       16     // Vi buoc: 1,2,4,8,16,32,64,128,256

// Driver objects
TMC2208Stepper driverX(&SerialX, R_SENSE);
TMC2208Stepper driverY(&SerialY, R_SENSE);
TMC2208Stepper driverZ(&SerialZ, R_SENSE);
TMC2208Stepper* drivers[NUM_AXES] = {&driverX, &driverY, &driverZ};

// ==========================================
// THONG SO HIEU CHINH CO KHI (tung truc)
// ==========================================
// STEPS_PER_MM se duoc tinh lai tu microstep thuc te
// Cong thuc: MOTOR_STEPS_PER_REV * microstep / MM_PER_REV
#define MOTOR_STEPS_PER_REV  200   // Full step/vong
#define MM_PER_REV           8.0f  // Buoc vit me (mm/vong) - DIEU CHINH THEO THUC TE
float STEPS_PER_MM[NUM_AXES] = {80.0, 80.0, 80.0};  // Se tinh lai trong setup

// Mang chan de truy cap bang chi so truc
const int DIR_PINS[NUM_AXES]  = {X_DIR_PIN,  Y_DIR_PIN,  Z_DIR_PIN};
const int STEP_PINS[NUM_AXES] = {X_STEP_PIN, Y_STEP_PIN, Z_STEP_PIN};
const int EN_PINS[NUM_AXES]   = {X_EN_PIN,   Y_EN_PIN,   Z_EN_PIN};
const int SW_PINS[NUM_AXES]   = {X_SW_PIN,   Y_SW_PIN,   Z_SW_PIN};
const char AXIS_NAMES[NUM_AXES] = {'X', 'Y', 'Z'};

// Trang thai cac truc
long currentPositionSteps[NUM_AXES] = {0, 0, 0};
bool isHomed[NUM_AXES] = {false, false, false};

// Thong so homing - tinh rieng tung truc tu microstep thuc te
long maxHomingSteps[NUM_AXES] = {20000, 20000, 20000};
int  backoffSteps[NUM_AXES]   = {160, 160, 160};

// Thoi gian debounce cong tac (microseconds)
#define SW_DEBOUNCE_US  5000  // 5ms xac nhan cong tac

// Thong so toc do chuan (cho 1/16 microstep)
#define BASE_HOMING_DELAY_US  800
#define BASE_MOVE_DELAY_US    400
#define BASE_MICROSTEP        16

// Delay thuc te tinh theo microstep tung truc (tinh trong setup)
// VD: 1/256 -> delay = 800*16/256 = 50us (cung toc do vat ly)
int homingDelayUs[NUM_AXES] = {BASE_HOMING_DELAY_US, BASE_HOMING_DELAY_US, BASE_HOMING_DELAY_US};
int moveDelayUs[NUM_AXES]   = {BASE_MOVE_DELAY_US, BASE_MOVE_DELAY_US, BASE_MOVE_DELAY_US};

// ==========================================
// HAM DEBOUNCE CONG TAC (50ms + 3 lan kiem tra)
// Dung cho Homing - do chinh xac cao
// ==========================================
bool isSwitchPressed(int axis) {
  if (digitalRead(SW_PINS[axis]) == HIGH) return false;
  delay(50);
  if (digitalRead(SW_PINS[axis]) == HIGH) return false;
  delay(20);
  if (digitalRead(SW_PINS[axis]) == HIGH) return false;
  return true;
}

// ==========================================
// KIEM TRA CONG TAC NHANH (khong delay)
// Dung trong vong lap di chuyen toc do cao
// ==========================================
bool isSwitchPressedFast(int axis) {
  if (digitalRead(SW_PINS[axis]) == HIGH) return false;
  delayMicroseconds(100);
  if (digitalRead(SW_PINS[axis]) == HIGH) return false;
  return true;
}

// ==========================================
// TINH IHOLD TU mA
// ==========================================
uint8_t scaleCurrentToIhold(int holdMa, int runMa) {
  float ratio = (float)holdMa / (float)runMa;
  if (ratio > 1.0f) ratio = 1.0f;
  if (ratio < 0.0f) ratio = 0.0f;
  return (uint8_t)(ratio * 31.0f);
}

// ==========================================
// KHOI TAO MOT DRIVER TMC2208 QUA UART
// ==========================================
void initDriver(TMC2208Stepper* drv, HardwareSerial* ser, char axis) {
  ser->begin(115200);
  drv->begin();

  // Luon cau hinh driver (khong bo qua du test_connection loi)
  drv->rms_current(RUN_CURRENT_MA);
  drv->ihold(scaleCurrentToIhold(HOLD_CURRENT_MA, RUN_CURRENT_MA));
  drv->microsteps(MICROSTEPS);
  drv->en_spreadCycle(false);  // StealthChop
  drv->pwm_autoscale(true);
  drv->pwm_autograd(true);

  Serial.print("  Driver ");
  Serial.print(axis);
  Serial.print(": ");
  Serial.flush();
  uint8_t result = drv->test_connection();
  if (result == 0) {
    Serial.println("[OK]");
  } else {
    Serial.print("[CANH BAO] Ma: ");
    Serial.print(result);
    Serial.print(" - Da cau hinh: ");
    Serial.print(drv->rms_current());
    Serial.print("mA, 1/");
    Serial.println(drv->microsteps());
  }
  Serial.flush();
}

// ==========================================
// IN TRANG THAI TAT CA 3 DRIVER
// ==========================================
void printAllDriverStatus() {
  Serial.println("\n===== TRANG THAI 3 DRIVER TMC2208 =====");
  for (int i = 0; i < NUM_AXES; i++) {
    Serial.print("--- Truc ");
    Serial.print(AXIS_NAMES[i]);
    Serial.println(" ---");
    Serial.print("  Dong RMS  : "); Serial.print(drivers[i]->rms_current()); Serial.println(" mA");
    Serial.print("  Microstep : 1/"); Serial.println(drivers[i]->microsteps());
    Serial.print("  Che do    : ");
    Serial.println(drivers[i]->en_spreadCycle() ? "SpreadCycle (luc lon)" : "StealthChop (chay em)");
    if (drivers[i]->otpw()) Serial.println("  !! CANH BAO: Nhiet do driver cao !!");
    if (drivers[i]->ot())   Serial.println("  !! LOI: Qua nhiet - driver tat !!");
    Serial.print("  Open load : A="); Serial.print(drivers[i]->ola() ? "CO" : "KO");
    Serial.print("  B="); Serial.println(drivers[i]->olb() ? "CO" : "KO");
    Serial.print("  Short GND : A="); Serial.print(drivers[i]->s2ga() ? "!! CO !!" : "KO");
    Serial.print("  B="); Serial.println(drivers[i]->s2gb() ? "!! CO !!" : "KO");
  }
  Serial.print("Vi tri hien tai: ");
  printCurrentPositions();
  Serial.println("========================================");
}

// ==========================================
// IN VI TRI HIEN TAI CUA TAT CA CAC TRUC
// ==========================================
void printCurrentPositions() {
  for (int i = 0; i < NUM_AXES; i++) {
    Serial.print(AXIS_NAMES[i]);
    Serial.print("=");
    Serial.print((float)currentPositionSteps[i] / STEPS_PER_MM[i], 2);
    Serial.print("mm");
    if (i < NUM_AXES - 1) Serial.print("  ");
  }
  Serial.println();
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("=========================================");
  Serial.println("  3-DOF + TMC2208 UART - Nucleo-F446RE");
  Serial.println("=========================================");

  for (int i = 0; i < NUM_AXES; i++) {
    pinMode(EN_PINS[i], OUTPUT);
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(SW_PINS[i], INPUT_PULLUP);
    digitalWrite(EN_PINS[i], LOW); // Bat driver
  }

  // ========== KHOI TAO 3 DRIVER TMC2208 ==========
  Serial.println("Dang khoi tao 3 driver TMC2208...");
  initDriver(&driverX, &SerialX, 'X');
  initDriver(&driverY, &SerialY, 'Y');
  initDriver(&driverZ, &SerialZ, 'Z');

  // Doc microstep THUC TE tu driver va tinh lai thong so
  Serial.println("--- Tinh lai STEPS_PER_MM tu microstep thuc te ---");
  int maxMs = 0;
  for (int i = 0; i < NUM_AXES; i++) {
    int ms = drivers[i]->microsteps();
    if (ms == 0) ms = 1;  // 0 = full step
    STEPS_PER_MM[i] = (float)(MOTOR_STEPS_PER_REV * ms) / MM_PER_REV;
    if (ms > maxMs) maxMs = ms;
    Serial.print("  "); Serial.print(AXIS_NAMES[i]);
    Serial.print(": 1/"); Serial.print(ms);
    Serial.print(" -> STEPS_PER_MM = "); Serial.println(STEPS_PER_MM[i], 1);
  }
  // Tinh MAX_HOMING_STEPS va BACKOFF RIENG TUNG TRUC
  // VD: 1/256 -> 100mm * 6400 = 640000 buoc, 1/16 -> 100mm * 400 = 40000 buoc
  Serial.println("--- Tinh homing steps tung truc ---");
  for (int i = 0; i < NUM_AXES; i++) {
    int ms = drivers[i]->microsteps();
    if (ms == 0) ms = 1;
    maxHomingSteps[i] = (long)(100.0 * MOTOR_STEPS_PER_REV * ms / MM_PER_REV);
    backoffSteps[i] = (int)(2.0 * MOTOR_STEPS_PER_REV * ms / MM_PER_REV);
    Serial.print("  "); Serial.print(AXIS_NAMES[i]);
    Serial.print(": maxSteps="); Serial.print(maxHomingSteps[i]);
    Serial.print(", backoff="); Serial.println(backoffSteps[i]);
  }

  // Tinh delay tung truc theo microstep thuc te
  // Cong thuc: BASE_DELAY * BASE_MICROSTEP / actual_microstep
  // -> Cung toc do vat ly bat ke microstep
  Serial.println("--- Tinh delay tung truc ---");
  for (int i = 0; i < NUM_AXES; i++) {
    int ms = drivers[i]->microsteps();
    if (ms == 0) ms = 1;
    homingDelayUs[i] = max(20, (int)((long)BASE_HOMING_DELAY_US * BASE_MICROSTEP / ms));
    moveDelayUs[i]   = max(10, (int)((long)BASE_MOVE_DELAY_US * BASE_MICROSTEP / ms));
    Serial.print("  "); Serial.print(AXIS_NAMES[i]);
    Serial.print(": 1/"); Serial.print(ms);
    Serial.print(" -> homingDelay="); Serial.print(homingDelayUs[i]);
    Serial.print("us, moveDelay="); Serial.print(moveDelayUs[i]);
    Serial.println("us");
  }

  Serial.println("-----------------------------------------");
  Serial.println("CAC LENH DI CHUYEN:");
  Serial.println("  home              Homing 3 truc");
  Serial.println("  homeX/homeY/homeZ Homing rieng 1 truc");
  Serial.println("  X50.5 Y30 Z10     Di chuyen toa do");
  Serial.println("CAC LENH DRIVER:");
  Serial.println("  status            Trang thai ca 3 driver");
  Serial.println("  current 800       Dat dong chay (mA)");
  Serial.println("  microstep 32      Dat vi buoc");
  Serial.println("  stealthchop       Che do chay em");
  Serial.println("  spreadcycle       Che do luc lon");
  Serial.println("=========================================");
}

// ==========================================
// LOOP - DOC LENH TU SERIAL
// ==========================================
void loop() {
  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim();

    if (inputStr.length() == 0) return;

    if (inputStr.equalsIgnoreCase("home")) {
      homeAllAxes();
    }
    else if (inputStr.equalsIgnoreCase("homeX")) {
      runHoming(0);
      printHomingDone(0);
    }
    else if (inputStr.equalsIgnoreCase("homeY")) {
      runHoming(1);
      printHomingDone(1);
    }
    else if (inputStr.equalsIgnoreCase("homeZ")) {
      runHoming(2);
      printHomingDone(2);
    }
    else if (inputStr.equalsIgnoreCase("sw")) {
      // Doc trang thai cong tac 10 lan, cach nhau 200ms
      Serial.println("Doc cong tac 10 lan (nhan thu cong tac trong luc doc):");
      for (int t = 0; t < 10; t++) {
        Serial.print("  ");
        for (int i = 0; i < NUM_AXES; i++) {
          Serial.print("SW_"); Serial.print(AXIS_NAMES[i]);
          Serial.print("="); Serial.print(digitalRead(SW_PINS[i]));
          Serial.print("  ");
        }
        Serial.println();
        delay(500);
      }
      Serial.println("Xong. (0=dang nhan, 1=nha)");
    }
    else if (inputStr.equalsIgnoreCase("status")) {
      printAllDriverStatus();
    }
    else if (inputStr.equalsIgnoreCase("stealthchop")) {
      for (int i = 0; i < NUM_AXES; i++) drivers[i]->en_spreadCycle(false);
      Serial.println("-> Da bat StealthChop cho ca 3 truc");
    }
    else if (inputStr.equalsIgnoreCase("spreadcycle")) {
      for (int i = 0; i < NUM_AXES; i++) drivers[i]->en_spreadCycle(true);
      Serial.println("-> Da bat SpreadCycle cho ca 3 truc");
    }
    else if (inputStr.startsWith("current ") || inputStr.startsWith("Current ")) {
      int mA = inputStr.substring(8).toInt();
      if (mA >= 100 && mA <= 2000) {
        for (int i = 0; i < NUM_AXES; i++) {
          drivers[i]->rms_current(mA);
          drivers[i]->ihold(scaleCurrentToIhold(mA / 2, mA));
        }
        Serial.print("-> Dong dien dat lai: "); Serial.print(mA); Serial.println(" mA");
      } else {
        Serial.println("!! Dong dien phai tu 100 den 2000 mA");
      }
    }
    else if (inputStr.startsWith("microstep ") || inputStr.startsWith("Microstep ")) {
      int ms = inputStr.substring(10).toInt();
      if (ms == 1 || ms == 2 || ms == 4 || ms == 8 || ms == 16 ||
          ms == 32 || ms == 64 || ms == 128 || ms == 256) {
        for (int i = 0; i < NUM_AXES; i++) drivers[i]->microsteps(ms);
        Serial.print("-> Microstep dat lai: 1/"); Serial.println(ms);
        Serial.println("!! LUU Y: Tinh lai STEPS_PER_MM trong code !!");
      } else {
        Serial.println("!! Gia tri hop le: 1, 2, 4, 8, 16, 32, 64, 128, 256");
      }
    }
    else if (inputStr.equalsIgnoreCase("test")) {
      testAllMotors();
    }
    else {
      parseAndMove(inputStr);
    }
  }
}

// ==========================================
// TEST TUNG DONG CO (200 buoc moi truc)
// ==========================================
void testAllMotors() {
  Serial.println("\n===== TEST TUNG DONG CO =====");
  for (int i = 0; i < NUM_AXES; i++) {
    Serial.print("Test truc ");
    Serial.print(AXIS_NAMES[i]);
    Serial.print(": EN=");
    // Dam bao EN = LOW
    digitalWrite(EN_PINS[i], LOW);
    Serial.print(digitalRead(EN_PINS[i]));
    Serial.print(" STEP_pin=");
    Serial.print(STEP_PINS[i]);
    Serial.print(" DIR_pin=");
    Serial.print(DIR_PINS[i]);
    Serial.println(" -> 200 buoc...");
    Serial.flush();

    digitalWrite(DIR_PINS[i], HIGH);
    for (int s = 0; s < 200; s++) {
      digitalWrite(STEP_PINS[i], HIGH);
      delayMicroseconds(800);
      digitalWrite(STEP_PINS[i], LOW);
      delayMicroseconds(800);
    }
    Serial.println("  -> Xong. Dong co co quay khong?");
    delay(500);
  }
  Serial.println("===== KET THUC TEST =====");
}

// ==========================================
// HOMING TAT CA 3 TRUC (DONG THOI)
// Dung polling nhanh, khong dung delay()
// ==========================================
void homeAllAxes() {
  Serial.println("\n=========================================");
  Serial.println("BAT DAU HOMING DONG THOI CA 3 TRUC...");
  Serial.println("=========================================");

  // In trang thai cong tac hien tai de kiem tra
  Serial.println("Trang thai cong tac truoc homing:");
  for (int i = 0; i < NUM_AXES; i++) {
    Serial.print("  SW_"); Serial.print(AXIS_NAMES[i]);
    Serial.print(" (pin "); Serial.print(SW_PINS[i]); Serial.print("): ");
    Serial.println(digitalRead(SW_PINS[i]) == LOW ? "!! DANG NHAN (LOW)" : "Nha (HIGH) OK");
  }
  Serial.flush();

  for (int i = 0; i < NUM_AXES; i++) {
    isHomed[i] = false;
  }

  // GIAI DOAN 0: Lui ra neu cong tac dang bi nhan
  bool needBackoff = false;
  for (int i = 0; i < NUM_AXES; i++) {
    if (digitalRead(SW_PINS[i]) == LOW) {
      delay(5); // debounce
      if (digitalRead(SW_PINS[i]) == LOW) {
        needBackoff = true;
        Serial.print("  !! SW_"); Serial.print(AXIS_NAMES[i]);
        Serial.println(" dang nhan -> lui ra truoc");
      }
    }
  }
  if (needBackoff) {
    Serial.println("[0] Lui ra khoi cong tac dang nhan...");
    Serial.flush();
    for (int i = 0; i < NUM_AXES; i++) {
      digitalWrite(DIR_PINS[i], HIGH); // Huong ra xa
    }
    delayMicroseconds(20);
    for (int s = 0; s < 2000; s++) { // Lui toi da ~2000 buoc
      for (int i = 0; i < NUM_AXES; i++) {
        if (digitalRead(SW_PINS[i]) == HIGH) continue; // Da nha
        digitalWrite(STEP_PINS[i], HIGH);
      }
      delayMicroseconds(200);
      for (int i = 0; i < NUM_AXES; i++) {
        digitalWrite(STEP_PINS[i], LOW);
      }
      delayMicroseconds(200);
    }
    delay(200); // Cho on dinh
    Serial.println("  -> Lui xong.");
  }

  // Dat huong ve phia cong tac
  for (int i = 0; i < NUM_AXES; i++) {
    digitalWrite(DIR_PINS[i], LOW);
  }
  delayMicroseconds(20); // DIR setup time

  // GIAI DOAN 1: Tim cong tac - moi truc co timer rieng (micros)
  Serial.println("[1/2] Tim cong tac hanh trinh...");
  Serial.flush();

  bool triggered[NUM_AXES]  = {false, false, false};
  bool stepHigh[NUM_AXES]   = {false, false, false};
  long stepCount[NUM_AXES]  = {0, 0, 0};
  // Debounce theo thoi gian (5ms) - khong phu thuoc tan so buoc
  unsigned long swLowSince[NUM_AXES] = {0, 0, 0}; // Thoi diem bat dau LOW
  bool swTiming[NUM_AXES] = {false, false, false};  // Dang dem thoi gian
  unsigned long nextStep[NUM_AXES];
  unsigned long now = micros();
  for (int i = 0; i < NUM_AXES; i++) nextStep[i] = now;

  bool allTriggered = false;
  while (!allTriggered) {
    now = micros();
    allTriggered = true;
    for (int i = 0; i < NUM_AXES; i++) {
      if (triggered[i]) continue;
      if (stepCount[i] >= maxHomingSteps[i]) {
        triggered[i] = true;
        Serial.print("!! LOI: Truc "); Serial.print(AXIS_NAMES[i]);
        Serial.print(" het "); Serial.print(maxHomingSteps[i]);
        Serial.println(" buoc, khong thay cong tac !!");
        Serial.flush();
        continue;
      }
      allTriggered = false;

      if ((long)(now - nextStep[i]) >= 0) {
        if (stepHigh[i]) {
          // Ha STEP
          digitalWrite(STEP_PINS[i], LOW);
          stepHigh[i] = false;
          nextStep[i] = now + homingDelayUs[i];
          stepCount[i]++;
          // Check switch - debounce theo thoi gian (5ms lien tuc LOW)
          if (digitalRead(SW_PINS[i]) == LOW) {
            if (!swTiming[i]) {
              swLowSince[i] = now;
              swTiming[i] = true;
            } else if ((now - swLowSince[i]) >= SW_DEBOUNCE_US) {
              triggered[i] = true;
              Serial.print("  Truc "); Serial.print(AXIS_NAMES[i]);
              Serial.print(" cham cong tac ("); Serial.print(stepCount[i]);
              Serial.println(" buoc)"); Serial.flush();
            }
          } else {
            swTiming[i] = false; // Reset neu nha
          }
        } else {
          // Len STEP
          digitalWrite(STEP_PINS[i], HIGH);
          stepHigh[i] = true;
          nextStep[i] = now + homingDelayUs[i];
        }
      }
    }
  }

  // Ha tat ca STEP pins truoc giai doan 2
  for (int i = 0; i < NUM_AXES; i++) {
    if (stepHigh[i]) { digitalWrite(STEP_PINS[i], LOW); stepHigh[i] = false; }
    if (stepCount[i] < maxHomingSteps[i]) digitalWrite(DIR_PINS[i], HIGH);
  }
  delayMicroseconds(20);

  // GIAI DOAN 2: Lui ra - moi truc co timer rieng
  Serial.println("[2/2] Lui ra khoi cong tac...");
  Serial.flush();

  bool backedOff[NUM_AXES] = {false, false, false};
  long backCount[NUM_AXES] = {0, 0, 0};
  now = micros();
  for (int i = 0; i < NUM_AXES; i++) nextStep[i] = now;

  bool allBackedOff = false;
  while (!allBackedOff) {
    now = micros();
    allBackedOff = true;
    for (int i = 0; i < NUM_AXES; i++) {
      if (backedOff[i]) continue;
      if (stepCount[i] >= maxHomingSteps[i] || backCount[i] >= backoffSteps[i]) {
        backedOff[i] = true;
        if (stepCount[i] < maxHomingSteps[i]) {
          currentPositionSteps[i] = 0;
          isHomed[i] = true;
        }
        continue;
      }
      allBackedOff = false;

      if ((long)(now - nextStep[i]) >= 0) {
        if (stepHigh[i]) {
          digitalWrite(STEP_PINS[i], LOW);
          stepHigh[i] = false;
          nextStep[i] = now + homingDelayUs[i];
          backCount[i]++;
        } else {
          digitalWrite(STEP_PINS[i], HIGH);
          stepHigh[i] = true;
          nextStep[i] = now + homingDelayUs[i];
        }
      }
    }
  }

  // Ket qua
  Serial.println("-----------------------------------------");
  bool allOk = true;
  for (int i = 0; i < NUM_AXES; i++) if (!isHomed[i]) allOk = false;
  if (allOk) {
    Serial.println("-> TAT CA 3 TRUC HOMING THANH CONG!");
  } else {
    Serial.println("!! CO TRUC HOMING THAT BAI.");
  }
  Serial.print("Vi tri: ");
  printCurrentPositions();
  Serial.println("NHAP TOA DO (VD: X50.5 Y30 Z10):");
  Serial.println("=========================================");
}

void printHomingDone(int axis) {
  Serial.print("Vi tri hien tai: ");
  printCurrentPositions();
  Serial.println("Nhap toa do tiep theo (hoac 'home'):");
}

// ==========================================
// HOMING MOT TRUC
// ==========================================
void runHoming(int axis) {
  Serial.print("[HOMING] Truc ");
  Serial.print(AXIS_NAMES[axis]);
  Serial.println(" - Dang tim cong tac hanh trinh...");
  Serial.flush();

  // Kiem tra cong tac co bi nhan san khong
  if (isSwitchPressed(axis)) {
    Serial.print("  !! Cong tac truc ");
    Serial.print(AXIS_NAMES[axis]);
    Serial.println(" dang bi nhan! Lui ra truoc...");
    Serial.flush();
    digitalWrite(DIR_PINS[axis], HIGH);
    delayMicroseconds(20);
    for (int i = 0; i < backoffSteps[axis] * 2; i++) {
      digitalWrite(STEP_PINS[axis], HIGH);
      delayMicroseconds(homingDelayUs[axis]);
      digitalWrite(STEP_PINS[axis], LOW);
      delayMicroseconds(homingDelayUs[axis]);
    }
    delay(200);
  }

  // Di chuyen ve phia cong tac
  Serial.print("  Dang buoc... EN=");
  Serial.print(digitalRead(EN_PINS[axis]));
  Serial.print(" SW=");
  Serial.print(digitalRead(SW_PINS[axis]));
  Serial.print(" delay=");
  Serial.print(homingDelayUs[axis]);
  Serial.println("us");
  Serial.flush();

  digitalWrite(DIR_PINS[axis], LOW);
  long stepCount = 0;

  while (!isSwitchPressed(axis) && stepCount < maxHomingSteps[axis]) {
    digitalWrite(STEP_PINS[axis], HIGH);
    delayMicroseconds(homingDelayUs[axis]);
    digitalWrite(STEP_PINS[axis], LOW);
    delayMicroseconds(homingDelayUs[axis]);
    stepCount++;
  }

  if (stepCount >= maxHomingSteps[axis]) {
    Serial.print("!! LOI: Truc ");
    Serial.print(AXIS_NAMES[axis]);
    Serial.println(" khong tim thay cong tac !!");
    isHomed[axis] = false;
    return;
  }

  // Lui lai 2mm de nha cong tac
  digitalWrite(DIR_PINS[axis], HIGH);
  delayMicroseconds(20);
  for (int i = 0; i < backoffSteps[axis]; i++) {
    digitalWrite(STEP_PINS[axis], HIGH);
    delayMicroseconds(homingDelayUs[axis]);
    digitalWrite(STEP_PINS[axis], LOW);
    delayMicroseconds(homingDelayUs[axis]);
  }

  currentPositionSteps[axis] = 0;
  isHomed[axis] = true;
  Serial.print("-> Truc ");
  Serial.print(AXIS_NAMES[axis]);
  Serial.println(" homing thanh cong! Reset ve 0.");
}

// ==========================================
// PHAN TICH LENH VA DI CHUYEN
// Dinh dang: X50.5 Y30 Z10
// Co the chi dinh 1, 2 hoac ca 3 truc
// ==========================================
void parseAndMove(String input) {
  float targetMm[NUM_AXES];
  bool hasTarget[NUM_AXES] = {false, false, false};

  // Mac dinh giu nguyen vi tri hien tai
  for (int i = 0; i < NUM_AXES; i++) {
    targetMm[i] = (float)currentPositionSteps[i] / STEPS_PER_MM[i];
  }

  int len = input.length();
  int idx = 0;

  while (idx < len) {
    char c = toupper(input.charAt(idx));
    int axisIdx = -1;

    if (c == 'X')      axisIdx = 0;
    else if (c == 'Y') axisIdx = 1;
    else if (c == 'Z') axisIdx = 2;

    if (axisIdx >= 0) {
      idx++;
      int start = idx;
      // Doc so theo sau ky tu truc
      while (idx < len && (input.charAt(idx) == '-' ||
                           input.charAt(idx) == '.' ||
                           isDigit(input.charAt(idx)))) {
        idx++;
      }
      if (start < idx) {
        targetMm[axisIdx] = input.substring(start, idx).toFloat();
        hasTarget[axisIdx] = true;
      }
    } else {
      idx++;
    }
  }

  // Kiem tra co truc nao duoc chi dinh khong
  bool anyTarget = false;
  for (int i = 0; i < NUM_AXES; i++) {
    if (hasTarget[i]) { anyTarget = true; break; }
  }

  if (!anyTarget) {
    Serial.println("!! Lenh khong hop le. VD: X50.5 Y30 Z10");
    return;
  }

  // In muc tieu
  Serial.print("\nLenh: Di chuyen den ");
  for (int i = 0; i < NUM_AXES; i++) {
    if (hasTarget[i]) {
      Serial.print(AXIS_NAMES[i]);
      Serial.print("=");
      Serial.print(targetMm[i], 2);
      Serial.print("mm ");
    }
  }
  Serial.println();

  // Tinh so buoc can di chuyen cho tung truc
  long stepsToMove[NUM_AXES];
  for (int i = 0; i < NUM_AXES; i++) {
    long targetSteps = (long)(targetMm[i] * STEPS_PER_MM[i]);
    stepsToMove[i] = targetSteps - currentPositionSteps[i];
  }

  // Di chuyen dong thoi cac truc
  moveMultiAxis(stepsToMove);

  Serial.print("-> Hoan thanh. Vi tri: ");
  printCurrentPositions();
  Serial.println("Nhap toa do tiep theo (hoac 'home'):");
}

// ==========================================
// DI CHUYEN DONG THOI NHIEU TRUC
// Su dung thuat toan Bresenham mo rong
// Dam bao cac truc den dich cung luc
// ==========================================
void moveMultiAxis(long stepsToMove[]) {
  long absSteps[NUM_AXES];
  int  dirMult[NUM_AXES];
  long maxSteps = 0;

  // Thiet lap huong va tinh so buoc tuyet doi
  for (int i = 0; i < NUM_AXES; i++) {
    if (stepsToMove[i] > 0) {
      digitalWrite(DIR_PINS[i], HIGH);  // Ra xa cong tac
      absSteps[i] = stepsToMove[i];
      dirMult[i] = 1;
    } else if (stepsToMove[i] < 0) {
      digitalWrite(DIR_PINS[i], LOW);   // Ve phia cong tac
      absSteps[i] = -stepsToMove[i];
      dirMult[i] = -1;
    } else {
      absSteps[i] = 0;
      dirMult[i] = 0;
    }
    if (absSteps[i] > maxSteps) {
      maxSteps = absSteps[i];
    }
  }

  if (maxSteps == 0) {
    Serial.println("(Da o dung vi tri, khong can di chuyen)");
    return;
  }

  // Tinh delay cho vong lap Bresenham dua tren truc co nhieu buoc nhat
  // Truc master quyet dinh toc do vong lap
  int loopDelay = BASE_MOVE_DELAY_US;
  for (int i = 0; i < NUM_AXES; i++) {
    if (absSteps[i] == maxSteps) {
      loopDelay = moveDelayUs[i];
      break;
    }
  }

  // Bien dem Bresenham cho tung truc
  long counter[NUM_AXES] = {0, 0, 0};
  bool stopped[NUM_AXES] = {false, false, false};

  for (long step = 0; step < maxSteps; step++) {
    // Kiem tra cong tac hanh trinh (nhanh, khong debounce day du)
    for (int i = 0; i < NUM_AXES; i++) {
      if (!stopped[i] && absSteps[i] > 0 && isSwitchPressedFast(i)) {
        stopped[i] = true;
        Serial.print("!! CANH BAO: Truc ");
        Serial.print(AXIS_NAMES[i]);
        Serial.println(" dung khan cap do cham cong tac !!");
      }
    }

    // Kiem tra tat ca truc con hoat dong da dung chua
    bool allDone = true;
    for (int i = 0; i < NUM_AXES; i++) {
      if (absSteps[i] > 0 && !stopped[i]) {
        allDone = false;
        break;
      }
    }
    if (allDone) break;

    // Tao xung STEP theo Bresenham
    bool stepped[NUM_AXES] = {false, false, false};
    bool anyStep = false;

    for (int i = 0; i < NUM_AXES; i++) {
      if (stopped[i] || absSteps[i] == 0) continue;

      counter[i] += absSteps[i];
      if (counter[i] >= maxSteps) {
        counter[i] -= maxSteps;
        digitalWrite(STEP_PINS[i], HIGH);
        stepped[i] = true;
        anyStep = true;
      }
    }

    if (anyStep) {
      delayMicroseconds(loopDelay);

      // Ha tat ca chan STEP va cap nhat vi tri
      for (int i = 0; i < NUM_AXES; i++) {
        if (stepped[i]) {
          digitalWrite(STEP_PINS[i], LOW);
          currentPositionSteps[i] += dirMult[i];
        }
      }

      delayMicroseconds(loopDelay);
    }
  }
}
