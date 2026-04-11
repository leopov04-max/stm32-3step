// ==========================================
// DIEU KHIEN STEPPER QUA TMC2208 UART
// STM32 Blue Pill + TMCStepper Library
// ==========================================
// UART dung de cau hinh driver (dong dien, microstep, StealthChop)
// STEP/DIR van dung de dieu khien chuyen dong
// ==========================================

#include <TMCStepper.h>

// ==========================================
// CAU HINH CHAN KET NOI (STM32 Blue Pill)
// ==========================================
#define DIR_PIN   PA4
#define STEP_PIN  PA5
#define EN_PIN    PA7
#define SW_PIN    PB0   // Cong tac hanh trinh (Normally Open)

// UART: dung USART2 (PA2=TX2, PA3=RX2)
// PA2 (TX) noi qua dien tro 1kOhm den PDN_UART
// PA3 (RX) noi truc tiep den PDN_UART
HardwareSerial Serial2(PA3, PA2);  // RX, TX
#define SERIAL_PORT Serial2
#define R_SENSE     0.11f   // Dien tro cam bien dong (Ohm) - gia tri chuan TMC2208

// ==========================================
// THONG SO DRIVER (thay doi qua UART)
// ==========================================
#define RUN_CURRENT_MA    800   // Dong dien chay (mA)
#define HOLD_CURRENT_MA   400   // Dong dien giu (mA)
#define MICROSTEPS        16    // Vi buoc: 1, 2, 4, 8, 16, 32, 64, 128, 256

// ==========================================
// THONG SO HIEU CHINH CO KHI
// ==========================================
// STEPS_PER_MM phu thuoc vao microstep va co cau
// VD: 200 step/vong * 16 microstep / 8mm pitch = 400 step/mm
// Hay dieu chinh lai theo thuc te cua ban
const float STEPS_PER_MM = 400.0;

// ==========================================
// THONG SO TOC DO
// ==========================================
const int HOMING_DELAY_US = 400;    // Toc do homing (us/nua_chu_ky)
const int MOVE_DELAY_US   = 200;    // Toc do di chuyen (us/nua_chu_ky)
const long MAX_HOMING_STEPS = 100000;
const int BACKOFF_STEPS = (int)(2.0 * STEPS_PER_MM); // Lui 2mm

// ==========================================
// BIEN TRANG THAI
// ==========================================
TMC2208Stepper driver(&SERIAL_PORT, R_SENSE);

long currentPositionSteps = 0;
bool isHomed = false;

// ==========================================
// HAM DEBOUNCE CONG TAC (50ms + 3 lan)
// ==========================================
bool isSwitchPressed() {
  if (digitalRead(SW_PIN) == HIGH) return false;
  delay(50);
  if (digitalRead(SW_PIN) == HIGH) return false;
  delay(20);
  if (digitalRead(SW_PIN) == HIGH) return false;
  return true;
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  // Cau hinh chan dieu khien
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(SW_PIN, INPUT_PULLUP);
  digitalWrite(EN_PIN, LOW); // Bat driver

  // ========== CAU HINH TMC2208 QUA UART ==========
  SERIAL_PORT.begin(115200);
  driver.begin();

  // Dong dien
  driver.rms_current(RUN_CURRENT_MA);
  driver.ihold(scaleCurrentToIhold(HOLD_CURRENT_MA));

  // Microstep
  driver.microsteps(MICROSTEPS);

  // StealthChop (chay em)
  driver.en_spreadCycle(false);   // false = StealthChop, true = SpreadCycle
  driver.pwm_autoscale(true);     // Tu dong dieu chinh PWM
  driver.pwm_autograd(true);      // Tu dong hoc PWM gradient

  // Xac nhan ket noi UART
  Serial.println("=========================================");
  Serial.println("  TMC2208 UART MOTOR CONTROLLER");
  Serial.println("=========================================");

  uint8_t result = driver.test_connection();
  if (result == 0) {
    Serial.println("[OK] TMC2208 UART ket noi thanh cong!");
    printDriverConfig();
  } else {
    Serial.print("[LOI] TMC2208 UART that bai! Ma loi: ");
    Serial.println(result);
    Serial.println("Kiem tra: day noi PDN_UART, dien tro 1kOhm, nguon VIO");
  }

  Serial.println("-----------------------------------------");
  Serial.println("CAC LENH:");
  Serial.println("  home              Homing ve goc");
  Serial.println("  50.5              Di chuyen den 50.5mm");
  Serial.println("  status            Doc trang thai driver");
  Serial.println("  current 800       Dat dong chay (mA)");
  Serial.println("  microstep 32      Dat vi buoc");
  Serial.println("  stealthchop       Bat che do chay em");
  Serial.println("  spreadcycle       Bat che do luc lon");
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
      runHoming();
    }
    else if (inputStr.equalsIgnoreCase("status")) {
      printDriverStatus();
    }
    else if (inputStr.equalsIgnoreCase("stealthchop")) {
      driver.en_spreadCycle(false);
      Serial.println("-> Da bat StealthChop (chay em)");
    }
    else if (inputStr.equalsIgnoreCase("spreadcycle")) {
      driver.en_spreadCycle(true);
      Serial.println("-> Da bat SpreadCycle (luc lon)");
    }
    else if (inputStr.startsWith("current ") || inputStr.startsWith("Current ")) {
      int mA = inputStr.substring(8).toInt();
      if (mA >= 100 && mA <= 2000) {
        driver.rms_current(mA);
        Serial.print("-> Dong dien dat lai: ");
        Serial.print(mA);
        Serial.println(" mA");
      } else {
        Serial.println("!! Dong dien phai tu 100 den 2000 mA");
      }
    }
    else if (inputStr.startsWith("microstep ") || inputStr.startsWith("Microstep ")) {
      int ms = inputStr.substring(10).toInt();
      if (ms == 1 || ms == 2 || ms == 4 || ms == 8 || ms == 16 ||
          ms == 32 || ms == 64 || ms == 128 || ms == 256) {
        // Quy doi lai STEPS_PER_MM khi thay doi microstep khong thuc hien o day
        // Vi no phu thuoc vao co cau co khi. Ban phai tu tinh lai.
        driver.microsteps(ms);
        Serial.print("-> Microstep dat lai: 1/");
        Serial.println(ms);
        Serial.println("!! LUU Y: Ban can tinh lai STEPS_PER_MM trong code !!");
      } else {
        Serial.println("!! Gia tri hop le: 1, 2, 4, 8, 16, 32, 64, 128, 256");
      }
    }
    else {
      // Xem nhu toa do mm
      float targetMm = inputStr.toFloat();
      long targetSteps = (long)(targetMm * STEPS_PER_MM);
      long stepsToMove = targetSteps - currentPositionSteps;

      Serial.print("\nLenh: Di chuyen den ");
      Serial.print(targetMm);
      Serial.println(" mm");

      moveSteps(stepsToMove);

      Serial.print("-> Vi tri hien tai: ");
      Serial.print((float)currentPositionSteps / STEPS_PER_MM, 2);
      Serial.println(" mm");
      Serial.println("Nhap toa do tiep theo (hoac 'home'):");
    }
  }
}

// ==========================================
// HOMING
// ==========================================
void runHoming() {
  Serial.println("Dang chay ve tim Goc (Homing)...");

  digitalWrite(DIR_PIN, LOW);
  long stepCount = 0;

  while (!isSwitchPressed() && stepCount < MAX_HOMING_STEPS) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(HOMING_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(HOMING_DELAY_US);
    stepCount++;
  }

  if (stepCount >= MAX_HOMING_STEPS) {
    Serial.println("!! LOI: Khong tim thay cong tac hanh trinh !!");
    return;
  }

  // Lui lai 2mm de nha cong tac
  digitalWrite(DIR_PIN, HIGH);
  for (int i = 0; i < BACKOFF_STEPS; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(HOMING_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(HOMING_DELAY_US);
  }

  currentPositionSteps = 0;
  isHomed = true;
  Serial.println("-> Homing thanh cong! Reset ve 0.");
  Serial.println("NHAP TOA DO (VD: 50.5):");
}

// ==========================================
// HAM DI CHUYEN (giu nguyen logic da kiem chung)
// ==========================================
void moveSteps(long steps) {
  if (steps == 0) {
    Serial.println("(Da o dung vi tri nay)");
    return;
  }

  int direction_multiplier = 1;
  if (steps > 0) {
    digitalWrite(DIR_PIN, HIGH);
  } else {
    digitalWrite(DIR_PIN, LOW);
    steps = -steps;
    direction_multiplier = -1;
  }

  for (long i = 0; i < steps; i++) {
    if (isSwitchPressed()) {
      Serial.println("!! DUNG KHAN CAP: Cham cong tac hanh trinh !!");
      break;
    }

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(MOVE_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(MOVE_DELAY_US);

    currentPositionSteps += direction_multiplier;
  }
}

// ==========================================
// IN CAU HINH DRIVER
// ==========================================
void printDriverConfig() {
  Serial.println("----- CAU HINH DRIVER -----");
  Serial.print("  Dong RMS  : "); Serial.print(driver.rms_current()); Serial.println(" mA");
  Serial.print("  Microstep : 1/"); Serial.println(driver.microsteps());
  Serial.print("  Che do    : ");
  Serial.println(driver.en_spreadCycle() ? "SpreadCycle (luc lon)" : "StealthChop (chay em)");
  Serial.print("  STEPS/MM  : "); Serial.println(STEPS_PER_MM);
}

// ==========================================
// IN TRANG THAI DRIVER (DIAGNOSTICS)
// ==========================================
void printDriverStatus() {
  Serial.println("\n===== TRANG THAI TMC2208 =====");

  // Doc thanh ghi DRV_STATUS
  uint32_t drv_status = driver.DRV_STATUS();

  Serial.print("  Dong RMS     : "); Serial.print(driver.rms_current()); Serial.println(" mA");
  Serial.print("  Microstep    : 1/"); Serial.println(driver.microsteps());
  Serial.print("  Che do       : ");
  Serial.println(driver.en_spreadCycle() ? "SpreadCycle" : "StealthChop");

  // Canh bao nhiet do
  if (driver.otpw()) {
    Serial.println("  !! CANH BAO: Nhiet do driver cao !!");
  }
  if (driver.ot()) {
    Serial.println("  !! LOI: Qua nhiet - driver da tat !!");
  }

  // Trang thai chung
  Serial.print("  Open load A  : ");
  Serial.println(driver.ola() ? "CO" : "Khong");
  Serial.print("  Open load B  : ");
  Serial.println(driver.olb() ? "CO" : "Khong");
  Serial.print("  Short GND A  : ");
  Serial.println(driver.s2ga() ? "!! CO !!" : "Khong");
  Serial.print("  Short GND B  : ");
  Serial.println(driver.s2gb() ? "!! CO !!" : "Khong");

  Serial.print("  Vi tri hien tai: ");
  Serial.print((float)currentPositionSteps / STEPS_PER_MM, 2);
  Serial.println(" mm");
  Serial.println("==============================");
}

// ==========================================
// TINH IHOLD TU mA (noi bo)
// ==========================================
uint8_t scaleCurrentToIhold(int holdMa) {
  // IHOLD la gia tri 0-31, ty le voi dong dien
  // Tinh gan dung: holdMa / runCurrentMa * 31
  float ratio = (float)holdMa / (float)RUN_CURRENT_MA;
  if (ratio > 1.0) ratio = 1.0;
  if (ratio < 0.0) ratio = 0.0;
  return (uint8_t)(ratio * 31.0);
}
