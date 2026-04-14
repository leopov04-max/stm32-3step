// ==========================================
// DIEU KHIEN 3 TRUC STEPPER MOTOR (3-DOF)
// STM32F103C8T6 Blue Pill + 3x TMC2240 MKS V1.0
// Giao tiep SPI - STEP/DIR dieu khien chuyen dong
// ==========================================
// - SPI cau hinh driver: dong dien, microstep, StealthChop
// - STEP/DIR dieu khien chuyen dong
// - Bresenham noi suy dong thoi 3 truc
// ==========================================
// ARDUINO IDE SETUP:
//   Board: "Generic STM32F1 series"
//   Board part number: "BluePill F103C8"
//   USB support: "CDC (generic 'Serial' supersede U(S)ART)"
//   Upload: "STM32CubeProgrammer (SWD)" hoac "Serial"
//   LUU Y: Neu USB khong nhan, han them R 1.5kOhm tu PA12 len 3.3V
//          hoac thay R10 tren board thanh 1.5kOhm
// ==========================================

#include <SPI.h>

#define NUM_AXES 3

// ==========================================
// CAU HINH CHAN KET NOI (STM32F103C8T6 Blue Pill)
// ==========================================

// SPI1 Bus (chung cho 3 driver)
// PA5 = SCK, PA6 = MISO (SDO), PA7 = MOSI (SDI)

// Chip Select (moi driver 1 chan rieng, Active LOW)
#define CS_X_PIN   PA4
#define CS_Y_PIN   PB0
#define CS_Z_PIN   PB1

// --- Truc X ---
#define X_STEP_PIN  PA0
#define X_DIR_PIN   PA1
#define X_EN_PIN    PA2
#define X_SW_PIN    PB13   // Cong tac hanh trinh X (NO)

// --- Truc Y ---
#define Y_STEP_PIN  PA3
#define Y_DIR_PIN   PA8
#define Y_EN_PIN    PA9
#define Y_SW_PIN    PB14   // Cong tac hanh trinh Y (NO)

// --- Truc Z ---
#define Z_STEP_PIN  PA10
#define Z_DIR_PIN   PB10
#define Z_EN_PIN    PB12
#define Z_SW_PIN    PB15   // Cong tac hanh trinh Z (NO)

// LED on-board (PC13, Active LOW)
#define LED_PIN     PC13

// ==========================================
// TMC2240 REGISTER MAP
// ==========================================
#define TMC_GCONF         0x00   // General configuration
#define TMC_GSTAT         0x01   // Global status flags
#define TMC_IOIN          0x04   // Input/output pin status + VERSION
#define TMC_DRV_CONF      0x0A   // Driver configuration
#define TMC_GLOBAL_SCALER 0x0B   // Global current scaler
#define TMC_IHOLD_IRUN    0x10   // Hold/run current
#define TMC_TPOWERDOWN    0x11   // Power-down delay
#define TMC_TSTEP         0x12   // Actual step time
#define TMC_TCOOLTHRS     0x14   // CoolStep lower threshold
#define TMC_THIGH         0x15   // High velocity threshold
#define TMC_CHOPCONF      0x6C   // Chopper configuration
#define TMC_COOLCONF      0x6D   // CoolStep configuration
#define TMC_DRV_STATUS    0x6F   // Driver status
#define TMC_PWMCONF       0x70   // StealthChop PWM configuration

// ==========================================
// TMC2240 THONG SO
// ==========================================
// R_SENSE: Dien tro cam bien dong tren module MKS TMC2240 V1.0
// KIEM TRA LAI GIA TRI NAY TREN BOARD CUA BAN!
// Gia tri thuong gap: 0.075, 0.11, 0.05 Ohm
#define R_SENSE       0.075f   // Ohm - THAY DOI NEU KHAC
#define V_FS          0.325f   // Full-scale voltage (V) - tu datasheet TMC2240

#define RUN_CURRENT_MA    800    // Dong dien chay (mA)
#define HOLD_CURRENT_MA   400    // Dong dien giu (mA)

// Vi buoc rieng tung truc
const int MICROSTEPS_AXIS[NUM_AXES] = {16, 16, 16};  // X, Y, Z

// SPI Settings: 1MHz, MSB first, Mode 3 (CPOL=1, CPHA=1)
// Giam toc do SPI de debug - tang len 4MHz sau khi ket noi OK
SPISettings tmcSpiSettings(1000000, MSBFIRST, SPI_MODE3);

// Mang chan de truy cap bang chi so truc
const int CS_PINS[NUM_AXES]   = {CS_X_PIN,  CS_Y_PIN,  CS_Z_PIN};
const int STEP_PINS[NUM_AXES] = {X_STEP_PIN, Y_STEP_PIN, Z_STEP_PIN};
const int DIR_PINS[NUM_AXES]  = {X_DIR_PIN,  Y_DIR_PIN,  Z_DIR_PIN};
const int EN_PINS[NUM_AXES]   = {X_EN_PIN,   Y_EN_PIN,   Z_EN_PIN};
const int SW_PINS[NUM_AXES]   = {X_SW_PIN,   Y_SW_PIN,   Z_SW_PIN};
const char AXIS_NAMES[NUM_AXES] = {'X', 'Y', 'Z'};

// ==========================================
// THONG SO HIEU CHINH CO KHI
// ==========================================
#define MOTOR_STEPS_PER_REV  200    // Full step/vong (1.8 do)
#define MM_PER_REV           8.0f   // Buoc vit me (mm/vong) - DIEU CHINH THEO THUC TE

float STEPS_PER_MM[NUM_AXES] = {400.0, 400.0, 400.0};  // Se tinh lai trong setup

// ==========================================
// BIEN TRANG THAI
// ==========================================
long currentPositionSteps[NUM_AXES] = {0, 0, 0};
bool isHomed[NUM_AXES] = {false, false, false};

// TAM VO HIEU HOA TRUC - dat false de bo qua truc do
bool axisEnabled[NUM_AXES] = {true, true, true};

// Thong so homing (se tinh lai tu microstep thuc te)
long maxHomingSteps[NUM_AXES] = {50000, 50000, 50000};
int  backoffSteps[NUM_AXES]   = {400, 400, 400};

// Delay TRUC TIEP (microseconds) cho tung truc
// Lon hon = cham hon, Nho hon = nhanh hon
int homingDelayUs[NUM_AXES] = {600, 600, 600};
int moveDelayUs[NUM_AXES]   = {600, 600, 650};

// Debounce cong tac hanh trinh
#define SW_DEBOUNCE_US  5000  // 5ms

// SPI status byte tu lan transfer cuoi
uint8_t tmcStatus[NUM_AXES] = {0, 0, 0};

// ==========================================
// TMC2240 SPI - DOC/GHI THANH GHI
// ==========================================

// Ghi 1 thanh ghi 32-bit qua SPI
void tmcWrite(int axis, uint8_t reg, uint32_t data) {
  SPI.beginTransaction(tmcSpiSettings);
  digitalWrite(CS_PINS[axis], LOW);
  delayMicroseconds(100);  // CS setup time (TMC2240 can toi thieu 100ns)

  tmcStatus[axis] = SPI.transfer(reg | 0x80);  // Bit 7 = 1: Write
  SPI.transfer((data >> 24) & 0xFF);
  SPI.transfer((data >> 16) & 0xFF);
  SPI.transfer((data >> 8) & 0xFF);
  SPI.transfer(data & 0xFF);

  delayMicroseconds(100);  // CS hold time
  digitalWrite(CS_PINS[axis], HIGH);
  SPI.endTransaction();
  delayMicroseconds(100);  // CS high time giua cac transfer
}

// Doc 1 thanh ghi 32-bit qua SPI
// TMC2240 tra du lieu tu lan doc TRUOC -> can doc 2 lan
uint32_t tmcRead(int axis, uint8_t reg) {
  uint32_t data = 0;

  // Lan 1: Gui dia chi doc (du lieu tra ve la cua lan doc truoc - bo qua)
  SPI.beginTransaction(tmcSpiSettings);
  digitalWrite(CS_PINS[axis], LOW);
  delayMicroseconds(100);
  SPI.transfer(reg & 0x7F);  // Bit 7 = 0: Read
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  delayMicroseconds(100);
  digitalWrite(CS_PINS[axis], HIGH);
  SPI.endTransaction();

  delayMicroseconds(100);  // CS high time giua 2 lan transfer

  // Lan 2: Du lieu thuc su cua thanh ghi duoc tra ve
  SPI.beginTransaction(tmcSpiSettings);
  digitalWrite(CS_PINS[axis], LOW);
  delayMicroseconds(100);
  tmcStatus[axis] = SPI.transfer(reg & 0x7F);
  data  = (uint32_t)SPI.transfer(0x00) << 24;
  data |= (uint32_t)SPI.transfer(0x00) << 16;
  data |= (uint32_t)SPI.transfer(0x00) << 8;
  data |= (uint32_t)SPI.transfer(0x00);
  delayMicroseconds(100);
  digitalWrite(CS_PINS[axis], HIGH);
  SPI.endTransaction();

  return data;
}

// Dong bo SPI (gui dummy transfer sau khi khoi dong)
void tmcSync(int axis) {
  SPI.beginTransaction(tmcSpiSettings);
  digitalWrite(CS_PINS[axis], LOW);
  delayMicroseconds(100);
  for (int i = 0; i < 5; i++) SPI.transfer(0x00);
  delayMicroseconds(100);
  digitalWrite(CS_PINS[axis], HIGH);
  SPI.endTransaction();
  delayMicroseconds(100);
}

// ==========================================
// TMC2240 - TINH TOAN DONG DIEN
// ==========================================
// Cong thuc: I_RMS = (GLOBALSCALER/256) * (CS+1)/32 * V_FS / (sqrt(2) * R_SENSE)
// Voi GLOBALSCALER = 0 (= 256, full scale):
// CS = round(I_mA/1000 * sqrt(2) * R_SENSE * 32 / V_FS) - 1

uint8_t currentToCS(uint16_t current_mA) {
  float cs_f = (float)current_mA / 1000.0f * 1.41421f * R_SENSE * 32.0f / V_FS - 1.0f;
  int cs = (int)(cs_f + 0.5f);
  if (cs < 0) cs = 0;
  if (cs > 31) cs = 31;
  return (uint8_t)cs;
}

// Tinh dong dien thuc te tu gia tri CS (de hien thi)
uint16_t csToCurrentMA(uint8_t cs) {
  float i_rms = ((float)(cs + 1) / 32.0f) * V_FS / (1.41421f * R_SENSE);
  return (uint16_t)(i_rms * 1000.0f);
}

// ==========================================
// TMC2240 - CHUYEN DOI MICROSTEP <-> MRES
// ==========================================

uint8_t microstepsToMRES(int ms) {
  switch (ms) {
    case 256: return 0;
    case 128: return 1;
    case 64:  return 2;
    case 32:  return 3;
    case 16:  return 4;
    case 8:   return 5;
    case 4:   return 6;
    case 2:   return 7;
    case 1:   return 8;
    default:  return 4;  // Mac dinh 16 microsteps
  }
}

int mresToMicrosteps(uint8_t mres) {
  if (mres > 8) return 1;
  return 256 >> mres;
}

// ==========================================
// TMC2240 - DAT DONG DIEN
// ==========================================

void tmcSetCurrent(int axis, uint16_t runMA, uint16_t holdMA) {
  uint8_t irun = currentToCS(runMA);
  uint8_t ihold = currentToCS(holdMA);
  uint8_t iholddelay = 6;  // Delay tu IRUN -> IHOLD
  uint8_t irundelay = 4;   // Delay khi bat dau chay

  uint32_t val = (uint32_t)ihold
               | ((uint32_t)irun << 8)
               | ((uint32_t)iholddelay << 16)
               | ((uint32_t)irundelay << 20);

  tmcWrite(axis, TMC_IHOLD_IRUN, val);
}

// ==========================================
// TMC2240 - DAT MICROSTEP
// ==========================================

void tmcSetMicrosteps(int axis, int ms) {
  uint8_t mres = microstepsToMRES(ms);

  // Doc CHOPCONF hien tai, chi thay doi MRES va INTPOL
  uint32_t chopconf = tmcRead(axis, TMC_CHOPCONF);
  chopconf &= ~(0x0FUL << 24);            // Xoa MRES [27:24]
  chopconf |= ((uint32_t)mres << 24);     // Dat MRES moi
  chopconf |= (1UL << 28);                // INTPOL = 1 (noi suy 256)
  tmcWrite(axis, TMC_CHOPCONF, chopconf);
}

int tmcGetMicrosteps(int axis) {
  uint32_t chopconf = tmcRead(axis, TMC_CHOPCONF);
  uint8_t mres = (chopconf >> 24) & 0x0F;
  return mresToMicrosteps(mres);
}

// ==========================================
// TMC2240 - STEALTHCHOP / SPREADCYCLE
// ==========================================

void tmcEnableStealthChop(int axis) {
  // GCONF: en_pwm_mode = 1 (bit 2)
  uint32_t gconf = tmcRead(axis, TMC_GCONF);
  gconf |= (1UL << 2);
  tmcWrite(axis, TMC_GCONF, gconf);

  // PWMCONF cho StealthChop
  uint32_t pwmconf = 36UL              // PWM_OFS = 36
                    | (0UL << 8)        // PWM_GRAD = 0
                    | (0UL << 16)       // PWM_FREQ = 0 (1/1024 fCLK)
                    | (1UL << 18)       // pwm_autoscale = 1
                    | (1UL << 19)       // pwm_autograd = 1
                    | (4UL << 20)       // pwm_reg = 4
                    | (12UL << 24);     // pwm_lim = 12
  tmcWrite(axis, TMC_PWMCONF, pwmconf);
}

void tmcEnableSpreadCycle(int axis) {
  // GCONF: en_pwm_mode = 0 (bit 2)
  uint32_t gconf = tmcRead(axis, TMC_GCONF);
  gconf &= ~(1UL << 2);
  tmcWrite(axis, TMC_GCONF, gconf);
}

// ==========================================
// TMC2240 - KIEM TRA KET NOI
// ==========================================

bool tmcTestConnection(int axis) {
  // Doc IOIN - truong VERSION tai bits [31:24]
  // TMC2240 VERSION = 0x40
  // Neu doc duoc 0x00 hoac 0xFF -> loi ket noi
  uint32_t ioin = tmcRead(axis, TMC_IOIN);
  uint8_t version = (ioin >> 24) & 0xFF;
  return (version != 0x00 && version != 0xFF);
}

// ==========================================
// SPI RESET - Tat SPI, toggle GPIO, bat lai
// Giup dong bo SPI state machine cua TMC2240
// (Bit-bang warmup truoc khi dung SPI hardware)
// ==========================================

void spiReset() {
  Serial.println("SPI reset: end -> GPIO warmup -> begin...");
  
  // Tat SPI hardware
  SPI.end();
  delay(10);
  
  // Cau hinh SPI pins thanh GPIO
  pinMode(PA5, OUTPUT);   // SCK
  pinMode(PA7, OUTPUT);   // MOSI
  pinMode(PA6, INPUT);    // MISO
  
  // Mode 3 idle: SCK = HIGH, MOSI = LOW
  digitalWrite(PA5, HIGH);
  digitalWrite(PA7, LOW);
  delay(1);
  
  // Toggle SCK voi CS LOW de reset TMC2240 SPI state machine
  for (int c = 0; c < NUM_AXES; c++) {
    if (!axisEnabled[c]) continue;
    digitalWrite(CS_PINS[c], LOW);
    delayMicroseconds(100);
    // Gui 48 clock cycles (> 40 bit = 1 frame) de flush
    for (int i = 0; i < 48; i++) {
      digitalWrite(PA5, LOW);
      delayMicroseconds(5);
      digitalWrite(PA5, HIGH);
      delayMicroseconds(5);
    }
    delayMicroseconds(100);
    digitalWrite(CS_PINS[c], HIGH);
    delayMicroseconds(200);
  }
  
  // Khoi dong lai SPI hardware
  SPI.begin();
  delay(50);
  
  // Khoi tao lai CS pins (SPI.begin co the chiem PA4 NSS)
  for (int c = 0; c < NUM_AXES; c++) {
    pinMode(CS_PINS[c], OUTPUT);
    digitalWrite(CS_PINS[c], HIGH);
  }
  delay(10);
  
  Serial.println("SPI reset xong.");
}

// ==========================================
// TMC2240 - KHOI TAO DRIVER
// ==========================================

void tmcInit(int axis) {
  int ms = MICROSTEPS_AXIS[axis];

  // Dam bao EN=HIGH (driver tat) trong luc init
  digitalWrite(EN_PINS[axis], HIGH);
  delay(10);

  // Dong bo SPI - gui nhieu dummy bytes de reset state machine
  tmcSync(axis);
  delay(50);
  tmcSync(axis);
  delay(50);

  // Xoa co loi GSTAT
  tmcWrite(axis, TMC_GSTAT, 0x07);
  delay(10);

  // CHOPCONF: Thong so chopper + microstep
  // QUAN TRONG: ghi CHOPCONF voi TOFF > 0 de bat chopper
  uint32_t chopconf = 3UL                // TOFF = 3 (bat driver output)
                    | (4UL << 4)          // HSTRT = 4
                    | (1UL << 7)          // HEND = 1
                    | (2UL << 15)         // TBL = 2 (36 clock blank)
                    | ((uint32_t)microstepsToMRES(ms) << 24)  // MRES
                    | (1UL << 28);        // INTPOL = 1 (noi suy 256)
  tmcWrite(axis, TMC_CHOPCONF, chopconf);
  delay(10);

  // Xac nhan CHOPCONF da ghi thanh cong
  uint32_t readback = tmcRead(axis, TMC_CHOPCONF);
  Serial.print("    CHOPCONF ghi: 0x");
  for (int b = 28; b >= 0; b -= 4) Serial.print((chopconf >> b) & 0x0F, HEX);
  Serial.print(" doc lai: 0x");
  for (int b = 28; b >= 0; b -= 4) Serial.print((readback >> b) & 0x0F, HEX);
  if ((readback & 0x0F) == 3) Serial.println(" [OK - TOFF=3]");
  else Serial.println(" [LOI - TOFF != 3, ghi that bai!]");

  // DRV_CONF
  tmcWrite(axis, TMC_DRV_CONF, 0x00000000);
  delay(10);

  // GLOBAL_SCALER = 0 (= 256, full scale)
  tmcWrite(axis, TMC_GLOBAL_SCALER, 0);
  delay(10);

  // Dat dong dien
  tmcSetCurrent(axis, RUN_CURRENT_MA, HOLD_CURRENT_MA);
  delay(10);

  // Xac nhan IHOLD_IRUN
  uint32_t ihold_irun_rb = tmcRead(axis, TMC_IHOLD_IRUN);
  Serial.print("    IHOLD_IRUN doc lai: 0x");
  for (int b = 28; b >= 0; b -= 4) Serial.print((ihold_irun_rb >> b) & 0x0F, HEX);
  Serial.println();

  // GCONF: StealthChop + multistep filter
  uint32_t gconf = (1UL << 2)   // en_pwm_mode (StealthChop)
                 | (1UL << 3);  // multistep_filt
  tmcWrite(axis, TMC_GCONF, gconf);
  delay(10);

  // PWMCONF cho StealthChop
  uint32_t pwmconf = 36UL              // PWM_OFS
                    | (0UL << 8)        // PWM_GRAD
                    | (0UL << 16)       // PWM_FREQ
                    | (1UL << 18)       // pwm_autoscale
                    | (1UL << 19)       // pwm_autograd
                    | (4UL << 20)       // pwm_reg
                    | (12UL << 24);     // pwm_lim
  tmcWrite(axis, TMC_PWMCONF, pwmconf);
  delay(10);

  // TPOWERDOWN
  tmcWrite(axis, TMC_TPOWERDOWN, 10);
  delay(10);
}

// ==========================================
// TMC2240 - DOC TRANG THAI DRIVER
// ==========================================

uint32_t tmcGetDrvStatus(int axis) {
  return tmcRead(axis, TMC_DRV_STATUS);
}

void printDriverInfo(int axis) {
  Serial.print("  Driver "); Serial.print(AXIS_NAMES[axis]); Serial.print(": ");

  bool connected = tmcTestConnection(axis);
  if (connected) {
    int ms = tmcGetMicrosteps(axis);
    uint32_t ihold_irun = tmcRead(axis, TMC_IHOLD_IRUN);
    uint8_t irun = (ihold_irun >> 8) & 0x1F;
    uint8_t ihold = ihold_irun & 0x1F;

    Serial.print("[OK] ");
    Serial.print(csToCurrentMA(irun));
    Serial.print("mA run, ");
    Serial.print(csToCurrentMA(ihold));
    Serial.print("mA hold, 1/");
    Serial.println(ms);
  } else {
    uint32_t ioin = tmcRead(axis, TMC_IOIN);
    uint8_t version = (ioin >> 24) & 0xFF;
    Serial.print("[LOI] SPI KHONG PHAN HOI! VERSION=0x");
    Serial.println(version, HEX);
    Serial.println("    Kiem tra: day SPI, chan CS, nguon VCC_IO 3.3V");
  }
}

// ==========================================
// HAM DEBOUNCE CONG TAC (50ms + 3 lan)
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
// IN VI TRI HIEN TAI
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
// IN TRANG THAI TAT CA 3 DRIVER
// ==========================================
void printAllDriverStatus() {
  Serial.println("\n===== TRANG THAI DRIVER TMC2240 =====");
  for (int i = 0; i < NUM_AXES; i++) {
    Serial.print("--- Truc ");
    Serial.print(AXIS_NAMES[i]);
    if (!axisEnabled[i]) {
      Serial.println(" --- [TAM TAT]");
      continue;
    }
    Serial.println(" ---");

    uint32_t drvStatus = tmcGetDrvStatus(i);

    // Dong dien
    uint32_t ihold_irun = tmcRead(i, TMC_IHOLD_IRUN);
    uint8_t irun = (ihold_irun >> 8) & 0x1F;
    uint8_t ihold = ihold_irun & 0x1F;
    Serial.print("  Dong RMS  : Run=");
    Serial.print(csToCurrentMA(irun));
    Serial.print("mA, Hold=");
    Serial.print(csToCurrentMA(ihold));
    Serial.println("mA");

    // Microstep
    Serial.print("  Microstep : 1/");
    Serial.println(tmcGetMicrosteps(i));

    // Che do
    uint32_t gconf = tmcRead(i, TMC_GCONF);
    Serial.print("  Che do    : ");
    Serial.println((gconf & (1UL << 2)) ? "StealthChop (chay em)" : "SpreadCycle (luc lon)");

    // Canh bao nhiet
    if (drvStatus & (1UL << 26)) Serial.println("  !! CANH BAO: Nhiet do driver cao (OTPW) !!");
    if (drvStatus & (1UL << 25)) Serial.println("  !! LOI: Qua nhiet - driver tat (OT) !!");

    // Open load
    Serial.print("  Open load : A=");
    Serial.print((drvStatus & (1UL << 29)) ? "CO" : "KO");
    Serial.print("  B=");
    Serial.println((drvStatus & (1UL << 30)) ? "CO" : "KO");

    // Short to GND
    Serial.print("  Short GND : A=");
    Serial.print((drvStatus & (1UL << 27)) ? "!! CO !!" : "KO");
    Serial.print("  B=");
    Serial.println((drvStatus & (1UL << 28)) ? "!! CO !!" : "KO");

    // Short to VS
    Serial.print("  Short VS  : A=");
    Serial.print((drvStatus & (1UL << 12)) ? "!! CO !!" : "KO");
    Serial.print("  B=");
    Serial.println((drvStatus & (1UL << 13)) ? "!! CO !!" : "KO");

    // StealthChop active
    Serial.print("  Stealth   : ");
    Serial.println((drvStatus & (1UL << 14)) ? "Dang hoat dong" : "Khong");

    // Standstill
    Serial.print("  Dung im   : ");
    Serial.println((drvStatus & (1UL << 31)) ? "Co" : "Khong");

    // SPI status byte
    Serial.print("  SPI Status: 0x");
    Serial.println(tmcStatus[i], HEX);
  }
  Serial.print("\nVi tri hien tai: ");
  printCurrentPositions();
  Serial.println("========================================");
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  // USB Serial
  Serial.begin(115200);
  unsigned long startMs = millis();
  while (!Serial && millis() - startMs < 3000) delay(10);
  delay(500);

  // LED on-board
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Tat LED (active LOW)

  Serial.println("=========================================");
  Serial.println("  3-DOF + TMC2240 SPI - Blue Pill F103C8");
  Serial.println("=========================================");

  // Cau hinh chan STEP/DIR/EN/SW truoc
  // QUAN TRONG: EN = HIGH (tat driver) TRUOC khi init SPI!
  // TMC2240 can duoc cau hinh qua SPI truoc khi bat driver
  for (int i = 0; i < NUM_AXES; i++) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(EN_PINS[i], OUTPUT);
    digitalWrite(EN_PINS[i], HIGH);  // TAT driver truoc (disable)
    pinMode(SW_PINS[i], INPUT_PULLUP);
  }

  // Khoi tao SPI1 TRUOC - de tranh xung dot voi PA4 (SPI1_NSS)
  SPI.begin();
  delay(100);

  // Cau hinh CS pins SAU SPI.begin()
  // QUAN TRONG: SPI.begin() co the chiem PA4 (NSS), nen phai
  // khoi tao lai CS pins sau do
  for (int i = 0; i < NUM_AXES; i++) {
    pinMode(CS_PINS[i], OUTPUT);
    digitalWrite(CS_PINS[i], HIGH);
  }
  delay(10);

  // In trang thai chan de debug
  Serial.println("--- Kiem tra chan SPI ---");
  Serial.print("  SCK  (PA5) : "); Serial.println(digitalRead(PA5));
  Serial.print("  MISO (PA6) : "); Serial.println(digitalRead(PA6));
  Serial.print("  MOSI (PA7) : "); Serial.println(digitalRead(PA7));
  for (int i = 0; i < NUM_AXES; i++) {
    Serial.print("  CS_"); Serial.print(AXIS_NAMES[i]);
    Serial.print("  (pin "); Serial.print(CS_PINS[i]);
    Serial.print(") : "); Serial.println(digitalRead(CS_PINS[i]));
  }
  for (int i = 0; i < NUM_AXES; i++) {
    Serial.print("  EN_"); Serial.print(AXIS_NAMES[i]);
    Serial.print("  (pin "); Serial.print(EN_PINS[i]);
    Serial.print(") : "); Serial.print(digitalRead(EN_PINS[i]));
    Serial.println(digitalRead(EN_PINS[i]) == LOW ? " [BAT]" : " [TAT]");
  }

  // ========== THU SPI RAW TRUOC KHI INIT DRIVER ==========
  Serial.println("Thu SPI raw (doc IOIN = reg 0x04)...");
  for (int i = 0; i < NUM_AXES; i++) {
    if (!axisEnabled[i]) continue;
    // Thu SPI transfer raw
    SPI.beginTransaction(tmcSpiSettings);
    digitalWrite(CS_PINS[i], LOW);
    delayMicroseconds(10);
    uint8_t b0 = SPI.transfer(0x04);  // Read IOIN
    uint8_t b1 = SPI.transfer(0x00);
    uint8_t b2 = SPI.transfer(0x00);
    uint8_t b3 = SPI.transfer(0x00);
    uint8_t b4 = SPI.transfer(0x00);
    delayMicroseconds(10);
    digitalWrite(CS_PINS[i], HIGH);
    SPI.endTransaction();
    Serial.print("  "); Serial.print(AXIS_NAMES[i]);
    Serial.print(": status=0x"); Serial.print(b0, HEX);
    Serial.print(" data=0x"); Serial.print(b1, HEX);
    Serial.print(" "); Serial.print(b2, HEX);
    Serial.print(" "); Serial.print(b3, HEX);
    Serial.print(" "); Serial.println(b4, HEX);
    if (b0 == 0xFF && b1 == 0xFF) {
      Serial.println("    -> TAT CA 0xFF: MISO bi keo HIGH hoac driver chua cap dien!");
      Serial.println("    -> Kiem tra: VCC_IO co 3.3V? VM co 12-24V? Day SDO noi dung?");
    } else if (b0 == 0x00 && b1 == 0x00) {
      Serial.println("    -> TAT CA 0x00: MISO bi keo LOW hoac CS khong hoat dong");
    }
    delay(10);
  }

  // ========== SPI RESET TRUOC KHI INIT ==========
  spiReset();

  // Kiem tra SPI sau reset
  Serial.println("Kiem tra SPI sau reset...");
  for (int i = 0; i < NUM_AXES; i++) {
    if (!axisEnabled[i]) continue;
    uint32_t ioin = tmcRead(i, TMC_IOIN);
    uint8_t ver = (ioin >> 24) & 0xFF;
    Serial.print("  "); Serial.print(AXIS_NAMES[i]);
    Serial.print(": VERSION=0x"); Serial.print(ver, HEX);
    Serial.println(ver == 0x40 ? " [OK]" : " [CHUA KET NOI]");
  }

  // ========== KHOI TAO DRIVER TMC2240 QUA SPI ==========
  Serial.println("Dang khoi tao driver TMC2240...");
  Serial.println("(EN=HIGH, driver tat trong luc cau hinh SPI)");

  for (int i = 0; i < NUM_AXES; i++) {
    if (!axisEnabled[i]) {
      Serial.print("  Driver "); Serial.print(AXIS_NAMES[i]);
      Serial.println(": [BO QUA - truc tam tat]");
      continue;
    }

    tmcInit(i);

    // Retry: neu SPI khong phan hoi, reset va init lai 1 lan
    if (!tmcTestConnection(i)) {
      Serial.println("  -> SPI that bai! Thu reset va init lai...");
      spiReset();
      tmcInit(i);
    }

    printDriverInfo(i);
  }

  // BAT DRIVER sau khi da cau hinh xong
  Serial.println("--- Bat driver (EN=LOW) sau khi cau hinh ---");
  for (int i = 0; i < NUM_AXES; i++) {
    if (axisEnabled[i]) {
      digitalWrite(EN_PINS[i], LOW);
      delay(10);
      Serial.print("  EN_"); Serial.print(AXIS_NAMES[i]);
      Serial.print(" = "); Serial.print(digitalRead(EN_PINS[i]));
      Serial.println(" [BAT]");
    }
  }
  delay(100);  // Cho driver on dinh sau khi bat

  // Tinh STEPS_PER_MM va thong so homing tu microstep thuc te
  Serial.println("--- Tinh STEPS_PER_MM tu microstep ---");
  for (int i = 0; i < NUM_AXES; i++) {
    int ms;
    if (axisEnabled[i]) {
      ms = tmcGetMicrosteps(i);
      if (ms == 0) ms = 1;
    } else {
      ms = MICROSTEPS_AXIS[i];
    }

    STEPS_PER_MM[i] = (float)(MOTOR_STEPS_PER_REV * ms) / MM_PER_REV;
    maxHomingSteps[i] = (long)(100.0f * STEPS_PER_MM[i]);  // Toi da 100mm
    backoffSteps[i] = (int)(0.3f * STEPS_PER_MM[i]);       // Lui 0.3mm

    Serial.print("  "); Serial.print(AXIS_NAMES[i]);
    if (!axisEnabled[i]) Serial.print(" [TAT]");
    Serial.print(": 1/"); Serial.print(ms);
    Serial.print(" -> STEPS_PER_MM="); Serial.print(STEPS_PER_MM[i], 1);
    Serial.print(", maxHome="); Serial.print(maxHomingSteps[i]);
    Serial.print(", backoff="); Serial.println(backoffSteps[i]);
  }

  // In delay tung truc
  Serial.println("--- Delay tung truc (us) ---");
  for (int i = 0; i < NUM_AXES; i++) {
    Serial.print("  "); Serial.print(AXIS_NAMES[i]);
    if (!axisEnabled[i]) { Serial.println(": [TAT]"); continue; }
    Serial.print(": homingDelay="); Serial.print(homingDelayUs[i]);
    Serial.print("us, moveDelay="); Serial.print(moveDelayUs[i]);
    Serial.println("us");
  }

  // LED bat sang bao hoan tat
  digitalWrite(LED_PIN, LOW);

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
  Serial.println("CHAN DOAN:");
  Serial.println("  diag              Kiem tra ket noi SPI");
  Serial.println("  test              Quay thu tung dong co");
  Serial.println("  sw                Doc cong tac hanh trinh");
  Serial.println("  reg 6C            Doc thanh ghi (hex)");
  Serial.println("  reinit            Khoi tao lai driver");
  Serial.println("  loopback          Test loopback SPI");
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
      for (int i = 0; i < NUM_AXES; i++) {
        if (axisEnabled[i]) tmcEnableStealthChop(i);
      }
      Serial.println("-> Da bat StealthChop cho cac truc dang bat");
    }
    else if (inputStr.equalsIgnoreCase("spreadcycle")) {
      for (int i = 0; i < NUM_AXES; i++) {
        if (axisEnabled[i]) tmcEnableSpreadCycle(i);
      }
      Serial.println("-> Da bat SpreadCycle cho cac truc dang bat");
    }
    else if (inputStr.startsWith("current ") || inputStr.startsWith("Current ")) {
      int mA = inputStr.substring(8).toInt();
      if (mA >= 100 && mA <= 3000) {
        for (int i = 0; i < NUM_AXES; i++) {
          if (!axisEnabled[i]) continue;
          tmcSetCurrent(i, mA, mA / 2);
        }
        Serial.print("-> Dong dien dat lai: Run="); Serial.print(mA);
        Serial.print("mA, Hold="); Serial.print(mA / 2);
        Serial.println("mA");
      } else {
        Serial.println("!! Dong dien phai tu 100 den 3000 mA");
      }
    }
    else if (inputStr.startsWith("microstep ") || inputStr.startsWith("Microstep ")) {
      int ms = inputStr.substring(10).toInt();
      if (ms == 1 || ms == 2 || ms == 4 || ms == 8 || ms == 16 ||
          ms == 32 || ms == 64 || ms == 128 || ms == 256) {
        for (int i = 0; i < NUM_AXES; i++) {
          if (axisEnabled[i]) tmcSetMicrosteps(i, ms);
        }
        Serial.print("-> Microstep dat lai: 1/"); Serial.println(ms);
        Serial.println("!! LUU Y: Tinh lai STEPS_PER_MM bang lenh 'status' !!");
      } else {
        Serial.println("!! Gia tri hop le: 1, 2, 4, 8, 16, 32, 64, 128, 256");
      }
    }
    else if (inputStr.equalsIgnoreCase("test")) {
      testAllMotors();
    }
    else if (inputStr.equalsIgnoreCase("diag")) {
      runFullDiag();
    }
    else if (inputStr.startsWith("reg ") || inputStr.startsWith("Reg ")) {
      // Doc thanh ghi bat ky: "reg 6C"
      String regStr = inputStr.substring(4);
      regStr.trim();
      uint8_t reg = (uint8_t)strtol(regStr.c_str(), NULL, 16);
      Serial.print("Doc thanh ghi 0x");
      if (reg < 0x10) Serial.print("0");
      Serial.print(reg, HEX);
      Serial.println(":");
      for (int i = 0; i < NUM_AXES; i++) {
        if (!axisEnabled[i]) continue;
        uint32_t val = tmcRead(i, reg);
        Serial.print("  "); Serial.print(AXIS_NAMES[i]);
        Serial.print(": 0x");
        // In day du 8 ky tu hex
        for (int b = 28; b >= 0; b -= 4) {
          Serial.print((val >> b) & 0x0F, HEX);
        }
        Serial.println();
      }
    }
    else if (inputStr.equalsIgnoreCase("loopback")) {
      runLoopbackTest();
    }
    else if (inputStr.equalsIgnoreCase("reinit")) {
      Serial.println("\n--- REINIT DRIVER ---");
      spiReset();
      for (int i = 0; i < NUM_AXES; i++) {
        if (!axisEnabled[i]) continue;
        tmcInit(i);
        printDriverInfo(i);
      }
      Serial.println("--- Bat driver (EN=LOW) ---");
      for (int i = 0; i < NUM_AXES; i++) {
        if (!axisEnabled[i]) continue;
        digitalWrite(EN_PINS[i], LOW);
        delay(10);
        Serial.print("  EN_"); Serial.print(AXIS_NAMES[i]);
        Serial.print(" = "); Serial.println(digitalRead(EN_PINS[i]));
      }
      Serial.println("--- REINIT XONG ---");
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
    if (!axisEnabled[i]) {
      Serial.print("Test truc "); Serial.print(AXIS_NAMES[i]);
      Serial.println(": [BO QUA - truc tam tat]");
      continue;
    }
    Serial.print("Test truc ");
    Serial.print(AXIS_NAMES[i]);
    Serial.print(": EN=");
    digitalWrite(EN_PINS[i], LOW);
    Serial.print(digitalRead(EN_PINS[i]));
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

  // In trang thai cong tac truoc homing
  Serial.println("Trang thai cong tac truoc homing:");
  for (int i = 0; i < NUM_AXES; i++) {
    if (!axisEnabled[i]) {
      Serial.print("  SW_"); Serial.print(AXIS_NAMES[i]); Serial.println(": [BO QUA]");
      continue;
    }
    Serial.print("  SW_"); Serial.print(AXIS_NAMES[i]);
    Serial.print(" (pin "); Serial.print(SW_PINS[i]); Serial.print("): ");
    Serial.println(digitalRead(SW_PINS[i]) == LOW ? "!! DANG NHAN (LOW)" : "Nha (HIGH) OK");
  }
  Serial.flush();

  for (int i = 0; i < NUM_AXES; i++) {
    isHomed[i] = axisEnabled[i] ? false : true;
  }

  // GIAI DOAN 0: Lui ra neu cong tac dang bi nhan
  bool needBackoff = false;
  for (int i = 0; i < NUM_AXES; i++) {
    if (!axisEnabled[i]) continue;
    if (digitalRead(SW_PINS[i]) == LOW) {
      delay(5);
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
      digitalWrite(DIR_PINS[i], HIGH);
    }
    delayMicroseconds(20);
    for (int s = 0; s < 2000; s++) {
      for (int i = 0; i < NUM_AXES; i++) {
        if (!axisEnabled[i]) continue;
        if (digitalRead(SW_PINS[i]) == HIGH) continue;
        digitalWrite(STEP_PINS[i], HIGH);
      }
      delayMicroseconds(200);
      for (int i = 0; i < NUM_AXES; i++) {
        digitalWrite(STEP_PINS[i], LOW);
      }
      delayMicroseconds(200);
    }
    delay(200);
    Serial.println("  -> Lui xong.");
  }

  // Dat huong ve phia cong tac
  for (int i = 0; i < NUM_AXES; i++) {
    digitalWrite(DIR_PINS[i], LOW);
  }
  delayMicroseconds(20);

  // GIAI DOAN 1: Tim cong tac - moi truc co timer rieng (micros)
  Serial.println("[1/2] Tim cong tac hanh trinh...");
  Serial.flush();

  bool triggered[NUM_AXES]  = {false, false, false};
  for (int i = 0; i < NUM_AXES; i++) { if (!axisEnabled[i]) triggered[i] = true; }
  bool stepHigh[NUM_AXES]   = {false, false, false};
  long stepCount[NUM_AXES]  = {0, 0, 0};
  unsigned long swLowSince[NUM_AXES] = {0, 0, 0};
  bool swTiming[NUM_AXES] = {false, false, false};
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
          digitalWrite(STEP_PINS[i], LOW);
          stepHigh[i] = false;
          nextStep[i] = now + homingDelayUs[i];
          stepCount[i]++;
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
            swTiming[i] = false;
          }
        } else {
          digitalWrite(STEP_PINS[i], HIGH);
          stepHigh[i] = true;
          nextStep[i] = now + homingDelayUs[i];
        }
      }
    }
  }

  // Ha tat ca STEP pins truoc giai doan 2
  for (int i = 0; i < NUM_AXES; i++) {
    if (!axisEnabled[i]) continue;
    if (stepHigh[i]) { digitalWrite(STEP_PINS[i], LOW); stepHigh[i] = false; }
    if (stepCount[i] < maxHomingSteps[i]) digitalWrite(DIR_PINS[i], HIGH);
  }
  delayMicroseconds(20);

  // GIAI DOAN 2: Lui ra khoi cong tac
  Serial.println("[2/2] Lui ra khoi cong tac...");
  Serial.flush();

  bool backedOff[NUM_AXES] = {false, false, false};
  for (int i = 0; i < NUM_AXES; i++) { if (!axisEnabled[i]) backedOff[i] = true; }
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

  // Lui lai de nha cong tac
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
// ==========================================
void parseAndMove(String input) {
  float targetMm[NUM_AXES];
  bool hasTarget[NUM_AXES] = {false, false, false};

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

  bool anyTarget = false;
  for (int i = 0; i < NUM_AXES; i++) {
    if (hasTarget[i]) { anyTarget = true; break; }
  }

  if (!anyTarget) {
    Serial.println("!! Lenh khong hop le. VD: X50.5 Y30 Z10");
    return;
  }

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

  long stepsToMove[NUM_AXES];
  for (int i = 0; i < NUM_AXES; i++) {
    long targetSteps = (long)(targetMm[i] * STEPS_PER_MM[i]);
    stepsToMove[i] = targetSteps - currentPositionSteps[i];
  }

  moveMultiAxis(stepsToMove);

  Serial.print("-> Hoan thanh. Vi tri: ");
  printCurrentPositions();
  Serial.println("Nhap toa do tiep theo (hoac 'home'):");
}

// ==========================================
// DI CHUYEN DONG THOI NHIEU TRUC
// Su dung thuat toan Bresenham mo rong
// ==========================================
void moveMultiAxis(long stepsToMove[]) {
  long absSteps[NUM_AXES];
  int  dirMult[NUM_AXES];
  long maxSteps = 0;

  for (int i = 0; i < NUM_AXES; i++) {
    if (stepsToMove[i] > 0) {
      digitalWrite(DIR_PINS[i], HIGH);
      absSteps[i] = stepsToMove[i];
      dirMult[i] = 1;
    } else if (stepsToMove[i] < 0) {
      digitalWrite(DIR_PINS[i], LOW);
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

  int loopDelay = moveDelayUs[0];
  for (int i = 0; i < NUM_AXES; i++) {
    if (absSteps[i] == maxSteps) {
      loopDelay = moveDelayUs[i];
      break;
    }
  }

  long counter[NUM_AXES] = {0, 0, 0};
  bool stopped[NUM_AXES] = {false, false, false};

  for (long step = 0; step < maxSteps; step++) {
    // Kiem tra cong tac hanh trinh
    for (int i = 0; i < NUM_AXES; i++) {
      if (!stopped[i] && absSteps[i] > 0 && isSwitchPressedFast(i)) {
        stopped[i] = true;
        Serial.print("!! CANH BAO: Truc ");
        Serial.print(AXIS_NAMES[i]);
        Serial.println(" dung khan cap do cham cong tac !!");
      }
    }

    bool allDone = true;
    for (int i = 0; i < NUM_AXES; i++) {
      if (absSteps[i] > 0 && !stopped[i]) {
        allDone = false;
        break;
      }
    }
    if (allDone) break;

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

// ==========================================
// LOOPBACK TEST: Noi MOSI(PA7) vao MISO(PA6) truc tiep
// De kiem tra MCU GPIO + duong SPI co hoat dong khong
// ==========================================
void runLoopbackTest() {
  Serial.println("\n===== SPI LOOPBACK TEST =====");
  Serial.println("CACH LAM: Thao day MISO khoi driver.");
  Serial.println("Noi 1 day: PA7 (MOSI) -> PA6 (MISO) truc tiep.");
  Serial.println("Doi 3 giay roi test...\n");
  delay(3000);

  // Tat SPI hardware, test bang GPIO
  SPI.end();
  delay(10);
  pinMode(PA7, OUTPUT);
  pinMode(PA6, INPUT);

  int pass = 0;
  int fail = 0;

  Serial.println("[1] GPIO LOOPBACK (PA7 -> PA6):");
  uint8_t testVals[] = {0x00, 0xFF, 0xAA, 0x55, 0x0F, 0xF0, 0xA5, 0x5A};
  for (int t = 0; t < 8; t++) {
    // Gui tung bit tu PA7, doc lai tu PA6
    uint8_t txByte = testVals[t];
    uint8_t rxByte = 0;
    for (int bit = 7; bit >= 0; bit--) {
      int val = (txByte >> bit) & 1;
      digitalWrite(PA7, val);
      delayMicroseconds(10);
      int readVal = digitalRead(PA6);
      rxByte |= (readVal << bit);
    }
    bool ok = (txByte == rxByte);
    if (ok) pass++; else fail++;
    Serial.print("  TX=0x"); Serial.print(txByte, HEX);
    Serial.print(" RX=0x"); Serial.print(rxByte, HEX);
    Serial.println(ok ? " [OK]" : " [LOI!]");
  }

  Serial.print("\nKET QUA: "); Serial.print(pass); Serial.print("/8 pass, ");
  Serial.print(fail); Serial.println("/8 fail");

  if (pass == 8) {
    Serial.println("-> GPIO PA6/PA7 HOAT DONG TOT!");
    Serial.println("-> LOI O PHIA DRIVER (SDO khong xuat du lieu)");
    Serial.println("");
    Serial.println("=== KIEM TRA DRIVER ===");
    Serial.println("1. Motor co BO CUNG khi cap VM + EN=GND khong?");
    Serial.println("   (Thao tat ca SPI, chi noi VM+GND+VCC_IO+EN=GND)");
    Serial.println("   Neu KHONG bo cung -> driver hong hoac day motor sai");
    Serial.println("2. Do dien ap VCC_IO bang VOM: phai = 3.0-3.3V");
    Serial.println("3. Do dien ap VM bang VOM: phai = 12-24V");
    Serial.println("4. MKS TMC2240: Kiem tra lai solder pad SPI/standalone");
    Serial.println("   Chup anh mat truoc + mat sau board gui cho toi");
  } else if (pass == 0) {
    Serial.println("-> PA7->PA6 KHONG KET NOI!");
    Serial.println("   Kiem tra lai day loopback giua PA7 va PA6");
  } else {
    Serial.println("-> KET NOI KHONG ON DINH - kiem tra tiep xuc");
  }

  // Khoi phuc SPI
  SPI.begin();
  for (int i = 0; i < NUM_AXES; i++) {
    pinMode(CS_PINS[i], OUTPUT);
    digitalWrite(CS_PINS[i], HIGH);
  }
  delay(10);
  Serial.println("\n===== KET THUC LOOPBACK =====\n");
}

// ==========================================
// CHAN DOAN SPI - KIEM TRA KET NOI TMC2240
// ==========================================
void runFullDiag() {
  Serial.println("\n===== CHAN DOAN SPI TMC2240 =====");

  // Kiem tra trang thai chan
  Serial.println("[0] TRANG THAI CHAN:");
  for (int i = 0; i < NUM_AXES; i++) {
    if (!axisEnabled[i]) {
      Serial.print("  "); Serial.print(AXIS_NAMES[i]); Serial.println(": [TAT]");
      continue;
    }
    Serial.print("  "); Serial.print(AXIS_NAMES[i]);
    Serial.print(": EN="); Serial.print(digitalRead(EN_PINS[i]));
    Serial.print("(0=bat) CS_pin="); Serial.print(CS_PINS[i]);
    Serial.println();
  }

  // Doc VERSION tu IOIN
  Serial.println("\n[1] DOC IOIN (VERSION + IO STATUS):");
  for (int i = 0; i < NUM_AXES; i++) {
    if (!axisEnabled[i]) {
      Serial.print("  "); Serial.print(AXIS_NAMES[i]); Serial.println(": [BO QUA]");
      continue;
    }
    uint32_t ioin = tmcRead(i, TMC_IOIN);
    uint8_t version = (ioin >> 24) & 0xFF;
    Serial.print("  "); Serial.print(AXIS_NAMES[i]);
    Serial.print(": IOIN=0x");
    for (int b = 28; b >= 0; b -= 4) Serial.print((ioin >> b) & 0x0F, HEX);
    Serial.print(" VERSION=0x"); Serial.print(version, HEX);
    if (version == 0x40) Serial.println(" [OK - TMC2240]");
    else if (version == 0x00 || version == 0xFF) Serial.println(" [LOI - KHONG KET NOI]");
    else { Serial.print(" [KHAC - Co the TMC"); Serial.print(version, HEX); Serial.println("]"); }
  }

  // Doc GCONF
  Serial.println("\n[2] DOC GCONF:");
  for (int i = 0; i < NUM_AXES; i++) {
    if (!axisEnabled[i]) continue;
    uint32_t gconf = tmcRead(i, TMC_GCONF);
    Serial.print("  "); Serial.print(AXIS_NAMES[i]);
    Serial.print(": 0x");
    for (int b = 28; b >= 0; b -= 4) Serial.print((gconf >> b) & 0x0F, HEX);
    Serial.print(" en_pwm="); Serial.print((gconf >> 2) & 1);
    Serial.print(" multistep_filt="); Serial.println((gconf >> 3) & 1);
  }

  // Doc CHOPCONF
  Serial.println("\n[3] DOC CHOPCONF:");
  for (int i = 0; i < NUM_AXES; i++) {
    if (!axisEnabled[i]) continue;
    uint32_t chopconf = tmcRead(i, TMC_CHOPCONF);
    uint8_t mres = (chopconf >> 24) & 0x0F;
    Serial.print("  "); Serial.print(AXIS_NAMES[i]);
    Serial.print(": 0x");
    for (int b = 28; b >= 0; b -= 4) Serial.print((chopconf >> b) & 0x0F, HEX);
    Serial.print(" MRES="); Serial.print(mres);
    Serial.print("(1/"); Serial.print(mresToMicrosteps(mres));
    Serial.print(") TOFF="); Serial.print(chopconf & 0x0F);
    Serial.print(" INTPOL="); Serial.println((chopconf >> 28) & 1);
  }

  // Doc DRV_STATUS
  Serial.println("\n[4] DOC DRV_STATUS:");
  for (int i = 0; i < NUM_AXES; i++) {
    if (!axisEnabled[i]) continue;
    uint32_t ds = tmcGetDrvStatus(i);
    Serial.print("  "); Serial.print(AXIS_NAMES[i]);
    Serial.print(": 0x");
    for (int b = 28; b >= 0; b -= 4) Serial.print((ds >> b) & 0x0F, HEX);
    Serial.print(" OT="); Serial.print((ds >> 25) & 1);
    Serial.print(" OTPW="); Serial.print((ds >> 26) & 1);
    Serial.print(" S2G="); Serial.print(((ds >> 27) & 1) | ((ds >> 28) & 1));
    Serial.print(" OLA="); Serial.print((ds >> 29) & 1);
    Serial.print(" OLB="); Serial.print((ds >> 30) & 1);
    Serial.print(" STST="); Serial.println((ds >> 31) & 1);
  }

  // Doc GSTAT
  Serial.println("\n[5] DOC GSTAT (co loi):");
  for (int i = 0; i < NUM_AXES; i++) {
    if (!axisEnabled[i]) continue;
    uint32_t gstat = tmcRead(i, TMC_GSTAT);
    Serial.print("  "); Serial.print(AXIS_NAMES[i]);
    Serial.print(": 0x");
    Serial.print(gstat, HEX);
    Serial.print(" reset="); Serial.print(gstat & 1);
    Serial.print(" drv_err="); Serial.print((gstat >> 1) & 1);
    Serial.print(" uv_cp="); Serial.println((gstat >> 2) & 1);
    // Xoa co loi
    tmcWrite(i, TMC_GSTAT, 0x07);
  }

  // ===== BIT-BANG TEST: kiem tra tung day SPI rieng le =====
  Serial.println("\n[6] BIT-BANG SPI TEST (khong dung SPI hardware):");
  Serial.println("    Tat SPI hardware, test tung day bang GPIO...");

  // Tam dung SPI hardware
  SPI.end();
  delay(10);

  // Cau hinh lai cac chan SPI thanh GPIO
  pinMode(PA5, OUTPUT);   // SCK
  pinMode(PA7, OUTPUT);   // MOSI
  pinMode(PA6, INPUT);    // MISO

  // Test 1: Doc MISO khi khong lam gi
  Serial.print("  MISO(PA6) mac dinh   = "); Serial.println(digitalRead(PA6));

  // Test 2: Toggle tung CS va doc MISO
  Serial.println("  Toggle CS va doc MISO:");
  for (int i = 0; i < NUM_AXES; i++) {
    if (!axisEnabled[i]) continue;
    digitalWrite(PA5, LOW);   // SCK LOW
    digitalWrite(PA7, LOW);   // MOSI LOW
    digitalWrite(CS_PINS[i], HIGH);
    delayMicroseconds(100);
    int miso_before = digitalRead(PA6);

    digitalWrite(CS_PINS[i], LOW);
    delayMicroseconds(100);
    int miso_after = digitalRead(PA6);
    digitalWrite(CS_PINS[i], HIGH);

    Serial.print("    CS_"); Serial.print(AXIS_NAMES[i]);
    Serial.print(": MISO truoc="); Serial.print(miso_before);
    Serial.print(" sau CS_LOW="); Serial.print(miso_after);
    if (miso_before == miso_after) Serial.println(" (KHONG THAY DOI - driver khong phan hoi)");
    else Serial.println(" (THAY DOI - driver co phan hoi!)");
  }

  // Test 3: Bit-bang gui lenh doc IOIN (0x04) cho driver X
  Serial.println("  Bit-bang doc IOIN(0x04) driver X:");
  digitalWrite(CS_PINS[0], LOW);
  delayMicroseconds(10);

  // Gui byte 0x04 (read IOIN) bang bit-bang
  uint8_t txByte = 0x04;
  uint8_t rxByte = 0;
  for (int bit = 7; bit >= 0; bit--) {
    // Dat MOSI
    digitalWrite(PA7, (txByte >> bit) & 1);
    delayMicroseconds(5);
    // Clock HIGH (CPOL=1, CPHA=1: sample on falling edge)
    digitalWrite(PA5, HIGH);
    delayMicroseconds(5);
    // Doc MISO
    rxByte |= (digitalRead(PA6) << bit);
    // Clock LOW
    digitalWrite(PA5, LOW);
    delayMicroseconds(5);
  }
  // Gui 4 byte dummy, doc du lieu
  uint8_t rxData[4] = {0, 0, 0, 0};
  for (int byteIdx = 0; byteIdx < 4; byteIdx++) {
    for (int bit = 7; bit >= 0; bit--) {
      digitalWrite(PA7, LOW);
      delayMicroseconds(5);
      digitalWrite(PA5, HIGH);
      delayMicroseconds(5);
      rxData[byteIdx] |= (digitalRead(PA6) << bit);
      digitalWrite(PA5, LOW);
      delayMicroseconds(5);
    }
  }
  delayMicroseconds(10);
  digitalWrite(CS_PINS[0], HIGH);

  Serial.print("    Status=0x"); Serial.print(rxByte, HEX);
  Serial.print(" Data=0x");
  Serial.print(rxData[0], HEX); Serial.print(" ");
  Serial.print(rxData[1], HEX); Serial.print(" ");
  Serial.print(rxData[2], HEX); Serial.print(" ");
  Serial.println(rxData[3], HEX);
  if (rxByte == 0x00 && rxData[0] == 0x00 && rxData[1] == 0x00) {
    Serial.println("    -> 0x00: MISO LUON LOW -> Day SDO khong noi hoac driver chua cap dien");
  } else if (rxByte == 0xFF && rxData[0] == 0xFF) {
    Serial.println("    -> 0xFF: MISO LUON HIGH -> Day SDO bi hoc hoac pull-up sai");
  } else {
    Serial.println("    -> CO DU LIEU! SPI hardware co the bi loi cau hinh");
  }

  // Test 4: Kiem tra SCK co thay doi duoc khong (debug day noi)
  Serial.println("  Kiem tra toggle SCK:");
  for (int t = 0; t < 3; t++) {
    digitalWrite(PA5, HIGH);
    delayMicroseconds(100);
    int h = digitalRead(PA5);
    digitalWrite(PA5, LOW);
    delayMicroseconds(100);
    int l = digitalRead(PA5);
    Serial.print("    SCK HIGH="); Serial.print(h);
    Serial.print(" LOW="); Serial.println(l);
  }

  Serial.println("  Kiem tra toggle MOSI:");
  for (int t = 0; t < 3; t++) {
    digitalWrite(PA7, HIGH);
    delayMicroseconds(100);
    int h = digitalRead(PA7);
    digitalWrite(PA7, LOW);
    delayMicroseconds(100);
    int l = digitalRead(PA7);
    Serial.print("    MOSI HIGH="); Serial.print(h);
    Serial.print(" LOW="); Serial.println(l);
  }

  // Khoi phuc SPI hardware
  SPI.begin();
  // Khoi phuc CS pins
  for (int i = 0; i < NUM_AXES; i++) {
    pinMode(CS_PINS[i], OUTPUT);
    digitalWrite(CS_PINS[i], HIGH);
  }
  delay(10);

  Serial.println("\n===== KET LUAN =====");
  Serial.println("- VERSION=0x40: TMC2240 ket noi thanh cong");
  Serial.println("- VERSION=0x00/0xFF: Kiem tra day SPI, CS, VCC_IO, VM");
  Serial.println("- Neu bit-bang cung 0x00: loi phan cung (day, nguon)");
  Serial.println("- Neu bit-bang co du lieu: loi SPI hardware config");
  Serial.println("- drv_err=1: Loi driver (qua nhiet, ngan mach...)");
  Serial.println("- uv_cp=1: Charge pump thap (kiem tra nguon VM)");
  Serial.println("");
  Serial.println("KIEM TRA PHAN CUNG:");
  Serial.println("  1. VM (12-24V) da cap cho driver chua?");
  Serial.println("  2. VCC_IO noi 3.3V chua?");
  Serial.println("  3. MKS TMC2240: co jumper/solder pad chon SPI?");
  Serial.println("     (Nhieu module MKS mac dinh UART, can han chon SPI)");
  Serial.println("  4. Day SDO(MISO) noi dung chan PA6?");
  Serial.println("  5. Day SDI(MOSI) noi dung chan PA7?");
  Serial.println("  6. Day SCK noi dung chan PA5?");
  Serial.println("===================================\n");
}
