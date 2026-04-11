// ==========================================
// CẤU HÌNH CHÂN KẾT NỐI (STM32 Blue Pill)
// ==========================================
#define DIR_PIN PA4
#define STEP_PIN PA5
#define EN_PIN PA7
#define SW_PIN PB0 // Công tắc hành trình (Normally Open)

// ==========================================
// THÔNG SỐ HIỆU CHỈNH CƠ KHÍ
// ==========================================
const float STEPS_PER_MM = 80.0;

long currentPositionSteps = 0;
bool isHomed = false;

// ================== HÀM DEBOUNCE MẠNH (50ms + 3 lần kiểm tra)
// ==================
bool isSwitchPressed() {
  if (digitalRead(SW_PIN) == HIGH)
    return false;
  delay(50); // debounce rất mạnh
  if (digitalRead(SW_PIN) == HIGH)
    return false;
  delay(20);
  if (digitalRead(SW_PIN) == HIGH)
    return false;
  return true; // LOW thật sự sau 3 lần kiểm tra
}

void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(SW_PIN, INPUT_PULLUP);

  digitalWrite(EN_PIN, LOW); // bật driver

  Serial.println("=========================================");
  Serial.println("He thong da khoi dong. KHONG TU DONG HOMING.");
  Serial.println("Vui long nhap 'home' de chay ve goc thuc te.");
}

void loop() {
  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim();

    if (inputStr.length() > 0) {
      if (inputStr.equalsIgnoreCase("home")) {
        runHoming();
      } else {
        float targetMm = inputStr.toFloat();
        long targetSteps = (long)(targetMm * STEPS_PER_MM);
        long stepsToMove = targetSteps - currentPositionSteps;

        Serial.print("\nLenh nhan duoc: Di chuyen den moc ");
        Serial.print(targetMm);
        Serial.println(" mm");

        moveSteps(stepsToMove);

        Serial.print("-> Da hoan thanh / Dung lai. Toa do thuc te hien tai: ");
        Serial.print((float)currentPositionSteps / STEPS_PER_MM);
        Serial.println(" mm\n");
        Serial.println("Nhap toa do mm tiep theo (Hoac nhap 'home' de ve Goc "
                       "bat cu luc nao):");
      }
    }
  }
}

// ==========================================
// HÀM HOMING (không có emergency stop)
// ==========================================
void runHoming() {
  Serial.println("Dang chay ve tim Goc (Homing)...");

  digitalWrite(DIR_PIN, LOW); // về phía công tắc
  long stepCount = 0;
  const long MAX_STEPS = 20000;

  while (!isSwitchPressed() && stepCount < MAX_STEPS) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(800);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(800);
    stepCount++;
  }

  if (stepCount >= MAX_STEPS) {
    Serial.println(
        "!! LOI: Khong tim thay cong tac sau 20000 buoc !! Kiem tra day noi.");
    return;
  }

  // Lùi lại 2mm để nhả công tắc
  digitalWrite(DIR_PIN, HIGH);
  for (int i = 0; i < 160; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(800);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(800);
  }

  currentPositionSteps = 0;
  isHomed = true;
  Serial.println("-> Homing thanh cong! He toa do da duoc reset ve 0.");
  Serial.println("=========================================");
  Serial.println("NHAP TOA DO BAN MUON DEN TREN SERIAL MONITOR (VD: 50.5)");
}

// ==========================================
// HÀM DI CHUYỂN
// ==========================================
void moveSteps(long steps) {
  if (steps == 0) {
    Serial.println("(Ban dang o dung vi tri nay roi, khong can di chuyen)");
    return;
  }

  int direction_multiplier = 1;
  if (steps > 0) {
    digitalWrite(DIR_PIN, HIGH); // dương = ra xa công tắc
  } else {
    digitalWrite(DIR_PIN, LOW); // âm = về phía công tắc
    steps = -steps;
    direction_multiplier = -1;
  }

  for (long i = 0; i < steps; i++) {
    if (isSwitchPressed()) {
      Serial.println(
          "!! CANH BAO: Dung khan cap do cham cong tac hanh trinh !!");
      break;
    }

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(400);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(400);

    currentPositionSteps += direction_multiplier;
  }
}