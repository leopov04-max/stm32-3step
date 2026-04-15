# Motor Controller

## 1. Muc dich cua chuong trinh

Day la sketch Arduino cho STM32 Blue Pill dung de dieu khien 1 truc stepper qua giao tiep `STEP/DIR/EN`, co cong tac hanh trinh de tim goc `home`, va nhan lenh tu `Serial Monitor`.

Code hien tai cho phep:

- Bat driver dong co.
- Nhan lenh `home` de chay ve goc co khi.
- Nhan toa do theo don vi `mm`.
- Quy doi `mm` sang `step`.
- Di chuyen den vi tri dich tuyet doi.
- Dung lai neu cham cong tac hanh trinh trong luc chay.

## 2. Phan cung co the dang duoc su dung

Phan nay la suy doan dua tren code, khong phai xac nhan tuyet doi.

### Vi dieu khien va Driver

- STM32 Blue Pill
- TMC2208

### Ket noi dieu khien dong co

Code dang dung cac chan sau:

- `PA4` -> `DIR_PIN`
- `PA5` -> `STEP_PIN`
- `PA7` -> `EN_PIN`
- `PB0` -> `SW_PIN` (cong tac hanh trinh)

Kieu giao tiep nay thuong dung voi cac driver nhu:

- A4988
- DRV8825
- TB6600
- hoac driver bat ky co 3 chan `STEP`, `DIR`, `EN`

### Dong co co the dang dung

- 1 dong co stepper 2 pha
- co co cau truyen dong tinh tien, vi du vit me hoac day dai

Gia tri:

- `STEPS_PER_MM = 80.0`

Dieu nay cho thay he thong da duoc hieu chinh de 1 mm tuong ung 80 xung step. Gia tri nay phu thuoc vao:

- step angle cua dong co
- microstep cua driver
- ti so truyen
- buoc vit me hoac pitch day dai

### Cong tac hanh trinh

Code cau hinh:

- `pinMode(SW_PIN, INPUT_PULLUP);`

Nen cong tac kha nang cao dang duoc dau theo kieu:

- 1 dau vao `PB0`
- 1 dau xuong `GND`
- trang thai binh thuong la `HIGH`
- khi nhan cong tac se thanh `LOW`

Ghi chu trong code goi day la cong tac hanh trinh `Normally Open`, nhung cach dau day thuc te can duoc xac nhan lai ngoai mach.

### Nguon va ket noi voi may tinh

He thong dang co:

- 1 nguon rieng cho driver va dong co
- 1 ket noi USB/Serial de gui lenh tu may tinh

## 3. Chuc nang cua code hien tai

### 3.1. Khoi tao he thong

Trong `setup()`:

- mo `Serial` o toc do `115200`
- cau hinh cac chan vao/ra
- bat driver bang cach ghi `EN_PIN = LOW`
- in huong dan len Serial

He thong **khong tu dong homing** khi vua cap nguon ma chi **homing** khi nhan duoc lenh.

### 3.2. Debounce cong tac hanh trinh

Ham `isSwitchPressed()` doc cong tac 3 lan:

- lan 1 doc ngay
- delay `50 ms`
- lan 2
- delay `20 ms`
- lan 3

Chi khi ca 3 lan deu cho ket qua nhan cong tac thi moi xem la hop le.

Tac dung:

- giam rung tin hieu co khi
- tranh dung sai do tiep diem bi doi trang thai nhanh

Nhuoc diem:

- moi lan kiem tra co the mat toi da khoang `70 ms`
- he thong phan ung cham hon

### 3.3. Nhan lenh tu Serial

Trong `loop()`, chuong trinh doc tung dong lenh ket thuc bang ky tu xuong dong.

Neu nhan:

- `home` -> chay quy trinh homing
- mot so, vi du `50` hoac `50.5` -> coi la toa do dich theo `mm`

Code dang xu ly toa do theo **moc tuyet doi**, khong phai di them tuong doi.

Vi du:

- dang o `0 mm`, nhap `50` -> chay den `50 mm`
- dang o `50 mm`, nhap `20` -> chay nguoc ve `20 mm`

### 3.4. Homing

Ham `runHoming()` se:

1. quay dong co theo chieu ve phia cong tac
2. tao xung `STEP` voi tre `800 us`
3. dung khi cham cong tac hoac vuot qua `20000` buoc
4. neu khong tim thay cong tac trong `20000` buoc thi bao loi
5. sau khi cham cong tac, lui ra `160` buoc
6. dat `currentPositionSteps = 0`
7. dat `isHomed = true`

Do `STEPS_PER_MM = 80`, viec lui `160` buoc tuong ung:

- `2 mm`

Y nghia:

- sau khi homing, vi tri hien tai duoc xem la moc `0 mm`
- moc `0 mm` trong code la vi tri sau khi da roi cong tac 2 mm, khong phai diem dang de cong tac

### 3.5. Di chuyen den vi tri dich

Ham `moveSteps(long steps)` se:

- bo qua neu so buoc bang `0`
- chon chieu quay dua theo dau cua `steps`
- phat xung `STEP`
- cap nhat `currentPositionSteps` sau moi buoc

Toc do phat xung trong ham nay:

- `HIGH 400 us`
- `LOW 400 us`

Moi chu ky step mat khoang `800 us`, chua tinh them chi phi xu ly.

### 3.6. Bao ve bang cong tac hanh trinh

Trong luc di chuyen, neu `isSwitchPressed()` tra ve `true`, chuong trinh:

- in canh bao
- dung vong lap chay buoc

Dieu nay giup tranh co cau tiep tuc day vao diem gioi han.

## 4. Cach su dung hien tai

### Buoc 1. Nap code

- nap sketch vao STM32 Blue Pill
- cap nguon cho mach dieu khien va driver

### Buoc 2. Mo Serial Monitor

Cau hinh:

- baud rate: `115200`
- line ending: `Newline`

### Buoc 3. Chay homing

Nhap:

```text
home
```

Luc nay truc se chay ve phia cong tac, cham cong tac, sau do lui ra 2 mm va dat moc hien tai la `0 mm`.

### Buoc 4. Nhap toa do dich

Vi du:

```text
10
25.5
0
```

Y nghia:

- `10` -> di den `10 mm`
- `25.5` -> di den `25.5 mm`
- `0` -> quay lai moc `0 mm`

## 5. Gioi han va hanh vi can luu y

Day la nhung gi code hien tai **dang co** hoac **chua co**:

- Khong tu dong homing khi bat nguon.
- Van cho phep nhap toa do va di chuyen ngay ca khi chua `home`.
- Bien `isHomed` da duoc cap nhat nhung chua duoc dung de khoa lenh di chuyen.
- Khong co gioi han mem min/max theo `mm`.
- Khong co tang/giam toc, nen dong co chay voi toc do co dinh.
- Khong co xu ly lenh Serial sai dinh dang mot cach chat che.
- Neu nhap chuoi khong phai so, `toFloat()` co the dua ve `0`, dan toi chay ve moc `0 mm`.
- Cong tac hanh trinh duoc kiem tra trong moi huong di chuyen, nen neu cong tac dang bi nhan thi truc co the dung ngay.
- Homing co gioi han an toan la `20000` buoc, tuong ung `250 mm` voi cau hinh hien tai.

## 6. Tom tat logic dieu khien

Luong hoat dong tong quat:

1. khoi dong board
2. cho lenh tu Serial
3. neu lenh la `home` -> tim goc
4. neu lenh la so -> doi sang `step`
5. tinh sai lech giua vi tri dich va vi tri hien tai
6. phat xung de di chuyen
7. cap nhat vi tri noi bo

## 7. De xuat cai tien tiep theo

Neu muon dung he thong an toan va de van hanh hon, nen bo sung:

- chan khong cho di chuyen neu chua homing
- kiem tra chuoi nhap co hop le hay khong
- gioi han hanh trinh mem theo `mm`
- nut emergency stop rieng
- dieu khien tang/giam toc
- hien thi trang thai hien tai ro hon tren Serial
