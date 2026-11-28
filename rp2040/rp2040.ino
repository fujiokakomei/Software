#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <PCA95x5.h> // hideakitai/PCA95x5

#include <Arduino.h>

constexpr uint8_t SENSOR_COUNT = 12;
constexpr uint8_t PCA9555_ADDR = 0x20;
constexpr uint8_t VL53L0X_DEFAULT_ADDR = 0x29;
constexpr uint32_t USB_SERIAL_BAUD = 115200;
constexpr uint32_t TEENSY_SERIAL_BAUD = 115200;
constexpr uint8_t FRAME_HEADER[] = {'V', 'L'};
constexpr size_t PAYLOAD_SIZE = SENSOR_COUNT * sizeof(uint16_t);

// 起動時のXSHUT操作後の待ち時間。初回は動作確認済みの長めの値、
// 動作中の復旧では止まっている時間を短くするため短めの値を使う。
constexpr uint32_t SETUP_BOOT_DELAY_MS = 100;
constexpr uint32_t RECOVERY_BOOT_DELAY_MS = 20;

// I2Cが応答しないときに待つ上限。超えたらI2C周辺回路をリセットする。
constexpr uint32_t I2C_TIMEOUT_MS = 10;
// センサーが応答しない、または新しい測定値が来ない状態が続いたら全センサーを再初期化する。
constexpr uint8_t MAX_CONSECUTIVE_FAILURES = 3;
constexpr uint32_t STALE_MEASUREMENT_MS = 500;
// loop()が止まったらRP2040を再起動する（再起動後はsetup()で再初期化される）。
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 4000;
constexpr uint32_t STATUS_PRINT_INTERVAL_MS = 1000;

PCA9555 ioex;

// 使用しないVL53L0Xの番号（0始まり）。未使用がない場合は範囲外のSENSOR_COUNTを残す。
constexpr uint8_t UNUSED_SENSOR_NUMBERS[] = {4,5,7,8};

// VL53L0Xオブジェクトの配列
Adafruit_VL53L0X lox[SENSOR_COUNT];

// 各センサーに割り当てる一意のI2Cアドレス (初期値0x29以外の衝突しないアドレス)
const uint8_t sensor_addresses[SENSOR_COUNT] = {
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B
};

uint16_t distance_data[SENSOR_COUNT];

// 状態監視用
uint32_t last_measurement_ms[SENSOR_COUNT];
uint8_t consecutive_failures = 0;
uint32_t recovery_count = 0;
uint32_t frames_sent = 0;
uint32_t last_status_print_ms = 0;
uint16_t last_failed_mask = 0;

bool isUnusedSensor(uint8_t sensor_number) {
    for (uint8_t unused_sensor_number : UNUSED_SENSOR_NUMBERS) {
        if (sensor_number == unused_sensor_number) {
            return true;
        }
    }
    return false;
}

// PCA9555の出力を設定する関数 (16ビット分)
bool setPCA9555Outputs(uint16_t pin_states) {
    return ioex.write(pin_states);
}

// PCA9555を初期化し、全ピンを出力に設定する。
bool initPCA9555() {
    ioex.attach(Wire, PCA9555_ADDR);
    return ioex.polarity(PCA95x5::Polarity::ORIGINAL_ALL) &&
           ioex.direction(PCA95x5::Direction::OUT_ALL) &&
           ioex.write(PCA95x5::Level::L_ALL);
}

// 指定アドレスのデバイスがI2CでACKを返すか
bool i2cDevicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

// SDAがLOWに張り付いたまま（どれかのデバイスが送信途中で止まった）I2Cバスを解放する。
// SCLを最大9回手動で動かしてデバイスにデータを吐き出させ、最後にSTOP条件を出す。
void recoverI2CBus() {
    Wire.end();

    pinMode(PIN_WIRE0_SDA, INPUT_PULLUP);
    pinMode(PIN_WIRE0_SCL, OUTPUT);
    digitalWrite(PIN_WIRE0_SCL, HIGH);
    delayMicroseconds(10);
    for (uint8_t i = 0; i < 9 && digitalRead(PIN_WIRE0_SDA) == LOW; i++) {
        digitalWrite(PIN_WIRE0_SCL, LOW);
        delayMicroseconds(10);
        digitalWrite(PIN_WIRE0_SCL, HIGH);
        delayMicroseconds(10);
    }
    // STOP条件 (SCL=HIGHの間にSDAをLOW→HIGH)
    pinMode(PIN_WIRE0_SDA, OUTPUT);
    digitalWrite(PIN_WIRE0_SDA, LOW);
    delayMicroseconds(10);
    digitalWrite(PIN_WIRE0_SDA, HIGH);
    delayMicroseconds(10);

    Wire.begin();
    Wire.setTimeout(I2C_TIMEOUT_MS, true);
    //Wire.setClock(400000); // test/teensy-rp2040-uart の確認時は100kHzで動作
}

// PCA9555と使用する全VL53L0Xを初期化する。失敗したらfalse（停止はしない）。
bool initSensors(uint32_t boot_delay_ms) {
    if (!initPCA9555()) {
        Serial.print("Failed to initialize PCA9555 (I2C error: ");
        Serial.print(ioex.i2c_error());
        Serial.println(")");
        return false;
    }

    // 1. すべてのVL53をシャットダウン (XSHUT = LOW)。リセットされてアドレスが0x29に
    //    戻ったセンサーが残っていても、ここで全部止まるのでアドレスが衝突しない。
    if (!setPCA9555Outputs(PCA95x5::Level::L_ALL)) {
        Serial.println("Failed to shut down VL53L0X sensors.");
        return false;
    }
    delay(10);

    // 2. 使用するセンサーだけを1つずつ起動し、アドレスを再割り当て
    uint16_t active_sensor_mask = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (isUnusedSensor(i)) {
            distance_data[i] = 0;
            continue;
        }
        rp2040.wdt_reset();

        delay(boot_delay_ms);
        Serial.print("Initializing sensor ");
        Serial.println(i);

        // 初期化済みのセンサーもHIGHのまま維持する。
        active_sensor_mask |= static_cast<uint16_t>(1u << i);
        if (!setPCA9555Outputs(active_sensor_mask)) {
            Serial.print("Failed to enable sensor ");
            Serial.println(i);
            return false;
        }
        delay(boot_delay_ms);

        // 初期アドレス(0x29)でライブラリを初期化、新しいアドレスを設定
        if (!lox[i].begin(sensor_addresses[i], false, &Wire)) {
            Serial.print("Failed to boot VL53L0X #");
            Serial.println(i);
            return false;
        }

        // 連続測定モード
        lox[i].startRangeContinuous();
        last_measurement_ms[i] = millis();
    }

    Serial.println("All active sensors initialized successfully.");
    return true;
}

// 初期化が成功するまでI2Cバスの解放と再初期化を繰り返す。
void initSensorsWithRetry(uint32_t boot_delay_ms) {
    while (!initSensors(boot_delay_ms)) {
        Serial.println("Retrying sensor initialization...");
        rp2040.wdt_reset();
        recoverI2CBus();
        delay(100);
    }
    consecutive_failures = 0;
}

uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    return crc;
}

void sendDistanceFrame()
{
    // フレーム形式: 'V', 'L', 距離データ(12個 x 16 bit, little endian), CRC-8
    uint8_t payload[PAYLOAD_SIZE];
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
    {
        payload[i * 2] = static_cast<uint8_t>(distance_data[i] & 0xFF);
        payload[i * 2 + 1] = static_cast<uint8_t>(distance_data[i] >> 8);
    }

    Serial1.write(FRAME_HEADER, sizeof(FRAME_HEADER));
    Serial1.write(payload, sizeof(payload));
    Serial1.write(crc8(payload, sizeof(payload)));
    ++frames_sent;
}

void printStatus() {
    const uint32_t now = millis();
    if (now - last_status_print_ms < STATUS_PRINT_INTERVAL_MS) {
        return;
    }
    last_status_print_ms = now;

    Serial.print("frames=");
    Serial.print(frames_sent);
    Serial.print(" recoveries=");
    Serial.print(recovery_count);
    Serial.print(" failed_mask=0x");
    Serial.print(last_failed_mask, HEX);
    Serial.print(" dist:");
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        Serial.print(' ');
        Serial.print(distance_data[i]);
    }
    Serial.println();
}

void setup() {
    // デバッグ用USBシリアル（PC未接続でも待たない）
    Serial.begin(USB_SERIAL_BAUD);

    // Teensy通信用 (XIAO RP2040のSerial1 → Teensy RX8/TX8)
    Serial1.begin(TEENSY_SERIAL_BAUD);

    Wire.begin();
    Wire.setTimeout(I2C_TIMEOUT_MS, true);
    //Wire.setClock(400000); // test/teensy-rp2040-uart の確認時は100kHzで動作

    // 電源の瞬断などでRP2040が再起動した直後は、VL53が通信途中のままバスを
    // 掴んでいることがあるので、最初にバスを解放しておく。
    recoverI2CBus();

    rp2040.wdt_begin(WATCHDOG_TIMEOUT_MS);
    initSensorsWithRetry(SETUP_BOOT_DELAY_MS);
}

void loop() {
    rp2040.wdt_reset();

    const uint32_t now = millis();
    uint16_t failed_mask = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (isUnusedSensor(i)) {
            // 未使用センサー（ピンは物理的に抜け）
            distance_data[i] = 0;
            continue;
        }

        // ライブラリはI2Cエラーを返さないので、先に自分でACKを確認する。
        // センサーが電源の瞬断でリセットされるとアドレスが0x29に戻り、ここで応答しなくなる。
        if (!i2cDevicePresent(sensor_addresses[i])) {
            failed_mask |= static_cast<uint16_t>(1u << i);
            continue;
        }

        if (lox[i].isRangeComplete()) {
            distance_data[i] = lox[i].readRangeResult();
            last_measurement_ms[i] = now;
        } else if (now - last_measurement_ms[i] > STALE_MEASUREMENT_MS) {
            // 応答はするが連続測定が止まっている
            failed_mask |= static_cast<uint16_t>(1u << i);
        }
        // 測定が完了していない場合は前回の値を維持
    }
    if (Wire.getTimeoutFlag()) {
        Wire.clearTimeoutFlag();
        failed_mask |= 0x8000; // I2Cタイムアウト
    }

    last_failed_mask = failed_mask;
    if (failed_mask != 0) {
        ++consecutive_failures;
    } else {
        consecutive_failures = 0;
    }

    if (consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
        Serial.print("Sensor failure detected (mask=0x");
        Serial.print(failed_mask, HEX);
        Serial.println("). Reinitializing all sensors.");
        ++recovery_count;
        recoverI2CBus();
        // 0x29に戻ったセンサーがいると衝突するので、全センサーを再初期化する
        initSensorsWithRetry(RECOVERY_BOOT_DELAY_MS);
        return;
    }

    // to teensy
    sendDistanceFrame();
    printStatus();

    delay(50);
}
