#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <PCA95x5.h> // hideakitai/PCA95x5

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

constexpr uint8_t SENSOR_COUNT = 12;
constexpr uint8_t PCA9555_ADDR = 0x20;
constexpr uint8_t VL53L0X_DEFAULT_ADDR = 0x29;
constexpr uint32_t USB_SERIAL_BAUD = 115200;
constexpr uint32_t TEENSY_SERIAL_BAUD = 115200;
constexpr uint8_t FRAME_HEADER[] = {'V', 'L'};
// 状況メッセージのフレーム: 'M', 'G', 長さ(1〜MESSAGE_MAX_LENGTH), 文字列(長さ分), CRC-8(長さ+文字列)
// Teensy側 UpperVL53s が受け取ってデバッグ出力に流す。
constexpr uint8_t MESSAGE_HEADER[] = {'M', 'G'};
constexpr size_t MESSAGE_MAX_LENGTH = 200;
constexpr size_t PAYLOAD_SIZE = SENSOR_COUNT * sizeof(uint16_t);

// 起動時のXSHUT操作後の待ち時間。初回は動作確認済みの長めの値、
// 動作中の復旧では止まっている時間を短くするため短めの値を使う。
constexpr uint32_t SETUP_BOOT_DELAY_MS = 200;
constexpr uint32_t RECOVERY_BOOT_DELAY_MS = 20;

// I2Cが応答しないときに待つ上限。超えたらI2C周辺回路をリセットする。
constexpr uint32_t I2C_TIMEOUT_MS = 10;
// センサーが応答しない、または新しい測定値が来ない状態が続いたら全センサーを再初期化する。
constexpr uint8_t MAX_CONSECUTIVE_FAILURES = 3;
constexpr uint32_t STALE_MEASUREMENT_MS = 500;
// loop()が止まったらRP2040を再起動する（再起動後はsetup()で再初期化される）。
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 4000;
constexpr uint32_t STATUS_PRINT_INTERVAL_MS = 1000;
// 初期化に失敗したセンサーを、他のセンサーを止めずに単独で再初期化する間隔
constexpr uint32_t RETRY_FAILED_SENSOR_INTERVAL_MS = 1000;

// 測定値がないセンサーの値（Teensy側 UpperVL53s::NO_DATA と同じ値）。
// 0 は「壁が目の前」と区別できないので使わない。0xFFFF は VL53 の「範囲外」。
constexpr uint16_t NO_DATA = 0xFFFE;

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
bool sensor_ready[SENSOR_COUNT];      // 初期化に成功して測定中
uint8_t range_status[SENSOR_COUNT];   // 最後の測定のRangeStatus (0:正常 1:Sigma 2:Signal 3:MinRange 4:Phase)
uint32_t last_retry_ms = 0;
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

void sendDistanceFrame();
void logMessage(const char* format, ...);

uint16_t activeSensorMask() {
    uint16_t mask = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (sensor_ready[i]) {
            mask |= static_cast<uint16_t>(1u << i);
        }
    }
    return mask;
}

// XSHUT が LOW になっているセンサー i を起動して、アドレスを割り当てる。
// 他の初期化済みセンサーは固有アドレスにいるので、0x29 にいるのはこのセンサーだけになる。
bool bootSensor(uint8_t i, uint32_t boot_delay_ms) {
    rp2040.wdt_reset();
    logMessage("Initializing sensor %u", i);

    if (!setPCA9555Outputs(activeSensorMask() | static_cast<uint16_t>(1u << i))) {
        logMessage("Failed to enable sensor %u (PCA9555)", i);
        return false;
    }
    delay(boot_delay_ms);

    // 0x29 で応答しない（配線が外れている、電源が来ていない）ならすぐ諦める
    if (!i2cDevicePresent(VL53L0X_DEFAULT_ADDR)) {
        logMessage("VL53L0X #%u does not answer at 0x29 (配線・電源を確認)", i);
        setPCA9555Outputs(activeSensorMask());
        return false;
    }

    // 初期アドレス(0x29)でライブラリを初期化、新しいアドレスを設定
    if (!lox[i].begin(sensor_addresses[i], false, &Wire)) {
        logMessage("Failed to boot VL53L0X #%u (起動時の校正中に近くに物があると失敗することがある)", i);
        // 0x29 のまま残ると次のセンサーとアドレスが衝突するので、XSHUT を LOW に戻す
        setPCA9555Outputs(activeSensorMask());
        return false;
    }

    // 連続測定モード
    lox[i].startRangeContinuous();
    sensor_ready[i] = true;
    distance_data[i] = NO_DATA;
    last_measurement_ms[i] = millis();
    return true;
}

// PCA9555と使用する全VL53L0Xを初期化する。PCA9555が使えないときだけfalse。
// 一部のVL53が起動できなくても、起動できたセンサーで測定を始める（失敗分は loop() で再試行）。
bool initSensors(uint32_t boot_delay_ms) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        sensor_ready[i] = false;
        range_status[i] = 0;
        distance_data[i] = isUnusedSensor(i) ? 0 : NO_DATA;
    }

    if (!initPCA9555()) {
        logMessage("Failed to initialize PCA9555 (I2C error: %d)", static_cast<int>(ioex.i2c_error()));
        return false;
    }

    // 1. すべてのVL53をシャットダウン (XSHUT = LOW)。リセットされてアドレスが0x29に
    //    戻ったセンサーが残っていても、ここで全部止まるのでアドレスが衝突しない。
    if (!setPCA9555Outputs(PCA95x5::Level::L_ALL)) {
        logMessage("Failed to shut down VL53L0X sensors (PCA9555)");
        return false;
    }
    delay(10);

    // 2. 使用するセンサーだけを1つずつ起動し、アドレスを再割り当て
    uint8_t ready_count = 0;
    uint8_t active_count = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (isUnusedSensor(i)) {
            continue;
        }
        ++active_count;
        delay(boot_delay_ms);
        if (bootSensor(i, boot_delay_ms)) {
            ++ready_count;
        }
    }

    logMessage("Sensors initialized: %u / %u ready (mask=0x%03X)", ready_count, active_count, activeSensorMask());
    last_retry_ms = millis();
    return true;
}

// PCA9555の初期化が成功するまでI2Cバスの解放と再初期化を繰り返す。
// 待っている間も「データなし」のフレームを送り、RP2040が動いていることをTeensyに伝える。
void initSensorsWithRetry(uint32_t boot_delay_ms) {
    while (!initSensors(boot_delay_ms)) {
        logMessage("Retrying sensor initialization...");
        sendDistanceFrame();
        rp2040.wdt_reset();
        recoverI2CBus();
        delay(100);
    }
    consecutive_failures = 0;
}

// 起動に失敗したセンサーを1つずつ再初期化する（他のセンサーは止めない）。
void retryFailedSensors() {
    if (millis() - last_retry_ms < RETRY_FAILED_SENSOR_INTERVAL_MS) {
        return;
    }
    last_retry_ms = millis();
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (!isUnusedSensor(i) && !sensor_ready[i]) {
            bootSensor(i, RECOVERY_BOOT_DELAY_MS);
            return; // 1周期に1つだけ（送信が止まる時間を短くする）
        }
    }
}

uint8_t crc8(const uint8_t *data, size_t len);

// 状況メッセージをTeensyに送る
void sendMessageFrame(const char* text) {
    size_t length = strlen(text);
    if (length == 0) {
        return;
    }
    if (length > MESSAGE_MAX_LENGTH) {
        length = MESSAGE_MAX_LENGTH;
    }
    uint8_t frame[1 + MESSAGE_MAX_LENGTH];
    frame[0] = static_cast<uint8_t>(length);
    memcpy(frame + 1, text, length);

    Serial1.write(MESSAGE_HEADER, sizeof(MESSAGE_HEADER));
    Serial1.write(frame, 1 + length);
    Serial1.write(crc8(frame, 1 + length));
}

// printf形式のログ。USBシリアルに出し、同じ内容をTeensyにも送る。
void logMessage(const char* format, ...) {
    char text[MESSAGE_MAX_LENGTH + 1];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);
    va_end(args);

    Serial.println(text);
    sendMessageFrame(text);
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

const char* resetReasonName() {
    switch (rp2040.getResetReason()) {
    case RP2040::PWRON_RESET: return "power on";
    case RP2040::RUN_PIN_RESET: return "RUN pin";
    case RP2040::SOFT_RESET: return "soft reset";
    case RP2040::WDT_RESET: return "WATCHDOG (loop stopped)";
    case RP2040::DEBUG_RESET: return "debugger";
    case RP2040::GLITCH_RESET: return "glitch";
    case RP2040::BROWNOUT_RESET: return "BROWNOUT (電圧低下)";
    default: return "unknown";
    }
}

// 1秒ごとの状況。USBシリアルとTeensyの両方に送る。
// 例: "up=12s frames=200 recov=0 ready=0x64F fail=0x000 0:124/0 1:300/0 ..."
//   番号:距離/RangeStatus（0:正常 1:Sigma 2:Signal 3:MinRange 4:Phase）、NG は起動できていない
void printStatus() {
    const uint32_t now = millis();
    if (now - last_status_print_ms < STATUS_PRINT_INTERVAL_MS) {
        return;
    }
    last_status_print_ms = now;

    char text[MESSAGE_MAX_LENGTH + 1];
    size_t length = snprintf(text, sizeof(text), "up=%lus frames=%lu recov=%lu ready=0x%03X fail=0x%04X",
                             static_cast<unsigned long>(now / 1000), static_cast<unsigned long>(frames_sent),
                             static_cast<unsigned long>(recovery_count), activeSensorMask(), last_failed_mask);
    for (uint8_t i = 0; i < SENSOR_COUNT && length < sizeof(text); i++) {
        if (isUnusedSensor(i)) {
            continue;
        }
        if (!sensor_ready[i]) {
            length += snprintf(text + length, sizeof(text) - length, " %u:NG", i);
        } else {
            length += snprintf(text + length, sizeof(text) - length, " %u:%u/%u", i, distance_data[i], range_status[i]);
        }
    }
    Serial.println(text);
    sendMessageFrame(text);
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

    logMessage("RP2040 boot (reset reason: %s)", resetReasonName());

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
        if (!sensor_ready[i]) {
            distance_data[i] = NO_DATA; // 起動に失敗したセンサー（retryFailedSensors で再試行）
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
            range_status[i] = lox[i].readRangeStatus();
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
        logMessage("Sensor failure detected (fail=0x%04X, 0x8000=I2C timeout). Reinitializing all sensors.", failed_mask);
        ++recovery_count;
        recoverI2CBus();
        // 0x29に戻ったセンサーがいると衝突するので、全センサーを再初期化する
        initSensorsWithRetry(RECOVERY_BOOT_DELAY_MS);
        return;
    }

    // to teensy
    sendDistanceFrame();
    printStatus();
    retryFailedSensors();

    delay(50);
}
