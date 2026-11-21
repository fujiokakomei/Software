#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <PCA95x5.h>

#include <Arduino.h>

constexpr uint8_t SENSOR_COUNT = 12;
constexpr uint8_t PCA9555_ADDR = 0x20; // PCA9555のI2Cアドレス（設定に合わせて変更してください）
constexpr uint32_t USB_SERIAL_BAUD = 115200;
constexpr uint32_t USB_SERIAL_WAIT_MS = 2000;

// hideakitai/PCA95x5ライブラリでPCA9555を制御する。
PCA9555 ioex;

// 使用しないVL53L0Xの番号（0始まり）をここに記入する。
// 例: センサー2番と7番を使用しない場合は {2, 7} に変更する。
// 未使用センサーがない場合は、範囲外のSENSOR_COUNTを残す。
constexpr uint8_t UNUSED_SENSOR_NUMBERS[] = {SENSOR_COUNT};

// VL53L0Xオブジェクトの配列
Adafruit_VL53L0X lox[SENSOR_COUNT];

// 各センサーに割り当てる一意のI2Cアドレス (初期値0x29以外の衝突しないアドレス)
const uint8_t sensor_addresses[SENSOR_COUNT] = {
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B
};

// 距離データを格納する配列
uint16_t distance_data[SENSOR_COUNT];

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

// PCへUSBシリアルで送るテスト用のCSV形式:
// VL,<sensor 0の距離>,...,<sensor 11の距離>\n
void sendDistanceLine()
{
    Serial.print(F("VL"));
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        Serial.print(',');
        Serial.print(distance_data[i]);
    }
    Serial.println();
}

void setup() {
    // PCとのUSBシリアル通信。ボーレートはPC側の設定用で、USB通信速度自体には影響しない。
    Serial.begin(USB_SERIAL_BAUD);
    const uint32_t serialWaitStartedAt = millis();
    while (!Serial && millis() - serialWaitStartedAt < USB_SERIAL_WAIT_MS) {
    }
    Serial.println(F("RP2040 VL53L0X USB test started"));
    Serial.println(F("Format: VL,<sensor0>,...,<sensor11>"));

    // I2Cバスの初期化 (標準ピンを使用)
    Wire.begin();
    //Wire.setClock(400000); // I2C高速モード (400kHz)

    Serial.println("Initializing PCA9555...");
    if (!initPCA9555()) {
        Serial.print("Failed to initialize PCA9555 (I2C error: ");
        Serial.print(ioex.i2c_error());
        Serial.println(")");
        while (1);
    }

    // 1. すべてのVL53をシャットダウン (XSHUT = LOW)
    // PCA9555のピン0〜11にXSHUTが接続されていると仮定
    if (!setPCA9555Outputs(PCA95x5::Level::L_ALL)) {
        Serial.println("Failed to shut down VL53L0X sensors.");
        while (1);
    }
    delay(10);

    // 2. 使用するセンサーだけを1つずつ起動し、アドレスを再割り当て
    uint16_t active_sensor_mask = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (isUnusedSensor(i)) {
            // 未使用センサーはXSHUTをLOWのままにし、初期化しない。
            distance_data[i] = 0;
            continue;
        }

        delay(1000);
        Serial.print("Initializing sensor ");
        Serial.println(i);

        // 対象のセンサーのXSHUTだけをHIGH
        // 初期化済みの使用センサーもHIGHのまま維持する。
        active_sensor_mask |= static_cast<uint16_t>(1u << i);
        if (!setPCA9555Outputs(active_sensor_mask)) {
            Serial.print("Failed to enable sensor ");
            Serial.println(i);
            while (1);
        }
        delay(1000); // 起動を待つ

        // 初期アドレス(0x29)でライブラリを初期化、新しいアドレスを設定
        if (!lox[i].begin(sensor_addresses[i], false, &Wire)) {
            Serial.print("Failed to boot VL53L0X #");
            Serial.println(i);
            while (1); // 起動失敗時は停止
        }
        
        // 連続測定モードを開始
        lox[i].startRangeContinuous();
    }

    Serial.println("All active sensors initialized successfully.");
}

void loop() {
    // 各センサーからデータを取得
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (isUnusedSensor(i)) {
            // 未使用センサー分には固定値を送る。
            distance_data[i] = 0;
            continue;
        }

        if (lox[i].isRangeComplete()) {
            distance_data[i] = lox[i].readRangeResult();
        } else {
            // 測定が完了していない場合は前回の値を維持、またはエラー値
            // distance_data[i] = 0; 
        }
    }

    // PCへ距離データをUSBシリアルで送信
    sendDistanceLine();

    delay(50); // 送信頻度の調整 (ミリ秒)
}
