#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <PCA95x5.h> // hideakitai/PCA95x5

#include <Arduino.h>

constexpr uint8_t SENSOR_COUNT = 12;
constexpr uint8_t PCA9555_ADDR = 0x20;
constexpr uint32_t USB_SERIAL_BAUD = 115200;
constexpr uint32_t TEENSY_SERIAL_BAUD = 115200;
constexpr uint8_t FRAME_HEADER[] = {'V', 'L'};
constexpr size_t PAYLOAD_SIZE = SENSOR_COUNT * sizeof(uint16_t);

PCA9555 ioex;

// 使用しないVL53L0Xの番号（0始まり）。未使用がない場合は範囲外のSENSOR_COUNTを残す。
constexpr uint8_t UNUSED_SENSOR_NUMBERS[] = {4,5,7,8}; //未使用なし

// VL53L0Xオブジェクトの配列
Adafruit_VL53L0X lox[SENSOR_COUNT];

// 各センサーに割り当てる一意のI2Cアドレス (初期値0x29以外の衝突しないアドレス)
const uint8_t sensor_addresses[SENSOR_COUNT] = {
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B
};

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

// 初期化失敗時は停止する。デバッグ文字列はTeensy側のフレーム受信を
// 乱さないようにUSBシリアル（Serial）にだけ出す。
void haltWithMessage(const char* message) {
    Serial.println(message);
    while (1) {
        delay(1000);
    }
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
}

void setup() {
    // デバッグ用USBシリアル（PC未接続でも待たない）
    Serial.begin(USB_SERIAL_BAUD);

    // Teensy通信用 (XIAO RP2040のSerial1 → Teensy RX8/TX8)
    Serial1.begin(TEENSY_SERIAL_BAUD);

    Wire.begin();
    //Wire.setClock(400000); // test/teensy-rp2040-uart の確認時は100kHzで動作

    if (!initPCA9555()) {
        Serial.print("Failed to initialize PCA9555 (I2C error: ");
        Serial.print(ioex.i2c_error());
        Serial.println(")");
        haltWithMessage("halted");
    }

    // 1. すべてのVL53をシャットダウン (XSHUT = LOW)
    if (!setPCA9555Outputs(PCA95x5::Level::L_ALL))
    {
        haltWithMessage("Failed to shut down VL53L0X sensors.");
    }
    delay(10);

    // 2. 使用するセンサーだけを1つずつ起動し、アドレスを再割り当て
    uint16_t active_sensor_mask = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        if (isUnusedSensor(i))
        {
            distance_data[i] = 0;
            continue;
        }

        delay(500);
        Serial.print("Initializing sensor ");
        Serial.println(i);

        // 初期化済みのセンサーもHIGHのまま維持する。
        active_sensor_mask |= static_cast<uint16_t>(1u << i);
        if (!setPCA9555Outputs(active_sensor_mask))
        {
            Serial.print("Failed to enable sensor ");
            Serial.println(i);
            haltWithMessage("halted");
        }
        delay(500);

        // 初期アドレス(0x29)でライブラリを初期化、新しいアドレスを設定
        if (!lox[i].begin(sensor_addresses[i], false, &Wire))
        {
            Serial.print("Failed to boot VL53L0X #");
            Serial.println(i);
            haltWithMessage("halted");
        }

        // 連続測定モード
        lox[i].startRangeContinuous();
    }

    Serial.println("All active sensors initialized successfully.");
}

void loop() {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (isUnusedSensor(i)) {
            // 未使用センサー（ピンは物理的に抜け）
            distance_data[i] = 0;
            continue;
        }

        if (lox[i].isRangeComplete()) {
            distance_data[i] = lox[i].readRangeResult();
        }
        // 測定が完了していない場合は前回の値を維持
    }

    // to teensy
    sendDistanceFrame();

    delay(50);
}
