#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#include <Arduino.h>

constexpr uint8_t SENSOR_COUNT = 12;
constexpr uint8_t PCA9555_ADDR = 0x20; // PCA9555のI2Cアドレス（設定に合わせて変更してください）
constexpr uint8_t FRAME_HEADER[] = {'V', 'L'};
constexpr size_t PAYLOAD_SIZE = SENSOR_COUNT * sizeof(uint16_t);

// VL53L0Xオブジェクトの配列
Adafruit_VL53L0X lox[SENSOR_COUNT];

// 各センサーに割り当てる一意のI2Cアドレス (初期値0x29以外の衝突しないアドレス)
const uint8_t sensor_addresses[SENSOR_COUNT] = {
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B
};

// 距離データを格納する配列
uint16_t distance_data[SENSOR_COUNT];

// PCA9555の出力を設定する関数 (16ビット分)
void setPCA9555Outputs(uint16_t pin_states) {
    Wire.beginTransmission(PCA9555_ADDR);
    Wire.write(0x02); // Output Port 0 レジスタアドレス
    Wire.write(pin_states & 0xFF);        // Port 0 (ビット0-7)
    Wire.write((pin_states >> 8) & 0xFF); // Port 1 (ビット8-15)
    Wire.endTransmission();
}

// PCA9555のピン方向を「出力」に設定する関数
void initPCA9555() {
    Wire.beginTransmission(PCA9555_ADDR);
    Wire.write(0x06); // Configuration Port 0 レジスタアドレス
    Wire.write(0x00); // すべて出力設定 (Port 0)
    Wire.write(0x00); // すべて出力設定 (Port 1)
    Wire.endTransmission();
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
    // デバッグ用USBシリアル
    Serial.begin(115200);
    
    // 相手側マイコン通信用シリアル (XIAO RP2040のSerial1: TX=D7, RX=D8)
    Serial1.begin(115200); 

    // I2Cバスの初期化 (標準ピンを使用)
    Wire.begin();
    Wire.setClock(400000); // I2C高速モード (400kHz)

    Serial.println("Initializing PCA9555...");
    initPCA9555();

    // 1. すべてのVL53をシャットダウン (XSHUT = LOW)
    // PCA9555のピン0〜11にXSHUTが接続されていると仮定
    setPCA9555Outputs(0x0000); 
    delay(10);

    // 2. 1つずつセンサーを起動し、アドレスを再割り当て
    for (int i = 0; i < SENSOR_COUNT; i++) {
        Serial.print("Initializing sensor ");
        Serial.println(i);

        // 対象のセンサーのXSHUTだけをHIGH
        // (例: i=0 のときは 0x0001, i=1 のときは 0x0003 と累積させていく)
        uint16_t current_mask = 0;
        for(int j = 0; j <= i; j++) {
            current_mask |= (1 << j);
        }
        setPCA9555Outputs(current_mask);
        delay(10); // 起動を待つ

        // 初期アドレス(0x29)でライブラリを初期化、新しいアドレスを設定
        if (!lox[i].begin(sensor_addresses[i], false, &Wire)) {
            Serial.print("Failed to boot VL53L0X #");
            Serial.println(i);
            while (1); // 起動失敗時は停止
        }
        
        // 連続測定モードを開始
        lox[i].startRangeContinuous();
    }

    Serial.println("All sensors initialized successfully.");
}

void loop() {
    // 各センサーからデータを取得
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (lox[i].isRangeComplete()) {
            distance_data[i] = lox[i].readRangeResult();
        } else {
            // 測定が完了していない場合は前回の値を維持、またはエラー値
            // distance_data[i] = 0; 
        }
    }

    // 別のマイコンへ距離データをフレーム化して送信
    sendDistanceFrame();

    // デバッグ用にUSBシリアルにも表示
    for (int i = 0; i < SENSOR_COUNT; i++) {
        Serial.print(distance_data[i]);
        Serial.print("\t");
    }
    Serial.println();

    delay(50); // 送信頻度の調整 (ミリ秒)
}
