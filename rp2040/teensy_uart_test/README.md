# Teensy 4.1 / XIAO RP2040 UART 通信テスト

このスケッチは VL53 や I2C を初期化せず、Teensy 4.1 と XIAO RP2040 の UART を双方向に確認するためのものです。

## 配線

両ボードは **3.3 V ロジック**で接続します。USB はそれぞれの書き込み・ログ確認用に接続してください。

| 接続元 | 接続先 |
| --- | --- |
| XIAO RP2040 D7 / TX | Teensy 4.1 pin 34 / RX8 |
| XIAO RP2040 D8 / RX | Teensy 4.1 pin 35 / TX8 |
| XIAO RP2040 GND | Teensy 4.1 GND |

TX と RX は必ず交差させます。5 V 信号は接続しません。

## 書き込みと確認

1. Arduino IDE で `teensy_uart_test.ino` を開き、XIAO RP2040 向けに書き込む。
2. `teensy4_1` ディレクトリで `pio run -e teensy41 --target upload` を実行し、Teensy 4.1 に書き込む。
3. 両方の USB シリアルモニタを 115200 baud で開く。

Teensy は毎秒 `PING,<連番>` を送ります。RP2040 は同じ連番の `PONG,<連番>,<RP2040のmillis>` を返します。Teensy 側に `RX PONG ... RTT=... ms` が連続表示されれば、両方向の通信が成功です。`TIMEOUT` が出る場合は TX/RX の交差、GND 共通、ボーレートを確認してください。
