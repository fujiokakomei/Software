"""UnitV AI Camera のカメラ・UART 疎通テスト。

MaixPy IDE からこのファイルを UnitV に保存して実行する。
シリアルモニタは 115200 bps で開くこと。起動後に
``UNITV_CAMERA_TEST_READY`` が届き、以降は約 1 秒ごとに
フレームサイズ・中心画素の RGB 値・FPS を送る。

受信した改行区切りの文字列は ``RX: ...`` として返すため、
接続先から ``ping\\n`` を送ると双方向通信も確認できる。
"""

import sensor
import time
from machine import UART
from fpioa_manager import fm


# unitv/main.py と同じ UART を使う。配線: UnitV TX(35) -> 接続先 RX,
# UnitV RX(34) <- 接続先 TX, GND 同士を共通にする。
UART_RX_PIN = 34
UART_TX_PIN = 35
UART_BAUDRATE = 115200
STATUS_INTERVAL_FRAMES = 30


def send_line(text):
    """USB シリアルと外部マイコン用 UART の両方へ状態を出す。"""
    print(text)
    uart.write(text + "\n")


def handle_received_data(rx_buffer):
    """UART の受信バッファを改行単位で処理して、残りを返す。"""
    data = uart.read()
    if data:
        rx_buffer += data

    while b"\n" in rx_buffer:
        line, rx_buffer = rx_buffer.split(b"\n", 1)
        line = line.strip()
        if line:
            try:
                text = line.decode("utf-8")
            except UnicodeError:
                text = "<non-utf8>"
            send_line("RX: " + text)
    return rx_buffer


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

fm.register(UART_RX_PIN, fm.fpioa.UART1_RX, force=True)
fm.register(UART_TX_PIN, fm.fpioa.UART1_TX, force=True)
uart = UART(
    UART.UART1,
    UART_BAUDRATE,
    8,
    0,
    1,
    timeout=10,
    read_buf_len=4096,
)

clock = time.clock()
frame_count = 0
rx_buffer = b""
send_line("UNITV_CAMERA_TEST_READY")

while True:
    clock.tick()
    img = sensor.snapshot()

    # 描画前に中心画素を読む。これで実際にカメラが捉えた色を報告できる。
    center_x = img.width() // 2
    center_y = img.height() // 2
    pixel = img.get_pixel(center_x, center_y)

    # 画面中央に十字を描画する。画像が連続して取得できているかを
    # IDE の Frame Buffer でもすぐに確認できる。
    img.draw_cross(center_x, center_y, color=(0, 255, 0))

    rx_buffer = handle_received_data(rx_buffer)

    frame_count += 1
    if frame_count % STATUS_INTERVAL_FRAMES == 0:
        status = "CAM frame={} size={}x{} center_rgb={},{},{} fps={:.1f}".format(
            frame_count,
            img.width(),
            img.height(),
            pixel[0],
            pixel[1],
            pixel[2],
            clock.fps(),
        )
        send_line(status)
