# Untitled - By: fujio - 月 10 19 2026

import sensor, image, time
import KPU as kpu
from machine import UART
from fpioa_manager import fm
import gc, sys, ct_reader 


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((224, 224))
sensor.skip_frames(time = 2000) # 起動直後の不安定な状態をスキップ


fm.register(34, fm.fpioa.UART1_RX, force=True)
uart = UART(
    UART.UART1,
    115200,
    8,
    0,
    1,
    timeout=1000,
    read_buf_len=4096
)

buffer = b""


model_1_path = "/sd/model1.kmodel"
model_2_path = "/sd/model2.kmodel"



input_size = (224, 224)


labels_1 = ['ct', 'letter']
anchors_1 = [3.09, 3.06, 2.81, 2.66, 2.38, 2.31, 2.59, 1.69, 2.59, 0.97]

# Unharmed: 2 (deploy 0)
# Stable: 3 (deploy 1)
# Harmed: 1 (deploy 2)
labels_2 = ['3', '1', '2']
anchors_2 = [2.19, 1.59, 1.36, 0.78, 3.91, 3.41, 2.62, 2.5, 1.19, 1.22]


def switch_model(num):
    global task
    if num == 1:
        kpu.deinit(task)
        task = kpu.load(model_1_path)
        kpu.init_yolo2(task, 0.5, 0.3, 5, anchors_1)
    elif num == 2:
        kpu.deinit(task)
        task = kpu.load(model_2_path)
        kpu.init_yolo2(task, 0.5, 0.3, 5, anchors_2)
    else:
        print("model not found")



frame_count = 0
now_mode = "A"

last_found = None
last_rect = None  

task = kpu.load(model_1_path)
kpu.init_yolo2(task, 0.5, 0.3, 5, anchors_1)


while True:
    try:
        data = uart.read()
        if data: # 受信チェック
            buffer += data
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                # CRLF（\r\n）にも対応
                line = line.strip()
                if line:
                    text = line.decode("utf-8")
                    if text == "mode a":
                        now_mode = "A"
                        switch_model(1)
                        print("mode a")
                    elif text == "mode b":
                        now_mode = "B"
                        switch_model(2)
                        print("mode b")
                    elif text == "mode c":
                        now_mode = "C"
                        print("mode c")



        if now_mode == "A": # モードA
            img = sensor.snapshot()
            if frame_count % 2 == 0:
                objects = kpu.run_yolo2(task, img)

                if objects:
                    for obj in objects:
                        img.draw_rectangle(obj.rect())
                        label_text = labels_1[obj.classid()]
                        score = round(obj.value(), 2)

                        # ログのテキスト作成（例: "m, 0.68"）
                        log_line = str(label_text) + ", " + str(score)
                        print(log_line)

                        x, y, w, h = obj.rect()
                        cx = x + w / 2
                        cy = y + h / 2

                        if 100 < cx and cx < 124:
                            uart.write("found\n")
                            last_found, last_rect = label_text, (x, y, w, h) 
                            # Teensy から mode b が返るまで同じ検出を通知しない。
                            now_mode = "WAIT_B"
                            break
                #frame_count += 1
        elif now_mode == "WAIT_B":
            # found を一度送った後の応答待ち。UART 受信処理で mode b に遷移する。
            pass
        elif now_mode == "B":
            if last_found == "letter": #文字でモードBに入ったとき
                results = []
                for i in range(3):
                    img = sensor.snapshot()
                    objects = kpu.run_yolo2(task, img)
                    frame_result = None
                    frame_score = 0
                    if objects:
                        for obj in objects:
                            img.draw_rectangle(obj.rect())
                            label_text = labels_2[obj.classid()]
                            score = round(obj.value(), 2)

                            # ログのテキスト作成（例: "m, 0.68"）
                            log_line = str(label_text) + ", " + str(score)
                            print(log_line)

                            # 各フレームで最も信頼度が高い候補だけを採用する。
                            if score > 0.7 and score > frame_score:
                                frame_result = label_text
                                frame_score = score
                    results.append(frame_result)

                # 3フレームすべてで同じラベルを検出できた場合だけ判定する。
                if (None not in results and
                        results[0] == results[1] == results[2]):
                    if results[0] == "2":
                        uart.write("0\n") #投下数0
                    elif results[0] == "3":
                        uart.write("1\n")
                    elif results[0] == "1":
                        uart.write("2\n")
                else:
                    uart.write("-1\n") #被災者違ったわ
            else: #Cognitive TargetでモードBに入った時
                uart.write(str(ct_reader.decide(sensor.snapshot, last_rect)) + "\n")

        elif now_mode == "C":
            pass



        frame_count += 1

    except Exception as e:
        print("Error:", e)
