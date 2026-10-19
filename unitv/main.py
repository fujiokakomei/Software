# Untitled - By: fujio - 月 10 19 2026

import sensor, image, lcd, time
import KPU as kpu
from machine import UART
import gc, sys


sensor.reset()                      # Reset and initialize the sensor. It will
                                    # run automatically, call sensor.run(0) to stop
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.

model_1_path = "/sd/model1.kmodel"
model_2_path = "/sd/model2.kmodel"



input_size = (224, 224)


labels_1 = ['ct', 'letter']
anchors_1 = [3.09, 3.06, 2.81, 2.66, 2.38, 2.31, 2.59, 1.69, 2.59, 0.97]

labels_2 = ['3', '1', '2']
anchors_2 = [2.19, 1.59, 1.36, 0.78, 3.91, 3.41, 2.62, 2.5, 1.19, 1.22]
