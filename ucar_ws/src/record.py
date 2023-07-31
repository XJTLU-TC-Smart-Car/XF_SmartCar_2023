import cv2
import time
import os

# 设定ROI区域的偏移量
x_offset = int(640 * 0.25)
y_offset_up = int(480 * 0.22)
y_offset_down = int(480 * 0.28)

# 设定保存地址
save_path = '/home/ucar/record_vid'

# 打开摄像头
cap = cv2.VideoCapture(0)

# 设定视频编码方式
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
timestamp = time.strftime("%Y%m%d-%H%M%S")
out = cv2.VideoWriter(os.path.join(save_path, f'{timestamp}.avi'), fourcc, 20.0, (640-2*x_offset,480-y_offset_up-y_offset_down))
while(cap.isOpened()):
    # 读取帧
    ret, frame = cap.read()
    if ret == True:
        # 创建一个矩形框来表示ROI区域
        frame = cv2.flip(frame, 1)
        roi = frame[y_offset_up:480-y_offset_down, x_offset:x_offset+640-2*x_offset]
        cv2.rectangle(frame, (x_offset, y_offset_up), (x_offset+640-2*x_offset, 480-y_offset_down), (0, 255, 0), 2)
        # 显示帧
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # 写入帧
        out.write(roi)
    else:
        break

# 释放摄像头和输出对象
cap.release()
out.release()

# 关闭所有OpenCV窗口
cv2.destroyAllWindows()