import socket
import cv2
import numpy as np
import pickle
import struct
from ultralytics import YOLO

# Sunucu IP ve port bilgisi
client_ip = '192.168.1.16'  # Raspberry Pi'nin IP adresi
client_port = 5003

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((client_ip, client_port))

MESSAGE_PORT = 12345
PC_IP = "0.0.0.0"
print("Raspberry bağlantısı başarılı")

# Soket oluştur
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((PC_IP, MESSAGE_PORT))
server_socket.listen(5)

data_channel, address = server_socket.accept()

print(address)

print("Karşılıklı Bağlantı kuruldu.")

model = YOLO("best.pt")

X_MAX_ANGLE = 90
Y_MAX_ANGLE = 65

currentAngle = {"X": 0, "Y": -25}

reset = 0

def angle(height, width, frameXAngle, frameYAngle, currentAngle, image, reset):
    center_of_camera = {"X": width / 2, "Y": height / 2}
    water = 0
    results = model.predict(image)[0]
    print(reset)
    if reset == 300:
        currentAngle = {"X": 0, "Y": -25}
        reset = 0

    if results.boxes:
        # Tespit edilen nesneleri çerçeve üzerine çiz
        reset = 0
        for result in results[0].boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result
            if class_id == 0 and score >= 0.3:
                no_target = 0

                # Hedefi dikdörtgene al
                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                # Kameranın ortasına artı işareti çiz
                cv2.line(image, (int(width / 2) - 10, int(height / 2)),
                         (int(width / 2) + 10, int(height / 2)), (0, 255, 0), 2)
                cv2.line(image, (int(width / 2), int(height / 2) - 10),
                         (int(width / 2), int(height / 2) + 10), (0, 255, 0), 2)

                center_of_target = {"X": (x1 + x2) / 2, "Y": (y1 + y2) / 2}

                X_distance = center_of_camera["X"] - center_of_target["X"]
                Y_distance = center_of_target["Y"] - center_of_camera["Y"]

                dintance = {"X": X_distance, "Y": Y_distance}

                one_angle_for_pixel_X = frameXAngle / (center_of_camera["X"] / 2)
                one_angle_for_pixel_Y = frameYAngle / (center_of_camera["Y"] / 2)

                target_angle_X = one_angle_for_pixel_X * dintance["X"]
                target_angle_Y = one_angle_for_pixel_Y * dintance["Y"]

                target_angle = {"X": target_angle_X, "Y": target_angle_Y}
                currentAngle["X"] += target_angle["X"]
                currentAngle["Y"] += target_angle["Y"]
                if currentAngle["X"] >= X_MAX_ANGLE or currentAngle["X"] <= -X_MAX_ANGLE or currentAngle[
                    "Y"] >= Y_MAX_ANGLE or currentAngle["Y"] <= -Y_MAX_ANGLE:
                    currentAngle = {"X": 0, "Y": -25}
                if x1 < center_of_camera["X"] < x2 and y1 < center_of_camera["Y"] < y2:
                    water = 1
    else:
        reset += 1
    return currentAngle,water,reset


while True:

    # Görüntü alımı ve gösterimi
    length = client_socket.recv(16)
    string_data = b''

    # Veriyi tampona al
    while len(string_data) < int(length):
        string_data += client_socket.recv(int(length) - len(string_data))

    # Baytları görüntüye dönüştür
    data = np.frombuffer(string_data, dtype='uint8')

    # Görüntüyü yeniden şekillendir
    im = cv2.imdecode(data, cv2.COLOR_BGR2RGB)
    im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)

    height, width, sz = im.shape

    currentAngle , water,reset= angle(height, width, 10, 5, currentAngle, im,reset)

    cv2.imshow('Received Frame', im)

    nesne_bytes = pickle.dumps({"X": currentAngle["X"],"Y":currentAngle["Y"],"Water":water})
    data_channel.send(nesne_bytes)

    print("Çalışıyor")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

client_socket.close()
cv2.destroyAllWindows()
