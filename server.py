import socket
import json
import time
import enum
import base64
import numpy as np
import cv2


class FrameState(enum.Enum):
    ACK = 0
    THIS_FRAME_OK = 1
    THIS_FRAME_FAIL = 2
    CALIB_OK = 3


def base64_to_cv2_image(base64_string, is_depth=False):
    img_data = base64.b64decode(base64_string)

    nparr = np.frombuffer(img_data, np.uint8 if not is_depth else np.uint16)

    flags = cv2.IMREAD_COLOR if not is_depth else cv2.IMREAD_UNCHANGED
    img = cv2.imdecode(nparr, flags)

    return img


def server_program():
    host = socket.gethostname()
    port = 10066
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind(('0.0.0.0', port))
        server_socket.listen(5)
        print(f"Server listening on {host}:{port}")

        while True:
            saved_img = False
            # 建立客户端连接
            client_socket, addr = server_socket.accept()
            print(f"Got connection from {addr}")
            while True:
                try:
                    data = b''
                    data_len_raw = client_socket.recv(1024).decode()
                    if not data_len_raw:
                        raise ConnectionResetError(
                            "Client disconnected.Can't get data length")
                    client_socket.send(data_len_raw.encode())

                    data_len = int(data_len_raw)

                    while True:
                        # 接收剩余的数据
                        chunk = client_socket.recv(32768)
                        if not chunk:
                            raise ConnectionResetError(
                                "Client disconnected.Can't get data")
                        data += chunk
                        print(f"reading data: |{len(data)}/{data_len}|")
                        if len(data) == int(data_len):
                            print("Data received")
                            break

                    data = data.decode()  # 解码数据
                    try:
                        # 尝试将接收到的数据转换为字典（假设它是有效的JSON格式）
                        parsed_data = json.loads(data)
                        # print("Parsed JSON:", parsed_data)  # 打印解析后的数据
                        response = str(
                            FrameState.THIS_FRAME_OK.value)  # 构建响应数据
                        print(response)
                        client_socket.send(response.encode())  # 发送响应
                        if 'img' in parsed_data:
                            img_base64 = parsed_data['img']
                            # depth_base64 = parsed_data['depth']

                            # 转换到图像格式
                            image = base64_to_cv2_image(img_base64)
                            # depth_image = base64_to_cv2_image(depth_base64, is_depth=True)
                            if not saved_img:
                                cv2.imwrite('image.jpg', image)
                                # cv2.imwrite('depth.png', depth_image)
                                saved_img = True
                                print("Image saved")
                    except json.JSONDecodeError as e:
                        print("Invalid JSON received:", e)
                except ConnectionResetError as e:
                    print("Client disconnected:", e)
                    client_socket.close()
                    break


if __name__ == '__main__':
    server_program()
