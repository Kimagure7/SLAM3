import socket
import json

def server_program():
    # 获取主机名
    host = socket.gethostname()
    # 设置端口号
    port = 10066  # 使用你想要的端口号
    # 创建socket对象
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 绑定端口
    server_socket.bind(('0.0.0.0', port))
    # 设置最大连接数，超过后排队
    server_socket.listen(5)
    print(f"Server listening on {host}:{port}")
    while True:
        # 建立客户端连接
        client_socket, addr = server_socket.accept()
        print(f"Got connection from {addr}")
        
        try:
            while True:
                # 接收数据（1024表示每次接收的最大字节数）
                data = client_socket.recv(1024).decode()
                if not data:
                    break
                
                try:
                    # 尝试将接收到的数据转换为字典（假设它是有效的JSON格式）
                    parsed_data = json.loads(data)
                    print("Parsed JSON:", parsed_data)  # 打印解析后的数据
                    if 'frame_count' in parsed_data:  # 检查'frame_count'字段是否存在
                        frame_count = parsed_data['frame_count']
                        
                        response = {'received_frame_count': frame_count}  # 构建响应数据
                        
                        client_socket.send(json.dumps(response).encode())  # 发送响应
                        
                        print("Sent frame count:", frame_count)
                    
                    else:
                        print("No 'frame_count' field in received JSON")
                    
                except json.JSONDecodeError as e:
                    print("Invalid JSON received:", e)
                
        except Exception as e:
            print("Error occurred:", e)
        
        finally:
            client_socket.close()

if __name__ == '__main__':
    server_program()
