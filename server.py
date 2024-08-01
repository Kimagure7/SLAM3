import socket

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
                
                print("Received:", data)
                
        except Exception as e:
            print("Error occurred:", e)
        
        finally:
            client_socket.close()

if __name__ == '__main__':
    server_program()
