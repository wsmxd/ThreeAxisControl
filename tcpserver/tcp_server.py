import socket

# 1. 创建 TCP 服务端
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# 监听所有网卡，端口 8080 (必须和代码里填的一致)
server.bind(('0.0.0.0', 10000))
server.listen(1)
print("等待 ESP8266 连接...")

# 2. 等待客户端连接
client, addr = server.accept()
print(f"ESP8266 已连接！客户端地址: {addr}")

# 3. 发送指令循环
while True:
    cmd = input("\n请输入要发送的指令 (LED_ON / LED_OFF)，输入 exit 退出: ")
    if cmd.lower() == 'exit':
        break
    # 发送数据，加个回车换行更稳妥
    client.sendall((cmd + "\r\n").encode('utf-8'))
    print(f"已发送: {cmd}")

client.close()
server.close()