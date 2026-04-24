import serial
import struct
import threading
import time
import sys

# --- 协议边界与常量 ---
SOF = 0x7B  # '{'
EOF = 0x7D  # '}'
ESC = 0x25  # '%'
XOR = 0x20
CMD_ECHO = 0x99

def calculate_crc16_ccitt(data: bytes) -> int:
    """与单片机端完全对齐的 CRC16-CCITT 算法"""
    crc = 0xFFFF
    for byte in data:
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ byte) & 0xFF]
        crc &= 0xFFFF
    return crc

# 提前生成与 C 完全一致的查表 (利用 Python 动态生成，省去硬编码)
crc16_table = []
for i in range(256):
    crc = i << 8
    for _ in range(8):
        if crc & 0x8000:
            crc = (crc << 1) ^ 0x1021
        else:
            crc = crc << 1
        crc &= 0xFFFF
    crc16_table.append(crc)

class HermesDatalink:
    def __init__(self, port, baudrate=921600, local_id=0xFF):
        # 增加串口异常捕获与显式的 8N1 设置
        try:
            self.serial = serial.Serial(
                port=port, 
                baudrate=baudrate, 
                bytesize=serial.EIGHTBITS, 
                parity=serial.PARITY_NONE, 
                stopbits=serial.STOPBITS_ONE, 
                timeout=0.1
            )
            print(f"[Hermes] 成功打开串口: {port}, 波特率: {baudrate}")
        except serial.SerialException as e:
            print(f"\n[错误] 无法打开串口 {port}。请检查:")
            print("  1. 串口线是否插好？")
            print("  2. 该端口号是否被其他串口助手占用？")
            print(f"  系统详细报错: {e}\n")
            sys.exit(1)

        self.local_id = local_id
        
        # 注册回调函数
        self.on_receive = None
        self.on_bypass = None
        
        # 启动后台 RX 线程
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_task, daemon=True)
        self.rx_thread.start()

    def send_frame(self, target_id: int, cmd: int, payload: bytes):
        """发送一帧数据（封装、转义、追加 CRC）"""
        # 1. Header 打包: ID(1) + CMD(1) + LEN(2)
        raw_hdr = struct.pack('<BBH', target_id, cmd, len(payload))
        h_crc = calculate_crc16_ccitt(raw_hdr)
        raw_hdr += struct.pack('<H', h_crc) # 追加 Header CRC

        # 2. Header 转义处理
        tx_buf = bytearray([SOF])
        for b in raw_hdr:
            if b in (SOF, EOF, ESC):
                tx_buf.append(ESC)
                tx_buf.append(b ^ XOR)
            else:
                tx_buf.append(b)
        tx_buf.append(EOF)

        # 3. Payload 盲发 (不转义!)
        if len(payload) > 0:
            tx_buf.extend(payload)
            p_crc = calculate_crc16_ccitt(payload)
            tx_buf.extend(struct.pack('<H', p_crc))
        else:
            tx_buf.extend(b'\x00\x00')

        # 发送物理层数据
        self.serial.write(tx_buf)

    def _rx_task(self):
        """后台驻留线程，一模一样的 3 态接收 FSM"""
        STATE_WAIT, STATE_HDR, STATE_ESC, STATE_PLD = 0, 1, 2, 3
        state = STATE_WAIT
        
        hdr_buf = bytearray()
        pld_buf = bytearray()
        expected_len = 0
        current_cmd = 0
        current_target = 0

        while self.running:
            try:
                raw_bytes = self.serial.read(self.serial.in_waiting or 1)
            except serial.SerialException:
                # 防止在运行中途拔掉串口线导致崩溃
                print("\n[错误] 串口连接异常断开！")
                self.running = False
                break
                
            if not raw_bytes: continue
            
            for byte in raw_bytes:
                # 铁律：看到 SOF 就强行重启 Header 解析
                if byte == SOF:
                    state = STATE_HDR
                    hdr_buf.clear()
                    continue

                if state == STATE_WAIT:
                    if self.on_bypass: self.on_bypass(byte)
                    continue

                elif state == STATE_HDR:
                    if byte == ESC:
                        state = STATE_ESC
                    elif byte == EOF:
                        # Header 解析结算
                        if len(hdr_buf) != 6:
                            state = STATE_WAIT; continue
                            
                        # 校验 Header CRC (前4字节 vs 后2字节)
                        h_crc_rcv = struct.unpack('<H', hdr_buf[4:6])[0]
                        h_crc_calc = calculate_crc16_ccitt(hdr_buf[0:4])
                        if h_crc_rcv != h_crc_calc:
                            print(f"[Hermes] Header CRC Error!")
                            state = STATE_WAIT; continue

                        current_target, current_cmd, expected_len = struct.unpack('<BBH', hdr_buf[0:4])
                        state = STATE_PLD
                        pld_buf.clear()
                    else:
                        hdr_buf.append(byte)

                elif state == STATE_ESC:
                    hdr_buf.append(byte ^ XOR)
                    state = STATE_HDR

                elif state == STATE_PLD:
                    pld_buf.append(byte)
                    if len(pld_buf) == expected_len + 2:
                        # 校验 Payload CRC
                        actual_pld = pld_buf[:-2]
                        p_crc_rcv = struct.unpack('<H', pld_buf[-2:])[0]
                        p_crc_calc = calculate_crc16_ccitt(actual_pld)
                        
                        if p_crc_calc == p_crc_rcv:
                            if self.on_receive:
                                self.on_receive(current_target, current_cmd, actual_pld)
                        else:
                            print(f"[Hermes] Payload CRC Error!")
                            
                        state = STATE_WAIT

    def close(self):
        self.running = False
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()

# ================= 测试脚本 =================
if __name__ == "__main__":
    
    # 默认使用 COM8，允许通过命令行参数覆盖 (如: python hermes.py COM3)
    port_name = "COM8" if len(sys.argv) < 2 else sys.argv[1]
    target_baudrate = 921600
    
    # 初始化 Hermes
    hermes = HermesDatalink(port=port_name, baudrate=target_baudrate)

    def my_rx_callback(target_id, cmd, payload):
        if cmd == CMD_ECHO:
            print(f">>> [MCU 回显成功] 收到了: {payload.decode('utf-8', errors='replace')}")
        else:
            print(f">>> [收到数据] CMD: 0x{cmd:02X}, Data: {payload.hex()}")

    hermes.on_receive = my_rx_callback

    print("=======================================")
    print(" Hermes Datalink 调试终端启动")
    print("=======================================")
    
    try:
        while True:
            msg = input("请输入要发送给单片机的文本 (输入 'q' 退出): ")
            if msg.lower() == 'q':
                break
            
            # 以 CMD=0x99 发送一串文本给下位机
            payload_data = msg.encode('utf-8')
            hermes.send_frame(target_id=0x01, cmd=CMD_ECHO, payload=payload_data)
            print(f"<<< [已发送] {len(payload_data)} Bytes")
            
            time.sleep(0.1) # 稍作等待，让接收线程打印日志
            
    except KeyboardInterrupt:
        pass
    finally:
        hermes.close()
        print("已关闭连接。")