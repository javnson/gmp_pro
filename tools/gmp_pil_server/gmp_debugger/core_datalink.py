import serial
import struct
import threading
from PyQt5.QtCore import QObject, pyqtSignal

# --- 协议边界与常量 ---
SOF = 0x7B
EOF = 0x7D
ESC = 0x25
XOR = 0x20

# 查表生成
crc16_table = []
for i in range(256):
    crc = i << 8
    for _ in range(8):
        if crc & 0x8000: crc = (crc << 1) ^ 0x1021
        else:            crc = crc << 1
        crc &= 0xFFFF
    crc16_table.append(crc)

def calculate_crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ byte) & 0xFF]
        crc &= 0xFFFF
    return crc

class HermesDatalinkQt(QObject):
    sig_frame_received = pyqtSignal(int, int, bytes)
    sig_log_msg = pyqtSignal(str)
    # 【新增】连接状态同步信号，True为已连接，False为断开
    sig_conn_state = pyqtSignal(bool) 

    def __init__(self, local_id=0xFF):
        super().__init__()
        self.serial = serial.Serial()
        self.local_id = local_id
        self.running = False
        self.rx_thread = None

    def connect_serial(self, port, baudrate, bytesize, parity, stopbits):
        if self.serial.is_open:
            self.close()
        try:
            self.serial.port = port
            self.serial.baudrate = baudrate
            self.serial.bytesize = bytesize
            self.serial.parity = parity
            self.serial.stopbits = stopbits
            self.serial.timeout = 0.1 
            self.serial.open()
            
            self.running = True
            self.rx_thread = threading.Thread(target=self._rx_task, daemon=True)
            self.rx_thread.start()
            self.sig_log_msg.emit(f"✅ 串口 {port} 已打开 (Baud: {baudrate})")
            self.sig_conn_state.emit(True) # 通知 UI 连接成功
            return True
        except Exception as e:
            self.sig_log_msg.emit(f"❌ 打开串口失败: {str(e)}")
            self.sig_conn_state.emit(False)
            return False

    def close(self):
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=0.2)
        if self.serial.is_open:
            self.serial.close()
        self.sig_log_msg.emit("🔌 串口已断开")
        self.sig_conn_state.emit(False) # 通知 UI 连接断开

    def send_frame(self, target_id: int, cmd: int, payload: bytes):
        if not self.serial.is_open:
            self.sig_log_msg.emit("⚠️ 串口未打开，发送取消")
            return

        raw_hdr = struct.pack('<BBH', target_id, cmd, len(payload))
        h_crc = calculate_crc16_ccitt(raw_hdr)
        raw_hdr += struct.pack('<H', h_crc)

        tx_buf = bytearray([SOF])
        for b in raw_hdr:
            if b in (SOF, EOF, ESC):
                tx_buf.append(ESC)
                tx_buf.append(b ^ XOR)
            else:
                tx_buf.append(b)
        tx_buf.append(EOF)

        if len(payload) > 0:
            tx_buf.extend(payload)
            p_crc = calculate_crc16_ccitt(payload)
            tx_buf.extend(struct.pack('<H', p_crc))
        else:
            tx_buf.extend(b'\x00\x00')

        try:
            self.serial.write(tx_buf)
        except Exception as e:
            self.sig_log_msg.emit(f"❌ 发送异常: {str(e)}")
            self.close()

    def _rx_task(self):
        STATE_WAIT, STATE_HDR, STATE_ESC, STATE_PLD = 0, 1, 2, 3
        state = STATE_WAIT
        hdr_buf = bytearray()
        pld_buf = bytearray()
        expected_len, current_cmd, current_target = 0, 0, 0

        while self.running:
            try:
                raw_bytes = self.serial.read(self.serial.in_waiting or 1)
            except Exception as e:
                # 【关键防护】这里捕获到了串口意外拔出
                if self.running:
                    self.sig_log_msg.emit(f"❌ 串口硬件断开！({str(e)})")
                    self.close() # 触发清理并通知 UI
                break
                
            if not raw_bytes: continue
            
            for byte in raw_bytes:
                if byte == SOF:
                    state = STATE_HDR; hdr_buf.clear(); continue
                
                if state == STATE_WAIT: continue
                elif state == STATE_HDR:
                    if byte == ESC: state = STATE_ESC
                    elif byte == EOF:
                        if len(hdr_buf) != 6: state = STATE_WAIT; continue
                        if calculate_crc16_ccitt(hdr_buf[0:4]) != struct.unpack('<H', hdr_buf[4:6])[0]:
                            self.sig_log_msg.emit("⚠️ Header CRC Error")
                            state = STATE_WAIT; continue
                        current_target, current_cmd, expected_len = struct.unpack('<BBH', hdr_buf[0:4])
                        state = STATE_PLD; pld_buf.clear()
                    else:
                        hdr_buf.append(byte)
                elif state == STATE_ESC:
                    hdr_buf.append(byte ^ XOR); state = STATE_HDR
                elif state == STATE_PLD:
                    pld_buf.append(byte)
                    if len(pld_buf) == expected_len + 2:
                        actual_pld = pld_buf[:-2]
                        if calculate_crc16_ccitt(actual_pld) == struct.unpack('<H', pld_buf[-2:])[0]:
                            self.sig_frame_received.emit(current_target, current_cmd, bytes(actual_pld))
                        else:
                            self.sig_log_msg.emit("⚠️ Payload CRC Error")
                        state = STATE_WAIT