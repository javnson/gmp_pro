import serial
import struct
import threading
from datetime import datetime
from PyQt5.QtCore import QObject, pyqtSignal

SOF, EOF, ESC, XOR = 0x7B, 0x7D, 0x25, 0x20

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

def get_time_str():
    """获取当前时间的毫秒级字符串"""
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]

class HermesDatalinkQt(QObject):
    sig_frame_received = pyqtSignal(int, int, bytes)
    sig_log_msg = pyqtSignal(str)
    sig_conn_state = pyqtSignal(bool)
    
    # 【核心升级】全局总线事件，携带丰富的时间戳和解析信息
    sig_bus_event = pyqtSignal(dict) 

    def __init__(self, local_id=0xFF):
        super().__init__()
        self.serial = serial.Serial()
        self.local_id = local_id
        self.running = False
        self.rx_thread = None

    def connect_serial(self, port, baudrate, bytesize, parity, stopbits):
        if self.serial.is_open: self.close()
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
            self.sig_log_msg.emit(f"✅ 串口 {port} 已打开")
            self.sig_conn_state.emit(True)
            return True
        except Exception as e:
            self.sig_log_msg.emit(f"❌ 打开串口失败: {str(e)}")
            self.sig_conn_state.emit(False)
            return False

    def close(self):
        self.running = False
        if self.rx_thread: self.rx_thread.join(timeout=0.2)
        if self.serial.is_open: self.serial.close()
        self.sig_log_msg.emit("🔌 串口已断开")
        self.sig_conn_state.emit(False)

    def send_raw(self, data: bytes):
        """盲发纯净字节，触发 RAW 事件"""
        if not self.serial.is_open: return
        try:
            self.serial.write(data)
            self.sig_bus_event.emit({'dir': 'TX', 'type': 'RAW', 'time': get_time_str(), 'data': data})
        except Exception as e:
            self.sig_log_msg.emit(f"❌ 发送异常: {str(e)}")
            self.close()

    def send_frame(self, target_id: int, cmd: int, payload: bytes):
        """发送协议帧，触发 DL 事件"""
        if not self.serial.is_open: return
        raw_hdr = struct.pack('<BBH', target_id, cmd, len(payload))
        h_crc = calculate_crc16_ccitt(raw_hdr)
        raw_hdr += struct.pack('<H', h_crc)

        tx_buf = bytearray([SOF])
        for b in raw_hdr:
            if b in (SOF, EOF, ESC):
                tx_buf.append(ESC); tx_buf.append(b ^ XOR)
            else: tx_buf.append(b)
        tx_buf.append(EOF)

        if len(payload) > 0:
            tx_buf.extend(payload)
            tx_buf.extend(struct.pack('<H', calculate_crc16_ccitt(payload)))
        else:
            tx_buf.extend(b'\x00\x00')

        try:
            self.serial.write(tx_buf)
            # 全局广播 DL 发送事件，这样 RAW 页面也能看到
            self.sig_bus_event.emit({
                'dir': 'TX', 'type': 'DL', 'time': get_time_str(), 'data': bytes(tx_buf),
                'dl_target': target_id, 'dl_cmd': cmd, 'dl_payload': payload, 'dl_crc_ok': True, 'error': ''
            })
        except Exception as e:
            self.sig_log_msg.emit(f"❌ 发送异常: {str(e)}")
            self.close()

    def _rx_task(self):
        STATE_WAIT, STATE_HDR, STATE_ESC, STATE_PLD = 0, 1, 2, 3
        state = STATE_WAIT
        hdr_buf = bytearray()
        pld_buf = bytearray()
        frame_raw_buf = bytearray() # 记录完整协议帧的物理字节
        bypass_buf = bytearray()    # 收集非协议的游离字节
        expected_len, current_cmd, current_target = 0, 0, 0

        while self.running:
            try:
                raw_bytes = self.serial.read(self.serial.in_waiting or 1)
            except Exception as e:
                if self.running:
                    self.sig_log_msg.emit(f"❌ 串口硬件断开！({str(e)})")
                    self.close()
                break
                
            if not raw_bytes: continue
            
            for byte in raw_bytes:
                if byte == SOF:
                    if bypass_buf:
                        self.sig_bus_event.emit({'dir': 'RX', 'type': 'RAW', 'time': get_time_str(), 'data': bytes(bypass_buf)})
                        bypass_buf.clear()
                    state = STATE_HDR; hdr_buf.clear(); frame_raw_buf.clear(); frame_raw_buf.append(byte)
                    continue
                
                if state == STATE_WAIT:
                    bypass_buf.append(byte)
                elif state == STATE_HDR:
                    frame_raw_buf.append(byte)
                    if byte == ESC: state = STATE_ESC
                    elif byte == EOF:
                        if len(hdr_buf) != 6:
                            bypass_buf.extend(frame_raw_buf) # 假 Header，退回给 bypass
                            state = STATE_WAIT; continue
                        if calculate_crc16_ccitt(hdr_buf[0:4]) != struct.unpack('<H', hdr_buf[4:6])[0]:
                            current_target, current_cmd, expected_len = struct.unpack('<BBH', hdr_buf[0:4])
                            self.sig_bus_event.emit({
                                'dir': 'RX', 'type': 'DL', 'time': get_time_str(), 'data': bytes(frame_raw_buf),
                                'dl_target': current_target, 'dl_cmd': current_cmd, 'dl_payload': b'', 'dl_crc_ok': False, 'error': 'Header CRC'
                            })
                            state = STATE_WAIT; continue
                        
                        current_target, current_cmd, expected_len = struct.unpack('<BBH', hdr_buf[0:4])
                        state = STATE_PLD; pld_buf.clear()
                    else:
                        hdr_buf.append(byte)
                elif state == STATE_ESC:
                    frame_raw_buf.append(byte)
                    hdr_buf.append(byte ^ XOR); state = STATE_HDR
                elif state == STATE_PLD:
                    frame_raw_buf.append(byte)
                    pld_buf.append(byte)
                    if len(pld_buf) == expected_len + 2:
                        actual_pld = pld_buf[:-2]
                        crc_ok = (calculate_crc16_ccitt(actual_pld) == struct.unpack('<H', pld_buf[-2:])[0])
                        self.sig_bus_event.emit({
                            'dir': 'RX', 'type': 'DL', 'time': get_time_str(), 'data': bytes(frame_raw_buf),
                            'dl_target': current_target, 'dl_cmd': current_cmd, 'dl_payload': bytes(actual_pld), 'dl_crc_ok': crc_ok, 'error': '' if crc_ok else 'Payload CRC'
                        })
                        if crc_ok:
                            self.sig_frame_received.emit(current_target, current_cmd, bytes(actual_pld))
                        state = STATE_WAIT

            # 块结束，抛出游离字节
            if bypass_buf:
                self.sig_bus_event.emit({'dir': 'RX', 'type': 'RAW', 'time': get_time_str(), 'data': bytes(bypass_buf)})
                bypass_buf.clear()