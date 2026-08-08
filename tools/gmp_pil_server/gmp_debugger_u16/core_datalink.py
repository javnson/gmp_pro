import serial
import struct
import threading
import queue
import time
from datetime import datetime
from PyQt5.QtCore import QObject, pyqtSignal

SOF, EOF, ESC, XOR = 0x7B, 0x7D, 0x25, 0x20

# 预计算 CRC 表以极速校验
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
    
    # 全局总线事件，携带丰富的时间戳和解析信息
    sig_bus_event = pyqtSignal(dict) 

    def __init__(self, local_id=0xFF):
        super().__init__()
        self.serial = serial.Serial()
        self.local_id = local_id
        self.running = False
        
        self.rx_thread = None
        self.tx_thread = None
        
        # 【核心架构】：线程安全的优先级调度队列
        self.tx_queue = queue.PriorityQueue()
        self._tx_seq = 0 # 自增序列号，确保相同优先级时的先进先出(FIFO)稳定性
        self._seq_lock = threading.Lock()

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
            
            # 清空历史残留的脏队列
            while not self.tx_queue.empty():
                try: self.tx_queue.get_nowait()
                except queue.Empty: break

            # 启动收发双擎
            self.rx_thread = threading.Thread(target=self._rx_task, daemon=True)
            self.tx_thread = threading.Thread(target=self._tx_task, daemon=True)
            self.rx_thread.start()
            self.tx_thread.start()
            
            self.sig_log_msg.emit(f"✅ 串口 {port} 已打开 (启动双核调度队列)")
            self.sig_conn_state.emit(True)
            return True
        except Exception as e:
            self.sig_log_msg.emit(f"❌ 打开串口失败: {str(e)}")
            self.sig_conn_state.emit(False)
            return False

    def close(self):
        self.running = False
        if self.rx_thread: self.rx_thread.join(timeout=0.2)
        if self.tx_thread: self.tx_thread.join(timeout=0.2)
        if self.serial.is_open: self.serial.close()
        self.sig_log_msg.emit("🔌 串口已断开")
        self.sig_conn_state.emit(False)

    # =========================================================
    # 发送接口层：只负责拼装与投递，绝不触碰物理串口
    # =========================================================
    def send_raw(self, data: bytes, priority: int = 1):
        """
        盲发纯净字节 (默认普通优先级 1)
        priority: 0=最高级(插队), 1=普通级, 2=低级
        """
        if not self.serial.is_open: return
        
        with self._seq_lock:
            self._tx_seq += 1
            seq = self._tx_seq
            
        event_dict = {'dir': 'TX', 'type': 'RAW', 'data': data}
        # 将打包好的任务推入优先级队列
        self.tx_queue.put((priority, seq, data, event_dict))

    def send_frame(self, target_id: int, cmd: int, payload: bytes, priority: int = 1):
        """
        封装并投递协议帧 (加入 Priority 仲裁参数)
        """
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

        with self._seq_lock:
            self._tx_seq += 1
            seq = self._tx_seq

        event_dict = {
            'dir': 'TX', 'type': 'DL', 'dl_target': target_id, 'dl_cmd': cmd, 
            'dl_payload': payload, 'dl_crc_ok': True, 'error': ''
        }
        
        # 投递：元组的第一个元素 priority 决定了发出的先后顺序
        self.tx_queue.put((priority, seq, bytes(tx_buf), event_dict))

    # =========================================================
    # 物理层任务：发送引擎 (Tx Task)
    # =========================================================
    def _tx_task(self):
        """冷酷的物理层发牌员，严格按照优先级消化队列"""
        while self.running:
            try:
                # 阻塞式拿取任务，超时 0.1s 用于响应退出信号
                priority, seq, tx_data, ev_dict = self.tx_queue.get(timeout=0.1)
                
                if not self.running or not self.serial.is_open:
                    continue
                
                # 物理写入，并打上最精确的发包时间戳
                self.serial.write(tx_data)
                
                ev_dict['time'] = get_time_str()
                ev_dict['data'] = tx_data
                self.sig_bus_event.emit(ev_dict)
                
                # 标记该任务已被处理完成
                self.tx_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                if self.running:
                    self.sig_log_msg.emit(f"❌ 物理发送管线崩溃: {str(e)}")
                    self.close()

    # =========================================================
    # 物理层任务：接收引擎 (Rx Task) - 保持极速响应
    # =========================================================
    def _rx_task(self):
        STATE_WAIT, STATE_HDR, STATE_ESC, STATE_PLD = 0, 1, 2, 3
        state = STATE_WAIT
        hdr_buf = bytearray()
        pld_buf = bytearray()
        frame_raw_buf = bytearray() 
        bypass_buf = bytearray()    
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
                if byte == SOF and state != STATE_PLD:
                    if bypass_buf:
                        self.sig_bus_event.emit({'dir': 'RX', 'type': 'RAW', 'time': get_time_str(), 'data': bytes(bypass_buf)})
                        bypass_buf.clear()
                    state = STATE_HDR
                    hdr_buf.clear()
                    frame_raw_buf.clear()
                    frame_raw_buf.append(byte)
                    continue
                
                if state == STATE_WAIT:
                    bypass_buf.append(byte)
                    
                elif state == STATE_HDR:
                    frame_raw_buf.append(byte)
                    if byte == ESC: 
                        state = STATE_ESC
                    elif byte == EOF:
                        if len(hdr_buf) != 6:
                            bypass_buf.extend(frame_raw_buf) 
                            state = STATE_WAIT
                            continue
                        
                        h_crc_calc = calculate_crc16_ccitt(hdr_buf[0:4])
                        h_crc_rcv = struct.unpack('<H', hdr_buf[4:6])[0]
                        
                        if h_crc_calc != h_crc_rcv:
                            current_target, current_cmd, expected_len = struct.unpack('<BBH', hdr_buf[0:4])
                            self.sig_bus_event.emit({
                                'dir': 'RX', 'type': 'DL', 'time': get_time_str(), 'data': bytes(frame_raw_buf),
                                'dl_target': current_target, 'dl_cmd': current_cmd, 'dl_payload': b'', 'dl_crc_ok': False, 'error': 'Header CRC'
                            })
                            state = STATE_WAIT
                            continue
                        
                        current_target, current_cmd, expected_len = struct.unpack('<BBH', hdr_buf[0:4])
                        
                        if expected_len == 0:
                            self.sig_bus_event.emit({
                                'dir': 'RX', 'type': 'DL', 'time': get_time_str(), 'data': bytes(frame_raw_buf),
                                'dl_target': current_target, 'dl_cmd': current_cmd, 'dl_payload': b'', 'dl_crc_ok': True, 'error': ''
                            })
                            self.sig_frame_received.emit(current_target, current_cmd, b'')
                            state = STATE_WAIT
                        else:
                            state = STATE_PLD
                            pld_buf.clear()
                    else:
                        hdr_buf.append(byte)
                        
                elif state == STATE_ESC:
                    frame_raw_buf.append(byte)
                    hdr_buf.append(byte ^ XOR)
                    state = STATE_HDR
                    
                elif state == STATE_PLD:
                    frame_raw_buf.append(byte)
                    pld_buf.append(byte)
                    
                    if len(pld_buf) == expected_len + 2:
                        actual_pld = pld_buf[:-2]
                        p_crc_calc = calculate_crc16_ccitt(actual_pld)
                        p_crc_rcv = struct.unpack('<H', pld_buf[-2:])[0]
                        crc_ok = (p_crc_calc == p_crc_rcv)
                        
                        self.sig_bus_event.emit({
                            'dir': 'RX', 'type': 'DL', 'time': get_time_str(), 'data': bytes(frame_raw_buf),
                            'dl_target': current_target, 'dl_cmd': current_cmd, 'dl_payload': bytes(actual_pld), 'dl_crc_ok': crc_ok, 'error': '' if crc_ok else 'Payload CRC'
                        })
                        if crc_ok:
                            self.sig_frame_received.emit(current_target, current_cmd, bytes(actual_pld))
                        state = STATE_WAIT

            if bypass_buf:
                self.sig_bus_event.emit({'dir': 'RX', 'type': 'RAW', 'time': get_time_str(), 'data': bytes(bypass_buf)})
                bypass_buf.clear()