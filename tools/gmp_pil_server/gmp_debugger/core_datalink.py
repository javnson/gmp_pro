import serial
import struct
import threading
import queue
import time
from datetime import datetime
from PyQt5.QtCore import QObject, pyqtSignal

SOF, EOF, ESC, XOR = 0x7B, 0x7D, 0x25, 0x20
MAX_DL_PAYLOAD = 256
RX_INTERBYTE_TIMEOUT_SECONDS = 0.25
RAW_EVENT_INTERVAL_SECONDS = 0.10
RAW_EVENT_MAX_BYTES = 2048

# Precompute the CRC table for low-overhead frame validation.
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
    """Return the current local time with millisecond precision."""
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]

class HermesDatalinkQt(QObject):
    sig_frame_received = pyqtSignal(int, int, bytes)
    sig_log_event = pyqtSignal(str, str)
    """Structured log event containing a source name and plain-text message."""

    sig_log_msg = pyqtSignal(str)
    """Legacy unclassified log signal retained for external page compatibility."""

    sig_conn_state = pyqtSignal(bool)
    
    # Shared bus events include timestamps and decoded frame metadata.
    sig_bus_event = pyqtSignal(dict) 

    def __init__(self, local_id=0xFF):
        super().__init__()
        self.serial = serial.Serial()
        self.local_id = local_id
        self.running = False
        
        self.rx_thread = None
        self.tx_thread = None
        
        # Thread-safe priority queue shared by all feature pages.
        self.tx_queue = queue.PriorityQueue()
        self._tx_seq = 0  # Preserve FIFO order among requests with equal priority.
        self._seq_lock = threading.Lock()
        self._close_lock = threading.Lock()

    def emit_log(self, source: str, message: str) -> None:
        """Publish one source-classified message to the application log."""
        self.sig_log_event.emit(source, message)

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
            
            # Discard stale requests from a previous connection.
            while not self.tx_queue.empty():
                try: self.tx_queue.get_nowait()
                except queue.Empty: break

            # Start independent receive and transmit workers.
            self.rx_thread = threading.Thread(target=self._rx_task, daemon=True)
            self.tx_thread = threading.Thread(target=self._tx_task, daemon=True)
            self.rx_thread.start()
            self.tx_thread.start()
            
            self.emit_log("System", f"Serial port {port} opened with queued I/O workers.")
            self.sig_conn_state.emit(True)
            return True
        except Exception as e:
            self.emit_log("System", f"Failed to open the serial port: {str(e)}")
            self.sig_conn_state.emit(False)
            return False

    def close(self):
        """Stop both workers without attempting to join the calling worker."""
        with self._close_lock:
            current_thread = threading.current_thread()
            was_connected = self.running or self.serial.is_open
            self.running = False
            if self.rx_thread and self.rx_thread is not current_thread:
                self.rx_thread.join(timeout=0.2)
            if self.tx_thread and self.tx_thread is not current_thread:
                self.tx_thread.join(timeout=0.2)
            if self.serial.is_open:
                self.serial.close()
            if was_connected:
                self.emit_log("System", "Serial port disconnected.")
                self.sig_conn_state.emit(False)

    # =========================================================
    # Public transmit API: frame and enqueue without touching hardware directly.
    # =========================================================
    def send_raw(self, data: bytes, priority: int = 1):
        """
        Enqueue unframed bytes with normal priority by default.
        Priority 0 is urgent, 1 is normal, and 2 is background.
        """
        if not self.serial.is_open: return
        
        with self._seq_lock:
            self._tx_seq += 1
            seq = self._tx_seq
            
        event_dict = {'dir': 'TX', 'type': 'RAW', 'data': data}
        # Submit the immutable transfer to the priority queue.
        self.tx_queue.put((priority, seq, data, event_dict))

    def send_frame(self, target_id: int, cmd: int, payload: bytes, priority: int = 1):
        """
        Encode and enqueue one Data Link frame with explicit priority.
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
        
        # The first tuple field controls dispatch order.
        self.tx_queue.put((priority, seq, bytes(tx_buf), event_dict))

    # =========================================================
    # Physical transmit worker.
    # =========================================================
    def _tx_task(self):
        """Drain queued transfers strictly by priority and FIFO sequence."""
        while self.running:
            try:
                # A short timeout keeps shutdown responsive.
                priority, seq, tx_data, ev_dict = self.tx_queue.get(timeout=0.1)
                
                if not self.running or not self.serial.is_open:
                    continue
                
                # Timestamp the event immediately after the physical write.
                self.serial.write(tx_data)
                
                ev_dict['time'] = get_time_str()
                ev_dict['data'] = tx_data
                self.sig_bus_event.emit(ev_dict)
                
                # Mark the queued transfer complete.
                self.tx_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                if self.running:
                    self.emit_log("System", f"Serial transmit worker failed: {str(e)}")
                    self.close()

    # =========================================================
    # Low-latency physical receive worker.
    # =========================================================
    def _rx_task(self):
        STATE_WAIT, STATE_HDR, STATE_ESC, STATE_PLD = 0, 1, 2, 3
        state = STATE_WAIT
        hdr_buf = bytearray()
        pld_buf = bytearray()
        frame_raw_buf = bytearray() 
        bypass_buf = bytearray()    
        expected_len, current_cmd, current_target = 0, 0, 0
        last_byte_time = time.monotonic()
        last_raw_event_time = last_byte_time

        while self.running:
            try:
                raw_bytes = self.serial.read(self.serial.in_waiting or 1)
            except Exception as e:
                if self.running:
                    self.emit_log("System", f"Serial hardware disconnected: {str(e)}")
                    self.close()
                break
                
            now = time.monotonic()
            if not raw_bytes:
                if state != STATE_WAIT and now - last_byte_time >= RX_INTERBYTE_TIMEOUT_SECONDS:
                    self.sig_bus_event.emit({
                        'dir': 'RX', 'type': 'DL', 'time': get_time_str(),
                        'data': bytes(frame_raw_buf), 'dl_target': current_target,
                        'dl_cmd': current_cmd, 'dl_payload': b'', 'dl_crc_ok': False,
                        'error': 'Receive timeout'
                    })
                    state = STATE_WAIT
                    hdr_buf.clear()
                    pld_buf.clear()
                    frame_raw_buf.clear()
                if bypass_buf and now - last_raw_event_time >= RAW_EVENT_INTERVAL_SECONDS:
                    self.sig_bus_event.emit({
                        'dir': 'RX', 'type': 'RAW', 'time': get_time_str(),
                        'data': bytes(bypass_buf)
                    })
                    bypass_buf.clear()
                    last_raw_event_time = now
                continue
            
            for byte in raw_bytes:
                last_byte_time = time.monotonic()
                if byte == SOF and state != STATE_PLD:
                    state = STATE_HDR
                    hdr_buf.clear()
                    frame_raw_buf.clear()
                    frame_raw_buf.append(byte)
                    continue
                
                if state == STATE_WAIT:
                    if len(bypass_buf) < RAW_EVENT_MAX_BYTES:
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

                        if expected_len > MAX_DL_PAYLOAD:
                            self.sig_bus_event.emit({
                                'dir': 'RX', 'type': 'DL', 'time': get_time_str(),
                                'data': bytes(frame_raw_buf), 'dl_target': current_target,
                                'dl_cmd': current_cmd, 'dl_payload': b'', 'dl_crc_ok': False,
                                'error': 'Payload length exceeds MTU'
                            })
                            state = STATE_WAIT
                            continue
                        
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
                        if len(hdr_buf) < 6:
                            hdr_buf.append(byte)
                        else:
                            state = STATE_WAIT
                            hdr_buf.clear()
                            frame_raw_buf.clear()
                        
                elif state == STATE_ESC:
                    frame_raw_buf.append(byte)
                    if len(hdr_buf) < 6:
                        hdr_buf.append(byte ^ XOR)
                        state = STATE_HDR
                    else:
                        state = STATE_WAIT
                        hdr_buf.clear()
                        frame_raw_buf.clear()
                    
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

            now = time.monotonic()
            if bypass_buf and now - last_raw_event_time >= RAW_EVENT_INTERVAL_SECONDS:
                self.sig_bus_event.emit({'dir': 'RX', 'type': 'RAW', 'time': get_time_str(), 'data': bytes(bypass_buf)})
                bypass_buf.clear()
                last_raw_event_time = now
