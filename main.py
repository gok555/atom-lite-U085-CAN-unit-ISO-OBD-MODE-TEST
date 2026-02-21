
import M5
from M5 import *
import time
from unit import CANUnit
from hardware import I2C, Pin
import bluetooth
import struct
from micropython import const
from esp32 import NVS

# ==========================================
# ATOM S3 + ATOM CAN Base
# Ver 4.0: OBD-II ISO 15765-4 (CAN) Support
# BLE UART → Smartphone Dashboard
# ==========================================

TX_PIN = 6
RX_PIN = 5
FIFO   = 0

# ---- OBD-II CAN IDs ----
OBD_REQ_ID  = 0x7DF   # Functional broadcast
OBD_RESP_ID = 0x7E8   # ECU response (primary)

# ---- Operating Mode ----
# 0 = Generic CAN keypad mode (original)
# 1 = OBD-II polling mode
op_mode = 0

# ---- OBD PID Definitions ----
# key: PID (int), value: (name, unit, formula_id)
# formula_id:
#   0: A - 40
#   1: A / 2.55
#   2: (256*A + B) / 4   (RPM)
#   3: A                 (raw byte)
#   4: (256*A + B) / 100 (MAF)
#   5: A / 2 - 64        (timing)
#   6: (256*A + B) / 20  (fuel rate)
#   7: A - 125           (torque %)
OBD_PIDS = {
    0x04: ("LOAD",  "%",    1),
    0x05: ("CLT",   "C",    0),
    0x06: ("STFT1", "%",    1),
    0x07: ("LTFT1", "%",    1),
    0x0B: ("MAP",   "kPa",  3),
    0x0C: ("RPM",   "rpm",  2),
    0x0D: ("SPD",   "km/h", 3),
    0x0E: ("TADV",  "deg",  5),
    0x0F: ("IAT",   "C",    0),
    0x10: ("MAF",   "g/s",  4),
    0x11: ("TPS",   "%",    1),
    0x2F: ("FUEL",  "%",    1),
    0x46: ("AIRT",  "C",    0),
    0x5C: ("OILT",  "C",    0),
    0x5E: ("FRATE", "L/h",  6),
    0x62: ("TRQ",   "%",    7),
}

# Active PID list to cycle through (default: most common)
active_pids = [0x0C, 0x0D, 0x05, 0x04, 0x11, 0x0F, 0x0B, 0x10]

# Current values dict: pid -> value (or None)
obd_vals = {pid: None for pid in OBD_PIDS}

# Polling state
obd_poll_idx    = 0
obd_last_req_ms = 0
OBD_POLL_INTERVAL = 100  # ms between requests
OBD_RESP_TIMEOUT  = 80   # ms to wait for response
obd_waiting_resp  = False
obd_waiting_pid   = None

# ---- Keypad / CAN TX State (kept from original) ----
can_tx_id   = 0x100
k_meter_id  = 0x200
can_state   = bytearray(8)
can_error   = False

rx_monitors = [
    [0x90, 2, 1],
    [0x90, 2, 1],
    [0x90, 2, 1],
    [0x90, 2, 1],
    [0x90, 2, 1],
    [0x90, 2, 1],
    [0x90, 2, 1],
]
rx_config_req   = None
last_rx_monitor_val = "---"
last_temp_disp  = "0"
rx_count        = 0
last_rx_id      = 0

nvs = None

DEVICE_NAME = "M5AtomS3_OBD"

# ---- BLE UUIDs (Nordic UART) ----
UART_UUID = bluetooth.UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e")
UART_TX   = bluetooth.UUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e")
UART_RX   = bluetooth.UUID("6e400002-b5a3-f393-e0a9-e50e24dcca9e")

_IRQ_CENTRAL_CONNECT    = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE        = const(3)


# ===================================================
# NVS
# ===================================================
def init_nvs():
    global nvs, can_tx_id, k_meter_id, rx_monitors, op_mode, active_pids
    try:
        nvs = NVS("obd_app")
        print("NVS OK")
    except:
        nvs = None
        print("NVS Fail")

    if nvs:
        try: can_tx_id  = nvs.get_i32("my_id")
        except OSError: pass
        try: k_meter_id = nvs.get_i32("k_meter_id")
        except OSError: pass
        try: op_mode    = nvs.get_i32("op_mode")
        except OSError: pass

        # Active PID list stored as comma-separated hex in a namespace key
        # (NVS only supports i32, so store each PID index)
        try:
            pid_count = nvs.get_i32("pid_cnt")
            pids_tmp = []
            for i in range(pid_count):
                try:
                    p = nvs.get_i32(f"pid{i}")
                    pids_tmp.append(p)
                except OSError: pass
            if pids_tmp:
                active_pids = pids_tmp
        except OSError: pass

        # Load rx_monitors
        try:
            rx_monitors[0][0] = nvs.get_i32("rx_id")
            rx_monitors[0][1] = nvs.get_i32("rx_idx")
            rx_monitors[0][2] = nvs.get_i32("rx_m")
        except OSError: pass
        for i in range(1, 7):
            try:
                rx_monitors[i][0] = nvs.get_i32(f"rx{i}_id")
                rx_monitors[i][1] = nvs.get_i32(f"rx{i}_idx")
                rx_monitors[i][2] = nvs.get_i32(f"rx{i}_m")
            except OSError: pass


def save_obd_pids():
    if not nvs: return
    try:
        nvs.set_i32("pid_cnt", len(active_pids))
        for i, p in enumerate(active_pids):
            nvs.set_i32(f"pid{i}", p)
        nvs.commit()
    except: pass


def load_btn_nvs():
    if not nvs: return
    try:
        for i in range(8):
            try:
                off_val = nvs.get_i32(f"btn{i+1}_off")
                can_state[i] = off_val & 0xFF
            except OSError: pass
    except: pass


# ===================================================
# OBD Helper
# ===================================================
def calc_obd_value(pid, a, b):
    """Apply PID formula and return float value."""
    if pid not in OBD_PIDS:
        return a
    _, _, fid = OBD_PIDS[pid]
    if fid == 0: return a - 40
    if fid == 1: return round(a / 2.55, 1)
    if fid == 2: return int((256 * a + b) / 4)
    if fid == 3: return a
    if fid == 4: return round((256 * a + b) / 100, 2)
    if fid == 5: return round(a / 2 - 64, 1)
    if fid == 6: return round((256 * a + b) / 20, 1)
    if fid == 7: return a - 125
    return a


def obd_send_request(pid):
    """Send OBD Mode 01 request."""
    data = bytearray([0x02, 0x01, pid, 0x00, 0x00, 0x00, 0x00, 0x00])
    try:
        can.send(id=OBD_REQ_ID, data=data)
    except:
        pass


# ===================================================
# BLE UART
# ===================================================
class BLEUART:
    def __init__(self, ble):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._tx, self._rx),) = self._ble.gatts_register_services([
            (UART_UUID, (
                (UART_TX, bluetooth.FLAG_NOTIFY),
                (UART_RX, bluetooth.FLAG_WRITE),
            )),
        ])
        self._connections = set()
        self._rx_cb = None
        self._adv_payload = self._mk_adv(DEVICE_NAME)
        self._advertise()

    def _mk_adv(self, name):
        nb = name.encode()
        return b'\x02\x01\x06' + bytes([len(nb)+1, 0x09]) + nb

    def _advertise(self):
        try: self._ble.gap_advertise(100_000, self._adv_payload)
        except: pass

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            time.sleep(0.5)
            self.send_state_sync()
            time.sleep(0.1)
            self.send_config()
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            self._connections.discard(conn_handle)
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            if self._rx_cb:
                self._rx_cb(self._ble.gatts_read(self._rx))

    def send(self, data):
        for h in self._connections:
            try: self._ble.gatts_notify(h, self._tx, data)
            except: pass

    def send_state_sync(self):
        self.send(("STATE=" + ",".join(str(b) for b in can_state)).encode())

    def send_status(self, is_err):
        self.send((f"STATUS={'CAN_ERR' if is_err else 'CAN_OK'}").encode())

    def send_config(self):
        self.send((f"ID={hex(can_tx_id)}").encode())
        time.sleep(0.05)
        self.send((f"KID={hex(k_meter_id)}").encode())
        time.sleep(0.05)
        self.send((f"MODE={op_mode}").encode())
        time.sleep(0.05)
        # Send active PID list
        pid_str = ",".join(hex(p) for p in active_pids)
        self.send((f"PIDS={pid_str}").encode())
        time.sleep(0.05)
        self.send((f"RXID={hex(rx_monitors[0][0])}").encode())

    def send_obd_data(self):
        """Send all current OBD values in one burst."""
        parts = []
        for pid in active_pids:
            if pid in OBD_PIDS and obd_vals.get(pid) is not None:
                name = OBD_PIDS[pid][0]
                parts.append(f"{name}:{obd_vals[pid]}")
        if parts:
            try: self.send(("OBD=" + ",".join(parts)).encode())
            except: pass

    def on_rx(self, cb):
        self._rx_cb = cb


# ===================================================
# BLE RX Handler
# ===================================================
def on_ble_rx(data):
    global can_state, can_tx_id, k_meter_id, rx_monitors, rx_config_req
    global op_mode, active_pids, obd_vals

    try:
        cmd = data.decode().strip()
        print("BLE RX:", cmd)
        updated = False

        if cmd == "GETCONFIG":
            uart.send_config()

        elif cmd.startswith("SET_MODE="):
            # SET_MODE=0 (CAN keypad) or SET_MODE=1 (OBD)
            try:
                val = int(cmd.split("=")[1])
                op_mode = val
                if nvs:
                    nvs.set_i32("op_mode", val)
                    nvs.commit()
                uart.send((f"MODE={op_mode}").encode())
                print("Mode:", op_mode)
            except: pass

        elif cmd.startswith("SET_PIDS="):
            # SET_PIDS=0x0C,0x0D,0x05,...  (up to 8 PIDs)
            try:
                pid_part = cmd.split("=")[1]
                new_pids = []
                for tok in pid_part.split(","):
                    tok = tok.strip()
                    p = int(tok, 16) if tok.startswith("0x") or tok.startswith("0X") else int(tok, 16)
                    if p in OBD_PIDS:
                        new_pids.append(p)
                if new_pids:
                    active_pids = new_pids[:16]  # max 16 PIDs
                    obd_vals = {pid: None for pid in OBD_PIDS}
                    save_obd_pids()
                    pid_str = ",".join(hex(p) for p in active_pids)
                    uart.send((f"PIDS={pid_str}").encode())
                    print("PIDs updated:", active_pids)
            except Exception as e:
                print("SET_PIDS err:", e)

        elif cmd.startswith("CFGBTN="):
            try:
                parts = cmd.split('=')[1].split(',')
                if len(parts) == 3:
                    idx   = int(parts[0])
                    on_v  = int(parts[1])
                    off_v = int(parts[2])
                    if 0 <= idx < 8:
                        if can_state[idx] != (on_v & 0xFF):
                            can_state[idx] = off_v & 0xFF
                            updated = True
                        if nvs:
                            try:
                                nvs.set_i32(f"btn{idx+1}_on", on_v)
                                nvs.set_i32(f"btn{idx+1}_off", off_v)
                                nvs.commit()
                            except: pass
            except: pass

        elif '=' in cmd:
            parts = cmd.split('=', 1)
            key = parts[0]
            val_str = parts[1]

            if key == "ID":
                try:
                    v = int(val_str, 16)
                    can_tx_id = v
                    if nvs: nvs.set_i32("my_id", v); nvs.commit()
                except: pass

            elif key == "KID":
                try:
                    v = int(val_str, 16)
                    k_meter_id = v
                    if nvs: nvs.set_i32("k_meter_id", v); nvs.commit()
                except: pass

            elif key == "SET_RX":
                try:
                    rx_parts = val_str.split(',')
                    if len(rx_parts) >= 4:
                        sl  = int(rx_parts[0])
                        rid = int(rx_parts[1])
                        rix = int(rx_parts[2])
                        rm  = int(rx_parts[3])
                        if 0 <= sl < 7:
                            rx_monitors[sl][0] = rid
                            rx_monitors[sl][1] = rix
                            rx_monitors[sl][2] = rm
                            rx_config_req = (sl, rid, rix, rm)
                            if sl == 0:
                                try: uart.send(f"RXID={hex(rid)}".encode())
                                except: pass
                    elif len(rx_parts) == 3:
                        rid = int(rx_parts[0])
                        rix = int(rx_parts[1])
                        rm  = int(rx_parts[2])
                        rx_monitors[0][0] = rid
                        rx_monitors[0][1] = rix
                        rx_monitors[0][2] = rm
                        rx_config_req = (0, rid, rix, rm)
                        try: uart.send(f"RXID={hex(rid)}".encode())
                        except: pass
                except Exception as e:
                    print("SET_RX Err:", e)

            elif key.isdigit() and val_str.isdigit():
                idx = int(key) - 1
                v   = int(val_str)
                if 0 <= idx < 8:
                    if can_state[idx] != v:
                        can_state[idx] = v & 0xFF
                        updated = True

        elif cmd.isdigit():
            idx = int(cmd) - 1
            if 0 <= idx < 8:
                can_state[idx] ^= 1
                updated = True

        if updated:
            try: can.send(id=can_tx_id, data=can_state)
            except: pass
            uart.send_state_sync()

    except Exception as e:
        print("BLE RX Err:", e)


# ===================================================
# Baudrate Detection
# ===================================================
def detect_baudrate():
    M5.Display.clear()
    M5.Display.setCursor(0, 0)
    M5.Display.setTextSize(2)
    M5.Display.setTextColor(0xFFFF, 0x0000)
    M5.Display.print("Detecting\nBaudrate...")
    print("--- Auto Baudrate Detection ---")
    try:
        for attempt in range(1, 4):
            for rate in [500000, 250000, 1000000]:
                can_t = None
                try:
                    can_t = CANUnit(id=0, port=(TX_PIN, RX_PIN), mode=CANUnit.NORMAL, baudrate=rate)
                    start = time.ticks_ms()
                    detected = False
                    while time.ticks_diff(time.ticks_ms(), start) < 1500:
                        try:
                            if can_t.any(0) > 0:
                                detected = True
                                break
                        except: pass
                        time.sleep_ms(50)
                    if hasattr(can_t, 'deinit'): can_t.deinit()
                    if detected:
                        print(f"Locked: {rate}")
                        return rate
                except Exception as e:
                    print(f"  Err {rate}:", e)
                    if can_t and hasattr(can_t, 'deinit'): can_t.deinit()
                time.sleep_ms(100)
    except KeyboardInterrupt: pass
    print("Default: 500000")
    return 500000


# ===================================================
# LCD Update
# ===================================================
def update_lcd():
    M5.Display.setTextSize(2)
    M5.Display.setCursor(0, 0)
    if can_error:
        M5.Display.setTextColor(0xF800, 0x0000)
        M5.Display.print("CAN: ERR  ")
    else:
        M5.Display.setTextColor(0x07FF, 0x0000)
        M5.Display.print("CAN: OK   ")

    M5.Display.setCursor(0, 20)
    if len(uart._connections) > 0:
        M5.Display.setTextColor(0xFFE0, 0x0000)
        M5.Display.print("BLE: ON   ")
    else:
        M5.Display.setTextColor(0x07FF, 0x0000)
        M5.Display.print("BLE: --   ")

    M5.Display.setCursor(0, 40)
    M5.Display.setTextColor(0xFFFF, 0x0000)
    mode_str = "OBD" if op_mode == 1 else "CAN"
    M5.Display.print(f"Mode:{mode_str}  ")

    M5.Display.setCursor(0, 60)
    M5.Display.setTextColor(0xFFFF, 0x0000)
    M5.Display.print(f"Rate:{detected_rate//1000}k  ")

    # Show first 3 OBD values
    y = 80
    if op_mode == 1:
        for pid in active_pids[:3]:
            if pid in OBD_PIDS:
                name, unit, _ = OBD_PIDS[pid]
                val = obd_vals.get(pid)
                val_str = str(val) if val is not None else "--"
                M5.Display.setCursor(0, y)
                M5.Display.setTextColor(0x07E0, 0x0000)
                M5.Display.print(f"{name}:{val_str}{unit}      ")
                y += 20
    else:
        M5.Display.setCursor(0, y)
        M5.Display.setTextColor(0xFFE0, 0x0000)
        M5.Display.print(f"PAD:{can_tx_id:X}     ")
        M5.Display.setCursor(0, y + 20)
        M5.Display.print(f"RX:{rx_monitors[0][0]:X}  ")


# ===================================================
# MAIN SETUP
# ===================================================
M5.begin()
init_nvs()
load_btn_nvs()

detected_rate = detect_baudrate()

can = None
try:
    can = CANUnit(id=0, port=(TX_PIN, RX_PIN), mode=CANUnit.NORMAL, baudrate=detected_rate)
    # Accept all messages (no hardware filter)
    try:
        if hasattr(can, 'setfilter'):
            can.setfilter(0, CANUnit.FILTER_RAW_SINGLE, [0, 0])
        elif hasattr(can, 'set_filter'):
            can.set_filter(0, 0, 0)
    except: pass
    if op_mode == 0:
        can.send(id=can_tx_id, data=can_state)
    print("CAN Init OK")
except Exception as e:
    print("CAN Init Err:", e)

ble  = bluetooth.BLE()
uart = BLEUART(ble)
uart.on_rx(on_ble_rx)

# K-Meter (optional)
kmeter_addr  = 0x66
kmeter_found = False
i2c0         = None
try:
    i2c0 = I2C(0, scl=Pin(1), sda=Pin(2), freq=100000)
    time.sleep(0.3)
    if kmeter_addr in i2c0.scan():
        kmeter_found = True
        print("KMeter OK")
except: pass

M5.Display.clear()

# Timing variables
last_display_update = 0
DISPLAY_INTERVAL    = 250

last_can_send_time  = 0
CAN_SEND_INTERVAL   = 200

last_ble_sync_time  = 0
BLE_SYNC_INTERVAL   = 1000

last_temp_send_time = 0
TEMP_SEND_INTERVAL  = 500

last_obd_send_time  = 0
OBD_SEND_INTERVAL   = 200  # send BLE OBD bundle every 200ms

print(">>> OBD Ver 4.0 Ready <<<")

# ===================================================
# MAIN LOOP
# ===================================================
while True:
    M5.update()
    now = time.ticks_ms()

    # ---- Button Test ----
    if BtnA.wasPressed():
        if op_mode == 1:
            # Switch back to show keypad
            op_mode = 0
            uart.send(b"MODE=0")
        else:
            op_mode = 1
            uart.send(b"MODE=1")

    # ---- K-Meter (Temperature Sensor) ----
    if kmeter_found and time.ticks_diff(now, last_temp_send_time) > TEMP_SEND_INTERVAL:
        try:
            data = i2c0.readfrom_mem(kmeter_addr, 0x00, 4)
            val_int  = struct.unpack('<i', data)[0]
            temp_val = val_int / 100.0
            t_int    = int(temp_val)
            sign     = 1 if temp_val >= 0 else 0
            abs_val  = abs(t_int)
            sensor_data = bytearray([0x01, sign, (abs_val >> 8) & 0xFF, abs_val & 0xFF, 0,0,0,0])
            last_temp_disp = str(t_int)
            try: can.send(id=k_meter_id, data=sensor_data)
            except: pass
            try: uart.send(f"TEMP={t_int}".encode())
            except: pass
        except: pass
        last_temp_send_time = now

    # ---- CAN TX (keypad state) - only in CAN mode ----
    if op_mode == 0 and time.ticks_diff(now, last_can_send_time) > CAN_SEND_INTERVAL:
        try:
            can.send(id=can_tx_id, data=can_state)
            can_error = False
        except:
            can_error = True
        last_can_send_time = now

    # ---- BLE Periodic Sync ----
    if time.ticks_diff(now, last_ble_sync_time) > BLE_SYNC_INTERVAL:
        if op_mode == 0:
            uart.send_state_sync()
        uart.send_status(can_error)
        last_ble_sync_time = now

    # ---- OBD Polling (mode 1) ----
    if op_mode == 1 and can:
        if not obd_waiting_resp:
            # Time to send next request?
            if time.ticks_diff(now, obd_last_req_ms) >= OBD_POLL_INTERVAL:
                if active_pids:
                    pid = active_pids[obd_poll_idx % len(active_pids)]
                    obd_send_request(pid)
                    obd_waiting_pid   = pid
                    obd_waiting_resp  = True
                    obd_last_req_ms   = now
        else:
            # Timeout check
            if time.ticks_diff(now, obd_last_req_ms) > OBD_RESP_TIMEOUT:
                # No response - advance
                obd_waiting_resp = False
                obd_poll_idx     = (obd_poll_idx + 1) % max(len(active_pids), 1)

    # ---- CAN RX ----
    if can and can.any(FIFO):
        try:
            msg = can.recv(FIFO)
            if msg:
                can_id, is_ext, rtr, dlc, rx_data = msg
                last_rx_id = can_id
                rx_count  += 1

                # -- OBD Response --
                if op_mode == 1 and obd_waiting_resp:
                    # Accept 0x7E8 or 0x7E0-0x7EF (any ECU)
                    if 0x7E0 <= can_id <= 0x7EF and dlc >= 4:
                        resp_mode = rx_data[1]  # Should be 0x41
                        resp_pid  = rx_data[2]
                        if resp_mode == 0x41 and resp_pid == obd_waiting_pid:
                            a = rx_data[3]
                            b = rx_data[4] if dlc > 4 else 0
                            val = calc_obd_value(resp_pid, a, b)
                            obd_vals[resp_pid] = val
                            obd_waiting_resp = False
                            obd_poll_idx = (obd_poll_idx + 1) % max(len(active_pids), 1)

                # -- K-Meter match --
                elif can_id == k_meter_id and dlc >= 4:
                    t_high = rx_data[2]
                    t_low  = rx_data[3]
                    val    = (t_high << 8) | t_low
                    if rx_data[1] == 0: val = -val
                    last_temp_disp = str(val)

                # -- Generic RX Monitors (CAN mode) --
                elif op_mode == 0:
                    for slot_i in range(7):
                        cfg_id   = rx_monitors[slot_i][0]
                        cfg_idx  = rx_monitors[slot_i][1]
                        cfg_mode = rx_monitors[slot_i][2]
                        if can_id == cfg_id:
                            try:
                                v = 0
                                if cfg_mode == 0:
                                    if cfg_idx < dlc: v = rx_data[cfg_idx]
                                elif cfg_mode == 1:
                                    if cfg_idx + 1 < dlc:
                                        v = rx_data[cfg_idx] | (rx_data[cfg_idx+1] << 8)
                                elif cfg_mode == 2:
                                    if cfg_idx + 1 < dlc:
                                        v = (rx_data[cfg_idx] << 8) | rx_data[cfg_idx+1]
                                if slot_i == 0:
                                    last_rx_monitor_val = str(v)
                                prefix = "VAL" if slot_i == 0 else f"VAL{slot_i+1}"
                                try: uart.send(f"{prefix}={v}".encode())
                                except: pass
                            except: pass
        except: pass

    # ---- OBD BLE Bundle Send ----
    if op_mode == 1 and time.ticks_diff(now, last_obd_send_time) > OBD_SEND_INTERVAL:
        uart.send_obd_data()
        last_obd_send_time = now

    # ---- Deferred NVS Save ----
    if rx_config_req:
        try:
            (sl, rid, rix, rm) = rx_config_req
            rx_config_req = None
            if nvs:
                try:
                    if sl == 0:
                        nvs.set_i32("rx_id",  rid)
                        nvs.set_i32("rx_idx", rix)
                        nvs.set_i32("rx_m",   rm)
                    else:
                        nvs.set_i32(f"rx{sl}_id",  rid)
                        nvs.set_i32(f"rx{sl}_idx", rix)
                        nvs.set_i32(f"rx{sl}_m",   rm)
                    nvs.commit()
                except: pass
        except: rx_config_req = None

    # ---- LCD ----
    if time.ticks_diff(now, last_display_update) > DISPLAY_INTERVAL:
        update_lcd()
        last_display_update = now

    time.sleep_ms(10)
