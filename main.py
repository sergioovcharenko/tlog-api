from fastapi import FastAPI, UploadFile, File
from pymavlink import mavutil
import tempfile
import os
import math

app = FastAPI()

@app.get("/")
def root(): return {"status": "ok"}

@app.get("/health")
def health(): return {"status": "ok"}

@app.post("/analyze")
async def analyze(file: UploadFile = File(...)):
    data = await file.read()
    temp = tempfile.NamedTemporaryFile(delete=False, suffix=".tlog")
    
    try:
        temp.write(data)
        temp.close()

        mav = mavutil.mavlink_connection(temp.name)
        
        # Базові змінні
        message_count = 0
        max_alt = 0.0  
        start_alt = None 
        need_alt_reset = True
        max_speed = 0.0
        max_roll = 0.0
        max_pitch = 0.0
        
        # Зв'язок
        min_rssi = 255 
        telem_rssi = None
        telem_remrssi = None
        
        # Керування (PWM)
        rc_min = [9999, 9999, 9999, 9999]
        rc_max = [0, 0, 0, 0]
        max_throttle = 0
        
        # Батарея 6S
        min_voltage = 999.0
        max_current = 0.0
        start_voltage = None 
        
        # Датчики та Вібрації
        max_vib_x, max_vib_y, max_vib_z = 0.0, 0.0, 0.0
        clip_count = 0
        max_temp = -99.0
        ekf_compass = 0.0
        ekf_vel = 0.0
        
        has_gps = False
        
        # --- НОВИЙ АЛГОРИТМ ЧАСУ (msg._timestamp) ---
        first_timestamp = None
        current_timestamp = 0.0 
        
        # Стан польоту
        was_armed = False
        current_mode = "Невідомо"
        flight_modes = set()
        
        # Детектори 0.0.0.0
        gps_had_lock = False
        ekf_had_origin = False
        
        raw_timeline = []

        def add_event(text, t_stamp, mode, is_error=False):
            raw_timeline.append({
                "timestamp": t_stamp or 0,
                "mode": mode,
                "text": text,
                "isError": is_error
            })

        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None: break
            
            message_count += 1
            msg_type = msg.get_type()
            
            # Читаємо реальний час TLOG-файлу
            t_stamp = getattr(msg, '_timestamp', 0.0)
            if t_stamp > 0:
                current_timestamp = t_stamp
                if first_timestamp is None: 
                    first_timestamp = t_stamp
            
            # --- РЕЖИМИ ТА АРМІНГ ---
            if msg_type == 'HEARTBEAT':
                if msg.get_srcComponent() == 1:
                    new_mode = mav.flightmode
                    is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    
                    # Зміна режиму
                    if new_mode and new_mode != current_mode:
                        if current_mode != "Невідомо":
                            add_event(f"🔄 Режим змінено на {new_mode}", current_timestamp, new_mode)
                        current_mode = new_mode
                        flight_modes.add(current_mode)
                    
                    # Арм / Дизарм
                    if is_armed and not was_armed:
                        add_event("🔴 Двигуни запущено", current_timestamp, current_mode)
                        was_armed = True
                        need_alt_reset = True
                    elif not is_armed and was_armed:
                        add_event("🟢 Двигуни зупинено", current_timestamp, current_mode)
                        was_armed = False

            # --- ПОЛІТ ---
            elif msg_type == 'VFR_HUD':
                if need_alt_reset or start_alt is None:
                    start_alt = msg.alt
                    need_alt_reset = False
                
                rel_alt = msg.alt - start_alt
                if rel_alt > max_alt: max_alt = rel_alt
                if msg.groundspeed > max_speed: max_speed = msg.groundspeed
                if msg.throttle > max_throttle: max_throttle = msg.throttle
                    
            elif msg_type == 'ATTITUDE':
                r_deg = abs(math.degrees(msg.roll))
                p_deg = abs(math.degrees(msg.pitch))
                if r_deg > max_roll: max_roll = r_deg
                if p_deg > max_pitch: max_pitch = p_deg
                    
            # --- ЖИВЛЕННЯ ---
            elif msg_type == 'SYS_STATUS':
                volt = msg.voltage_battery / 1000.0  
                curr = msg.current_battery / 100.0   
                if start_voltage is None and volt > 5.0: start_voltage = volt
                if 0 < volt < min_voltage: min_voltage = volt
                if curr > max_current: max_current = curr
                    
            # --- ЗВ'ЯЗОК, RC, TELEMETRY ---
            elif msg_type == 'RC_CHANNELS':
                if hasattr(msg, 'rssi') and 0 < msg.rssi < 255:
                    if msg.rssi < min_rssi: min_rssi = msg.rssi
                
                chans = [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw]
                for i in range(4):
                    if 500 < chans[i] < 2500:
                        if chans[i] < rc_min[i]: rc_min[i] = chans[i]
                        if chans[i] > rc_max[i]: rc_max[i] = chans[i]

            elif msg_type in ['RADIO', 'RADIO_STATUS']:
                telem_rssi = msg.rssi
                telem_remrssi = msg.remrssi
                    
            # --- GPS та 0.0.0.0 ---
            elif msg_type == 'GPS_RAW_INT':
                if msg.satellites_visible > 0: has_gps = True
                if msg.lat != 0 or msg.lon != 0:
                    gps_had_lock = True
                elif msg.lat == 0 and msg.lon == 0 and gps_had_lock:
                    add_event("🚨 КРИТИЧНО: GPS втратив позицію (0.0.0.0)", current_timestamp, current_mode, True)
                    gps_had_lock = False

            elif msg_type == 'GLOBAL_POSITION_INT':
                if msg.lat != 0 or msg.lon != 0:
                    ekf_had_origin = True
                elif msg.lat == 0 and msg.lon == 0 and ekf_had_origin:
                    add_event("🚨 КРИТИЧНО: EKF скинув координати в нуль", current_timestamp, current_mode, True)
                    ekf_had_origin = False
                    
            # --- ВІБРАЦІЇ ТА EKF ---
            elif msg_type == 'VIBRATION':
                if msg.vibration_x > max_vib_x: max_vib_x = msg.vibration_x
                if msg.vibration_y > max_vib_y: max_vib_y = msg.vibration_y
                if msg.vibration_z > max_vib_z: max_vib_z = msg.vibration_z
                clip_count = max(clip_count, msg.clipping_0, msg.clipping_1, msg.clipping_2)
                
            elif msg_type == 'EKF_STATUS_REPORT':
                if msg.compass_variance > ekf_compass: ekf_compass = msg.compass_variance
                if msg.velocity_variance > ekf_vel: ekf_vel = msg.velocity_variance
                
            elif msg_type == 'RAW_IMU':
                temp_c = msg.temperature / 100.0
                if temp_c > max_temp: max_temp = temp_c

            # --- СИСТЕМНІ ПОВІДОМЛЕННЯ ---
            elif msg_type == 'STATUSTEXT':
                try:
                    txt = msg.text.decode('utf-8') if isinstance(msg.text, bytes) else msg.text
                    is_err = msg.severity <= 4
                    prefix = "⚠️ ПОМИЛКА: " if is_err else "ℹ️ "
                    add_event(f"{prefix}{txt}", current_timestamp, current_mode, is_err)
                except: pass

        if min_voltage == 999.0: min_voltage = 0.0
        if start_voltage is None: start_voltage = 0.0
        if max_temp == -99.0: max_temp = 0.0

        # Форматування Часу
        duration_sec = 0
        if first_timestamp and current_timestamp:
            duration_sec = int(current_timestamp - first_timestamp)
        mins, secs = divmod(duration_sec, 60)
        
        timeline = []
        for ev in sorted(raw_timeline, key=lambda x: x['timestamp']):
            t_sec = int(ev['timestamp'] - (first_timestamp or 0))
            t_m, t_s = divmod(max(0, t_sec), 60)
            timeline.append({
                "time": f"{t_m:02d}:{t_s:02d}",
                "mode": ev['mode'],
                "text": ev['text'],
                "isError": ev['isError']
            })

        rssi_percent = round((min_rssi / 254.0) * 100) if min_rssi != 255 else 0
        modes_str = ", ".join(flight_modes) if flight_modes else "Невідомо"
        
        def format_rc(idx):
            if rc_min[idx] == 9999: return "—"
            if rc_max[idx] - rc_min[idx] < 10: return f"{rc_min[idx]} (Не задіяний)"
            return f"{rc_min[idx]} - {rc_max[idx]}"

        return {
            "success": True,
            "flight": {
                "durationText": f"{mins} хв {secs} с",
                "maxAltitude": round(max_alt, 1),
                "maxSpeed": round(max_speed, 1),
                "maxRoll": round(max_roll, 1),
                "maxPitch": round(max_pitch, 1),
                "modes": modes_str,
                "msgCount": message_count
            },
            "battery": {
                "armVoltage": round(start_voltage, 2),
                "minVoltage": round(min_voltage, 2),
                "maxCurrent": round(max_current, 2),
                "voltageSag": round(max(0, start_voltage - min_voltage), 2)
            },
            "radio": {
                "rssi": f"{rssi_percent}%" if min_rssi != 255 else "Немає",
                "maxThrottle": f"{max_throttle}%",
                "hasGps": "Присутній" if has_gps else "Без GPS",
                "telemRssi": str(telem_rssi) if telem_rssi is not None else "—",
                "telemRemRssi": str(telem_remrssi) if telem_remrssi is not None else "—",
                "ch1": format_rc(0),
                "ch2": format_rc(1),
                "ch3": format_rc(2),
                "ch4": format_rc(3)
            },
            "health": {
                "vibX": round(max_vib_x, 1),
                "vibY": round(max_vib_y, 1),
                "vibZ": round(max_vib_z, 1),
                "clipping": clip_count,
                "maxTemp": round(max_temp, 1),
                "ekfCompass": round(ekf_compass, 2)
            },
            "timeline": timeline
        }

    finally:
        try: os.unlink(temp.name)
        except: pass
