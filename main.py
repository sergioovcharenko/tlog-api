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
        max_speed = 0.0
        max_roll = 0.0
        max_pitch = 0.0
        min_rssi = 255 
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
        first_time_ms = None
        last_time_ms = None
        current_time_ms = 0 # Глобальний трекер часу
        
        # Стан польоту
        was_armed = False
        last_mode = None
        flight_modes = set()
        
        # Детектор 0.0.0.0
        had_zero_coords = False
        
        # Хронологія
        raw_timeline = []

        def add_event(title, text, time_ms, is_error=False):
            raw_timeline.append({
                "time_ms": time_ms or 0,
                "title": title,
                "text": text,
                "isError": is_error
            })

        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None: break
            
            message_count += 1
            msg_type = msg.get_type()
            
            # Оновлюємо глобальний час
            t_ms = getattr(msg, 'time_boot_ms', 0)
            if t_ms > 0:
                current_time_ms = t_ms
                if first_time_ms is None: first_time_ms = t_ms
                last_time_ms = t_ms
            
            # --- РЕЖИМИ ТА АРМІНГ ---
            if msg_type == 'HEARTBEAT':
                if msg.get_srcComponent() == 1:
                    current_mode = mav.flightmode
                    is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    
                    if is_armed and not was_armed:
                        add_event("🔴 ARMED", "Дрон озброєно (двигуни активні)", current_time_ms)
                        was_armed = True
                    elif not is_armed and was_armed:
                        add_event("🟢 DISARMED", "Дрон знято з охорони", current_time_ms)
                        was_armed = False
                        
                    if current_mode and current_mode != last_mode:
                        if last_mode is not None:
                            add_event("🔄 РЕЖИМ", f"Зміна режиму: {last_mode} -> {current_mode}", current_time_ms)
                        flight_modes.add(current_mode)
                        last_mode = current_mode

            # --- ПОЛІТ ---
            elif msg_type == 'VFR_HUD':
                if start_alt is None: start_alt = msg.alt
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
                    
            # --- ЗВ'ЯЗОК ТА GPS ---
            elif msg_type == 'RC_CHANNELS':
                if hasattr(msg, 'rssi') and 0 < msg.rssi < 255:
                    if msg.rssi < min_rssi: min_rssi = msg.rssi
                    
            elif msg_type == 'GPS_RAW_INT':
                if msg.satellites_visible > 0: has_gps = True
                # Перевірка на нульові координати від самого GPS модуля
                if msg.lat == 0 and msg.lon == 0:
                    if not had_zero_coords:
                        mode_info = f" (Режим: {last_mode})" if last_mode else ""
                        add_event("🚨 0.0.0.0", f"КРИТИЧНО: GPS втратив позицію (0.0.0.0){mode_info}", current_time_ms, True)
                        had_zero_coords = True
                else:
                    had_zero_coords = False

            elif msg_type == 'GLOBAL_POSITION_INT':
                # Перевірка на нульові координати від EKF (глобальна оцінка позиції)
                if msg.lat == 0 and msg.lon == 0:
                    if not had_zero_coords:
                        mode_info = f" (Режим: {last_mode})" if last_mode else ""
                        add_event("🚨 EKF 0.0.0.0", f"КРИТИЧНО: EKF скинув координати в нуль{mode_info}", current_time_ms, True)
                        had_zero_coords = True
                else:
                    had_zero_coords = False
                    
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
                    add_event("⚠️ ПОМИЛКА" if is_err else "ℹ️ ІНФО", txt, current_time_ms, is_err)
                except: pass

        if min_voltage == 999.0: min_voltage = 0.0
        if start_voltage is None: start_voltage = 0.0
        if max_temp == -99.0: max_temp = 0.0

        # Час
        duration_sec = 0
        if first_time_ms and last_time_ms:
            duration_sec = (last_time_ms - first_time_ms) // 1000
        mins, secs = divmod(duration_sec, 60)
        
        # Форматуємо Хронологію
        timeline = []
        for ev in sorted(raw_timeline, key=lambda x: x['time_ms']):
            t_sec = (ev['time_ms'] - (first_time_ms or 0)) // 1000
            t_m, t_s = divmod(max(0, t_sec), 60)
            timeline.append({
                "time": f"{t_m:02d}:{t_s:02d}",
                "title": ev['title'],
                "text": ev['text'],
                "isError": ev['isError']
            })

        rssi_percent = round((min_rssi / 254.0) * 100) if min_rssi != 255 else 0
        modes_str = ", ".join(flight_modes) if flight_modes else "Невідомо"

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
                "hasGps": "Присутній" if has_gps else "Без GPS"
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
