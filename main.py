from fastapi import FastAPI, UploadFile, File
from pymavlink import mavutil
import tempfile
import os
import math

app = FastAPI()

def parse_dbm(raw_val):
    if raw_val is None or raw_val == 0:
        return 0
    if raw_val < 0:
        return raw_val
    if raw_val > 127:
        return raw_val - 256
    if 0 < raw_val <= 100:
        return round(raw_val / 1.9 - 127)
    return -raw_val

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
        max_speed = 0.0
        max_roll = 0.0
        max_pitch = 0.0
        
        # Розрахунок висоти (Динамічний нуль землі)
        latest_baro_alt = 0.0
        ground_baro_alt = None
        is_currently_armed = False
        
        # Поточні параметри для хронології
        curr_alt = 0.0
        curr_dist = 0.0
        curr_voltage = 0.0
        curr_amp = 0.0
        curr_rssi_pct = 0
        curr_dbm = 0
        
        # Дальномір / Локальна позиція
        max_rf_alt = 0.0
        max_dist = 0.0
        has_rangefinder = False
        rangefinder_failed_flag = False
        
        # Зв'язок & dBm
        min_rssi = 255 
        min_dbm = 0 
        telem_rssi_raw = None
        telem_remrssi_raw = None
        
        # Керування (PWM)
        rc_min = {i: 9999 for i in range(1, 9)}
        rc_max = {i: 0 for i in range(1, 9)}
        last_rc_state = {i: 0 for i in range(1, 9)}
        max_throttle = 0
        
        # Батарея 6S
        min_voltage = 999.0
        max_current = 0.0
        start_voltage = None 
        voltage_at_land_mode = None
        reboot_or_second_battery = False
        
        # Датчики та Вібрації
        max_vib_x, max_vib_y, max_vib_z = 0.0, 0.0, 0.0
        clip_count = 0
        max_temp = -99.0
        
        # Оптична навігація
        vnav_quality_min_loiter = 999
        vnav_quality_max_loiter = 0
        vnav_samples = 0
        loiter_has_origin = False
        loiter_origin_failed = False
        
        has_gps = False
        first_timestamp = None
        current_timestamp = 0.0 
        
        # Стан польоту
        was_armed = False
        current_mode = "Невідомо"
        flight_modes = set()
        land_mode_triggered = False
        
        raw_timeline = []

        def add_event(text, t_stamp, mode, is_error=False):
            # Обчислюємо висоту від замороженої точки землі
            alt_calc = max(0.0, latest_baro_alt - (ground_baro_alt if ground_baro_alt is not None else latest_baro_alt))
            display_alt = curr_alt if curr_alt > 0 else alt_calc
            
            raw_timeline.append({
                "timestamp": t_stamp or 0,
                "mode": mode,
                "alt": f"{round(display_alt, 1)} м",
                "dist": f"{round(curr_dist, 1)} м" if curr_dist > 0 else "0.0 м",
                "volt": round(curr_voltage, 1) if curr_voltage > 0 else "—",
                "curr": f"{round(curr_amp, 1)} А" if curr_amp > 0 else "0.0 А",
                "rssi": f"{curr_rssi_pct}%" if curr_rssi_pct > 0 else "—",
                "dbm": f"{round(curr_dbm)} dBm" if curr_dbm != 0 else "—",
                "text": text,
                "isError": is_error
            })

        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None: break
            
            message_count += 1
            msg_type = msg.get_type()
            
            t_stamp = getattr(msg, '_timestamp', 0.0)
            if t_stamp > 0:
                current_timestamp = t_stamp
                if first_timestamp is None: 
                    first_timestamp = t_stamp

            # --- 1. СТАН ДВИГУНІВ ТА РЕЖИМИ ---
            if msg_type == 'HEARTBEAT':
                if msg.get_srcComponent() == 1:
                    new_mode = mav.flightmode
                    is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    is_currently_armed = is_armed
                    
                    if new_mode and new_mode != current_mode:
                        if current_mode != "Невідомо":
                            add_event(f"🔄 Режим змінено на {new_mode}", current_timestamp, new_mode)
                        current_mode = new_mode
                        flight_modes.add(current_mode)
                        if current_mode == "LAND":
                            land_mode_triggered = True
                    
                    if is_armed and not was_armed:
                        # Фіксуємо земну висоту строго в момент запуску
                        if latest_baro_alt > 0:
                            ground_baro_alt = latest_baro_alt
                        add_event("🟢 Двигуни запущено", current_timestamp, current_mode)
                        was_armed = True
                    elif not is_armed and was_armed:
                        add_event("🔴 Двигуни зупинено", current_timestamp, current_mode)
                        was_armed = False

            # --- 2. ЖИВЛЕННЯ & ПЕРЕПІДКТЮЧЕННЯ ---
            elif msg_type == 'SYS_STATUS':
                volt = msg.voltage_battery / 1000.0  
                curr = msg.current_battery / 100.0   
                
                if volt > 5.0:
                    if curr_voltage > 5.0 and curr_voltage < 22.0 and volt > 24.5:
                        reboot_or_second_battery = True
                        ground_baro_alt = latest_baro_alt
                        add_event("🔋 Заміна батареї / Новий політ", current_timestamp, current_mode)
                    
                    curr_voltage = volt
                    if start_voltage is None: start_voltage = volt
                    if volt < min_voltage: min_voltage = volt
                
                if curr >= 0:
                    curr_amp = curr
                    if curr > max_current: max_current = curr
                
                if current_mode == 'LAND' and voltage_at_land_mode is None:
                    voltage_at_land_mode = volt

                if current_mode == 'LOITER':
                    vnav_val = getattr(msg, 'load', 0) / 10.0
                    if vnav_val > 0:
                        vnav_samples += 1
                        if vnav_val < vnav_quality_min_loiter: vnav_quality_min_loiter = vnav_val
                        if vnav_val > vnav_quality_max_loiter: vnav_quality_max_loiter = vnav_val

            # --- 3. РОЗРАХУНОК ВИСОТИ ТА ДАЛЬНОСТІ ---
            elif msg_type == 'VFR_HUD':
                latest_baro_alt = msg.alt
                
                # Поки дрон на землі, постійно оновлюємо нуль
                if not is_currently_armed or ground_baro_alt is None:
                    ground_baro_alt = msg.alt

                rel_baro_alt = max(0.0, latest_baro_alt - ground_baro_alt)
                curr_alt = rel_baro_alt
                
                if curr_alt > max_alt and curr_alt < 1000.0:
                    max_alt = curr_alt
                
                if msg.groundspeed > max_speed: max_speed = msg.groundspeed
                if msg.throttle > max_throttle: max_throttle = msg.throttle

            elif msg_type == 'ALTITUDE':
                if hasattr(msg, 'altitude_relative') and not math.isnan(msg.altitude_relative):
                    if msg.altitude_relative >= 0:
                        curr_alt = msg.altitude_relative
                        if curr_alt > max_alt and curr_alt < 1000.0:
                            max_alt = curr_alt

            elif msg_type in ['LOCAL_POSITION_NED', 'POSITION_TARGET_LOCAL_NED']:
                x = getattr(msg, 'x', 0.0)
                y = getattr(msg, 'y', 0.0)
                d_val = math.sqrt(x*x + y*y)
                if 0.0 <= d_val <= 10000.0:
                    curr_dist = d_val
                    if curr_dist > max_dist:
                        max_dist = curr_dist

            elif msg_type in ['RANGEFINDER', 'DISTANCE_SENSOR']:
                rf_dist = getattr(msg, 'distance', 0)
                if msg_type == 'DISTANCE_SENSOR':
                    rf_dist = getattr(msg, 'current_distance', 0) / 100.0
                
                if 0.1 <= rf_dist <= 50.0 and not rangefinder_failed_flag:
                    has_rangefinder = True
                    curr_alt = rf_dist
                    if rf_dist > max_rf_alt: 
                        max_rf_alt = rf_dist

            # --- 4. ЗВ'ЯЗОК & RC ---
            elif msg_type == 'RC_CHANNELS':
                if hasattr(msg, 'rssi') and 0 < msg.rssi < 255:
                    if msg.rssi < min_rssi: min_rssi = msg.rssi
                    curr_rssi_pct = round((msg.rssi / 254.0) * 100)
                
                chans = [
                    msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                    getattr(msg, 'chan5_raw', 0), getattr(msg, 'chan6_raw', 0),
                    getattr(msg, 'chan7_raw', 0), getattr(msg, 'chan8_raw', 0)
                ]
                
                for ch_num in range(1, 9):
                    val = chans[ch_num - 1]
                    if 800 < val < 2200:
                        if val < rc_min[ch_num]: rc_min[ch_num] = val
                        if val > rc_max[ch_num]: rc_max[ch_num] = val
                        
                        if ch_num >= 5:
                            prev = last_rc_state[ch_num]
                            if prev > 0 and abs(val - prev) > 250:
                                state_str = "АКТИВНО" if val > 1600 else ("СЕРЕДНЄ" if 1300 <= val <= 1600 else "ВИМК")
                                add_event(f"🎮 CH{ch_num} переведено в {state_str} ({val} us)", current_timestamp, current_mode)
                            last_rc_state[ch_num] = val

            elif msg_type in ['RADIO', 'RADIO_STATUS']:
                telem_rssi_raw = msg.rssi
                telem_remrssi_raw = msg.remrssi
                dbm_val = parse_dbm(msg.rssi)
                curr_dbm = dbm_val
                if min_dbm == 0 or dbm_val < min_dbm:
                    min_dbm = dbm_val

            elif msg_type == 'ATTITUDE':
                r_deg = abs(math.degrees(msg.roll))
                p_deg = abs(math.degrees(msg.pitch))
                if r_deg > max_roll: max_roll = r_deg
                if p_deg > max_pitch: max_pitch = p_deg

            elif msg_type == 'GLOBAL_POSITION_INT':
                if hasattr(msg, 'relative_alt') and msg.relative_alt != 0:
                    rel_g = msg.relative_alt / 1000.0
                    if rel_g >= 0:
                        curr_alt = rel_g
                if current_mode == 'LOITER':
                    if msg.lat != 0 or msg.lon != 0:
                        loiter_has_origin = True
                    elif msg.lat == 0 and msg.lon == 0 and not loiter_has_origin:
                        loiter_origin_failed = True

            elif msg_type == 'VIBRATION':
                if msg.vibration_x > max_vib_x: max_vib_x = msg.vibration_x
                if msg.vibration_y > max_vib_y: max_vib_y = msg.vibration_y
                if msg.vibration_z > max_vib_z: max_vib_z = msg.vibration_z
                clip_count = max(clip_count, msg.clipping_0, msg.clipping_1, msg.clipping_2)
                
            elif msg_type == 'STATUSTEXT':
                try:
                    txt = msg.text.decode('utf-8') if isinstance(msg.text, bytes) else msg.text
                    if "No rangefinder" in txt or "VISP: No rangefinder" in txt:
                        rangefinder_failed_flag = True
                    is_err = msg.severity <= 4
                    prefix = "⚠️ ПОМИЛКА: " if is_err else "ℹ️ "
                    add_event(f"{prefix}{txt}", current_timestamp, current_mode, is_err)
                except: pass

        # Завершення
        if min_voltage == 999.0: min_voltage = 0.0
        if start_voltage is None: start_voltage = 0.0
        if vnav_quality_min_loiter == 999: vnav_quality_min_loiter = 0

        final_max_altitude = max_rf_alt if (has_rangefinder and max_rf_alt > 0 and not rangefinder_failed_flag) else max_alt
        duration_sec = int(current_timestamp - first_timestamp) if first_timestamp and current_timestamp else 0
        mins, secs = divmod(duration_sec, 60)
        
        timeline = []
        for ev in sorted(raw_timeline, key=lambda x: x['timestamp']):
            t_sec = int(ev['timestamp'] - (first_timestamp or 0))
            t_m, t_s = divmod(max(0, t_sec), 60)
            timeline.append({
                "time": f"{t_m:02d}:{t_s:02d}",
                "mode": ev['mode'],
                "alt": ev['alt'],
                "dist": ev['dist'],
                "volt": ev['volt'],
                "curr": ev['curr'],
                "rssi": ev['rssi'],
                "dbm": ev['dbm'],
                "text": ev['text'],
                "isError": ev['isError']
            })

        rssi_percent = round((min_rssi / 254.0) * 100) if min_rssi != 255 else 0
        modes_str = ", ".join(flight_modes) if flight_modes else "Невідомо"

        # AI Висновок
        ai_alerts = []
        is_critical = False

        if reboot_or_second_battery:
            ai_alerts.append("ℹ️ <b>Зафіксовано заміну батареї:</b> Напруга зросла до 25.2V. Нуль висоти перекалібровано.")

        if rangefinder_failed_flag:
            ai_alerts.append("📡 <b>Відвалився далекомір (Rangefinder):</b> Система VISP втратила зв'язок із сенсором. Висота розрахована за барометром.")

        if 'LOITER' in flight_modes:
            if loiter_origin_failed and not loiter_has_origin:
                ai_alerts.append("👁 <b>Збій оптичного захоплення в Loiter:</b> Модуль не зміг відбити точку 0.0.0.0.")
                is_critical = True
            elif loiter_has_origin:
                ai_alerts.append("✅ <b>Оптична навігація в Loiter:</b> Точку 0.0.0.0 успішно зафіксовано оптикою.")

            if vnav_samples > 0:
                if vnav_quality_min_loiter < 40:
                    ai_alerts.append(f"⚠️ <b>Низька якість Оптичної Навігації:</b> Якість розпізнавання падала до {round(vnav_quality_min_loiter)}%.")
                else:
                    ai_alerts.append(f"👁 <b>Якість Оптичної Навігації:</b> {round(vnav_quality_min_loiter)}%–{round(vnav_quality_max_loiter)}%.")

        if 0 < min_voltage <= 16.8:
            is_critical = True
            if land_mode_triggered:
                ai_alerts.append(f"🪫 <b>Посадка за розрядом:</b> Напруга впала до {round(min_voltage,1)}V (поріг 16.8V). Автопілот примусово перевів борт у LAND.")
            else:
                ai_alerts.append(f"🚨 <b>Критичний розряд без LAND:</b> Напруга просіла до {round(min_voltage,1)}V.")
        elif 16.8 < min_voltage < 18.0:
            ai_alerts.append(f"🔋 <b>Глибока просадка:</b> Напруга падала до {round(min_voltage,1)}V.")

        if min_dbm <= -128 or (telem_rssi_raw == 0 and telem_remrssi_raw == 0):
            ai_alerts.append("📡 <b>-128 dBm: Втрата відео та телеметрії.</b> Повний розрив каналу зв'язку.")
            is_critical = True
        elif -128 < min_dbm <= -90:
            ai_alerts.append(f"📡 <b>{round(min_dbm)} dBm: Втрата відеосигналу (-90...-100 dBm).</b> Телеметрія була присутня.")
        elif -90 < min_dbm <= -85:
            ai_alerts.append(f"📶 <b>{round(min_dbm)} dBm: Підсипання відео (-85...-90 dBm).</b> Граничний рівень відеосигналу.")
        elif min_dbm < 0:
            ai_alerts.append(f"✅ <b>Рівень відео та телеметрії в нормі:</b> Мінімальний сигнал {round(min_dbm)} dBm.")

        if was_armed:
            ai_alerts.append("❗️ <b>Обрив логу під навантаженням:</b> Файл закінчився при працюючих двигунах.")
            is_critical = True

        if max_roll > 80 or max_pitch > 80:
            ai_alerts.append(f"🔄 <b>Перевороти в повітрі:</b> Нахил {max(max_roll, max_pitch)}°.")
            is_critical = True

        if is_critical:
            ai_verdict = "⚠️ БОРТ ВТРАЧЕНО АБО СТАЛАСЯ АВАРІЙНА СИТУАЦІЯ:"
        elif len(ai_alerts) > 0:
            ai_verdict = "📊 РЕЗУЛЬТАТИ АНАЛІЗУ ПОЛЬОТУ:"
        else:
            ai_verdict = "✅ Політ пройшов у штатному режимі."

        return {
            "success": True,
            "ai": {
                "verdict": ai_verdict,
                "isCritical": is_critical,
                "alerts": ai_alerts
            },
            "flight": {
                "durationText": f"{mins} хв {secs} с",
                "maxAltitude": round(final_max_altitude, 1),
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
                "hasGps": "Без GPS (Оптична навігація)" if not has_gps else "GPS Присутній",
                "telemRssi": f"{round(min_dbm)} dBm" if min_dbm != 0 else "—",
                "telemRemRssi": str(telem_remrssi_raw) if telem_remrssi_raw is not None else "—"
            },
            "health": {
                "vibX": round(max_vib_x, 1),
                "vibY": round(max_vib_y, 1),
                "vibZ": round(max_vib_z, 1),
                "clipping": clip_count,
                "maxTemp": round(max_temp, 1),
                "engineLoadLoiter": f"{round(vnav_quality_min_loiter)}% – {round(vnav_quality_max_loiter)}%" if vnav_samples > 0 else "Немає даних"
            },
            "timeline": timeline
        }

    finally:
        try: os.unlink(temp.name)
        except: pass
