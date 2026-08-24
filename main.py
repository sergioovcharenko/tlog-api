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
        
        # Зв'язок & dBm
        min_rssi = 255 
        min_dbm = 0 
        telem_rssi_raw = None
        telem_remrssi_raw = None
        
        # Керування (PWM)
        rc_min = [9999, 9999, 9999, 9999]
        rc_max = [0, 0, 0, 0]
        max_throttle = 0
        
        # Батарея 6S (25.2V -> 16.8V)
        min_voltage = 999.0
        max_current = 0.0
        start_voltage = None 
        voltage_at_land_mode = None
        
        # Датчики та Вібрації
        max_vib_x, max_vib_y, max_vib_z = 0.0, 0.0, 0.0
        clip_count = 0
        max_temp = -99.0
        ekf_compass = 0.0
        ekf_vel = 0.0
        
        # Оптична навігація (Якість оптичного модуля в Loiter на 50-200м)
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
                    
                    if new_mode and new_mode != current_mode:
                        if current_mode != "Невідомо":
                            add_event(f"🔄 Режим змінено на {new_mode}", current_timestamp, new_mode)
                        current_mode = new_mode
                        flight_modes.add(current_mode)
                        if current_mode == "LAND":
                            land_mode_triggered = True
                    
                    if is_armed and not was_armed:
                        add_event("🔴 Двигуни запущено", current_timestamp, current_mode)
                        was_armed = True
                        need_alt_reset = True
                    elif not is_armed and was_armed:
                        add_event("🟢 Двигуни зупинено", current_timestamp, current_mode)
                        was_armed = False

            # --- ПОЛІТ & АНАЛІЗ ЯКОСТІ ОПТИКИ В LOITER ---
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
                    
            # --- ЖИВЛЕННЯ (6S: 25.2V -> 16.8V) ТА ЯКІСТЬ ОПТИЧНОЇ НАВІГАЦІЇ ---
            elif msg_type == 'SYS_STATUS':
                volt = msg.voltage_battery / 1000.0  
                curr = msg.current_battery / 100.0   
                if start_voltage is None and volt > 5.0: start_voltage = volt
                if 0 < volt < min_voltage: min_voltage = volt
                if curr > max_current: max_current = curr
                
                if current_mode == 'LAND' and voltage_at_land_mode is None:
                    voltage_at_land_mode = volt

                # Беремо Engine Load як показник якості роботи оптичного модуля у Loiter (50-200м)
                if current_mode == 'LOITER':
                    rel_alt_now = max_alt # або точна висота з VFR_HUD
                    if 50.0 <= rel_alt_now <= 200.0:
                        vnav_val = getattr(msg, 'load', 0) / 10.0 # Отримуємо якість у %
                        if vnav_val > 0:
                            vnav_samples += 1
                            if vnav_val < vnav_quality_min_loiter: vnav_quality_min_loiter = vnav_val
                            if vnav_val > vnav_quality_max_loiter: vnav_quality_max_loiter = vnav_val
                    
            # --- ЗВ'ЯЗОК, RC, TELEMETRY (dBm) ---
            elif msg_type == 'RC_CHANNELS':
                if hasattr(msg, 'rssi') and 0 < msg.rssi < 255:
                    if msg.rssi < min_rssi: min_rssi = msg.rssi
                chans = [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw]
                for i in range(4):
                    if 500 < chans[i] < 2500:
                        if chans[i] < rc_min[i]: rc_min[i] = chans[i]
                        if chans[i] > rc_max[i]: rc_max[i] = chans[i]

            elif msg_type in ['RADIO', 'RADIO_STATUS']:
                telem_rssi_raw = msg.rssi
                telem_remrssi_raw = msg.remrssi
                dbm_val = msg.rssi if msg.rssi < 0 else (msg.rssi / 2.0 - 121 if msg.rssi < 200 else -msg.rssi)
                if min_dbm == 0 or dbm_val < min_dbm:
                    min_dbm = dbm_val

            # --- ВІДБИТТЯ ТОЧКИ 0,0,0,0 В LOITER ---
            elif msg_type == 'GLOBAL_POSITION_INT':
                if current_mode == 'LOITER':
                    if msg.lat != 0 or msg.lon != 0:
                        loiter_has_origin = True
                    elif msg.lat == 0 and msg.lon == 0 and not loiter_has_origin:
                        loiter_origin_failed = True

            # --- ВІБРАЦІЇ ---
            elif msg_type == 'VIBRATION':
                if msg.vibration_x > max_vib_x: max_vib_x = msg.vibration_x
                if msg.vibration_y > max_vib_y: max_vib_y = msg.vibration_y
                if msg.vibration_z > max_vib_z: max_vib_z = msg.vibration_z
                clip_count = max(clip_count, msg.clipping_0, msg.clipping_1, msg.clipping_2)
                
            elif msg_type == 'STATUSTEXT':
                try:
                    txt = msg.text.decode('utf-8') if isinstance(msg.text, bytes) else msg.text
                    is_err = msg.severity <= 4
                    prefix = "⚠️ ПОМИЛКА: " if is_err else "ℹ️ "
                    add_event(f"{prefix}{txt}", current_timestamp, current_mode, is_err)
                except: pass

        # Завершення
        if min_voltage == 999.0: min_voltage = 0.0
        if start_voltage is None: start_voltage = 0.0
        if vnav_quality_min_loiter == 999: vnav_quality_min_loiter = 0

        duration_sec = int(current_timestamp - first_timestamp) if first_timestamp and current_timestamp else 0
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

        # ==========================================
        # 🤖 AI ВИСНОВОК: ЯКІСТЬ ОПТИКИ + 6S + LOITER
        # ==========================================
        ai_alerts = []
        is_critical = False

        # 1. Аналіз роботи та якості Оптичної Навігації в LOITER (50-200м)
        if 'LOITER' in flight_modes:
            if loiter_origin_failed and not loiter_has_origin:
                ai_alerts.append("👁 <b>Збій оптичного захоплення в Loiter:</b> Модуль не змог відбити точку 0.0.0.0.")
                is_critical = True
            elif loiter_has_origin:
                ai_alerts.append("✅ <b>Оптична навігація в Loiter:</b> Точку 0.0.0.0 успішно зафіксовано оптикою.")

            if vnav_samples > 0:
                if vnav_quality_min_loiter < 40:
                    ai_alerts.append(f"⚠️ <b>Низька якість Оптичної Навігації:</b> На висоті 50-200м якість розпізнавання падала до {round(vnav_quality_min_loiter)}%. Високий ризик дрейфу або зриву точки.")
                else:
                    ai_alerts.append(f"👁 <b>Якість Оптичної Навігації (Loiter 50-200м):</b> Працювала стабільно на рівні {round(vnav_quality_min_loiter)}%–{round(vnav_quality_max_loiter)}%.")

        # 2. Живлення 6S (25.2V -> 16.8V LAND)
        if 0 < min_voltage <= 16.8:
            is_critical = True
            if land_mode_triggered:
                ai_alerts.append(f"🪫 <b>Посадка за розрядом:</b> Напруга впала до {round(min_voltage,1)}V (поріг 16.8V). Автопілот примусово перевів борт у режим LAND.")
            else:
                ai_alerts.append(f"🚨 <b>Критичний розряд без LAND:</b> Напруга просіла до {round(min_voltage,1)}V (нижче 16.8V), але режим LAND не встиг відпрацювати.")
        elif 16.8 < min_voltage < 18.0:
            ai_alerts.append(f"🔋 <b>Глибока просадка:</b> Напруга падала до {round(min_voltage,1)}V (межа посадки 16.8V).")

        # 3. dBm Відео та Телеметрії
        if min_dbm == -128 or (telem_rssi_raw == 0 and telem_remrssi_raw == 0):
            ai_alerts.append("📡 <b>-128 dBm: Повна втрата зв'язку:</b> Відсутній сигнал відео та телеметрії.")
            is_critical = True
        elif -100 <= min_dbm < -90:
            ai_alerts.append(f"📡 <b>{round(min_dbm)} dBm: Втрата відеосигналу.</b> Телеметрія залишалась присутньою, але відеоканал був втрачений.")
        elif -90 <= min_dbm < -85:
            ai_alerts.append(f"📶 <b>{round(min_dbm)} dBm: Підсипання відео.</b> Граничний рівень сигналу, спостерігалися перешкоди.")
        elif min_dbm < 0 and min_dbm >= -85:
            ai_alerts.append(f"✅ <b>Рівень сигналу в нормі:</b> Рівень dBm не опускався нижче {round(min_dbm)} dBm.")

        # 4. Обрив логу та нахили
        if was_armed:
            ai_alerts.append("❗️ <b>Обрив логу під навантаженням:</b> Файл закінчився при працюючих двигунах. Ознака фізичного знищення або влучання.")
            is_critical = True

        if max_roll > 80 or max_pitch > 80:
            ai_alerts.append(f"🔄 <b>Перевороти в повітрі:</b> Зафіксовано нахил {max(max_roll, max_pitch)}°.")
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
                "hasGps": "Без GPS (Оптична навігація)" if not has_gps else "GPS Присутній",
                "telemRssi": f"{round(min_dbm)} dBm" if min_dbm != 0 else "—",
                "telemRemRssi": str(telem_remrssi_raw) if telem_remrssi_raw is not None else "—",
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
                "engineLoadLoiter": f"{round(vnav_quality_min_loiter)}% – {round(vnav_quality_max_loiter)}%" if vnav_samples > 0 else "Немає даних"
            },
            "timeline": timeline
        }

    finally:
        try: os.unlink(temp.name)
        except: pass
