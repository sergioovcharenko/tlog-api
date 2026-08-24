from fastapi import FastAPI, UploadFile, File
from pymavlink import mavutil
import tempfile
import os

app = FastAPI()

@app.get("/")
def root():
    return {"status": "ok"}

@app.get("/health")
def health():
    return {"status": "ok"}

@app.post("/analyze")
async def analyze(file: UploadFile = File(...)):
    data = await file.read()
    temp = tempfile.NamedTemporaryFile(delete=False, suffix=".tlog")
    
    try:
        temp.write(data)
        temp.close()

        mav = mavutil.mavlink_connection(temp.name)
        
        message_count = 0
        
        # Висота та швидкість без GPS (за барометром/EKF)
        max_alt = 0.0  
        start_alt = None 
        max_speed = 0.0
        
        min_voltage = 999.0
        max_current = 0.0
        
        has_gps = False
        
        first_time_ms = None
        last_time_ms = None
        
        autopilot_type = "ArduPilot"

        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None:
                break
            
            message_count += 1
            msg_type = msg.get_type()

            # Фіксуємо час у мілісекундах для тривалості польоту
            if hasattr(msg, 'time_boot_ms'):
                if first_time_ms is None:
                    first_time_ms = msg.time_boot_ms
                last_time_ms = msg.time_boot_ms
            
            # 1. Політ (VFR_HUD працює від барометра, GPS не потрібен)
            if msg_type == 'VFR_HUD':
                # Запам'ятовуємо висоту при увімкненні (0 метрів для нас)
                if start_alt is None:
                    start_alt = msg.alt
                
                # Рахуємо відносну висоту
                rel_alt = msg.alt - start_alt
                if rel_alt > max_alt:
                    max_alt = rel_alt
                    
                if msg.groundspeed > max_speed:
                    max_speed = msg.groundspeed
                    
            # 2. Батарея
            elif msg_type == 'SYS_STATUS':
                volt = msg.voltage_battery / 1000.0  
                curr = msg.current_battery / 100.0   
                
                if 0 < volt < min_voltage:
                    min_voltage = volt
                if curr > max_current:
                    max_current = curr
                    
            # 3. GPS (якщо раптом хтось підключить, скрипт це помітить)
            elif msg_type == 'GPS_RAW_INT':
                if msg.satellites_visible > 0:
                    has_gps = True

        if min_voltage == 999.0: min_voltage = 0.0

        # Точний час
        duration_sec = 0
        if first_time_ms and last_time_ms:
            duration_sec = (last_time_ms - first_time_ms) // 1000
            
        mins = duration_sec // 60
        secs = duration_sec % 60
        duration_text = f"{mins} хв {secs} с" if duration_sec > 0 else "Невідомо"

        # AI Оцінка (більше не знімає бали за відсутність GPS)
        score = 100
        recommendations = []
        
        # Приклад логіки для батареї (можна буде налаштувати під ваші збірки)
        if 0 < min_voltage < 14.0: 
            score -= 10
            recommendations.append("Фіксувалась сильна просадка напруги (нижче 14V).")

        summary = "Політ без GPS. Параметри в нормі." if score == 100 else "Виявлені відхилення по живленню."

        return {
            "success": True,
            "ai": {
                "score": score,
                "summary": summary,
                "recommendations": recommendations
            },
            "flight": {
                "durationText": duration_text,
                "maxAltitude": round(max_alt, 2),
                "maxSpeed": round(max_speed, 2),
                "distanceKm": 0,
                "autopilot": autopilot_type,
                "firmware": f"Рядків: {message_count}"
            },
            "battery": {
                "armVoltage": 0,
                "minVoltage": round(min_voltage, 2),
                "maxCurrent": round(max_current, 2),
                "voltageSag": 0,
                "statusText": "OK" if min_voltage >= 14.0 else "Увага: АКБ"
            },
            "gps": {
                "satellitesMin": "—",
                "satellitesMax": "—",
                "maxHdop": "—",
                "status": "NO_GPS",
                "statusText": "Відсутній (Без GPS)" if not has_gps else "Присутній"
            },
            "ekf": {}, "vibration": {}, "failsafe": {},
            "timeline": []
        }

    finally:
        try:
            os.unlink(temp.name)
        except:
            pass
