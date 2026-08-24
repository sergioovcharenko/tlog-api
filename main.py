from fastapi import FastAPI, UploadFile, File
from pymavlink import mavutil
import tempfile
import os

app = FastAPI()

@app.get("/")
def root():
    return {"status": "ok", "service": "TLOG AI Analyzer"}

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

        # Відкриваємо TLOG
        mav = mavutil.mavlink_connection(temp.name)
        
        # Змінні для збору статистики
        message_count = 0
        max_alt = 0.0
        max_speed = 0.0
        min_voltage = 999.0
        max_current = 0.0
        satellites_min = 99
        satellites_max = 0
        max_hdop = 0.0
        
        autopilot_type = "Невідомо"
        firmware_version = ""

        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None:
                break
            
            message_count += 1
            msg_type = msg.get_type()

            # 1. Автопілот та прошивка
            if msg_type == 'AUTOPILOT_VERSION':
                autopilot_type = "ArduPilot" if msg.flight_sw_version > 0 else "PX4 / Інше"
            
            # 2. Політ (Висота та швидкість)
            elif msg_type == 'VFR_HUD':
                if msg.alt > max_alt:
                    max_alt = msg.alt
                if msg.groundspeed > max_speed:
                    max_speed = msg.groundspeed
                    
            # 3. Батарея
            elif msg_type == 'SYS_STATUS':
                volt = msg.voltage_battery / 1000.0  # mV в V
                curr = msg.current_battery / 100.0   # cA в A
                
                if volt > 0 and volt < min_voltage:
                    min_voltage = volt
                if curr > max_current:
                    max_current = curr
                    
            # 4. GPS
            elif msg_type == 'GPS_RAW_INT':
                sats = msg.satellites_visible
                hdop = msg.eph / 100.0
                
                if sats < satellites_min and sats > 0:
                    satellites_min = sats
                if sats > satellites_max:
                    satellites_max = sats
                if hdop > max_hdop:
                    max_hdop = hdop

        # Убезпечення від некоректних даних
        if min_voltage == 999.0:
            min_voltage = 0.0
        if satellites_min == 99:
            satellites_min = 0

        # AI Висновок (Базова логіка)
        score = 100
        recommendations = []
        
        if satellites_min < 10:
            score -= 20
            recommendations.append("Низька кількість супутників. Перевірте розташування GPS-модуля.")
        if min_voltage < 14.0: # Умовно для 4S
            score -= 10
            recommendations.append("Напруга падала нижче критичної норми.")
            
        summary = "Політ пройшов у штатному режимі." if score > 80 else "Виявлено проблеми під час польоту, перевірте рекомендації."

        # Формуємо відповідь для HTML
        return {
            "success": True,
            "ai": {
                "score": score,
                "summary": summary,
                "recommendations": recommendations
            },
            "flight": {
                "durationText": f"~ {message_count // 50} сек", # Приблизно (залежить від частоти телеметрії)
                "maxAltitude": round(max_alt, 2),
                "maxSpeed": round(max_speed, 2),
                "distanceKm": 0, # Потребує складніших розрахунків координат
                "autopilot": autopilot_type,
                "firmware": firmware_version
            },
            "battery": {
                "armVoltage": 0,
                "minVoltage": round(min_voltage, 2),
                "maxCurrent": round(max_current, 2),
                "voltageSag": 0,
                "statusText": "OK" if min_voltage > 14.0 else "Критичний розряд"
            },
            "gps": {
                "satellitesMin": satellites_min,
                "satellitesMax": satellites_max,
                "maxHdop": round(max_hdop, 2),
                "status": "OK" if max_hdop < 2.0 else "WARNING",
                "statusText": "Чудовий прийом" if satellites_min > 12 else "Слабкий сигнал"
            },
            "ekf": {},
            "vibration": {},
            "failsafe": {},
            "timeline": [
                {"time": "00:00", "title": "TLOG", "text": f"Прочитано {message_count} повідомлень"}
            ],
            "debug": {
                "message_count": message_count
            }
        }

    finally:
        try:
            os.unlink(temp.name)
        except:
            pass
