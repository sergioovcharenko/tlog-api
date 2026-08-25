import json

# ==========================================
# НАЛАШТУВАННЯ КАНАЛІВ
# ==========================================
RC_CHANNEL_NAMES = {
    1: "Крен (Roll)",
    2: "Тангаж (Pitch)",
    3: "Газ (Throttle)",
    4: "Рискання (Yaw)",
    5: "Режим польоту (6 кнопок)",
    7: "Діапазон відео (Band)",
    8: "Канал відео (Channel)",
    11: "Активатор скиду (Trigger)",
    13: "Запобіжник/Вибір боєприпасу (Safety)"
}

# ==========================================
# ФУНКЦІЇ-ПОМІЧНИКИ ДЛЯ КАСТОМНИХ КАНАЛІВ
# ==========================================
def get_ch5_mode(pwm_val):
    if pwm_val <= 1230: return "AltHold (Кнопка 1)"
    elif 1231 <= pwm_val <= 1360: return "Loiter (Кнопка 2)"
    elif 1361 <= pwm_val <= 1490: return "Land (Кнопка 3)"
    elif 1491 <= pwm_val <= 1620: return "RTL (Кнопка 4)"
    elif 1621 <= pwm_val <= 1749: return "AltHold (Кнопка 5)"
    else: return "AltHold (Кнопка 6)"

def get_ch7_band(pwm_val):
    # CH7: Від себе (<1300) = 5.2, Середнє = 5.5, До себе (>1600) = 5.8
    if pwm_val < 1300: return "5.2 ГГц"
    elif 1300 <= pwm_val <= 1600: return "5.5 ГГц"
    else: return "5.8 ГГц"

def get_ch8_channel(pwm_val):
    # CH8: Від себе = Канал 1, Середнє = Канал 2, До себе = Канал 3
    if pwm_val < 1300: return "Кан 1"
    elif 1300 <= pwm_val <= 1600: return "Кан 2"
    else: return "Кан 3"

def get_ch13_safety(pwm_val):
    # CH13: Від себе (<1300) = На запобіжнику
    # Середнє = Знятий (Лівий), До себе (>1600) = Знятий (Правий)
    if pwm_val < 1300:
        return "НА ЗАПОБІЖНИКУ", False
    elif 1300 <= pwm_val <= 1600:
        return "Знято: ЛІВИЙ БОЄПРИПАС", True
    else:
        return "Знято: ПРАВИЙ БОЄПРИПАС", True

# ==========================================
# ГОЛОВНА ФУНКЦІЯ АНАЛІЗУ
# ==========================================
def analyze_telemetry(messages):
    rc_min = {i: 9999 for i in range(1, 17)}
    rc_max = {i: 0 for i in range(1, 17)}
    last_rc_state = {i: 0 for i in range(1, 17)}
    
    timeline = []
    min_rssi = 255
    current_mode = "UNKNOWN"
    current_timestamp = 0 

    for msg in messages:
        msg_type = getattr(msg, "get_type", lambda: msg.get("mavpackettype"))()
        
        if msg_type == "RC_CHANNELS":
            if hasattr(msg, "rssi") and 0 < msg.rssi < 255:
                if msg.rssi < min_rssi:
                    min_rssi = msg.rssi
            
            for ch_num in range(1, 17):
                val = getattr(msg, f"chan{ch_num}_raw", 0)
                
                if 800 < val < 2200:
                    if val < rc_min[ch_num]: rc_min[ch_num] = val
                    if val > rc_max[ch_num]: rc_max[ch_num] = val

                    # ========== CH5 (Режими польоту) ==========
                    if ch_num == 5:
                        prev_val = last_rc_state[ch_num]
                        if prev_val > 0:
                            current_btn = get_ch5_mode(val)
                            prev_btn = get_ch5_mode(prev_val)
                            if current_btn != prev_btn:
                                timeline.append({"time": current_timestamp, "mode": current_mode, 
                                                 "event": f"🛩️ Режим польоту (CH5) перемкнуто на {current_btn} ({val} us)", "is_pilot_action": True})
                    
                    # ========== CH7 (Діапазон відео) ==========
                    elif ch_num == 7:
                        prev_val = last_rc_state[ch_num]
                        if prev_val > 0 and abs(val - prev_val) > 250:
                            band = get_ch7_band(val)
                            # Беремо поточний стан CH8 (щоб показати повну картину)
                            ch8_val = getattr(msg, "chan8_raw", last_rc_state[8])
                            chan = get_ch8_channel(ch8_val)
                            timeline.append({"time": current_timestamp, "mode": current_mode,
                                             "event": f"📺 Зміна частоти відео: тепер {band}, {chan}", "is_pilot_action": True})
                    
                    # ========== CH8 (Канал відео) ==========
                    elif ch_num == 8:
                        prev_val = last_rc_state[ch_num]
                        if prev_val > 0 and abs(val - prev_val) > 250:
                            chan = get_ch8_channel(val)
                            # Беремо поточний стан CH7
                            ch7_val = getattr(msg, "chan7_raw", last_rc_state[7])
                            band = get_ch7_band(ch7_val)
                            timeline.append({"time": current_timestamp, "mode": current_mode,
                                             "event": f"📺 Зміна каналу відео: тепер {band}, {chan}", "is_pilot_action": True})

                    # ========== CH13 (Запобіжник) ==========
                    elif ch_num == 13:
                        prev_val = last_rc_state[ch_num]
                        if prev_val > 0 and abs(val - prev_val) > 250:
                            status_text, is_armed = get_ch13_safety(val)
                            icon = "⚠️" if is_armed else "🔒"
                            timeline.append({"time": current_timestamp, "mode": current_mode,
                                             "event": f"{icon} Стан озброєння (CH13): {status_text}", "is_pilot_action": True})

                    # ========== CH11 (Активатор скиду) ==========
                    elif ch_num == 11:
                        prev_val = last_rc_state[ch_num]
                        if prev_val > 0 and abs(val - prev_val) > 250:
                            is_active = val > 1600 # Якщо тумблер увімкнено (ШІМ високий)
                            was_active = prev_val > 1600
                            
                            if is_active and not was_active:
                                # АКТИВАЦІЯ! Перевіряємо, в якому стані зараз CH13
                                ch13_val = getattr(msg, "chan13_raw", last_rc_state[13])
                                status_text, is_armed = get_ch13_safety(ch13_val)
                                
                                if not is_armed:
                                    # Спроба скиду на запобіжнику
                                    timeline.append({"time": current_timestamp, "mode": current_mode,
                                                     "event": "❌ Спроба скиду заблокована (CH13 на запобіжнику!)", "is_pilot_action": True})
                                else:
                                    # Успішний скид!
                                    timeline.append({"time": current_timestamp, "mode": current_mode,
                                                     "event": f"💣 СКИД! ({status_text})", "is_pilot_action": True})
                            
                            elif not is_active and was_active:
                                timeline.append({"time": current_timestamp, "mode": current_mode,
                                                 "event": "🔄 Активатор скиду (CH11) вимкнено", "is_pilot_action": True})

                    # ========== ІНШІ ТУМБЛЕРИ (CH6, CH9, CH10, CH12, CH14, CH15, CH16) ==========
                    elif ch_num > 5:
                        prev_val = last_rc_state[ch_num]
                        if prev_val > 0 and abs(val - prev_val) > 250:
                            if val > 1600: state_str = "АКТИВНО (High)"
                            elif 1300 <= val <= 1600: state_str = "СЕРЕДНЄ (Mid)"
                            else: state_str = "ВИМКНЕНО (Low)"
                            
                            ch_name = RC_CHANNEL_NAMES.get(ch_num, f"CH{ch_num}")
                            timeline.append({"time": current_timestamp, "mode": current_mode,
                                             "event": f"🎮 {ch_name} переведено в {state_str} ({val} us)", "is_pilot_action": True})
                    
                    last_rc_state[ch_num] = val

    rc_output = {}
    for i in range(1, 17):
        channel_data = {
            "name": RC_CHANNEL_NAMES.get(i, f"CH{i}"),
            "min": rc_min[i] if rc_min[i] != 9999 else 0,
            "max": rc_max[i],
            "last": last_rc_state[i]
        }
        # Додатковий вивід поточного стану для спеціальних каналів на фронтенд
        if last_rc_state[i] > 0:
            if i == 5: channel_data["current_mode"] = get_ch5_mode(last_rc_state[i])
            elif i == 7: channel_data["current_mode"] = get_ch7_band(last_rc_state[i])
            elif i == 8: channel_data["current_mode"] = get_ch8_channel(last_rc_state[i])
            elif i == 11: channel_data["current_mode"] = "Активовано" if last_rc_state[i] > 1600 else "Вимкнено"
            elif i == 13: channel_data["current_mode"] = get_ch13_safety(last_rc_state[i])[0]
            
        rc_output[f"CH{i}"] = channel_data

    return {
        "success": True,
        "rc_channels": rc_output,
        "radio_stats": {"min_rssi": min_rssi if min_rssi != 255 else 0},
        "timeline": timeline
    }
