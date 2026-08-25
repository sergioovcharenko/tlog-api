import math
import os
import tempfile
from fastapi import FastAPI, File, UploadFile
from pymavlink import mavutil

app = FastAPI()

MAX_ALTITUDE = 1000.0
MAX_CLIMB_RATE = 50.0
GROUND_ALTITUDE = 0.5
RADIO_DROPOUT_CRITICAL_SEC = 2.0

CH7_REVERSED = False
CH8_REVERSED = False

VTX_CHANNELS = {
    1: {1: 5180, 2: 5240, 3: 5300},
    2: {1: 5520, 2: 5580, 3: 5640},
    3: {1: 5700, 2: 5765, 3: 5825},
}
VTX_BAND_NAMES = {1: "5.2", 2: "5.5", 3: "5.8"}
VTX_CHANNEL_NAMES = {1: "K1", 2: "K2", 3: "K3"}


def valid_number(value):
    try:
        value = float(value)
        return not math.isnan(value) and not math.isinf(value)
    except (TypeError, ValueError):
        return False


def parse_dbm(raw_val):
    if raw_val is None:
        return 0
    try:
        raw_val = float(raw_val)
    except (TypeError, ValueError):
        return 0
    if raw_val == 0:
        return 0
    if raw_val < 0:
        return raw_val
    if raw_val > 127:
        return raw_val - 256
    if 0 < raw_val <= 100:
        return round(raw_val / 1.9 - 127)
    return -raw_val


def clean_text(value):
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="ignore").replace("\x00", "").strip()
    return str(value).replace("\x00", "").strip()


def three_position_switch(pwm, reversed_switch=False):
    if not valid_number(pwm):
        return None
    pwm = float(pwm)
    if not 800 <= pwm <= 2200:
        return None
    if pwm < 1300:
        pos = 1
    elif pwm <= 1700:
        pos = 2
    else:
        pos = 3
    if reversed_switch:
        if pos == 1:
            pos = 3
        elif pos == 3:
            pos = 1
    return pos


def get_vtx_state(ch7_pwm, ch8_pwm):
    band_pos = three_position_switch(ch7_pwm, CH7_REVERSED)
    channel_pos = three_position_switch(ch8_pwm, CH8_REVERSED)
    if band_pos is None or channel_pos is None:
        return None
    frequency = VTX_CHANNELS.get(band_pos, {}).get(channel_pos)
    if frequency is None:
        return None
    return {
        "bandPos": band_pos,
        "channelPos": channel_pos,
        "band": VTX_BAND_NAMES[band_pos],
        "channel": VTX_CHANNEL_NAMES[channel_pos],
        "frequency": frequency,
    }


@app.get("/")
@app.get("/health")
def health():
    return {"status": "ok"}


@app.post("/analyze")
async def analyze(file: UploadFile = File(...)):
    data = await file.read()
    temp = tempfile.NamedTemporaryFile(delete=False, suffix=".tlog")
    temp.write(data)
    temp.close()

    try:
        mav = mavutil.mavlink_connection(temp.name)

        message_count = 0
        max_alt = 0.0
        max_speed = 0.0
        max_roll = 0.0
        max_pitch = 0.0
        max_dist = 0.0

        curr_alt = 0.0
        last_valid_alt = 0.0
        last_alt_timestamp = None
        latest_baro_alt = None
        ground_baro_alt = None
        global_rel_alt = None
        local_rel_alt = None
        altitude_source = "NONE"

        rangefinder_failed_flag = False

        is_currently_armed = False
        was_armed = False
        ever_armed = False
        disarm_detected = False
        landed_successfully = False
        first_arm_timestamp = None
        last_disarm_timestamp = None

        curr_dist = 0.0
        curr_voltage = 0.0
        curr_amp = 0.0
        curr_rssi_pct = 0
        curr_dbm = 0

        curr_video_freq = None
        curr_vtx_band = None
        curr_vtx_channel = None
        last_video_freq = None
        video_change_count = 0
        video_freq_seen = set()
        ch7_current = 0
        ch8_current = 0

        min_rssi = 255
        min_dbm = 0
        telem_rssi_raw = None
        telem_remrssi_raw = None
        radio_status_seen = False
        radio_bad_start = None
        max_radio_bad_duration = 0.0
        radio_bad_samples = 0

        rc_min = {i: 9999 for i in range(1, 19)}
        rc_max = {i: 0 for i in range(1, 19)}
        last_rc_state = {i: 0 for i in range(1, 19)}
        max_throttle = 0

        min_voltage = 999.0
        max_current = 0.0
        start_voltage = None
        arm_voltage = None
        reboot_or_second_battery = False

        max_vib_x = 0.0
        max_vib_y = 0.0
        max_vib_z = 0.0
        clip_count = 0
        curr_temp = None
        max_temp = -99.0
        temp_source = None
        temp_priority = -1

        has_gps = False

        # Legacy heuristic for optical navigation / LOITER diagnostics.
        # NOTE: SYS_STATUS.load is autopilot CPU load, not a native optical-quality metric.
        # Kept here to reproduce the behavior of the earlier analyzer.
        vnav_quality_min_loiter = 999.0
        vnav_quality_max_loiter = 0.0
        vnav_samples = 0
        loiter_has_origin = False
        loiter_origin_failed = False

        first_timestamp = None
        current_timestamp = 0.0
        current_mode = "Невідомо"
        flight_modes = set()
        land_mode_triggered = False
        raw_timeline = []

        # MAVLink 2 STATUSTEXT can be fragmented into 50-byte chunks.
        # Keep fragments here until the full message is assembled.
        statustext_chunks = {}

        def update_flight_altitude(new_alt, timestamp=None, source="UNKNOWN"):
            nonlocal curr_alt, max_alt, last_valid_alt, last_alt_timestamp, altitude_source
            if not valid_number(new_alt):
                return False
            new_alt = float(new_alt)
            if new_alt < -5.0 or new_alt > MAX_ALTITUDE:
                return False
            new_alt = max(0.0, new_alt)
            if last_alt_timestamp is not None and timestamp is not None:
                dt = timestamp - last_alt_timestamp
                if dt > 0:
                    allowed_change = max(MAX_CLIMB_RATE * dt, 5.0)
                    if abs(new_alt - last_valid_alt) > allowed_change:
                        return False
            if new_alt < GROUND_ALTITUDE:
                new_alt = 0.0
            curr_alt = new_alt
            last_valid_alt = new_alt
            altitude_source = source
            if timestamp is not None:
                last_alt_timestamp = timestamp
            if is_currently_armed:
                max_alt = max(max_alt, new_alt)
            return True

        def update_temperature(value, source, priority):
            nonlocal curr_temp, max_temp, temp_source, temp_priority
            if not valid_number(value):
                return
            value = float(value)
            if not (-50.0 < value < 150.0):
                return
            if priority > temp_priority:
                curr_temp = value
                max_temp = value
                temp_source = source
                temp_priority = priority
            elif priority == temp_priority:
                curr_temp = value
                max_temp = max(max_temp, value)

        def add_event(text, t_stamp, mode, is_error=False, is_pilot_action=False, event_type="SYSTEM"):
            raw_timeline.append({
                "timestamp": t_stamp or 0,
                "mode": mode,
                "alt": f"{round(curr_alt, 1)} м",
                "dist": f"{round(curr_dist, 1)} м" if curr_dist > 0 else "0.0 м",
                "vtxBand": curr_vtx_band,
                "vtxChannel": curr_vtx_channel,
                "videoFreq": curr_video_freq,
                "volt": round(curr_voltage, 2) if curr_voltage > 0 else None,
                "curr": round(curr_amp, 1) if curr_amp >= 0 else None,
                "rssi": curr_rssi_pct if curr_rssi_pct > 0 else None,
                "dbm": round(curr_dbm) if curr_dbm != 0 else None,
                "temp": round(curr_temp, 1) if curr_temp is not None else None,
                "system_text": "" if is_pilot_action else text,
                "pilot_text": text if is_pilot_action else "",
                "eventType": event_type,
                "isError": is_error,
            })

        def update_vtx_from_rc(ch7, ch8, timestamp):
            nonlocal curr_video_freq, curr_vtx_band, curr_vtx_channel
            nonlocal last_video_freq, video_change_count
            state = get_vtx_state(ch7, ch8)
            if state is None:
                return
            new_freq = state["frequency"]
            new_band = state["band"]
            new_channel = state["channel"]
            curr_video_freq = new_freq
            curr_vtx_band = new_band
            curr_vtx_channel = new_channel
            video_freq_seen.add(new_freq)
            if last_video_freq is None:
                last_video_freq = new_freq
                add_event(
                    f"📺 VTX: {new_band} GHz / {new_channel} → {new_freq} MHz",
                    timestamp,
                    current_mode,
                    False,
                    False,
                    "VIDEO",
                )
                return
            if new_freq != last_video_freq:
                old_freq = last_video_freq
                last_video_freq = new_freq
                video_change_count += 1
                add_event(
                    f"📺 VTX змінено: {old_freq} → {new_freq} MHz ({new_band} / {new_channel})",
                    timestamp,
                    current_mode,
                    False,
                    False,
                    "VIDEO",
                )


        def decode_statustext_chunk(value):
            """Decode one STATUSTEXT chunk without stripping meaningful spaces."""
            if isinstance(value, bytes):
                return value.decode("utf-8", errors="ignore").replace("\x00", "")
            return str(value).replace("\x00", "")

        def emit_statustext(full_text, timestamp, mode, severity):
            """Add one complete STATUSTEXT message to the timeline."""
            nonlocal rangefinder_failed_flag

            full_text = full_text.strip()
            if not full_text:
                return

            if "No rangefinder" in full_text or "VISP: No rangefinder" in full_text:
                rangefinder_failed_flag = True

            is_err = severity <= 4
            prefix = "⚠️ ПОМИЛКА: " if is_err else "ℹ️ "
            add_event(
                f"{prefix}{full_text}",
                timestamp,
                mode,
                is_err,
                False,
                "SYSTEM",
            )

        def handle_statustext(msg, timestamp, mode):
            """Reassemble MAVLink 2 STATUSTEXT chunks into one logical message."""
            chunk_text = decode_statustext_chunk(getattr(msg, "text", ""))
            severity = int(getattr(msg, "severity", 6))
            msg_id = int(getattr(msg, "id", 0) or 0)
            chunk_seq = int(getattr(msg, "chunk_seq", 0) or 0)

            # MAVLink 1 / legacy STATUSTEXT: no chunk identifier.
            if msg_id == 0:
                emit_statustext(chunk_text, timestamp, mode, severity)
                return

            # MAVLink STATUSTEXT text field is 50 bytes. A shorter chunk is final.
            chunk_len = len(chunk_text.encode("utf-8", errors="ignore"))

            if chunk_seq == 0:
                # If the same ID was left unfinished, flush it rather than losing it.
                old = statustext_chunks.pop(msg_id, None)
                if old and old["text"]:
                    emit_statustext(old["text"], old["timestamp"], old["mode"], old["severity"])

                statustext_chunks[msg_id] = {
                    "text": chunk_text,
                    "severity": severity,
                    "timestamp": timestamp,
                    "mode": mode,
                    "next_seq": 1,
                }
            else:
                state = statustext_chunks.get(msg_id)
                if state is None:
                    # We missed the first chunk. Do not create a misleading partial message.
                    return

                if chunk_seq != state["next_seq"]:
                    # Sequence gap/out-of-order chunk: flush what we have and drop this fragment.
                    statustext_chunks.pop(msg_id, None)
                    emit_statustext(state["text"], state["timestamp"], state["mode"], state["severity"])
                    return

                state["text"] += chunk_text
                state["next_seq"] += 1

            # Final chunk is shorter than the 50-byte STATUSTEXT payload.
            if chunk_len < 50:
                state = statustext_chunks.pop(msg_id, None)
                if state is not None:
                    emit_statustext(
                        state["text"],
                        state["timestamp"],
                        state["mode"],
                        state["severity"],
                    )

        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None:
                break

            message_count += 1
            msg_type = msg.get_type()
            t_stamp = getattr(msg, "_timestamp", 0.0)
            if t_stamp > 0:
                current_timestamp = t_stamp
                if first_timestamp is None:
                    first_timestamp = t_stamp

            if msg_type == "HEARTBEAT":
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
                        ever_armed = True
                        if first_arm_timestamp is None:
                            first_arm_timestamp = current_timestamp
                        if curr_voltage > 5.0:
                            arm_voltage = curr_voltage
                        if latest_baro_alt is not None:
                            ground_baro_alt = latest_baro_alt
                        if curr_alt < 2.0:
                            curr_alt = 0.0
                            last_valid_alt = 0.0
                        add_event("🟢 Двигуни запущено", current_timestamp, current_mode)
                        was_armed = True

                    elif not is_armed and was_armed:
                        disarm_detected = True
                        last_disarm_timestamp = current_timestamp
                        if curr_alt < 5.0 or current_mode == "LAND":
                            curr_alt = 0.0
                            last_valid_alt = 0.0
                            landed_successfully = True
                        add_event("🔴 Двигуни зупинено", current_timestamp, current_mode)
                        was_armed = False

            elif msg_type == "SYS_STATUS":
                volt = msg.voltage_battery / 1000.0
                curr = msg.current_battery / 100.0
                if volt > 5.0:
                    if curr_voltage > 5.0 and curr_voltage < 22.0 and volt > 24.5:
                        reboot_or_second_battery = True
                        if latest_baro_alt is not None:
                            ground_baro_alt = latest_baro_alt
                        add_event("🔋 Заміна батареї / Новий політ", current_timestamp, current_mode)
                    curr_voltage = volt
                    if start_voltage is None:
                        start_voltage = volt
                    min_voltage = min(min_voltage, volt)
                if curr >= 0:
                    curr_amp = curr
                    max_current = max(max_current, curr)

                # Legacy LOITER quality heuristic from the previous analyzer.
                # SYS_STATUS.load is scaled by 10 here to preserve prior behavior.
                if current_mode == "LOITER":
                    vnav_val = getattr(msg, "load", 0) / 10.0
                    if vnav_val > 0:
                        vnav_samples += 1
                        vnav_quality_min_loiter = min(vnav_quality_min_loiter, vnav_val)
                        vnav_quality_max_loiter = max(vnav_quality_max_loiter, vnav_val)

            elif msg_type == "VFR_HUD":
                latest_baro_alt = float(msg.alt)
                if ground_baro_alt is None:
                    ground_baro_alt = latest_baro_alt
                baro_rel_alt = max(0.0, latest_baro_alt - ground_baro_alt)
                if global_rel_alt is None:
                    update_flight_altitude(baro_rel_alt, current_timestamp, "BARO")
                if valid_number(msg.groundspeed):
                    max_speed = max(max_speed, float(msg.groundspeed))
                if valid_number(msg.throttle):
                    max_throttle = max(max_throttle, float(msg.throttle))

            elif msg_type == "ALTITUDE":
                alt_rel = getattr(msg, "altitude_relative", None)
                if valid_number(alt_rel) and global_rel_alt is None and local_rel_alt is None:
                    update_flight_altitude(max(0.0, float(alt_rel)), current_timestamp, "ALTITUDE")

            elif msg_type == "LOCAL_POSITION_NED":
                x = getattr(msg, "x", 0.0)
                y = getattr(msg, "y", 0.0)
                z = getattr(msg, "z", 0.0)
                if valid_number(x) and valid_number(y) and valid_number(z):
                    x, y, z = float(x), float(y), float(z)
                    d_val = math.sqrt(x * x + y * y)
                    if 0.0 <= d_val <= 10000.0:
                        curr_dist = d_val
                        max_dist = max(max_dist, curr_dist)
                    ned_alt = -z
                    if -1000.0 <= ned_alt <= 1000.0:
                        local_rel_alt = max(0.0, ned_alt)
                        if global_rel_alt is None:
                            update_flight_altitude(local_rel_alt, current_timestamp, "LOCAL_NED")

            elif msg_type in ["RANGEFINDER", "DISTANCE_SENSOR"]:
                if msg_type == "RANGEFINDER":
                    rf_dist = getattr(msg, "distance", 0)
                else:
                    rf_dist = getattr(msg, "current_distance", 0) / 100.0
                if valid_number(rf_dist) and 0.1 <= float(rf_dist) <= 50.0 and not rangefinder_failed_flag:
                    pass

            elif msg_type == "RC_CHANNELS":
                if hasattr(msg, "rssi") and 0 < msg.rssi < 255:
                    min_rssi = min(min_rssi, msg.rssi)
                    curr_rssi_pct = round((msg.rssi / 254.0) * 100)

                channels = {}
                for ch_num in range(1, 19):
                    val = getattr(msg, f"chan{ch_num}_raw", 0)
                    channels[ch_num] = val
                    if valid_number(val) and 800 < val < 2200:
                        rc_min[ch_num] = min(rc_min[ch_num], val)
                        rc_max[ch_num] = max(rc_max[ch_num], val)

                ch7 = channels.get(7, 0)
                ch8 = channels.get(8, 0)
                if 800 < ch7 < 2200 and 800 < ch8 < 2200:
                    ch7_current = ch7
                    ch8_current = ch8
                    update_vtx_from_rc(ch7, ch8, current_timestamp)

                for ch_num in range(5, 9):
                    val = channels.get(ch_num, 0)
                    if not (800 < val < 2200):
                        continue
                    prev = last_rc_state[ch_num]
                    if prev > 0 and abs(val - prev) > 250:
                        pos = three_position_switch(val)
                        state_str = {1: "ПОЗИЦІЯ 1", 2: "ПОЗИЦІЯ 2", 3: "ПОЗИЦІЯ 3"}.get(pos, "НЕВІДОМО")
                        add_event(
                            f"🎮 CH{ch_num}: {state_str} ({val} us)",
                            current_timestamp,
                            current_mode,
                            False,
                            True,
                            "PILOT",
                        )
                    last_rc_state[ch_num] = val

            elif msg_type in ["RADIO", "RADIO_STATUS"]:
                radio_status_seen = True
                telem_rssi_raw = getattr(msg, "rssi", 0)
                telem_remrssi_raw = getattr(msg, "remrssi", 0)
                dbm_val = parse_dbm(telem_rssi_raw)
                curr_dbm = dbm_val
                if dbm_val != 0 and (min_dbm == 0 or dbm_val < min_dbm):
                    min_dbm = dbm_val
                radio_bad = dbm_val <= -128 or (telem_rssi_raw == 0 and telem_remrssi_raw == 0)
                if radio_bad:
                    radio_bad_samples += 1
                    if radio_bad_start is None:
                        radio_bad_start = current_timestamp
                elif radio_bad_start is not None:
                    duration = current_timestamp - radio_bad_start
                    max_radio_bad_duration = max(max_radio_bad_duration, duration)
                    radio_bad_start = None

            elif msg_type == "ATTITUDE":
                if valid_number(msg.roll):
                    max_roll = max(max_roll, abs(math.degrees(msg.roll)))
                if valid_number(msg.pitch):
                    max_pitch = max(max_pitch, abs(math.degrees(msg.pitch)))

            elif msg_type == "GLOBAL_POSITION_INT":
                if msg.lat != 0 or msg.lon != 0:
                    has_gps = True
                if hasattr(msg, "relative_alt"):
                    rel_g = float(msg.relative_alt) / 1000.0
                    if valid_number(rel_g) and 0.0 <= rel_g <= MAX_ALTITUDE:
                        global_rel_alt = rel_g
                        update_flight_altitude(global_rel_alt, current_timestamp, "GLOBAL_REL")

                # Legacy LOITER-origin heuristic used by the earlier analyzer.
                if current_mode == "LOITER":
                    if msg.lat != 0 or msg.lon != 0:
                        loiter_has_origin = True
                    elif msg.lat == 0 and msg.lon == 0 and not loiter_has_origin:
                        loiter_origin_failed = True

            elif msg_type == "VIBRATION":
                max_vib_x = max(max_vib_x, msg.vibration_x)
                max_vib_y = max(max_vib_y, msg.vibration_y)
                max_vib_z = max(max_vib_z, msg.vibration_z)
                clip_count = max(clip_count, msg.clipping_0, msg.clipping_1, msg.clipping_2)

            elif msg_type == "MCU_STATUS":
                raw_temp = getattr(msg, "mcu_temperature", None)
                if valid_number(raw_temp):
                    raw_temp = float(raw_temp)
                    if abs(raw_temp) > 150:
                        raw_temp /= 100.0
                    update_temperature(raw_temp, "MCU_STATUS", 3)

            elif msg_type == "HIGHRES_IMU":
                raw_temp = getattr(msg, "temperature", None)
                if valid_number(raw_temp):
                    update_temperature(float(raw_temp), "HIGHRES_IMU", 2)

            elif msg_type in ["SCALED_PRESSURE", "SCALED_PRESSURE2", "SCALED_PRESSURE3"]:
                raw_temp = getattr(msg, "temperature", None)
                if valid_number(raw_temp) and float(raw_temp) != 0:
                    update_temperature(float(raw_temp) / 100.0, msg_type, 1)

            elif msg_type == "TEMPERATURE":
                raw_temp = getattr(msg, "temperature", None)
                if valid_number(raw_temp):
                    raw_temp = float(raw_temp)
                    if abs(raw_temp) > 150:
                        raw_temp /= 100.0
                    update_temperature(raw_temp, "TEMPERATURE", 1)

            elif msg_type == "STATUSTEXT":
                try:
                    handle_statustext(msg, current_timestamp, current_mode)
                except Exception:
                    pass

        # Flush any unfinished STATUSTEXT at end-of-log so it is not silently lost.
        for state in list(statustext_chunks.values()):
            if state.get("text"):
                emit_statustext(
                    state["text"],
                    state["timestamp"],
                    state["mode"],
                    state["severity"],
                )
        statustext_chunks.clear()

        if radio_bad_start is not None:
            max_radio_bad_duration = max(
                max_radio_bad_duration,
                max(0.0, current_timestamp - radio_bad_start),
            )

        if min_voltage == 999.0:
            min_voltage = 0.0
        if start_voltage is None:
            start_voltage = 0.0
        if arm_voltage is None:
            arm_voltage = start_voltage
        if vnav_quality_min_loiter == 999.0:
            vnav_quality_min_loiter = 0.0

        final_max_altitude = max(0.0, min(max_alt, MAX_ALTITUDE))

        if first_arm_timestamp is not None:
            end_time = last_disarm_timestamp if last_disarm_timestamp is not None else current_timestamp
            duration_sec = max(0, int(end_time - first_arm_timestamp))
        elif first_timestamp is not None and current_timestamp:
            duration_sec = max(0, int(current_timestamp - first_timestamp))
        else:
            duration_sec = 0
        mins, secs = divmod(duration_sec, 60)

        timeline = []
        base_t = first_timestamp or 0
        for ev in sorted(raw_timeline, key=lambda x: x["timestamp"]):
            elapsed = max(0.0, ev["timestamp"] - base_t)
            t_minutes = int(elapsed // 60)
            t_seconds = elapsed - t_minutes * 60
            timeline.append({
                "time": f"{t_minutes:02d}:{t_seconds:06.3f}",
                "mode": ev["mode"],
                "alt": ev["alt"],
                "dist": ev["dist"],
                "vtxBand": ev["vtxBand"],
                "vtxChannel": ev["vtxChannel"],
                "videoFreq": ev["videoFreq"],
                "volt": ev["volt"],
                "curr": ev["curr"],
                "rssi": ev["rssi"],
                "dbm": ev["dbm"],
                "temp": ev["temp"],
                "systemText": ev["system_text"],
                "pilotText": ev["pilot_text"],
                "eventType": ev["eventType"],
                "isError": ev["isError"],
            })

        rssi_percent = round((min_rssi / 254.0) * 100) if min_rssi != 255 else 0
        modes_str = ", ".join(sorted(flight_modes)) if flight_modes else "Невідомо"
        display_temp = f"{round(max_temp, 1)} °C" if max_temp != -99.0 else "Немає даних"

        ai_alerts = []
        is_critical = False

        if reboot_or_second_battery:
            ai_alerts.append("ℹ️ <b>Зафіксовано зміну живлення / новий політ.</b>")

        if rangefinder_failed_flag:
            ai_alerts.append(
                "📡 <b>Відвалився далекомір (Rangefinder):</b> висота продовжувала визначатися іншими джерелами."
            )

        if "LOITER" in flight_modes:
            if loiter_has_origin:
                ai_alerts.append(
                    "✅ <b>Оптична навігація в Loiter:</b> Точку 0.0.0.0 успішно зафіксовано оптикою."
                )
            elif loiter_origin_failed:
                ai_alerts.append(
                    "⚠️ <b>Оптична навігація в Loiter:</b> Точку 0.0.0.0 не вдалося зафіксувати."
                )

            if vnav_samples > 0:
                if vnav_quality_min_loiter < 40:
                    ai_alerts.append(
                        f"⚠️ <b>Низька якість Оптичної Навігації:</b> Якість розпізнавання падала до {round(vnav_quality_min_loiter)}%."
                    )
                else:
                    ai_alerts.append(
                        f"👁 <b>Якість Оптичної Навігації:</b> {round(vnav_quality_min_loiter)}%–{round(vnav_quality_max_loiter)}%."
                    )

        if radio_status_seen:
            if max_radio_bad_duration >= RADIO_DROPOUT_CRITICAL_SEC:
                ai_alerts.append(
                    f"📡 <b>Тривалий критичний стан RADIO_STATUS:</b> до {round(max_radio_bad_duration, 2)} с."
                )
                is_critical = True
            elif radio_bad_samples > 0:
                ai_alerts.append(
                    "📶 <b>Зафіксовано короткі граничні значення RADIO_STATUS.</b> Одиничне -128 не трактується як втрата борта."
                )

        if curr_video_freq is not None:
            ai_alerts.append(
                f"📺 <b>Відеоканал визначено за CH7 + CH8:</b> {curr_vtx_band} GHz / {curr_vtx_channel} / {curr_video_freq} MHz. Змін за лог: {video_change_count}."
            )
        else:
            ai_alerts.append(
                "ℹ️ <b>Відеочастоту не вдалося визначити:</b> у TLOG немає коректних значень CH7/CH8."
            )

        if 0 < min_voltage <= 16.8:
            is_critical = True
            ai_alerts.append(f"🪫 <b>Критична просадка:</b> {round(min_voltage, 1)} V.")
        elif 16.8 < min_voltage < 18.0:
            ai_alerts.append(f"🔋 <b>Глибока просадка:</b> {round(min_voltage, 1)} V.")

        if max_current > 80.0:
            ai_alerts.append(f"⚡ <b>Високий струм:</b> {round(max_current, 1)} A.")

        if max_temp != -99.0:
            if max_temp >= 85.0:
                ai_alerts.append(f"🌡 <b>Критична температура:</b> {round(max_temp, 1)} °C.")
                is_critical = True
            elif max_temp >= 70.0:
                ai_alerts.append(f"🌡 <b>Висока температура:</b> {round(max_temp, 1)} °C.")

        if max_roll > 80 or max_pitch > 80:
            ai_alerts.append(
                f"🔄 <b>Великий кут нахилу:</b> {round(max(max_roll, max_pitch), 1)}°."
            )
            is_critical = True

        log_ended_armed = ever_armed and was_armed
        if log_ended_armed:
            ai_alerts.append(
                "❗ <b>Лог закінчився при ARMED:</b> у файлі немає підтвердженого DISARM."
            )
            is_critical = True

        if landed_successfully:
            ai_verdict = (
                "⚠️ БОРТ ЗАВЕРШИВ ПОЛІТ. ПІД ЧАС ПОЛЬОТУ ЗАФІКСОВАНО КРИТИЧНІ ПОДІЇ:"
                if is_critical
                else "✅ БОРТ ЗАВЕРШИВ ПОЛІТ ТА БУВ РОЗЗБРОЄНИЙ."
            )
        elif disarm_detected:
            ai_verdict = (
                "⚠️ У ЛОГУ Є DISARM. ПІД ЧАС ПОЛЬОТУ ЗАФІКСОВАНО КРИТИЧНІ ПОДІЇ:"
                if is_critical
                else "✅ У ЛОГУ Є ЗАВЕРШЕННЯ ПОЛЬОТУ ТА DISARM."
            )
        elif log_ended_armed:
            ai_verdict = "🚨 ЛОГ ОБІРВАВСЯ ПРИ ARMED. ПОТРІБНА ПЕРЕВІРКА:"
        elif is_critical:
            ai_verdict = "⚠️ ПІД ЧАС ПОЛЬОТУ ЗАФІКСОВАНО КРИТИЧНІ ПОДІЇ:"
        else:
            ai_verdict = "📊 РЕЗУЛЬТАТИ АНАЛІЗУ ПОЛЬОТУ:"

        def channel_range(ch):
            return f"{rc_min[ch]}–{rc_max[ch]} us" if rc_max[ch] > 0 else "—"

        return {
            "success": True,
            "ai": {
                "verdict": ai_verdict,
                "isCritical": is_critical,
                "landedSuccessfully": landed_successfully,
                "disarmDetected": disarm_detected,
                "alerts": ai_alerts,
            },
            "flight": {
                "durationText": f"{mins} хв {secs} с",
                "maxAltitude": round(final_max_altitude, 1),
                "maxDistance": round(max_dist, 1),
                "maxSpeed": round(max_speed, 1),
                "maxRoll": round(max_roll, 1),
                "maxPitch": round(max_pitch, 1),
                "modes": modes_str,
                "msgCount": message_count,
                "altitudeSource": altitude_source,
            },
            "battery": {
                "armVoltage": round(arm_voltage, 2),
                "minVoltage": round(min_voltage, 2),
                "maxCurrent": round(max_current, 2),
                "voltageSag": round(max(0, arm_voltage - min_voltage), 2),
            },
            "radio": {
                "rssi": f"{rssi_percent}%" if min_rssi != 255 else "Немає",
                "telemRssi": f"{round(min_dbm)} dBm" if min_dbm != 0 else "—",
                "maxThrottle": f"{round(max_throttle)}%",
                "maxDropout": round(max_radio_bad_duration, 2),
                "hasGps": "GPS Присутній" if has_gps else "Без GPS / локальна навігація",
            },
            "video": {
                "frequency": curr_video_freq,
                "band": curr_vtx_band,
                "channel": curr_vtx_channel,
                "changeCount": video_change_count,
                "uniqueCount": len(video_freq_seen),
                "ch7Pwm": ch7_current,
                "ch8Pwm": ch8_current,
            },
            "health": {
                "vibX": round(max_vib_x, 1),
                "vibY": round(max_vib_y, 1),
                "vibZ": round(max_vib_z, 1),
                "clipping": clip_count,
                "maxTemp": display_temp,
                "tempSource": temp_source or "—",
                "opticalQualityLoiter": (
                    f"{round(vnav_quality_min_loiter)}%–{round(vnav_quality_max_loiter)}%"
                    if vnav_samples > 0
                    else "Немає даних"
                ),
            },
            "radioChannels": {
                "ch1": channel_range(1),
                "ch2": channel_range(2),
                "ch3": channel_range(3),
                "ch4": channel_range(4),
                "ch7": channel_range(7),
                "ch8": channel_range(8),
            },
            "timeline": timeline,
        }

    finally:
        if os.path.exists(temp.name):
            try:
                os.unlink(temp.name)
            except Exception:
                pass
