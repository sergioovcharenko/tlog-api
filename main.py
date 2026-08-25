import math
import os
import re
import tempfile

from fastapi import FastAPI, File, UploadFile
from pymavlink import mavutil


app = FastAPI()


# ============================================================
# CONFIG
# ============================================================

MAX_ALTITUDE = 1000.0
MAX_CLIMB_RATE = 50.0
GROUND_ALTITUDE = 0.5

# Керування
CTRL_LOW_MIN = 410.0
CTRL_LOW_MAX = 485.0

CTRL_HIGH_MIN = 820.0
CTRL_HIGH_MAX = 895.0

# Відео
VIDEO_MIN = 4900.0
VIDEO_MAX = 6000.0

# -128 dBm / нульові RADIO_STATUS повинні тривати
# не менше цього часу, щоб вважати це тривалою проблемою.
RADIO_DROPOUT_CRITICAL_SEC = 2.0


# ============================================================
# HELPERS
# ============================================================

def valid_number(value):
    try:
        value = float(value)
        return not math.isnan(value) and not math.isinf(value)
    except (TypeError, ValueError):
        return False


def parse_dbm(raw_val):
    """
    Обережна інтерпретація поля RADIO_STATUS.rssi.

    ВАЖЛИВО:
    не кожен радіомодем передає тут фізичний dBm.
    Тому значення використовується як індикатор,
    але -128 більше не означає автоматично втрату борта.
    """

    if raw_val is None:
        return 0

    try:
        raw_val = float(raw_val)
    except (TypeError, ValueError):
        return 0

    if raw_val == 0:
        return 0

    # Уже signed
    if raw_val < 0:
        return raw_val

    # signed byte у вигляді unsigned
    if raw_val > 127:
        return raw_val - 256

    # Деякі модеми віддають умовну шкалу
    if 0 < raw_val <= 100:
        return round(raw_val / 1.9 - 127)

    return -raw_val


def normalize_frequency(value):
    """
    Нормалізація частоти в MHz.

    Приклади:
    433.5       -> 433.5 MHz
    433500      -> 433.5 MHz
    433500000   -> 433.5 MHz
    """

    if not valid_number(value):
        return None

    value = abs(float(value))

    # Hz
    if value >= 10_000_000:
        value /= 1_000_000.0

    # kHz
    elif value >= 100_000:
        value /= 1000.0

    return value


def classify_frequency(freq):
    if freq is None:
        return None

    if CTRL_LOW_MIN <= freq <= CTRL_LOW_MAX:
        return "LOW"

    if CTRL_HIGH_MIN <= freq <= CTRL_HIGH_MAX:
        return "HIGH"

    if VIDEO_MIN <= freq <= VIDEO_MAX:
        return "VIDEO"

    return None


def clean_text(value):
    if isinstance(value, bytes):
        try:
            return value.decode("utf-8", errors="ignore").replace("\x00", "").strip()
        except Exception:
            return str(value)

    return str(value).replace("\x00", "").strip()


# ============================================================
# HEALTH
# ============================================================

@app.get("/")
@app.get("/health")
def health():
    return {"status": "ok"}


# ============================================================
# ANALYZE
# ============================================================

@app.post("/analyze")
async def analyze(file: UploadFile = File(...)):

    data = await file.read()

    temp = tempfile.NamedTemporaryFile(
        delete=False,
        suffix=".tlog"
    )

    temp.write(data)
    temp.close()

    try:
        mav = mavutil.mavlink_connection(temp.name)

        # ====================================================
        # BASE METRICS
        # ====================================================

        message_count = 0

        max_alt = 0.0
        max_speed = 0.0
        max_roll = 0.0
        max_pitch = 0.0
        max_dist = 0.0

        # ====================================================
        # ALTITUDE
        # ====================================================

        curr_alt = 0.0
        last_valid_alt = 0.0
        last_alt_timestamp = None

        latest_baro_alt = None
        ground_baro_alt = None
        baro_rel_alt = None

        global_rel_alt = None
        local_rel_alt = None

        altitude_source = "NONE"

        # ====================================================
        # RANGEFINDER
        # ====================================================

        rangefinder_alt = None
        max_rf_alt = 0.0
        has_rangefinder = False
        rangefinder_failed_flag = False

        # ====================================================
        # ARM / LAND
        # ====================================================

        is_currently_armed = False
        was_armed = False
        ever_armed = False

        landed_successfully = False

        arm_timestamp = None
        disarm_timestamp = None

        # ====================================================
        # CURRENT VALUES
        # ====================================================

        curr_dist = 0.0

        curr_voltage = 0.0
        curr_amp = 0.0

        curr_rssi_pct = 0
        curr_dbm = 0

        # ====================================================
        # RF / ППРЧ / VIDEO
        # ====================================================

        curr_ctrl_low = None
        curr_ctrl_high = None
        curr_video_freq = None

        last_ctrl_low = None
        last_ctrl_high = None
        last_video_freq = None

        low_hop_count = 0
        high_hop_count = 0
        video_change_count = 0

        low_freq_seen = set()
        high_freq_seen = set()
        video_freq_seen = set()

        first_rf_timestamp = None
        last_rf_timestamp = None

        explicit_rf_data_found = False

        # ====================================================
        # RADIO
        # ====================================================

        min_rssi = 255
        min_dbm = 0

        telem_rssi_raw = None
        telem_remrssi_raw = None

        radio_status_seen = False

        radio_bad_start = None
        max_radio_bad_duration = 0.0
        radio_bad_samples = 0

        # ====================================================
        # RC
        # ====================================================

        rc_min = {i: 9999 for i in range(1, 9)}
        rc_max = {i: 0 for i in range(1, 9)}

        last_rc_state = {i: 0 for i in range(1, 9)}

        max_throttle = 0

        # ====================================================
        # BATTERY
        # ====================================================

        min_voltage = 999.0
        max_current = 0.0

        start_voltage = None
        voltage_at_land_mode = None

        reboot_or_second_battery = False

        # ====================================================
        # HEALTH
        # ====================================================

        max_vib_x = 0.0
        max_vib_y = 0.0
        max_vib_z = 0.0

        clip_count = 0

        curr_temp = None
        max_temp = -99.0

        # ====================================================
        # OPTICAL / GPS
        # ====================================================

        vnav_quality_min_loiter = 999
        vnav_quality_max_loiter = 0
        vnav_samples = 0

        loiter_has_origin = False
        loiter_origin_failed = False

        has_gps = False

        # ====================================================
        # TIME / MODES
        # ====================================================

        first_timestamp = None
        current_timestamp = 0.0

        current_mode = "Невідомо"

        flight_modes = set()

        land_mode_triggered = False

        # ====================================================
        # TIMELINE
        # ====================================================

        raw_timeline = []

        # ====================================================
        # ALTITUDE UPDATE
        # ====================================================

        def update_flight_altitude(
            new_alt,
            timestamp=None,
            source="UNKNOWN"
        ):

            nonlocal curr_alt
            nonlocal max_alt
            nonlocal last_valid_alt
            nonlocal last_alt_timestamp
            nonlocal altitude_source

            if not valid_number(new_alt):
                return False

            new_alt = float(new_alt)

            if new_alt < -5.0:
                return False

            if new_alt > MAX_ALTITUDE:
                return False

            new_alt = max(0.0, new_alt)

            # Захист від одиничних стрибків
            if (
                last_alt_timestamp is not None
                and timestamp is not None
            ):
                dt = timestamp - last_alt_timestamp

                if dt > 0:
                    allowed_change = max(
                        MAX_CLIMB_RATE * dt,
                        5.0
                    )

                    if abs(new_alt - last_valid_alt) > allowed_change:
                        return False

            if new_alt < GROUND_ALTITUDE:
                new_alt = 0.0

            curr_alt = new_alt
            last_valid_alt = new_alt
            altitude_source = source

            if timestamp is not None:
                last_alt_timestamp = timestamp

            if new_alt > max_alt:
                max_alt = new_alt

            return True

        # ====================================================
        # TEMPERATURE
        # ====================================================

        def update_temperature(value):

            nonlocal curr_temp
            nonlocal max_temp

            if not valid_number(value):
                return

            value = float(value)

            if not (-50.0 < value < 150.0):
                return

            curr_temp = value

            if max_temp == -99.0 or value > max_temp:
                max_temp = value

        # ====================================================
        # TIMELINE EVENT
        # ====================================================

        def add_event(
            text,
            t_stamp,
            mode,
            is_error=False,
            is_pilot_action=False,
            event_type="SYSTEM"
        ):

            raw_timeline.append({
                "timestamp": t_stamp or 0,

                "mode": mode,

                "alt": f"{round(curr_alt, 1)} м",

                "dist": (
                    f"{round(curr_dist, 1)} м"
                    if curr_dist > 0
                    else "0.0 м"
                ),

                "ctrlLow": (
                    round(curr_ctrl_low, 3)
                    if curr_ctrl_low is not None
                    else None
                ),

                "ctrlHigh": (
                    round(curr_ctrl_high, 3)
                    if curr_ctrl_high is not None
                    else None
                ),

                "videoFreq": (
                    round(curr_video_freq, 3)
                    if curr_video_freq is not None
                    else None
                ),

                "volt": (
                    round(curr_voltage, 2)
                    if curr_voltage > 0
                    else None
                ),

                "curr": (
                    round(curr_amp, 1)
                    if curr_amp >= 0
                    else None
                ),

                "rssi": (
                    curr_rssi_pct
                    if curr_rssi_pct > 0
                    else None
                ),

                "dbm": (
                    round(curr_dbm)
                    if curr_dbm != 0
                    else None
                ),

                "temp": (
                    round(curr_temp, 1)
                    if curr_temp is not None
                    else None
                ),

                "system_text": (
                    ""
                    if is_pilot_action
                    else text
                ),

                "pilot_text": (
                    text
                    if is_pilot_action
                    else ""
                ),

                "eventType": event_type,

                "isError": is_error
            })

        # ====================================================
        # FREQUENCY UPDATE
        # ====================================================

        def update_frequency(
            band,
            freq,
            timestamp,
            source="MAVLink"
        ):

            nonlocal curr_ctrl_low
            nonlocal curr_ctrl_high
            nonlocal curr_video_freq

            nonlocal last_ctrl_low
            nonlocal last_ctrl_high
            nonlocal last_video_freq

            nonlocal low_hop_count
            nonlocal high_hop_count
            nonlocal video_change_count

            nonlocal first_rf_timestamp
            nonlocal last_rf_timestamp

            nonlocal explicit_rf_data_found

            freq = normalize_frequency(freq)

            if freq is None:
                return

            if band is None:
                band = classify_frequency(freq)

            if band is None:
                return

            if band == "LOW":
                if not (CTRL_LOW_MIN <= freq <= CTRL_LOW_MAX):
                    return

            elif band == "HIGH":
                if not (CTRL_HIGH_MIN <= freq <= CTRL_HIGH_MAX):
                    return

            elif band == "VIDEO":
                if not (VIDEO_MIN <= freq <= VIDEO_MAX):
                    return

            else:
                return

            explicit_rf_data_found = True

            if first_rf_timestamp is None:
                first_rf_timestamp = timestamp

            last_rf_timestamp = timestamp

            # ------------------------------------------------
            # LOW
            # ------------------------------------------------

            if band == "LOW":

                curr_ctrl_low = freq
                low_freq_seen.add(round(freq, 3))

                if last_ctrl_low is None:
                    last_ctrl_low = freq

                    add_event(
                        f"📡 CTRL LOW: {freq:.3f} MHz",
                        timestamp,
                        current_mode,
                        False,
                        False,
                        "RF_LOW"
                    )

                elif abs(freq - last_ctrl_low) >= 0.001:

                    old_freq = last_ctrl_low

                    last_ctrl_low = freq
                    low_hop_count += 1

                    add_event(
                        (
                            f"📡 ППРЧ LOW: "
                            f"{old_freq:.3f} → "
                            f"{freq:.3f} MHz"
                        ),
                        timestamp,
                        current_mode,
                        False,
                        False,
                        "RF_LOW"
                    )

            # ------------------------------------------------
            # HIGH
            # ------------------------------------------------

            elif band == "HIGH":

                curr_ctrl_high = freq
                high_freq_seen.add(round(freq, 3))

                if last_ctrl_high is None:
                    last_ctrl_high = freq

                    add_event(
                        f"📡 CTRL HIGH: {freq:.3f} MHz",
                        timestamp,
                        current_mode,
                        False,
                        False,
                        "RF_HIGH"
                    )

                elif abs(freq - last_ctrl_high) >= 0.001:

                    old_freq = last_ctrl_high

                    last_ctrl_high = freq
                    high_hop_count += 1

                    add_event(
                        (
                            f"📡 ППРЧ HIGH: "
                            f"{old_freq:.3f} → "
                            f"{freq:.3f} MHz"
                        ),
                        timestamp,
                        current_mode,
                        False,
                        False,
                        "RF_HIGH"
                    )

            # ------------------------------------------------
            # VIDEO
            # ------------------------------------------------

            elif band == "VIDEO":

                curr_video_freq = freq
                video_freq_seen.add(round(freq, 3))

                if last_video_freq is None:
                    last_video_freq = freq

                    add_event(
                        f"📺 VTX / VIDEO: {freq:.3f} MHz",
                        timestamp,
                        current_mode,
                        False,
                        False,
                        "VIDEO"
                    )

                elif abs(freq - last_video_freq) >= 0.001:

                    old_freq = last_video_freq

                    last_video_freq = freq
                    video_change_count += 1

                    add_event(
                        (
                            f"📺 VTX / VIDEO: "
                            f"{old_freq:.3f} → "
                            f"{freq:.3f} MHz"
                        ),
                        timestamp,
                        current_mode,
                        False,
                        False,
                        "VIDEO"
                    )

        # ====================================================
        # PARAM FREQUENCY
        # ====================================================

        def inspect_parameter_for_frequency(
            param_id,
            param_value,
            timestamp
        ):

            pid = clean_text(param_id).lower()

            if not any(
                word in pid
                for word in [
                    "freq",
                    "frequency",
                    "rf",
                    "vtx",
                    "video"
                ]
            ):
                return

            freq = normalize_frequency(param_value)

            if freq is None:
                return

            band = classify_frequency(freq)

            # Уточнюємо за назвою параметра
            if any(x in pid for x in ["vtx", "video"]):
                if VIDEO_MIN <= freq <= VIDEO_MAX:
                    band = "VIDEO"

            elif any(x in pid for x in ["low", "400", "410"]):
                if CTRL_LOW_MIN <= freq <= CTRL_LOW_MAX:
                    band = "LOW"

            elif any(x in pid for x in ["high", "800", "820"]):
                if CTRL_HIGH_MIN <= freq <= CTRL_HIGH_MAX:
                    band = "HIGH"

            if band is not None:
                update_frequency(
                    band,
                    freq,
                    timestamp,
                    f"PARAM:{pid}"
                )

        # ====================================================
        # STATUSTEXT FREQUENCY
        # ====================================================

        def extract_frequencies_from_text(
            text,
            timestamp
        ):

            if not text:
                return

            txt = clean_text(text)
            lower = txt.lower()

            # Щоб випадкові числа у звичайних повідомленнях
            # не сприймались як частота, вимагаємо RF-контекст.
            rf_context = any(
                word in lower
                for word in [
                    "freq",
                    "frequency",
                    "mhz",
                    "мгц",
                    "vtx",
                    "video",
                    "відео",
                    "rf",
                    "channel",
                    "band"
                ]
            )

            if not rf_context:
                return

            pattern = (
                r"(?<!\d)"
                r"(\d{3,10}(?:\.\d+)?)"
                r"\s*"
                r"(mhz|мгц|khz|кгц|hz|гц)?"
            )

            matches = re.findall(
                pattern,
                lower
            )

            for raw_value, unit in matches:

                try:
                    value = float(raw_value)
                except Exception:
                    continue

                if unit in ("hz", "гц"):
                    value /= 1_000_000.0

                elif unit in ("khz", "кгц"):
                    value /= 1000.0

                freq = normalize_frequency(value)

                if freq is None:
                    continue

                band = classify_frequency(freq)

                if band is None:
                    continue

                if any(
                    word in lower
                    for word in [
                        "vtx",
                        "video",
                        "відео"
                    ]
                ):
                    if VIDEO_MIN <= freq <= VIDEO_MAX:
                        band = "VIDEO"

                elif any(
                    word in lower
                    for word in [
                        "low",
                        "400",
                        "410"
                    ]
                ):
                    if CTRL_LOW_MIN <= freq <= CTRL_LOW_MAX:
                        band = "LOW"

                elif any(
                    word in lower
                    for word in [
                        "high",
                        "800",
                        "820"
                    ]
                ):
                    if CTRL_HIGH_MIN <= freq <= CTRL_HIGH_MAX:
                        band = "HIGH"

                update_frequency(
                    band,
                    freq,
                    timestamp,
                    "STATUSTEXT"
                )

        # ====================================================
        # GENERIC MAVLINK FIELDS
        # ====================================================

        def inspect_message_for_frequency(
            msg,
            timestamp
        ):

            try:
                d = msg.to_dict()
            except Exception:
                return

            for key, value in d.items():

                key_lower = str(key).lower()

                # Не беремо довільні числа.
                # Тільки поля, назва яких прямо натякає на частоту.
                if not any(
                    token in key_lower
                    for token in [
                        "frequency",
                        "freq",
                        "vtx_freq",
                        "rf_freq",
                        "tx_freq"
                    ]
                ):
                    continue

                if not valid_number(value):
                    continue

                freq = normalize_frequency(value)

                if freq is None:
                    continue

                band = classify_frequency(freq)

                if band is None:
                    continue

                if "vtx" in key_lower or "video" in key_lower:
                    if VIDEO_MIN <= freq <= VIDEO_MAX:
                        band = "VIDEO"

                elif "low" in key_lower or "400" in key_lower:
                    if CTRL_LOW_MIN <= freq <= CTRL_LOW_MAX:
                        band = "LOW"

                elif "high" in key_lower or "800" in key_lower:
                    if CTRL_HIGH_MIN <= freq <= CTRL_HIGH_MAX:
                        band = "HIGH"

                update_frequency(
                    band,
                    freq,
                    timestamp,
                    f"FIELD:{key}"
                )

        # ====================================================
        # MAVLINK LOOP
        # ====================================================

        while True:

            msg = mav.recv_match(
                blocking=False
            )

            if msg is None:
                break

            message_count += 1

            msg_type = msg.get_type()

            t_stamp = getattr(
                msg,
                "_timestamp",
                0.0
            )

            if t_stamp > 0:
                current_timestamp = t_stamp

                if first_timestamp is None:
                    first_timestamp = t_stamp

            # Додатково перевіряємо всі MAVLink повідомлення
            # на явно названі поля частоти.
            inspect_message_for_frequency(
                msg,
                current_timestamp
            )

            # =================================================
            # HEARTBEAT
            # =================================================

            if msg_type == "HEARTBEAT":

                if msg.get_srcComponent() == 1:

                    new_mode = mav.flightmode

                    is_armed = bool(
                        msg.base_mode
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )

                    is_currently_armed = is_armed

                    # -----------------------------------------
                    # MODE
                    # -----------------------------------------

                    if new_mode and new_mode != current_mode:

                        if current_mode != "Невідомо":

                            add_event(
                                f"🔄 Режим змінено на {new_mode}",
                                current_timestamp,
                                new_mode
                            )

                        current_mode = new_mode

                        flight_modes.add(
                            current_mode
                        )

                        if current_mode == "LAND":
                            land_mode_triggered = True

                    # -----------------------------------------
                    # ARM
                    # -----------------------------------------

                    if is_armed and not was_armed:

                        ever_armed = True
                        arm_timestamp = current_timestamp

                        if latest_baro_alt is not None:
                            ground_baro_alt = latest_baro_alt

                        if curr_alt < 2.0:
                            curr_alt = 0.0
                            last_valid_alt = 0.0

                        add_event(
                            "🟢 Двигуни запущено",
                            current_timestamp,
                            current_mode
                        )

                        was_armed = True

                    # -----------------------------------------
                    # DISARM
                    # -----------------------------------------

                    elif not is_armed and was_armed:

                        disarm_timestamp = current_timestamp

                        # Нормальний DISARM наприкінці польоту
                        # є сильною ознакою, що борт повернувся.
                        if curr_alt < 5.0:
                            curr_alt = 0.0
                            last_valid_alt = 0.0
                            landed_successfully = True

                        elif current_mode == "LAND":
                            landed_successfully = True

                        add_event(
                            "🔴 Двигуни зупинено",
                            current_timestamp,
                            current_mode
                        )

                        was_armed = False

            # =================================================
            # SYS_STATUS
            # =================================================

            elif msg_type == "SYS_STATUS":

                volt = msg.voltage_battery / 1000.0
                curr = msg.current_battery / 100.0

                if volt > 5.0:

                    if (
                        curr_voltage > 5.0
                        and curr_voltage < 22.0
                        and volt > 24.5
                    ):
                        reboot_or_second_battery = True

                        if latest_baro_alt is not None:
                            ground_baro_alt = latest_baro_alt

                        add_event(
                            "🔋 Заміна батареї / Новий політ",
                            current_timestamp,
                            current_mode
                        )

                    curr_voltage = volt

                    if start_voltage is None:
                        start_voltage = volt

                    min_voltage = min(
                        min_voltage,
                        volt
                    )

                if curr >= 0:
                    curr_amp = curr
                    max_current = max(
                        max_current,
                        curr
                    )

                if (
                    current_mode == "LAND"
                    and voltage_at_land_mode is None
                ):
                    voltage_at_land_mode = volt

                # Залишаю твою стару логіку
                # як індикатор для LOITER.
                if current_mode == "LOITER":

                    vnav_val = (
                        getattr(
                            msg,
                            "load",
                            0
                        )
                        / 10.0
                    )

                    if vnav_val > 0:

                        vnav_samples += 1

                        vnav_quality_min_loiter = min(
                            vnav_quality_min_loiter,
                            vnav_val
                        )

                        vnav_quality_max_loiter = max(
                            vnav_quality_max_loiter,
                            vnav_val
                        )

            # =================================================
            # VFR_HUD
            # =================================================

            elif msg_type == "VFR_HUD":

                latest_baro_alt = float(
                    msg.alt
                )

                if ground_baro_alt is None:
                    ground_baro_alt = latest_baro_alt

                baro_rel_alt = max(
                    0.0,
                    latest_baro_alt - ground_baro_alt
                )

                # GLOBAL_POSITION_INT.relative_alt
                # має вищий пріоритет.
                if global_rel_alt is None:

                    update_flight_altitude(
                        baro_rel_alt,
                        current_timestamp,
                        "BARO"
                    )

                if valid_number(msg.groundspeed):
                    max_speed = max(
                        max_speed,
                        float(msg.groundspeed)
                    )

                if valid_number(msg.throttle):
                    max_throttle = max(
                        max_throttle,
                        float(msg.throttle)
                    )

            # =================================================
            # ALTITUDE
            # =================================================

            elif msg_type == "ALTITUDE":

                alt_rel = getattr(
                    msg,
                    "altitude_relative",
                    None
                )

                if valid_number(alt_rel):

                    alt_rel = max(
                        0.0,
                        float(alt_rel)
                    )

                    if (
                        global_rel_alt is None
                        and local_rel_alt is None
                    ):
                        update_flight_altitude(
                            alt_rel,
                            current_timestamp,
                            "ALTITUDE"
                        )

            # =================================================
            # LOCAL_POSITION_NED
            # =================================================

            elif msg_type == "LOCAL_POSITION_NED":

                x = getattr(msg, "x", 0.0)
                y = getattr(msg, "y", 0.0)
                z = getattr(msg, "z", 0.0)

                if (
                    valid_number(x)
                    and valid_number(y)
                    and valid_number(z)
                ):

                    x = float(x)
                    y = float(y)
                    z = float(z)

                    d_val = math.sqrt(
                        x * x + y * y
                    )

                    if 0.0 <= d_val <= 10000.0:
                        curr_dist = d_val
                        max_dist = max(
                            max_dist,
                            curr_dist
                        )

                    # NED: Z позитивний вниз.
                    ned_alt = -z

                    if -1000.0 <= ned_alt <= 1000.0:

                        local_rel_alt = max(
                            0.0,
                            ned_alt
                        )

                        if global_rel_alt is None:
                            update_flight_altitude(
                                local_rel_alt,
                                current_timestamp,
                                "LOCAL_NED"
                            )

            # =================================================
            # POSITION_TARGET_LOCAL_NED
            # =================================================

            elif msg_type == "POSITION_TARGET_LOCAL_NED":

                x = getattr(msg, "x", 0.0)
                y = getattr(msg, "y", 0.0)

                if valid_number(x) and valid_number(y):

                    x = float(x)
                    y = float(y)

                    d_val = math.sqrt(
                        x * x + y * y
                    )

                    if 0.0 <= d_val <= 10000.0:
                        curr_dist = d_val
                        max_dist = max(
                            max_dist,
                            curr_dist
                        )

            # =================================================
            # RANGEFINDER
            # =================================================

            elif msg_type in [
                "RANGEFINDER",
                "DISTANCE_SENSOR"
            ]:

                if msg_type == "RANGEFINDER":

                    rf_dist = getattr(
                        msg,
                        "distance",
                        0
                    )

                else:

                    rf_dist = (
                        getattr(
                            msg,
                            "current_distance",
                            0
                        )
                        / 100.0
                    )

                if (
                    valid_number(rf_dist)
                    and 0.1 <= float(rf_dist) <= 50.0
                    and not rangefinder_failed_flag
                ):
                    rf_dist = float(rf_dist)

                    has_rangefinder = True
                    rangefinder_alt = rf_dist

                    max_rf_alt = max(
                        max_rf_alt,
                        rf_dist
                    )

            # =================================================
            # RC CHANNELS
            # =================================================

            elif msg_type == "RC_CHANNELS":

                if (
                    hasattr(msg, "rssi")
                    and 0 < msg.rssi < 255
                ):

                    min_rssi = min(
                        min_rssi,
                        msg.rssi
                    )

                    curr_rssi_pct = round(
                        (msg.rssi / 254.0) * 100
                    )

                chans = [
                    msg.chan1_raw,
                    msg.chan2_raw,
                    msg.chan3_raw,
                    msg.chan4_raw,
                    getattr(msg, "chan5_raw", 0),
                    getattr(msg, "chan6_raw", 0),
                    getattr(msg, "chan7_raw", 0),
                    getattr(msg, "chan8_raw", 0)
                ]

                for ch_num in range(1, 9):

                    val = chans[
                        ch_num - 1
                    ]

                    if 800 < val < 2200:

                        rc_min[ch_num] = min(
                            rc_min[ch_num],
                            val
                        )

                        rc_max[ch_num] = max(
                            rc_max[ch_num],
                            val
                        )

                        # Залишаємо твої старі дії CH5-CH8
                        if ch_num >= 5:

                            prev = last_rc_state[ch_num]

                            if (
                                prev > 0
                                and abs(val - prev) > 250
                            ):

                                if val > 1600:
                                    state_str = "АКТИВНО"

                                elif 1300 <= val <= 1600:
                                    state_str = "СЕРЕДНЄ"

                                else:
                                    state_str = "ВИМК"

                                add_event(
                                    (
                                        f"🎮 CH{ch_num} переведено "
                                        f"в {state_str} ({val} us)"
                                    ),
                                    current_timestamp,
                                    current_mode,
                                    False,
                                    True,
                                    "PILOT"
                                )

                            last_rc_state[ch_num] = val

            # =================================================
            # RADIO STATUS
            # =================================================

            elif msg_type in [
                "RADIO",
                "RADIO_STATUS"
            ]:

                radio_status_seen = True

                telem_rssi_raw = getattr(
                    msg,
                    "rssi",
                    0
                )

                telem_remrssi_raw = getattr(
                    msg,
                    "remrssi",
                    0
                )

                dbm_val = parse_dbm(
                    telem_rssi_raw
                )

                curr_dbm = dbm_val

                if (
                    dbm_val != 0
                    and (
                        min_dbm == 0
                        or dbm_val < min_dbm
                    )
                ):
                    min_dbm = dbm_val

                radio_bad = (
                    dbm_val <= -128
                    or (
                        telem_rssi_raw == 0
                        and telem_remrssi_raw == 0
                    )
                )

                if radio_bad:

                    radio_bad_samples += 1

                    if radio_bad_start is None:
                        radio_bad_start = current_timestamp

                else:

                    if radio_bad_start is not None:

                        duration = (
                            current_timestamp
                            - radio_bad_start
                        )

                        max_radio_bad_duration = max(
                            max_radio_bad_duration,
                            duration
                        )

                        radio_bad_start = None

            # =================================================
            # ATTITUDE
            # =================================================

            elif msg_type == "ATTITUDE":

                if valid_number(msg.roll):
                    r_deg = abs(
                        math.degrees(
                            msg.roll
                        )
                    )

                    max_roll = max(
                        max_roll,
                        r_deg
                    )

                if valid_number(msg.pitch):
                    p_deg = abs(
                        math.degrees(
                            msg.pitch
                        )
                    )

                    max_pitch = max(
                        max_pitch,
                        p_deg
                    )

            # =================================================
            # GLOBAL_POSITION_INT
            # =================================================

            elif msg_type == "GLOBAL_POSITION_INT":

                if msg.lat != 0 or msg.lon != 0:
                    has_gps = True

                if hasattr(
                    msg,
                    "relative_alt"
                ):

                    rel_g = (
                        float(msg.relative_alt)
                        / 1000.0
                    )

                    if (
                        valid_number(rel_g)
                        and 0.0 <= rel_g <= MAX_ALTITUDE
                    ):
                        global_rel_alt = rel_g

                        update_flight_altitude(
                            global_rel_alt,
                            current_timestamp,
                            "GLOBAL_REL"
                        )

                if current_mode == "LOITER":

                    if msg.lat != 0 or msg.lon != 0:
                        loiter_has_origin = True

                    elif (
                        msg.lat == 0
                        and msg.lon == 0
                        and not loiter_has_origin
                    ):
                        loiter_origin_failed = True

            # =================================================
            # PARAM_VALUE
            # =================================================

            elif msg_type == "PARAM_VALUE":

                inspect_parameter_for_frequency(
                    getattr(
                        msg,
                        "param_id",
                        ""
                    ),
                    getattr(
                        msg,
                        "param_value",
                        None
                    ),
                    current_timestamp
                )

            # =================================================
            # VIBRATION
            # =================================================

            elif msg_type == "VIBRATION":

                max_vib_x = max(
                    max_vib_x,
                    msg.vibration_x
                )

                max_vib_y = max(
                    max_vib_y,
                    msg.vibration_y
                )

                max_vib_z = max(
                    max_vib_z,
                    msg.vibration_z
                )

                clip_count = max(
                    clip_count,
                    msg.clipping_0,
                    msg.clipping_1,
                    msg.clipping_2
                )

            # =================================================
            # TEMPERATURE
            # =================================================

            elif msg_type == "TEMPERATURE":

                raw_temp = getattr(
                    msg,
                    "temperature",
                    None
                )

                if valid_number(raw_temp):

                    raw_temp = float(raw_temp)

                    if abs(raw_temp) > 150:
                        raw_temp /= 100.0

                    update_temperature(
                        raw_temp
                    )

            elif msg_type == "HIGHRES_IMU":

                raw_temp = getattr(
                    msg,
                    "temperature",
                    None
                )

                if valid_number(raw_temp):
                    update_temperature(
                        float(raw_temp)
                    )

            elif msg_type in [
                "SCALED_PRESSURE",
                "SCALED_PRESSURE2",
                "SCALED_PRESSURE3"
            ]:

                raw_temp = getattr(
                    msg,
                    "temperature",
                    None
                )

                if valid_number(raw_temp):

                    raw_temp = float(raw_temp)

                    if raw_temp != 0:
                        update_temperature(
                            raw_temp / 100.0
                        )

            elif msg_type == "MCU_STATUS":

                raw_temp = getattr(
                    msg,
                    "mcu_temperature",
                    None
                )

                if valid_number(raw_temp):

                    raw_temp = float(raw_temp)

                    if abs(raw_temp) > 150:
                        raw_temp /= 100.0

                    update_temperature(
                        raw_temp
                    )

            # =================================================
            # STATUSTEXT
            # =================================================

            elif msg_type == "STATUSTEXT":

                try:
                    txt = clean_text(
                        msg.text
                    )

                    extract_frequencies_from_text(
                        txt,
                        current_timestamp
                    )

                    if (
                        "No rangefinder" in txt
                        or "VISP: No rangefinder" in txt
                    ):
                        rangefinder_failed_flag = True

                    severity = getattr(
                        msg,
                        "severity",
                        6
                    )

                    is_err = severity <= 4

                    prefix = (
                        "⚠️ ПОМИЛКА: "
                        if is_err
                        else "ℹ️ "
                    )

                    add_event(
                        f"{prefix}{txt}",
                        current_timestamp,
                        current_mode,
                        is_err,
                        False,
                        "SYSTEM"
                    )

                except Exception:
                    pass

        # ====================================================
        # FINAL RADIO PERIOD
        # ====================================================

        if radio_bad_start is not None:

            max_radio_bad_duration = max(
                max_radio_bad_duration,
                max(
                    0.0,
                    current_timestamp
                    - radio_bad_start
                )
            )

        # ====================================================
        # SUMMARY
        # ====================================================

        if min_voltage == 999.0:
            min_voltage = 0.0

        if start_voltage is None:
            start_voltage = 0.0

        if vnav_quality_min_loiter == 999:
            vnav_quality_min_loiter = 0

        # RANGEFINDER НЕ визначає Max Altitude
        final_max_altitude = max_alt

        final_max_altitude = max(
            0.0,
            min(
                final_max_altitude,
                MAX_ALTITUDE
            )
        )

        duration_sec = 0

        if (
            first_timestamp is not None
            and current_timestamp
        ):
            duration_sec = max(
                0,
                int(
                    current_timestamp
                    - first_timestamp
                )
            )

        mins, secs = divmod(
            duration_sec,
            60
        )

        # ====================================================
        # TIMELINE OUTPUT
        # ====================================================

        timeline = []

        base_t = first_timestamp or 0

        for ev in sorted(
            raw_timeline,
            key=lambda x: x["timestamp"]
        ):

            elapsed = max(
                0.0,
                ev["timestamp"]
                - base_t
            )

            t_minutes = int(
                elapsed // 60
            )

            t_seconds = (
                elapsed
                - t_minutes * 60
            )

            timeline.append({
                "time": (
                    f"{t_minutes:02d}:"
                    f"{t_seconds:06.3f}"
                ),

                "mode": ev["mode"],
                "alt": ev["alt"],
                "dist": ev["dist"],

                "ctrlLow": ev["ctrlLow"],
                "ctrlHigh": ev["ctrlHigh"],
                "videoFreq": ev["videoFreq"],

                "volt": ev["volt"],
                "curr": ev["curr"],

                "rssi": ev["rssi"],
                "dbm": ev["dbm"],

                "temp": ev["temp"],

                "systemText": ev["system_text"],
                "pilotText": ev["pilot_text"],

                "eventType": ev["eventType"],
                "isError": ev["isError"]
            })

        # ====================================================
        # DISPLAY VALUES
        # ====================================================

        rssi_percent = (
            round(
                (min_rssi / 254.0)
                * 100
            )
            if min_rssi != 255
            else 0
        )

        modes_str = (
            ", ".join(
                sorted(flight_modes)
            )
            if flight_modes
            else "Невідомо"
        )

        display_temp = (
            f"{round(max_temp, 1)} °C"
            if max_temp != -99.0
            else "Немає даних"
        )

        # ====================================================
        # RF STATS
        # ====================================================

        rf_duration = 0.0

        if (
            first_rf_timestamp is not None
            and last_rf_timestamp is not None
            and last_rf_timestamp > first_rf_timestamp
        ):
            rf_duration = (
                last_rf_timestamp
                - first_rf_timestamp
            )

        low_hop_rate = (
            low_hop_count / rf_duration
            if rf_duration > 0
            else 0.0
        )

        high_hop_rate = (
            high_hop_count / rf_duration
            if rf_duration > 0
            else 0.0
        )

        # ====================================================
        # AI VERDICT
        # ====================================================

        ai_alerts = []
        is_critical = False

        # ----------------------------------------------------
        # BATTERY RESTART / CHANGE
        # ----------------------------------------------------

        if reboot_or_second_battery:

            ai_alerts.append(
                (
                    "ℹ️ <b>Зафіксовано зміну живлення "
                    "/ новий політ:</b> "
                    "помітний стрибок напруги."
                )
            )

        # ----------------------------------------------------
        # RANGEFINDER
        # ----------------------------------------------------

        if rangefinder_failed_flag:

            ai_alerts.append(
                (
                    "📡 <b>Відвалився далекомір "
                    "(Rangefinder):</b> "
                    "VISP втратила зв'язок із сенсором. "
                    "Основна висота визначалась "
                    "EKF/барометром."
                )
            )

        # ----------------------------------------------------
        # LOITER
        # ----------------------------------------------------

        if "LOITER" in flight_modes:

            if (
                loiter_origin_failed
                and not loiter_has_origin
            ):

                ai_alerts.append(
                    (
                        "👁 <b>Проблема позиціонування "
                        "у Loiter:</b> "
                        "не зафіксовано коректну позиційну точку."
                    )
                )

                is_critical = True

            elif loiter_has_origin:

                ai_alerts.append(
                    (
                        "✅ <b>Навігація в Loiter:</b> "
                        "позиційна точка була зафіксована."
                    )
                )

            if vnav_samples > 0:

                if vnav_quality_min_loiter < 40:

                    ai_alerts.append(
                        (
                            "⚠️ <b>Низька якість "
                            "Оптичної Навігації:</b> "
                            f"падала до "
                            f"{round(vnav_quality_min_loiter)}%."
                        )
                    )

                else:

                    ai_alerts.append(
                        (
                            "👁 <b>Якість Оптичної Навігації:</b> "
                            f"{round(vnav_quality_min_loiter)}%"
                            "–"
                            f"{round(vnav_quality_max_loiter)}%."
                        )
                    )

        # ----------------------------------------------------
        # RADIO
        # ----------------------------------------------------

        if radio_status_seen:

            if (
                max_radio_bad_duration
                >= RADIO_DROPOUT_CRITICAL_SEC
            ):

                ai_alerts.append(
                    (
                        "📡 <b>Тривалий критичний стан "
                        "радіоканалу:</b> "
                        f"до {round(max_radio_bad_duration, 2)} с. "
                        "Потрібна перевірка RF-лінку."
                    )
                )

                is_critical = True

            elif radio_bad_samples > 0:

                ai_alerts.append(
                    (
                        "📶 <b>Зафіксовано короткі "
                        "граничні значення RADIO_STATUS.</b> "
                        "Одиничне значення -128 dBm "
                        "не трактується як втрата борта."
                    )
                )

            elif min_dbm < 0:

                ai_alerts.append(
                    (
                        "📶 <b>Мінімальний рівень "
                        "радіоканалу:</b> "
                        f"{round(min_dbm)} dBm."
                    )
                )

        # ----------------------------------------------------
        # RF / ППРЧ
        # ----------------------------------------------------

        if explicit_rf_data_found:

            ai_alerts.append(
                (
                    "📡 <b>RF-частоти знайдені у TLOG:</b> "
                    f"LOW — {low_hop_count} змін, "
                    f"HIGH — {high_hop_count} змін, "
                    f"VIDEO — {video_change_count} змін."
                )
            )

        else:

            ai_alerts.append(
                (
                    "ℹ️ <b>Фактичні RF-частоти "
                    "не знайдено у MAVLink:</b> "
                    "відомі діапазони LOW 410–485 МГц "
                    "та HIGH 820–895 МГц, але конкретну "
                    "частоту ППРЧ у кожний момент часу "
                    "неможливо відновити без її запису в TLOG."
                )
            )

        # ----------------------------------------------------
        # BATTERY
        # ----------------------------------------------------

        if 0 < min_voltage <= 16.8:

            is_critical = True

            if land_mode_triggered:

                ai_alerts.append(
                    (
                        "🪫 <b>Посадка при низькій напрузі:</b> "
                        f"{round(min_voltage, 1)} V."
                    )
                )

            else:

                ai_alerts.append(
                    (
                        "🚨 <b>Критична просадка:</b> "
                        f"{round(min_voltage, 1)} V."
                    )
                )

        elif 16.8 < min_voltage < 18.0:

            ai_alerts.append(
                (
                    "🔋 <b>Глибока просадка:</b> "
                    f"{round(min_voltage, 1)} V."
                )
            )

        # ----------------------------------------------------
        # CURRENT
        # ----------------------------------------------------

        if max_current > 80.0:

            ai_alerts.append(
                (
                    "⚡ <b>Перевищення струму:</b> "
                    f"максимальне споживання "
                    f"{round(max_current, 1)} A."
                )
            )

        # ----------------------------------------------------
        # TEMPERATURE
        # ----------------------------------------------------

        if max_temp != -99.0:

            if max_temp >= 85.0:

                ai_alerts.append(
                    (
                        "🌡 <b>Критична температура "
                        "польотного контролера:</b> "
                        f"{round(max_temp, 1)} °C."
                    )
                )

                is_critical = True

            elif max_temp >= 70.0:

                ai_alerts.append(
                    (
                        "🌡 <b>Висока температура "
                        "польотного контролера:</b> "
                        f"{round(max_temp, 1)} °C."
                    )
                )

        # ----------------------------------------------------
        # ATTITUDE
        # ----------------------------------------------------

        if max_roll > 80 or max_pitch > 80:

            ai_alerts.append(
                (
                    "🔄 <b>Зафіксовано великий кут "
                    "нахилу:</b> "
                    f"{round(max(max_roll, max_pitch), 1)}°."
                )
            )

            is_critical = True

        # ----------------------------------------------------
        # LOG END
        # ----------------------------------------------------

        log_ended_armed = (
            ever_armed
            and was_armed
        )

        if log_ended_armed:

            ai_alerts.append(
                (
                    "❗ <b>Лог закінчився при ARMED:</b> "
                    "у файлі немає підтвердженого DISARM "
                    "після завершення польоту."
                )
            )

            is_critical = True

        # ====================================================
        # FINAL VERDICT
        # ====================================================

        if landed_successfully:

            if is_critical:

                ai_verdict = (
                    "⚠️ БОРТ ПОВЕРНУВСЯ. "
                    "ПІД ЧАС ПОЛЬОТУ ЗАФІКСОВАНО "
                    "КРИТИЧНІ ПОДІЇ:"
                )

            else:

                ai_verdict = (
                    "✅ БОРТ ПОВЕРНУВСЯ "
                    "ТА ПОЛІТ ЗАВЕРШЕНО."
                )

        elif log_ended_armed:

            ai_verdict = (
                "🚨 ЛОГ ОБІРВАВСЯ ПРИ ARMED. "
                "МОЖЛИВА АВАРІЙНА СИТУАЦІЯ:"
            )

        elif is_critical:

            ai_verdict = (
                "⚠️ ПІД ЧАС ПОЛЬОТУ "
                "ЗАФІКСОВАНО КРИТИЧНІ ПОДІЇ:"
            )

        elif len(ai_alerts) > 0:

            ai_verdict = (
                "📊 РЕЗУЛЬТАТИ АНАЛІЗУ ПОЛЬОТУ:"
            )

        else:

            ai_verdict = (
                "✅ Політ пройшов "
                "у штатному режимі."
            )

        # ====================================================
        # RETURN JSON
        # ====================================================

        return {
            "success": True,

            "ai": {
                "verdict": ai_verdict,
                "isCritical": is_critical,
                "landedSuccessfully": landed_successfully,
                "alerts": ai_alerts
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
                "altitudeSource": altitude_source
            },

            "battery": {
                "armVoltage": round(start_voltage, 2),
                "minVoltage": round(min_voltage, 2),
                "maxCurrent": round(max_current, 2),
                "voltageSag": round(
                    max(
                        0,
                        start_voltage - min_voltage
                    ),
                    2
                )
            },

            "radio": {
                "rssi": (
                    f"{rssi_percent}%"
                    if min_rssi != 255
                    else "Немає"
                ),

                "maxThrottle": f"{round(max_throttle)}%",

                "hasGps": (
                    "GPS Присутній"
                    if has_gps
                    else "Без GPS / локальна навігація"
                ),

                "telemRssi": (
                    f"{round(min_dbm)} dBm"
                    if min_dbm != 0
                    else "—"
                ),

                "maxDropout": round(
                    max_radio_bad_duration,
                    2
                )
            },

            "rf": {
                "dataFound": explicit_rf_data_found,

                "lowBand": "410–485 МГц",
                "highBand": "820–895 МГц",

                "lowCurrent": (
                    round(curr_ctrl_low, 3)
                    if curr_ctrl_low is not None
                    else None
                ),

                "highCurrent": (
                    round(curr_ctrl_high, 3)
                    if curr_ctrl_high is not None
                    else None
                ),

                "videoCurrent": (
                    round(curr_video_freq, 3)
                    if curr_video_freq is not None
                    else None
                ),

                "lowHopCount": low_hop_count,
                "highHopCount": high_hop_count,
                "videoChangeCount": video_change_count,

                "lowHopRate": round(
                    low_hop_rate,
                    2
                ),

                "highHopRate": round(
                    high_hop_rate,
                    2
                ),

                "lowUnique": len(
                    low_freq_seen
                ),

                "highUnique": len(
                    high_freq_seen
                ),

                "videoUnique": len(
                    video_freq_seen
                )
            },

            "health": {
                "vibX": round(max_vib_x, 1),
                "vibY": round(max_vib_y, 1),
                "vibZ": round(max_vib_z, 1),

                "clipping": clip_count,

                "maxTemp": display_temp,

                "engineLoadLoiter": (
                    (
                        f"{round(vnav_quality_min_loiter)}% – "
                        f"{round(vnav_quality_max_loiter)}%"
                    )
                    if vnav_samples > 0
                    else "Немає даних"
                )
            },

            "radioChannels": {
                "ch1": (
                    f"{rc_min[1]}–{rc_max[1]} us"
                    if rc_max[1] > 0
                    else "—"
                ),

                "ch2": (
                    f"{rc_min[2]}–{rc_max[2]} us"
                    if rc_max[2] > 0
                    else "—"
                ),

                "ch3": (
                    f"{rc_min[3]}–{rc_max[3]} us"
                    if rc_max[3] > 0
                    else "—"
                ),

                "ch4": (
                    f"{rc_min[4]}–{rc_max[4]} us"
                    if rc_max[4] > 0
                    else "—"
                )
            },

            "timeline": timeline
        }

    finally:

        if os.path.exists(temp.name):

            try:
                os.unlink(
                    temp.name
                )

            except Exception:
                pass
