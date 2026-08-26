
import math
import os
import tempfile
import re

from fastapi import FastAPI, File, UploadFile
from pymavlink import mavutil

app = FastAPI()

# ============================================================
# CONFIG
# ============================================================

MAX_ALTITUDE = 1000.0
MAX_CLIMB_RATE = 50.0
GROUND_ALTITUDE = 0.5
RADIO_DROPOUT_CRITICAL_SEC = 2.0
VIBRATION_CRITICAL_THRESHOLD = 36.0

# Heading-based conditional antenna sector analysis.
# IMPORTANT: VFR_HUD.heading is the aircraft heading, not the physical antenna azimuth.
# This is therefore a heuristic/conditional sector, exactly as requested.
ANTENNA_BEAM_WIDTH_DEG = 30.0
ANTENNA_HALF_ANGLE_DEG = ANTENNA_BEAM_WIDTH_DEG / 2.0
HEADING_REFERENCE_STABLE_TOLERANCE_DEG = 5.0
HEADING_REFERENCE_MIN_DURATION_SEC = 10.0
HEADING_OUTSIDE_CONFIRM_SEC = 3.0
HEADING_SAMPLE_MAX_GAP_SEC = 1.6

CH7_REVERSED = False
CH8_REVERSED = False

VTX_CHANNELS = {
    1: {1: 5180, 2: 5240, 3: 5300},
    2: {1: 5520, 2: 5580, 3: 5640},
    3: {1: 5700, 2: 5765, 3: 5825},
}

VTX_BAND_NAMES = {1: "5.2", 2: "5.5", 3: "5.8"}
VTX_CHANNEL_NAMES = {1: "K1", 2: "K2", 3: "K3"}

# ============================================================
# HELPERS
# ============================================================

def valid_number(value):
    try:
        value = float(value)
        return not math.isnan(value) and not math.isinf(value)
    except (TypeError, ValueError):
        return False


def heading_difference_deg(a, b):
    """Smallest absolute angular difference, 0..180 degrees."""
    if not valid_number(a) or not valid_number(b):
        return None
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def circular_mean_deg(values):
    vals = [float(v) % 360.0 for v in values if valid_number(v)]
    if not vals:
        return None
    s = sum(math.sin(math.radians(v)) for v in vals)
    c = sum(math.cos(math.radians(v)) for v in vals)
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return vals[0]
    return math.degrees(math.atan2(s, c)) % 360.0


def format_timeline_time(timestamp, base_t):
    elapsed = float(timestamp) - float(base_t)
    sign = "-" if elapsed < 0 else ""
    elapsed = abs(elapsed)
    minutes = int(elapsed // 60)
    seconds = elapsed - minutes * 60
    return f"{sign}{minutes:02d}:{seconds:06.3f}"


def analyze_heading_sector(raw_timeline, arm_timestamp):
    """
    Find the longest stable heading segment and use its circular mean as the
    conditional antenna-sector center. Then identify confirmed excursions
    outside +/-15 deg lasting at least HEADING_OUTSIDE_CONFIRM_SEC.
    """
    result = {
        "available": False,
        "reference": None,
        "sectorMin": None,
        "sectorMax": None,
        "beamWidth": ANTENNA_BEAM_WIDTH_DEG,
        "halfAngle": ANTENNA_HALF_ANGLE_DEG,
        "referenceDuration": 0.0,
        "episodes": [],
        "firstOutsideTimestamp": None,
        "maxDeviation": 0.0,
        "returnedToSector": False,
    }

    if arm_timestamp is None:
        return result

    # One-Hz snapshots are deliberately used so duplicate event rows do not
    # artificially extend a stable/outside period.
    samples = []
    for ev in sorted(raw_timeline, key=lambda x: x.get("timestamp", 0)):
        if ev.get("eventType") != "SNAPSHOT":
            continue
        ts = ev.get("timestamp")
        hdg = ev.get("azimuth")
        if ts is None or ts < arm_timestamp or not valid_number(hdg):
            continue
        samples.append((float(ts), float(hdg) % 360.0))

    if len(samples) < 2:
        return result

    # Longest contiguous stable segment. A new sample must remain within the
    # tolerance of the current circular mean.
    best = []
    current = [samples[0]]
    for sample in samples[1:]:
        ts, hdg = sample
        prev_ts = current[-1][0]
        mean = circular_mean_deg([h for _, h in current])
        diff = heading_difference_deg(hdg, mean)
        if (
            ts - prev_ts <= HEADING_SAMPLE_MAX_GAP_SEC
            and diff is not None
            and diff <= HEADING_REFERENCE_STABLE_TOLERANCE_DEG
        ):
            current.append(sample)
        else:
            if len(current) > len(best):
                best = current
            current = [sample]
    if len(current) > len(best):
        best = current

    if len(best) < 2:
        return result

    stable_duration = best[-1][0] - best[0][0]
    if stable_duration < HEADING_REFERENCE_MIN_DURATION_SEC:
        return result

    reference = circular_mean_deg([h for _, h in best])
    if reference is None:
        return result

    result["available"] = True
    result["reference"] = round(reference, 1)
    result["sectorMin"] = round((reference - ANTENNA_HALF_ANGLE_DEG) % 360.0, 1)
    result["sectorMax"] = round((reference + ANTENNA_HALF_ANGLE_DEG) % 360.0, 1)
    result["referenceDuration"] = round(stable_duration, 1)

    # Detect confirmed outside-sector episodes on 1 Hz samples.
    episodes = []
    active = None
    for ts, hdg in samples:
        dev = heading_difference_deg(hdg, reference) or 0.0
        outside = dev > ANTENNA_HALF_ANGLE_DEG

        if outside:
            if active is None:
                active = {
                    "start": ts,
                    "end": ts,
                    "firstHeading": hdg,
                    "maxDeviation": dev,
                    "maxHeading": hdg,
                }
            else:
                # A large telemetry gap ends the previous episode.
                if ts - active["end"] > HEADING_SAMPLE_MAX_GAP_SEC:
                    duration = active["end"] - active["start"]
                    if duration >= HEADING_OUTSIDE_CONFIRM_SEC:
                        active["duration"] = duration
                        episodes.append(active)
                    active = {
                        "start": ts,
                        "end": ts,
                        "firstHeading": hdg,
                        "maxDeviation": dev,
                        "maxHeading": hdg,
                    }
                else:
                    active["end"] = ts
                    if dev > active["maxDeviation"]:
                        active["maxDeviation"] = dev
                        active["maxHeading"] = hdg
        else:
            if active is not None:
                duration = active["end"] - active["start"]
                if duration >= HEADING_OUTSIDE_CONFIRM_SEC:
                    active["duration"] = duration
                    active["returned"] = True
                    active["returnTimestamp"] = ts
                    episodes.append(active)
                active = None

    if active is not None:
        duration = active["end"] - active["start"]
        if duration >= HEADING_OUTSIDE_CONFIRM_SEC:
            active["duration"] = duration
            active["returned"] = False
            active["returnTimestamp"] = None
            episodes.append(active)

    for ep in episodes:
        ep.setdefault("returned", False)
        ep.setdefault("returnTimestamp", None)
        ep["start"] = float(ep["start"])
        ep["end"] = float(ep["end"])
        ep["duration"] = round(float(ep["duration"]), 1)
        ep["firstHeading"] = round(float(ep["firstHeading"]), 1)
        ep["maxHeading"] = round(float(ep["maxHeading"]), 1)
        ep["maxDeviation"] = round(float(ep["maxDeviation"]), 1)

    result["episodes"] = episodes
    if episodes:
        result["firstOutsideTimestamp"] = episodes[0]["start"]
        result["maxDeviation"] = max(ep["maxDeviation"] for ep in episodes)
        result["returnedToSector"] = all(ep.get("returned", False) for ep in episodes)

    # Annotate every timeline row. Rows inside a confirmed episode become red
    # in the HTML, including event rows that occur between the 1 Hz snapshots.
    for ev in raw_timeline:
        hdg = ev.get("azimuth")
        ts = ev.get("timestamp")
        if not valid_number(hdg) or ts is None:
            ev["headingSector"] = None
            continue

        dev = heading_difference_deg(float(hdg), reference) or 0.0
        confirmed = any(ep["start"] <= ts <= ep["end"] for ep in episodes)
        ev["headingSector"] = {
            "reference": round(reference, 1),
            "sectorMin": result["sectorMin"],
            "sectorMax": result["sectorMax"],
            "deviation": round(dev, 1),
            "outside": dev > ANTENNA_HALF_ANGLE_DEG,
            "confirmedOutside": confirmed,
            "beamWidth": ANTENNA_BEAM_WIDTH_DEG,
        }

    return result


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
        try:
            return value.decode("utf-8", errors="ignore").replace("\x00", "").strip()
        except Exception:
            return str(value)

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


def parse_initial_pos_ned(text):
    """
    Parses messages such as:
      EKF3 IMU0 initial pos NED = 0.0,0.0,0.0 (m)
      EKF3 IMU0 initial pos NED = -0.1,0.4,0.8,0.0 (m)

    Returns (N, E, D) for the first three values, or None.
    """
    if not text:
        return None

    text = clean_text(text)

    if "initial pos NED" not in text or "=" not in text:
        return None

    values_part = text.split("=", 1)[1]
    values_part = (
        values_part
        .replace("(m)", "")
        .replace("m)", "")
        .strip()
    )

    nums = re.findall(r"[-+]?\d+(?:\.\d+)?", values_part)

    if len(nums) < 3:
        return None

    try:
        return (
            float(nums[0]),
            float(nums[1]),
            float(nums[2]),
        )
    except (TypeError, ValueError):
        return None


def is_primary_false_ned(coords, limit=0.9):
    """
    Primary false/initial optical coordinates may be small non-zero values.
    Examples: 0.1,0.2,0.3 or -0.1,0.4,-0.8.
    All three N/E/D values must be within ±limit meters.
    """
    if not coords or len(coords) < 3:
        return False

    return all(abs(float(v)) <= limit for v in coords[:3])


def format_ned(coords):
    if not coords:
        return None

    def fmt(v):
        if abs(v) < 0.0005:
            v = 0.0
        return f"{v:.1f}"

    return ",".join(fmt(v) for v in coords[:3])


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

    temp = tempfile.NamedTemporaryFile(delete=False, suffix=".tlog")
    temp.write(data)
    temp.close()

    try:
        mav = mavutil.mavlink_connection(temp.name)

        # Base
        message_count = 0
        max_alt = 0.0
        max_speed = 0.0
        max_roll = 0.0
        max_pitch = 0.0
        max_dist = 0.0

        # Altitude
        curr_alt = 0.0
        last_valid_alt = 0.0
        last_alt_timestamp = None
        latest_baro_alt = None
        ground_baro_alt = None
        baro_rel_alt = None
        global_rel_alt = None
        local_rel_alt = None
        altitude_source = "NONE"

        # Rangefinder
        rangefinder_failed_flag = False

        # Arm / land
        is_currently_armed = False
        was_armed = False
        ever_armed = False
        landed_successfully = False
        disarm_detected = False
        arm_timestamp = None
        disarm_timestamp = None

        # Current state
        curr_dist = 0.0
        curr_azimuth = None
        curr_voltage = 0.0
        curr_amp = 0.0
        curr_rssi_pct = 0
        curr_dbm = 0

        # VTX
        curr_video_freq = None
        curr_vtx_band = None
        curr_vtx_channel = None
        last_video_freq = None
        video_change_count = 0
        video_freq_seen = set()
        ch7_current = 0
        ch8_current = 0

        # Radio
        min_rssi = 255
        min_dbm = 0
        telem_rssi_raw = None
        telem_remrssi_raw = None
        radio_status_seen = False
        radio_bad_start = None
        max_radio_bad_duration = 0.0
        radio_bad_samples = 0

        # RC
        rc_min = {i: 9999 for i in range(1, 19)}
        rc_max = {i: 0 for i in range(1, 19)}
        last_rc_state = {i: 0 for i in range(1, 19)}
        max_throttle = 0

        # Battery
        min_voltage = 999.0
        max_current = 0.0
        start_voltage = None
        arm_voltage = None
        reboot_or_second_battery = False

        # Health
        max_vib_x = 0.0
        max_vib_y = 0.0
        max_vib_z = 0.0
        curr_vib_x = None
        curr_vib_y = None
        curr_vib_z = None
        vibration_critical_events = []
        vibration_critical_active = False
        vibration_critical_peak = None
        clip_count = 0
        curr_temp = None
        max_temp = -99.0
        temp_source_priority = 0

        # ESC telemetry (ESC1..ESC4)
        esc_temp_current = [None, None, None, None]
        esc_temp_max = [None, None, None, None]
        esc_rpm_current = [None, None, None, None]
        esc_rpm_max = [0, 0, 0, 0]
        esc_current_current = [None, None, None, None]
        esc_current_max = [0.0, 0.0, 0.0, 0.0]

        # Optical navigation / EKF
        # First false coordinates do not have to be exact zeroes.
        optical_zero_detected = False  # compatibility field
        optical_zero_timestamp = None
        optical_zero_text = None

        first_loiter_timestamp = None
        first_loiter_mode_seen = False

        primary_false_ned_detected = False
        primary_false_ned_coords = None
        primary_false_ned_timestamp = None
        primary_false_ned_text = None

        ned_initializations = []

        # SYS_STATUS.load is kept only as a diagnostic field.
        # It is NOT treated as optical-navigation quality in the AI conclusion.
        vnav_quality_min_loiter = 999
        vnav_quality_max_loiter = 0
        vnav_samples = 0

        ekf_variance_count = 0
        ekf_stopped_aiding_count = 0
        loiter_position_fail_count = 0
        external_nav_recovery_count = 0
        smart_rtl_bad_position_count = 0
        prearm_position_count = 0

        # Navigation
        has_gps = False

        # Time / modes
        first_timestamp = None
        current_timestamp = 0.0
        current_mode = "Невідомо"
        flight_modes = set()
        land_mode_triggered = False
        disarm_mode = None

        # Timeline
        raw_timeline = []
        last_snapshot_second = None

        # STATUSTEXT MAVLink2 chunks
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

        def update_temperature(value, priority=1):
            nonlocal curr_temp, max_temp, temp_source_priority

            if not valid_number(value):
                return

            value = float(value)

            if not (-50.0 < value < 150.0):
                return

            if priority >= temp_source_priority:
                curr_temp = value
                temp_source_priority = priority

            if max_temp == -99.0 or value > max_temp:
                max_temp = value

        def vibration_snapshot():
            if curr_vib_x is None or curr_vib_y is None or curr_vib_z is None:
                return None

            return {
                "x": round(curr_vib_x, 1),
                "y": round(curr_vib_y, 1),
                "z": round(curr_vib_z, 1),
                "isCritical": max(curr_vib_x, curr_vib_y, curr_vib_z) >= VIBRATION_CRITICAL_THRESHOLD,
                "threshold": VIBRATION_CRITICAL_THRESHOLD,
            }

        def add_event(
            text,
            t_stamp,
            mode,
            is_error=False,
            is_pilot_action=False,
            event_type="SYSTEM",
        ):
            raw_timeline.append(
                {
                    "timestamp": t_stamp or 0,
                    "mode": mode,
                    "alt": f"{round(curr_alt, 1)} м",
                    "dist": f"{round(curr_dist, 1)} м" if curr_dist > 0 else "0.0 м",
                    "azimuth": round(curr_azimuth, 1) if curr_azimuth is not None else None,
                    "vtxBand": curr_vtx_band,
                    "vtxChannel": curr_vtx_channel,
                    "videoFreq": curr_video_freq,
                    "volt": round(curr_voltage, 2) if curr_voltage > 0 else None,
                    "curr": round(curr_amp, 1) if curr_amp >= 0 else None,
                    "rssi": curr_rssi_pct if curr_rssi_pct > 0 else None,
                    "dbm": round(curr_dbm) if curr_dbm != 0 else None,
                    "temp": round(curr_temp, 1) if curr_temp is not None else None,
                    "esc": [
                        {
                            "id": i + 1,
                            "temp": round(esc_temp_current[i], 1) if esc_temp_current[i] is not None else None,
                            "maxTemp": round(esc_temp_max[i], 1) if esc_temp_max[i] is not None else None,
                            "rpm": int(esc_rpm_current[i]) if esc_rpm_current[i] is not None else None,
                            "maxRpm": int(esc_rpm_max[i]),
                            "current": round(esc_current_current[i], 1) if esc_current_current[i] is not None else None,
                            "maxCurrent": round(esc_current_max[i], 1),
                        }
                        for i in range(4)
                    ],
                    "vibration": vibration_snapshot(),
                    "system_text": "" if is_pilot_action else text,
                    "pilot_text": text if is_pilot_action else "",
                    "eventType": event_type,
                    "isError": is_error,
                }
            )

        def add_snapshot(t_stamp):
            """Додає телеметричний рядок без системної/пілотської події."""
            raw_timeline.append(
                {
                    "timestamp": t_stamp or 0,
                    "mode": current_mode,
                    "alt": f"{round(curr_alt, 1)} м",
                    "dist": f"{round(curr_dist, 1)} м" if curr_dist > 0 else "0.0 м",
                    "azimuth": round(curr_azimuth, 1) if curr_azimuth is not None else None,
                    "vtxBand": curr_vtx_band,
                    "vtxChannel": curr_vtx_channel,
                    "videoFreq": curr_video_freq,
                    "volt": round(curr_voltage, 2) if curr_voltage > 0 else None,
                    "curr": round(curr_amp, 1) if curr_amp >= 0 else None,
                    "rssi": curr_rssi_pct if curr_rssi_pct > 0 else None,
                    "dbm": round(curr_dbm) if curr_dbm != 0 else None,
                    "temp": round(curr_temp, 1) if curr_temp is not None else None,
                    "esc": [
                        {
                            "id": i + 1,
                            "temp": round(esc_temp_current[i], 1) if esc_temp_current[i] is not None else None,
                            "maxTemp": round(esc_temp_max[i], 1) if esc_temp_max[i] is not None else None,
                            "rpm": int(esc_rpm_current[i]) if esc_rpm_current[i] is not None else None,
                            "maxRpm": int(esc_rpm_max[i]),
                            "current": round(esc_current_current[i], 1) if esc_current_current[i] is not None else None,
                            "maxCurrent": round(esc_current_max[i], 1),
                        }
                        for i in range(4)
                    ],
                    "vibration": vibration_snapshot(),
                    "system_text": "",
                    "pilot_text": "",
                    "eventType": "SNAPSHOT",
                    "isError": False,
                }
            )

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

        def process_complete_statustext(full_txt, severity, timestamp, mode):
            nonlocal rangefinder_failed_flag
            nonlocal optical_zero_detected, optical_zero_timestamp, optical_zero_text
            nonlocal primary_false_ned_detected
            nonlocal primary_false_ned_coords
            nonlocal primary_false_ned_timestamp
            nonlocal primary_false_ned_text
            nonlocal ekf_variance_count, ekf_stopped_aiding_count
            nonlocal loiter_position_fail_count, external_nav_recovery_count
            nonlocal smart_rtl_bad_position_count, prearm_position_count

            txt_lower = full_txt.lower()

            if "no rangefinder" in txt_lower or "visp: no rangefinder" in txt_lower:
                rangefinder_failed_flag = True

            if "ekf variance" in txt_lower:
                ekf_variance_count += 1

            if "stopped aiding" in txt_lower:
                ekf_stopped_aiding_count += 1

            if "mode change to loiter failed" in txt_lower and "requires position" in txt_lower:
                loiter_position_fail_count += 1

            if "using external nav data" in txt_lower:
                external_nav_recovery_count += 1

            if "smartrtl deactivated" in txt_lower and "bad position" in txt_lower:
                smart_rtl_bad_position_count += 1

            if "prearm: need position estimate" in txt_lower:
                prearm_position_count += 1

            # Save every original "initial pos NED" found in the TLOG.
            ned_coords = parse_initial_pos_ned(full_txt)

            if ned_coords is not None:
                item = {
                    "timestamp": timestamp,
                    "mode": mode,
                    "coords": ned_coords,
                    "text": full_txt,
                    "isSmallPrimaryCandidate": is_primary_false_ned(ned_coords),
                }
                ned_initializations.append(item)

            is_err = severity <= 4

            # Keep original ArduPilot STATUSTEXT in timeline.
            add_event(
                full_txt,
                timestamp,
                mode,
                is_err,
                False,
                "SYSTEM",
            )

        # ====================================================
        # MAVLINK LOOP
        # ====================================================

        needed_messages = [
            "HEARTBEAT", "SYS_STATUS", "VFR_HUD", "ALTITUDE",
            "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "RC_CHANNELS",
            "RADIO", "RADIO_STATUS", "ATTITUDE", "VIBRATION",
            "TEMPERATURE", "HIGHRES_IMU", "SCALED_PRESSURE",
            "SCALED_PRESSURE2", "SCALED_PRESSURE3", "MCU_STATUS",
            "STATUSTEXT", "ESC_TELEMETRY_1_TO_4",
        ]

        while True:
            msg = mav.recv_match(type=needed_messages, blocking=False)

            if msg is None:
                break

            message_count += 1
            msg_type = msg.get_type()
            t_stamp = getattr(msg, "_timestamp", 0.0)

            if t_stamp > 0:
                current_timestamp = t_stamp

                if first_timestamp is None:
                    first_timestamp = t_stamp

                # 1 Hz telemetry timeline while ARMED.
                # 00:00 corresponds to ARM; events keep their exact millisecond timestamps.
                if is_currently_armed and arm_timestamp is not None:
                    flight_second = int(current_timestamp - arm_timestamp)
                    if flight_second >= 0 and flight_second != last_snapshot_second:
                        last_snapshot_second = flight_second
                        add_snapshot(arm_timestamp + flight_second)

            # HEARTBEAT
            if msg_type == "HEARTBEAT":
                if msg.get_srcComponent() == 1:
                    new_mode = mav.flightmode
                    is_armed = bool(
                        msg.base_mode
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )

                    is_currently_armed = is_armed

                    if new_mode and new_mode != current_mode:
                        if current_mode != "Невідомо":
                            add_event(
                                f"🔄 Режим змінено на {new_mode}",
                                current_timestamp,
                                new_mode,
                            )

                        current_mode = new_mode
                        flight_modes.add(current_mode)

                        if current_mode == "LOITER" and first_loiter_timestamp is None:
                            first_loiter_timestamp = current_timestamp
                            first_loiter_mode_seen = True

                        if current_mode == "LAND":
                            land_mode_triggered = True

                    if is_armed and not was_armed:
                        ever_armed = True
                        arm_timestamp = current_timestamp

                        if latest_baro_alt is not None:
                            ground_baro_alt = latest_baro_alt

                        if curr_alt < 2.0:
                            curr_alt = 0.0
                            last_valid_alt = 0.0

                        if curr_voltage > 0:
                            arm_voltage = curr_voltage

                        add_event(
                            "🟢 Двигуни запущено",
                            current_timestamp,
                            current_mode,
                        )

                        was_armed = True

                    elif not is_armed and was_armed:
                        disarm_detected = True
                        disarm_timestamp = current_timestamp
                        disarm_mode = current_mode

                        if curr_alt < 5.0 or current_mode == "LAND":
                            landed_successfully = True
                            curr_alt = 0.0
                            last_valid_alt = 0.0

                        add_event(
                            "🔴 Двигуни зупинено",
                            current_timestamp,
                            current_mode,
                        )

                        was_armed = False

            # SYS_STATUS
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
                            current_mode,
                        )

                    curr_voltage = volt

                    if start_voltage is None:
                        start_voltage = volt

                    if is_currently_armed and arm_voltage is None:
                        arm_voltage = volt

                    if is_currently_armed:
                        min_voltage = min(min_voltage, volt)

                if curr >= 0:
                    curr_amp = curr

                    if is_currently_armed:
                        max_current = max(max_current, curr)

                # Повертаємо твою стару логіку "якості оптичної навігації"
                if current_mode == "LOITER":
                    vnav_val = getattr(msg, "load", 0) / 10.0

                    if vnav_val > 0:
                        vnav_samples += 1
                        vnav_quality_min_loiter = min(
                            vnav_quality_min_loiter,
                            vnav_val,
                        )
                        vnav_quality_max_loiter = max(
                            vnav_quality_max_loiter,
                            vnav_val,
                        )

            # VFR_HUD
            elif msg_type == "VFR_HUD":
                latest_baro_alt = float(msg.alt)

                # Азимут/курс БПЛА безпосередньо з MAVLink VFR_HUD.heading.
                # У Timeline показуємо тільки числове значення в градусах.
                heading_val = getattr(msg, "heading", None)
                if valid_number(heading_val):
                    heading_val = float(heading_val)
                    if 0.0 <= heading_val <= 360.0:
                        curr_azimuth = heading_val % 360.0

                if ground_baro_alt is None:
                    ground_baro_alt = latest_baro_alt

                baro_rel_alt = max(
                    0.0,
                    latest_baro_alt - ground_baro_alt,
                )

                if global_rel_alt is None:
                    update_flight_altitude(
                        baro_rel_alt,
                        current_timestamp,
                        "BARO",
                    )

                if valid_number(msg.groundspeed):
                    max_speed = max(
                        max_speed,
                        float(msg.groundspeed),
                    )

                if valid_number(msg.throttle):
                    max_throttle = max(
                        max_throttle,
                        float(msg.throttle),
                    )

            # ALTITUDE
            elif msg_type == "ALTITUDE":
                alt_rel = getattr(
                    msg,
                    "altitude_relative",
                    None,
                )

                if valid_number(alt_rel):
                    alt_rel = max(
                        0.0,
                        float(alt_rel),
                    )

                    if global_rel_alt is None and local_rel_alt is None:
                        update_flight_altitude(
                            alt_rel,
                            current_timestamp,
                            "ALTITUDE",
                        )

            # LOCAL POSITION
            elif msg_type == "LOCAL_POSITION_NED":
                x = getattr(msg, "x", 0.0)
                y = getattr(msg, "y", 0.0)
                z = getattr(msg, "z", 0.0)

                if valid_number(x) and valid_number(y) and valid_number(z):
                    x = float(x)
                    y = float(y)
                    z = float(z)

                    d_val = math.sqrt(x * x + y * y)

                    if 0.0 <= d_val <= 10000.0:
                        curr_dist = d_val
                        max_dist = max(
                            max_dist,
                            curr_dist,
                        )


                    ned_alt = -z

                    if -1000.0 <= ned_alt <= 1000.0:
                        local_rel_alt = max(
                            0.0,
                            ned_alt,
                        )

                        if global_rel_alt is None:
                            update_flight_altitude(
                                local_rel_alt,
                                current_timestamp,
                                "LOCAL_NED",
                            )

            # RC CHANNELS
            elif msg_type == "RC_CHANNELS":
                if hasattr(msg, "rssi") and 0 < msg.rssi < 255:
                    min_rssi = min(
                        min_rssi,
                        msg.rssi,
                    )

                    curr_rssi_pct = round(
                        (msg.rssi / 254.0) * 100
                    )

                channels = {}

                for ch_num in range(1, 19):
                    val = getattr(
                        msg,
                        f"chan{ch_num}_raw",
                        0,
                    )

                    channels[ch_num] = val

                    if valid_number(val) and 800 < val < 2200:
                        rc_min[ch_num] = min(
                            rc_min[ch_num],
                            val,
                        )

                        rc_max[ch_num] = max(
                            rc_max[ch_num],
                            val,
                        )

                ch7 = channels.get(7, 0)
                ch8 = channels.get(8, 0)

                if 800 < ch7 < 2200 and 800 < ch8 < 2200:
                    ch7_current = ch7
                    ch8_current = ch8

                    update_vtx_from_rc(
                        ch7,
                        ch8,
                        current_timestamp,
                    )

                for ch_num in range(5, 9):
                    val = channels.get(
                        ch_num,
                        0,
                    )

                    if not 800 < val < 2200:
                        continue

                    prev = last_rc_state[ch_num]

                    if prev > 0 and abs(val - prev) > 250:
                        pos = three_position_switch(val)

                        if pos == 1:
                            state_str = "ПОЗИЦІЯ 1"
                        elif pos == 2:
                            state_str = "ПОЗИЦІЯ 2"
                        else:
                            state_str = "ПОЗИЦІЯ 3"

                        add_event(
                            f"🎮 CH{ch_num}: {state_str} ({val} us)",
                            current_timestamp,
                            current_mode,
                            False,
                            True,
                            "PILOT",
                        )

                    last_rc_state[ch_num] = val

            # RADIO
            elif msg_type in ["RADIO", "RADIO_STATUS"]:
                radio_status_seen = True

                telem_rssi_raw = getattr(msg, "rssi", 0)
                telem_remrssi_raw = getattr(msg, "remrssi", 0)

                dbm_val = parse_dbm(telem_rssi_raw)
                curr_dbm = dbm_val

                if dbm_val != 0 and (min_dbm == 0 or dbm_val < min_dbm):
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
                        duration = current_timestamp - radio_bad_start

                        max_radio_bad_duration = max(
                            max_radio_bad_duration,
                            duration,
                        )

                        radio_bad_start = None

            # ATTITUDE
            elif msg_type == "ATTITUDE":
                if valid_number(msg.roll):
                    max_roll = max(
                        max_roll,
                        abs(math.degrees(msg.roll)),
                    )

                if valid_number(msg.pitch):
                    max_pitch = max(
                        max_pitch,
                        abs(math.degrees(msg.pitch)),
                    )

            # GLOBAL POSITION
            elif msg_type == "GLOBAL_POSITION_INT":
                if msg.lat != 0 or msg.lon != 0:
                    has_gps = True

                if hasattr(msg, "relative_alt"):
                    rel_g = float(msg.relative_alt) / 1000.0

                    if valid_number(rel_g) and 0.0 <= rel_g <= MAX_ALTITUDE:
                        global_rel_alt = rel_g

                        update_flight_altitude(
                            global_rel_alt,
                            current_timestamp,
                            "GLOBAL_REL",
                        )

            # VIBRATION
            elif msg_type == "VIBRATION":
                vib_x = float(msg.vibration_x) if valid_number(msg.vibration_x) else 0.0
                vib_y = float(msg.vibration_y) if valid_number(msg.vibration_y) else 0.0
                vib_z = float(msg.vibration_z) if valid_number(msg.vibration_z) else 0.0

                curr_vib_x = vib_x
                curr_vib_y = vib_y
                curr_vib_z = vib_z

                max_vib_x = max(max_vib_x, vib_x)
                max_vib_y = max(max_vib_y, vib_y)
                max_vib_z = max(max_vib_z, vib_z)

                vib_peak = max(vib_x, vib_y, vib_z)
                vib_is_critical = vib_peak >= VIBRATION_CRITICAL_THRESHOLD

                # Створюємо окремий червоний рядок на початку кожного
                # епізоду критичних вібрацій, щоб не засмічувати timeline.
                if vib_is_critical and not vibration_critical_active:
                    vibration_critical_active = True
                    vibration_critical_peak = {
                        "timestamp": current_timestamp,
                        "mode": current_mode,
                        "x": vib_x,
                        "y": vib_y,
                        "z": vib_z,
                        "peak": vib_peak,
                    }
                    vibration_critical_events.append(vibration_critical_peak)

                    add_event(
                        f"🚨 Критичні вібрації: X={vib_x:.1f}, Y={vib_y:.1f}, Z={vib_z:.1f}",
                        current_timestamp,
                        current_mode,
                        True,
                        False,
                        "VIBRATION_CRITICAL",
                    )

                elif vib_is_critical and vibration_critical_active and vibration_critical_peak is not None:
                    # Оновлюємо максимум поточного критичного епізоду.
                    if vib_peak > vibration_critical_peak["peak"]:
                        vibration_critical_peak.update({
                            "x": vib_x,
                            "y": vib_y,
                            "z": vib_z,
                            "peak": vib_peak,
                        })

                elif not vib_is_critical:
                    vibration_critical_active = False
                    vibration_critical_peak = None

                clip_count = max(
                    clip_count,
                    msg.clipping_0,
                    msg.clipping_1,
                    msg.clipping_2,
                )

            # TEMP
            elif msg_type == "TEMPERATURE":
                raw_temp = getattr(
                    msg,
                    "temperature",
                    None,
                )

                if valid_number(raw_temp):
                    raw_temp = float(raw_temp)

                    if abs(raw_temp) > 150:
                        raw_temp /= 100.0

                    update_temperature(
                        raw_temp,
                        1,
                    )

            elif msg_type == "HIGHRES_IMU":
                raw_temp = getattr(
                    msg,
                    "temperature",
                    None,
                )

                if valid_number(raw_temp):
                    update_temperature(
                        float(raw_temp),
                        2,
                    )

            elif msg_type in [
                "SCALED_PRESSURE",
                "SCALED_PRESSURE2",
                "SCALED_PRESSURE3",
            ]:
                raw_temp = getattr(
                    msg,
                    "temperature",
                    None,
                )

                if valid_number(raw_temp) and float(raw_temp) != 0:
                    update_temperature(
                        float(raw_temp) / 100.0,
                        1,
                    )

            elif msg_type == "MCU_STATUS":
                raw_temp = getattr(
                    msg,
                    "mcu_temperature",
                    None,
                )

                if valid_number(raw_temp):
                    raw_temp = float(raw_temp)

                    if abs(raw_temp) > 150:
                        raw_temp /= 100.0

                    update_temperature(
                        raw_temp,
                        3,
                    )

            # ESC TELEMETRY 1..4
            elif msg_type == "ESC_TELEMETRY_1_TO_4":
                temperatures = list(getattr(msg, "temperature", []) or [])
                rpms = list(getattr(msg, "rpm", []) or [])
                currents = list(getattr(msg, "current", []) or [])

                for i in range(4):
                    if i < len(temperatures):
                        t = temperatures[i]
                        if valid_number(t):
                            t = float(t)
                            if 0 < t < 150:
                                esc_temp_current[i] = t
                                if esc_temp_max[i] is None or t > esc_temp_max[i]:
                                    esc_temp_max[i] = t

                    if i < len(rpms):
                        rpm = rpms[i]
                        if valid_number(rpm):
                            rpm = float(rpm)
                            if rpm >= 0:
                                esc_rpm_current[i] = rpm
                                if rpm > esc_rpm_max[i]:
                                    esc_rpm_max[i] = rpm

                    if i < len(currents):
                        raw_current = currents[i]
                        if valid_number(raw_current):
                            # MAVLink ESC_TELEMETRY current = centiampere (0.01 A)
                            amps = float(raw_current) / 100.0
                            if 0 <= amps < 500:
                                esc_current_current[i] = amps
                                if amps > esc_current_max[i]:
                                    esc_current_max[i] = amps

            # STATUSTEXT with MAVLink2 chunk reassembly
            elif msg_type == "STATUSTEXT":
                try:
                    txt = clean_text(msg.text)
                    severity = getattr(msg, "severity", 6)
                    msg_id = getattr(msg, "id", 0)
                    chunk_seq = getattr(msg, "chunk_seq", 0)

                    if not msg_id:
                        process_complete_statustext(
                            txt,
                            severity,
                            current_timestamp,
                            current_mode,
                        )
                        continue

                    if chunk_seq == 0:
                        statustext_chunks[msg_id] = {
                            "text": txt,
                            "severity": severity,
                            "timestamp": current_timestamp,
                            "mode": current_mode,
                            "next_seq": 1,
                        }

                    elif msg_id in statustext_chunks:
                        item = statustext_chunks[msg_id]

                        if chunk_seq == item["next_seq"]:
                            item["text"] += txt
                            item["next_seq"] += 1

                    if (
                        len(txt.encode("utf-8", errors="ignore")) < 50
                        and msg_id in statustext_chunks
                    ):
                        item = statustext_chunks.pop(msg_id)

                        process_complete_statustext(
                            item["text"],
                            item["severity"],
                            item["timestamp"],
                            item["mode"],
                        )

                except Exception:
                    pass

        # Flush incomplete STATUSTEXT chunks at EOF
        for item in list(statustext_chunks.values()):
            process_complete_statustext(
                item["text"],
                item["severity"],
                item["timestamp"],
                item["mode"],
            )

        # ====================================================
        # PRIMARY FALSE NED SELECTION
        # ====================================================
        # ВАЖЛИВО:
        # первинні хибні координати не обов'язково рівні 0.0,0.0,0.0.
        # Допускаємо змішані значення в межах приблизно -0.9 ... +0.9 м.
        #
        # Головний критерій за фактичними логами:
        # беремо ПЕРШИЙ малий "initial pos NED", який реально записаний
        # у LOITER. Не прив'язуємо його до жорсткого вікна 5/15/60 секунд,
        # бо STATUSTEXT може з'явитися пізніше від самого HEARTBEAT/зміни режиму.

        primary_false_ned_detected = False
        primary_false_ned_coords = None
        primary_false_ned_timestamp = None
        primary_false_ned_text = None

        small_candidates = [
            item
            for item in ned_initializations
            if item["isSmallPrimaryCandidate"]
        ]

        # 1) Найнадійніше: сам STATUSTEXT був отриманий, коли поточний режим LOITER.
        small_in_loiter = [
            item
            for item in small_candidates
            if str(item.get("mode", "")).upper() == "LOITER"
        ]

        if small_in_loiter:
            selected = min(
                small_in_loiter,
                key=lambda item: (
                    item["timestamp"]
                    if item["timestamp"] is not None
                    else float("inf")
                ),
            )

            primary_false_ned_detected = True
            primary_false_ned_coords = selected["coords"]
            primary_false_ned_timestamp = selected["timestamp"]
            primary_false_ned_text = selected["text"]

        # 2) Fallback: якщо STATUSTEXT прийшов буквально біля першого LOITER,
        # але поле mode ще не встигло оновитися через порядок MAVLink-пакетів.
        elif first_loiter_timestamp is not None and small_candidates:
            around_first_loiter = [
                item
                for item in small_candidates
                if (
                    item["timestamp"] is not None
                    and abs(item["timestamp"] - first_loiter_timestamp) <= 120.0
                )
            ]

            if around_first_loiter:
                selected = min(
                    around_first_loiter,
                    key=lambda item: abs(
                        item["timestamp"] - first_loiter_timestamp
                    ),
                )

                primary_false_ned_detected = True
                primary_false_ned_coords = selected["coords"]
                primary_false_ned_timestamp = selected["timestamp"]
                primary_false_ned_text = selected["text"]

        # 3) Останній fallback:
        # якщо у логові є тільки один малий initial pos NED, не ігноруємо його.
        # Це захищає від ситуацій, коли mode у STATUSTEXT ще "Невідомо".
        elif len(small_candidates) == 1:
            selected = small_candidates[0]

            primary_false_ned_detected = True
            primary_false_ned_coords = selected["coords"]
            primary_false_ned_timestamp = selected["timestamp"]
            primary_false_ned_text = selected["text"]

        # Compatibility fields used by the existing HTML/API.
        optical_zero_detected = primary_false_ned_detected
        optical_zero_timestamp = primary_false_ned_timestamp
        optical_zero_text = primary_false_ned_text

        repeated_ned_initializations = []

        if primary_false_ned_timestamp is not None:
            for item in ned_initializations:
                if (
                    item["timestamp"] is not None
                    and item["timestamp"] > primary_false_ned_timestamp + 0.25
                ):
                    repeated_ned_initializations.append(item)

        elif first_loiter_timestamp is not None:
            for item in ned_initializations:
                if (
                    item["timestamp"] is not None
                    and item["timestamp"] >= first_loiter_timestamp
                ):
                    repeated_ned_initializations.append(item)

        # Final radio bad period
        if radio_bad_start is not None:
            max_radio_bad_duration = max(
                max_radio_bad_duration,
                max(
                    0.0,
                    current_timestamp - radio_bad_start,
                ),
            )

        # Summary
        if min_voltage == 999.0:
            min_voltage = 0.0

        if start_voltage is None:
            start_voltage = 0.0

        if arm_voltage is None:
            arm_voltage = start_voltage

        if vnav_quality_min_loiter == 999:
            vnav_quality_min_loiter = 0

        final_max_altitude = max(
            0.0,
            min(
                max_alt,
                MAX_ALTITUDE,
            ),
        )

        if (
            arm_timestamp is not None
            and disarm_timestamp is not None
            and disarm_timestamp >= arm_timestamp
        ):
            duration_sec = max(
                0,
                int(disarm_timestamp - arm_timestamp),
            )
        elif arm_timestamp is not None and current_timestamp:
            duration_sec = max(
                0,
                int(current_timestamp - arm_timestamp),
            )
        elif first_timestamp is not None and current_timestamp:
            duration_sec = max(
                0,
                int(current_timestamp - first_timestamp),
            )
        else:
            duration_sec = 0

        mins, secs = divmod(duration_sec, 60)

        # Heading-based conditional antenna sector.
        heading_sector = analyze_heading_sector(raw_timeline, arm_timestamp)

        # Timeline
        # 00:00.000 = момент ARM.
        # Події до ARM показуються з мінусом, наприклад -00:32.983.
        timeline = []
        base_t = arm_timestamp or first_timestamp or 0

        for ev in sorted(raw_timeline, key=lambda x: x["timestamp"]):
            elapsed = ev["timestamp"] - base_t

            sign = ""
            if elapsed < 0:
                sign = "-"
                elapsed = abs(elapsed)

            t_minutes = int(elapsed // 60)
            t_seconds = elapsed - t_minutes * 60

            timeline.append(
                {
                    "time": f"{sign}{t_minutes:02d}:{t_seconds:06.3f}",
                    "mode": ev["mode"],
                    "alt": ev["alt"],
                    "dist": ev["dist"],
                    "azimuth": ev.get("azimuth"),
                    "headingSector": ev.get("headingSector"),
                    "vtxBand": ev["vtxBand"],
                    "vtxChannel": ev["vtxChannel"],
                    "videoFreq": ev["videoFreq"],
                    "volt": ev["volt"],
                    "curr": ev["curr"],
                    "rssi": ev["rssi"],
                    "dbm": ev["dbm"],
                    "temp": ev["temp"],
                    "esc": ev["esc"],
                    "vibration": ev.get("vibration"),
                    "systemText": ev["system_text"],
                    "pilotText": ev["pilot_text"],
                    "eventType": ev["eventType"],
                    "isError": ev["isError"],
                }
            )

        # Display
        rssi_percent = (
            round((min_rssi / 254.0) * 100)
            if min_rssi != 255
            else 0
        )

        modes_str = (
            ", ".join(sorted(flight_modes))
            if flight_modes
            else "Невідомо"
        )

        display_temp = (
            f"{round(max_temp, 1)} °C"
            if max_temp != -99.0
            else "Немає даних"
        )

        # ====================================================
        # AI / FLIGHT ANALYSIS
        # ====================================================

        ai_alerts = []
        is_critical = False

        # Завершення польоту / LAND -> automatic DISARM
        if disarm_detected:
            if disarm_mode == "LAND":
                ai_alerts.append(
                    "✅ <b>Посадка завершена:</b> перед DISARM був активний "
                    "режим LAND. Після завершення посадки автопілот "
                    "автоматично виконав DISARM."
                )
            else:
                ai_alerts.append(
                    "ℹ️ <b>DISARM:</b> зафіксовано вимкнення двигунів "
                    f"у режимі {disarm_mode or current_mode}. "
                    "Автоматичну посадку LAND за цим DISARM не підтверджено."
                )

        # Primary false coordinates around first LOITER.
        if first_loiter_timestamp is not None:
            if primary_false_ned_detected:
                ned_text = format_ned(primary_false_ned_coords)

                ai_alerts.append(
                    "✅ <b>Первинні хибні координати відбито:</b> "
                    "при першому переході в LOITER у TLOG зафіксовано "
                    f"initial pos NED = {ned_text} м. "
                    "Початкову точку External/Optical Nav встановлено."
                )
            else:
                ai_alerts.append(
                    "⚠️ <b>Первинні хибні координати не зафіксовано:</b> "
                    "у TLOG не знайдено малого initial pos NED "
                    "приблизно в межах -0.9…+0.9 м по N/E/D "
                    "для первинної роботи LOITER."
                )

        elif ever_armed:
            ai_alerts.append(
                "ℹ️ <b>LOITER не зафіксовано:</b> у цьому TLOG немає "
                "переходу в LOITER, тому момент відбиття первинних "
                "хибних координат за цією логікою не підтверджено."
            )

        # Later NED re-initializations are not treated as the first false coordinates.
        if repeated_ned_initializations:
            large_reinits = [
                item
                for item in repeated_ned_initializations
                if not item["isSmallPrimaryCandidate"]
            ]

            if large_reinits:
                examples = ", ".join(
                    format_ned(item["coords"])
                    for item in large_reinits[:3]
                )

                ai_alerts.append(
                    "🔄 <b>Повторна ініціалізація NED:</b> "
                    f"після первинного відбиття зафіксовано "
                    f"{len(large_reinits)} великих/ненульових "
                    f"initial pos NED. Приклад: {examples} м."
                )

        # EKF / External navigation
        if (
            ekf_variance_count
            or ekf_stopped_aiding_count
            or loiter_position_fail_count
            or smart_rtl_bad_position_count
        ):
            parts = []

            if ekf_variance_count:
                parts.append(f"EKF variance: {ekf_variance_count}")

            if ekf_stopped_aiding_count:
                parts.append(f"stopped aiding: {ekf_stopped_aiding_count}")

            if loiter_position_fail_count:
                parts.append(
                    f"LOITER requires position: {loiter_position_fail_count}"
                )

            if smart_rtl_bad_position_count:
                parts.append(
                    f"SmartRTL bad position: {smart_rtl_bad_position_count}"
                )

            ai_alerts.append(
                "⚠️ <b>Нестабільність позиціонування / EKF:</b> "
                + "; ".join(parts)
                + "."
            )

        if external_nav_recovery_count:
            ai_alerts.append(
                f"🔄 <b>External Nav:</b> автопілот повторно переходив "
                f"на зовнішню навігацію {external_nav_recovery_count} раз(и)."
            )

        if prearm_position_count:
            ai_alerts.append(
                f"ℹ️ <b>PreArm Position:</b> до запуску зафіксовано "
                f"{prearm_position_count} повідомлень про відсутність "
                "Position Estimate."
            )

        # Rangefinder
        if rangefinder_failed_flag:
            ai_alerts.append(
                "📡 <b>Rangefinder недоступний:</b> "
                "VISP повідомляв про відсутність даних далекоміра. "
                "Висота продовжувала визначатися іншими джерелами."
            )

        # Radio
        if radio_status_seen:
            if max_radio_bad_duration >= RADIO_DROPOUT_CRITICAL_SEC:
                ai_alerts.append(
                    "📡 <b>Аномалія RADIO_STATUS:</b> "
                    f"граничні/нульові значення тривали до "
                    f"{round(max_radio_bad_duration, 2)} с. "
                    "Фактична втрата керування цим параметром не підтверджена."
                )
            elif radio_bad_samples > 0:
                ai_alerts.append(
                    "📶 <b>RADIO_STATUS:</b> були короткі граничні "
                    "значення; одиничний -128 не трактується як втрата борта."
                )

        # Video
        if curr_video_freq is not None:
            ai_alerts.append(
                "📺 <b>Відеоканал:</b> "
                f"{curr_vtx_band} GHz / {curr_vtx_channel} / "
                f"{curr_video_freq} MHz. "
                f"Зафіксовано змін VTX: {video_change_count}."
            )
        else:
            ai_alerts.append(
                "ℹ️ <b>Відеоканал:</b> частоту не вдалося визначити "
                "за CH7 + CH8."
            )

        # Battery
        if 0 < min_voltage <= 16.8:
            is_critical = True
            ai_alerts.append(
                "🪫 <b>Критична напруга:</b> "
                f"мінімум {round(min_voltage, 2)} V."
            )
        elif 16.8 < min_voltage < 18.0:
            ai_alerts.append(
                "🔋 <b>Глибока просадка напруги:</b> "
                f"мінімум {round(min_voltage, 2)} V."
            )
        elif min_voltage > 0:
            ai_alerts.append(
                "🔋 <b>Живлення:</b> "
                f"ARM {round(arm_voltage, 2)} V, "
                f"мінімум {round(min_voltage, 2)} V, "
                f"просадка {round(max(0, arm_voltage - min_voltage), 2)} V."
            )

        # Current
        if max_current > 80.0:
            ai_alerts.append(
                "⚡ <b>Високий струм:</b> "
                f"пікове споживання {round(max_current, 1)} A (>80 A)."
            )
        else:
            ai_alerts.append(
                "⚡ <b>Струм:</b> "
                f"максимальне споживання {round(max_current, 1)} A."
            )

        # FC temperature
        if max_temp != -99.0:
            if max_temp >= 85.0:
                ai_alerts.append(
                    "🌡 <b>Критична температура FC:</b> "
                    f"{round(max_temp, 1)} °C."
                )
                is_critical = True
            elif max_temp >= 70.0:
                ai_alerts.append(
                    "🌡 <b>Висока температура FC:</b> "
                    f"{round(max_temp, 1)} °C."
                )
            else:
                ai_alerts.append(
                    "🌡 <b>Температура FC:</b> "
                    f"максимум {round(max_temp, 1)} °C — в межах норми."
                )

        # ESC temperatures
        esc_temps_available = [
            t for t in esc_temp_max
            if t is not None
        ]

        if esc_temps_available:
            hottest_index = max(
                range(4),
                key=lambda i: esc_temp_max[i] if esc_temp_max[i] is not None else -1,
            )
            hottest_temp = esc_temp_max[hottest_index]

            for i, esc_t in enumerate(esc_temp_max):
                if esc_t is not None and esc_t >= 85.0:
                    ai_alerts.append(
                        f"🌡 <b>Критична температура ESC{i + 1}:</b> "
                        f"{round(esc_t, 1)} °C."
                    )
                    is_critical = True

            if hottest_temp < 85.0:
                ai_alerts.append(
                    f"🧊 <b>ESC:</b> найвища температура ESC"
                    f"{hottest_index + 1} — {round(hottest_temp, 1)} °C. "
                    "Критичного перегріву не зафіксовано."
                )

            if len(esc_temps_available) >= 2:
                spread = max(esc_temps_available) - min(esc_temps_available)

                if spread >= 20.0:
                    ai_alerts.append(
                        "⚠️ <b>Нерівномірний нагрів ESC:</b> "
                        f"різниця між ESC становила {round(spread, 1)} °C."
                    )

        # Conditional antenna sector by aircraft heading
        if heading_sector.get("available"):
            ref = heading_sector["reference"]
            smin = heading_sector["sectorMin"]
            smax = heading_sector["sectorMax"]
            ref_dur = heading_sector["referenceDuration"]
            episodes = heading_sector.get("episodes", [])

            if episodes:
                first = episodes[0]
                first_time = format_timeline_time(first["start"], base_t)
                max_ep = max(episodes, key=lambda x: x["maxDeviation"])
                return_text = (
                    "Heading надалі повертався в умовний сектор."
                    if heading_sector.get("returnedToSector")
                    else "До кінця зафіксованого епізоду повернення в сектор не підтверджено."
                )
                ai_alerts.append(
                    "📡 <b>Ймовірний вихід за умовний сектор АС по Heading:</b> "
                    f"стабільний базовий Heading {ref:.1f}° утримувався ~{ref_dur:.0f} с; "
                    f"при розкритті {ANTENNA_BEAM_WIDTH_DEG:.0f}° умовний сектор "
                    f"{smin:.1f}°–{smax:.1f}°. "
                    f"Перший підтверджений вихід: {first_time}, Heading {first['firstHeading']:.1f}°, "
                    f"тривалість {first['duration']:.1f} с. "
                    f"Максимальне відхилення {max_ep['maxDeviation']:.1f}° "
                    f"(Heading {max_ep['maxHeading']:.1f}°). {return_text} "
                    "Це оцінка за курсом БПЛА, а не пряме вимірювання фізичного азимута антенної станції."
                )
            else:
                ai_alerts.append(
                    "✅ <b>Умовний сектор АС по Heading:</b> "
                    f"базовий Heading {ref:.1f}° (стабільно ~{ref_dur:.0f} с), "
                    f"сектор {smin:.1f}°–{smax:.1f}°. "
                    f"Підтверджених виходів довше {HEADING_OUTSIDE_CONFIRM_SEC:.0f} с не зафіксовано."
                )
        elif ever_armed:
            ai_alerts.append(
                "ℹ️ <b>Умовний сектор АС по Heading не визначено:</b> "
                f"не знайдено стабільного Heading тривалістю щонайменше "
                f"{HEADING_REFERENCE_MIN_DURATION_SEC:.0f} с у межах "
                f"±{HEADING_REFERENCE_STABLE_TOLERANCE_DEG:.0f}°."
            )

        # Vibration
        if vibration_critical_events:
            is_critical = True

            def _vib_time(ts):
                elapsed = ts - base_t
                sign = "-" if elapsed < 0 else ""
                elapsed = abs(elapsed)
                minutes = int(elapsed // 60)
                seconds = elapsed - minutes * 60
                return f"{sign}{minutes:02d}:{seconds:06.3f}"

            vib_examples = []
            for item in vibration_critical_events[:5]:
                vib_examples.append(
                    f"{_vib_time(item['timestamp'])} "
                    f"(X={item['x']:.1f}, Y={item['y']:.1f}, Z={item['z']:.1f})"
                )

            extra_count = len(vibration_critical_events) - len(vib_examples)
            extra_text = f"; ще епізодів: {extra_count}" if extra_count > 0 else ""

            ai_alerts.append(
                "🚨 <b>Звернути увагу на критичні вібрації:</b> "
                f"поріг {VIBRATION_CRITICAL_THRESHOLD:.0f}. "
                f"Максимум X={max_vib_x:.1f}, Y={max_vib_y:.1f}, Z={max_vib_z:.1f}. "
                "Час: " + "; ".join(vib_examples) + extra_text + "."
            )
        else:
            ai_alerts.append(
                "✅ <b>Вібрації:</b> "
                f"максимум X={max_vib_x:.1f}, Y={max_vib_y:.1f}, Z={max_vib_z:.1f}; "
                f"критичний поріг {VIBRATION_CRITICAL_THRESHOLD:.0f} не перевищено."
            )

        # Attitude
        if max_roll > 80 or max_pitch > 80:
            ai_alerts.append(
                "🔄 <b>Великий кут нахилу:</b> "
                f"{round(max(max_roll, max_pitch), 1)}°."
            )
            is_critical = True

        # End state
        log_ended_armed = ever_armed and was_armed

        if log_ended_armed:
            ai_alerts.append(
                "❗ <b>Лог закінчився при ARMED:</b> "
                "у файлі немає підтвердженого DISARM."
            )
            is_critical = True

        # Final verdict
        navigation_problem = (
            ekf_variance_count > 0
            or ekf_stopped_aiding_count > 0
            or loiter_position_fail_count > 0
            or smart_rtl_bad_position_count > 0
        )

        if disarm_detected:
            if is_critical:
                ai_verdict = (
                    "⚠️ БОРТ ЗАВЕРШИВ ПОЛІТ. "
                    "ЗАФІКСОВАНО КРИТИЧНІ ПОДІЇ:"
                )
            elif navigation_problem or max_current > 80.0 or rangefinder_failed_flag:
                ai_verdict = (
                    "🟡 БОРТ ЗАВЕРШИВ ПОЛІТ. "
                    "ПІД ЧАС ПОЛЬОТУ ЗАФІКСОВАНО ВІДХИЛЕННЯ:"
                )
            else:
                ai_verdict = (
                    "✅ БОРТ ЗАВЕРШИВ ПОЛІТ. "
                    "КРИТИЧНИХ ВІДХИЛЕНЬ НЕ ЗАФІКСОВАНО:"
                )
        elif log_ended_armed:
            ai_verdict = (
                "🚨 ЛОГ ОБІРВАВСЯ ПРИ ARMED. "
                "ПОТРІБНА ПЕРЕВІРКА:"
            )
        elif is_critical:
            ai_verdict = (
                "⚠️ ПІД ЧАС ПОЛЬОТУ ЗАФІКСОВАНО "
                "КРИТИЧНІ ПОДІЇ:"
            )
        else:
            ai_verdict = "📊 ПОВНИЙ АНАЛІЗ ПОЛЬОТУ:"

        return {
            "success": True,
            "ai": {
                "verdict": ai_verdict,
                "isCritical": is_critical,
                "landedSuccessfully": landed_successfully,
                "disarmMode": disarm_mode,
                "disarmDetected": disarm_detected,
                "opticalZeroDetected": optical_zero_detected,
                "opticalZeroText": optical_zero_text,
                "firstLoiterTimestamp": first_loiter_timestamp,
                "primaryFalseNedDetected": primary_false_ned_detected,
                "primaryFalseNed": (
                    list(primary_false_ned_coords)
                    if primary_false_ned_coords is not None
                    else None
                ),
                "primaryFalseNedText": primary_false_ned_text,
                "nedInitializationCount": len(ned_initializations),
                "repeatedNedInitializationCount": len(repeated_ned_initializations),
                "ekfVarianceCount": ekf_variance_count,
                "ekfStoppedAidingCount": ekf_stopped_aiding_count,
                "loiterPositionFailCount": loiter_position_fail_count,
                "externalNavRecoveryCount": external_nav_recovery_count,
                "smartRtlBadPositionCount": smart_rtl_bad_position_count,
                "headingSector": {
                    "available": heading_sector.get("available", False),
                    "reference": heading_sector.get("reference"),
                    "sectorMin": heading_sector.get("sectorMin"),
                    "sectorMax": heading_sector.get("sectorMax"),
                    "beamWidth": heading_sector.get("beamWidth", ANTENNA_BEAM_WIDTH_DEG),
                    "referenceDuration": heading_sector.get("referenceDuration", 0.0),
                    "episodeCount": len(heading_sector.get("episodes", [])),
                    "maxDeviation": heading_sector.get("maxDeviation", 0.0),
                    "returnedToSector": heading_sector.get("returnedToSector", False),
                },
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
                "voltageSag": round(
                    max(
                        0,
                        arm_voltage - min_voltage,
                    ),
                    2,
                ),
            },
            "radio": {
                "rssi": (
                    f"{rssi_percent}%"
                    if min_rssi != 255
                    else "Немає"
                ),
                "telemRssi": (
                    f"{round(min_dbm)} dBm"
                    if min_dbm != 0
                    else "—"
                ),
                "maxThrottle": f"{round(max_throttle)}%",
                "maxDropout": round(max_radio_bad_duration, 2),
                "hasGps": (
                    "GPS Присутній"
                    if has_gps
                    else "Без GPS / локальна навігація"
                ),
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
                "vibrationCriticalThreshold": VIBRATION_CRITICAL_THRESHOLD,
                "vibrationCriticalCount": len(vibration_critical_events),
                "clipping": clip_count,
                "maxTemp": display_temp,
                "opticalQuality": (
                    f"{round(vnav_quality_min_loiter)}%–"
                    f"{round(vnav_quality_max_loiter)}%"
                    if vnav_samples > 0
                    else "Немає даних"
                ),
                "esc": [
                    {
                        "id": i + 1,
                        "temp": round(esc_temp_current[i], 1) if esc_temp_current[i] is not None else None,
                        "maxTemp": round(esc_temp_max[i], 1) if esc_temp_max[i] is not None else None,
                        "rpm": int(esc_rpm_current[i]) if esc_rpm_current[i] is not None else None,
                        "maxRpm": int(esc_rpm_max[i]),
                        "current": round(esc_current_current[i], 1) if esc_current_current[i] is not None else None,
                        "maxCurrent": round(esc_current_max[i], 1),
                    }
                    for i in range(4)
                ],
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
                ),
                "ch7": (
                    f"{rc_min[7]}–{rc_max[7]} us"
                    if rc_max[7] > 0
                    else "—"
                ),
                "ch8": (
                    f"{rc_min[8]}–{rc_max[8]} us"
                    if rc_max[8] > 0
                    else "—"
                ),
            },
            "timeline": timeline,
        }

    finally:
        if os.path.exists(temp.name):
            try:
                os.unlink(temp.name)
            except Exception:
                pass
