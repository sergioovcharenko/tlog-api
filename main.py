
import math
import os
import tempfile
import re
import statistics

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
RADIO_LONG_LOSS_SEC = 60.0
# Робочі пороги радіолінії, задані користувачем.
RADIO_NORMAL_DBM = -85.0
RADIO_VIDEO_DEGRADED_DBM = -90.0
RADIO_VIDEO_LOST_DBM = -100.0
RADIO_LINK_LOST_DBM = -128.0
VIBRATION_CRITICAL_THRESHOLD = 36.0
ATTITUDE_CRITICAL_THRESHOLD_DEG = 35.0

# V20 — percentage display for all four primary control axes (CH1..CH4).
# Percent is measured from calibrated TRIM/center toward MIN/MAX.
CONTROL_CENTER_DEADBAND_US = 25.0
CONTROL_AXIS_REVERSED = {1: False, 2: False, 3: False, 4: False}
# Compatibility aliases used by the previous V19 throttle block.
THROTTLE_CENTER_DEADBAND_US = CONTROL_CENTER_DEADBAND_US
THROTTLE_REVERSED = CONTROL_AXIS_REVERSED[3]

# RPM / motor diagnostics for this QUAD X numbering:
# diagonal pairs: Motor 1 <-> Motor 2 and Motor 3 <-> Motor 4.
RPM_DIAGONAL_PAIRS = ((0, 1), (2, 3))
RPM_MIN_ACTIVE = 800.0
RPM_DIAGONAL_WARNING_PCT = 25.0
RPM_DIAGONAL_CRITICAL_PCT = 35.0
RPM_DIAGONAL_RECOVERY_PCT = 20.0
RPM_CRITICAL_PERSIST_SEC = 0.8
RPM_THRUST_CORRELATION_SEC = 3.0
MECHANICAL_CORRELATION_SEC = 2.5

# V14 — causal window around Potential Thrust Loss.
THRUST_CAUSAL_PRE_SEC = 4.0
THRUST_CAUSAL_POST_SEC = 4.0
THRUST_ESC_CURRENT_DROP_PCT = 35.0
THRUST_DESCENT_DELTA_MPS = 3.0
THRUST_DESCENT_MIN_MPS = 5.0
THRUST_VOLTAGE_DROP_V = 1.0

# V15 — EKF / optical-navigation pilot-action rules.
# User operational rule:
#   LOITER + EKF variance -> switch to ALT HOLD.
#   If ALT HOLD follows promptly, the EKF variance / stopped aiding sequence
#   is treated as an expected transition, not as a flight-critical EKF fault.
EKF_ALT_HOLD_CORRECT_SEC = 5.0
EKF_ALT_HOLD_LATE_SEC = 10.0

# Repeated stopped aiding in LOITER approximately every second indicates
# optical-navigation aiding is not working reliably.
EKF_STOPPED_AIDING_REPEAT_MIN_COUNT = 3
EKF_STOPPED_AIDING_REPEAT_MIN_GAP_SEC = 0.5
EKF_STOPPED_AIDING_REPEAT_MAX_GAP_SEC = 1.5

# "stopped aiding" shortly after LOITER -> ALT HOLD is considered normal
# when preceded by EKF variance in LOITER.
EKF_STOPPED_AIDING_NORMAL_AFTER_SWITCH_SEC = 5.0
EKF_VARIANCE_LOOKBACK_BEFORE_SWITCH_SEC = 10.0

# LAND diagnostics.
# ArduPilot LAND_SPEED / LAND_SPEED_HIGH are stored in cm/s,
# LAND_ALT_LOW in cm.
LAND_SPEED_TOLERANCE_MPS = 1.0
LAND_SPEED_RATIO_WARNING = 1.35
LAND_SPEED_MIN_ABNORMAL_MPS = 1.5
LAND_FREEFALL_ACCEL_MPS2 = 2.5
LAND_ANALYSIS_MIN_SAMPLES = 5

# Ground/calibration context.
GROUND_CAL_ATT_IGNORE_DURING_ACCEL_CAL = True
GROUND_RADIO_LOSS_NEVER_MEANS_AIRCRAFT_LOSS = True
FLIGHT_TAKEOFF_ALT_M = 2.0
FLIGHT_LANDED_ALT_M = 0.8
FLIGHT_MIN_AIRBORNE_SEC = 1.5
FLIGHT_MIN_GROUND_GAP_SEC = 1.0

# Antenna-station inference from the TLOG.
# IMPORTANT: this is an estimate, not a direct measurement of antenna azimuth.
# It assumes LOCAL_POSITION_NED origin is at/near the antenna station.
ANTENNA_BEAM_WIDTH_DEG = 30.0
ANTENNA_HALF_ANGLE_DEG = ANTENNA_BEAM_WIDTH_DEG / 2.0
ANTENNA_MIN_DISTANCE_M = 10.0
ANTENNA_MIN_RADIO_SAMPLES = 15
ANTENNA_TOP_SIGNAL_FRACTION = 0.30
ANTENNA_SAMPLE_MAX_GAP_SEC = 2.2

# V16 — safety guard for V12 360° axis refinement.
# The radio optimizer may REFINE the initial axis, but must not flip
# the antenna to a weak/ambiguous opposite candidate.
ANTENNA_AXIS_MIN_SCORE_GAP = 1.0
ANTENNA_AXIS_MAX_REFINEMENT_SHIFT_DEG = 90.0
ANTENNA_AXIS_ALLOWED_STABILITIES = {"MEDIUM", "HIGH"}

# 5-ознакова модель ймовірного виходу БПЛА за сектор АС.
# Це НЕ окремий "датчик", а сукупність незалежних ознак.
ANTENNA_TREND_LOOKBACK_SEC = 15.0
ANTENNA_TREND_MIN_SAMPLES = 5
ANTENNA_DEVIATION_GROWTH_MIN_DEG = 3.0
ANTENNA_DBM_WORSEN_MIN_DB = 5.0
ANTENNA_DBM_WORSEN_MIN_SLOPE_DB_PER_SEC = 0.20
ANTENNA_DBM_WORSEN_MIN_FRACTION = 0.60

CH7_REVERSED = False
CH8_REVERSED = False

# V18 — TX16S switch diagnostics.
# Mapping inferred from the supplied TLOGs / current setup.
# Change only these four channel numbers if the EdgeTX mixer is remapped.
TX16_SWITCH_CHANNELS = {
    "SC": 7,   # 3-pos: away=safety, middle=right mechanism, toward=left mechanism
    "SD": 8,   # 3-pos: only toward removes Emergency STOP safety
    "SF": 10,  # 2-pos activator, active at high PWM
    "SH": 6,   # momentary Emergency STOP command, active at high PWM
}
TX16_ACTIVE_PWM_MIN = 1700
TX16_INACTIVE_PWM_MAX = 1300

# Payload-actuator diagnostics. This does NOT prove mechanical movement;
# it confirms that the autopilot changed the configured PWM output.
PAYLOAD_CONFIRM_SERVO = 7
PAYLOAD_SERVO_DELTA_US = 250
PAYLOAD_SERVO_CORRELATION_SEC = 3.0

# Emergency STOP diagnostic rule supplied by the user.
EMERGENCY_STOP_MIN_HOLD_SEC = 8.0
EMERGENCY_STOP_EXPECTED_HOLD_SEC = 10.0
EMERGENCY_DISARM_CORRELATION_SEC = 3.0

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


def radio_state_from_dbm(dbm):
    """Класифікація радіолінії за робочими порогами аналізатора."""
    if not valid_number(dbm):
        return "UNKNOWN"
    v = float(dbm)
    if v <= RADIO_LINK_LOST_DBM:
        return "LINK_LOST"
    if v <= RADIO_VIDEO_LOST_DBM:
        # Діапазон -101…-127 користувач окремо не задавав.
        # Вважаємо його дуже слабкою телеметрією, але не повною втратою до -128.
        return "VERY_WEAK_TELEMETRY"
    if v <= RADIO_VIDEO_DEGRADED_DBM:
        return "VIDEO_LOST_TELEMETRY_OK"
    if v < RADIO_NORMAL_DBM:
        return "VIDEO_DEGRADED"
    return "NORMAL"


def radio_state_text(state):
    return {
        "NORMAL": "Норма: відео та телеметрія стабільні",
        "VIDEO_DEGRADED": "Підсипання / деградація відео",
        "VIDEO_LOST_TELEMETRY_OK": "Втрата відео, телеметрія присутня",
        "VERY_WEAK_TELEMETRY": "Дуже слабка телеметрія",
        "LINK_LOST": "Відсутні відео та телеметрія",
        "UNKNOWN": "Немає даних",
    }.get(state, "Немає даних")


def circular_weighted_mean(samples):
    """samples = [(angle_deg, weight), ...]"""
    if not samples:
        return None, 0.0
    sin_sum = sum(w * math.sin(math.radians(a)) for a, w in samples)
    cos_sum = sum(w * math.cos(math.radians(a)) for a, w in samples)
    wsum = sum(w for _, w in samples) or 1.0
    angle = math.degrees(math.atan2(sin_sum, cos_sum)) % 360.0
    resultant = math.sqrt(sin_sum*sin_sum + cos_sum*cos_sum) / wsum
    return angle, max(0.0, min(1.0, resultant))




def estimate_map_antenna_direction(raw_timeline, flight_number):
    """
    V12 — гібридна геометрія + 360° радіооцінка осі АС для 2D-карти.

    Джерела позиції за пріоритетом:
      1. POSITION_NED
      2. GS_DR_POSITION = groundspeed + heading + dt
      3. ATTITUDE_DR_POSITION = roll/pitch/yaw + приблизна модель швидкості + dt
      4. HEADING_FALLBACK — тільки діагностика, НЕ геометрія.

    dBm використовується як радіодоказ для оцінки осі/меж сектора,
    але НЕ використовується для "домальовування" руху.
    """

    result = {
        "available": False,
        "drawable": False,
        "method": None,
        "center": None,
        "sectorMin": None,
        "sectorMax": None,
        "beamWidth": None,
        "halfAngle": None,
        "confidence": 0,
        "sampleCount": 0,
        "minDistanceUsed": None,
        "signalContrastDb": None,
        "angularCoverageDeg": None,
        "beamWidthDynamic": False,
        "beamWidthReason": None,
        "beamStrength": None,
        "beamDropThresholdDb": None,
        "beamHalfAngleEstimated": None,
        "angularSignalProfile": [],
        "angularTrendFraction": None,

        "sourceCounts": {
            "ned": 0,
            "gsDr": 0,
            "attitudeDr": 0,
            "heading": 0,
        },

        "positionSourceQuality": None,

        # V12 axis-search diagnostics.
        "axisInitial": None,
        "axisOptimized": None,
        "axisShiftDeg": None,
        "axisScore": None,
        "axisSecond": None,
        "axisSecondScore": None,
        "axisScoreGap": None,
        "axisStability": None,
        "axisUncertaintyDeg": None,
        "axisOptimizationUsed": False,
        "axisOptimizationRejected": False,
        "axisOptimizationRejectReason": None,
        "axisCandidateShiftDeg": None,
        "axisInsideMedianDbm": None,
        "axisOutsideMedianDbm": None,
        "axisInsideCount": 0,
        "axisOutsideCount": 0,
    }

    rows = [
        ev
        for ev in sorted(
            raw_timeline,
            key=lambda x: x.get("timestamp", 0),
        )
        if ev.get("eventType") == "SNAPSHOT"
        and ev.get("flightNumber") == flight_number
    ]

    if len(rows) < 2:
        return result

    # Approximate speed models requested for this aircraft.
    # Used ONLY when real groundspeed is unavailable.
    LOITER_KMH_PER_DEG = 2.75
    LOITER_MAX_KMH = 55.0

    ALTHOLD_KMH_PER_DEG = 110.0 / 30.0
    ALTHOLD_MAX_KMH = 110.0

    def mode_key(mode):
        return (
            str(mode or "")
            .upper()
            .replace(" ", "")
            .replace("_", "")
            .replace("-", "")
        )

    def attitude_velocity(ev):
        """
        Estimate world-frame velocity from Roll/Pitch/Yaw.

        pitch < 0 -> approximately forward
        roll  > 0 -> approximately right

        This is only a fallback and has a strict confidence cap.
        """
        att = ev.get("attitude")

        if not isinstance(att, dict):
            return None

        roll = att.get("roll")
        pitch = att.get("pitch")
        yaw = att.get("yaw")

        if not (
            valid_number(roll)
            and valid_number(pitch)
            and valid_number(yaw)
        ):
            return None

        mode = mode_key(ev.get("mode"))

        if "LOITER" in mode:
            k = LOITER_KMH_PER_DEG
            max_kmh = LOITER_MAX_KMH
        elif "ALTHOLD" in mode:
            k = ALTHOLD_KMH_PER_DEG
            max_kmh = ALTHOLD_MAX_KMH
        else:
            return None

        forward_kmh = -float(pitch) * k
        right_kmh = float(roll) * k

        magnitude = math.hypot(
            forward_kmh,
            right_kmh,
        )

        if magnitude > max_kmh and magnitude > 0.0:
            scale = max_kmh / magnitude
            forward_kmh *= scale
            right_kmh *= scale

        yaw_rad = math.radians(float(yaw) % 360.0)

        north_kmh = (
            forward_kmh * math.cos(yaw_rad)
            - right_kmh * math.sin(yaw_rad)
        )

        east_kmh = (
            forward_kmh * math.sin(yaw_rad)
            + right_kmh * math.cos(yaw_rad)
        )

        return (
            north_kmh / 3.6,
            east_kmh / 3.6,
        )

    # --------------------------------------------------------
    # Build parallel dead-reckoning positions.
    # GS_DR and ATTITUDE_DR are kept separate so we know
    # exactly which fallback supported a point.
    # --------------------------------------------------------
    gs_n = 0.0
    gs_e = 0.0

    att_n = 0.0
    att_e = 0.0

    prev_t = None

    prev_gs_vn = None
    prev_gs_ve = None

    prev_att_vn = None
    prev_att_ve = None

    samples = []

    for ev in rows:
        ts = ev.get("timestamp")

        if not valid_number(ts):
            continue

        ts = float(ts)

        # -------- real groundspeed + heading --------
        heading = ev.get("azimuth")
        groundspeed = ev.get("groundspeed")

        gs_vn = None
        gs_ve = None

        if (
            valid_number(heading)
            and valid_number(groundspeed)
        ):
            h = math.radians(
                float(heading) % 360.0
            )

            gs = max(
                0.0,
                float(groundspeed),
            )

            gs_vn = gs * math.cos(h)
            gs_ve = gs * math.sin(h)

        # -------- attitude fallback velocity --------
        att_vel = attitude_velocity(ev)

        if att_vel is not None:
            att_vn, att_ve = att_vel
        else:
            att_vn = None
            att_ve = None

        if prev_t is not None:
            dt = ts - prev_t

            if 0.0 < dt <= 3.0:

                # Trapezoidal GS integration.
                if gs_vn is not None and gs_ve is not None:
                    if (
                        prev_gs_vn is not None
                        and prev_gs_ve is not None
                    ):
                        gs_n += (
                            0.5
                            * (prev_gs_vn + gs_vn)
                            * dt
                        )

                        gs_e += (
                            0.5
                            * (prev_gs_ve + gs_ve)
                            * dt
                        )
                    else:
                        gs_n += gs_vn * dt
                        gs_e += gs_ve * dt

                # Trapezoidal attitude integration.
                if att_vn is not None and att_ve is not None:
                    if (
                        prev_att_vn is not None
                        and prev_att_ve is not None
                    ):
                        att_n += (
                            0.5
                            * (prev_att_vn + att_vn)
                            * dt
                        )

                        att_e += (
                            0.5
                            * (prev_att_ve + att_ve)
                            * dt
                        )
                    else:
                        att_n += att_vn * dt
                        att_e += att_ve * dt

        if gs_vn is not None and gs_ve is not None:
            prev_gs_vn = gs_vn
            prev_gs_ve = gs_ve

        if att_vn is not None and att_ve is not None:
            prev_att_vn = att_vn
            prev_att_ve = att_ve

        prev_t = ts

        # -------- choose position source per snapshot --------
        ned_n = ev.get("nedNorth")
        ned_e = ev.get("nedEast")

        has_ned = (
            valid_number(ned_n)
            and valid_number(ned_e)
        )

        has_gs = (
            gs_vn is not None
            and gs_ve is not None
        )

        has_att = (
            att_vn is not None
            and att_ve is not None
        )

        if has_ned:
            pos_n = float(ned_n)
            pos_e = float(ned_e)
            source = "NED"

        elif has_gs:
            pos_n = gs_n
            pos_e = gs_e
            source = "GS_DR"

        elif has_att:
            pos_n = att_n
            pos_e = att_e
            source = "ATTITUDE_DR"

        else:
            pos_n = None
            pos_e = None
            source = None

        if pos_n is not None and pos_e is not None:
            dist = math.hypot(
                pos_n,
                pos_e,
            )

            pos_az = (
                (
                    math.degrees(
                        math.atan2(
                            pos_e,
                            pos_n,
                        )
                    )
                    + 360.0
                )
                % 360.0
                if dist >= 0.5
                else None
            )
        else:
            dist = None
            pos_az = None

        dbm = ev.get("dbm")
        dbm = (
            float(dbm)
            if valid_number(dbm)
            else None
        )

        samples.append({
            "timestamp": ts,
            "source": source,
            "distance": dist,
            "positionAzimuth": pos_az,
            "heading": (
                float(heading) % 360.0
                if valid_number(heading)
                else None
            ),
            "dbm": dbm,
        })

    def geometry_candidates(
        source_name,
        min_distance,
    ):
        out = []

        for x in samples:
            if x["source"] != source_name:
                continue

            az = x["positionAzimuth"]
            dist = x["distance"]
            dbm = x["dbm"]

            if (
                az is None
                or dist is None
                or dbm is None
            ):
                continue

            if dist < min_distance:
                continue

            # -128 is a loss marker, not a good sample for axis scoring.
            if (
                dbm <= RADIO_LINK_LOST_DBM
                or dbm >= 0
            ):
                continue

            # Approximate distance compensation.
            corrected = (
                dbm
                + 20.0
                * math.log10(
                    max(dist, 1.0)
                )
            )

            out.append({
                "score": corrected,
                "azimuth": az,
                "dbm": dbm,
                "distance": dist,
                "timestamp": x["timestamp"],
            })

        return out

    NED_MIN_DISTANCE_M = ANTENNA_MIN_DISTANCE_M
    GS_DR_MIN_DISTANCE_M = 200.0
    ATT_DR_MIN_DISTANCE_M = 200.0

    ned_candidates = geometry_candidates(
        "NED",
        NED_MIN_DISTANCE_M,
    )

    gs_candidates = geometry_candidates(
        "GS_DR",
        GS_DR_MIN_DISTANCE_M,
    )

    att_candidates = geometry_candidates(
        "ATTITUDE_DR",
        ATT_DR_MIN_DISTANCE_M,
    )

    result["sourceCounts"]["ned"] = len(
        ned_candidates
    )

    result["sourceCounts"]["gsDr"] = len(
        gs_candidates
    )

    result["sourceCounts"]["attitudeDr"] = len(
        att_candidates
    )

    candidates = None
    method = None
    confidence_cap = 0.0
    min_distance_used = None
    position_quality = None

    if len(ned_candidates) >= ANTENNA_MIN_RADIO_SAMPLES:
        candidates = ned_candidates
        method = "POSITION_NED"
        confidence_cap = 1.00
        min_distance_used = NED_MIN_DISTANCE_M
        position_quality = "HIGH"

    elif len(gs_candidates) >= ANTENNA_MIN_RADIO_SAMPLES:
        candidates = gs_candidates
        method = "GS_DR_POSITION"
        confidence_cap = 0.70
        min_distance_used = GS_DR_MIN_DISTANCE_M
        position_quality = "MEDIUM"

    elif len(att_candidates) >= ANTENNA_MIN_RADIO_SAMPLES:
        candidates = att_candidates
        method = "ATTITUDE_DR_POSITION"
        confidence_cap = 0.45
        min_distance_used = ATT_DR_MIN_DISTANCE_M
        position_quality = "LOW"

    if candidates:
        candidates = sorted(
            candidates,
            key=lambda x: x["score"],
            reverse=True,
        )

        top_n = max(
            ANTENNA_MIN_RADIO_SAMPLES,
            int(
                math.ceil(
                    len(candidates)
                    * ANTENNA_TOP_SIGNAL_FRACTION
                )
            ),
        )

        top = candidates[
            :min(
                top_n,
                len(candidates),
            )
        ]

        min_score = min(
            x["score"]
            for x in top
        )

        weighted = [
            (
                x["azimuth"],
                max(
                    1.0,
                    (x["score"] - min_score)
                    + 1.0,
                ),
            )
            for x in top
        ]

        reference, concentration = (
            circular_weighted_mean(
                weighted
            )
        )

        # ====================================================
        # V12 — 360° SEARCH FOR THE MOST PLAUSIBLE ANTENNA AXIS
        # ====================================================
        # Physical antenna sector is fixed: 30° total = ±15°.
        # dBm does NOT move coordinates and does NOT change beam width.
        # It only scores which possible antenna axis best explains
        # the reconstructed NED / GS_DR / ATTITUDE_DR geometry.
        axis_initial = reference

        def _median(vals):
            vals = [float(v) for v in vals if valid_number(v)]
            return float(statistics.median(vals)) if vals else None

        def _signed_angle_delta_deg(angle, center):
            return (
                (float(angle) - float(center) + 180.0)
                % 360.0
                - 180.0
            )

        def _axis_score(axis_deg):
            inside = []
            outside_near = []
            outside_all = []
            inside_raw = []
            outside_raw = []

            left_inside = 0
            right_inside = 0

            trend_bins = [
                (0.0, 5.0),
                (5.0, 10.0),
                (10.0, 15.0),
                (15.0, 25.0),
                (25.0, 40.0),
                (40.0, 60.0),
                (60.0, 90.0),
            ]
            binned = [[] for _ in trend_bins]

            for x in candidates:
                az = x.get("azimuth")
                corrected = x.get("score")
                raw_dbm = x.get("dbm")

                if (
                    not valid_number(az)
                    or not valid_number(corrected)
                ):
                    continue

                signed = _signed_angle_delta_deg(
                    az,
                    axis_deg,
                )
                dev = abs(signed)
                corrected = float(corrected)

                if dev <= ANTENNA_HALF_ANGLE_DEG:
                    inside.append(corrected)

                    if valid_number(raw_dbm):
                        inside_raw.append(float(raw_dbm))

                    if signed < 0:
                        left_inside += 1
                    else:
                        right_inside += 1

                else:
                    outside_all.append(corrected)

                    if valid_number(raw_dbm):
                        outside_raw.append(float(raw_dbm))

                    if dev <= 60.0:
                        outside_near.append(corrected)

                for bi, (lo, hi) in enumerate(trend_bins):
                    in_bin = (
                        lo <= dev < hi
                        if bi < len(trend_bins) - 1
                        else lo <= dev <= hi
                    )

                    if in_bin:
                        binned[bi].append(corrected)
                        break

            min_group = max(
                8,
                int(round(len(candidates) * 0.02)),
            )

            outside_cmp = (
                outside_near
                if len(outside_near) >= min_group
                else outside_all
            )

            if (
                len(inside) < min_group
                or len(outside_cmp) < min_group
            ):
                return None

            inside_med = _median(inside)
            outside_med = _median(outside_cmp)

            if (
                inside_med is None
                or outside_med is None
            ):
                return None

            # Positive means corrected signal is stronger inside ±15°.
            contrast_db = inside_med - outside_med

            inside_weak_fraction = (
                sum(
                    1
                    for v in inside_raw
                    if v <= RADIO_VIDEO_LOST_DBM
                )
                / len(inside_raw)
                if inside_raw
                else 0.0
            )

            outside_strong_fraction = (
                sum(
                    1
                    for v in outside_raw
                    if v >= RADIO_NORMAL_DBM
                )
                / len(outside_raw)
                if outside_raw
                else 0.0
            )

            med_bins = []

            for (lo, hi), vals in zip(
                trend_bins,
                binned,
            ):
                if len(vals) >= 5:
                    med_bins.append(
                        (
                            lo,
                            hi,
                            _median(vals),
                            len(vals),
                        )
                    )

            trend_checks = 0
            trend_good = 0

            for i in range(1, len(med_bins)):
                prev_med = med_bins[i - 1][2]
                curr_med = med_bins[i][2]

                if prev_med is None or curr_med is None:
                    continue

                trend_checks += 1

                # Allow +1.5 dB RF noise / multipath.
                if curr_med <= prev_med + 1.5:
                    trend_good += 1

            trend_fraction_local = (
                trend_good / trend_checks
                if trend_checks
                else 0.0
            )

            side_total = left_inside + right_inside

            side_balance = (
                2.0
                * min(left_inside, right_inside)
                / side_total
                if side_total > 0
                else 0.0
            )

            sample_balance = min(
                1.0,
                min(
                    len(inside),
                    len(outside_cmp),
                )
                / max(
                    20.0,
                    len(candidates) * 0.08,
                ),
            )

            score = (
                2.20 * contrast_db
                + 3.00 * trend_fraction_local
                + 1.25 * side_balance
                + 1.00 * sample_balance
                - 7.00 * inside_weak_fraction
                - 4.00 * outside_strong_fraction
            )

            return {
                "axis": float(axis_deg) % 360.0,
                "score": float(score),
                "contrastDb": float(contrast_db),
                "insideMedian": float(inside_med),
                "outsideMedian": float(outside_med),
                "insideCount": len(inside),
                "outsideCount": len(outside_cmp),
                "insideWeakFraction": float(
                    inside_weak_fraction
                ),
                "outsideStrongFraction": float(
                    outside_strong_fraction
                ),
                "trendFraction": float(
                    trend_fraction_local
                ),
                "sideBalance": float(side_balance),
            }

        axis_trials = []

        for axis_deg in range(360):
            trial = _axis_score(axis_deg)

            if trial is not None:
                axis_trials.append(trial)

        axis_best = None
        axis_second = None
        axis_gap = None
        axis_uncertainty = None
        axis_stability = "LOW"
        axis_optimization_used = False

        if axis_trials:
            axis_trials.sort(
                key=lambda z: z["score"],
                reverse=True,
            )

            axis_best = axis_trials[0]

            # Second distinct candidate: >=10° from the winner.
            for trial in axis_trials[1:]:
                if (
                    heading_difference_deg(
                        trial["axis"],
                        axis_best["axis"],
                    )
                    >= 10.0
                ):
                    axis_second = trial
                    break

            if axis_second is not None:
                axis_gap = (
                    axis_best["score"]
                    - axis_second["score"]
                )

            tolerance = max(
                0.8,
                abs(axis_best["score"]) * 0.08,
            )

            near_best_offsets = []

            for trial in axis_trials:
                if (
                    axis_best["score"]
                    - trial["score"]
                    <= tolerance
                ):
                    d = heading_difference_deg(
                        trial["axis"],
                        axis_best["axis"],
                    )

                    if d is not None and d <= 35.0:
                        near_best_offsets.append(d)

            axis_uncertainty = (
                max(near_best_offsets)
                if near_best_offsets
                else 0.0
            )

            sufficient_radio_separation = (
                axis_best["contrastDb"] >= 1.5
            )

            sufficient_counts = (
                axis_best["insideCount"] >= 8
                and axis_best["outsideCount"] >= 8
            )

            # First calculate the optimizer's evidence quality.
            raw_axis_candidate_ok = bool(
                sufficient_radio_separation
                and sufficient_counts
            )

            if axis_gap is not None:
                if (
                    axis_gap >= 3.0
                    and axis_uncertainty <= 6.0
                    and axis_best["contrastDb"] >= 4.0
                ):
                    axis_stability = "HIGH"

                elif (
                    axis_gap >= 1.0
                    and axis_uncertainty <= 12.0
                    and axis_best["contrastDb"] >= 2.0
                ):
                    axis_stability = "MEDIUM"

                else:
                    axis_stability = "LOW"

            # V16: compare the 360° winner with the INITIAL radio/geometric axis.
            # The optimizer is allowed to REFINE the axis, not blindly replace it.
            axis_candidate_shift = (
                heading_difference_deg(
                    axis_best["axis"],
                    axis_initial,
                )
                if (
                    axis_best is not None
                    and valid_number(axis_initial)
                )
                else None
            )

            axis_reject_reasons = []

            if not raw_axis_candidate_ok:
                axis_reject_reasons.append(
                    "недостатнє радіорозділення або замало зразків"
                )

            if axis_stability not in ANTENNA_AXIS_ALLOWED_STABILITIES:
                axis_reject_reasons.append(
                    "низька стабільність радіокандидата"
                )

            if (
                axis_gap is None
                or axis_gap < ANTENNA_AXIS_MIN_SCORE_GAP
            ):
                axis_reject_reasons.append(
                    "слабкий відрив від другого кандидата"
                )

            if (
                valid_number(axis_candidate_shift)
                and axis_candidate_shift
                > ANTENNA_AXIS_MAX_REFINEMENT_SHIFT_DEG
            ):
                axis_reject_reasons.append(
                    "розворот >90° — можлива задня пелюстка / 180° неоднозначність"
                )

            axis_optimization_used = bool(
                raw_axis_candidate_ok
                and not axis_reject_reasons
            )

            axis_optimization_rejected = bool(
                axis_best is not None
                and not axis_optimization_used
            )

            axis_optimization_reject_reason = (
                "; ".join(axis_reject_reasons)
                if axis_reject_reasons
                else None
            )

            if axis_optimization_used:
                reference = axis_best["axis"]

        # Applied shift. If V16 rejects the radio candidate, this stays 0°
        # because the initial axis remains the actual sector axis.
        axis_shift = (
            heading_difference_deg(
                reference,
                axis_initial,
            )
            if (
                valid_number(reference)
                and valid_number(axis_initial)
            )
            else None
        )

        # Ensure variables exist even when no axis_trials were available.
        if not axis_trials:
            axis_candidate_shift = None
            axis_optimization_rejected = False
            axis_optimization_reject_reason = None

        top_scores = [
            x["score"]
            for x in top
        ]

        lower = candidates[
            len(candidates) // 2:
        ]

        lower_scores = [
            x["score"]
            for x in lower
        ]

        contrast = None

        if top_scores and lower_scores:
            contrast = max(
                0.0,
                float(
                    statistics.median(
                        top_scores
                    )
                    - statistics.median(
                        lower_scores
                    )
                ),
            )

        deviations = [
            heading_difference_deg(
                x["azimuth"],
                reference,
            )
            for x in candidates
        ]

        deviations = [
            d
            for d in deviations
            if d is not None
        ]

        coverage = (
            min(
                180.0,
                max(deviations) * 2.0,
            )
            if deviations
            else 0.0
        )

        sample_factor = min(
            1.0,
            len(candidates) / 60.0,
        )

        contrast_factor = min(
            1.0,
            (contrast or 0.0) / 12.0,
        )

        coverage_factor = min(
            1.0,
            coverage / 45.0,
        )

        # ----------------------------------------------------
        # Dynamic beam width from corrected dBm vs angle.
        # ----------------------------------------------------
        bin_edges = [
            0, 5, 10, 15, 20, 25,
            30, 40, 50, 60, 90, 180,
        ]

        prof = []

        for i in range(
            len(bin_edges) - 1
        ):
            lo = float(
                bin_edges[i]
            )

            hi = float(
                bin_edges[i + 1]
            )

            values = []

            for x in candidates:
                dev = (
                    heading_difference_deg(
                        x["azimuth"],
                        reference,
                    )
                )

                if dev is None:
                    continue

                in_bin = (
                    lo <= dev < hi
                    if i < len(bin_edges) - 2
                    else lo <= dev <= hi
                )

                if in_bin:
                    values.append(
                        float(x["score"])
                    )

            if values:
                prof.append({
                    "fromDeg": lo,
                    "toDeg": hi,
                    "sampleCount": len(values),
                    "medianCorrectedDbm": float(
                        statistics.median(
                            values
                        )
                    ),
                    "meanCorrectedDbm": float(
                        sum(values)
                        / len(values)
                    ),
                })

        central_values = []

        for x in candidates:
            dev = (
                heading_difference_deg(
                    x["azimuth"],
                    reference,
                )
            )

            if (
                dev is not None
                and dev <= 10.0
            ):
                central_values.append(
                    float(x["score"])
                )

        if len(central_values) >= 5:
            center_signal = float(
                statistics.median(
                    central_values
                )
            )

        elif prof:
            center_signal = max(
                p["medianCorrectedDbm"]
                for p in prof
            )

        else:
            center_signal = None

        for p in prof:
            if center_signal is not None:
                p["dropDb"] = round(
                    max(
                        0.0,
                        center_signal
                        - p["medianCorrectedDbm"],
                    ),
                    2,
                )
            else:
                p["dropDb"] = None

        valid_prof = [
            p
            for p in prof
            if p["sampleCount"] >= 5
            and p["dropDb"] is not None
        ]

        trend_checks = 0
        trend_good = 0

        for i in range(
            1,
            len(valid_prof),
        ):
            prev_p = valid_prof[
                i - 1
            ]

            curr_p = valid_prof[
                i
            ]

            trend_checks += 1

            # Allow small RF noise / multipath.
            if (
                curr_p[
                    "medianCorrectedDbm"
                ]
                <= prev_p[
                    "medianCorrectedDbm"
                ]
                + 1.5
            ):
                trend_good += 1

        trend_fraction = (
            trend_good / trend_checks
            if trend_checks > 0
            else 0.0
        )

        STRONG_DROP_DB = 6.0
        MODERATE_DROP_DB = 3.0

        estimated_half_angle = None
        beam_reason = None
        beam_strength = None

        # Strong boundary.
        for i, p in enumerate(
            valid_prof
        ):
            if p["toDeg"] <= 10.0:
                continue

            if (
                p["dropDb"]
                < STRONG_DROP_DB
            ):
                continue

            next_ok = True

            if (
                i + 1
                < len(valid_prof)
            ):
                next_p = valid_prof[
                    i + 1
                ]

                next_ok = (
                    next_p["dropDb"]
                    >= STRONG_DROP_DB
                    - 1.0
                )

            if next_ok:
                estimated_half_angle = float(
                    p["fromDeg"]
                )

                beam_strength = "STRONG"

                beam_reason = (
                    f"Стійке падіння ≥{STRONG_DROP_DB:.0f} dB "
                    f"починається біля "
                    f"{estimated_half_angle:.1f}°"
                )
                break

        # Moderate boundary, only with a very clean trend.
        if estimated_half_angle is None:
            for p in valid_prof:
                if p["toDeg"] <= 10.0:
                    continue

                if (
                    p["dropDb"]
                    < MODERATE_DROP_DB
                ):
                    continue

                moderate_ok = (
                    trend_fraction >= 0.80
                    and coverage >= 25.0
                    and len(valid_prof) >= 3
                )

                if not moderate_ok:
                    continue

                estimated_half_angle = float(
                    p["fromDeg"]
                )

                beam_strength = "MODERATE"

                beam_reason = (
                    f"Помірне падіння ≥{MODERATE_DROP_DB:.0f} dB "
                    f"при чистому тренді "
                    f"{trend_fraction:.0%}; "
                    f"межа біля "
                    f"{estimated_half_angle:.1f}°"
                )
                break

        beam_dynamic = (
            estimated_half_angle
            is not None
            and len(valid_prof) >= 3
            and coverage >= 25.0
            and (
                (
                    beam_strength
                    == "STRONG"
                    and trend_fraction >= 0.60
                )
                or
                (
                    beam_strength
                    == "MODERATE"
                    and trend_fraction >= 0.80
                )
            )
        )

        if beam_dynamic:
            max_observed_half = max(
                10.0,
                coverage / 2.0,
            )

            estimated_half_angle = max(
                10.0,
                min(
                    60.0,
                    estimated_half_angle,
                    max_observed_half,
                ),
            )

            dynamic_beam_width = (
                estimated_half_angle
                * 2.0
            )

        else:
            dynamic_beam_width = None

            if estimated_half_angle is None:
                beam_reason = (
                    "Не знайдено стійкої межі "
                    "за зміною dBm"
                )

            elif coverage < 25.0:
                beam_reason = (
                    f"Недостатнє кутове "
                    f"покриття: {coverage:.1f}°"
                )

            elif trend_fraction < 0.60:
                beam_reason = (
                    "dBm не погіршується "
                    "достатньо послідовно"
                )

            else:
                beam_reason = (
                    "Недостатньо кутових "
                    "груп із даними"
                )

        if beam_strength == "STRONG":
            beam_evidence_factor = min(
                1.0,
                0.65
                + 0.35
                * trend_fraction,
            )

        elif beam_strength == "MODERATE":
            beam_evidence_factor = min(
                0.82,
                0.50
                + 0.32
                * trend_fraction,
            )

        else:
            beam_evidence_factor = 0.40

        quality = (
            concentration
            * sample_factor
            * (
                0.45
                + 0.20
                * contrast_factor
                + 0.15
                * coverage_factor
                + 0.20
                * beam_evidence_factor
            )
        )

        # V12 confidence keeps the source cap, but also accounts
        # for how clearly the 360° search separates the winning axis.
        if (
            axis_best is not None
            and axis_optimization_used
        ):
            axis_contrast_factor = min(
                1.0,
                max(
                    0.0,
                    axis_best["contrastDb"] / 10.0,
                ),
            )

            axis_gap_factor = min(
                1.0,
                max(
                    0.0,
                    (axis_gap or 0.0) / 4.0,
                ),
            )

            axis_uncertainty_factor = max(
                0.0,
                1.0
                - min(
                    1.0,
                    (axis_uncertainty or 0.0)
                    / 20.0,
                ),
            )

            axis_quality_factor = (
                0.45
                + 0.30 * axis_contrast_factor
                + 0.15 * axis_gap_factor
                + 0.10 * axis_uncertainty_factor
            )

            quality = min(
                1.0,
                quality * 0.75
                + axis_quality_factor * 0.25,
            )

        confidence = int(
            round(
                min(
                    confidence_cap,
                    quality
                    * confidence_cap,
                )
                * 100.0
            )
        )

        # ATTITUDE_DR is less reliable.
        # Require a little more confidence before drawing.
        draw_threshold = (
            40
            if method
            == "ATTITUDE_DR_POSITION"
            else 35
        )

        drawable = bool(
            beam_dynamic
            and confidence
            >= draw_threshold
        )

        beam_width_result = (
            round(
                dynamic_beam_width,
                1,
            )
            if beam_dynamic
            else None
        )

        half_angle_result = (
            round(
                estimated_half_angle,
                1,
            )
            if beam_dynamic
            else None
        )

        # V11:
        # The physical antenna sector is known a priori:
        # 30° total width = ±15° from the estimated antenna axis.
        #
        # The dBm-vs-angle analysis is kept ONLY as a diagnostic of
        # where signal degradation starts outside/around the physical sector.
        physical_half_angle = ANTENNA_HALF_ANGLE_DEG
        physical_beam_width = ANTENNA_BEAM_WIDTH_DEG

        # We can draw the physical sector whenever the AXIS itself has
        # reasonable confidence. Beam width is no longer inferred from dBm.
        axis_draw_threshold = (
            20
            if method == "ATTITUDE_DR_POSITION"
            else 25
        )

        drawable = bool(
            confidence >= axis_draw_threshold
        )

        radio_degradation_angle = (
            round(estimated_half_angle, 1)
            if estimated_half_angle is not None
            else None
        )

        result.update({
            "available": True,
            "drawable": drawable,
            "method": method,
            "center": round(
                reference,
                1,
            ),

            # PHYSICAL SECTOR: ALWAYS 30° / ±15°.
            "sectorMin": round(
                (
                    reference
                    - physical_half_angle
                )
                % 360.0,
                1,
            ),
            "sectorMax": round(
                (
                    reference
                    + physical_half_angle
                )
                % 360.0,
                1,
            ),
            "beamWidth": physical_beam_width,
            "halfAngle": physical_half_angle,
            "beamWidthDynamic": False,
            "beamWidthReason": (
                "Фізична ширина АС задана: 30° (±15°). "
                "dBm не змінює фізичний сектор."
            ),

            # Separate radio behavior outside/around the physical sector.
            "radioDegradationAngle": radio_degradation_angle,
            "radioDegradationReason": beam_reason,
            "radioDegradationStrength": beam_strength,
            "radioDegradationDetected": bool(beam_dynamic),
            "beamStrength": beam_strength,
            "beamDropThresholdDb": (
                STRONG_DROP_DB
                if beam_strength == "STRONG"
                else (
                    MODERATE_DROP_DB
                    if beam_strength == "MODERATE"
                    else None
                )
            ),

            # Kept for backward compatibility, but this is NOT beam width.
            "beamHalfAngleEstimated": radio_degradation_angle,
            "confidence": confidence,
            "sampleCount": len(
                candidates
            ),
            "minDistanceUsed": (
                min_distance_used
            ),
            "signalContrastDb": (
                round(
                    contrast,
                    1,
                )
                if contrast
                is not None
                else None
            ),
            "angularCoverageDeg": round(
                coverage,
                1,
            ),
            "angularTrendFraction": round(
                trend_fraction,
                3,
            ),
            "angularSignalProfile": [
                {
                    "fromDeg": round(
                        p["fromDeg"],
                        1,
                    ),
                    "toDeg": round(
                        p["toDeg"],
                        1,
                    ),
                    "sampleCount": (
                        p["sampleCount"]
                    ),
                    "medianCorrectedDbm": round(
                        p[
                            "medianCorrectedDbm"
                        ],
                        2,
                    ),
                    "dropDb": (
                        round(
                            p["dropDb"],
                            2,
                        )
                        if p["dropDb"]
                        is not None
                        else None
                    ),
                }
                for p in prof
            ],
            "positionSourceQuality": (
                position_quality
            ),

            # V12 axis-search diagnostics.
            "axisInitial": (
                round(axis_initial, 1)
                if valid_number(axis_initial)
                else None
            ),
            "axisOptimized": (
                round(axis_best["axis"], 1)
                if axis_best is not None
                else None
            ),
            "axisShiftDeg": (
                round(axis_shift, 1)
                if valid_number(axis_shift)
                else None
            ),
            "axisScore": (
                round(axis_best["score"], 2)
                if axis_best is not None
                else None
            ),
            "axisSecond": (
                round(axis_second["axis"], 1)
                if axis_second is not None
                else None
            ),
            "axisSecondScore": (
                round(axis_second["score"], 2)
                if axis_second is not None
                else None
            ),
            "axisScoreGap": (
                round(axis_gap, 2)
                if valid_number(axis_gap)
                else None
            ),
            "axisStability": axis_stability,
            "axisUncertaintyDeg": (
                round(axis_uncertainty, 1)
                if valid_number(axis_uncertainty)
                else None
            ),
            "axisOptimizationUsed": bool(
                axis_optimization_used
            ),
            "axisOptimizationRejected": bool(
                axis_optimization_rejected
            ),
            "axisOptimizationRejectReason": (
                axis_optimization_reject_reason
            ),
            "axisCandidateShiftDeg": (
                round(axis_candidate_shift, 1)
                if valid_number(axis_candidate_shift)
                else None
            ),
            "axisInsideMedianDbm": (
                round(axis_best["insideMedian"], 2)
                if axis_best is not None
                else None
            ),
            "axisOutsideMedianDbm": (
                round(axis_best["outsideMedian"], 2)
                if axis_best is not None
                else None
            ),
            "axisInsideCount": (
                axis_best["insideCount"]
                if axis_best is not None
                else 0
            ),
            "axisOutsideCount": (
                axis_best["outsideCount"]
                if axis_best is not None
                else 0
            ),
        })

        return result

    # --------------------------------------------------------
    # Final diagnostic fallback: Heading + dBm.
    # Never treated as geometric position.
    # --------------------------------------------------------
    heading_samples = []

    for x in samples:
        h = x["heading"]
        dbm = x["dbm"]

        if (
            h is None
            or dbm is None
        ):
            continue

        if (
            dbm < RADIO_NORMAL_DBM
            or dbm >= 0
        ):
            continue

        w = max(
            1.0,
            min(
                25.0,
                dbm
                - RADIO_NORMAL_DBM
                + 1.0,
            ),
        )

        heading_samples.append(
            (h, w)
        )

    result["sourceCounts"]["heading"] = len(
        heading_samples
    )

    if (
        len(heading_samples)
        >= ANTENNA_MIN_RADIO_SAMPLES
    ):
        reference, concentration = (
            circular_weighted_mean(
                heading_samples
            )
        )

        sample_factor = min(
            1.0,
            len(heading_samples)
            / 60.0,
        )

        confidence = int(
            round(
                min(
                    0.25,
                    concentration
                    * sample_factor
                    * 0.25,
                )
                * 100.0
            )
        )

        result.update({
            "available": True,
            "drawable": False,
            "method": "HEADING_FALLBACK",
            "center": round(
                reference,
                1,
            ),
            "sectorMin": None,
            "sectorMax": None,
            "beamWidth": ANTENNA_BEAM_WIDTH_DEG,
            "halfAngle": ANTENNA_HALF_ANGLE_DEG,
            "beamWidthDynamic": False,
            "beamWidthReason": (
                "Фізична ширина АС відома: 30° (±15°), "
                "але Heading не дає надійної геометричної осі АС."
            ),
            "radioDegradationAngle": None,
            "radioDegradationReason": None,
            "radioDegradationStrength": None,
            "radioDegradationDetected": False,
            "confidence": confidence,
            "sampleCount": len(
                heading_samples
            ),
            "positionSourceQuality": (
                "VERY_LOW"
            ),
        })

    return result



def analyze_antenna_direction(raw_timeline, arm_timestamp):
    """
    Оцінка напрямку АС та втрати зв'язку.

    PRIMARY: позиційний азимут БПЛА з LOCAL_POSITION_NED + dBm + дальність.
    FALLBACK: Heading БПЛА + dBm, якщо геометричного NED-азимута недостатньо.

    Важливо: fallback по Heading є лише евристикою, тому що Heading показує
    напрямок носа БПЛА, а не геометричний напрямок від АС до БПЛА.
    """
    result = {
        "available": False,
        "method": None,
        "center": None,
        "sectorMin": None,
        "sectorMax": None,
        "beamWidth": ANTENNA_BEAM_WIDTH_DEG,
        "halfAngle": ANTENNA_HALF_ANGLE_DEG,
        "confidence": 0,
        "radioSampleCount": 0,
        "longLossEpisodes": [],
        "probableSectorExitCount": 0,
        "firstProbableExitTimestamp": None,
        "maxDeviation": 0.0,
        "probableBoardLoss": False,
        "probableBoardLossDueSector": False,

        # Five-sign evidence model.
        "sectorEvidenceEpisodes": [],
        "strongestSectorEvidence": None,
        "sectorEvidenceScore": 0,
        "sectorEvidenceLevel": "NONE",
    }

    if arm_timestamp is None:
        return result

    snapshots = []
    for ev in sorted(raw_timeline, key=lambda x: x.get("timestamp", 0)):
        if ev.get("eventType") != "SNAPSHOT":
            continue
        ts = ev.get("timestamp")
        if ts is None or float(ts) < arm_timestamp:
            continue
        pos_az = ev.get("positionAzimuth")
        heading = ev.get("azimuth")
        dist = ev.get("distValue")
        dbm = ev.get("dbm")
        snapshots.append({
            "timestamp": float(ts),
            "positionAzimuth": float(pos_az) % 360.0 if valid_number(pos_az) else None,
            "heading": float(heading) % 360.0 if valid_number(heading) else None,
            "distance": float(dist) if valid_number(dist) else None,
            "dbm": float(dbm) if valid_number(dbm) else None,
        })

    # --------------------------------------------------------
    # 1) PRIMARY: оцінюємо вісь АС за NED-позицією + добрим сигналом.
    # Для визначення осі використовуємо в першу чергу NORMAL (>= -85 dBm).
    # Якщо таких мало, допускаємо всі не-втрачені зразки > -128 dBm.
    # --------------------------------------------------------
    good_position = []
    fallback_position = []
    for x in snapshots:
        az, dist, dbm = x["positionAzimuth"], x["distance"], x["dbm"]
        if az is None or dist is None or dbm is None or dist < ANTENNA_MIN_DISTANCE_M:
            continue
        if dbm <= RADIO_LINK_LOST_DBM or dbm >= 0:
            continue
        corrected = dbm + 20.0 * math.log10(max(dist, 1.0))
        fallback_position.append((corrected, az, dbm, dist, x["timestamp"]))
        if dbm >= RADIO_NORMAL_DBM:
            good_position.append((corrected, az, dbm, dist, x["timestamp"]))

    scored = good_position if len(good_position) >= ANTENNA_MIN_RADIO_SAMPLES else fallback_position
    result["radioSampleCount"] = len(scored)

    reference = None
    compare_key = "positionAzimuth"

    if len(scored) >= ANTENNA_MIN_RADIO_SAMPLES:
        scored.sort(key=lambda x: x[0], reverse=True)
        top_n = max(ANTENNA_MIN_RADIO_SAMPLES, int(math.ceil(len(scored) * ANTENNA_TOP_SIGNAL_FRACTION)))
        top = scored[:min(top_n, len(scored))]
        min_score = min(x[0] for x in top)
        weighted = [(x[1], (x[0] - min_score) + 1.0) for x in top]
        reference, concentration = circular_weighted_mean(weighted)
        sample_factor = min(1.0, len(scored) / 60.0)
        confidence = int(round(concentration * sample_factor * 100.0))
        result["method"] = "POSITION_NED"
        result["confidence"] = confidence

    # --------------------------------------------------------
    # 1b) FALLBACK: якщо NED не вистачає — оцінка за Heading + нормальним dBm.
    # Це спеціально позначається як евристика з обмеженою впевненістю.
    # --------------------------------------------------------
    if reference is None:
        heading_samples = []
        for x in snapshots:
            h, dbm = x["heading"], x["dbm"]
            if h is None or dbm is None or dbm < RADIO_NORMAL_DBM or dbm >= 0:
                continue
            # Кращий dBm має більшу вагу, але без надмірного домінування.
            w = max(1.0, min(25.0, dbm - RADIO_NORMAL_DBM + 1.0))
            heading_samples.append((h, w))
        if len(heading_samples) >= ANTENNA_MIN_RADIO_SAMPLES:
            reference, concentration = circular_weighted_mean(heading_samples)
            sample_factor = min(1.0, len(heading_samples) / 60.0)
            # Heading-fallback навмисно обмежуємо 65%.
            confidence = int(round(min(0.65, concentration * sample_factor * 0.65) * 100.0))
            result["method"] = "HEADING_FALLBACK"
            result["confidence"] = confidence
            result["radioSampleCount"] = len(heading_samples)
            compare_key = "heading"

    if reference is not None:
        result["available"] = True
        result["center"] = round(reference, 1)
        result["sectorMin"] = round((reference - ANTENNA_HALF_ANGLE_DEG) % 360.0, 1)
        result["sectorMax"] = round((reference + ANTENNA_HALF_ANGLE_DEG) % 360.0, 1)

    # --------------------------------------------------------
    # 2) 5 ОЗНАК ВИХОДУ ЗА СЕКТОР АС
    #
    # 1. Геометричний вихід за межу сектора.
    # 2. Відхилення від осі АС продовжує збільшуватись.
    # 3. dBm має стійкий тренд на погіршення.
    # 4. Сигнал доходить до -128 dBm.
    # 5. Після виходу немає повернення в сектор АБО немає
    #    відновлення радіоканалу після -128.
    #
    # Важливо: одна ознака сама по собі НЕ є доказом втрати БПЛА.
    # --------------------------------------------------------
    evidence_episodes = []

    def median_or_none(values):
        vals = [float(v) for v in values if valid_number(v)]
        if not vals:
            return None
        return float(statistics.median(vals))

    def linear_slope(samples):
        """Least-squares slope y/sec for [(timestamp, value), ...]."""
        pts = [(float(t), float(v)) for t, v in samples if valid_number(t) and valid_number(v)]
        if len(pts) < 2:
            return None
        t0 = pts[0][0]
        xs = [t - t0 for t, _ in pts]
        ys = [v for _, v in pts]
        xbar = sum(xs) / len(xs)
        ybar = sum(ys) / len(ys)
        den = sum((x - xbar) ** 2 for x in xs)
        if den <= 1e-9:
            return None
        return sum((x - xbar) * (y - ybar) for x, y in zip(xs, ys)) / den

    def fraction_worsening(values):
        vals = [float(v) for v in values if valid_number(v)]
        if len(vals) < 2:
            return 0.0
        # dBm "погіршується", коли число стає більш від'ємним.
        worsened = sum(1 for a, b in zip(vals, vals[1:]) if b <= a)
        return worsened / max(1, len(vals) - 1)

    # Enrich snapshots with angular deviation / inside-outside.
    for x in snapshots:
        angle = x.get(compare_key)
        dev = None
        outside = False
        if reference is not None and valid_number(angle):
            dev = heading_difference_deg(float(angle), reference)
            outside = dev is not None and dev > ANTENNA_HALF_ANGLE_DEG
        x["sectorDeviation"] = dev
        x["outsideSector"] = outside

    if reference is not None and snapshots:
        # Build contiguous geometric outside-sector runs.
        runs = []
        active_run = None

        for idx, x in enumerate(snapshots):
            if x.get("outsideSector"):
                if active_run is None:
                    active_run = {
                        "startIndex": idx,
                        "endIndex": idx,
                        "start": x["timestamp"],
                        "end": x["timestamp"],
                    }
                elif x["timestamp"] - snapshots[active_run["endIndex"]]["timestamp"] <= ANTENNA_SAMPLE_MAX_GAP_SEC * 2.5:
                    active_run["endIndex"] = idx
                    active_run["end"] = x["timestamp"]
                else:
                    runs.append(active_run)
                    active_run = {
                        "startIndex": idx,
                        "endIndex": idx,
                        "start": x["timestamp"],
                        "end": x["timestamp"],
                    }
            elif active_run is not None:
                runs.append(active_run)
                active_run = None

        if active_run is not None:
            runs.append(active_run)

        for run in runs:
            start_i = run["startIndex"]
            end_i = run["endIndex"]
            start_ts = run["start"]
            end_ts = run["end"]

            # Include a look-back window before the geometric exit so the
            # dBm/deviation trend can show the approach to the sector edge.
            trend_start_ts = start_ts - ANTENNA_TREND_LOOKBACK_SEC
            trend_samples = [
                x for x in snapshots
                if trend_start_ts <= x["timestamp"] <= end_ts
            ]
            outside_samples = snapshots[start_i:end_i + 1]

            deviations = [
                float(x["sectorDeviation"])
                for x in trend_samples
                if valid_number(x.get("sectorDeviation"))
            ]
            dbm_samples = [
                (x["timestamp"], float(x["dbm"]))
                for x in trend_samples
                if valid_number(x.get("dbm")) and float(x["dbm"]) < 0
            ]

            # SIGN 1: geometric outside-sector.
            sign1_outside = bool(outside_samples)

            # SIGN 2: angular deviation keeps growing.
            sign2_dev_growing = False
            dev_growth = None
            dev_slope = None
            if len(deviations) >= ANTENNA_TREND_MIN_SAMPLES:
                chunk = max(1, len(deviations) // 3)
                first_med = median_or_none(deviations[:chunk])
                last_med = median_or_none(deviations[-chunk:])
                if first_med is not None and last_med is not None:
                    dev_growth = last_med - first_med
                dev_slope = linear_slope([
                    (x["timestamp"], x["sectorDeviation"])
                    for x in trend_samples
                    if valid_number(x.get("sectorDeviation"))
                ])
                sign2_dev_growing = bool(
                    dev_growth is not None
                    and dev_growth >= ANTENNA_DEVIATION_GROWTH_MIN_DEG
                    and (dev_slope is None or dev_slope > 0)
                )

            # SIGN 3: stable worsening dBm trend.
            sign3_dbm_worsening = False
            dbm_drop = None
            dbm_slope = None
            dbm_worse_fraction = 0.0
            if len(dbm_samples) >= ANTENNA_TREND_MIN_SAMPLES:
                dbm_values = [v for _, v in dbm_samples]
                chunk = max(1, len(dbm_values) // 3)
                first_med = median_or_none(dbm_values[:chunk])
                last_med = median_or_none(dbm_values[-chunk:])
                if first_med is not None and last_med is not None:
                    # Positive number means signal got worse by N dB.
                    dbm_drop = first_med - last_med
                dbm_slope = linear_slope(dbm_samples)
                dbm_worse_fraction = fraction_worsening(dbm_values)

                sign3_dbm_worsening = bool(
                    dbm_drop is not None
                    and dbm_drop >= ANTENNA_DBM_WORSEN_MIN_DB
                    and dbm_slope is not None
                    and dbm_slope <= -ANTENNA_DBM_WORSEN_MIN_SLOPE_DB_PER_SEC
                    and dbm_worse_fraction >= ANTENNA_DBM_WORSEN_MIN_FRACTION
                )

            # SIGN 4: reaches complete-loss indicator -128 dBm.
            sign4_reaches_128 = any(
                valid_number(x.get("dbm")) and float(x["dbm"]) <= RADIO_LINK_LOST_DBM
                for x in trend_samples
            )

            # SIGN 5: after geometric exit there is no geometric return OR
            # after reaching -128 there is no radio recovery.
            future = snapshots[end_i + 1:]
            returned_to_sector = any(
                not bool(x.get("outsideSector"))
                for x in future
                if valid_number(x.get(compare_key))
            )

            radio_recovered_after_128 = False
            if sign4_reaches_128:
                first_128_ts = next(
                    (
                        x["timestamp"]
                        for x in trend_samples
                        if valid_number(x.get("dbm"))
                        and float(x["dbm"]) <= RADIO_LINK_LOST_DBM
                    ),
                    None,
                )
                if first_128_ts is not None:
                    radio_recovered_after_128 = any(
                        valid_number(x.get("dbm"))
                        and float(x["dbm"]) > RADIO_LINK_LOST_DBM
                        for x in snapshots
                        if x["timestamp"] > first_128_ts
                    )

            sign5_no_return_or_recovery = bool(
                (not returned_to_sector)
                or (sign4_reaches_128 and not radio_recovered_after_128)
            )

            signs = {
                "outsideSector": sign1_outside,
                "deviationGrowing": sign2_dev_growing,
                "dbmWorsening": sign3_dbm_worsening,
                "reachedMinus128": sign4_reaches_128,
                "noReturnOrRecovery": sign5_no_return_or_recovery,
            }
            score = sum(1 for v in signs.values() if v)

            if score >= 5:
                level = "VERY_HIGH"
            elif score >= 4:
                level = "HIGH"
            elif score >= 3:
                level = "MEDIUM"
            elif score >= 2:
                level = "LOW"
            else:
                level = "WEAK"

            evidence_episodes.append({
                "start": start_ts,
                "end": end_ts,
                "duration": round(max(0.0, end_ts - start_ts), 2),
                "score": score,
                "maxScore": 5,
                "level": level,
                "signs": signs,
                "firstOutsideAngle": round(float(outside_samples[0].get(compare_key)), 1)
                    if outside_samples and valid_number(outside_samples[0].get(compare_key)) else None,
                "maxDeviation": round(
                    max(
                        [float(x["sectorDeviation"]) for x in outside_samples if valid_number(x.get("sectorDeviation"))]
                        or [0.0]
                    ),
                    1,
                ),
                "deviationGrowth": round(dev_growth, 1) if dev_growth is not None else None,
                "deviationSlopeDegPerSec": round(dev_slope, 3) if dev_slope is not None else None,
                "dbmDrop": round(dbm_drop, 1) if dbm_drop is not None else None,
                "dbmSlopePerSec": round(dbm_slope, 3) if dbm_slope is not None else None,
                "dbmWorseningFraction": round(dbm_worse_fraction, 2),
                "returnedToSector": returned_to_sector,
                "radioRecoveredAfterMinus128": radio_recovered_after_128,
            })

        if evidence_episodes:
            # Prefer higher score; for equal score prefer larger deviation and later/longer evidence.
            strongest = max(
                evidence_episodes,
                key=lambda ep: (
                    ep.get("score", 0),
                    ep.get("maxDeviation", 0.0),
                    ep.get("duration", 0.0),
                ),
            )
            result["sectorEvidenceEpisodes"] = evidence_episodes
            result["strongestSectorEvidence"] = strongest
            result["sectorEvidenceScore"] = strongest.get("score", 0)
            result["sectorEvidenceLevel"] = strongest.get("level", "NONE")

    # --------------------------------------------------------
    # 2) Безперервні епізоди -128 dBm >= 60 с.
    # Якщо після епізоду з'явився dBm > -128 — зв'язок відновився.
    # Якщо епізод доходить до EOF — recovered=False.
    # --------------------------------------------------------
    long_episodes = []
    active = None

    def close_episode(ep, recovered, recovery_ts=None):
        if ep is None:
            return
        duration = ep["end"] - ep["start"]
        if duration < RADIO_LONG_LOSS_SEC:
            return

        angle_samples = ep["angleSamples"]
        outside_samples = 0
        max_dev = 0.0
        first_angle = None
        if reference is not None:
            for _, angle in angle_samples:
                dev = heading_difference_deg(angle, reference) or 0.0
                if first_angle is None:
                    first_angle = angle
                if dev > ANTENNA_HALF_ANGLE_DEG:
                    outside_samples += 1
                max_dev = max(max_dev, dev)

        outside_fraction = outside_samples / len(angle_samples) if angle_samples and reference is not None else 0.0
        probable_exit = reference is not None and len(angle_samples) >= 10 and outside_fraction >= 0.60

        long_episodes.append({
            "start": ep["start"],
            "end": ep["end"],
            "duration": round(duration, 1),
            "recovered": bool(recovered),
            "recoveryTimestamp": recovery_ts,
            "firstAngle": round(first_angle, 1) if first_angle is not None else None,
            "angleType": compare_key,
            "outsideFraction": round(outside_fraction, 2),
            "maxDeviation": round(max_dev, 1),
            "probableSectorExit": probable_exit,
            "probableBoardLoss": not bool(recovered),
        })

    for x in snapshots:
        ts, dbm = x["timestamp"], x["dbm"]
        is_lost = dbm is not None and dbm <= RADIO_LINK_LOST_DBM
        if is_lost:
            if active is None:
                active = {"start": ts, "end": ts, "angleSamples": []}
            elif ts - active["end"] > ANTENNA_SAMPLE_MAX_GAP_SEC:
                close_episode(active, False, None)
                active = {"start": ts, "end": ts, "angleSamples": []}
            active["end"] = ts
            angle = x.get(compare_key)
            if angle is not None:
                active["angleSamples"].append((ts, angle))
        else:
            if active is not None:
                close_episode(active, True, ts)
                active = None

    if active is not None:
        close_episode(active, False, None)

    result["longLossEpisodes"] = long_episodes
    probable = [x for x in long_episodes if x["probableSectorExit"]]
    unrecovered = [x for x in long_episodes if not x.get("recovered")]
    unrecovered_sector = [x for x in probable if not x.get("recovered")]
    result["probableSectorExitCount"] = len(probable)
    result["probableBoardLoss"] = bool(unrecovered)
    result["probableBoardLossDueSector"] = bool(unrecovered_sector)
    if probable:
        result["firstProbableExitTimestamp"] = probable[0]["start"]
        result["maxDeviation"] = max(x["maxDeviation"] for x in probable)

    # --------------------------------------------------------
    # 3) Анотація Timeline.
    # --------------------------------------------------------
    for ev in raw_timeline:
        ts = ev.get("timestamp")
        dbm = ev.get("dbm")
        state = radio_state_from_dbm(dbm)
        ev["radioState"] = state
        ev["radioStateText"] = radio_state_text(state)

        if ts is None:
            ev["antennaSector"] = None
            continue

        episode = next((x for x in long_episodes if x["start"] <= ts <= x["end"]), None)
        angle = ev.get(compare_key)
        dev = None
        outside = False
        if reference is not None and valid_number(angle):
            dev = heading_difference_deg(float(angle), reference)
            outside = dev is not None and dev > ANTENNA_HALF_ANGLE_DEG

        probable_here = bool(
            episode
            and episode.get("probableSectorExit")
            and outside
            and valid_number(dbm)
            and float(dbm) <= RADIO_LINK_LOST_DBM
        )

        # Find the matching five-sign geometric exit episode, if any.
        evidence_here = next(
            (
                ep for ep in result.get("sectorEvidenceEpisodes", [])
                if ep.get("start") is not None
                and ep.get("end") is not None
                and float(ep["start"]) <= float(ts) <= float(ep["end"])
            ),
            None,
        )

        ev["antennaSector"] = {
            "available": result["available"],
            "method": result["method"],
            "center": result["center"],
            "sectorMin": result["sectorMin"],
            "sectorMax": result["sectorMax"],
            "positionAzimuth": ev.get("positionAzimuth"),
            "heading": ev.get("azimuth"),
            "comparisonAngle": round(float(angle), 1) if valid_number(angle) else None,
            "angleType": compare_key,
            "deviation": round(float(dev), 1) if dev is not None else None,
            "outside": outside,
            "longLoss": episode is not None,
            "probableSectorExit": probable_here,
            "probableBoardLoss": bool(episode and not episode.get("recovered")),
            "beamWidth": ANTENNA_BEAM_WIDTH_DEG,
            "evidenceScore": evidence_here.get("score", 0) if evidence_here else 0,
            "evidenceLevel": evidence_here.get("level") if evidence_here else None,
            "evidenceSigns": evidence_here.get("signs") if evidence_here else None,
        }

    # ========================================================
    # V8 SAFETY — HEADING_FALLBACK is NOT geometric position.
    #
    # Heading tells where the aircraft nose points, not where the
    # aircraft is located relative to the antenna station.
    # Therefore a "sector exit" based on HEADING_FALLBACK must never
    # be presented as a confirmed geometric 5/5 exit.
    # ========================================================
    if result.get("method") == "HEADING_FALLBACK":
        original_score = int(result.get("sectorEvidenceScore", 0) or 0)

        # Keep the raw evidence episodes for diagnostics, but downgrade
        # their interpretation because sign #1 is not truly geometric.
        result["headingFallbackOriginalEvidenceScore"] = original_score
        result["headingFallbackGeometricEvidenceValid"] = False

        # Cap the effective score at 2/5.
        # This allows "possible radio correlation" but prevents
        # HIGH / VERY_HIGH geometric-sector conclusions.
        capped_score = min(original_score, 2)

        result["sectorEvidenceScore"] = capped_score

        if capped_score >= 2:
            result["sectorEvidenceLevel"] = "LOW"
        elif capped_score == 1:
            result["sectorEvidenceLevel"] = "WEAK"
        else:
            result["sectorEvidenceLevel"] = "NONE"

        # Heading-only cannot establish a geometric sector exit.
        result["probableSectorExitCount"] = 0
        result["firstProbableExitTimestamp"] = None
        result["probableBoardLossDueSector"] = False

        # Do not let a Heading-only sector interpretation claim board
        # loss due to antenna sector.
        if result.get("probableBoardLossDueSector"):
            result["probableBoardLossDueSector"] = False

        # Mark all evidence episodes as non-geometric diagnostics.
        for ep in result.get("sectorEvidenceEpisodes", []):
            ep["geometricPositionValid"] = False
            ep["methodWarning"] = (
                "HEADING_FALLBACK: Heading показує напрямок носа БПЛА, "
                "а не позиційний азимут від АС до борта."
            )

        strongest = result.get("strongestSectorEvidence")
        if isinstance(strongest, dict):
            strongest["geometricPositionValid"] = False
            strongest["effectiveScore"] = capped_score
            strongest["effectiveLevel"] = result["sectorEvidenceLevel"]
            strongest["methodWarning"] = (
                "HEADING_FALLBACK не підтверджує геометричний вихід за сектор."
            )


    return result


def analyze_flight_sessions(raw_timeline, log_end_timestamp=None):
    """One flight = ARM->DISARM. Re-takeoff without DISARM stays in same flight."""
    events=sorted([e for e in raw_timeline if e.get("timestamp") is not None],
                  key=lambda e:e["timestamp"])
    sessions=[]
    active=None

    for ev in events:
        txt=str(ev.get("system_text") or "")
        ts=ev["timestamp"]
        if "🟢 Двигуни запущено" in txt:
            if active is not None:
                active["endedArmed"]=True
                active["endTimestamp"]=ts
                active["duration"]=round(max(0.0,ts-active["armTimestamp"]),2)
            active={
                "number":len(sessions)+1,"armTimestamp":ts,"disarmTimestamp":None,
                "endTimestamp":None,"duration":None,"endedArmed":False,
                "takeoffEpisodes":[],"maxAltitude":0.0,"maxCurrent":0.0,
                "minVoltage":None,"maxVibration":0.0,"maxTilt":0.0
            }
            sessions.append(active)
        elif "🔴 Двигуни зупинено" in txt and active is not None:
            active["disarmTimestamp"]=ts
            active["endTimestamp"]=ts
            active["duration"]=round(max(0.0,ts-active["armTimestamp"]),2)
            active=None

    if active is not None:
        end_ts=log_end_timestamp if log_end_timestamp is not None else (events[-1]["timestamp"] if events else active["armTimestamp"])
        active["endedArmed"]=True
        active["endTimestamp"]=end_ts
        active["duration"]=round(max(0.0,end_ts-active["armTimestamp"]),2)

    for s in sessions:
        evs=[e for e in events if e["timestamp"]>=s["armTimestamp"] and
             (s["endTimestamp"] is None or e["timestamp"]<=s["endTimestamp"])]

        for ev in evs:
            if valid_number(ev.get("alt")):
                s["maxAltitude"]=max(s["maxAltitude"],max(0.0,float(ev["alt"])))
            if valid_number(ev.get("curr")):
                s["maxCurrent"]=max(s["maxCurrent"],max(0.0,float(ev["curr"])))
            if valid_number(ev.get("volt")) and float(ev["volt"])>0:
                if s["minVoltage"] is None or float(ev["volt"])<s["minVoltage"]:
                    s["minVoltage"]=float(ev["volt"])
            vib=ev.get("vibration")
            if isinstance(vib,dict):
                vals=[abs(float(vib[k])) for k in ("x","y","z") if valid_number(vib.get(k))]
                if vals:s["maxVibration"]=max(s["maxVibration"],max(vals))
            att=ev.get("attitude")
            if isinstance(att,dict):
                vals=[abs(float(att[k])) for k in ("roll","pitch") if valid_number(att.get(k))]
                if vals:s["maxTilt"]=max(s["maxTilt"],max(vals))

        airborne=False
        take_candidate=None
        ground_candidate=None
        ep=None
        for ev in evs:
            if not valid_number(ev.get("alt")): continue
            ts=ev["timestamp"]; alt=max(0.0,float(ev["alt"]))
            if not airborne:
                if alt>=FLIGHT_TAKEOFF_ALT_M:
                    if take_candidate is None: take_candidate=ts
                    if ts-take_candidate>=FLIGHT_MIN_AIRBORNE_SEC:
                        airborne=True
                        ep={"number":len(s["takeoffEpisodes"])+1,
                            "startTimestamp":take_candidate,"endTimestamp":None,
                            "duration":None,"endReason":None,"maxAltitude":alt}
                        s["takeoffEpisodes"].append(ep)
                        ground_candidate=None
                else:
                    take_candidate=None
            else:
                ep["maxAltitude"]=max(float(ep.get("maxAltitude") or 0),alt)
                if alt<=FLIGHT_LANDED_ALT_M:
                    if ground_candidate is None: ground_candidate=ts
                    if ts-ground_candidate>=FLIGHT_MIN_GROUND_GAP_SEC:
                        ep["endTimestamp"]=ground_candidate
                        ep["duration"]=round(max(0.0,ground_candidate-ep["startTimestamp"]),2)
                        ep["endReason"]="landed"
                        ep=None; airborne=False; take_candidate=None
                else:
                    ground_candidate=None

        if airborne and ep is not None:
            ep["endTimestamp"]=s["endTimestamp"]
            ep["duration"]=round(max(0.0,s["endTimestamp"]-ep["startTimestamp"]),2) if s["endTimestamp"] is not None else None
            ep["endReason"]="log_end" if s["endedArmed"] else "disarm"

        s["takeoffEpisodeCount"]=len(s["takeoffEpisodes"])
        s["hasRepeatedTakeoff"]=s["takeoffEpisodeCount"]>1
        s["maxAltitude"]=round(float(s["maxAltitude"]),1)
        s["maxCurrent"]=round(float(s["maxCurrent"]),1)
        s["minVoltage"]=round(float(s["minVoltage"]),2) if s["minVoltage"] is not None else None
        s["maxVibration"]=round(float(s["maxVibration"]),1)
        s["maxTilt"]=round(float(s["maxTilt"]),1)

    for ev in raw_timeline:
        ev["flightNumber"]=None; ev["takeoffEpisodeNumber"]=None
        ts=ev.get("timestamp")
        if ts is None: continue
        for s in sessions:
            if ts>=s["armTimestamp"] and (s["endTimestamp"] is None or ts<=s["endTimestamp"]):
                ev["flightNumber"]=s["number"]
                for ep in s["takeoffEpisodes"]:
                    if ts>=ep["startTimestamp"] and (ep.get("endTimestamp") is None or ts<=ep["endTimestamp"]):
                        ev["takeoffEpisodeNumber"]=ep["number"]; break
                break

    def clone_near(ts):
        return dict(min(events,key=lambda e:abs(e["timestamp"]-ts))) if events else {"timestamp":ts,"mode":"","alt":0.0,"dist":0.0}

    markers=[]
    for s in sessions:
        r=clone_near(s["armTimestamp"])
        r.update(timestamp=s["armTimestamp"],system_text="",analysis_text=f"🟢 ПОЛІТ №{s['number']} — ПОЧАТОК СЕСІЇ / ARM",
                 eventType="FLIGHT_SESSION_START",is_error=False,flightNumber=s["number"],takeoffEpisodeNumber=None)
        markers.append(r)
        for ep in s["takeoffEpisodes"]:
            r=clone_near(ep["startTimestamp"])
            r.update(timestamp=ep["startTimestamp"],
                     system_text="",analysis_text=f"↗ {'ПОВТОРНИЙ ЗЛІТ' if ep['number']>1 else 'ЗЛІТ'} — політ №{s['number']}, епізод {ep['number']}",
                     eventType="FLIGHT_TAKEOFF",is_error=False,flightNumber=s["number"],takeoffEpisodeNumber=ep["number"])
            markers.append(r)
            if ep.get("endTimestamp") is not None and ep.get("endReason")=="landed":
                r=clone_near(ep["endTimestamp"])
                r.update(timestamp=ep["endTimestamp"],
                         system_text="",analysis_text=f"↘ ПОСАДКА — політ №{s['number']}, епізод {ep['number']} завершено БЕЗ DISARM",
                         eventType="FLIGHT_LANDING",is_error=False,flightNumber=s["number"],takeoffEpisodeNumber=ep["number"])
                markers.append(r)
        if s.get("disarmTimestamp") is not None:
            r=clone_near(s["disarmTimestamp"])
            r.update(timestamp=s["disarmTimestamp"],system_text="",analysis_text=f"🔵 ПОЛІТ №{s['number']} — ЗАВЕРШЕННЯ СЕСІЇ / DISARM",
                     eventType="FLIGHT_SESSION_END",is_error=False,flightNumber=s["number"],takeoffEpisodeNumber=None)
            markers.append(r)
        elif s.get("endedArmed"):
            r=clone_near(s["endTimestamp"])
            r.update(timestamp=s["endTimestamp"],system_text="",analysis_text=f"🚨 ПОЛІТ №{s['number']} — TLOG ЗАВЕРШИВСЯ ПРИ ARMED",
                     eventType="FLIGHT_SESSION_OPEN_AT_END",is_error=True,flightNumber=s["number"],takeoffEpisodeNumber=None)
            markers.append(r)

    raw_timeline.extend(markers)
    raw_timeline.sort(key=lambda e:e.get("timestamp",0))
    return sessions

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
        max_roll_timestamp = None
        max_pitch_timestamp = None
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
        curr_azimuth = None  # aircraft Heading from VFR_HUD
        curr_position_azimuth = None  # geometric NED azimuth from origin to aircraft

        # Raw LOCAL_POSITION_NED coordinates for 2D map.
        curr_ned_north = None
        curr_ned_east = None
        curr_ned_down = None

        # Current VFR_HUD groundspeed in m/s.
        curr_groundspeed = None

        curr_voltage = 0.0
        curr_amp = 0.0
        curr_rssi_pct = 0
        curr_dbm = 0
        curr_vertical_speed_down = None

        # LAND analysis
        land_params = {
            "LAND_SPEED": None,
            "LAND_SPEED_HIGH": None,
            "LAND_ALT_LOW": None,
        }
        land_entries = []
        active_land_entry = None
        land_samples = []

        # Attitude (ATTITUDE message)
        curr_roll = None
        curr_pitch = None
        curr_yaw = None
        attitude_critical_events = []
        attitude_critical_active = False
        attitude_critical_peak = None

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

        # V20 — primary stick controls CH1 Roll, CH2 Pitch, CH3 Throttle, CH4 Yaw.
        # Prefer RCx_MIN/TRIM/MAX embedded in the TLOG; otherwise use 1000/1500/2000 us.
        rc_axis_cal = {
            ch: {f"RC{ch}_MIN": None, f"RC{ch}_TRIM": None, f"RC{ch}_MAX": None}
            for ch in range(1, 5)
        }
        curr_control_pwm = {ch: None for ch in range(1, 5)}
        control_extremes = {
            ch: {"negative": 0.0, "positive": 0.0}
            for ch in range(1, 5)
        }
        # V19 compatibility values for CH3.
        curr_ch3_pwm = None
        throttle_max_up_pct = 0.0
        throttle_max_down_pct = 0.0

        # V18 — TX16S named-switch diagnostics.
        tx16_switch_pwm = {"SC": None, "SD": None, "SF": None, "SH": None}
        tx16_switch_state = {"SC": None, "SD": None, "SF": None, "SH": None}
        payload_commands = []
        pending_payload_command = None
        servo_output_last = {i: None for i in range(1, 17)}
        servo_output_events = []

        emergency_stop_attempts = []
        active_emergency_attempt = None
        emergency_stop_confirmed_count = 0

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

        # RPM diagnostic events.
        rpm_drop_events = []
        rpm_pair_candidate = {"1-2": None, "3-4": None}
        rpm_pair_active = {"1-2": False, "3-4": False}
        potential_thrust_loss_events = []

        # Ground calibration context.
        accel_calibration_events = []
        accel_calibration_active = False
        accel_calibration_success = False
        accel_calibration_requires_reboot = False
        accel_calibration_start_ts = None
        accel_calibration_end_ts = None

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

        # V15 — preserve exact EKF event timestamps + flight mode.
        ekf_variance_events = []
        ekf_stopped_aiding_events = []
        mode_transition_events = []

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

        def attitude_snapshot():
            if curr_roll is None or curr_pitch is None or curr_yaw is None:
                return None

            # Yaw is displayed for orientation, but is not a tilt angle.
            # Critical tilt is determined by Roll/Pitch only.
            is_critical = (
                abs(curr_roll) >= ATTITUDE_CRITICAL_THRESHOLD_DEG
                or abs(curr_pitch) >= ATTITUDE_CRITICAL_THRESHOLD_DEG
            )

            return {
                "roll": round(curr_roll, 1),
                "pitch": round(curr_pitch, 1),
                "yaw": round(curr_yaw % 360.0, 1),
                "isCritical": is_critical,
                "threshold": ATTITUDE_CRITICAL_THRESHOLD_DEG,
            }

        def rpm_analysis_snapshot():
            """
            Compare RPM inside the configured diagonal pairs:
              M1 <-> M2 and M3 <-> M4.
            """
            rpms = [
                float(v) if valid_number(v) else None
                for v in esc_rpm_current
            ]

            pairs = []
            any_warning = False
            any_critical = False

            for a, b in RPM_DIAGONAL_PAIRS:
                ra = rpms[a]
                rb = rpms[b]
                name = f"{a + 1}-{b + 1}"

                item = {
                    "pair": name,
                    "motorA": a + 1,
                    "motorB": b + 1,
                    "rpmA": int(round(ra)) if ra is not None else None,
                    "rpmB": int(round(rb)) if rb is not None else None,
                    "differencePct": None,
                    "lowerMotor": None,
                    "higherMotor": None,
                    "isWarning": False,
                    "isCritical": False,
                }

                if ra is not None and rb is not None:
                    high = max(ra, rb)

                    if high >= RPM_MIN_ACTIVE and high > 0:
                        diff = abs(ra - rb) / high * 100.0
                        lower = a + 1 if ra < rb else b + 1
                        higher = b + 1 if ra < rb else a + 1

                        item["differencePct"] = round(diff, 1)
                        item["lowerMotor"] = lower
                        item["higherMotor"] = higher
                        item["isWarning"] = diff >= RPM_DIAGONAL_WARNING_PCT
                        item["isCritical"] = diff >= RPM_DIAGONAL_CRITICAL_PCT

                        any_warning = any_warning or item["isWarning"]
                        any_critical = any_critical or item["isCritical"]

                pairs.append(item)

            return {
                "pairs": pairs,
                "isWarning": any_warning,
                "isCritical": any_critical,
                "warningThresholdPct": RPM_DIAGONAL_WARNING_PCT,
                "criticalThresholdPct": RPM_DIAGONAL_CRITICAL_PCT,
                "pairsText": "M1↔M2, M3↔M4",
            }

        CONTROL_AXIS_META = {
            1: {"key": "roll", "name": "Roll", "positive": "RIGHT", "negative": "LEFT",
                "positiveLabel": "→ Праворуч", "negativeLabel": "← Ліворуч"},
            2: {"key": "pitch", "name": "Pitch", "positive": "BACK", "negative": "FORWARD",
                "positiveLabel": "↓ До себе", "negativeLabel": "↑ Від себе"},
            3: {"key": "throttle", "name": "Throttle", "positive": "UP", "negative": "DOWN",
                "positiveLabel": "↑ Вгору", "negativeLabel": "↓ Вниз"},
            4: {"key": "yaw", "name": "Yaw", "positive": "RIGHT", "negative": "LEFT",
                "positiveLabel": "→ Праворуч", "negativeLabel": "← Ліворуч"},
        }

        def control_calibration(ch_num):
            cal = rc_axis_cal.get(ch_num, {})
            min_pwm = cal.get(f"RC{ch_num}_MIN")
            trim_pwm = cal.get(f"RC{ch_num}_TRIM")
            max_pwm = cal.get(f"RC{ch_num}_MAX")

            min_pwm = float(min_pwm) if valid_number(min_pwm) and 800 < float(min_pwm) < 1400 else 1000.0
            max_pwm = float(max_pwm) if valid_number(max_pwm) and 1600 < float(max_pwm) < 2200 else 2000.0

            if not (min_pwm + 100 < max_pwm):
                min_pwm, max_pwm = 1000.0, 2000.0

            if valid_number(trim_pwm) and min_pwm + 100 < float(trim_pwm) < max_pwm - 100:
                center_pwm = float(trim_pwm)
            else:
                center_pwm = (min_pwm + max_pwm) / 2.0

            return min_pwm, center_pwm, max_pwm

        def control_snapshot(ch_num, pwm=None):
            value = curr_control_pwm.get(ch_num) if pwm is None else pwm
            if not valid_number(value):
                return None

            value = float(value)
            if not 800 < value < 2200:
                return None

            meta = CONTROL_AXIS_META[ch_num]
            min_pwm, center_pwm, max_pwm = control_calibration(ch_num)
            delta = value - center_pwm

            if CONTROL_AXIS_REVERSED.get(ch_num, False):
                delta = -delta

            if abs(delta) <= CONTROL_CENTER_DEADBAND_US:
                side = "CENTER"
                pct = 0.0
                label = "Центр 0%"
                short_label = "0%"
            elif delta > 0:
                span = max(1.0, max_pwm - center_pwm)
                pct = min(100.0, max(0.0, abs(delta) / span * 100.0))
                side = meta["positive"]
                label = f"{meta['positiveLabel']} {pct:.0f}%"
                short_label = f"{meta['positiveLabel'].split()[0]} {pct:.0f}%"
            else:
                span = max(1.0, center_pwm - min_pwm)
                pct = min(100.0, max(0.0, abs(delta) / span * 100.0))
                side = meta["negative"]
                label = f"{meta['negativeLabel']} {pct:.0f}%"
                short_label = f"{meta['negativeLabel'].split()[0]} {pct:.0f}%"

            return {
                "channel": ch_num,
                "axis": meta["key"],
                "name": meta["name"],
                "pwm": int(round(value)),
                "direction": side,
                "percent": round(pct, 1),
                "label": label,
                "shortLabel": short_label,
                "centerPwm": int(round(center_pwm)),
                "minPwm": int(round(min_pwm)),
                "maxPwm": int(round(max_pwm)),
            }

        def controls_snapshot():
            return {
                CONTROL_AXIS_META[ch]["key"]: control_snapshot(ch)
                for ch in range(1, 5)
            }

        # V19 compatibility wrappers.
        def throttle_calibration():
            return control_calibration(3)

        def throttle_snapshot(pwm=None):
            return control_snapshot(3, pwm)

        def is_serious_system_text(text):
            t = str(text or "").lower()
            serious_patterns = (
                "potential thrust loss",
                "crash:",
                "crash ",
                "failsafe",
                "ekf variance",
                "ekf3 imu0 stopped aiding",
                "smart rtl failed",
                "smart rtl deactivated",
                "motor emergency",
            )
            return any(p in t for p in serious_patterns)

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
                    "distValue": round(curr_dist, 1) if curr_dist >= 0 else None,
                    "azimuth": round(curr_azimuth, 1) if curr_azimuth is not None else None,
                    "positionAzimuth": round(curr_position_azimuth, 1) if curr_position_azimuth is not None else None,
                    "nedNorth": round(curr_ned_north, 3) if curr_ned_north is not None else None,
                    "nedEast": round(curr_ned_east, 3) if curr_ned_east is not None else None,
                    "nedDown": round(curr_ned_down, 3) if curr_ned_down is not None else None,
                    "groundspeed": round(curr_groundspeed, 3) if curr_groundspeed is not None else None,
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
                    "attitude": attitude_snapshot(),
                    "rpmAnalysis": rpm_analysis_snapshot(),
                    "verticalSpeedDown": round(curr_vertical_speed_down, 2) if valid_number(curr_vertical_speed_down) else None,
                    "throttle": throttle_snapshot(),
                    "controls": controls_snapshot(),
                    # Raw ArduPilot STATUSTEXT stays in "system_text".
                    # Our own calculated/synthetic events go to "analysis_text".
                    "system_text": (
                        text
                        if (not is_pilot_action and event_type in ("SYSTEM", "POTENTIAL_THRUST_LOSS"))
                        else ""
                    ),
                    "analysis_text": (
                        text
                        if (not is_pilot_action and event_type not in ("SYSTEM", "POTENTIAL_THRUST_LOSS"))
                        else ""
                    ),
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
                    "distValue": round(curr_dist, 1) if curr_dist >= 0 else None,
                    "azimuth": round(curr_azimuth, 1) if curr_azimuth is not None else None,
                    "positionAzimuth": round(curr_position_azimuth, 1) if curr_position_azimuth is not None else None,
                    "nedNorth": round(curr_ned_north, 3) if curr_ned_north is not None else None,
                    "nedEast": round(curr_ned_east, 3) if curr_ned_east is not None else None,
                    "nedDown": round(curr_ned_down, 3) if curr_ned_down is not None else None,
                    "groundspeed": round(curr_groundspeed, 3) if curr_groundspeed is not None else None,
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
                    "attitude": attitude_snapshot(),
                    "rpmAnalysis": rpm_analysis_snapshot(),
                    "verticalSpeedDown": round(curr_vertical_speed_down, 2) if valid_number(curr_vertical_speed_down) else None,
                    "throttle": throttle_snapshot(),
                    "controls": controls_snapshot(),
                    "system_text": "",
                    "analysis_text": "",
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

        def tx16_three_pos_label(switch_name, pwm):
            pos = three_position_switch(pwm)
            if pos is None:
                return None

            if switch_name == "SC":
                return {
                    1: "ВІД СЕБЕ — запобіжник",
                    2: "СЕРЕДНЄ — правий механізм",
                    3: "ДО СЕБЕ — лівий механізм",
                }[pos]

            if switch_name == "SD":
                return {
                    1: "ВІД СЕБЕ — запобіжник",
                    2: "СЕРЕДНЄ — запобіжник",
                    3: "ДО СЕБЕ — запобіжник знято",
                }[pos]

            return f"ПОЗИЦІЯ {pos}"

        def tx16_two_pos_active(pwm):
            if not valid_number(pwm):
                return None
            pwm = float(pwm)
            if not 800 < pwm < 2200:
                return None
            if pwm >= TX16_ACTIVE_PWM_MIN:
                return True
            if pwm <= TX16_INACTIVE_PWM_MAX:
                return False
            return None

        def tx16_state_text(name, pwm):
            if name in ("SC", "SD"):
                return tx16_three_pos_label(name, pwm)

            active = tx16_two_pos_active(pwm)
            if active is None:
                return None
            if name == "SF":
                return "АКТИВАТОР УВІМКНЕНО" if active else "активатор вимкнено"
            if name == "SH":
                return "EMERGENCY STOP УТРИМУЄТЬСЯ" if active else "Emergency STOP відпущено"
            return "активно" if active else "неактивно"

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
            nonlocal potential_thrust_loss_events
            nonlocal accel_calibration_active, accel_calibration_success
            nonlocal accel_calibration_requires_reboot
            nonlocal accel_calibration_start_ts, accel_calibration_end_ts
            nonlocal accel_calibration_events
            nonlocal attitude_critical_active, attitude_critical_peak

            txt_lower = full_txt.lower()

            if "no rangefinder" in txt_lower or "visp: no rangefinder" in txt_lower:
                rangefinder_failed_flag = True

            if "ekf variance" in txt_lower:
                ekf_variance_count += 1
                ekf_variance_events.append({
                    "timestamp": float(timestamp),
                    "mode": current_mode,
                    "text": full_txt,
                })

            if (
                "stopped aiding" in txt_lower
                and "ekf3 imu0" in txt_lower
            ):
                ekf_stopped_aiding_count += 1
                ekf_stopped_aiding_events.append({
                    "timestamp": float(timestamp),
                    "mode": current_mode,
                    "text": full_txt,
                })
            elif "stopped aiding" in txt_lower:
                # Keep legacy count for other stopped-aiding texts, but V15
                # optical-navigation logic below is specifically for EKF3 IMU0.
                ekf_stopped_aiding_count += 1

            if "mode change to loiter failed" in txt_lower and "requires position" in txt_lower:
                loiter_position_fail_count += 1

            if "using external nav data" in txt_lower:
                external_nav_recovery_count += 1

            if "smartrtl deactivated" in txt_lower and "bad position" in txt_lower:
                smart_rtl_bad_position_count += 1

            if "prearm: need position estimate" in txt_lower:
                prearm_position_count += 1

            # Detect accelerometer calibration sequence.
            # Typical ArduPilot messages include positioning instructions,
            # 'Calibration successful', and 'Accels calibrated requires reboot'.
            accel_markers = (
                "place vehicle level",
                "place vehicle on its left side",
                "place vehicle on its right side",
                "place vehicle nose down",
                "place vehicle nose up",
                "place vehicle on its back",
                "place vehicle on left side",
                "place vehicle on right side",
                "accelerometer calibration",
                "accel calibration",
                "calibration successful",
                "accels calibrated requires reboot",
                "accel calibrated requires reboot",
            )

            if any(marker in txt_lower for marker in accel_markers):
                if accel_calibration_start_ts is None:
                    accel_calibration_start_ts = timestamp

                accel_calibration_active = True

                # If a DISARMED ATT episode was opened just before the first
                # calibration instruction, it belongs to the calibration context.
                # Do not carry it forward as a flight-critical event.
                if not is_currently_armed:
                    attitude_critical_active = False
                    attitude_critical_peak = None

                accel_calibration_events.append({
                    "timestamp": timestamp,
                    "text": full_txt,
                })

                if "calibration successful" in txt_lower:
                    accel_calibration_success = True
                    accel_calibration_end_ts = timestamp

                if "accels calibrated requires reboot" in txt_lower:
                    accel_calibration_requires_reboot = True
                    accel_calibration_end_ts = timestamp

            # Potential Thrust Loss is a warning candidate, not automatic proof
            # of motor failure. Correlate it later with RPM / ATT / VIB.
            thrust_match = re.search(
                r"potential\s+thrust\s+loss\s*\(?\s*(\d+)\s*\)?",
                full_txt,
                flags=re.IGNORECASE,
            )
            thrust_motor = None
            if thrust_match:
                try:
                    thrust_motor = int(thrust_match.group(1))
                except (TypeError, ValueError):
                    thrust_motor = None

                potential_thrust_loss_events.append({
                    "timestamp": timestamp,
                    "mode": mode,
                    "motor": thrust_motor,
                    "text": full_txt,
                    "rpm": [
                        int(round(v)) if valid_number(v) else None
                        for v in esc_rpm_current
                    ],
                })

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
            # Potential Thrust Loss gets its own red/clickable event type.
            event_type = "POTENTIAL_THRUST_LOSS" if thrust_match else "SYSTEM"
            add_event(
                full_txt,
                timestamp,
                mode,
                bool(thrust_match or is_serious_system_text(full_txt)),
                False,
                event_type,
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
            "STATUSTEXT", "ESC_TELEMETRY_1_TO_4", "PARAM_VALUE",
            "SERVO_OUTPUT_RAW",
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
                        previous_mode = current_mode

                        if previous_mode != "Невідомо":
                            mode_transition_events.append({
                                "timestamp": float(current_timestamp),
                                "from": previous_mode,
                                "to": new_mode,
                            })

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
                            active_land_entry = {
                                "timestamp": current_timestamp,
                                "altitude": float(curr_alt) if valid_number(curr_alt) else None,
                                "modeBefore": None,
                            }
                            land_entries.append(active_land_entry)

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

                        # V18 — confirm Emergency STOP only when a valid SD+SH
                        # hold preceded DISARM. This distinguishes an operator
                        # Emergency STOP sequence from an ordinary DISARM.
                        if active_emergency_attempt is not None:
                            hold_duration = max(
                                0.0,
                                current_timestamp - active_emergency_attempt["startTimestamp"],
                            )
                            if hold_duration >= EMERGENCY_STOP_MIN_HOLD_SEC:
                                active_emergency_attempt["durationSec"] = round(hold_duration, 3)
                                active_emergency_attempt["thresholdReached"] = True
                                active_emergency_attempt["confirmedByDisarm"] = True
                                active_emergency_attempt["disarmTimestamp"] = current_timestamp
                                active_emergency_attempt["status"] = "CONFIRMED_DISARM"
                                emergency_stop_confirmed_count += 1
                                add_event(
                                    f"🛑 EMERGENCY STOP ПІДТВЕРДЖЕНО: "
                                    f"SH утримувався {hold_duration:.1f} с → DISARM",
                                    current_timestamp,
                                    current_mode,
                                    True,
                                    False,
                                    "EMERGENCY_STOP",
                                )

                        was_armed = False


            # PARAMETERS used for LAND diagnostics.
            elif msg_type == "PARAM_VALUE":
                try:
                    raw_param_id = getattr(msg, "param_id", "")
                    if isinstance(raw_param_id, bytes):
                        param_id = raw_param_id.decode("utf-8", errors="ignore")
                    else:
                        param_id = str(raw_param_id)
                    param_id = param_id.strip("\x00 ").upper()

                    value = getattr(msg, "param_value", None)

                    if param_id in land_params and valid_number(value):
                        land_params[param_id] = float(value)

                    if valid_number(value):
                        for ch_num in range(1, 5):
                            if param_id in rc_axis_cal[ch_num]:
                                rc_axis_cal[ch_num][param_id] = float(value)
                                break
                except Exception:
                    pass

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
                    curr_groundspeed = max(0.0, float(msg.groundspeed))
                    max_speed = max(
                        max_speed,
                        curr_groundspeed,
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

                    curr_ned_north = x
                    curr_ned_east = y
                    curr_ned_down = z

                    d_val = math.sqrt(x * x + y * y)

                    if 0.0 <= d_val <= 10000.0:
                        curr_dist = d_val
                        max_dist = max(
                            max_dist,
                            curr_dist,
                        )
                        if d_val >= 0.5:
                            curr_position_azimuth = (
                                math.degrees(math.atan2(y, x)) + 360.0
                            ) % 360.0

                    ned_alt = -z

                    vz = getattr(msg, "vz", None)
                    if valid_number(vz):
                        # LOCAL_POSITION_NED vz is positive downward in NED.
                        curr_vertical_speed_down = float(vz)

                    if current_mode == "LAND" and is_currently_armed:
                        land_samples.append({
                            "timestamp": current_timestamp,
                            "altitude": max(0.0, float(ned_alt)) if valid_number(ned_alt) else None,
                            "verticalSpeedDown": float(curr_vertical_speed_down) if valid_number(curr_vertical_speed_down) else None,
                            "roll": float(curr_roll) if valid_number(curr_roll) else None,
                            "pitch": float(curr_pitch) if valid_number(curr_pitch) else None,
                            "vibration": vibration_snapshot(),
                            "rpmAnalysis": rpm_analysis_snapshot(),
                            "rpms": [
                                int(round(v)) if valid_number(v) else None
                                for v in esc_rpm_current
                            ],
                            "curr": float(curr_amp) if valid_number(curr_amp) else None,
                            "volt": float(curr_voltage) if valid_number(curr_voltage) else None,
                        })

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

                # V20 — update all four primary control axes and their maximum deflection.
                for control_ch in range(1, 5):
                    control_val = channels.get(control_ch, 0)
                    if valid_number(control_val) and 800 < float(control_val) < 2200:
                        curr_control_pwm[control_ch] = int(round(float(control_val)))
                        state = control_snapshot(control_ch, curr_control_pwm[control_ch])
                        if state is not None and state.get("direction") != "CENTER":
                            meta = CONTROL_AXIS_META[control_ch]
                            bucket = (
                                "positive"
                                if state.get("direction") == meta["positive"]
                                else "negative"
                            )
                            control_extremes[control_ch][bucket] = max(
                                control_extremes[control_ch][bucket],
                                float(state.get("percent", 0.0)),
                            )

                # V19 compatibility for CH3.
                curr_ch3_pwm = curr_control_pwm.get(3)
                throttle_max_up_pct = control_extremes[3]["positive"]
                throttle_max_down_pct = control_extremes[3]["negative"]

                # Existing VTX logic remains unchanged.
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

                # Existing generic pilot-channel events, extended through CH12.
                for ch_num in range(5, 13):
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

                # ----------------------------------------------------
                # V18 — named TX16S switches: SC / SD / SF / SH
                # ----------------------------------------------------
                named_pwm = {
                    name: channels.get(ch_num, 0)
                    for name, ch_num in TX16_SWITCH_CHANNELS.items()
                }

                previous_named_state = dict(tx16_switch_state)

                for name, pwm in named_pwm.items():
                    if valid_number(pwm) and 800 < float(pwm) < 2200:
                        tx16_switch_pwm[name] = int(round(float(pwm)))
                        new_state = tx16_state_text(name, pwm)

                        if (
                            new_state is not None
                            and tx16_switch_state.get(name) is not None
                            and new_state != tx16_switch_state.get(name)
                        ):
                            add_event(
                                f"🎛️ {name}: {new_state} ({int(round(float(pwm)))} us)",
                                current_timestamp,
                                current_mode,
                                False,
                                True,
                                "TX16_SWITCH",
                            )

                        if new_state is not None:
                            tx16_switch_state[name] = new_state

                sc_pos = three_position_switch(named_pwm.get("SC", 0))
                sd_pos = three_position_switch(named_pwm.get("SD", 0))
                sf_active = tx16_two_pos_active(named_pwm.get("SF", 0))
                sh_active = tx16_two_pos_active(named_pwm.get("SH", 0))

                prev_sf_active = None
                prev_sh_active = None
                if previous_named_state.get("SF") is not None:
                    prev_sf_active = "УВІМКНЕНО" in previous_named_state["SF"]
                if previous_named_state.get("SH") is not None:
                    prev_sh_active = "УТРИМУЄТЬСЯ" in previous_named_state["SH"]

                # SC + SF: detect the operator command. Mechanical movement is
                # confirmed separately below by the SERVO_OUTPUT_RAW response.
                if sf_active is True and prev_sf_active is False:
                    if sc_pos in (2, 3):
                        mechanism = "RIGHT" if sc_pos == 2 else "LEFT"
                        mechanism_ua = "правий" if sc_pos == 2 else "лівий"
                        command = {
                            "timestamp": current_timestamp,
                            "scPosition": sc_pos,
                            "mechanism": mechanism,
                            "sfPwm": tx16_switch_pwm.get("SF"),
                            "scPwm": tx16_switch_pwm.get("SC"),
                            "confirmedByServo": False,
                            "servo": PAYLOAD_CONFIRM_SERVO,
                            "servoTimestamp": None,
                            "servoFrom": None,
                            "servoTo": None,
                        }
                        payload_commands.append(command)
                        pending_payload_command = command
                        add_event(
                            f"📦 Команда виконавчого механізму: {mechanism_ua}; "
                            f"SC={sc_pos}, SF=активовано",
                            current_timestamp,
                            current_mode,
                            False,
                            True,
                            "PAYLOAD_COMMAND",
                        )
                    elif sc_pos == 1:
                        add_event(
                            "🔒 SF активовано, але SC залишається на запобіжнику — "
                            "команда виконавчого механізму заблокована",
                            current_timestamp,
                            current_mode,
                            False,
                            True,
                            "PAYLOAD_BLOCKED",
                        )

                # SD + SH: Emergency STOP hold logic.
                if sh_active is True and prev_sh_active is False:
                    if sd_pos == 3:
                        attempt = {
                            "startTimestamp": current_timestamp,
                            "endTimestamp": None,
                            "durationSec": 0.0,
                            "thresholdReached": False,
                            "confirmedByDisarm": False,
                            "disarmTimestamp": None,
                            "status": "HOLDING",
                        }
                        emergency_stop_attempts.append(attempt)
                        active_emergency_attempt = attempt
                        add_event(
                            "🛑 Emergency STOP: SD до себе, SH активовано — "
                            "початок відліку утримання",
                            current_timestamp,
                            current_mode,
                            False,
                            True,
                            "EMERGENCY_STOP_HOLD",
                        )
                    else:
                        add_event(
                            "🔒 SH активовано, але SD на запобіжнику — "
                            "Emergency STOP заблокований",
                            current_timestamp,
                            current_mode,
                            False,
                            True,
                            "EMERGENCY_STOP_BLOCKED",
                        )

                # While SH is held, emit one exact event when the minimum
                # 8-second condition is reached.
                if (
                    active_emergency_attempt is not None
                    and sh_active is True
                    and sd_pos == 3
                    and not active_emergency_attempt.get("thresholdReached")
                ):
                    held = current_timestamp - active_emergency_attempt["startTimestamp"]
                    if held >= EMERGENCY_STOP_MIN_HOLD_SEC:
                        active_emergency_attempt["thresholdReached"] = True
                        active_emergency_attempt["durationSec"] = round(held, 3)
                        active_emergency_attempt["status"] = "THRESHOLD_REACHED"
                        add_event(
                            f"🛑 Emergency STOP: SH утримується {held:.1f} с — "
                            f"мінімальний поріг {EMERGENCY_STOP_MIN_HOLD_SEC:.0f} с досягнуто",
                            current_timestamp,
                            current_mode,
                            True,
                            False,
                            "EMERGENCY_STOP",
                        )

                # SH release or SD returned to safety ends an active attempt.
                stop_holding = (
                    active_emergency_attempt is not None
                    and (sh_active is False or sd_pos != 3)
                )
                if stop_holding:
                    held = max(
                        0.0,
                        current_timestamp - active_emergency_attempt["startTimestamp"],
                    )
                    active_emergency_attempt["endTimestamp"] = current_timestamp
                    active_emergency_attempt["durationSec"] = round(held, 3)

                    if active_emergency_attempt.get("confirmedByDisarm"):
                        active_emergency_attempt["status"] = "CONFIRMED_DISARM"
                    elif held >= EMERGENCY_STOP_MIN_HOLD_SEC:
                        active_emergency_attempt["thresholdReached"] = True
                        active_emergency_attempt["status"] = "COMPLETED_NO_DISARM"
                        add_event(
                            f"🛑 Emergency STOP: SH відпущено після {held:.1f} с; "
                            "умова витримки виконана, але DISARM у цей момент не підтверджено",
                            current_timestamp,
                            current_mode,
                            True,
                            False,
                            "EMERGENCY_STOP",
                        )
                    else:
                        active_emergency_attempt["status"] = "CANCELLED_SHORT_HOLD"
                        add_event(
                            f"ℹ️ Emergency STOP не завершено: SH утримувався "
                            f"лише {held:.1f} с (< {EMERGENCY_STOP_MIN_HOLD_SEC:.0f} с)",
                            current_timestamp,
                            current_mode,
                            False,
                            False,
                            "EMERGENCY_STOP_ATTEMPT",
                        )

                    active_emergency_attempt = None

            # SERVO OUTPUTS — V18 correlation with the configured actuator output.
            elif msg_type == "SERVO_OUTPUT_RAW":
                for servo_num in range(1, 17):
                    val = getattr(msg, f"servo{servo_num}_raw", None)
                    if not valid_number(val):
                        continue
                    val = int(round(float(val)))
                    if not 500 <= val <= 2500:
                        continue

                    prev = servo_output_last.get(servo_num)
                    servo_output_last[servo_num] = val

                    if prev is None or abs(val - prev) < PAYLOAD_SERVO_DELTA_US:
                        continue

                    out_event = {
                        "timestamp": current_timestamp,
                        "servo": servo_num,
                        "from": prev,
                        "to": val,
                    }
                    servo_output_events.append(out_event)

                    if servo_num == PAYLOAD_CONFIRM_SERVO:
                        add_event(
                            f"⚙️ SERVO{servo_num}: {prev} → {val} us",
                            current_timestamp,
                            current_mode,
                            False,
                            False,
                            "SERVO_OUTPUT",
                        )

                        if pending_payload_command is not None:
                            dt = current_timestamp - pending_payload_command["timestamp"]
                            if 0.0 <= dt <= PAYLOAD_SERVO_CORRELATION_SEC:
                                pending_payload_command["confirmedByServo"] = True
                                pending_payload_command["servoTimestamp"] = current_timestamp
                                pending_payload_command["servoFrom"] = prev
                                pending_payload_command["servoTo"] = val
                                add_event(
                                    f"✅ Команду виконавчого механізму підтверджено виходом "
                                    f"SERVO{servo_num}: {prev} → {val} us",
                                    current_timestamp,
                                    current_mode,
                                    False,
                                    False,
                                    "PAYLOAD_CONFIRM",
                                )
                                pending_payload_command = None

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
                    curr_roll = math.degrees(msg.roll)
                    abs_roll = abs(curr_roll)
                    if abs_roll > max_roll:
                        max_roll = abs_roll
                        max_roll_timestamp = current_timestamp

                if valid_number(msg.pitch):
                    curr_pitch = math.degrees(msg.pitch)
                    abs_pitch = abs(curr_pitch)
                    if abs_pitch > max_pitch:
                        max_pitch = abs_pitch
                        max_pitch_timestamp = current_timestamp

                if valid_number(msg.yaw):
                    curr_yaw = math.degrees(msg.yaw) % 360.0

                # Critical tilt episode. Roll/Pitch >= 35 deg on ANY tilt axis.
                # Yaw is shown in the dropdown but does not trigger tilt criticality.
                if curr_roll is not None and curr_pitch is not None:
                    crit = (
                        abs(curr_roll) >= ATTITUDE_CRITICAL_THRESHOLD_DEG
                        or abs(curr_pitch) >= ATTITUDE_CRITICAL_THRESHOLD_DEG
                    )

                    # During accelerometer calibration the vehicle is intentionally
                    # placed on its sides / nose / back. Large Roll/Pitch is normal
                    # in this context and must NOT become a flight-critical ATT event.
                    if (
                        GROUND_CAL_ATT_IGNORE_DURING_ACCEL_CAL
                        and not is_currently_armed
                        and accel_calibration_active
                    ):
                        crit = False

                    peak = max(abs(curr_roll), abs(curr_pitch))

                    if crit:
                        if (not attitude_critical_active) or attitude_critical_peak is None:
                            attitude_critical_active = True
                            attitude_critical_peak = {
                                "timestamp": current_timestamp,
                                "roll": curr_roll,
                                "pitch": curr_pitch,
                                "yaw": curr_yaw,
                                "peak": peak,
                            }
                            # Add one exact timeline row at the start of each episode.
                            add_event(
                                f"Критичний кут нахилу: Roll {curr_roll:.1f}°, Pitch {curr_pitch:.1f}°",
                                current_timestamp,
                                current_mode,
                                is_error=True,
                                event_type="ATTITUDE_CRITICAL",
                            )
                        elif peak > attitude_critical_peak["peak"]:
                            attitude_critical_peak.update({
                                "timestamp": current_timestamp,
                                "roll": curr_roll,
                                "pitch": curr_pitch,
                                "yaw": curr_yaw,
                                "peak": peak,
                            })
                    elif attitude_critical_active:
                        if attitude_critical_peak is not None:
                            attitude_critical_events.append(dict(attitude_critical_peak))
                        attitude_critical_active = False
                        attitude_critical_peak = None

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

                # Detect sustained RPM asymmetry inside the real diagonal pairs:
                # M1<->M2 and M3<->M4.
                if (
                    is_currently_armed
                    and valid_number(curr_alt)
                    and float(curr_alt) >= FLIGHT_TAKEOFF_ALT_M
                ):
                    rpm_diag = rpm_analysis_snapshot()

                    for pair in rpm_diag.get("pairs", []):
                        pair_name = pair["pair"]
                        diff = pair.get("differencePct")
                        critical_now = bool(pair.get("isCritical"))

                        if critical_now and diff is not None:
                            candidate = rpm_pair_candidate.get(pair_name)

                            if candidate is None:
                                candidate = {
                                    "start": current_timestamp,
                                    "peakTimestamp": current_timestamp,
                                    "peakDifferencePct": diff,
                                    "lowerMotor": pair.get("lowerMotor"),
                                    "higherMotor": pair.get("higherMotor"),
                                    "rpmA": pair.get("rpmA"),
                                    "rpmB": pair.get("rpmB"),
                                }
                                rpm_pair_candidate[pair_name] = candidate
                            elif diff > candidate.get("peakDifferencePct", 0):
                                candidate.update({
                                    "peakTimestamp": current_timestamp,
                                    "peakDifferencePct": diff,
                                    "lowerMotor": pair.get("lowerMotor"),
                                    "higherMotor": pair.get("higherMotor"),
                                    "rpmA": pair.get("rpmA"),
                                    "rpmB": pair.get("rpmB"),
                                })

                            duration = current_timestamp - candidate["start"]

                            if (
                                duration >= RPM_CRITICAL_PERSIST_SEC
                                and not rpm_pair_active.get(pair_name, False)
                            ):
                                rpm_pair_active[pair_name] = True

                                event = {
                                    "timestamp": current_timestamp,
                                    "startTimestamp": candidate["start"],
                                    "mode": current_mode,
                                    "pair": pair_name,
                                    "differencePct": float(candidate["peakDifferencePct"]),
                                    "lowerMotor": candidate.get("lowerMotor"),
                                    "higherMotor": candidate.get("higherMotor"),
                                    "rpmA": candidate.get("rpmA"),
                                    "rpmB": candidate.get("rpmB"),
                                    "rpms": [
                                        int(round(v)) if valid_number(v) else None
                                        for v in esc_rpm_current
                                    ],
                                }
                                rpm_drop_events.append(event)

                                low_motor = event.get("lowerMotor")
                                add_event(
                                    f"🚨 Аномальна різниця RPM у діагоналі M{pair_name}: "
                                    f"{event['differencePct']:.1f}%"
                                    + (
                                        f"; нижчий RPM у Motor {low_motor}"
                                        if low_motor else ""
                                    ),
                                    current_timestamp,
                                    current_mode,
                                    True,
                                    False,
                                    "RPM_DIAGONAL_CRITICAL",
                                )

                        else:
                            if diff is None or diff < RPM_DIAGONAL_RECOVERY_PCT:
                                rpm_pair_candidate[pair_name] = None
                                rpm_pair_active[pair_name] = False

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

        # Close an attitude episode that is still active at EOF.
        if attitude_critical_active and attitude_critical_peak is not None:
            attitude_critical_events.append(dict(attitude_critical_peak))
            attitude_critical_active = False
            attitude_critical_peak = None


        # ====================================================
        # LAND ANALYSIS
        # ====================================================
        land_analysis = {
            "available": False,
            "classification": "NO_DATA",
            "entryTimestamp": None,
            "entryAltitude": None,
            "maxDescentRate": None,
            "maxDescentTimestamp": None,
            "meanDescentRate": None,
            "expectedHighSpeed": None,
            "expectedLowSpeed": None,
            "lowAltitudeThreshold": None,
            "matchesConfiguredSpeed": False,
            "abnormalDescent": False,
            "possibleFreeFall": False,
            "notes": [],
        }

        if land_entries and len(land_samples) >= LAND_ANALYSIS_MIN_SAMPLES:
            entry = land_entries[0]
            entry_ts = entry.get("timestamp")
            relevant = [
                s for s in land_samples
                if entry_ts is None or s.get("timestamp", 0) >= entry_ts
            ]

            valid_vs = [
                s for s in relevant
                if valid_number(s.get("verticalSpeedDown"))
                and valid_number(s.get("altitude"))
            ]

            if valid_vs:
                land_analysis["available"] = True
                land_analysis["entryTimestamp"] = entry_ts
                land_analysis["entryAltitude"] = (
                    round(float(entry.get("altitude")), 1)
                    if valid_number(entry.get("altitude"))
                    else round(float(valid_vs[0]["altitude"]), 1)
                )

                max_s = max(valid_vs, key=lambda s: float(s["verticalSpeedDown"]))
                max_vs = max(0.0, float(max_s["verticalSpeedDown"]))
                land_analysis["maxDescentRate"] = round(max_vs, 2)
                land_analysis["maxDescentTimestamp"] = max_s["timestamp"]

                positive_rates = [
                    max(0.0, float(s["verticalSpeedDown"]))
                    for s in valid_vs
                    if float(s["verticalSpeedDown"]) > 0
                ]
                if positive_rates:
                    land_analysis["meanDescentRate"] = round(
                        sum(positive_rates) / len(positive_rates), 2
                    )

                land_speed = land_params.get("LAND_SPEED")
                land_speed_high = land_params.get("LAND_SPEED_HIGH")
                land_alt_low = land_params.get("LAND_ALT_LOW")

                low_speed_mps = (
                    float(land_speed) / 100.0
                    if valid_number(land_speed) and float(land_speed) > 0
                    else None
                )
                high_speed_mps = (
                    float(land_speed_high) / 100.0
                    if valid_number(land_speed_high) and float(land_speed_high) > 0
                    else None
                )
                low_alt_m = (
                    float(land_alt_low) / 100.0
                    if valid_number(land_alt_low) and float(land_alt_low) >= 0
                    else None
                )

                land_analysis["expectedLowSpeed"] = (
                    round(low_speed_mps, 2) if low_speed_mps is not None else None
                )
                land_analysis["expectedHighSpeed"] = (
                    round(high_speed_mps, 2) if high_speed_mps is not None else None
                )
                land_analysis["lowAltitudeThreshold"] = (
                    round(low_alt_m, 1) if low_alt_m is not None else None
                )

                # Compare actual descent to the configured phase speed sample-by-sample.
                comparisons = []
                for s in valid_vs:
                    alt = float(s["altitude"])
                    actual = max(0.0, float(s["verticalSpeedDown"]))
                    expected = None

                    if (
                        high_speed_mps is not None
                        and low_alt_m is not None
                        and alt > low_alt_m
                    ):
                        expected = high_speed_mps
                    elif low_speed_mps is not None:
                        expected = low_speed_mps
                    elif high_speed_mps is not None:
                        expected = high_speed_mps

                    if expected is not None and expected > 0:
                        comparisons.append((s, actual, expected))

                if comparisons:
                    within = [
                        abs(actual - expected) <= LAND_SPEED_TOLERANCE_MPS
                        for _, actual, expected in comparisons
                    ]
                    match_fraction = sum(within) / len(within)
                    land_analysis["matchesConfiguredSpeed"] = match_fraction >= 0.60

                    abnormal = [
                        (s, actual, expected)
                        for s, actual, expected in comparisons
                        if (
                            actual >= LAND_SPEED_MIN_ABNORMAL_MPS
                            and actual > expected * LAND_SPEED_RATIO_WARNING
                            and (actual - expected) > LAND_SPEED_TOLERANCE_MPS
                        )
                    ]

                    if abnormal:
                        land_analysis["abnormalDescent"] = True
                        worst = max(
                            abnormal,
                            key=lambda x: x[1] - x[2],
                        )
                        land_analysis["worstUnexpectedTimestamp"] = worst[0]["timestamp"]
                        land_analysis["worstUnexpectedActual"] = round(worst[1], 2)
                        land_analysis["worstUnexpectedExpected"] = round(worst[2], 2)

                # Approximate free-fall-like acceleration check:
                # rapidly increasing downward speed is different from a controller
                # settling onto a near-constant commanded descent speed.
                accel_peaks = []
                prev = None
                for s in valid_vs:
                    if prev is not None:
                        dt = s["timestamp"] - prev["timestamp"]
                        if dt > 0.03:
                            dv = (
                                float(s["verticalSpeedDown"])
                                - float(prev["verticalSpeedDown"])
                            )
                            accel_peaks.append(dv / dt)
                    prev = s

                max_down_accel = max(accel_peaks) if accel_peaks else 0.0
                land_analysis["maxDownAcceleration"] = round(max_down_accel, 2)

                if (
                    land_analysis["abnormalDescent"]
                    and max_down_accel >= LAND_FREEFALL_ACCEL_MPS2
                ):
                    land_analysis["possibleFreeFall"] = True

                if land_analysis["matchesConfiguredSpeed"]:
                    land_analysis["classification"] = "CONTROLLED_FAST_LAND"
                    land_analysis["notes"].append(
                        "Фактична швидкість зниження переважно відповідає "
                        "параметрам LAND."
                    )
                elif land_analysis["abnormalDescent"]:
                    land_analysis["classification"] = "ABNORMAL_LAND_DESCENT"
                    land_analysis["notes"].append(
                        "Фактична швидкість зниження суттєво перевищує "
                        "розрахункову швидкість LAND."
                    )
                else:
                    land_analysis["classification"] = "LAND_UNCERTAIN"

                # Add one exact timeline marker for the most informative LAND point.
                marker_ts = (
                    land_analysis.get("worstUnexpectedTimestamp")
                    if land_analysis["abnormalDescent"]
                    else land_analysis.get("maxDescentTimestamp")
                )

                if marker_ts is not None:
                    nearest = min(
                        valid_vs,
                        key=lambda s: abs(s["timestamp"] - marker_ts),
                    )
                    marker_actual = max(
                        0.0,
                        float(nearest.get("verticalSpeedDown") or 0.0),
                    )
                    marker_alt = float(nearest.get("altitude") or 0.0)

                    if land_analysis["abnormalDescent"]:
                        marker_text = (
                            f"🚨 Аномальне зниження в LAND: "
                            f"{marker_actual:.2f} м/с вниз на висоті "
                            f"{marker_alt:.1f} м"
                        )
                        marker_type = "LAND_DESCENT_ABNORMAL"
                        marker_error = True
                    else:
                        marker_text = (
                            f"🛬 LAND: максимальна швидкість зниження "
                            f"{marker_actual:.2f} м/с на висоті "
                            f"{marker_alt:.1f} м"
                        )
                        marker_type = "LAND_DESCENT_CONTROLLED"
                        marker_error = False

                    # Temporarily restore sample values so the row contains
                    # telemetry close to the derived event timestamp.
                    old_alt = curr_alt
                    old_vz = curr_vertical_speed_down
                    curr_alt = marker_alt
                    curr_vertical_speed_down = marker_actual
                    add_event(
                        marker_text,
                        marker_ts,
                        "LAND",
                        is_error=marker_error,
                        event_type=marker_type,
                    )
                    curr_alt = old_alt
                    curr_vertical_speed_down = old_vz

        # Detect all ARM->DISARM sessions in this TLOG.
        flight_sessions = analyze_flight_sessions(raw_timeline, current_timestamp)
        first_flight_arm_timestamp = flight_sessions[0]["armTimestamp"] if flight_sessions else arm_timestamp

        # Add derived Timeline markers for accelerometer calibration.
        # IMPORTANT: clone the nearest already-valid timeline row so every field
        # expected by the serializer is present. This avoids HTTP 500/KeyError.
        if accel_calibration_events:
            existing_rows = [
                row for row in raw_timeline
                if row.get("timestamp") is not None
            ]

            for idx, cal_ev in enumerate(accel_calibration_events):
                if existing_rows:
                    nearest = min(
                        existing_rows,
                        key=lambda row: abs(
                            float(row.get("timestamp", 0.0))
                            - float(cal_ev["timestamp"])
                        ),
                    )
                    marker = dict(nearest)
                else:
                    marker = {
                        "timestamp": cal_ev["timestamp"],
                        "mode": current_mode,
                        "alt": "0.0 м",
                        "dist": "0.0 м",
                        "distValue": 0.0,
                        "azimuth": None,
                        "positionAzimuth": None,
                        "vtxBand": None,
                        "vtxChannel": None,
                        "videoFreq": None,
                        "volt": None,
                        "curr": None,
                        "rssi": None,
                        "dbm": None,
                        "temp": None,
                        "esc": esc_snapshot(),
                        "vibration": vibration_snapshot(),
                        "attitude": attitude_snapshot(),
                        "rpmAnalysis": rpm_analysis_snapshot(),
                        "verticalSpeedDown": None,
                    }

                marker.update({
                    "timestamp": cal_ev["timestamp"],
                    "system_text": "",
                    "analysis_text": (
                        "🧭 Проводилось калібрування акселерометра"
                        if idx == 0
                        else f"🧭 Калібрування акселерометра: {cal_ev['text']}"
                    ),
                    "pilot_text": "",
                    "isError": False,
                    "eventType": "ACCEL_CALIBRATION",
                })

                raw_timeline.append(marker)

            raw_timeline.sort(key=lambda row: row.get("timestamp", 0.0))

        # Estimate antenna pointing from NED position azimuth + dBm.
        antenna_analysis = analyze_antenna_direction(raw_timeline, first_flight_arm_timestamp)

        # Per-flight antenna estimate specifically for 2D map.
        antenna_map_analysis_by_flight = {}

        flight_numbers_for_map = sorted({
            int(ev.get("flightNumber"))
            for ev in raw_timeline
            if ev.get("flightNumber") is not None
        })

        for flight_no in flight_numbers_for_map:
            antenna_map_analysis_by_flight[str(flight_no)] = (
                estimate_map_antenna_direction(
                    raw_timeline,
                    flight_no,
                )
            )

        # Timeline
        # 00:00.000 = момент ARM.
        # Події до ARM показуються з мінусом, наприклад -00:32.983.
        timeline = []
        base_t = first_flight_arm_timestamp or arm_timestamp or first_timestamp or 0

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
                    "positionAzimuth": ev.get("positionAzimuth"),
                    "nedNorth": ev.get("nedNorth"),
                    "nedEast": ev.get("nedEast"),
                    "nedDown": ev.get("nedDown"),
                    "groundspeed": ev.get("groundspeed"),
                    "antennaSector": ev.get("antennaSector"),
                    "vtxBand": ev.get("vtxBand"),
                    "vtxChannel": ev.get("vtxChannel"),
                    "videoFreq": ev.get("videoFreq"),
                    "volt": ev.get("volt"),
                    "curr": ev.get("curr"),
                    "rssi": ev.get("rssi"),
                    "dbm": ev.get("dbm"),
                    "radioState": ev.get("radioState"),
                    "radioStateText": ev.get("radioStateText"),
                    "temp": ev.get("temp"),
                    "esc": ev.get("esc", []),
                    "vibration": ev.get("vibration"),
                    "attitude": ev.get("attitude"),
                    "rpmAnalysis": ev.get("rpmAnalysis"),
                    "verticalSpeedDown": ev.get("verticalSpeedDown"),
                    "throttle": ev.get("throttle"),
                    "controls": ev.get("controls"),
                    "flightNumber": ev.get("flightNumber"),
                    "takeoffEpisodeNumber": ev.get("takeoffEpisodeNumber"),
                    "systemText": ev.get("system_text", ""),
                    "analysisText": ev.get("analysis_text", ""),
                    "pilotText": ev.get("pilot_text", ""),
                    "eventType": ev.get("eventType", "SYSTEM"),
                    "isError": bool(ev.get("isError", False)),
                }
            )

        # V8: explicit AI wording guard for Heading-only antenna inference.
        heading_fallback_ai_note = None

        if antenna_analysis.get("method") == "HEADING_FALLBACK":
            heading_fallback_ai_note = (
                "⚠️ <b>АС / Heading fallback:</b> напрямок АС оцінено лише за "
                "Heading БПЛА + dBm. Це не геометричний позиційний азимут, тому "
                "вихід за сектор не вважається підтвердженим. Дані можуть свідчити "
                "лише про можливу радіокореляцію."
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

        if heading_fallback_ai_note:
            ai_alerts.append(heading_fallback_ai_note)

        # High-level context has priority over flight-only heuristics.
        ground_session = not ever_armed
        accelerometer_calibration_session = bool(
            ground_session and accel_calibration_events
        )

        # V18 — TX16S operator-action summary.
        if payload_commands:
            confirmed_payload = sum(1 for x in payload_commands if x.get("confirmedByServo"))
            first_payload_time = format_timeline_time(payload_commands[0]["timestamp"], base_t)
            ai_alerts.append(
                f'<span class="ai-jump" data-jump-time="{first_payload_time}">'
                f"📦 <b>Виконавчий механізм:</b> зафіксовано {len(payload_commands)} "
                f"команд(и) SC+SF; підтверджено зміною SERVO{PAYLOAD_CONFIRM_SERVO}: "
                f"{confirmed_payload}. Натисніть, щоб перейти до першої команди.</span>"
            )

        if emergency_stop_attempts:
            confirmed_emergency = [
                x for x in emergency_stop_attempts if x.get("confirmedByDisarm")
            ]
            threshold_emergency = [
                x for x in emergency_stop_attempts if x.get("thresholdReached")
            ]
            short_emergency = [
                x for x in emergency_stop_attempts
                if x.get("status") == "CANCELLED_SHORT_HOLD"
            ]
            first_estop_time = format_timeline_time(
                emergency_stop_attempts[0]["startTimestamp"],
                base_t,
            )

            if confirmed_emergency:
                is_critical = True
                ai_alerts.append(
                    f'<span class="ai-jump" data-jump-time="{first_estop_time}">'
                    f"🛑 <b>Emergency STOP:</b> підтверджено {len(confirmed_emergency)} "
                    f"випадок(и): SD знято із запобіжника + SH ≥ "
                    f"{EMERGENCY_STOP_MIN_HOLD_SEC:.0f} с + DISARM.</span>"
                )
            elif threshold_emergency:
                ai_alerts.append(
                    f'<span class="ai-jump" data-jump-time="{first_estop_time}">'
                    f"⚠️ <b>Emergency STOP:</b> поріг утримання SH ≥ "
                    f"{EMERGENCY_STOP_MIN_HOLD_SEC:.0f} с досягався, але DISARM "
                    f"не підтверджено. Натисніть для перевірки Timeline.</span>"
                )
            elif short_emergency:
                longest = max(float(x.get("durationSec") or 0.0) for x in short_emergency)
                ai_alerts.append(
                    f'<span class="ai-jump" data-jump-time="{first_estop_time}">'
                    f"ℹ️ <b>Emergency STOP:</b> були короткі спроби; "
                    f"максимальне утримання SH {longest:.1f} с, "
                    f"менше порога {EMERGENCY_STOP_MIN_HOLD_SEC:.0f} с.</span>"
                )

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


        # Flight-session structure in a TLOG that may contain several flights.
        if flight_sessions:
            count=len(flight_sessions)
            first_time=format_timeline_time(flight_sessions[0]["armTimestamp"],base_t)
            ai_alerts.append(
                f'<span class="ai-jump" data-jump-time="{first_time}">'
                f"🛫 <b>Структура TLOG:</b> виявлено {count} "
                f"{'польотну сесію' if count==1 else 'польотні сесії'}. "
                "Новий політ рахується тільки після нового ARM, який іде після DISARM. "
                "Посадка і повторний зліт без DISARM залишаються одним польотом. "
                "Натисніть, щоб перейти до першого ARM.</span>"
            )
            for s in flight_sessions:
                arm_t=format_timeline_time(s["armTimestamp"],base_t)
                dur=float(s.get("duration") or 0); mm=int(dur//60); ss=dur-mm*60
                if s.get("endedArmed"):
                    icon="🚨"; status="TLOG завершився при ARMED; DISARM не зафіксовано"
                else:
                    icon="✅"; status="DISARM "+format_timeline_time(s["disarmTimestamp"],base_t)
                n=s.get("takeoffEpisodeCount",0)
                if n==0: eps="підтверджений зліт ≥2 м не визначено"
                elif n==1: eps="1 злітно-посадковий епізод"
                else: eps=f"{n} злітно-посадкові епізоди; був повторний зліт без DISARM"
                minv=s.get("minVoltage")
                minvt=f"; MIN V {minv:.2f} V" if minv is not None else ""
                ai_alerts.append(
                    f'<span class="ai-jump" data-jump-time="{arm_t}">'
                    f"{icon} <b>Політ №{s['number']}:</b> ARM {arm_t}; {status}. "
                    f"Тривалість {mm:02d}:{ss:04.1f}; {eps}. "
                    f"MAX ALT {s.get('maxAltitude',0):.1f} м; MAX струм {s.get('maxCurrent',0):.1f} A{minvt}. "
                    "Натисніть, щоб перейти до початку польоту.</span>"
                )
                for ep in s.get("takeoffEpisodes",[]):
                    ep_t=format_timeline_time(ep["startTimestamp"],base_t)
                    label="Повторний зліт" if ep["number"]>1 else "Зліт"
                    endtxt=""
                    if ep.get("endTimestamp") is not None:
                        et=format_timeline_time(ep["endTimestamp"],base_t)
                        endtxt=f"; посадка {et} без DISARM" if ep.get("endReason")=="landed" else f"; завершення {et}"
                    ai_alerts.append(
                        f'<span class="ai-jump" data-jump-time="{ep_t}">'
                        f"↗ <b>{label} — політ №{s['number']}, епізод {ep['number']}:</b> "
                        f"{ep_t}{endtxt}; MAX ALT епізоду {float(ep.get('maxAltitude') or 0):.1f} м. "
                        "Натисніть, щоб перейти до рядка Timeline.</span>"
                    )
        # Ground / accelerometer calibration context.

        if accel_calibration_events:
            cal_start = (
                format_timeline_time(accel_calibration_start_ts, base_t)
                if accel_calibration_start_ts is not None
                else "—"
            )
            cal_end = (
                format_timeline_time(accel_calibration_end_ts, base_t)
                if accel_calibration_end_ts is not None
                else "—"
            )

            if accel_calibration_success:
                status_text = "калібрування завершилось успішно"
            else:
                status_text = "процедуру калібрування зафіксовано, але повідомлення про успішне завершення не знайдено"

            reboot_text = (
                " Після калібрування ArduPilot повідомив, що потрібне перезавантаження."
                if accel_calibration_requires_reboot
                else ""
            )

            ai_alerts.append(
                f'<span class="ai-jump" data-jump-time="{cal_start}">'
                "🧭 <b>Проводилось калібрування акселерометра.</b> "
                f"Початок процедури: {cal_start}; завершення: {cal_end}. "
                f"{status_text}.{reboot_text} "
                "Під час калібрування БПЛА штатно встановлюють рівно, на лівий/правий бік, "
                "носом вниз/вгору та на спину. Тому великі Roll/Pitch у цей період "
                "є очікуваними й не класифікуються як аварійний нахил. "
                "Натисніть, щоб перейти до початку калібрування."
                "</span>"
            )

        if ground_session:
            first_time = format_timeline_time(
                first_timestamp if first_timestamp is not None else 0,
                base_t,
            )
            ai_alerts.append(
                f'<span class="ai-jump" data-jump-time="{first_time}">'
                "ℹ️ <b>Наземна сесія:</b> ARM протягом TLOG не зафіксовано. "
                "Тому висота, ATT, RPM, VIB та радіоподії оцінюються в наземному контексті, "
                "а не як повноцінний політ."
                "</span>"
            )

        # LAND behavior analysis.
        if land_analysis.get("available") and not accelerometer_calibration_session:
            land_time = format_timeline_time(
                land_analysis.get("maxDescentTimestamp"),
                base_t,
            )
            entry_alt = land_analysis.get("entryAltitude")
            max_rate = land_analysis.get("maxDescentRate")
            high_speed = land_analysis.get("expectedHighSpeed")
            low_speed = land_analysis.get("expectedLowSpeed")
            low_alt = land_analysis.get("lowAltitudeThreshold")

            params_text = []
            if high_speed is not None:
                params_text.append(f"LAND_SPEED_HIGH ≈ {high_speed:.2f} м/с")
            if low_speed is not None:
                params_text.append(f"LAND_SPEED ≈ {low_speed:.2f} м/с")
            if low_alt is not None:
                params_text.append(f"LAND_ALT_LOW ≈ {low_alt:.1f} м")
            params_joined = "; ".join(params_text) if params_text else "параметри LAND у TLOG не знайдені"

            if land_analysis.get("classification") == "CONTROLLED_FAST_LAND":
                ai_alerts.append(
                    f'<span class="ai-jump" data-jump-time="{land_time}">'
                    "🛬 <b>LAND — кероване швидке зниження:</b> "
                    + (
                        f"режим LAND активовано приблизно на висоті {entry_alt:.1f} м. "
                        if entry_alt is not None else ""
                    )
                    + f"Максимальна фактична швидкість зниження — {max_rate:.2f} м/с. "
                    + f"{params_joined}. "
                    + "Форма зниження переважно відповідає заданій швидкості LAND; "
                    + "ознак вільного падіння лише за профілем вертикальної швидкості не підтверджено. "
                    + "Натисніть, щоб перейти до найбільш показового рядка Timeline."
                    "</span>"
                )
            elif land_analysis.get("classification") == "ABNORMAL_LAND_DESCENT":
                bad_time = format_timeline_time(
                    land_analysis.get("worstUnexpectedTimestamp")
                    or land_analysis.get("maxDescentTimestamp"),
                    base_t,
                )
                actual = land_analysis.get("worstUnexpectedActual") or max_rate
                expected = land_analysis.get("worstUnexpectedExpected")

                ai_alerts.append(
                    f'<span class="ai-jump" data-jump-time="{bad_time}">'
                    "🚨 <b>Аномальна втрата висоти в LAND:</b> "
                    + (
                        f"фактична швидкість досягла {actual:.2f} м/с "
                        if actual is not None else ""
                    )
                    + (
                        f"при розрахунковій швидкості близько {expected:.2f} м/с. "
                        if expected is not None else ""
                    )
                    + f"{params_joined}. "
                    + (
                        "Профіль має ознаки швидкого наростання вертикальної швидкості; "
                        if land_analysis.get("possibleFreeFall") else ""
                    )
                    + "Потрібна кореляція з RPM, ATT, VIB, напругою та струмом для відокремлення "
                    + "проблеми силової установки від помилки налаштування або оцінки висоти. "
                    + "Натисніть, щоб перейти до критичного рядка Timeline."
                    "</span>"
                )
                is_critical = True
            else:
                ai_alerts.append(
                    f'<span class="ai-jump" data-jump-time="{land_time}">'
                    "ℹ️ <b>LAND:</b> режим і вертикальне зниження зафіксовані, "
                    "але даних недостатньо для впевненої класифікації як штатного "
                    "або аномального профілю. "
                    + f"{params_joined}."
                    "</span>"
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

        # ====================================================
        # V15 — EKF / OPTICAL NAVIGATION / PILOT ACTION
        # ====================================================
        #
        # Operational rule:
        #   LOITER + EKF variance -> pilot should switch to ALT HOLD.
        #
        # Normal transition:
        #   LOITER -> EKF variance -> ALT HOLD -> EKF3 IMU0 stopped aiding
        #
        # Repeated "EKF3 IMU0 stopped aiding" approximately once per second
        # while still in LOITER means optical-navigation aiding is not working.
        # The conclusion must link to the first event time.
        def _mode_key(value):
            return (
                str(value or "")
                .upper()
                .replace(" ", "")
                .replace("_", "")
                .replace("-", "")
            )

        def _is_loiter(value):
            return _mode_key(value) == "LOITER"

        def _is_alt_hold(value):
            return _mode_key(value) == "ALTHOLD"

        def _loiter_to_alt_hold_transitions():
            return [
                x
                for x in mode_transition_events
                if (
                    _is_loiter(x.get("from"))
                    and _is_alt_hold(x.get("to"))
                )
            ]

        loiter_alt_hold_transitions = _loiter_to_alt_hold_transitions()

        def _first_alt_hold_after(timestamp, max_sec=None):
            candidates = [
                x
                for x in loiter_alt_hold_transitions
                if float(x["timestamp"]) >= float(timestamp)
            ]

            if max_sec is not None:
                candidates = [
                    x
                    for x in candidates
                    if (
                        float(x["timestamp"]) - float(timestamp)
                        <= float(max_sec)
                    )
                ]

            if not candidates:
                return None

            return min(
                candidates,
                key=lambda x: float(x["timestamp"]),
            )

        # ----------------------------------------------------
        # A) EKF variance in LOITER -> evaluate pilot response.
        # ----------------------------------------------------
        ekf_action_checks = []

        for ev in ekf_variance_events:
            if not _is_loiter(ev.get("mode")):
                continue

            ev_ts = float(ev["timestamp"])
            transition = _first_alt_hold_after(
                ev_ts,
                EKF_ALT_HOLD_LATE_SEC,
            )

            if transition is None:
                status = "MISSING"
                reaction_sec = None

            else:
                reaction_sec = (
                    float(transition["timestamp"])
                    - ev_ts
                )

                if reaction_sec <= EKF_ALT_HOLD_CORRECT_SEC:
                    status = "CORRECT"
                else:
                    status = "LATE"

            ekf_action_checks.append({
                "timestamp": ev_ts,
                "mode": ev.get("mode"),
                "status": status,
                "reactionSec": (
                    round(reaction_sec, 2)
                    if reaction_sec is not None
                    else None
                ),
                "altHoldTimestamp": (
                    float(transition["timestamp"])
                    if transition is not None
                    else None
                ),
            })

        # ----------------------------------------------------
        # B) Detect repeated EKF3 IMU0 stopped aiding in LOITER.
        #     Consecutive gaps ~1 sec, minimum 3 messages.
        # ----------------------------------------------------
        loiter_stopped = sorted(
            [
                x
                for x in ekf_stopped_aiding_events
                if _is_loiter(x.get("mode"))
            ],
            key=lambda x: float(x["timestamp"]),
        )

        stopped_aiding_series = []
        current_series = []

        for ev in loiter_stopped:
            if not current_series:
                current_series = [ev]
                continue

            gap = (
                float(ev["timestamp"])
                - float(current_series[-1]["timestamp"])
            )

            if (
                EKF_STOPPED_AIDING_REPEAT_MIN_GAP_SEC
                <= gap
                <= EKF_STOPPED_AIDING_REPEAT_MAX_GAP_SEC
            ):
                current_series.append(ev)
            else:
                if (
                    len(current_series)
                    >= EKF_STOPPED_AIDING_REPEAT_MIN_COUNT
                ):
                    stopped_aiding_series.append(
                        current_series
                    )
                current_series = [ev]

        if (
            len(current_series)
            >= EKF_STOPPED_AIDING_REPEAT_MIN_COUNT
        ):
            stopped_aiding_series.append(
                current_series
            )

        optical_navigation_failures = []

        for series in stopped_aiding_series:
            first_ts = float(series[0]["timestamp"])
            last_ts = float(series[-1]["timestamp"])

            transition = _first_alt_hold_after(
                first_ts,
                EKF_ALT_HOLD_LATE_SEC,
            )

            reaction_sec = (
                float(transition["timestamp"]) - first_ts
                if transition is not None
                else None
            )

            if reaction_sec is None:
                pilot_response = "MISSING"
            elif reaction_sec <= EKF_ALT_HOLD_CORRECT_SEC:
                pilot_response = "CORRECT"
            else:
                pilot_response = "LATE"

            gaps = [
                float(series[i]["timestamp"])
                - float(series[i - 1]["timestamp"])
                for i in range(1, len(series))
            ]

            mean_gap = (
                sum(gaps) / len(gaps)
                if gaps
                else None
            )

            optical_navigation_failures.append({
                "firstTimestamp": first_ts,
                "lastTimestamp": last_ts,
                "count": len(series),
                "meanGapSec": (
                    round(mean_gap, 2)
                    if mean_gap is not None
                    else None
                ),
                "pilotResponse": pilot_response,
                "reactionSec": (
                    round(reaction_sec, 2)
                    if reaction_sec is not None
                    else None
                ),
                "altHoldTimestamp": (
                    float(transition["timestamp"])
                    if transition is not None
                    else None
                ),
            })

        # ----------------------------------------------------
        # C) "stopped aiding" shortly AFTER LOITER -> ALT_HOLD
        #    is normal if EKF variance was seen shortly before.
        # ----------------------------------------------------
        normal_stopped_after_switch = []
        abnormal_stopped_events = []

        for ev in ekf_stopped_aiding_events:
            ev_ts = float(ev["timestamp"])

            matching_transition = None

            for tr in loiter_alt_hold_transitions:
                tr_ts = float(tr["timestamp"])
                dt = ev_ts - tr_ts

                if (
                    0.0 <= dt
                    <= EKF_STOPPED_AIDING_NORMAL_AFTER_SWITCH_SEC
                ):
                    matching_transition = tr
                    break

            preceded_by_variance = False

            if matching_transition is not None:
                tr_ts = float(
                    matching_transition["timestamp"]
                )

                preceded_by_variance = any(
                    (
                        _is_loiter(v.get("mode"))
                        and 0.0
                        <= tr_ts - float(v["timestamp"])
                        <= EKF_VARIANCE_LOOKBACK_BEFORE_SWITCH_SEC
                    )
                    for v in ekf_variance_events
                )

            if (
                matching_transition is not None
                and preceded_by_variance
            ):
                normal_stopped_after_switch.append(ev)
            else:
                abnormal_stopped_events.append(ev)

        # ----------------------------------------------------
        # D) AI conclusions.
        # ----------------------------------------------------
        for check in ekf_action_checks:
            event_time = format_timeline_time(
                check["timestamp"],
                base_t,
            )

            if check["status"] == "CORRECT":
                alt_time = format_timeline_time(
                    check["altHoldTimestamp"],
                    base_t,
                )

                ai_alerts.append(
                    f'<span class="ai-jump" data-jump-time="{event_time}">'
                    "✅ <b>EKF variance у LOITER — реакція пілота штатна:</b> "
                    f"подія {event_time}; перехід у ALT HOLD {alt_time} "
                    f"через {check['reactionSec']:.1f} с. "
                    "За прийнятим алгоритмом ця EKF variance не класифікується "
                    "як критична помилка польоту. "
                    "Натисніть, щоб перейти до EKF variance у Timeline."
                    "</span>"
                )

            elif check["status"] == "LATE":
                alt_time = format_timeline_time(
                    check["altHoldTimestamp"],
                    base_t,
                )

                ai_alerts.append(
                    f'<span class="ai-jump" data-jump-time="{event_time}">'
                    "⚠️ <b>EKF variance у LOITER — запізнілий перехід у ALT HOLD:</b> "
                    f"подія {event_time}; ALT HOLD {alt_time}; "
                    f"час реакції {check['reactionSec']:.1f} с. "
                    f"Бажаний перехід — до {EKF_ALT_HOLD_CORRECT_SEC:.0f} с. "
                    "Натисніть, щоб перейти до події в Timeline."
                    "</span>"
                )

            else:
                ai_alerts.append(
                    f'<span class="ai-jump" data-jump-time="{event_time}">'
                    "⚠️ <b>EKF variance у LOITER — перехід у ALT HOLD не зафіксовано:</b> "
                    f"перша подія {event_time}. "
                    "За прийнятим алгоритмом при EKF variance у LOITER "
                    "пілот має перейти в ALT HOLD. "
                    "Натисніть, щоб перейти до EKF variance у Timeline."
                    "</span>"
                )

        # Strong user-requested optical-navigation conclusion.
        for failure in optical_navigation_failures:
            first_time = format_timeline_time(
                failure["firstTimestamp"],
                base_t,
            )
            last_time = format_timeline_time(
                failure["lastTimestamp"],
                base_t,
            )

            response_text = ""

            if failure["pilotResponse"] == "CORRECT":
                alt_time = format_timeline_time(
                    failure["altHoldTimestamp"],
                    base_t,
                )
                response_text = (
                    f" Пілот перейшов у ALT HOLD {alt_time} "
                    f"через {failure['reactionSec']:.1f} с після початку серії."
                )

            elif failure["pilotResponse"] == "LATE":
                alt_time = format_timeline_time(
                    failure["altHoldTimestamp"],
                    base_t,
                )
                response_text = (
                    f" Перехід у ALT HOLD був запізнілим: {alt_time}, "
                    f"через {failure['reactionSec']:.1f} с."
                )

            else:
                response_text = (
                    " Перехід у ALT HOLD після початку повторюваних "
                    "повідомлень не зафіксовано."
                )

            mean_gap_text = (
                f"{failure['meanGapSec']:.1f} с"
                if failure.get("meanGapSec") is not None
                else "—"
            )

            ai_alerts.append(
                f'<span class="ai-jump" data-jump-time="{first_time}">'
                "🚨 <b>Модуль оптичної навігації не працює.</b> "
                f"У LOITER повідомлення «EKF3 IMU0 stopped aiding» "
                f"повторилося {failure['count']} раз(и) "
                f"з середнім інтервалом {mean_gap_text}; "
                f"серія {first_time}–{last_time}."
                + response_text
                + " Натисніть, щоб перейти до першого stopped aiding у Timeline."
                + "</span>"
            )

        # One compact information line for the expected sequence after switch.
        if normal_stopped_after_switch:
            first_normal = min(
                normal_stopped_after_switch,
                key=lambda x: float(x["timestamp"]),
            )
            normal_time = format_timeline_time(
                first_normal["timestamp"],
                base_t,
            )

            ai_alerts.append(
                f'<span class="ai-jump" data-jump-time="{normal_time}">'
                "ℹ️ <b>EKF3 IMU0 stopped aiding після переходу "
                "LOITER → ALT HOLD:</b> зафіксовано після EKF variance "
                "та штатного переходу в ALT HOLD. За прийнятою логікою "
                "цей stopped aiding є нормальною частиною завершення "
                "позиційного aiding і не піднімається як критична помилка. "
                "Натисніть, щоб перейти до повідомлення у Timeline."
                "</span>"
            )

        # Other EKF positioning events remain summarized separately.
        other_ekf_parts = []

        if loiter_position_fail_count:
            other_ekf_parts.append(
                f"LOITER requires position: {loiter_position_fail_count}"
            )

        if smart_rtl_bad_position_count:
            other_ekf_parts.append(
                f"SmartRTL bad position: {smart_rtl_bad_position_count}"
            )

        if other_ekf_parts:
            ai_alerts.append(
                "⚠️ <b>Інші події позиціонування / EKF:</b> "
                + "; ".join(other_ekf_parts)
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

        # Radio / video-link quality by dBm
        if radio_status_seen:
            if min_dbm != 0:
                worst_state = radio_state_from_dbm(min_dbm)
                if worst_state == "LINK_LOST":
                    ai_alerts.append(
                        "🔴 <b>Радіолінія:</b> зафіксовано -128 dBm — "
                        "за прийнятою логікою відсутні відео та телеметрія."
                    )
                elif worst_state == "VERY_WEAK_TELEMETRY":
                    ai_alerts.append(
                        f"🔴 <b>Дуже слабка телеметрія:</b> мінімум {round(min_dbm)} dBm. "
                        "Це нижче -100 dBm, але повна втрата лінка фіксується лише при -128 dBm."
                    )
                elif worst_state == "VIDEO_LOST_TELEMETRY_OK":
                    ai_alerts.append(
                        f"🟠 <b>Втрата відео:</b> сигнал погіршувався до {round(min_dbm)} dBm; "
                        "за прийнятою логікою телеметрія ще могла бути присутня."
                    )
                elif worst_state == "VIDEO_DEGRADED":
                    ai_alerts.append(
                        f"🟡 <b>Підсипання відео:</b> сигнал погіршувався до {round(min_dbm)} dBm."
                    )
                else:
                    ai_alerts.append(
                        f"✅ <b>Радіолінія:</b> мінімум {round(min_dbm)} dBm — робоча зона до -85 dBm."
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

        # Antenna direction + long -128 correlation
        if antenna_analysis.get("available"):
            center = antenna_analysis["center"]
            smin = antenna_analysis["sectorMin"]
            smax = antenna_analysis["sectorMax"]
            confidence = antenna_analysis.get("confidence", 0)
            method = antenna_analysis.get("method")
            episodes = antenna_analysis.get("longLossEpisodes", [])
            probable = [x for x in episodes if x.get("probableSectorExit")]
            unrecovered = [x for x in episodes if not x.get("recovered")]
            unrecovered_sector = [x for x in probable if not x.get("recovered")]

            method_text = (
                "LOCAL_POSITION_NED + dBm"
                if method == "POSITION_NED"
                else "Heading БПЛА + dBm (резервна евристика)"
            )
            ai_alerts.append(
                "📡 <b>Розрахунковий напрямок АС:</b> "
                f"≈ {center:.1f}°. Кут розкриття {ANTENNA_BEAM_WIDTH_DEG:.0f}°, "
                f"умовний сектор {smin:.1f}°–{smax:.1f}°. "
                f"Метод: {method_text}; {antenna_analysis.get('radioSampleCount', 0)} зразків; "
                f"умовна впевненість {confidence}%."
            )

            strongest_evidence = antenna_analysis.get("strongestSectorEvidence")
            if strongest_evidence and strongest_evidence.get("score", 0) >= 2:
                ev_score = int(strongest_evidence.get("score", 0))
                ev_signs = strongest_evidence.get("signs") or {}
                ev_t0 = format_timeline_time(strongest_evidence["start"], base_t)

                sign_labels = []
                if ev_signs.get("outsideSector"):
                    sign_labels.append("геометричний вихід за сектор")
                if ev_signs.get("deviationGrowing"):
                    sign_labels.append("відхилення від осі АС збільшувалось")
                if ev_signs.get("dbmWorsening"):
                    drop = strongest_evidence.get("dbmDrop")
                    if valid_number(drop):
                        sign_labels.append(f"dBm стійко погіршувався приблизно на {float(drop):.1f} dB")
                    else:
                        sign_labels.append("dBm мав стійкий тренд на погіршення")
                if ev_signs.get("reachedMinus128"):
                    sign_labels.append("сигнал дійшов до -128 dBm")
                if ev_signs.get("noReturnOrRecovery"):
                    sign_labels.append("не зафіксовано повернення в сектор або відновлення після втрати")

                level_text = {
                    "VERY_HIGH": "дуже висока",
                    "HIGH": "висока",
                    "MEDIUM": "середня",
                    "LOW": "низька",
                    "WEAK": "слабка",
                }.get(strongest_evidence.get("level"), "невизначена")

                ai_alerts.append(
                    "📐 <b>Оцінка виходу за сектор АС за 5 ознаками:</b> "
                    f"{ev_score}/5, {level_text} сукупна ймовірність. "
                    f"Початок епізоду ≈ {ev_t0}; "
                    f"максимальне відхилення від осі ≈ {strongest_evidence.get('maxDeviation', 0):.1f}°. "
                    "Ознаки: " + "; ".join(sign_labels) + ". "
                    "Оцінка є ймовірнісною: окремий фактор сам по собі не доводить причину втрати зв'язку."
                )

            if unrecovered_sector:
                first = unrecovered_sector[0]
                t0 = format_timeline_time(first["start"], base_t)
                angle_name = "позиційний азимут" if first.get("angleType") == "positionAzimuth" else "Heading"

                if ground_session:
                    ai_alerts.append(
                        "⚠️ <b>Радіоканал на землі:</b> "
                        f"від {t0} зафіксовано безперервний -128 dBm тривалістю "
                        f"{first['duration']:.1f} с без подальшого відновлення. "
                        "Оскільки ARM/політ не зафіксовано, ця подія НЕ класифікується "
                        "як втрата БПЛА. Це наземна відсутність/втрата радіолінії."
                    )
                else:
                    ai_alerts.append(
                        "🚨 <b>ВИСОКА ЙМОВІРНІСТЬ ВТРАТИ БПЛА ЧЕРЕЗ ВИХІД ІЗ ЗОНИ ЕФЕКТИВНОГО ПОКРИТТЯ АС:</b> "
                        f"від {t0} зафіксовано безперервний -128 dBm тривалістю {first['duration']:.1f} с "
                        "без подальшого відновлення. "
                        f"У {round(first['outsideFraction'] * 100)}% доступних зразків {angle_name} був поза умовним сектором АС; "
                        f"максимальне відхилення від осі ≈ {first['maxDeviation']:.1f}°. "
                        "Сукупність ознак відповідає ймовірній втраті борта після виходу із зони ефективного покриття АС."
                    )
                    is_critical = True
            elif unrecovered:
                first = unrecovered[0]
                t0 = format_timeline_time(first["start"], base_t)

                if ground_session:
                    ai_alerts.append(
                        "⚠️ <b>Тривала втрата радіоканалу на землі:</b> "
                        f"-128 dBm тривав {first['duration']:.1f} с від {t0} "
                        "і не відновився до кінця TLOG. "
                        "Оскільки ARM не було, це НЕ є ознакою втрати БПЛА."
                    )
                else:
                    ai_alerts.append(
                        "🚨 <b>ЙМОВІРНА ВТРАТА БПЛА:</b> "
                        f"-128 dBm тривав {first['duration']:.1f} с від {t0} і не відновився до кінця TLOG. "
                        "Вихід за сектор АС геометрично/по Heading не підтверджений достатньо впевнено."
                    )
                    is_critical = True
            elif probable:
                first = probable[0]
                t0 = format_timeline_time(first["start"], base_t)
                ai_alerts.append(
                    "⚠️ <b>Ймовірний тимчасовий вихід із зони ефективного покриття АС:</b> "
                    f"від {t0} -128 dBm тривав {first['duration']:.1f} с; "
                    f"{round(first['outsideFraction'] * 100)}% зразків були поза сектором. "
                    "Зв'язок надалі відновився, тому втрата борта не підтверджена."
                )
            elif episodes:
                longest = max(episodes, key=lambda x: x["duration"])
                t0 = format_timeline_time(longest["start"], base_t)
                rec = "зв'язок відновився" if longest.get("recovered") else "відновлення не підтверджено"
                ai_alerts.append(
                    "⚠️ <b>Тривала втрата радіоканалу:</b> "
                    f"-128 dBm від {t0}, тривалість {longest['duration']:.1f} с; {rec}. "
                    "Недостатньо ознак, щоб пов'язати її саме з виходом за сектор АС."
                )
            else:
                ai_alerts.append(
                    f"✅ <b>Безперервного -128 dBm понад {RADIO_LONG_LOSS_SEC:.0f} с "
                    "у поєднанні з виходом за умовний сектор АС не зафіксовано.</b>"
                )
        elif ever_armed:
            ai_alerts.append(
                "ℹ️ <b>Напрямок АС автоматично не визначено:</b> "
                "недостатньо даних і для LOCAL_POSITION_NED + dBm, і для резервної оцінки Heading + dBm."
            )

        # ====================================================
        # V14 — CAUSAL CORRELATION FOR POTENTIAL THRUST LOSS
        # ====================================================
        #
        # This is not a "root cause detector". It reconstructs the temporal
        # sequence of independent telemetry signs around each ArduPilot
        # Potential Thrust Loss message and returns an evidence strength.
        #
        # Evidence weights:
        #   matching sustained RPM asymmetry     30
        #   ESC current drop on same motor       20
        #   critical Roll/Pitch after event      15
        #   descent acceleration after event     15
        #   critical vibration near event        10
        #   voltage drop near event               5
        #   ArduPilot STATUSTEXT                  5
        #
        # Max = 100. This is evidence confidence, NOT mathematical probability.
        def _median_valid(values):
            vals = [
                float(v)
                for v in values
                if valid_number(v)
            ]
            return (
                float(statistics.median(vals))
                if vals
                else None
            )

        def _causal_samples(t0, start_rel, end_rel):
            lo = t0 + start_rel
            hi = t0 + end_rel

            return [
                x
                for x in raw_timeline
                if (
                    valid_number(x.get("timestamp"))
                    and lo <= float(x["timestamp"]) <= hi
                )
            ]

        def _esc_motor_value(sample, motor, key):
            if motor is None or not (1 <= motor <= 4):
                return None

            esc = sample.get("esc")
            if not isinstance(esc, list):
                return None

            for item in esc:
                if (
                    isinstance(item, dict)
                    and item.get("id") == motor
                    and valid_number(item.get(key))
                ):
                    return float(item[key])

            return None

        def _relative_time_text(dt):
            if not valid_number(dt):
                return "—"

            dt = float(dt)

            if abs(dt) < 0.05:
                return "у момент події"

            if dt < 0:
                return f"за {abs(dt):.1f} с до"

            return f"через {dt:.1f} с після"

        def build_thrust_causal_analysis(thrust):
            t0 = float(thrust["timestamp"])
            motor = thrust.get("motor")

            result = {
                "timestamp": t0,
                "motor": motor,
                "score": 5,  # ArduPilot warning itself
                "level": "LOW",
                "features": [],
                "sequence": [],
                "matchingRpm": False,
                "escCurrentDrop": False,
                "attitudeResponse": False,
                "descentResponse": False,
                "vibrationResponse": False,
                "voltageDrop": False,
            }

            result["sequence"].append({
                "timestamp": t0,
                "delta": 0.0,
                "kind": "STATUSTEXT",
                "text": (
                    f"ArduPilot: Potential Thrust Loss ({motor})"
                    if motor is not None
                    else "ArduPilot: Potential Thrust Loss"
                ),
            })

            # ------------------------------------------------
            # 1. Sustained RPM asymmetry, same motor.
            # Prefer startTimestamp because it tells us when the anomaly began.
            # ------------------------------------------------
            matching_rpm = [
                e for e in rpm_drop_events
                if (
                    motor is not None
                    and e.get("lowerMotor") == motor
                    and abs(
                        float(
                            e.get(
                                "startTimestamp",
                                e.get("timestamp", -1e9),
                            )
                        )
                        - t0
                    ) <= THRUST_CAUSAL_PRE_SEC + THRUST_CAUSAL_POST_SEC
                )
            ]

            if matching_rpm:
                rpm_ev = min(
                    matching_rpm,
                    key=lambda e: abs(
                        float(
                            e.get(
                                "startTimestamp",
                                e.get("timestamp", t0),
                            )
                        )
                        - t0
                    ),
                )

                rpm_ts = float(
                    rpm_ev.get(
                        "startTimestamp",
                        rpm_ev["timestamp"],
                    )
                )
                rpm_dt = rpm_ts - t0

                # Only count as causal evidence if onset is reasonably close:
                # up to 4 s before or 3 s after the warning.
                if -THRUST_CAUSAL_PRE_SEC <= rpm_dt <= 3.0:
                    result["matchingRpm"] = True
                    result["score"] += 30
                    result["features"].append(
                        f"RPM Motor {motor}: sustained diagonal asymmetry "
                        f"{rpm_ev.get('differencePct', 0.0):.1f}%"
                    )
                    result["sequence"].append({
                        "timestamp": rpm_ts,
                        "delta": rpm_dt,
                        "kind": "RPM",
                        "text": (
                            f"RPM anomaly M{rpm_ev.get('pair', '?')}: "
                            f"{rpm_ev.get('differencePct', 0.0):.1f}%; "
                            f"lower Motor {motor}"
                        ),
                    })

            # ------------------------------------------------
            # 2. ESC current drop on the same motor.
            # Baseline: -4.0..-0.8 s. Event/post: -0.3..+2.0 s.
            # ------------------------------------------------
            if motor is not None:
                pre = _causal_samples(t0, -4.0, -0.8)
                around = _causal_samples(t0, -0.3, 2.0)

                pre_curr = [
                    _esc_motor_value(x, motor, "current")
                    for x in pre
                ]
                post_curr_pairs = [
                    (
                        float(x["timestamp"]),
                        _esc_motor_value(x, motor, "current"),
                    )
                    for x in around
                ]
                post_curr_pairs = [
                    x for x in post_curr_pairs
                    if valid_number(x[1])
                ]

                baseline_current = _median_valid(pre_curr)

                if (
                    baseline_current is not None
                    and baseline_current >= 0.5
                    and post_curr_pairs
                ):
                    min_ts, min_current = min(
                        post_curr_pairs,
                        key=lambda x: x[1],
                    )

                    drop_pct = (
                        (baseline_current - min_current)
                        / max(baseline_current, 0.001)
                        * 100.0
                    )

                    if (
                        drop_pct >= THRUST_ESC_CURRENT_DROP_PCT
                        and min_current < baseline_current
                    ):
                        result["escCurrentDrop"] = True
                        result["score"] += 20
                        result["features"].append(
                            f"ESC{motor} current: "
                            f"{baseline_current:.1f} → {min_current:.1f} A "
                            f"(-{drop_pct:.0f}%)"
                        )
                        result["sequence"].append({
                            "timestamp": min_ts,
                            "delta": min_ts - t0,
                            "kind": "ESC_CURRENT",
                            "text": (
                                f"ESC{motor} current fell from baseline "
                                f"{baseline_current:.1f} A to {min_current:.1f} A "
                                f"(-{drop_pct:.0f}%)"
                            ),
                        })

            # ------------------------------------------------
            # 3. Critical attitude response after the warning.
            # ------------------------------------------------
            att_after = [
                x for x in attitude_critical_events
                if (
                    valid_number(x.get("timestamp"))
                    and 0.0
                    <= float(x["timestamp"]) - t0
                    <= 3.0
                )
            ]

            if att_after:
                att = min(
                    att_after,
                    key=lambda x: float(x["timestamp"]),
                )
                att_ts = float(att["timestamp"])
                result["attitudeResponse"] = True
                result["score"] += 15
                result["features"].append(
                    f"critical Roll/Pitch response to {att.get('peak', 0.0):.1f}°"
                )
                result["sequence"].append({
                    "timestamp": att_ts,
                    "delta": att_ts - t0,
                    "kind": "ATTITUDE",
                    "text": (
                        f"critical attitude: Roll {att.get('roll', 0.0):.1f}°, "
                        f"Pitch {att.get('pitch', 0.0):.1f}°, "
                        f"peak {att.get('peak', 0.0):.1f}°"
                    ),
                })

            # ------------------------------------------------
            # 4. Downward-speed response.
            # Compare pre-event baseline with max 0..+4 s.
            # ------------------------------------------------
            pre_vs_samples = _causal_samples(t0, -3.0, -0.5)
            post_vs_samples = _causal_samples(t0, 0.0, 4.0)

            baseline_vs = _median_valid([
                x.get("verticalSpeedDown")
                for x in pre_vs_samples
            ])

            post_vs_pairs = [
                (
                    float(x["timestamp"]),
                    float(x["verticalSpeedDown"]),
                )
                for x in post_vs_samples
                if valid_number(x.get("verticalSpeedDown"))
            ]

            if post_vs_pairs:
                max_vs_ts, max_vs = max(
                    post_vs_pairs,
                    key=lambda x: x[1],
                )

                baseline_vs_num = (
                    max(0.0, baseline_vs)
                    if baseline_vs is not None
                    else 0.0
                )
                descent_delta = max_vs - baseline_vs_num

                if (
                    max_vs >= THRUST_DESCENT_MIN_MPS
                    and descent_delta >= THRUST_DESCENT_DELTA_MPS
                ):
                    result["descentResponse"] = True
                    result["score"] += 15
                    result["features"].append(
                        f"vertical speed down: "
                        f"{baseline_vs_num:.1f} → {max_vs:.1f} m/s"
                    )
                    result["sequence"].append({
                        "timestamp": max_vs_ts,
                        "delta": max_vs_ts - t0,
                        "kind": "DESCENT",
                        "text": (
                            f"down speed increased from "
                            f"{baseline_vs_num:.1f} to {max_vs:.1f} m/s"
                        ),
                    })

            # ------------------------------------------------
            # 5. Critical vibration near the warning.
            # ------------------------------------------------
            vib_near = [
                x for x in vibration_critical_events
                if (
                    valid_number(x.get("timestamp"))
                    and abs(
                        float(x["timestamp"]) - t0
                    ) <= MECHANICAL_CORRELATION_SEC
                )
            ]

            if vib_near:
                vib = min(
                    vib_near,
                    key=lambda x: abs(float(x["timestamp"]) - t0),
                )
                vib_ts = float(vib["timestamp"])
                result["vibrationResponse"] = True
                result["score"] += 10
                result["features"].append(
                    f"critical vibration peak {vib.get('peak', 0.0):.1f}"
                )
                result["sequence"].append({
                    "timestamp": vib_ts,
                    "delta": vib_ts - t0,
                    "kind": "VIBRATION",
                    "text": (
                        f"critical vibration: "
                        f"X={vib.get('x', 0.0):.1f}, "
                        f"Y={vib.get('y', 0.0):.1f}, "
                        f"Z={vib.get('z', 0.0):.1f}"
                    ),
                })

            # ------------------------------------------------
            # 6. Voltage drop near / after warning.
            # ------------------------------------------------
            pre_volt = _median_valid([
                x.get("volt")
                for x in _causal_samples(t0, -3.0, -0.5)
            ])
            post_volt_pairs = [
                (
                    float(x["timestamp"]),
                    float(x["volt"]),
                )
                for x in _causal_samples(t0, 0.0, 3.0)
                if valid_number(x.get("volt"))
            ]

            if (
                pre_volt is not None
                and post_volt_pairs
            ):
                min_v_ts, min_v = min(
                    post_volt_pairs,
                    key=lambda x: x[1],
                )
                voltage_drop = pre_volt - min_v

                if voltage_drop >= THRUST_VOLTAGE_DROP_V:
                    result["voltageDrop"] = True
                    result["score"] += 5
                    result["features"].append(
                        f"voltage: {pre_volt:.2f} → {min_v:.2f} V"
                    )
                    result["sequence"].append({
                        "timestamp": min_v_ts,
                        "delta": min_v_ts - t0,
                        "kind": "VOLTAGE",
                        "text": (
                            f"voltage dropped "
                            f"{pre_volt:.2f} → {min_v:.2f} V"
                        ),
                    })

            result["score"] = max(
                0,
                min(100, int(round(result["score"]))),
            )

            if result["score"] >= 75:
                result["level"] = "VERY_HIGH"
            elif result["score"] >= 55:
                result["level"] = "HIGH"
            elif result["score"] >= 35:
                result["level"] = "MEDIUM"
            elif result["score"] >= 20:
                result["level"] = "LOW"
            else:
                result["level"] = "VERY_LOW"

            result["sequence"].sort(
                key=lambda x: x["timestamp"]
            )

            return result

        thrust_causal_analyses = [
            build_thrust_causal_analysis(x)
            for x in (
                []
                if accelerometer_calibration_session
                else potential_thrust_loss_events
            )
        ]

        # RPM / thrust diagnostics.
        if rpm_drop_events and not accelerometer_calibration_session:
            strongest_rpm = max(
                rpm_drop_events,
                key=lambda x: x.get("differencePct", 0.0),
            )
            rpm_time = format_timeline_time(strongest_rpm["timestamp"], base_t)
            pair = strongest_rpm.get("pair", "?")
            low_motor = strongest_rpm.get("lowerMotor")
            rpms = strongest_rpm.get("rpms", [None, None, None, None])
            rpm_values_text = ", ".join(
                f"M{i + 1}={v if v is not None else '—'}"
                for i, v in enumerate(rpms)
            )

            ai_alerts.append(
                f'<span class="ai-jump" data-jump-time="{rpm_time}">'
                "🚨 <b>Аномальна асиметрія RPM двигунів:</b> "
                f"о {rpm_time} у діагоналі M{pair} різниця досягла "
                f"{strongest_rpm.get('differencePct', 0.0):.1f}%. "
                + (
                    f"Нижчий RPM зафіксовано у Motor {low_motor}. "
                    if low_motor else ""
                )
                + f"RPM у цей момент: {rpm_values_text}. "
                f"Критичний поріг діагональної різниці: ≥ {RPM_DIAGONAL_CRITICAL_PCT:.0f}% "
                f"протягом ≥ {RPM_CRITICAL_PERSIST_SEC:.1f} с. "
                "Натисніть, щоб перейти до рядка Timeline."
                "</span>"
            )
            is_critical = True

        # V14 — chronological multi-sensor evidence around Potential Thrust Loss.
        for causal in thrust_causal_analyses:
            thrust_time = format_timeline_time(
                causal["timestamp"],
                base_t,
            )
            motor = causal.get("motor")
            score = causal.get("score", 0)
            level = causal.get("level", "VERY_LOW")

            level_ua = {
                "VERY_HIGH": "ДУЖЕ ВИСОКА",
                "HIGH": "ВИСОКА",
                "MEDIUM": "СЕРЕДНЯ",
                "LOW": "НИЗЬКА",
                "VERY_LOW": "ДУЖЕ НИЗЬКА",
            }.get(level, level)

            sequence_html = []

            for seq in causal.get("sequence", []):
                seq_time = format_timeline_time(
                    seq.get("timestamp"),
                    base_t,
                )
                dt_text = _relative_time_text(
                    seq.get("delta")
                )
                sequence_html.append(
                    f"{seq_time} — {dt_text}: {seq.get('text', '')}"
                )

            sequence_text = "; ".join(sequence_html)

            # Interpretation deliberately distinguishes evidence strength
            # from a mathematically proven root cause.
            if score >= 75:
                headline = (
                    f"🚨 <b>Висока узгодженість ознак реальної втрати тяги"
                    + (
                        f" Motor {motor}"
                        if motor is not None
                        else ""
                    )
                    + ":</b> "
                )
                interpretation = (
                    "Послідовність незалежних телеметричних ознак добре узгоджується "
                    "з реальною проблемою тяги/силового каналу. Конкретну першопричину "
                    "(двигун, ESC, проводка, пропелер або механічне пошкодження) "
                    "за одним TLOG встановити неможливо."
                )
                is_critical = True

            elif score >= 55:
                headline = (
                    f"🚨 <b>Ймовірна реальна втрата тяги"
                    + (
                        f" Motor {motor}"
                        if motor is not None
                        else ""
                    )
                    + ":</b> "
                )
                interpretation = (
                    "Є кілька незалежних підтверджень, але повної причинної картини "
                    "не вистачає. Потрібна перевірка ESC, двигуна, пропелера та живлення."
                )
                is_critical = True

            elif score >= 35:
                headline = (
                    f"⚠️ <b>Potential Thrust Loss"
                    + (
                        f" ({motor})"
                        if motor is not None
                        else ""
                    )
                    + " — частково підтверджено:</b> "
                )
                interpretation = (
                    "Є окремі супутні ознаки, але їх недостатньо для висновку "
                    "про підтверджену відмову силового каналу."
                )

            else:
                headline = (
                    f"⚠️ <b>Potential Thrust Loss"
                    + (
                        f" ({motor})"
                        if motor is not None
                        else ""
                    )
                    + " без достатнього телеметричного підтвердження:</b> "
                )
                interpretation = (
                    "Повідомлення ArduPilot збережено як попередження. "
                    "Незалежних ознак недостатньо для підтвердження реальної відмови."
                )

            ai_alerts.append(
                f'<span class="ai-jump" data-jump-time="{thrust_time}">'
                + headline
                + f"<b>Evidence confidence: {score}% ({level_ua}).</b> "
                + interpretation
                + (
                    " <br><b>Послідовність:</b> "
                    + sequence_text
                    if sequence_text
                    else ""
                )
                + " Натисніть, щоб перейти до Potential Thrust Loss у Timeline."
                + "</span>"
            )

        # Multi-sensor scenario for possible external mechanical influence.
        impact_candidates = []
        for rpm_event in rpm_drop_events:
            t0 = rpm_event["timestamp"]
            near_att = [
                x for x in attitude_critical_events
                if abs(x.get("timestamp", -1e9) - t0) <= MECHANICAL_CORRELATION_SEC
            ]
            near_vib = [
                x for x in vibration_critical_events
                if abs(x.get("timestamp", -1e9) - t0) <= MECHANICAL_CORRELATION_SEC
            ]
            near_thrust = [
                x for x in potential_thrust_loss_events
                if abs(x.get("timestamp", -1e9) - t0) <= RPM_THRUST_CORRELATION_SEC
            ]

            score = int(bool(near_att)) + int(bool(near_vib)) + int(bool(near_thrust))
            if score >= 1:
                impact_candidates.append({
                    "rpm": rpm_event,
                    "att": near_att,
                    "vib": near_vib,
                    "thrust": near_thrust,
                    "score": score,
                })

        if impact_candidates and not accelerometer_calibration_session:
            impact = max(
                impact_candidates,
                key=lambda x: (
                    x["score"],
                    x["rpm"].get("differencePct", 0.0),
                ),
            )
            event = impact["rpm"]
            impact_time = format_timeline_time(event["timestamp"], base_t)

            features = [
                f"асиметрія RPM {event.get('differencePct', 0.0):.1f}% "
                f"у діагоналі M{event.get('pair', '?')}"
            ]
            if impact["att"]:
                att = max(impact["att"], key=lambda x: x.get("peak", 0.0))
                features.append(
                    f"різка зміна просторового положення до "
                    f"{att.get('peak', 0.0):.1f}° по Roll/Pitch"
                )
            if impact["vib"]:
                vib = max(impact["vib"], key=lambda x: x.get("peak", 0.0))
                features.append(
                    f"вібрації до {vib.get('peak', 0.0):.1f}"
                )
            if impact["thrust"]:
                motors = sorted({
                    x.get("motor") for x in impact["thrust"]
                    if x.get("motor") is not None
                })
                features.append(
                    "Potential Thrust Loss"
                    + (f" для Motor {','.join(map(str, motors))}" if motors else "")
                )

            ai_alerts.append(
                f'<span class="ai-jump" data-jump-time="{impact_time}">'
                "🧩 <b>Ознаки ймовірного зовнішнього механічного впливу "
                "або пошкодження силової установки:</b> "
                + "; ".join(features)
                + ". Одночасна/близька в часі поява цих незалежних ознак "
                "нехарактерна для простої плавної зміни тяги. Така картина може "
                "відповідати контакту або удару стороннього об'єкта, потраплянню "
                "стороннього предмета в площину гвинтів, пошкодженню пропелера, "
                "двигуна чи ESC. За одним TLOG неможливо встановити конкретний "
                "предмет, його походження або підтвердити навмисне ураження. "
                "Натисніть, щоб перейти до найближчого критичного рядка Timeline."
                "</span>"
            )
            is_critical = True

        # Attitude / tilt. Threshold requested for timeline highlighting: >= 35 deg.
        if attitude_critical_events and not accelerometer_calibration_session:
            peak_event = max(attitude_critical_events, key=lambda x: x.get("peak", 0.0))
            peak_time = format_timeline_time(peak_event.get("timestamp"), base_t)
            ai_alerts.append(
                f'<span class="ai-jump" data-jump-time="{peak_time}">'
                "🔄 <b>Критичний кут нахилу БПЛА:</b> "
                f"о {peak_time} Roll {peak_event.get('roll', 0.0):.1f}°, "
                f"Pitch {peak_event.get('pitch', 0.0):.1f}°, "
                f"Yaw {(peak_event.get('yaw') or 0.0):.1f}°. "
                f"Поріг підсвічування Roll/Pitch: ≥ {ATTITUDE_CRITICAL_THRESHOLD_DEG:.0f}°. "
                "Натисніть, щоб перейти до рядка Timeline."
                "</span>"
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
        # V15: expected EKF variance -> ALT HOLD -> stopped aiding must not
        # turn an otherwise successful flight yellow by itself.
        navigation_problem = (
            any(
                x.get("status") in ("LATE", "MISSING")
                for x in ekf_action_checks
            )
            or bool(optical_navigation_failures)
            or loiter_position_fail_count > 0
            or smart_rtl_bad_position_count > 0
            or bool(abnormal_stopped_events)
        )

        # Calibration / ground context has the highest verdict priority.
        if accelerometer_calibration_session:
            # Flight-only anomalies must not promote a ground calibration log
            # to a "critical flight" verdict.
            is_critical = False

            if accel_calibration_success:
                ai_verdict = (
                    "🧭 НАЗЕМНА СЕСІЯ — ПРОВОДИЛОСЬ КАЛІБРУВАННЯ АКСЕЛЕРОМЕТРА. "
                    "КАЛІБРУВАННЯ ЗАВЕРШЕНО УСПІШНО:"
                )
            else:
                ai_verdict = (
                    "🧭 НАЗЕМНА СЕСІЯ — ПРОВОДИЛОСЬ КАЛІБРУВАННЯ АКСЕЛЕРОМЕТРА. "
                    "ПОТРІБНА ПЕРЕВІРКА ЗАВЕРШЕННЯ ПРОЦЕДУРИ:"
                )

        elif ground_session:
            # No ARM means this is not a completed/failed flight.
            ai_verdict = (
                "ℹ️ НАЗЕМНА СЕСІЯ — ARM І ФАКТИЧНИЙ ПОЛІТ НЕ ЗАФІКСОВАНО:"
            )

        elif disarm_detected:
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

                # V15 EKF pilot-action diagnostics.
                "ekfActionChecks": ekf_action_checks,
                "opticalNavigationFailures": optical_navigation_failures,
                "normalStoppedAidingAfterAltHoldCount": len(
                    normal_stopped_after_switch
                ),
                "abnormalStoppedAidingCount": len(
                    abnormal_stopped_events
                ),

                "loiterPositionFailCount": loiter_position_fail_count,
                "externalNavRecoveryCount": external_nav_recovery_count,
                "smartRtlBadPositionCount": smart_rtl_bad_position_count,
                "antennaMapAnalysisByFlight": antenna_map_analysis_by_flight,
                "antennaAnalysis": {
                    "available": antenna_analysis.get("available", False),
                    "method": antenna_analysis.get("method"),
                    "center": antenna_analysis.get("center"),
                    "sectorMin": antenna_analysis.get("sectorMin"),
                    "sectorMax": antenna_analysis.get("sectorMax"),
                    "beamWidth": antenna_analysis.get("beamWidth", ANTENNA_BEAM_WIDTH_DEG),
                    "confidence": antenna_analysis.get("confidence", 0),
                    "radioSampleCount": antenna_analysis.get("radioSampleCount", 0),
                    "longLossEpisodeCount": len(antenna_analysis.get("longLossEpisodes", [])),
                    "probableSectorExitCount": antenna_analysis.get("probableSectorExitCount", 0),
                    "maxDeviation": antenna_analysis.get("maxDeviation", 0.0),
                    "probableBoardLoss": antenna_analysis.get("probableBoardLoss", False),
                    "probableBoardLossDueSector": antenna_analysis.get("probableBoardLossDueSector", False),
                    "sectorEvidenceScore": antenna_analysis.get("sectorEvidenceScore", 0),
                    "sectorEvidenceLevel": antenna_analysis.get("sectorEvidenceLevel", "NONE"),
                    "strongestSectorEvidence": antenna_analysis.get("strongestSectorEvidence"),
                    "sectorEvidenceEpisodes": antenna_analysis.get("sectorEvidenceEpisodes", []),
                    "longLossEpisodes": antenna_analysis.get("longLossEpisodes", []),
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
                "attitudeCriticalThreshold": ATTITUDE_CRITICAL_THRESHOLD_DEG,
                "attitudeCriticalEventCount": len(attitude_critical_events),
                "rpmDiagonalCriticalThresholdPct": RPM_DIAGONAL_CRITICAL_PCT,
                "rpmDiagonalWarningThresholdPct": RPM_DIAGONAL_WARNING_PCT,
                "rpmDropEventCount": len(rpm_drop_events),
                "potentialThrustLossCount": len(potential_thrust_loss_events),
                "thrustCausalAnalyses": thrust_causal_analyses,
                "flightSessionCount": len(flight_sessions),
                "accelCalibration": {
                    "detected": bool(accel_calibration_events),
                    "success": accel_calibration_success,
                    "requiresReboot": accel_calibration_requires_reboot,
                    "groundSession": ground_session,
                    "isCalibrationSession": accelerometer_calibration_session,
                    "startTimestamp": accel_calibration_start_ts,
                    "endTimestamp": accel_calibration_end_ts,
                    "eventCount": len(accel_calibration_events),
                },
                "flightSessions": flight_sessions,
                "landAnalysis": land_analysis,
                "landParams": {
                    "LAND_SPEED": land_params.get("LAND_SPEED"),
                    "LAND_SPEED_HIGH": land_params.get("LAND_SPEED_HIGH"),
                    "LAND_ALT_LOW": land_params.get("LAND_ALT_LOW"),
                },
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
            "controls": {
                "deadbandUs": CONTROL_CENTER_DEADBAND_US,
                "axes": {
                    CONTROL_AXIS_META[ch]["key"]: {
                        "channel": ch,
                        "current": control_snapshot(ch),
                        "maxNegativePct": round(control_extremes[ch]["negative"], 1),
                        "maxPositivePct": round(control_extremes[ch]["positive"], 1),
                        "negativeLabel": CONTROL_AXIS_META[ch]["negativeLabel"],
                        "positiveLabel": CONTROL_AXIS_META[ch]["positiveLabel"],
                        "calibration": {
                            "min": int(round(control_calibration(ch)[0])),
                            "trim": int(round(control_calibration(ch)[1])),
                            "max": int(round(control_calibration(ch)[2])),
                            "source": (
                                f"TLOG RC{ch}_MIN/TRIM/MAX"
                                if all(valid_number(rc_axis_cal[ch].get(f"RC{ch}_{suffix}")) for suffix in ("MIN", "TRIM", "MAX"))
                                else "fallback / partial TLOG calibration"
                            ),
                        },
                    }
                    for ch in range(1, 5)
                },
            },
            "throttle": {
                "current": throttle_snapshot(),
                "maxUpPct": round(throttle_max_up_pct, 1),
                "maxDownPct": round(throttle_max_down_pct, 1),
                "deadbandUs": THROTTLE_CENTER_DEADBAND_US,
                "reversed": THROTTLE_REVERSED,
                "calibration": {
                    "min": throttle_snapshot().get("minPwm") if throttle_snapshot() else int(round(throttle_calibration()[0])),
                    "trim": throttle_snapshot().get("centerPwm") if throttle_snapshot() else int(round(throttle_calibration()[1])),
                    "max": throttle_snapshot().get("maxPwm") if throttle_snapshot() else int(round(throttle_calibration()[2])),
                    "source": "TLOG RC3_MIN/TRIM/MAX" if all(valid_number(rc_axis_cal[3].get(k)) for k in ("RC3_MIN", "RC3_TRIM", "RC3_MAX")) else "fallback / partial TLOG calibration",
                },
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
                "ch6": (
                    f"{rc_min[6]}–{rc_max[6]} us"
                    if rc_max[6] > 0
                    else "—"
                ),
                "ch10": (
                    f"{rc_min[10]}–{rc_max[10]} us"
                    if rc_max[10] > 0
                    else "—"
                ),
            },
            "tx16": {
                "mapping": dict(TX16_SWITCH_CHANNELS),
                "switchPwm": dict(tx16_switch_pwm),
                "switchState": dict(tx16_switch_state),
                "payloadCommandCount": len(payload_commands),
                "payloadConfirmedCount": sum(
                    1 for x in payload_commands if x.get("confirmedByServo")
                ),
                "payloadCommands": payload_commands,
                "payloadConfirmServo": PAYLOAD_CONFIRM_SERVO,
                "servoOutputEventCount": len(servo_output_events),
                "emergencyStopAttemptCount": len(emergency_stop_attempts),
                "emergencyStopThresholdCount": sum(
                    1 for x in emergency_stop_attempts if x.get("thresholdReached")
                ),
                "emergencyStopConfirmedCount": emergency_stop_confirmed_count,
                "emergencyStopAttempts": emergency_stop_attempts,
                "emergencyStopMinHoldSec": EMERGENCY_STOP_MIN_HOLD_SEC,
                "emergencyStopExpectedHoldSec": EMERGENCY_STOP_EXPECTED_HOLD_SEC,
            },
            "timeline": timeline,
        }

    finally:
        if os.path.exists(temp.name):
            try:
                os.unlink(temp.name)
            except Exception:
                pass
