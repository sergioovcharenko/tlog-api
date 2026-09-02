# v1.2 3D MAP: backend algorithms unchanged; 3D is UI-only in HTML.
# v1.1 «Швидкість» — safe performance branch; core analysis calculations preserved.
# V23.31: backend logic unchanged; distance display logic remains UI-only in HTML.
# V23.26 — power-priority diagnostics + exact per-flight battery statistics
# V23.28 — clearer powertrain wording; competing-cause logic preserved
# ============================================================
# TLOG ANALYZER V23
#
# Основні зміни V23:
# 1) Вісь антени: для прямолінійного радіального польоту пріоритет має
#    фактичний напрямок LOCAL_POSITION_NED (flight corridor), а dBm є перевіркою.
# 2) Якщо траєкторія не є прямолінійною — використовується попередня оцінка
#    POSITION_NED + dBm; Heading лишається лише резервною евристикою.
# 3) RC_CHANNELS CH1..CH18 зберігаються у кожному SNAPSHOT для випадаючих
#    віртуальних стіків TX16S у Timeline.
# ============================================================


import math
import os
import tempfile
import re
import statistics
import threading
import time
import webbrowser
from pathlib import Path

from fastapi import FastAPI, File, UploadFile
from fastapi.responses import FileResponse
from starlette.middleware.gzip import GZipMiddleware
import shutil
from pymavlink import mavutil

app = FastAPI()
# v1.0 «Швидкість»: compress large JSON responses. This does not alter analysis values.
app.add_middleware(GZipMiddleware, minimum_size=4096, compresslevel=5)

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

# V23: якщо фактичний NED-політ утворює чіткий радіальний коридор від АС,
# його напрямок є кращою оцінкою фізичної осі антени, ніж одиничні піки dBm.
# Це особливо важливо на далеких польотах, де multipath / AGC можуть зміщувати
# максимум RSSI/dBm убік від реального напрямку антени.
ANTENNA_PATH_MIN_MAX_DISTANCE_M = 100.0
ANTENNA_PATH_MIN_FAR_FRACTION = 0.35
ANTENNA_PATH_MIN_CONCENTRATION = 0.93
ANTENNA_PATH_MIN_SAMPLES = 15
ANTENNA_PATH_GOOD_SIGNAL_MIN_FRACTION = 0.35

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


def analyze_antenna_direction(raw_timeline, arm_timestamp):
    """
    Оцінка фізичного напрямку антенної станції (АС) та втрати зв'язку.

    V23 — порядок визначення осі АС:
      1. FLIGHT_PATH_NED — якщо політ утворює чіткий прямолінійний/радіальний
         коридор від NED-origin, вісь беремо з фактичної геометрії польоту.
         dBm тут використовується як перевірка, а не як єдине джерело напрямку.
      2. POSITION_NED — попередній алгоритм: позиційний азимут + dBm + дальність.
      3. HEADING_FALLBACK — лише резервна евристика, коли NED недостатньо.

    Чому так:
    Heading показує напрямок носа БПЛА, а не напрямок від АС до БПЛА.
    Окремі піки dBm можуть бути зміщені multipath/AGC. Якщо ж літак тисячі метрів
    летить уздовж одного NED-азимуту, цей коридор є сильним геометричним доказом
    напрямку фізичної антени.
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

        # Діагностика нового V23-методу — корисно показувати у UI.
        "flightPathCenter": None,
        "flightPathConcentration": 0.0,
        "flightPathSampleCount": 0,
        "flightPathGoodSignalFraction": 0.0,
        "flightPathMaxDistance": 0.0,

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

    # --------------------------------------------------------
    # Формуємо телеметричні знімки після ARM.
    # Для геометрії нам важливі positionAzimuth + distance.
    # ALT HOLD не використовуємо для визначення коридору, бо LOCAL_POSITION_NED
    # там може бути "замороженим" останнім значенням, а фронтенд уже робить DR.
    # --------------------------------------------------------
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
        mode = str(ev.get("mode") or "").upper().replace(" ", "_")

        snapshots.append({
            "timestamp": float(ts),
            "mode": mode,
            "positionAzimuth": float(pos_az) % 360.0 if valid_number(pos_az) else None,
            "heading": float(heading) % 360.0 if valid_number(heading) else None,
            "distance": float(dist) if valid_number(dist) else None,
            "dbm": float(dbm) if valid_number(dbm) else None,
        })

    reference = None
    compare_key = "positionAzimuth"

    # --------------------------------------------------------
    # 1) V23 PRIMARY: ФАКТИЧНИЙ NED-КОРИДОР ПОЛЬОТУ.
    # --------------------------------------------------------
    # Беремо лише геометрично далеку частину польоту. Близько до origin навіть
    # кілька метрів E/N можуть давати великий стрибок азимуту, тому такі точки
    # не повинні "крутити" фізичний сектор антени.
    path_candidates = [
        x for x in snapshots
        if x["positionAzimuth"] is not None
        and x["distance"] is not None
        and x["distance"] >= ANTENNA_MIN_DISTANCE_M
        and "ALTHOLD" not in x["mode"]
        and "ALT_HOLD" not in x["mode"]
    ]

    path_max_distance = max(
        [x["distance"] for x in path_candidates] or [0.0]
    )
    result["flightPathMaxDistance"] = round(path_max_distance, 1)

    far_threshold = max(
        ANTENNA_MIN_DISTANCE_M,
        path_max_distance * ANTENNA_PATH_MIN_FAR_FRACTION,
    )
    far_path = [
        x for x in path_candidates
        if x["distance"] >= far_threshold
    ]

    if far_path:
        # Чим далі точка від АС, тим стабільніший її геометричний азимут.
        # Вагу обмежуємо, щоб одна найдальша точка не домінувала над усіма.
        weighted_path = []
        for x in far_path:
            ratio = x["distance"] / max(far_threshold, 1.0)
            weight = max(1.0, min(8.0, ratio))
            weighted_path.append((x["positionAzimuth"], weight))

        path_center, path_concentration = circular_weighted_mean(weighted_path)
        result["flightPathSampleCount"] = len(far_path)
        result["flightPathConcentration"] = round(path_concentration, 3)
        result["flightPathCenter"] = round(path_center, 1) if path_center is not None else None

        signal_rows = [
            x for x in far_path
            if x["dbm"] is not None and x["dbm"] < 0
        ]
        good_signal_rows = [
            x for x in signal_rows
            if x["dbm"] >= RADIO_NORMAL_DBM
        ]
        good_signal_fraction = (
            len(good_signal_rows) / len(signal_rows)
            if signal_rows else 0.0
        )
        result["flightPathGoodSignalFraction"] = round(good_signal_fraction, 3)

        path_is_clear = bool(
            path_center is not None
            and path_max_distance >= ANTENNA_PATH_MIN_MAX_DISTANCE_M
            and len(far_path) >= ANTENNA_PATH_MIN_SAMPLES
            and path_concentration >= ANTENNA_PATH_MIN_CONCENTRATION
            # Якщо dBm є — хоча б частина далекого коридору повинна мати
            # робочий сигнал. Якщо dBm відсутній взагалі, геометрію не відкидаємо.
            and (
                not signal_rows
                or good_signal_fraction >= ANTENNA_PATH_GOOD_SIGNAL_MIN_FRACTION
            )
        )

        if path_is_clear:
            reference = path_center
            compare_key = "positionAzimuth"

            sample_factor = min(1.0, len(far_path) / 60.0)
            distance_factor = min(1.0, path_max_distance / 1000.0)
            signal_factor = (
                1.0 if not signal_rows
                else min(1.0, 0.65 + good_signal_fraction * 0.35)
            )
            confidence = int(round(
                path_concentration
                * (0.70 + 0.30 * sample_factor)
                * (0.75 + 0.25 * distance_factor)
                * signal_factor
                * 100.0
            ))
            result["method"] = "FLIGHT_PATH_NED"
            result["confidence"] = max(1, min(98, confidence))
            result["radioSampleCount"] = len(signal_rows)

    # --------------------------------------------------------
    # 1b) POSITION_NED + dBm.
    # Якщо прямолінійний коридор не підтверджений, використовуємо попередню
    # модель: нормалізуємо dBm по дальності та шукаємо напрямок сильного сигналу.
    # --------------------------------------------------------
    if reference is None:
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

        if len(scored) >= ANTENNA_MIN_RADIO_SAMPLES:
            scored.sort(key=lambda x: x[0], reverse=True)
            top_n = max(
                ANTENNA_MIN_RADIO_SAMPLES,
                int(math.ceil(len(scored) * ANTENNA_TOP_SIGNAL_FRACTION)),
            )
            top = scored[:min(top_n, len(scored))]
            min_score = min(x[0] for x in top)
            weighted = [(x[1], (x[0] - min_score) + 1.0) for x in top]
            reference, concentration = circular_weighted_mean(weighted)
            sample_factor = min(1.0, len(scored) / 60.0)
            confidence = int(round(concentration * sample_factor * 100.0))
            result["method"] = "POSITION_NED"
            result["confidence"] = confidence
            compare_key = "positionAzimuth"

    # --------------------------------------------------------
    # 1c) HEADING FALLBACK.
    # Це останній резерв: Heading = напрямок носа, а не геометрична лінія АС→БПЛА.
    # --------------------------------------------------------
    if reference is None:
        heading_samples = []
        for x in snapshots:
            h, dbm = x["heading"], x["dbm"]
            if h is None or dbm is None or dbm < RADIO_NORMAL_DBM or dbm >= 0:
                continue
            w = max(1.0, min(25.0, dbm - RADIO_NORMAL_DBM + 1.0))
            heading_samples.append((h, w))

        if len(heading_samples) >= ANTENNA_MIN_RADIO_SAMPLES:
            reference, concentration = circular_weighted_mean(heading_samples)
            sample_factor = min(1.0, len(heading_samples) / 60.0)
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

    return result


def analyze_flight_sessions(raw_timeline, log_end_timestamp=None, battery_voltage_samples=None, battery_current_samples=None):
    """One flight = ARM->DISARM. Re-takeoff without DISARM stays in same flight.

    V23.26: min/max battery values are calculated from every armed SYS_STATUS
    sample, not only from sparse Timeline snapshots. This keeps the flight card
    consistent with the global battery summary.
    """
    battery_voltage_samples = battery_voltage_samples or []
    battery_current_samples = battery_current_samples or []
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

        # Exact SYS_STATUS battery samples for this ARM->end interval.
        session_voltages = [
            float(v) for ts, v in battery_voltage_samples
            if ts >= s["armTimestamp"]
            and (s["endTimestamp"] is None or ts <= s["endTimestamp"])
            and valid_number(v) and float(v) > 0
        ]
        if session_voltages:
            exact_min = min(session_voltages)
            if s["minVoltage"] is None or exact_min < s["minVoltage"]:
                s["minVoltage"] = exact_min

        session_currents = [
            float(v) for ts, v in battery_current_samples
            if ts >= s["armTimestamp"]
            and (s["endTimestamp"] is None or ts <= s["endTimestamp"])
            and valid_number(v) and float(v) >= 0
        ]
        if session_currents:
            s["maxCurrent"] = max(s["maxCurrent"], max(session_currents))

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


# ============================================================
# V23.14 — TX16S SWITCH MONITORING
# ============================================================
# Mapping used by this analyzer. These are telemetry/display names only.
# ============================================================
# V23.14 — RADIO CALIBRATION DETECTION
# ============================================================
# Радіокалібрування визначаємо НЕ за одним повідомленням, а за характерною
# послідовністю RC_CHANNELS на землі: осі CH1..CH4 багаторазово проходять
# через низьке / середнє / високе положення і охоплюють майже весь PWM-діапазон.
# Це дозволяє відрізняти калібрування від звичайного короткого руху стіком.
RADIO_CAL_CHANNELS = (1, 2, 3, 4)
RADIO_CAL_LOW_PWM = 1200
RADIO_CAL_HIGH_PWM = 1800
RADIO_CAL_MIN_SPAN_PWM = 650
RADIO_CAL_MIN_TRANSITIONS = 10
RADIO_CAL_MIN_DURATION_SEC = 3.0


def radio_cal_bucket(pwm):
    """Груба зона положення осі для виявлення проходження MIN/CENTER/MAX."""
    if not valid_number(pwm):
        return None
    v = float(pwm)
    if v <= RADIO_CAL_LOW_PWM:
        return "LOW"
    if v >= RADIO_CAL_HIGH_PWM:
        return "HIGH"
    return "MID"

TX16_SWITCH_CHANNELS = {
    "SH": 6,
    "SC": 7,
    "SD": 8,
    "SF": 10,
}

def tx16_two_position_state(pwm):
    """Return OFF/ON for a two-position RC switch, or None in transition/invalid zone."""
    if not valid_number(pwm):
        return None
    pwm = float(pwm)
    if pwm <= 1300:
        return "OFF"
    if pwm >= 1700:
        return "ON"
    return None

def tx16_switch_state(name, pwm):
    """Normalized switch state used for change detection and Timeline text."""
    name = str(name or "").upper()
    if name in ("SC", "SD"):
        pos = three_position_switch(pwm)
        return f"POS{pos}" if pos is not None else None
    if name in ("SF", "SH"):
        return tx16_two_position_state(pwm)
    return None

def tx16_switch_state_text(name, pwm):
    """Human-readable neutral state label for the analyzer UI."""
    state = tx16_switch_state(name, pwm)
    if state is None:
        return "ПЕРЕХІДНА ЗОНА"
    if state.startswith("POS"):
        return "ПОЗИЦІЯ " + state[-1]
    return state

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

@app.get("/health")
def health():
    return {"status": "ok", "mode": "offline", "version": "V24"}


@app.get("/", include_in_schema=False)
def offline_index():
    """Serve the local analyzer UI from the same folder."""
    index_path = Path(__file__).resolve().parent / "index.html"
    return FileResponse(index_path, media_type="text/html; charset=utf-8")


# ============================================================
# ANALYZE
# ============================================================

@app.post("/analyze")
async def analyze(file: UploadFile = File(...)):
    # v1.1 «Швидкість»: preserve v1.0 chunked upload; calculation algorithms unchanged.
    # v1.0 «Швидкість»: copy the uploaded TLOG in chunks instead of creating
    # a second full-size bytes object in RAM. Parsing and calculation logic below
    # remains unchanged, so telemetry results stay 1:1 with the previous branch.
    temp = tempfile.NamedTemporaryFile(delete=False, suffix=".tlog")
    try:
        await file.seek(0)
        while True:
            chunk = await file.read(1024 * 1024)
            if not chunk:
                break
            temp.write(chunk)
    finally:
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

        curr_voltage = 0.0
        curr_amp = 0.0

        # V23.6: поточне навантаження силової установки.
        # Для мультикоптера беремо MAVLink VFR_HUD.throttle (0..100 %).
        # Це командний throttle/engine load, а не CPU load з SYS_STATUS.load.
        curr_engine_load = None

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
        # Поточні PWM усіх каналів. Зберігаємо їх у кожному SNAPSHOT,
        # щоб Timeline міг показати положення віртуальних стіків саме в цей момент.
        curr_rc_channels = {i: None for i in range(1, 19)}
        max_throttle = 0

        # V23.14 — накопичувач ознак РАДІОКАЛІБРУВАННЯ.
        # Рахуємо лише коли борт DISARMED, щоб звичайний політ зі значними
        # командами по стіках не помилково визначався як калібрування.
        radio_cal_stats = {
            ch: {"min": 9999, "max": 0, "low": False, "high": False}
            for ch in RADIO_CAL_CHANNELS
        }
        radio_cal_last_bucket = {ch: None for ch in RADIO_CAL_CHANNELS}
        radio_cal_transition_count = 0
        radio_cal_first_activity_ts = None
        radio_cal_last_activity_ts = None
        radio_cal_rc_message_count = 0

        # Battery
        min_voltage = 999.0
        max_current = 0.0
        start_voltage = None
        arm_voltage = None
        reboot_or_second_battery = False
        # V23.26: every armed SYS_STATUS sample is retained for exact
        # per-flight battery statistics (not displayed as extra Timeline rows).
        battery_voltage_samples = []
        battery_current_samples = []

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
                    "rcChannels": {
                        f"ch{i}": curr_rc_channels[i]
                        for i in range(1, 19)
                        if curr_rc_channels[i] is not None
                    },
                    "vtxBand": curr_vtx_band,
                    "vtxChannel": curr_vtx_channel,
                    "videoFreq": curr_video_freq,
                    "volt": round(curr_voltage, 2) if curr_voltage > 0 else None,
                    "curr": round(curr_amp, 1) if curr_amp >= 0 else None,
                    # V23.6: Engine Load = VFR_HUD.throttle у відсотках.
                    "engineLoad": round(curr_engine_load, 1) if valid_number(curr_engine_load) else None,
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
                    "rcChannels": {
                        f"ch{i}": curr_rc_channels[i]
                        for i in range(1, 19)
                        if curr_rc_channels[i] is not None
                    },
                    "vtxBand": curr_vtx_band,
                    "vtxChannel": curr_vtx_channel,
                    "videoFreq": curr_video_freq,
                    "volt": round(curr_voltage, 2) if curr_voltage > 0 else None,
                    "curr": round(curr_amp, 1) if curr_amp >= 0 else None,
                    # V23.6: Engine Load = VFR_HUD.throttle у відсотках.
                    "engineLoad": round(curr_engine_load, 1) if valid_number(curr_engine_load) else None,
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

                    if param_id in land_params:
                        value = getattr(msg, "param_value", None)
                        if valid_number(value):
                            land_params[param_id] = float(value)
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
                        battery_voltage_samples.append((current_timestamp, volt))

                if curr >= 0:
                    curr_amp = curr

                    if is_currently_armed:
                        max_current = max(max_current, curr)
                        battery_current_samples.append((current_timestamp, curr))

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

                # V23.6: Engine Load для Timeline.
                # VFR_HUD.throttle у ArduPilot/MAVLink передається як 0..100 %.
                # Зберігаємо саме поточне значення, а max_throttle лишається
                # окремою статистикою максимального навантаження за політ.
                throttle_val = getattr(msg, "throttle", None)
                if valid_number(throttle_val):
                    throttle_val = max(0.0, min(100.0, float(throttle_val)))
                    curr_engine_load = throttle_val
                    max_throttle = max(max_throttle, throttle_val)

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
                        curr_rc_channels[ch_num] = int(round(float(val)))
                        rc_min[ch_num] = min(
                            rc_min[ch_num],
                            val,
                        )

                        rc_max[ch_num] = max(
                            rc_max[ch_num],
                            val,
                        )

                # ----------------------------------------------------
                # V23.14: виявлення характерної процедури радіокалібрування.
                # Під час калібрування оператор проводить CH1..CH4 через
                # крайні положення. Фіксуємо повний діапазон та кількість
                # переходів між LOW/MID/HIGH, але тільки при DISARM.
                # ----------------------------------------------------
                if not is_currently_armed:
                    valid_cal_axes = 0
                    changed_this_message = False

                    for cal_ch in RADIO_CAL_CHANNELS:
                        cal_val = channels.get(cal_ch, 0)
                        if not valid_number(cal_val) or not 800 < float(cal_val) < 2200:
                            continue

                        valid_cal_axes += 1
                        cal_val = int(round(float(cal_val)))
                        st = radio_cal_stats[cal_ch]
                        st["min"] = min(st["min"], cal_val)
                        st["max"] = max(st["max"], cal_val)
                        if cal_val <= RADIO_CAL_LOW_PWM:
                            st["low"] = True
                        if cal_val >= RADIO_CAL_HIGH_PWM:
                            st["high"] = True

                        bucket = radio_cal_bucket(cal_val)
                        prev_bucket = radio_cal_last_bucket[cal_ch]
                        if prev_bucket is not None and bucket != prev_bucket:
                            radio_cal_transition_count += 1
                            changed_this_message = True
                        radio_cal_last_bucket[cal_ch] = bucket

                    if valid_cal_axes >= 3:
                        radio_cal_rc_message_count += 1
                        if changed_this_message:
                            if radio_cal_first_activity_ts is None:
                                radio_cal_first_activity_ts = current_timestamp
                            radio_cal_last_activity_ts = current_timestamp

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

                # V23.9: explicitly track SC / SD / SF / SH by interpreted state,
                # not merely by a >250 us raw jump. This fixes missed SF (CH10)
                # transitions and makes SC/SD changes visible by switch name.
                for switch_name, ch_num in TX16_SWITCH_CHANNELS.items():
                    val = channels.get(ch_num, 0)
                    if not valid_number(val) or not 800 < float(val) < 2200:
                        continue

                    val = int(round(float(val)))
                    prev_pwm = last_rc_state[ch_num]
                    prev_state = (
                        tx16_switch_state(switch_name, prev_pwm)
                        if valid_number(prev_pwm) and float(prev_pwm) > 0
                        else None
                    )
                    new_state = tx16_switch_state(switch_name, val)

                    # Emit an event only after the channel has been seen once and
                    # the interpreted position/state really changed.
                    if prev_pwm > 0 and new_state != prev_state:
                        add_event(
                            f"🎚 {switch_name} (CH{ch_num}): "
                            f"{tx16_switch_state_text(switch_name, val)} ({val} us)",
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
        flight_sessions = analyze_flight_sessions(
            raw_timeline,
            current_timestamp,
            battery_voltage_samples=battery_voltage_samples,
            battery_current_samples=battery_current_samples,
        )
        first_flight_arm_timestamp = flight_sessions[0]["armTimestamp"] if flight_sessions else arm_timestamp

        # ====================================================
        # V23.14 — RADIO CALIBRATION RESULT
        # ====================================================
        radio_cal_full_axes = []
        radio_cal_axis_ranges = {}
        for cal_ch in RADIO_CAL_CHANNELS:
            st = radio_cal_stats[cal_ch]
            if st["max"] > 0 and st["min"] < 9999:
                span = st["max"] - st["min"]
                radio_cal_axis_ranges[cal_ch] = {
                    "min": int(st["min"]),
                    "max": int(st["max"]),
                    "span": int(span),
                    "lowSeen": bool(st["low"]),
                    "highSeen": bool(st["high"]),
                }
                if st["low"] and st["high"] and span >= RADIO_CAL_MIN_SPAN_PWM:
                    radio_cal_full_axes.append(cal_ch)

        radio_cal_duration = 0.0
        if (
            radio_cal_first_activity_ts is not None
            and radio_cal_last_activity_ts is not None
        ):
            radio_cal_duration = max(
                0.0,
                float(radio_cal_last_activity_ts) - float(radio_cal_first_activity_ts),
            )

        # Висока впевненість: усі 4 осі пройшли MIN/MAX.
        # Помірний fallback: 3 осі + дуже багато переходів.
        radio_cal_detected = bool(
            (
                len(radio_cal_full_axes) == 4
                and radio_cal_transition_count >= RADIO_CAL_MIN_TRANSITIONS
                and radio_cal_duration >= RADIO_CAL_MIN_DURATION_SEC
            )
            or (
                len(radio_cal_full_axes) >= 3
                and radio_cal_transition_count >= RADIO_CAL_MIN_TRANSITIONS + 8
                and radio_cal_duration >= RADIO_CAL_MIN_DURATION_SEC
            )
        )

        radio_cal_confidence = 0
        if radio_cal_detected:
            axis_score = min(1.0, len(radio_cal_full_axes) / 4.0)
            transition_score = min(1.0, radio_cal_transition_count / 24.0)
            duration_score = min(1.0, radio_cal_duration / 15.0)
            radio_cal_confidence = int(round(100.0 * (
                0.60 * axis_score + 0.25 * transition_score + 0.15 * duration_score
            )))

            # Додаємо дві клікабельні часові мітки у Timeline: початок / кінець.
            existing_rows = [
                row for row in raw_timeline
                if row.get("timestamp") is not None
            ]

            def make_radio_cal_marker(ts, text, event_type):
                if existing_rows:
                    nearest = min(
                        existing_rows,
                        key=lambda row: abs(float(row.get("timestamp", 0.0)) - float(ts)),
                    )
                    marker = dict(nearest)
                else:
                    marker = {
                        "timestamp": ts, "mode": "DISARMED",
                        "alt": "0.0 м", "dist": "0.0 м", "distValue": 0.0,
                        "azimuth": None, "positionAzimuth": None,
                        "nedNorth": None, "nedEast": None, "nedDown": None,
                        "rcChannels": {}, "antennaSector": None,
                        "vtxBand": None, "vtxChannel": None, "videoFreq": None,
                        "volt": None, "curr": None, "engineLoad": None,
                        "rssi": None, "dbm": None, "temp": None,
                        "esc": [], "vibration": None, "attitude": None,
                        "rpmAnalysis": None, "verticalSpeedDown": None,
                    }
                marker.update({
                    "timestamp": ts,
                    "system_text": "",
                    "analysis_text": text,
                    "pilot_text": "",
                    "isError": False,
                    "eventType": event_type,
                })
                raw_timeline.append(marker)

            make_radio_cal_marker(
                radio_cal_first_activity_ts,
                "🎮 РАДІОКАЛІБРУВАННЯ: початок проходження стіків по діапазонах",
                "RADIO_CALIBRATION_START",
            )
            make_radio_cal_marker(
                radio_cal_last_activity_ts,
                "✅ РАДІОКАЛІБРУВАННЯ: завершення активного проходження стіків",
                "RADIO_CALIBRATION_END",
            )
            raw_timeline.sort(key=lambda row: row.get("timestamp", 0.0))

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
                        "engineLoad": None,
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
                    "rcChannels": ev.get("rcChannels", {}),
                    "antennaSector": ev.get("antennaSector"),
                    "vtxBand": ev.get("vtxBand"),
                    "vtxChannel": ev.get("vtxChannel"),
                    "videoFreq": ev.get("videoFreq"),
                    "volt": ev.get("volt"),
                    "curr": ev.get("curr"),
                    # V23.6: поточний Engine Load/Throttle, %.
                    "engineLoad": ev.get("engineLoad"),
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
                    "flightNumber": ev.get("flightNumber"),
                    "takeoffEpisodeNumber": ev.get("takeoffEpisodeNumber"),
                    "systemText": ev.get("system_text", ""),
                    "analysisText": ev.get("analysis_text", ""),
                    "pilotText": ev.get("pilot_text", ""),
                    "eventType": ev.get("eventType", "SYSTEM"),
                    "isError": bool(ev.get("isError", False)),
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

        # High-level context has priority over flight-only heuristics.
        ground_session = not ever_armed
        accelerometer_calibration_session = bool(
            ground_session and accel_calibration_events
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
        # V23.14 — окремий AI-висновок про проведення радіокалібрування.
        if radio_cal_detected:
            rc_start = format_timeline_time(radio_cal_first_activity_ts, base_t)
            rc_end = format_timeline_time(radio_cal_last_activity_ts, base_t)
            axes_text = ", ".join(f"CH{ch}" for ch in radio_cal_full_axes)
            ranges_text = "; ".join(
                f"CH{ch} {radio_cal_axis_ranges[ch]['min']}–{radio_cal_axis_ranges[ch]['max']} us"
                for ch in sorted(radio_cal_axis_ranges)
            )
            ai_alerts.append(
                f'<span class="ai-jump" data-jump-time="{rc_start}">'
                "🎮 <b>ВИЯВЛЕНО ПРОВЕДЕННЯ РАДІОКАЛІБРУВАННЯ TX16S / RC.</b> "
                f"Активне проходження каналів: {rc_start} → {rc_end} "
                f"({radio_cal_duration:.1f} с). Повний MIN/MAX підтверджено для: "
                f"{axes_text or '—'}. Переходів між зонами стіків: "
                f"{radio_cal_transition_count}. Впевненість: {radio_cal_confidence}%. "
                f"Діапазони: {ranges_text}. "
                "Великі відхилення CH1–CH4 у цьому інтервалі є очікуваними для "
                "процедури калібрування і не трактуються самі по собі як хибні команди. "
                "Натисніть, щоб перейти до початку радіокалібрування."
                "</span>"
            )
            ai_alerts.append(
                f'<span class="ai-jump" data-jump-time="{rc_end}">'
                "✅ <b>Завершення радіокалібрування:</b> "
                f"остання активна зміна RC зафіксована о {rc_end}. "
                "Натисніть, щоб перейти до цього рядка Timeline."
                "</span>"
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

        # V23.26 — PRIORITY CAUSE: severe 6S power collapse.
        # This has higher diagnostic priority than an RPM-only/external-impact
        # hypothesis. TLOG can show symptoms, but cannot prove an external cause.
        voltage_sag = max(0.0, (arm_voltage or 0.0) - (min_voltage or 0.0))
        severe_power_collapse = bool(
            ever_armed
            and min_voltage > 0
            and (
                min_voltage <= 15.0
                or (arm_voltage and arm_voltage > 0 and voltage_sag >= 7.0)
            )
        )
        power_with_thrust_loss = bool(
            severe_power_collapse
            and (potential_thrust_loss_events or rpm_drop_events)
        )

        if power_with_thrust_loss:
            is_critical = True
            power_jump_ts = None
            if potential_thrust_loss_events:
                power_jump_ts = potential_thrust_loss_events[0].get("timestamp")
            elif rpm_drop_events:
                power_jump_ts = rpm_drop_events[0].get("timestamp")
            power_jump_time = format_timeline_time(power_jump_ts, base_t) if power_jump_ts is not None else None
            thrust_text = ""
            if potential_thrust_loss_events:
                motors = sorted({
                    e.get("motor") for e in potential_thrust_loss_events
                    if e.get("motor") is not None
                })
                if motors:
                    thrust_text = f" Зафіксовано Potential Thrust Loss для Motor {','.join(map(str, motors))}."
                else:
                    thrust_text = " Зафіксовано Potential Thrust Loss."
            rpm_text = ""
            if rpm_drop_events:
                strongest = max(rpm_drop_events, key=lambda e: e.get("differencePct", 0.0))
                rpm_text = f" Максимальна асиметрія RPM: {strongest.get('differencePct', 0.0):.1f}%."
            body = (
                "🔴 <b>Критична проблема живлення / втрата тяги — ЗАФІКСОВАНО:</b> "
                f"напруга під час ARMED впала до {min_voltage:.2f} V"
                + (f" (просадка від ARM приблизно {voltage_sag:.2f} V)" if arm_voltage else "")
                + f", піковий струм {max_current:.1f} A."
                + thrust_text
                + rpm_text
                + " Ця картина підтверджує критичне просідання живлення та/або "
                  "недостатню доступну тягу, але сама по собі не встановлює першопричину. "
                  "Якщо одночасно присутні незалежні механічні ознаки, зовнішнє пошкодження "
                  "залишається окремою конкурентною версією."
            )
            if power_jump_time:
                ai_alerts.append(
                    f'<span class="ai-jump" data-jump-time="{power_jump_time}">'
                    + body
                    + " Натисніть, щоб перейти до критичної події Timeline.</span>"
                )
            else:
                ai_alerts.append(body)

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

        # Potential Thrust Loss is correlated with RPM, but is not sufficient alone.
        for thrust in ([] if accelerometer_calibration_session else potential_thrust_loss_events):
            thrust_time = format_timeline_time(thrust["timestamp"], base_t)
            motor = thrust.get("motor")

            nearby_rpm = [
                e for e in rpm_drop_events
                if abs(e["timestamp"] - thrust["timestamp"]) <= RPM_THRUST_CORRELATION_SEC
            ]
            matching_rpm = [
                e for e in nearby_rpm
                if motor is not None and e.get("lowerMotor") == motor
            ]

            if matching_rpm:
                e = max(matching_rpm, key=lambda x: x.get("differencePct", 0.0))
                ai_alerts.append(
                    f'<span class="ai-jump" data-jump-time="{thrust_time}">'
                    f"🚨 <b>Potential Thrust Loss ({motor}) підтверджено телеметрією RPM:</b> "
                    f"поруч із повідомленням зафіксовано діагональну різницю "
                    f"{e.get('differencePct', 0.0):.1f}% з нижчим RPM у Motor {motor}. "
                    "Це підсилює ймовірність реальної втрати тяги/проблеми двигуна, ESC "
                    "або механічного пошкодження пропелера. "
                    "Натисніть, щоб перейти до рядка Timeline."
                    "</span>"
                )
                is_critical = True
            elif nearby_rpm:
                ai_alerts.append(
                    f'<span class="ai-jump" data-jump-time="{thrust_time}">'
                    f"⚠️ <b>Potential Thrust Loss ({motor if motor is not None else '?'}) :</b> "
                    "повідомлення зафіксовано, однак найбільша RPM-асиметрія поруч "
                    "припала на інший двигун/діагональ. Потрібна ручна перевірка. "
                    "Саме повідомлення не вважається доказом відмови."
                    "</span>"
                )
            else:
                ai_alerts.append(
                    f'<span class="ai-jump" data-jump-time="{thrust_time}">'
                    f"⚠️ <b>Potential Thrust Loss ({motor if motor is not None else '?'}) без RPM-підтвердження:</b> "
                    "у часовому вікні ±3 с критичної діагональної різниці RPM не виявлено. "
                    "Подія класифікується як попередження, а не підтверджена відмова. "
                    "Натисніть, щоб перейти до рядка Timeline."
                    "</span>"
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

            # V23.27: do not suppress a possible external-impact hypothesis merely
            # because voltage also collapsed. When several independent mechanical
            # signs coincide and the TLOG ends while ARMED, show two competing causes.
            possible_shootdown = bool(
                ever_armed
                and was_armed
                and impact["score"] >= 2
            )

            if possible_shootdown:
                power_context = ""
                if severe_power_collapse:
                    power_context = (
                        f" Одночасно напруга впала до {min_voltage:.2f} V. Це може бути "
                        "окремою причиною, супутньою подією або наслідком пошкодження; "
                        "за одним TLOG встановити напрямок причинності неможливо."
                    )
                ai_alerts.append(
                    f'<span class="ai-jump" data-jump-time="{impact_time}">'
                    "⚠️ <b>Можливе збиття / зовнішній механічний вплив — ВИСОКА ЙМОВІРНІСТЬ, "
                    "АЛЕ НЕ ПІДТВЕРДЖЕНО ЛИШЕ TLOG:</b> "
                    + "; ".join(features)
                    + ". Після сукупності цих незалежних ознак TLOG завершується при ARMED "
                    "без підтвердженого DISARM. Така послідовність може відповідати "
                    "раптовому зовнішньому механічному впливу або пошкодженню силової установки."
                    + power_context
                    + " За одним TLOG неможливо встановити джерело впливу, конкретний засіб "
                      "або підтвердити факт збиття. Натисніть, щоб перейти до найближчого "
                      "критичного рядка Timeline."
                    "</span>"
                )
            else:
                ai_alerts.append(
                    f'<span class="ai-jump" data-jump-time="{impact_time}">'
                    "🧩 <b>Ознаки порушення роботи силової установки — ПРИЧИНА НЕ ВСТАНОВЛЕНА:</b> "
                    + "; ".join(features)
                    + ". Це означає, що робота одного або кількох моторів/каналів тяги "
                    "відрізнялась від очікуваної. Можливі причини: пошкодження пропелера, "
                    "несправність двигуна або ESC, проблема живлення чи зовнішнє механічне "
                    "пошкодження. Сам TLOG не дозволяє визначити конкретну причину або "
                    "підтвердити факт збиття. Натисніть, щоб перейти до найближчого "
                    "критичного рядка Timeline."
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
            # V23.23: останній ARMED-рядок також клікабельний.
            end_time = format_timeline_time(current_timestamp, base_t)
            ai_alerts.append(
                f'<span class="ai-jump" data-jump-time="{end_time}">'
                "❗ <b>Лог закінчився при ARMED:</b> "
                "у файлі немає підтвердженого DISARM. "
                "Натисніть, щоб перейти до останнього рядка Timeline."
                "</span>"
            )
            is_critical = True

        # Final verdict
        navigation_problem = (
            ekf_variance_count > 0
            or ekf_stopped_aiding_count > 0
            or loiter_position_fail_count > 0
            or smart_rtl_bad_position_count > 0
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
                "loiterPositionFailCount": loiter_position_fail_count,
                "externalNavRecoveryCount": external_nav_recovery_count,
                "smartRtlBadPositionCount": smart_rtl_bad_position_count,
                "antennaAnalysis": {
                    "available": antenna_analysis.get("available", False),
                    "method": antenna_analysis.get("method"),
                    "center": antenna_analysis.get("center"),
                    "sectorMin": antenna_analysis.get("sectorMin"),
                    "sectorMax": antenna_analysis.get("sectorMax"),
                    "beamWidth": antenna_analysis.get("beamWidth", ANTENNA_BEAM_WIDTH_DEG),
                    "confidence": antenna_analysis.get("confidence", 0),
                    "radioSampleCount": antenna_analysis.get("radioSampleCount", 0),
                    # V23: діагностика того, чому обрано саме такий напрямок АС.
                    "flightPathCenter": antenna_analysis.get("flightPathCenter"),
                    "flightPathConcentration": antenna_analysis.get("flightPathConcentration", 0.0),
                    "flightPathSampleCount": antenna_analysis.get("flightPathSampleCount", 0),
                    "flightPathGoodSignalFraction": antenna_analysis.get("flightPathGoodSignalFraction", 0.0),
                    "flightPathMaxDistance": antenna_analysis.get("flightPathMaxDistance", 0.0),
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
                "flightSessionCount": len(flight_sessions),
                "radioCalibration": {
                    "detected": radio_cal_detected,
                    "confidence": radio_cal_confidence,
                    "startTimestamp": radio_cal_first_activity_ts,
                    "endTimestamp": radio_cal_last_activity_ts,
                    "durationSec": round(radio_cal_duration, 2),
                    "transitionCount": radio_cal_transition_count,
                    "fullRangeChannels": radio_cal_full_axes,
                    "axisRanges": radio_cal_axis_ranges,
                    "rcMessageCount": radio_cal_rc_message_count,
                },
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
                # V23.8: TX16S switch mapping from this aircraft setup:
                # SH -> CH6, SC -> CH7, SD -> CH8, SF -> CH10.
                "ch6": (
                    f"{rc_min[6]}–{rc_max[6]} us"
                    if rc_max[6] > 0
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
                "ch10": (
                    f"{rc_min[10]}–{rc_max[10]} us"
                    if rc_max[10] > 0
                    else "—"
                ),
            },
            # V23.9: final/current TX16S switch states for the summary cards.
            "tx16Switches": {
                name: {
                    "channel": ch_num,
                    "pwm": curr_rc_channels.get(ch_num),
                    "state": tx16_switch_state_text(
                        name, curr_rc_channels.get(ch_num)
                    ) if curr_rc_channels.get(ch_num) is not None else "—",
                    "minPwm": rc_min[ch_num] if rc_max[ch_num] > 0 else None,
                    "maxPwm": rc_max[ch_num] if rc_max[ch_num] > 0 else None,
                }
                for name, ch_num in TX16_SWITCH_CHANNELS.items()
            },
            "timeline": timeline,
        }

    finally:
        if os.path.exists(temp.name):
            try:
                os.unlink(temp.name)
            except Exception:
                pass

# ============================================================
# OFFLINE V24 LAUNCHER
# ============================================================
def _open_browser():
    """Open the local UI shortly after Uvicorn starts."""
    time.sleep(1.2)
    webbrowser.open("http://127.0.0.1:8765/")


if __name__ == "__main__":
    import uvicorn

    threading.Thread(target=_open_browser, daemon=True).start()
    uvicorn.run(app, host="127.0.0.1", port=8765, log_level="warning")
