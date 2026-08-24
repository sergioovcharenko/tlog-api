from fastapi import FastAPI, UploadFile, File
from pymavlink import mavutil
import tempfile
import os
import math


app = FastAPI()


# ============================================================
# RSSI / dBm
# ============================================================

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


# ============================================================
# ROOT
# ============================================================

@app.get("/")
def root():
    return {"status": "ok"}


@app.get("/health")
def health():
    return {"status": "ok"}


# ============================================================
# ANALYZE TLOG
# ============================================================

@app.post("/analyze")
async def analyze(file: UploadFile = File(...)):

    data = await file.read()

    temp = tempfile.NamedTemporaryFile(
        delete=False,
        suffix=".tlog"
    )

    try:

        temp.write(data)
        temp.close()

        mav = mavutil.mavlink_connection(temp.name)

        # ====================================================
        # БАЗОВІ ЗМІННІ
        # ====================================================

        message_count = 0

        max_alt = 0.0
        max_speed = 0.0
        max_roll = 0.0
        max_pitch = 0.0


        # ====================================================
        # ВИСОТА
        # ====================================================

        # Остання барометрична висота
        latest_baro_alt = None

        # Барометричний нуль землі
        ground_baro_alt = None

        # Основна висота EKF
        ekf_alt = None

        # Резервна барометрична висота
        baro_rel_alt = None

        # Поточна висота, яку використовує аналізатор
        curr_alt = 0.0

        # Остання валідна висота
        last_valid_alt = 0.0

        # Timestamp останньої валідної висоти
        last_alt_timestamp = None


        # ====================================================
        # RANGEFINDER
        # ====================================================

        rangefinder_alt = None
        max_rf_alt = 0.0

        has_rangefinder = False
        rangefinder_failed_flag = False


        # ====================================================
        # СТАН ДВИГУНІВ
        # ====================================================

        is_currently_armed = False
        was_armed = False


        # ====================================================
        # ПОТОЧНІ ЗНАЧЕННЯ
        # ====================================================

        curr_dist = 0.0
        curr_voltage = 0.0
        curr_amp = 0.0
        curr_rssi_pct = 0
        curr_dbm = 0


        # ====================================================
        # ЗВ'ЯЗОК
        # ====================================================

        min_rssi = 255

        min_dbm = 0

        telem_rssi_raw = None
        telem_remrssi_raw = None


        # ====================================================
        # RC
        # ====================================================

        rc_min = {
            i: 9999
            for i in range(1, 9)
        }

        rc_max = {
            i: 0
            for i in range(1, 9)
        }

        last_rc_state = {
            i: 0
            for i in range(1, 9)
        }

        max_throttle = 0


        # ====================================================
        # БАТАРЕЯ
        # ====================================================

        min_voltage = 999.0
        max_current = 0.0

        start_voltage = None
        voltage_at_land_mode = None

        reboot_or_second_battery = False


        # ====================================================
        # ВІБРАЦІЇ / ДАТЧИКИ
        # ====================================================

        max_vib_x = 0.0
        max_vib_y = 0.0
        max_vib_z = 0.0

        clip_count = 0

        max_temp = -99.0


        # ====================================================
        # ОПТИЧНА НАВІГАЦІЯ
        # ====================================================

        vnav_quality_min_loiter = 999
        vnav_quality_max_loiter = 0
        vnav_samples = 0

        loiter_has_origin = False
        loiter_origin_failed = False


        # ====================================================
        # GPS
        # ====================================================

        has_gps = False


        # ====================================================
        # ЧАС
        # ====================================================

        first_timestamp = None
        current_timestamp = 0.0


        # ====================================================
        # РЕЖИМИ
        # ====================================================

        current_mode = "Невідомо"

        flight_modes = set()

        land_mode_triggered = False


        # ====================================================
        # TIMELINE
        # ====================================================

        raw_timeline = []


        # ====================================================
        # ФУНКЦІЯ ОНОВЛЕННЯ ВИСОТИ
        # ====================================================

        def update_flight_altitude(new_alt, timestamp=None):

            """
            Оновлює основну висоту польоту.

            Пріоритет джерел задається зовні:
                1. GLOBAL_POSITION_INT / EKF
                2. VFR_HUD / Barometer
                3. ALTITUDE / резерв

            Rangefinder сюди НЕ передається.

            Також є захист від одиничних
            нереалістичних стрибків.
            """

            nonlocal curr_alt
            nonlocal max_alt
            nonlocal last_valid_alt
            nonlocal last_alt_timestamp

            if new_alt is None:
                return

            try:
                new_alt = float(new_alt)

            except (TypeError, ValueError):
                return

            if math.isnan(new_alt) or math.isinf(new_alt):
                return

            # ------------------------------------------------
            # Від'ємна висота не потрібна
            # ------------------------------------------------

            new_alt = max(0.0, new_alt)


            # ------------------------------------------------
            # Захист від фізично нереальних значень
            # ------------------------------------------------

            if new_alt > 1000.0:
                return


            # ------------------------------------------------
            # Фільтр стрибків
            # ------------------------------------------------

            if (
                last_alt_timestamp is not None
                and timestamp is not None
            ):

                dt = timestamp - last_alt_timestamp

                if dt > 0:

                    # Максимально допустима зміна висоти.
                    #
                    # 30 м/с достатньо для того,
                    # щоб не обрізати нормальний політ,
                    # але прибирати одиничні помилкові значення.

                    max_change = max(
                        30.0 * dt,
                        3.0
                    )

                    if abs(new_alt - last_valid_alt) > max_change:
                        return


            # ------------------------------------------------
            # Посадка
            # ------------------------------------------------

            if new_alt < 0.5:
                new_alt = 0.0


            # ------------------------------------------------
            # Записуємо поточну висоту
            # ------------------------------------------------

            curr_alt = new_alt

            last_valid_alt = new_alt


            if timestamp is not None:
                last_alt_timestamp = timestamp


            # ------------------------------------------------
            # MAX ALTITUDE
            # ------------------------------------------------

            if curr_alt > max_alt:
                max_alt = curr_alt


        # ====================================================
        # ADD EVENT
        # ====================================================

        def add_event(
            text,
            t_stamp,
            mode,
            is_error=False
        ):

            # ВАЖЛИВО:
            #
            # Не перераховуємо висоту заново.
            #
            # Timeline використовує вже перевірену
            # основну висоту curr_alt.

            display_alt = curr_alt


            raw_timeline.append({

                "timestamp": t_stamp or 0,

                "mode": mode,

                "alt": f"{round(display_alt, 1)} м",

                "dist": (
                    f"{round(curr_dist, 1)} м"
                    if curr_dist > 0
                    else "0.0 м"
                ),

                "volt": (
                    round(curr_voltage, 1)
                    if curr_voltage > 0
                    else "—"
                ),

                "curr": (
                    f"{round(curr_amp, 1)} А"
                    if curr_amp > 0
                    else "0.0 А"
                ),

                "rssi": (
                    f"{curr_rssi_pct}%"
                    if curr_rssi_pct > 0
                    else "—"
                ),

                "dbm": (
                    f"{round(curr_dbm)} dBm"
                    if curr_dbm != 0
                    else "—"
                ),

                "text": text,

                "isError": is_error
            })


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


            # =================================================
            # TIMESTAMP
            # =================================================

            t_stamp = getattr(
                msg,
                "_timestamp",
                0.0
            )

            if t_stamp > 0:

                current_timestamp = t_stamp

                if first_timestamp is None:
                    first_timestamp = t_stamp


            # =================================================
            # 1. HEARTBEAT
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
                    # РЕЖИМ
                    # -----------------------------------------

                    if (
                        new_mode
                        and new_mode != current_mode
                    ):

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

                        # Фіксуємо барометричний нуль
                        # в момент запуску.

                        if latest_baro_alt is not None:
                            ground_baro_alt = latest_baro_alt


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

                        # Після посадки невеликий залишок
                        # висоти прибираємо.

                        if curr_alt < 2.0:
                            curr_alt = 0.0
                            last_valid_alt = 0.0


                        add_event(
                            "🔴 Двигуни зупинено",
                            current_timestamp,
                            current_mode
                        )

                        was_armed = False


            # =================================================
            # 2. SYS_STATUS
            # =================================================

            elif msg_type == "SYS_STATUS":

                volt = msg.voltage_battery / 1000.0

                curr = msg.current_battery / 100.0


                # -----------------------------------------
                # НАПРУГА
                # -----------------------------------------

                if volt > 5.0:

                    if (
                        curr_voltage > 5.0
                        and curr_voltage < 22.0
                        and volt > 24.5
                    ):

                        reboot_or_second_battery = True

                        # Новий політ / новий нуль
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


                    if volt < min_voltage:
                        min_voltage = volt


                # -----------------------------------------
                # СТРУМ
                # -----------------------------------------

                if curr >= 0:

                    curr_amp = curr

                    if curr > max_current:
                        max_current = curr


                # -----------------------------------------
                # LAND
                # -----------------------------------------

                if (
                    current_mode == "LAND"
                    and voltage_at_land_mode is None
                ):

                    voltage_at_land_mode = volt


                # -----------------------------------------
                # LOITER NAV QUALITY
                # -----------------------------------------

                if current_mode == "LOITER":

                    vnav_val = (
                        getattr(msg, "load", 0)
                        / 10.0
                    )

                    if vnav_val > 0:

                        vnav_samples += 1

                        if vnav_val < vnav_quality_min_loiter:
                            vnav_quality_min_loiter = vnav_val

                        if vnav_val > vnav_quality_max_loiter:
                            vnav_quality_max_loiter = vnav_val


            # =================================================
            # 3. VFR_HUD
            # =================================================

            elif msg_type == "VFR_HUD":

                latest_baro_alt = float(msg.alt)


                # -----------------------------------------
                # Барометричний нуль
                # -----------------------------------------

                if not is_currently_armed:

                    ground_baro_alt = latest_baro_alt


                if ground_baro_alt is None:

                    ground_baro_alt = latest_baro_alt


                # -----------------------------------------
                # Відносна барометрична висота
                # -----------------------------------------

                baro_rel_alt = max(
                    0.0,
                    latest_baro_alt
                    - ground_baro_alt
                )


                # -----------------------------------------
                # Барометр — тільки резерв.
                #
                # Якщо EKF ще не прийшов,
                # використовуємо його.
                # -----------------------------------------

                if ekf_alt is None:

                    update_flight_altitude(
                        baro_rel_alt,
                        current_timestamp
                    )


                # -----------------------------------------
                # SPEED
                # -----------------------------------------

                if msg.groundspeed > max_speed:
                    max_speed = msg.groundspeed


                # -----------------------------------------
                # THROTTLE
                # -----------------------------------------

                if msg.throttle > max_throttle:
                    max_throttle = msg.throttle


            # =================================================
            # 4. ALTITUDE
            # =================================================

            elif msg_type == "ALTITUDE":

                if hasattr(
                    msg,
                    "altitude_relative"
                ):

                    alt_rel = msg.altitude_relative


                    if (
                        not math.isnan(alt_rel)
                        and not math.isinf(alt_rel)
                    ):

                        alt_rel = max(
                            0.0,
                            alt_rel
                        )


                        # Використовуємо тільки якщо
                        # EKF і baro ще недоступні.

                        if (
                            ekf_alt is None
                            and baro_rel_alt is None
                        ):

                            update_flight_altitude(
                                alt_rel,
                                current_timestamp
                            )


            # =================================================
            # 5. LOCAL POSITION
            # =================================================

            elif msg_type in [
                "LOCAL_POSITION_NED",
                "POSITION_TARGET_LOCAL_NED"
            ]:

                x = getattr(
                    msg,
                    "x",
                    0.0
                )

                y = getattr(
                    msg,
                    "y",
                    0.0
                )


                d_val = math.sqrt(
                    x * x
                    + y * y
                )


                if 0.0 <= d_val <= 10000.0:

                    curr_dist = d_val


                    if curr_dist > max_dist:
                        max_dist = curr_dist


            # =================================================
            # 6. RANGEFINDER
            # =================================================

            elif msg_type in [
                "RANGEFINDER",
                "DISTANCE_SENSOR"
            ]:

                rf_dist = getattr(
                    msg,
                    "distance",
                    0
                )


                if msg_type == "DISTANCE_SENSOR":

                    rf_dist = (
                        getattr(
                            msg,
                            "current_distance",
                            0
                        )
                        / 100.0
                    )


                if (
                    0.1 <= rf_dist <= 50.0
                    and not rangefinder_failed_flag
                ):

                    has_rangefinder = True

                    # -------------------------------------
                    # ВАЖЛИВО:
                    #
                    # НЕ робимо:
                    #
                    # curr_alt = rf_dist
                    #
                    # Rangefinder — окремий датчик.
                    # Він НЕ замінює висоту EKF.
                    # -------------------------------------

                    rangefinder_alt = rf_dist


                    if rf_dist > max_rf_alt:
                        max_rf_alt = rf_dist


            # =================================================
            # 7. RC_CHANNELS
            # =================================================

            elif msg_type == "RC_CHANNELS":

                if hasattr(msg, "rssi"):

                    if 0 < msg.rssi < 255:

                        if msg.rssi < min_rssi:
                            min_rssi = msg.rssi


                        curr_rssi_pct = round(
                            (msg.rssi / 254.0)
                            * 100
                        )


                chans = [

                    msg.chan1_raw,
                    msg.chan2_raw,
                    msg.chan3_raw,
                    msg.chan4_raw,

                    getattr(
                        msg,
                        "chan5_raw",
                        0
                    ),

                    getattr(
                        msg,
                        "chan6_raw",
                        0
                    ),

                    getattr(
                        msg,
                        "chan7_raw",
                        0
                    ),

                    getattr(
                        msg,
                        "chan8_raw",
                        0
                    )
                ]


                for ch_num in range(1, 9):

                    val = chans[ch_num - 1]


                    if 800 < val < 2200:

                        if val < rc_min[ch_num]:
                            rc_min[ch_num] = val


                        if val > rc_max[ch_num]:
                            rc_max[ch_num] = val


                        if ch_num >= 5:

                            prev = last_rc_state[ch_num]


                            if (
                                prev > 0
                                and abs(val - prev) > 250
                            ):

                                state_str = (
                                    "АКТИВНО"
                                    if val > 1600
                                    else (
                                        "СЕРЕДНЄ"
                                        if 1300 <= val <= 1600
                                        else "ВИМК"
                                    )
                                )


                                add_event(
                                    f"🎮 CH{ch_num} переведено "
                                    f"в {state_str} ({val} us)",
                                    current_timestamp,
                                    current_mode
                                )


                            last_rc_state[ch_num] = val


            # =================================================
            # 8. RADIO
            # =================================================

            elif msg_type in [
                "RADIO",
                "RADIO_STATUS"
            ]:

                telem_rssi_raw = msg.rssi

                telem_remrssi_raw = msg.remrssi


                dbm_val = parse_dbm(
                    msg.rssi
                )


                curr_dbm = dbm_val


                if (
                    min_dbm == 0
                    or dbm_val < min_dbm
                ):

                    min_dbm = dbm_val


            # =================================================
            # 9. ATTITUDE
            # =================================================

            elif msg_type == "ATTITUDE":

                r_deg = abs(
                    math.degrees(msg.roll)
                )

                p_deg = abs(
                    math.degrees(msg.pitch)
                )


                if r_deg > max_roll:
                    max_roll = r_deg


                if p_deg > max_pitch:
                    max_pitch = p_deg


            # =================================================
            # 10. GLOBAL_POSITION_INT
            # =================================================

            elif msg_type == "GLOBAL_POSITION_INT":

                # -----------------------------------------
                # GPS
                # -----------------------------------------

                if (
                    msg.lat != 0
                    or msg.lon != 0
                ):

                    has_gps = True


                # -----------------------------------------
                # EKF RELATIVE ALTITUDE
                # -----------------------------------------

                if hasattr(
                    msg,
                    "relative_alt"
                ):

                    rel_g = (
                        msg.relative_alt
                        / 1000.0
                    )


                    if (
                        not math.isnan(rel_g)
                        and not math.isinf(rel_g)
                        and 0.0 <= rel_g <= 1000.0
                    ):

                        # Це наше головне джерело.
                        ekf_alt = rel_g


                        update_flight_altitude(
                            ekf_alt,
                            current_timestamp
                        )


                # -----------------------------------------
                # LOITER ORIGIN
                # -----------------------------------------

                if current_mode == "LOITER":

                    if (
                        msg.lat != 0
                        or msg.lon != 0
                    ):

                        loiter_has_origin = True

                    elif (
                        msg.lat == 0
                        and msg.lon == 0
                        and not loiter_has_origin
                    ):

                        loiter_origin_failed = True


            # =================================================
            # 11. VIBRATION
            # =================================================

            elif msg_type == "VIBRATION":

                if msg.vibration_x > max_vib_x:
                    max_vib_x = msg.vibration_x


                if msg.vibration_y > max_vib_y:
                    max_vib_y = msg.vibration_y


                if msg.vibration_z > max_vib_z:
                    max_vib_z = msg.vibration_z


                clip_count = max(
                    clip_count,
                    msg.clipping_0,
                    msg.clipping_1,
                    msg.clipping_2
                )


            # =================================================
            # 12. STATUSTEXT
            # =================================================

            elif msg_type == "STATUSTEXT":

                try:

                    txt = (
                        msg.text.decode("utf-8")
                        if isinstance(
                            msg.text,
                            bytes
                        )
                        else msg.text
                    )


                    if (
                        "No rangefinder" in txt
                        or "VISP: No rangefinder" in txt
                    ):

                        rangefinder_failed_flag = True


                    is_err = (
                        msg.severity <= 4
                    )


                    prefix = (
                        "⚠️ ПОМИЛКА: "
                        if is_err
                        else "ℹ️ "
                    )


                    add_event(
                        f"{prefix}{txt}",
                        current_timestamp,
                        current_mode,
                        is_err
                    )


                except Exception:
                    pass


        # ====================================================
        # ЗАВЕРШЕННЯ
        # ====================================================

        if min_voltage == 999.0:
            min_voltage = 0.0


        if start_voltage is None:
            start_voltage = 0.0


        if vnav_quality_min_loiter == 999:
            vnav_quality_min_loiter = 0


        # ====================================================
        # FINAL MAX ALTITUDE
        # ====================================================

        # ВАЖЛИВО:
        #
        # НЕ використовуємо max_rf_alt.
        #
        # MAX ALTITUDE = максимальна висота польоту
        # за EKF / baro.
        #
        # Rangefinder — окремий параметр.

        final_max_altitude = max_alt


        # ====================================================
        # DURATION
        # ====================================================

        duration_sec = int(
            current_timestamp
            - first_timestamp
        ) if (
            first_timestamp
            and current_timestamp
        ) else 0


        mins, secs = divmod(
            duration_sec,
            60
        )


        # ====================================================
        # TIMELINE
        # ====================================================

        timeline = []


        for ev in sorted(
            raw_timeline,
            key=lambda x: x["timestamp"]
        ):

            t_sec = int(
                ev["timestamp"]
                - (first_timestamp or 0)
            )


            t_m, t_s = divmod(
                max(0, t_sec),
                60
            )


            timeline.append({

                "time": f"{t_m:02d}:{t_s:02d}",

                "mode": ev["mode"],

                "alt": ev["alt"],

                "dist": ev["dist"],

                "volt": ev["volt"],

                "curr": ev["curr"],

                "rssi": ev["rssi"],

                "dbm": ev["dbm"],

                "text": ev["text"],

                "isError": ev["isError"]
            })


        # ====================================================
        # RSSI
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
            ", ".join(flight_modes)
            if flight_modes
            else "Невідомо"
        )


        # ====================================================
        # AI ВИСНОВОК
        # ====================================================

        ai_alerts = []

        is_critical = False


        # ====================================================
        # БАТАРЕЯ / ЗАМІНА
        # ====================================================

        if reboot_or_second_battery:

            ai_alerts.append(
                "ℹ️ <b>Зафіксовано заміну батареї:</b> "
                "Напруга зросла до 25.2V. "
                "Нуль висоти перекалібровано."
            )


        # ====================================================
        # RANGEFINDER
        # ====================================================

        if rangefinder_failed_flag:

            ai_alerts.append(
                "📡 <b>Відвалився далекомір "
                "(Rangefinder):</b> "
                "Система VISP втратила зв'язок "
                "із сенсором. "
                "Висота розрахована за EKF/барометром."
            )


        # ====================================================
        # LOITER
        # ====================================================

        if "LOITER" in flight_modes:

            if (
                loiter_origin_failed
                and not loiter_has_origin
            ):

                ai_alerts.append(
                    "👁 <b>Збій оптичного захоплення "
                    "в Loiter:</b> "
                    "Модуль не зміг відбити точку 0.0.0.0."
                )

                is_critical = True


            elif loiter_has_origin:

                ai_alerts.append(
                    "✅ <b>Оптична навігація в Loiter:</b> "
                    "Точку 0.0.0.0 успішно зафіксовано оптикою."
                )


            if vnav_samples > 0:

                if vnav_quality_min_loiter < 40:

                    ai_alerts.append(
                        f"⚠️ <b>Низька якість "
                        f"Оптичної Навігації:</b> "
                        f"Якість розпізнавання падала "
                        f"до {round(vnav_quality_min_loiter)}%."
                    )

                else:

                    ai_alerts.append(
                        f"👁 <b>Якість Оптичної Навігації:</b> "
                        f"{round(vnav_quality_min_loiter)}%–"
                        f"{round(vnav_quality_max_loiter)}%."
                    )


        # ====================================================
        # БАТАРЕЯ
        # ====================================================

        if 0 < min_voltage <= 16.8:

            is_critical = True


            if land_mode_triggered:

                ai_alerts.append(
                    f"🪫 <b>Посадка за розрядом:</b> "
                    f"Напруга впала до "
                    f"{round(min_voltage, 1)}V "
                    f"(поріг 16.8V). "
                    f"Автопілот примусово перевів "
                    f"борт у LAND."
                )

            else:

                ai_alerts.append(
                    f"🚨 <b>Критичний розряд без LAND:</b> "
                    f"Напруга просіла до "
                    f"{round(min_voltage, 1)}V."
                )


        elif 16.8 < min_voltage < 18.0:

            ai_alerts.append(
                f"🔋 <b>Глибока просадка:</b> "
                f"Напруга падала до "
                f"{round(min_voltage, 1)}V."
            )


        # ====================================================
        # RADIO
        # ====================================================

        if (
            min_dbm <= -128
            or (
                telem_rssi_raw == 0
                and telem_remrssi_raw == 0
            )
        ):

            ai_alerts.append(
                "📡 <b>-128 dBm: "
                "Втрата відео та телеметрії.</b> "
                "Повний розрив каналу зв'язку."
            )

            is_critical = True


        elif -128 < min_dbm <= -90:

            ai_alerts.append(
                f"📡 <b>{round(min_dbm)} dBm: "
                "Втрата відеосигналу "
                "(-90...-100 dBm).</b> "
                "Телеметрія була присутня."
            )


        elif -90 < min_dbm <= -85:

            ai_alerts.append(
                f"📶 <b>{round(min_dbm)} dBm: "
                "Підсипання відео "
                "(-85...-90 dBm).</b> "
                "Граничний рівень відеосигналу."
            )


        elif min_dbm < 0:

            ai_alerts.append(
                f"✅ <b>Рівень відео та телеметрії "
                f"в нормі:</b> "
                f"Мінімальний сигнал "
                f"{round(min_dbm)} dBm."
            )


        # ====================================================
        # LOG ENDED WHILE ARMED
        # ====================================================

        if was_armed:

            ai_alerts.append(
                "❗️ <b>Обрив логу під навантаженням:</b> "
                "Файл закінчився при працюючих двигунах."
            )

            is_critical = True


        # ====================================================
        # ATTITUDE
        # ====================================================

        if (
            max_roll > 80
            or max_pitch > 80
        ):

            ai_alerts.append(
                f"🔄 <b>Перевороти в повітрі:</b> "
                f"Нахил "
                f"{max(max_roll, max_pitch)}°."
            )

            is_critical = True


        # ====================================================
        # VERDICT
        # ====================================================

        if is_critical:

            ai_verdict = (
                "⚠️ БОРТ ВТРАЧЕНО "
                "АБО СТАЛАСЯ АВАРІЙНА СИТУАЦІЯ:"
            )

        elif len(ai_alerts) > 0:

            ai_verdict = (
                "📊 РЕЗУЛЬТАТИ АНАЛІЗУ ПОЛЬОТУ:"
            )

        else:

            ai_verdict = (
                "✅ Політ пройшов у штатному режимі."
            )


        # ====================================================
        # RETURN
        # ====================================================

        return {

            "success": True,


            # =================================================
            # AI
            # =================================================

            "ai": {

                "verdict": ai_verdict,

                "isCritical": is_critical,

                "alerts": ai_alerts
            },


            # =================================================
            # FLIGHT
            # =================================================

            "flight": {

                "durationText": (
                    f"{mins} хв {secs} с"
                ),

                # -----------------------------------------
                # ТЕПЕР MAX ALTITUDE БЕРЕТЬСЯ
                # ТІЛЬКИ З ОСНОВНОЇ ВИСОТИ
                # -----------------------------------------

                "maxAltitude": round(
                    final_max_altitude,
                    1
                ),

                "maxSpeed": round(
                    max_speed,
                    1
                ),

                "maxRoll": round(
                    max_roll,
                    1
                ),

                "maxPitch": round(
                    max_pitch,
                    1
                ),

                "modes": modes_str,

                "msgCount": message_count
            },


            # =================================================
            # BATTERY
            # =================================================

            "battery": {

                "armVoltage": round(
                    start_voltage,
                    2
                ),

                "minVoltage": round(
                    min_voltage,
                    2
                ),

                "maxCurrent": round(
                    max_current,
                    2
                ),

                "voltageSag": round(
                    max(
                        0,
                        start_voltage
                        - min_voltage
                    ),
                    2
                )
            },


            # =================================================
            # RADIO
            # =================================================

            "radio": {

                "rssi": (
                    f"{rssi_percent}%"
                    if min_rssi != 255
                    else "Немає"
                ),

                "maxThrottle": (
                    f"{max_throttle}%"
                ),

                "hasGps": (
                    "GPS Присутній"
                    if has_gps
                    else "Без GPS (Оптична навігація)"
                ),

                "telemRssi": (
                    f"{round(min_dbm)} dBm"
                    if min_dbm != 0
                    else "—"
                ),

                "telemRemRssi": (
                    str(telem_remrssi_raw)
                    if telem_remrssi_raw is not None
                    else "—"
                )
            },


            # =================================================
            # HEALTH
            # =================================================

            "health": {

                "vibX": round(
                    max_vib_x,
                    1
                ),

                "vibY": round(
                    max_vib_y,
                    1
                ),

                "vibZ": round(
                    max_vib_z,
                    1
                ),

                "clipping": clip_count,

                "maxTemp": round(
                    max_temp,
                    1
                ),

                "engineLoadLoiter": (
                    f"{round(vnav_quality_min_loiter)}% – "
                    f"{round(vnav_quality_max_loiter)}%"
                    if vnav_samples > 0
                    else "Немає даних"
                )
            },


            # =================================================
            # RANGEFINDER
            # =================================================

            "rangefinder": {

                "available": has_rangefinder,

                "current": (
                    round(
                        rangefinder_alt,
                        1
                    )
                    if rangefinder_alt is not None
                    else None
                ),

                "max": round(
                    max_rf_alt,
                    1
                ),

                "failed": rangefinder_failed_flag
            },


            # =================================================
            # TIMELINE
            # =================================================

            "timeline": timeline
        }


    finally:

        try:

            os.unlink(
                temp.name
            )

        except Exception:

            pass
