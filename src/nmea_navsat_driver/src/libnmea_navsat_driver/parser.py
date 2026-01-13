# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.

import re
import math


def safe_float(field):
    try:
        return float(field)
    except ValueError:
        return float("NaN")


def safe_int(field):
    try:
        return int(field)
    except ValueError:
        return 0


def convert_latitude(lat_string, hemisphere):
    try:
        if not lat_string or not hemisphere:
            return float("NaN")

        lat = float(lat_string)
        lat_deg = int(lat / 100)
        lat_min = lat - (lat_deg * 100)
        lat_decimal = lat_deg + (lat_min / 60)

        if hemisphere == "S":
            lat_decimal = -lat_decimal

        return lat_decimal
    except ValueError:
        return float("NaN")


def convert_longitude(lon_string, hemisphere):
    try:
        if not lon_string or not hemisphere:
            return float("NaN")

        lon = float(lon_string)
        lon_deg = int(lon / 100)
        lon_min = lon - (lon_deg * 100)
        lon_decimal = lon_deg + (lon_min / 60)

        if hemisphere == "W":
            lon_decimal = -lon_decimal

        return lon_decimal
    except ValueError:
        return float("NaN")


def convert_time(time_string):
    try:
        if not time_string:
            return float("NaN")

        hours = int(time_string[0:2])
        minutes = int(time_string[2:4])
        seconds = float(time_string[4:])

        return hours * 3600 + minutes * 60 + seconds
    except (ValueError, IndexError):
        return float("NaN")


def convert_status_flag(status_flag):
    if status_flag == "A":
        return True
    else:
        return False


def convert_knots_to_mps(knots):
    return knots * 0.514444


def parse_GPGGA(nmea_sentence):
    fields = nmea_sentence.split(",")

    if len(fields) < 15:
        return None

    try:
        time = convert_time(fields[1])
        latitude = convert_latitude(fields[2], fields[3])
        longitude = convert_longitude(fields[4], fields[5])

        fix_quality = safe_int(fields[6])
        num_satellites = safe_int(fields[7])
        hdop = safe_float(fields[8])
        altitude = safe_float(fields[9])

        return {
            "sentence_type": "GGA",
            "utc_time": time,
            "latitude": latitude,
            "longitude": longitude,
            "fix_quality": fix_quality,
            "num_satellites": num_satellites,
            "hdop": hdop,
            "altitude": altitude,
        }
    except (ValueError, IndexError):
        return None


def parse_GPRMC(nmea_sentence):
    fields = nmea_sentence.split(",")

    if len(fields) < 12:
        return None

    try:
        time = convert_time(fields[1])
        status = convert_status_flag(fields[2])
        latitude = convert_latitude(fields[3], fields[4])
        longitude = convert_longitude(fields[5], fields[6])

        speed_knots = safe_float(fields[7])
        speed_mps = convert_knots_to_mps(speed_knots)

        track = safe_float(fields[8])
        date_string = fields[9]

        return {
            "sentence_type": "RMC",
            "utc_time": time,
            "fix_valid": status,
            "latitude": latitude,
            "longitude": longitude,
            "speed": speed_mps,
            "track": track,
            "date": date_string,
        }
    except (ValueError, IndexError):
        return None


def parse_GPVTG(nmea_sentence):
    fields = nmea_sentence.split(",")

    if len(fields) < 9:
        return None

    try:
        track_true = safe_float(fields[1])
        track_magnetic = safe_float(fields[3])
        speed_knots = safe_float(fields[5])
        speed_kph = safe_float(fields[7])

        speed_mps = convert_knots_to_mps(speed_knots)

        return {"sentence_type": "VTG", "track": track_true, "speed": speed_mps}
    except (ValueError, IndexError):
        return None


def parse_PQTMSENMSG(sentence):
    """Parse PQTMSENMSG message for IMU raw data"""
    fields = sentence.split(",")

    if len(fields) < 10:
        return None

    try:
        last_field = fields[9].split("*")[0] if "*" in fields[9] else fields[9]

        data = {
            "sentence_type": "PQTMSENMSG",
            "msg_type": safe_int(fields[1]),
            "timestamp_ms": safe_int(fields[2]),
            "imu_temp_c": safe_float(fields[3]),
            "gyro_x_deg": safe_float(fields[4]),
            "gyro_y_deg": safe_float(fields[5]),
            "gyro_z_deg": safe_float(fields[6]),
            "acc_x_g": safe_float(fields[7]),
            "acc_y_g": safe_float(fields[8]),
            "acc_z_g": safe_float(last_field),
        }
        return data
    except (ValueError, IndexError):
        return None


def parse_PQTMDRPVA(sentence):
    """Parse PQTMDRPVA message for INS position, velocity, and attitude"""
    fields = sentence.split(",")

    if len(fields) < 16:
        return None

    try:
        last_field = fields[15].split("*")[0] if "*" in fields[15] else fields[15]

        data = {
            "sentence_type": "PQTMDRPVA",
            "msg_version": safe_int(fields[1]),
            "timestamp_ms": safe_int(fields[2]),
            "utc_time": convert_time(fields[3]),
            "solution_type": safe_int(fields[4]),
            "latitude": safe_float(fields[5]),
            "longitude": safe_float(fields[6]),
            "altitude": safe_float(fields[7]),
            "separation": safe_float(fields[8]),
            "vel_north": safe_float(fields[9]),
            "vel_east": safe_float(fields[10]),
            "vel_down": safe_float(fields[11]),
            "speed": safe_float(fields[12]),
            "roll_deg": safe_float(fields[13]),
            "pitch_deg": safe_float(fields[14]),
            "heading_deg": safe_float(last_field),
        }
        return data
    except (ValueError, IndexError):
        return None


def parse_nmea_sentence(nmea_sentence):
    """Parse an NMEA sentence and return a dictionary of the data."""
    if not nmea_sentence or not nmea_sentence.startswith("$"):
        return None

    parts = nmea_sentence.split(",")
    if len(parts) < 1:
        return None

    sentence_id = parts[0][1:]

    if sentence_id.endswith("GGA"):
        return parse_GPGGA(nmea_sentence)
    elif sentence_id.endswith("RMC"):
        return parse_GPRMC(nmea_sentence)
    elif sentence_id.endswith("VTG"):
        return parse_GPVTG(nmea_sentence)
    elif sentence_id == "PQTMSENMSG":
        return parse_PQTMSENMSG(nmea_sentence)
    elif sentence_id == "PQTMDRPVA":
        return parse_PQTMDRPVA(nmea_sentence)

    return None
