#!/usr/bin/env python3
"""
Locosys NaviGPS GT-31/BGT-31 Binary Datalog Converter
Based on navilink_decode_logpoint from GPSBabel

This parser reads the SBP (SiRF Binary Protocol) format used by Locosys devices.
Record size: 32 bytes per logpoint

Human-approved vibecode (Claude Sonnet 4.5)
"""

import struct
import sys
import os
from datetime import datetime, timedelta
from typing import List, Dict, Optional, Tuple
import json
import csv
from dataclasses import dataclass, asdict
import xml.etree.ElementTree as ET
from xml.dom import minidom

# Constants
SBP_HEADER_SIZE = 64
HEADER_SKIP = 7
SBP_RECORD_LEN = 32  # Fixed record size

@dataclass
class SBPHeader:
    """SBP file header structure"""
    file_id: str
    firmware_version: str
    device_model: str
    serial_number: str

    def to_dict(self) -> Dict:
        return asdict(self)

@dataclass
class LogPoint:
    """SBP logpoint data structure - matches navilink_decode_logpoint"""
    timestamp: Optional[datetime]
    latitude: float
    longitude: float
    altitude: float
    speed: float      # km/h
    heading: float    # degrees
    hdop: float
    sdop: float
    vsdop: float
    satellites: int

    def to_dict(self) -> Dict:
        data = asdict(self)
        data['timestamp'] = self.timestamp.isoformat() if self.timestamp else None
        return data

class SBPParser:
    """Parser for Locosys SBP binary format"""

    def __init__(self, filename: str, debug: bool = False, tz_offset: int = 0):
        self.filename = filename
        self.debug = debug
        self.tz_offset = tz_offset
        if tz_offset == 0:
            self.tz_offset_postfix = "Z"
        else:
            self.tz_offset_postfix = f"{tz_offset:+03d}00"
        self.header: Optional[SBPHeader] = None
        self.records: List[LogPoint] = []

    def _adjusted_ts(self, ts: Optional[datetime]) -> Optional[datetime]:
        if ts is None or self.tz_offset == 0:
            return ts
        return ts + timedelta(hours=self.tz_offset)

    def read_file(self) -> Tuple[Optional[SBPHeader], List[LogPoint]]:
        """Read and parse the entire SBP file"""
        with open(self.filename, 'rb') as f:
            # Read header (first 64 bytes)
            header_data = f.read(SBP_HEADER_SIZE)
            if len(header_data) < SBP_HEADER_SIZE:
                raise ValueError(f"File too small")

            self.header = self._parse_header(header_data)

            # Read records (32 bytes each)
            record_num = 0
            while True:
                record_data = f.read(SBP_RECORD_LEN)
                if len(record_data) < SBP_RECORD_LEN:
                    break

                point = self._decode_logpoint(record_data, record_num)
                if point:
                    self.records.append(point)
                record_num += 1

        return self.header, self.records

    def _parse_header(self, data: bytes) -> SBPHeader:
        """Parse the 64-byte header"""
        header_len = struct.unpack('<H', data[0:2])[0]

        file_info = {}
        if header_len > HEADER_SKIP:
            file_id_data = data[HEADER_SKIP:min(header_len, SBP_HEADER_SIZE)]
            # Remove null, 0xFF padding, and non-printable control characters
            decoded = file_id_data.replace(b'\x00', b'').replace(b'\xff', b'').decode('ascii', errors='ignore')
            decoded = ''.join(c for c in decoded if c.isprintable()).strip()
            if decoded:
                file_info['file_id'] = decoded

                # Format: MODEL,SERIAL,COUNT,FIRMWARE (comma-separated)
                parts = [p.strip().rstrip(';,') for p in decoded.split(',')]
                if len(parts) >= 1:
                    file_info['model'] = parts[0]
                if len(parts) >= 2:
                    file_info['serial'] = parts[1]
                if len(parts) >= 4:
                    file_info['firmware'] = parts[3]

        if self.debug:
            print(f"Header length: {header_len}")
            print(f"File ID: {file_info.get('file_id', 'Unknown')}")

        return SBPHeader(
            file_id=file_info.get('file_id', 'Unknown'),
            firmware_version=file_info.get('firmware', 'Unknown'),
            device_model=file_info.get('model', 'Unknown'),
            serial_number=file_info.get('serial', 'Unknown')
        )

    def _decode_logpoint(self, data: bytes, record_num: int) -> Optional[LogPoint]:
        """
        Decode a 32-byte SBP logpoint record.

        Based on navilink_decode_logpoint from GPSBabel:

        struct layout (32 bytes):
        offset  0:     HDOP (1 byte, uint8) - value * 0.2
        offset  1:     Satellite count (1 byte, uint8)
        offset  2-3:   UTC sub-second counter (2 bytes, uint16 LE) - (value % 1000) = milliseconds
        offset  4-7:   Packed date/time (4 bytes) - see _decode_sbp_datetime
        offset  8-11:  Device info (constant per file, not per-point)
        offset 12-15:  Latitude (4 bytes, int32 LE) - value / 10,000,000 = degrees
        offset 16-19:  Longitude (4 bytes, int32 LE) - value / 10,000,000 = degrees
        offset 20-23:  Altitude (4 bytes, int32 LE) - value / 100 = meters
        offset 24-25:  Speed (2 bytes, uint16 LE) - value / 100 = m/s
        offset 26-27:  Course over ground (2 bytes, uint16 LE) - value / 100 = degrees
        offset 28-29:  Vertical speed (2 bytes, int16 LE) - value / 100 = m/s
        offset 30:     SDOP (1 byte, uint8) - value * 0.036
        offset 31:     VSDOP (1 byte, uint8) - value * 0.036
        """
        if len(data) != SBP_RECORD_LEN:
            return None

        if self.debug:
            print(f"\nRecord {record_num}: {data.hex()}")

        try:
            # Parse HDOP (offset 0)
            hdop_raw = data[0]
            hdop = hdop_raw * 0.2

            # Satellite count (offset 1)
            satellites = data[1]
            if satellites > 12:
                if self.debug:
                    print(f"  Warning: Invalid satellite count {satellites}, setting to 0")
                satellites = 0

            # UTC milliseconds (offset 2-3)
            utc_msec = struct.unpack('<H', data[2:4])[0]

            # Packed date/time (offset 4-7)
            timestamp = self._decode_sbp_datetime(data[4:8], utc_msec)

            # Latitude (offset 12-15)
            lat_raw = struct.unpack('<i', data[12:16])[0]
            latitude = lat_raw / 10000000.0

            # Longitude (offset 16-19)
            lon_raw = struct.unpack('<i', data[16:20])[0]
            longitude = lon_raw / 10000000.0

            # Altitude (offset 20-23) - in centimeters
            alt_raw = struct.unpack('<i', data[20:24])[0]
            altitude = alt_raw / 100.0  # Convert to meters

            # Speed (offset 24-25) - in 0.01 m/s
            speed_raw = struct.unpack('<H', data[24:26])[0]
            speed_ms = speed_raw / 100.0
            speed_kmh = speed_ms * 3.6

            # Course/Heading (offset 26-27) - in 0.01 degrees
            heading_raw = struct.unpack('<H', data[26:28])[0]
            heading = heading_raw / 100.0
            if heading > 360:
                heading = heading % 360

            # SDOP (offset 30), VSDOP (offset 31) - resolution 0.036
            sdop = data[30] * 0.036
            vsdop = data[31] * 0.036

            if self.debug:
                print(f"  HDOP: {hdop_raw} -> {hdop:.1f}")
                print(f"  Satellites: {satellites}")
                print(f"  UTC msec: {utc_msec}")
                print(f"  Timestamp: {timestamp}")
                print(f"  Lat raw: {lat_raw} -> {latitude:.6f}")
                print(f"  Lon raw: {lon_raw} -> {longitude:.6f}")
                print(f"  Alt raw: {alt_raw} -> {altitude:.1f}m")
                print(f"  Speed raw: {speed_raw} -> {speed_kmh:.1f}km/h")
                print(f"  Heading raw: {heading_raw} -> {heading:.1f}°")

            # Validate
            if -90 <= latitude <= 90 and -180 <= longitude <= 180:
                if latitude != 0 or longitude != 0:
                    return LogPoint(
                        timestamp=timestamp,
                        latitude=latitude,
                        longitude=longitude,
                        altitude=altitude,
                        speed=speed_kmh,
                        heading=heading,
                        hdop=hdop,
                        sdop=sdop,
                        vsdop=vsdop,
                        satellites=satellites
                    )
            elif self.debug:
                print(f"  Skipping - coordinates out of range")

        except Exception as e:
            if self.debug:
                print(f"  Error: {e}")
                import traceback
                traceback.print_exc()

        return None

    def _decode_sbp_datetime(self, data: bytes, msec: int) -> Optional[datetime]:
        """
        Decode packed SBP date/time.

        Based on decode_sbp_datetime_packed from GPSBabel:

        Packed_Date_Time_UTC format (32 bits):
        bits 31-22: year*12 + month (10 bits) - real year = year + 2000
        bits 21-17: day (5 bits)
        bits 16-12: hour (5 bits)
        bits 11-6: minute (6 bits)
        bits 5-0: second (6 bits)
        """
        try:
            if len(data) < 4:
                return None

            # Parse the 4-byte packed value
            packed = struct.unpack('<I', data)[0]

            # Extract fields according to GPSBabel implementation
            sec = packed & 0x3F
            minute = (packed >> 6) & 0x3F
            hour = (packed >> 12) & 0x1F
            day = (packed >> 17) & 0x1F
            month_year = (packed >> 22) & 0x3FF

            month = month_year % 12
            year = 2000 + (month_year // 12)

            # Validate fields
            if month < 1 or month > 12:
                if self.debug:
                    print(f"  Invalid month: {month}")
                return None

            if day < 1 or day > 31:
                if self.debug:
                    print(f"  Invalid day: {day}")
                return None

            if hour > 23:
                if self.debug:
                    print(f"  Invalid hour: {hour}")
                return None

            if minute > 59:
                if self.debug:
                    print(f"  Invalid minute: {minute}")
                return None

            if sec > 59:
                if self.debug:
                    print(f"  Invalid second: {sec}")
                return None

            # Create datetime
            dt = datetime(year, month, day, hour, minute, sec)

            # Add milliseconds - raw value is a running counter, take modulo 1000
            ms = msec % 1000
            if ms > 0:
                dt = dt + timedelta(milliseconds=ms)

            return dt

        except Exception as e:
            if self.debug:
                print(f"  Time decode error: {e}")
            return None

    def _sanitize_string(self, text: str) -> str:
        """Sanitize string for XML output"""
        if not text:
            return ""
        # Remove any non-printable characters
        text = ''.join(char for char in text if char.isprintable() or char in '\n\r\t')
        # Escape XML special characters
        text = text.replace('&', '&amp;')
        text = text.replace('<', '&lt;')
        text = text.replace('>', '&gt;')
        text = text.replace('"', '&quot;')
        text = text.replace("'", '&apos;')
        return text

    def export_to_gpx(self, output_file: str):
        """Export to GPX format matching GPS-Speed reference"""
        if not self.records:
            print("No records to export")
            return

        # Create GPX root
        gpx = ET.Element('gpx')
        gpx.set('xmlns', 'https://www.gps-speed.com')
        gpx.set('creator', 'sbp2gpx')

        # Metadata block
        metadata = ET.SubElement(gpx, 'metadata')
        link = ET.SubElement(metadata, 'link')
        link.set('href', 'https://www.gps-speed.com')
        link_text = ET.SubElement(link, 'text')
        link_text.text = 'GPS-Results'
        if self.header:
            username = ET.SubElement(metadata, 'username')
            username.text = self.header.device_model if self.header.device_model != 'Unknown' else 'unknown'
            device = ET.SubElement(metadata, 'device')
            device.text = self.header.device_model if self.header.device_model != 'Unknown' else 'unknown'
            firmware = ET.SubElement(metadata, 'firmware')
            firmware.text = self.header.firmware_version if self.header.firmware_version != 'Unknown' else ''
            serialno = ET.SubElement(metadata, 'serialno')
            serialno.text = self.header.serial_number if self.header.serial_number != 'Unknown' else ''

        trk = ET.SubElement(gpx, 'trk')
        name = ET.SubElement(trk, 'name')
        name.text = 'exported tracks'

        trkseg = ET.SubElement(trk, 'trkseg')

        valid_points = 0
        for point in self.records:
            # Skip points with invalid coordinates
            if point.latitude == 0 and point.longitude == 0:
                continue

            trkpt = ET.SubElement(trkseg, 'trkpt',
                                  lat=f"{point.latitude:.9f}",
                                  lon=f"{point.longitude:.9f}")

            # Add altitude if valid
            if point.altitude != 0 and -1000 < point.altitude < 10000:
                ele = ET.SubElement(trkpt, 'ele')
                ele.text = f"{point.altitude:.2f}"

            # Add time with milliseconds if valid
            ts = self._adjusted_ts(point.timestamp)
            if ts:
                time_elem = ET.SubElement(trkpt, 'time')
                ms = ts.microsecond // 1000
                time_elem.text = ts.strftime(f'%Y-%m-%dT%H:%M:%S.{ms:03d}{self.tz_offset_postfix}')

            # Satellites as direct child
            if point.satellites > 0:
                sat = ET.SubElement(trkpt, 'sat')
                sat.text = str(point.satellites)

            # Speed in m/s as direct child
            if point.speed > 0:
                speed = ET.SubElement(trkpt, 'speed')
                speed.text = f"{point.speed / 3.6:.3f}"

            # Course over ground as direct child
            if point.heading > 0:
                cog = ET.SubElement(trkpt, 'cog')
                cog.text = f"{point.heading:.3f}"

            # HDOP as direct child
            if point.hdop > 0:
                hdop = ET.SubElement(trkpt, 'hdop')
                hdop.text = f"{point.hdop:.2f}"

            # SDOP as direct child
            if point.sdop > 0:
                sdop = ET.SubElement(trkpt, 'sdop')
                sdop.text = f"{point.sdop:.2f}"

            # VSDOP as direct child
            if point.vsdop > 0:
                vsdop = ET.SubElement(trkpt, 'vsdop')
                vsdop.text = f"{point.vsdop:.2f}"

            # Fix: SiRF message ID 43 (Geodetic Navigation Data), constant for Locosys SBP
            fix = ET.SubElement(trkpt, 'fix')
            fix.text = "43"

            valid_points += 1

        if valid_points == 0:
            print("No valid points to export")
            return

        # Convert to string and pretty print
        xml_str = ET.tostring(gpx, encoding='utf-8', method='xml')

        # Parse with minidom for pretty printing
        try:
            dom = minidom.parseString(xml_str)
            pretty_xml = dom.toprettyxml(indent="  ")

            # Remove XML declaration if it causes issues
            lines = pretty_xml.split('\n')
            # Ensure UTF-8 encoding declaration
            if lines and '<?xml' in lines[0]:
                lines[0] = '<?xml version="1.0" encoding="UTF-8"?>'
            pretty_xml = '\n'.join(lines)

            with open(output_file, 'w', encoding='utf-8') as f:
                f.write(pretty_xml)

            print(f"Exported {valid_points} track points to {output_file}")
        except Exception as e:
            # Fallback: write without pretty printing
            with open(output_file, 'wb') as f:
                f.write(xml_str)
            print(f"Exported {valid_points} track points to {output_file} (basic format)")

    def export_to_csv(self, output_file: str):
        """Export to CSV"""
        if not self.records:
            print("No records to export")
            return

        with open(output_file, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'latitude', 'longitude', 'altitude_m', 'speed_kmh',
                'heading_deg', 'hdop', 'satellites'
            ])

            for point in self.records:
                # Skip points with no coordinates
                if point.latitude == 0 and point.longitude == 0:
                    continue

                ts = self._adjusted_ts(point.timestamp)
                writer.writerow([
                    ts.strftime('%Y-%m-%d %H:%M:%S') if ts else '',
                    f"{point.latitude:.7f}",
                    f"{point.longitude:.7f}",
                    f"{point.altitude:.1f}" if point.altitude != 0 else '',
                    f"{point.speed:.1f}" if point.speed > 0 else '',
                    f"{point.heading:.1f}" if point.heading > 0 else '',
                    f"{point.hdop:.1f}" if point.hdop > 0 else '',
                    point.satellites if point.satellites > 0 else ''
                ])

        print(f"Exported {len(self.records)} records to {output_file}")

    def export_to_json(self, output_file: str):
        """Export to JSON"""
        def point_dict(p):
            d = p.to_dict()
            ts = self._adjusted_ts(p.timestamp)
            d['timestamp'] = ts.isoformat() if ts else None
            return d

        data = {
            'header': self.header.to_dict() if self.header else None,
            'record_count': len(self.records),
            'records': [point_dict(p) for p in self.records if p.latitude != 0 or p.longitude != 0]
        }

        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, default=str)

        print(f"Exported {len(self.records)} records to {output_file}")

    def print_summary(self):
        """Print summary"""
        print("\n=== File Info ===")
        if self.header:
            print(f"File ID: {self.header.file_id}")
            print(f"Device: {self.header.device_model}")
            print(f"Firmware: {self.header.firmware_version}")
            print(f"Serial: {self.header.serial_number}")

        print(f"\n=== Track Summary ===")
        print(f"Total records: {len(self.records)}")

        # Filter valid points
        valid_points = [p for p in self.records if p.latitude != 0 and p.longitude != 0]
        print(f"Valid positions: {len(valid_points)}")

        if valid_points:
            # Time range
            points_with_time = [p for p in valid_points if p.timestamp]
            if points_with_time:
                start_time = min(points_with_time, key=lambda p: p.timestamp).timestamp
                end_time = max(points_with_time, key=lambda p: p.timestamp).timestamp
                duration = end_time - start_time

                print(f"\nTime range:")
                print(f"  Start: {start_time.strftime('%Y-%m-%d %H:%M:%S')}")
                print(f"  End:   {end_time.strftime('%Y-%m-%d %H:%M:%S')}")
                print(f"  Duration: {duration.total_seconds():.0f} seconds ({duration.total_seconds() / 3600:.1f} hours)")
            else:
                print("\nTime range: No valid timestamps found")

            # Bounds
            min_lat = min(valid_points, key=lambda p: p.latitude).latitude
            max_lat = max(valid_points, key=lambda p: p.latitude).latitude
            min_lon = min(valid_points, key=lambda p: p.longitude).longitude
            max_lon = max(valid_points, key=lambda p: p.longitude).longitude

            print(f"\nBounds:")
            print(f"  Latitude: {min_lat:.6f} to {max_lat:.6f}")
            print(f"  Longitude: {min_lon:.6f} to {max_lon:.6f}")

            # Speed stats
            speeds = [p.speed for p in valid_points if p.speed > 0 and p.speed < 300]
            if speeds:
                print(f"\nSpeed stats (km/h):")
                print(f"  Max: {max(speeds):.1f}")
                print(f"  Avg: {sum(speeds) / len(speeds):.1f}")
                print(f"  Min: {min(speeds):.1f}")

            # Satellite stats
            sats = [p.satellites for p in valid_points if p.satellites > 0]
            if sats:
                print(f"\nSatellites:")
                print(f"  Max: {max(sats)}")
                print(f"  Avg: {sum(sats) / len(sats):.1f}")
                print(f"  Min: {min(sats)}")

            # HDOP stats
            hdops = [p.hdop for p in valid_points if p.hdop > 0]
            if hdops:
                print(f"\nHDOP:")
                print(f"  Max: {max(hdops):.1f}")
                print(f"  Avg: {sum(hdops) / len(hdops):.1f}")
                print(f"  Min: {min(hdops):.1f}")

def get_output_format(output_file: str) -> str:
    """Determine output format from file extension"""
    ext = os.path.splitext(output_file)[1].lower()
    format_map = {
        '.gpx': 'gpx',
        '.csv': 'csv',
        '.json': 'json',
        '.txt': 'csv',  # Default to CSV for .txt
    }
    return format_map.get(ext, 'json')  # Default to JSON

def main():
    """Main function with automatic format detection from file extension"""
    if len(sys.argv) < 2:
        print("sbp2gpx.py - Locosys NaviGPS GT-31/BGT-31 Binary Datalog File Converter")
        print("Repository: https://github.com/ern0/sbp2gpx/")
        print("\nUsage: sbp2gpx.py <input_file> [output_file] [-t=<offset>] [--debug]")
        print("\nOutput format is determined by output file extension:")
        print("  .gpx  - GPX format (GPS Exchange Format)")
        print("  .csv  - CSV format (Comma Separated Values)")
        print("  .json - JSON format (JavaScript Object Notation)")
        print("\nIf no output file is specified, prints summary only.")
        print("\nOptions:")
        print("  -t=<offset>          Hour offset added to all timestamps (e.g. -t=+2, -t=-1, default: 0)")
        print("  --timezone=<offset>  Same as -t")
        print("\nExamples:")
        print("  sbp2gpx.py track.sbp                    # Show summary")
        print("  sbp2gpx.py track.sbp track.gpx          # Export to GPX")
        print("  sbp2gpx.py track.sbp track.gpx -t=+2    # Export with UTC+2 timestamps")
        print("  sbp2gpx.py track.sbp track.csv --debug  # Export to CSV with debug")
        print("  sbp2gpx.py track.sbp track.json         # Export to JSON")
        sys.exit(1)

    input_file = sys.argv[1]

    if not os.path.exists(input_file):
        print(f"Error: File '{input_file}' not found")
        sys.exit(1)

    # Parse arguments
    output_file = None
    debug = False
    tz_offset = 0

    i = 2
    while i < len(sys.argv):
        arg = sys.argv[i].lower()
        if arg == '--debug':
            debug = True
        elif arg.startswith('-t=') or arg.startswith('--timezone='):
            val = arg.split('=', 1)[1]
            try:
                tz_offset = int(val)
            except ValueError:
                print(f"Error: Invalid timezone offset '{val}'")
                sys.exit(1)
        elif not output_file and not arg.startswith('-'):
            output_file = arg
        i += 1

    print(f"Reading {input_file}...")
    parser = SBPParser(input_file, debug=debug, tz_offset=tz_offset)

    try:
        header, records = parser.read_file()

        if not records:
            print("No records found")
            return

        # Handle output based on whether output file is specified
        if output_file:
            # Determine format from extension
            output_format = get_output_format(output_file)

            if output_format == 'gpx':
                parser.export_to_gpx(output_file)
            elif output_format == 'csv':
                parser.export_to_csv(output_file)
            elif output_format == 'json':
                parser.export_to_json(output_file)
            else:
                print(f"Unknown format for extension: {output_file}")
                parser.print_summary()
        else:
            # No output file specified, just show summary
            parser.print_summary()

    except Exception as e:
        print(f"Error: {e}")
        if debug:
            import traceback
            traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
