# sbp2gpx

Locosys NaviGPS GT-31/BGT-31 Binary Datalog File Converter to GPX/CSV/JSON, with time offset option

## The SPB format

Based on navilink_decode_logpoint from [GPSBabel](https://www.gpsbabel.org/):

```
struct layout (32 bytes):
    0:  HDOP (1 byte, uint8) - value * 0.2
    1:  Satellite count (1 byte, uint8)
  2-3:  UTC sub-second counter (2 bytes, uint16 LE) - (value % 1000) = milliseconds
  4-7:  Packed date/time (4 bytes) - see _decode_sbp_datetime
 8-11:  Device info (constant per file, not per-point)
12-15:  Latitude (4 bytes, int32 LE) - value / 10,000,000 = degrees
16-19:  Longitude (4 bytes, int32 LE) - value / 10,000,000 = degrees
20-23:  Altitude (4 bytes, int32 LE) - value / 100 = meters
24-25:  Speed (2 bytes, uint16 LE) - value / 100 = m/s
26-27:  Course over ground (2 bytes, uint16 LE) - value / 100 = degrees
28-29:  Vertical speed (2 bytes, int16 LE) - value / 100 = m/s
   30:  SDOP (1 byte, uint8) - value * 0.036
   31:  VSDOP (1 byte, uint8) - value * 0.036
```

Some fields are discovered by AI.

## Vibecoding story

I think LLMs are well-suited for tasks like this:
there are no complex features,
the task isn't time-critical,
and writing such a simple converter program is rather boring.

First, I thought it would be enough to simply instruct the AI:
"Write a converter from SBP to GPX."
But it turned out that
`.SBP` isn’t a common format,
and the AI struggled to build the converter.

So I looked into *GPSBabel*
and found some `.C` and `.H` files with
file format struct in a folder labeled "deprecated/".
To my surprise, the AI interpreted the C code and
managed to create the converter.

Later, we noticed some fields were missing from the output fields
that had been marked as "unused" in the C struct.
We provided a reference GPX file converted from an `.SBP` file,
and the AI figured out how those unused fields should be converted.

Magic!

## Needs more testing

The utility has only been tested on a few files,
so there may be bugs.
Please help by reporting issues,
or feel free to submit a PR.
