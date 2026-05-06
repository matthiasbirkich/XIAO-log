# Arduino BLE logger for XIAO nRF52840 Sense + XIAO Logger HAT

This contribution adds an Arduino/Bluefruit logger implementation for the Seeed Studio XIAO nRF52840 Sense used together with the XIAO Logger HAT.

## Features

- PCF8563 RTC alarm wake on `D0`
- Reed-switch / magnetic BLE wake on `D1`
- SHT40 temperature and relative humidity
- BH1750 light measurement
- Battery voltage logging via the XIAO Logger HAT voltage divider on `A3`
- HAT measurement rail / divider enable via `D10`
- Onboard XIAO nRF52840 Sense LSM6DS3 acceleration
- Deployment metadata entered once over BLE:
  - logger ID
  - salinity in PSU
  - GPS latitude / longitude
- Text CSV export over BLE through nRF Connect
- Pyto / Thonny Python converter for nRF Connect exports to clean `.xlsx`

## Tested setup

- Board: Seeed Studio XIAO nRF52840 Sense
- Arduino board package: Seeed nRF52 Boards, non-mbed
- BLE library: Bluefruit API from the non-mbed nRF52 core
- BLE client: nRF Connect on iPad
- Export post-processing: Pyto on iPad and Python/Thonny on PC

Do not select the mbed-enabled board package for this sketch, because the firmware uses `bluefruit.h`.

## BLE overview (nRF-connect App)

BLE device name:

```text
UW52840-CLEAN24
```

BLE service and characteristics:

```text
Service: A000
Command characteristic: A001
Status / CSV characteristic: A002
```

In nRF Connect, enable notifications on `A002` and write commands to `A001`.

## Typical deployment setup

```text
RTC_CHECK
IMU_CHECK
SET_ID=UW01_BREMERHAVEN
SET_SALINITY=31.8
SET_GPS=53.5390,8.5809
SET_TIME=2026-05-06 12:34:56
CLEAR
SET_INTERVAL_MIN=30
START
SLEEP
```

## End of deployment

```text
STOP
COUNT
INFO
CSV
```

Wait for `CSV_END`, export the nRF Connect log, and convert it with the Python tool.

## Data format

Current record size:

```text
26 bytes per record
```

One year at 30 minute interval:

```text
365 days × 24 hours × 2 records/hour = 17,520 records
17,520 × 26 + 12 byte header = 455,532 bytes
≈ 445 KiB
```

This fits comfortably in the 2 MB onboard flash.

## CSV columns

The BLE CSV export begins with deployment metadata:

```csv
META,logger_id,UW01_BREMERHAVEN
META,salinity_psu,31.800
META,lat_deg,53.5390000
META,lon_deg,8.5809000
```

Then measurement rows:

```csv
idx,unix,tempC,rhPct,vbat,bhMode,bhMtreg,bhRaw,lux,ax,ay,az,wake,flags
```
