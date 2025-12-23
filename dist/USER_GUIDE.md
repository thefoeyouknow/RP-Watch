# RPWatch User Guide

## Overview
RPWatch provides an analog pilot face with a digital 12-hour clock, an Activity tile for steps, and a Tools tile with Stopwatch, Timer, and Alarm.

## Navigation
- Swipe or button to switch tiles: Pilot → Activity → Tools → Settings → Clock Set.

## Pilot Face
- Analog hands show hour/minute/second (second ticks each second).
- Digital 12-hour clock centered above the analog center.

## Activity (Steps)
- Large step count and 300° progress arc.
- Step goal and percentage shown below.
- Detector uses high-pass filtering + hysteresis for stability; small lag is normal.

## Tools
### Stopwatch
- Start/Stop/Reset via on-screen controls.

### Timer
- Set hours/minutes/seconds using rollers.
- Start to show countdown with arc; Pause/Resume/Reset as needed.
- On completion, a modal appears and beeps.

### Alarm
- Set hour/minute (12-hour with AM/PM) using rollers.
- Choose repeat mode; enable/disable via switch.
- Triggers on minute boundary; shows modal and beeps until dismissed.

## Settings
- Adjust brightness, IMU behavior, and power-related options.
- Save settings; watchdog and low-power modes manage battery.

## Clock Set
- Set current time; saves to RTC.

## Flashing Firmware
- Option 1: BOOTSEL
  1. Hold BOOTSEL and connect USB.
  2. Copy `waveshare_watch_test.uf2` to the mounted drive.
  3. Device reboots automatically.
- Option 2: Picotool (device connected)
  - `picotool load build/waveshare_watch_test.elf -fx`

## Troubleshooting
- If steps increment at rest, slightly increase thresholds in Settings if available, or report field observations to adjust `STEP_HP_ALPHA`.
- If flashing via BOOTSEL fails, try a different USB cable/port.
