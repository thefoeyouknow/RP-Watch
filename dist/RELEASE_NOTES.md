# RPWatch Release Notes

Date: 2025-12-23
Version: 1.0.0

## Highlights
- Pilot face: Analog hands plus digital 12-hour clock (Montserrat 24), second hand ticks once per second.
- Activity: Step counter with 300° arc, labels for steps/goal/percentage.
- Tools: Restored Timer and Alarm with rollers and enable switch; modal alerts and beeping on completion/trigger.
- Stability: Step detection uses high-pass filtered magnitude with hysteresis and debounce to reduce false counts at rest.

## Changes Since Last Build
- Added digital clock to pilot and removed step arc from pilot (kept on Activity tile).
- Reduced digital clock font to improve balance and readability.
- Restored historical Timer/Alarm UI (rollers + enable slider) and logic.
- Implemented high-pass + hysteresis step detector; tuned for lower idle noise.
- Moved digital clock label upward to improve layout.

## Known Issues
- Minor step count lag under certain motion patterns is expected; prioritizes stability over immediacy.
- Alarm beeping cadence may vary based on power state; consistent in Active.

## Build Artifacts
- UF2: `dist/waveshare_watch_test.uf2`
- ELF: `build/waveshare_watch_test.elf`

## Compatibility
- Hardware: RP2040-based Waveshare watch.
- SDK: pico-sdk; LVGL UI.

## Next Candidates (Post-Release)
- Optional tuning for `STEP_HP_ALPHA` and thresholds based on field feedback.
- Minor UI polish on Activity labels alignment.
