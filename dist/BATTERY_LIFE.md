# RPWatch Battery Life Analysis

Date: 2025-12-23
Version: 1.1.2-Release

## Overview
This analysis estimates battery life across scenarios using component-level current draws and firmware duty cycles. Values are approximations; real measurements depend on hardware specifics (regulator efficiency, display/backlight characteristics, IMU model).

## Assumptions
- Battery: 250 mAh LiPo/coin cell (typical small watch)
- MCU: RP2040
  - Active at ~120–133 MHz: 20–30 mA (SoC + board overhead)
  - Idle (doze) with clocks gated: 3–8 mA
  - Deep sleep-like low activity: 1–3 mA
- Display (ST7789-like 240×240 SPI)
  - Backlight current depends on brightness: 5–25 mA (linear approximation)
  - Panel logic ~1–2 mA when updating; ~<1 mA when static
- IMU (LSM6DS3/BMI160 class)
  - Active accel/gyro: 0.6–1.5 mA
  - Low-power accel-only: 0.1–0.3 mA
  - Standby: ~0.01–0.05 mA
- Misc (ADC, sensors, leakage, regulator overhead): 0.5–2 mA variable

## Firmware Duty Cycle Model
- Display-on usage per day: `t_disp` hours
- Average backlight brightness fraction: `b` (0–1)
- Active UI/logic while display on: `I_active_ui ≈ 25 mA`
- Backlight current: `I_bl(b) ≈ 5 mA + 20 mA · b`
- Panel logic: `I_panel ≈ 1 mA` while updating
- IMU:
  - Activity tracking baseline in idle: `I_imu_idle ≈ 0.2 mA`
  - Active motion processing peaks: `I_imu_active ≈ 0.8 mA` during display interactions
- Idle baseline (display off): `I_idle ≈ 2.5 mA` (MCU dozing + IMU LP + misc)

## Average Current Calculation
Let the day have `24` hours; display-on time is `t_disp` and display-off is `24 − t_disp`.

- Display-on average current:
  $I_{on} = I_{active\_ui} + I_{panel} + I_{bl}(b) + I_{imu\_active}$

- Display-off average current:
  $I_{off} = I_{idle}$

- Daily average current:
  $I_{avg} = \frac{t_{disp} \cdot I_{on} + (24 - t_{disp}) \cdot I_{off}}{24}$

- Battery life in days:
  $T_{days} = \frac{\text{Capacity}_{mAh}}{I_{avg}} \div 24$

## Scenarios
### 1) Typical Use (2 h/day display, 30% brightness)
- `t_disp = 2`, `b = 0.3`
- `I_bl(0.3) ≈ 5 + 20·0.3 = 11 mA`
- `I_on ≈ 25 + 1 + 11 + 0.8 = 37.8 mA`
- `I_off ≈ 2.5 mA`
- $I_{avg} = \frac{2 \cdot 37.8 + 22 \cdot 2.5}{24} = \frac{75.6 + 55}{24} \approx 5.44\ \text{mA}$
- For 250 mAh: $T_{days} = \frac{250}{5.44} \div 24 \approx 1.91\ \text{days}$

### 2) Light Use (1 h/day display, 20% brightness)
- `t_disp = 1`, `b = 0.2`, `I_bl ≈ 9 mA`
- `I_on ≈ 25 + 1 + 9 + 0.8 = 35.8 mA`
- $I_{avg} = \frac{1 \cdot 35.8 + 23 \cdot 2.5}{24} = \frac{35.8 + 57.5}{24} \approx 3.88\ \text{mA}$
- $T_{days} \approx \frac{250}{3.88} \div 24 \approx 2.69\ \text{days}$

### 3) Heavy Use (4 h/day display, 50% brightness)
- `t_disp = 4`, `b = 0.5`, `I_bl ≈ 15 mA`
- `I_on ≈ 25 + 1 + 15 + 0.8 = 41.8 mA`
- $I_{avg} = \frac{4 \cdot 41.8 + 20 \cdot 2.5}{24} = \frac{167.2 + 50}{24} \approx 9.05\ \text{mA}$
- $T_{days} \approx \frac{250}{9.05} \div 24 \approx 1.15\ \text{days}$

### 4) Low-Power Overnight Strategy (Display-off 23 h/day)
- `t_disp = 1`, `b = 0.1`, `I_bl ≈ 7 mA`
- `I_on ≈ 25 + 1 + 7 + 0.8 = 33.8 mA`
- Idle tuned to `I_off ≈ 2.0 mA` via deeper doze and IMU LP
- $I_{avg} = \frac{1 \cdot 33.8 + 23 \cdot 2.0}{24} = \frac{33.8 + 46}{24} \approx 3.33\ \text{mA}$
- $T_{days} \approx \frac{250}{3.33} \div 24 \approx 3.12\ \text{days}$

## Firmware Levers That Impact Battery
- **Backlight PWM curve**: Reduce current at mid-levels; consider gamma-corrected mapping.
- **Display refresh gating**: Avoid continuous redraws; let LVGL idle when unchanged.
- **IMU low-power modes**: Use accel-only LP for idle, gate gyro.
- **ADC + USB polling**: Disable in sleep; sample battery less frequently.
- **Watchdog cadence**: Use longer wake intervals when idle.
- **Animation tick rate**: Prefer 1 Hz for second hand; batch UI updates.

## Practical Expectations
- With conservative brightness (20–30%) and ~1–2 h/day display-on, expect ~2–3 days on ~250 mAh.
- Larger batteries or more aggressive idle strategies can push closer to 3–4 days.
- Real-world measurements recommended: log current via USB power meter or inline shunt during typical cycles.

## Measurement Plan (Optional)
1. Instrument a USB inline power meter.
2. Record current during: idle, display-on, active UI, timer alarm.
3. Validate `I_idle`, `I_on` and adjust firmware parameters.
4. Iterate brightness mapping and IMU LP thresholds.
