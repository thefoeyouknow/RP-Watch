# Flashing RPWatch Firmware

## Using BOOTSEL (UF2)
1. Disconnect the device.
2. Hold BOOTSEL while connecting USB.
3. A drive mounts (RPI-RP2).
4. Copy `waveshare_watch_test.uf2` onto the drive.
5. The device reboots and runs the new firmware.

## Using Picotool (ELF)
- Ensure the device is connected and recognized.
- Run:

```powershell
picotool.exe load "build/waveshare_watch_test.elf" -fx
```

If using the VS Code Task "Run Project", it loads the ELF automatically and reboots the device.

## Notes
- On Windows with RP2040, forced commands may not work; use BOOTSEL if needed.
- For OpenOCD flashing, a CMSIS-DAP probe is required; see the VS Code "Flash" task.
