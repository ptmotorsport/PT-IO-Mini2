# Gear Output Host Tests

Run from the repository root with a C++11-capable `g++`:

```powershell
g++ -std=c++11 -Wall -Wextra -I test/gear_outputs_host -I src test/gear_outputs_host/test_main.cpp src/gear_outputs.cpp src/can_modes.cpp -o .pio/gear-output-tests.exe
.\.pio\gear-output-tests.exe
```

These tests compile the actual gear-output implementation and CAN dispatcher
against small host substitutes for Arduino strings, time, CAN frames, and
EEPROM. The `.pio` directory is created by a normal PlatformIO build.

Coverage includes all 256 gear bytes, default bindings, custom multi-gear and
shared-output bindings, disabled outputs, parser errors, unrelated and short
frames, persisted bindings, corrupt EEPROM recovery, configuration isolation,
signal timeout, and clock wraparound.

Hardware output polarity, main-loop override/safe-state application, and USB
JSON commands still require device-level verification.