#  Names the build output after the board variant and firmware version
# (e.g. "PT-IO-Mini2_fw9.bin") instead of the generic "firmware.bin", so a
# flashed/downloaded binary can always be identified at a glance.
import re

Import("env")

BOARD_NAME = "PT-IO-Mini2"

with open("src/main.cpp", encoding="utf-8") as f:
    match = re.search(r"FW_VERSION\s*=\s*0x([0-9A-Fa-f]+)", f.read())
fw_version = int(match.group(1), 16) if match else 0

env.Replace(PROGNAME=f"{BOARD_NAME}_fw{fw_version}")
