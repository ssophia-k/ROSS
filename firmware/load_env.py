"""PlatformIO pre-build script: inject .env values as -D build flags."""

import os
import sys

Import("env")  # noqa: F821 — PlatformIO SCons magic

env_path = os.path.join(env["PROJECT_DIR"], "..", ".env")

if not os.path.isfile(env_path):
    sys.stderr.write(
        f"\n*** Missing {env_path}\n"
        f"*** Run 'make setup-env' from the repo root first.\n\n"
    )
    env.Exit(1)

values = {}
with open(env_path) as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        key, _, val = line.partition("=")
        values[key.strip()] = val.strip()

for key in ("WIFI_SSID", "WIFI_PASS"):
    if key not in values or not values[key]:
        sys.stderr.write(
            f"\n*** {key} is empty in {env_path}\n"
            f"*** Run 'make setup-env' from the repo root to set it.\n\n"
        )
        env.Exit(1)

env.Append(CPPDEFINES=[
    ("WIFI_SSID", env.StringifyMacro(values["WIFI_SSID"])),
    ("WIFI_PASS", env.StringifyMacro(values["WIFI_PASS"])),
])
