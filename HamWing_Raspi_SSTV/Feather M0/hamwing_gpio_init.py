#!/usr/bin/env python3
"""
hamwing_gpio_init.py — HamWing GPIO safe-state initializer for Raspberry Pi

Usage:
  python3 hamwing_gpio_init.py
  
This script enforces safe GPIO states on the Pi after Feather M0 startup:
  1. Ensures PTT is HIGH (transmitter unkeyed, active-LOW logic)
  2. Ensures PD is HIGH (radios active, released from Feather control)
  3. Sets H/L to LOW (0.5W low power by default)
  4. Sets pins to OUTPUT mode so Pi has exclusive ownership
  
This prevents Pi UART GPIO from conflicting with Feather's UHF UART initialization.
Install as a systemd service to run automatically at boot.
"""

import RPi.GPIO as GPIO
import sys
import time

# BCM pin numbers — must match HamWing_RadioConfig.ino
PTT_PIN = 27   # Active-LOW PTT control (HIGH=idle, LOW=transmit)
PD_PIN = 4     # Power-down control (HIGH=active, LOW=sleep)
HL_PIN = 22    # Power level (HIGH=1W, LOW=0.5W)

def safe_cleanup():
    """Ensure GPIO is reset on exit."""
    GPIO.cleanup()

def main():
    try:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        print("HamWing GPIO Initialization")
        print(f"  PTT pin (GPIO{PTT_PIN}): OUTPUT HIGH (idle)")
        print(f"  PD pin (GPIO{PD_PIN}):   OUTPUT HIGH (active)")
        print(f"  H/L pin (GPIO{HL_PIN}):  OUTPUT LOW (0.5W)")
        print()

        # PTT: HIGH = idle (active-LOW keying)
        GPIO.setup(PTT_PIN, GPIO.OUT, initial=GPIO.HIGH)
        print(f"✓ GPIO{PTT_PIN} (PTT) set to OUTPUT HIGH (transmitter unkeyed)")

        # PD: HIGH = radios active
        GPIO.setup(PD_PIN, GPIO.OUT, initial=GPIO.HIGH)
        print(f"✓ GPIO{PD_PIN} (PD) set to OUTPUT HIGH (radios active)")

        # H/L: LOW = 0.5W low power
        GPIO.setup(HL_PIN, GPIO.OUT, initial=GPIO.LOW)
        print(f"✓ GPIO{HL_PIN} (H/L) set to OUTPUT LOW (0.5W low power)")

        print()
        print("✓ HamWing GPIO safe-state applied successfully")
        print()
        print("Next steps:")
        print("  1. Ensure Feather M0 has completed radio configuration (check Serial output)")
        print("  2. Your application can now control GPIO27 (PTT) and GPIO4 (PD)")
        print("  3. To transmit: GPIO27 LOW, to idle: GPIO27 HIGH")
        print()

        # Keep GPIO state active after script exits
        # (pin configuration persists in hardware)

    except Exception as e:
        print(f"✗ Error: {e}")
        safe_cleanup()
        sys.exit(1)

if __name__ == "__main__":
    main()
