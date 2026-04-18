#!/usr/bin/env python3
"""
pi_gpio_init.py  —  HamWing GPIO safe-state initializer
Run once at boot (before pi_sstv.py or any other script) via systemd.

Sets the shared DRA818 control lines to their safe idle states immediately,
so the Pi never holds PTT keyed or PD low during the boot window.

Safe idle state:
  PD  (GPIO4,  pin 7)  = OUTPUT HIGH  — radios active (Feather M0 also drives this HIGH)
  PTT (GPIO27, pin 13) = OUTPUT HIGH  — transmitter unkeyed (active-LOW, HIGH = idle)
  HL  (GPIO22, pin 15) = OUTPUT LOW   — low power (0.5 W) until pi_sstv.py decides otherwise
"""

import RPi.GPIO as GPIO
import sys

# BCM pin numbers — must match pi_sstv.py
DRA818_PTT_PIN        = 27
DRA818_POWER_DOWN_PIN =  4
DRA818_POWER_LEVEL_PIN = 22

def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    # PTT: OUTPUT HIGH = transmitter unkeyed (active-LOW PTT)
    GPIO.setup(DRA818_PTT_PIN,         GPIO.OUT, initial=GPIO.HIGH)

    # PD: OUTPUT HIGH = radios active.
    # The Feather M0 also drives this line HIGH.  Both drivers agree on HIGH,
    # so there is no contention.  The Pi switches to INPUT (releasing the line
    # back to the Feather) in pi_sstv.py after each TX window.
    GPIO.setup(DRA818_POWER_DOWN_PIN,  GPIO.OUT, initial=GPIO.HIGH)

    # HL: OUTPUT LOW = 0.5 W low power until pi_sstv.py changes it for TX.
    GPIO.setup(DRA818_POWER_LEVEL_PIN, GPIO.OUT, initial=GPIO.LOW)

    print("HamWing GPIO safe-state applied: PTT=HIGH(idle), PD=HIGH(active), HL=LOW(low-power)")
    # GPIO state persists after this script exits — the kernel keeps the pin
    # configuration until another process changes it or the system reboots.

if __name__ == "__main__":
    main()
