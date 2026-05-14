# Raspberry Pi Integration Guide for HamWing_RadioConfig

## Overview

The Feather M0 sketch configures both radio modules and hands off control to the Raspberry Pi. This guide covers Pi setup, safe GPIO initialization, and basic radio control.

---

## Phase 1: Disable Pi UART Console

The Raspberry Pi's default serial console (getty service) drives GPIO14/15, which contends with the Feather's UHF UART initialization. Disable it first.

### Step 1a: Disable serial-getty services

```bash
sudo systemctl disable serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@serial0.service
```

Verify:
```bash
sudo systemctl status serial-getty@ttyAMA0.service
# Should show "disabled"
```

### Step 1b: Disable UART in firmware config

Edit `/boot/firmware/config.txt` (Pi OS Bookworm+) or `/boot/config.txt` (older):

```bash
sudo nano /boot/firmware/config.txt
```

Find or add the line:
```
enable_uart=0
```

(If `enable_uart=1` exists, change it to `0`.)

Also remove UART from cmdline if present:
```bash
sudo nano /boot/firmware/cmdline.txt
```

Remove any entries like `console=serial0,115200` or `console=ttyAMA0,115200`. The line should be a single line with no serial console references.

### Step 1c: Reboot

```bash
sudo reboot
```

After reboot, verify:
```bash
systemctl status serial-getty@ttyAMA0.service | grep active
# Should output: inactive (disabled)
```

---

## Phase 2: Install GPIO Initialization Script

Copy `hamwing_gpio_init.py` to the Pi and make it executable:

```bash
# Copy script to home directory
cp hamwing_gpio_init.py ~/hamwing_gpio_init.py
chmod +x ~/hamwing_gpio_init.py
```

Test it manually:
```bash
python3 ~/hamwing_gpio_init.py
```

Expected output:
```
HamWing GPIO Initialization
  PTT pin (GPIO27): OUTPUT HIGH (idle)
  PD pin (GPIO4):   OUTPUT HIGH (active)
  H/L pin (GPIO22):  OUTPUT LOW (0.5W)

✓ GPIO27 (PTT) set to OUTPUT HIGH (transmitter unkeyed)
✓ GPIO4 (PD) set to OUTPUT HIGH (radios active)
✓ GPIO22 (H/L) set to OUTPUT LOW (0.5W low power)

✓ HamWing GPIO safe-state applied successfully
```

### (Optional) Install as Systemd Service

If you want GPIO initialized automatically on every boot:

Create `/etc/systemd/system/hamwing-gpio-init.service`:

```bash
sudo nano /etc/systemd/system/hamwing-gpio-init.service
```

Paste:
```ini
[Unit]
Description=HamWing GPIO Safe-State Initializer
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/bin/python3 /home/pi-user/hamwing_gpio_init.py
RemainAfterExit=yes
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

(Replace `pi-user` with your actual username, e.g., `pi` or `ubuntu`)

Enable and start:
```bash
sudo systemctl daemon-reload
sudo systemctl enable hamwing-gpio-init.service
sudo systemctl start hamwing-gpio-init.service
sudo systemctl status hamwing-gpio-init.service
```

---

## Phase 3: Basic Radio Control

Once the Feather has configured the radios, your Pi application owns these GPIO pins:

| GPIO | Function | Logic |
|------|----------|-------|
| 27   | PTT      | LOW = transmit, HIGH = idle |
| 4    | PD       | HIGH = radios active, LOW = sleep |
| 22   | H/L      | HIGH = 1W, LOW = 0.5W |

### Example: Python Radio Control

```python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Define pins
PTT_PIN = 27
PD_PIN = 4
HL_PIN = 22

# Initialize pins (high-impedance, released from Feather during setup)
# If using systemd service, they're already initialized

GPIO.setup(PTT_PIN, GPIO.OUT, initial=GPIO.HIGH)   # PTT idle
GPIO.setup(PD_PIN, GPIO.OUT, initial=GPIO.HIGH)    # Radios active
GPIO.setup(HL_PIN, GPIO.OUT, initial=GPIO.LOW)     # 0.5W low power

def key_transmitter(seconds=3):
    """Key the transmitter for specified duration."""
    print(f"Transmitting for {seconds}s...")
    GPIO.output(PTT_PIN, GPIO.LOW)   # Pull PTT LOW to key
    time.sleep(seconds)
    GPIO.output(PTT_PIN, GPIO.HIGH)  # Release PTT to HIGH
    print("Transmission complete.")

def set_power_level(watts=1):
    """Set radio power level."""
    if watts == 1:
        GPIO.output(HL_PIN, GPIO.HIGH)
        print("Power level: 1W (high)")
    else:
        GPIO.output(HL_PIN, GPIO.LOW)
        print("Power level: 0.5W (low)")

def set_radio_state(active=True):
    """Enable or disable radios (PD line)."""
    GPIO.output(PD_PIN, GPIO.HIGH if active else GPIO.LOW)
    state = "ACTIVE" if active else "SLEEP"
    print(f"Radios: {state}")

# Example usage
try:
    set_power_level(watts=1)
    time.sleep(0.5)
    key_transmitter(seconds=2)
    
except KeyboardInterrupt:
    print("\nCleanup...")
finally:
    GPIO.cleanup()
```

### Example: Shell Script

```bash
#!/bin/bash
# quick_key.sh — Simple PTT control

PTT_PIN=27

# Setup GPIO if not already done
sudo python3 /home/pi-user/hamwing_gpio_init.py

# Key transmitter for 3 seconds
echo "Keying transmitter (3s)..."
echo "27=$PTT_PIN" | sudo tee /sys/class/gpio/export > /dev/null 2>&1
echo "out" | sudo tee /sys/class/gpio/gpio$PTT_PIN/direction > /dev/null
echo "0" | sudo tee /sys/class/gpio/gpio$PTT_PIN/value > /dev/null  # LOW = key

sleep 3

echo "1" | sudo tee /sys/class/gpio/gpio$PTT_PIN/value > /dev/null  # HIGH = idle
echo "Transmission complete."
```

---

## Phase 4: Audio Input Configuration

The Feather M0 sketch currently configures the radios but does not drive audio. The Raspberry Pi provides audio via GPIO12/GPIO13 PWM through a low-pass filter to the DRA818 mic input.

### Audio Path

```
Raspberry Pi GPIO12, GPIO13 (PWM 10 kHz)
           ↓
Low-Pass Filter (R=1kΩ, C=100nF, fc≈1.6kHz)
           ↓
DRA818 MIC IN (100 mV max)
```

To drive audio from Pi to radio:

1. **Install PWM library:**
   ```bash
   pip install RPi.GPIO
   ```

2. **Example PWM audio:**
   ```python
   import RPi.GPIO as GPIO
   
   GPIO.setmode(GPIO.BCM)
   
   # PWM on GPIO12 (audio left or mono)
   GPIO.setup(12, GPIO.OUT, initial=0)
   pwm = GPIO.PWM(12, 10000)  # 10 kHz carrier
   pwm.start(50)              # 50% duty cycle
   
   # Vary duty cycle to modulate audio
   import time
   for i in range(50, 100):
       pwm.ChangeDutyCycle(i)
       time.sleep(0.1)
   
   pwm.stop()
   GPIO.cleanup()
   ```

---

## Phase 5: Integration Testing

After Feather startup and Pi GPIO init, test the full stack:

### Test Checklist

1. **Feather Serial Output:** Upload HamWing_RadioConfig and monitor Serial (115200 baud)
   - Should show `Config state VHF/UHF: OK / OK`
   - Should show `Handing off control to Raspberry Pi`
   - LED should show slow fade (success)

2. **Pi GPIO States:** After boot, check:
   ```bash
   # View GPIO states (requires gpio tool)
   # Or manually control:
   python3 hamwing_gpio_init.py
   echo "GPIO27 PTT: $(cat /sys/class/gpio/gpio27/value)"
   echo "GPIO4 PD:   $(cat /sys/class/gpio/gpio4/value)"
   echo "GPIO22 H/L: $(cat /sys/class/gpio/gpio22/value)"
   ```

3. **PTT Keying:** Run simple Python script above and listen on HT receiver
   - Should hear transmitter key and unkey cleanly
   - LED should go solid ON while Pi holds PTT LOW

4. **UHF UART:** If UHF failed to config when hat was installed before, it should now succeed
   - Feather will show `Config state VHF/UHF: OK / OK`

---

## Operator Workflow Quick Commands (pi_sstv.py)

After hardware setup is complete, use these commands for day-to-day operation.

### 1) Mission Run (normal operation)

```bash
python3 pi_sstv.py mission --config /home/pi-user/pi_sstv.cfg
```

### 2) Single Test Run (safe default: no RF TX)

```bash
# Encode-only by default (no radio transmit)
python3 pi_sstv.py test r36

# Explicitly allow RF TX for test mode
python3 pi_sstv.py test r36 --tx
```

### 3) Panel/Card Workflow (safe default: no RF TX)

```bash
# Use a folder of test images, encode-only by default
python3 pi_sstv.py panels pd50 --test-panel-source /home/pi-user/Desktop/pi_sstv/panels

# Explicitly allow RF TX for panel workflow
python3 pi_sstv.py panels pd50 --test-panel-source /home/pi-user/Desktop/pi_sstv/panels --tx
```

### 4) Hardware Diagnostics

```bash
# PTT GPIO key test (seconds)
python3 pi_sstv.py diag ptt 0.5

# Status LED test (seconds per state)
python3 pi_sstv.py diag led 1.5

# GPS NMEA/fix diagnostic stream (seconds)
python3 pi_sstv.py diag gps 30

# ALSA mixer/volume guardrail check
python3 pi_sstv.py diag alsa
```

### Notes

- The verb interface is recommended: `mission`, `test`, `panels`, `diag`, `info`, `config`.
- Legacy flags are still supported (for example: `--test`, `--test-panels`, `--ptt-test`).
- For compliance, keep callsign overlay enabled when transmitting SSTV.

---

## Troubleshooting

| Symptom | Cause | Solution |
|---------|-------|----------|
| UHF still failing to config | Pi UART console still active | Re-run step 1a/1b, reboot |
| GPIO27 (PTT) not controlling radio | Pi GPIO not initialized | Run `hamwing_gpio_init.py` |
| Feather LED fast blinking | Radio config failed | Check serial output, verify power, remove hat and retry |
| Pi UART errors ("Device not found") | UART disabled but app trying to use it | Ensure app doesn't open /dev/serial0 or /dev/ttyAMA0 |
| GPIO cleanup errors on exit | GPIO already in use | Add `GPIO.setwarnings(False)` at start of script |

---

## Advanced: Dual Pi Control (VHF + UHF on separate GPIO)

If you want independent PTT control for VHF and UHF, modify the Feather sketch:

In `HamWing_RadioConfig.ino`, release D16 and D17 (individual PTT lines) instead of monitoring them:

```cpp
// After config success, replace the loop's input reads with independent control:
GPIO.setup(PIN_V_PTT, GPIO.OUT, initial=GPIO.HIGH)   // Pi owns VHF PTT
GPIO.setup(PIN_U_PTT, GPIO.OUT, initial=GPIO.HIGH)   // Pi owns UHF PTT
```

Then on Pi, control separately:
```python
VHF_PTT = 16   # BCM pin for VHF
UHF_PTT = 17   # BCM pin for UHF

GPIO.setup(VHF_PTT, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(UHF_PTT, GPIO.OUT, initial=GPIO.HIGH)

# Key only VHF
GPIO.output(VHF_PTT, GPIO.LOW)
time.sleep(2)
GPIO.output(VHF_PTT, GPIO.HIGH)
```

---

## Next Steps

1. ✅ Disable Pi UART console (Phase 1)
2. ✅ Install GPIO init script (Phase 2)
3. ✅ Test basic PTT control (Phase 3)
4. ⚙️ (Optional) Configure audio input path (Phase 4)
5. ⚙️ Deploy your SSTV/APRS application (use gpio/PWM as shown above)

For SSTV-specific integration, refer to `pi_sstv.py` documentation or your application's GPIO requirements.
