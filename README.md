# waveshare-ups

Reads the Waveshare UPS HAT (B) battery management board via I2C (INA219 power monitor chip) and broadcasts one JSON telemetry line per poll cycle over TCP.

## Network interface

| Port | Protocol | Description |
|------|----------|-------------|
| 8564 | TCP | Data stream — one newline-delimited JSON line per poll |
| 8565 | TCP | Status query — send `status\n`, get a key=value reply |

```bash
# Stream live telemetry
nc 127.0.0.1 8564

# Query status
echo status | nc 127.0.0.1 8565
```

## JSON output

One object per poll cycle, emitted on stdout and broadcast to all connected TCP clients.

```json
{"ts_us":1751200000000000,"frame":42,
 "V":3.820,"I":-0.550,"P":2.100,
 "V_shunt_mV":55.0,
 "SOC":75.0,"charging":false,"online":true}
```

`V` is bus voltage (V), `I` is current (A, positive = charging), `P` is power (W), `V_shunt_mV` is the shunt voltage drop (mV), `SOC` is estimated state of charge (%) from linear interpolation between the configured empty/full voltages, `charging` is true when current exceeds `battery.charge_threshold`, and `online` is true when bus voltage is at or above `battery.v_empty`.

## Build

Requires a C++20 compiler and CMake 3.16+.

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Binary: `build/waveshare_ups`

## Run

```bash
./build/waveshare_ups [--config config.ini]
```

The default config path is `config.ini` in the working directory. Copy and edit before running:

```bash
cp config.ini my-config.ini
# edit my-config.ini: set device.addr to match your board (run i2cdetect -y 1 to find it)
./build/waveshare_ups --config my-config.ini
```

## Configuration

`config.ini`:

```ini
[device]
bus         = /dev/i2c-1       ; I2C bus (Raspberry Pi default)
addr        = 0x42             ; INA219 I2C address (Waveshare UPS HAT B default)
poll_ms     = 1000             ; polling interval in milliseconds
retry_secs  = 5                ; seconds to wait before reconnect after failure

[ina219]
r_shunt     = 0.1              ; shunt resistor value in Ohms
current_lsb = 0.0001           ; current resolution in A/bit (0.1 mA)
                                ; Cal = trunc(0.04096 / (current_lsb * r_shunt))

[battery]
v_empty           = 3.0        ; voltage considered empty (V) — Li-Ion cutoff
v_full            = 4.2        ; voltage considered full (V)  — Li-Ion max
charge_threshold  = 0.05       ; current above this (A) means charging

[output]
ctrl_port   = 8565             ; status : echo status | nc 127.0.0.1 8565
data_port   = 8564             ; stream : nc 127.0.0.1 8564
```

## Install

### From the APT repository

CI publishes to a signed APT repository (shared with other aipicam Raspberry Pi packages) hosted on Cloudflare R2, with two channels:

- **`main`** — pushing a `v*` tag publishes the clean release version here.
- **`nightly`** — every push (to any branch, and PRs) publishes a dev build here, versioned with a `+<UTC timestamp>` suffix.

```bash
curl -fsSL https://apt.aipicam.com/pubkey.asc | sudo gpg --dearmor -o /usr/share/keyrings/aipicam.gpg

# stable releases
echo "deb [signed-by=/usr/share/keyrings/aipicam.gpg] https://apt.aipicam.com main main" | sudo tee /etc/apt/sources.list.d/aipicam.list

# or nightly builds instead
echo "deb [signed-by=/usr/share/keyrings/aipicam.gpg] https://apt.aipicam.com nightly main" | sudo tee /etc/apt/sources.list.d/aipicam.list

sudo apt-get update
sudo apt-get install waveshare-ups
```

Builds run on GitHub's native `ubuntu-24.04-arm` hosted runner (no QEMU), producing **arm64** packages — a 64-bit Raspberry Pi OS is required for the CI packages. Uses the same `R2_ACCOUNT_ID`, `R2_ACCESS_KEY_ID`, `R2_SECRET_ACCESS_KEY`, `GPG_PRIVATE_KEY`, and `GPG_KEY_ID` repo secrets as [pi-fan-control](https://github.com/jacohanekom/pi-fan-control), since it publishes into the same shared repo.

### Build the .deb yourself

On the target machine (e.g. a 32-bit Pi, which produces `armhf`):

```bash
cd waveshare-ups
dpkg-buildpackage -us -uc -b
```

The package lands one level up: `../waveshare-ups_1.0.0+..._armhf.deb`

Install:

```bash
sudo dpkg -i ../waveshare-ups_*.deb
```

The package:
- Creates a dedicated `waveshare-ups` system user with `i2c` group membership
- Installs config to `/etc/waveshare-ups/config.ini`
- Registers and starts a systemd service

## systemd service

```bash
# Enable and start
systemctl enable --now waveshare-ups

# Check status
systemctl status waveshare-ups

# Follow logs
journalctl -fu waveshare-ups
```

Edit `/etc/waveshare-ups/config.ini` and `systemctl restart waveshare-ups` to apply changes. Verify `device.addr` matches your board (default `0x42`) — run `i2cdetect -y 1` to find it.
