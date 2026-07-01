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
addr        = 0x43             ; INA219 I2C address (Waveshare UPS HAT B default)
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

## Debian package

Build the `.deb` on the target machine (e.g. Raspberry Pi):

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

Edit `/etc/waveshare-ups/config.ini` and `systemctl restart waveshare-ups` to apply changes. Verify `device.addr` matches your board (default `0x43`) — run `i2cdetect -y 1` to find it.
