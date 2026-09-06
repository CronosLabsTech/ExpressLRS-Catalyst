# ExpressLRS — Catalyst fork

Fork of [ExpressLRS](https://github.com/ExpressLRS/ExpressLRS) for **CatalystESC**, a
combat-robotics ESC with an integrated ELRS receiver: an ESP32-C3 and an SX1280 on the
same board as two AT32F421/AM32 motor drivers. There is no flight controller and no
CRSF header — ELRS drives the motor channels directly as servo PWM.

Branch `catalyst-4.1.0` is cut from upstream tag **4.1.0**. Hardware definitions live in
the nested submodule at `src/hardware`
([ExpressLRS-targets-Catalyst](https://github.com/CronosLabsTech/ExpressLRS-targets-Catalyst),
branch `catalyst`), a fork of `ExpressLRS/targets`.

## Building

```bash
git clone --recurse-submodules -b catalyst-4.1.0 git@github.com:CronosLabsTech/ExpressLRS-Catalyst.git
cd ExpressLRS-Catalyst/src
pio run -e Unified_ESP32C3_2400_RX_via_UART
```

`--recurse-submodules` matters. Upstream's `build_env_setup.py` clones
`ExpressLRS/targets` into `src/hardware` on first build and never touches it again, so
without the submodule you get upstream's targets and no Catalyst layout — the firmware
builds fine and then silently ignores your pin map.

The `Unified_ESP32C3_2400_RX_via_UART` environment is pinned to
`board_config = catalyst.rx_2400.evb` and builds with `-D DEBUG_LOG`.

## What this fork changes

| Commit | Scope |
|---|---|
| `PWM_ESP32: size LEDC channel table to the SoC…` | **Upstream bug fixes, not Catalyst-specific.** Suitable to send upstream as-is. |
| `Catalyst: ESP32-C3 USB logging, USB pin preservation, and AM32 passthrough` | Board support. |
| `Track hardware definitions as a submodule…` | Build/repo plumbing. |
| `Add Catalyst settings to rx_config_t…` | Config storage. Reuses padding so the config version never moves. |
| `Expose the Catalyst settings in the handset Lua menu` | CRSF parameters, no web UI build. |
| `Add differential mix and invert-drive to the servo outputs` | Drive behaviour. |
| `Gate AM32 passthrough on config…` | Makes passthrough switchable. |
| `HACK: cap WiFi TX power at 11dBm…` | **Unresolved. See below - do not assume this is a supply problem.** |

That last one is isolated deliberately, and the reason for it is **not yet known**. The
original theory was that the EVB's AP7375 LDO could not source the current a 19.5 dBm
transmit draws, so the rail sagged and WiFi dropped. That theory is now disproven: the
cap was reverted and retested against a 500 mA 3.3 V buck, and 19.5 dBm still fails.
Supply capacity is therefore ruled out.

What has not been ruled out: transient response and local decoupling near the module
(a buck's rating says little about its behaviour on a 350 mA burst), the RF path itself,
or something that is not a power problem at all — the failure has never been
characterised beyond "WiFi drops". Worth capturing what actually happens at 19.5 dBm over
the USB log before theorising further: a brownout, a watchdog reset and a silent AP
disappearing are three different faults.

Note the cap applies to *every* ESP32 target, not just this board.

## Pin mapping

| Function | GPIO |
|---|---|
| Radio NRESET | 0 |
| Radio DIO1 | 1 |
| Radio NSS | 2 |
| VSENSE (battery voltage) | 3 |
| **PWM ch2 — Right motor driver** (`U501` PB4) | **4** |
| Radio SCK | 5 |
| Radio MISO | 6 |
| Radio MOSI | 7 |
| Radio BUSY | 10 |
| AM32 shared `/RST`, and BOOT strap | 9 |
| **PWM ch4** / USB D− / passthrough host pad | **18** |
| **PWM ch3** / USB D+ / passthrough host pad | **19** |
| **PWM ch1 — Left motor driver** (`U201` PB4) | **20** |
| Status LED | 21 |

Radio DIO2/DIO3 drive the two link-status LEDs straight from the SX1280 — no GPIO
involved. GPIO11–17 are the C3's dedicated SPI-flash pins and are unavailable.

VSENSE on GPIO3 is read by ELRS's own `AnalogVbat` device, configured entirely in the
layout JSON: `vbat: 3`, `vbat_atten: 7`, `vbat_scale: 909`, `vbat_offset: 0`. The
divider is 11:1 (100k over 10k, same as MK1), so 6S at 25.2 V lands at 2.29 V on the
pin. `vbat_atten` must be above `ADC_11db` (3) or `devAnalogVbat` skips ADC
characterization, the reading stays a raw count instead of millivolts, and the scale
means nothing. Battery telemetry then reaches the handset for free.

`serial_rx`/`serial_tx` are omitted entirely — valid for a PWM-only receiver, the WebUI
just won't offer a serial pin.

Mixing and invert-drive are implemented here — see **Catalyst settings** below.

> **Naming caveat:** the target is called *Catalyst ELRS EVB*, but the EVB and the
> form-factor board now share this exact pinout and configuration, so the one target
> serves both and there is no reason to split it. Only the name is stale. Renaming it
> would change `board_config`, the layout filename and the product name shown in the
> configurator, so it is not free — worth doing deliberately, not incidentally.

## Catalyst settings

Three settings, in the handset ELRS Lua script under **Other Devices > Catalyst EVB**.

| Setting | Values | Default |
|---|---|---|
| `Drive mix` | Tank / Mix | Tank |
| `AM32 config` | Allowed / Blocked | Allowed |
| `Invert drive` | Off / Ch1...Ch16 | Off |

They will not appear unless **telemetry is enabled** on the model. Device discovery
travels on the telemetry uplink, so with it off the receiver never answers the device
ping, and `lua/elrs.lua` keeps the whole `Other Devices` entry hidden - it only unhides
once more than one device is known. A missing entry therefore means "no second device
discovered", never "parameter registration is broken". `Std` telemetry ratio is enough.

### Storage

All three live in bits of what upstream calls `unused_padding` in `rx_config_t`, so
`RX_CONFIG_VERSION` never moves. That matters: a version bump would make stock
ExpressLRS see a future config and call `SetDefaults`, wiping the bind of anyone who
cross-flashes in either direction. `SetDefaults` memsets the struct, so all-zero is
guaranteed, and each field is defined so zero is the pre-existing behaviour - meaning no
upstream default needed changing either. A `static_assert` pins the struct at 92 bytes,
the size stock 4.1.0 has; grow it and the build fails rather than silently diverging.

### Drive mix and invert-drive

Ported from `Software/ATtiny1616/src/mixing.c`. Both combine outputs 0 (left) and 1
(right), which the per-channel loop in `servoCalcAllChannels` cannot express, so they are
computed up front and override those two outputs.

- **Tank** - each stick drives one side. Identical to stock behaviour, and the MK1 default.
- **Mix** - `left = throttle + steering`, `right = throttle - steering`, full steering
  authority.
- **Invert drive** - above 1600 us on the selected channel, the sides *swap and negate*.
  Flipping the robot mirrors steering as well as reversing each side, so a swap alone is
  not enough. Runs after the mix and before the per-output `inverted` flag.

Sources are each output's configured `inputChannel`, so stock ExpressLRS channel mapping
still selects the sticks. Deliberately dropped from the original: the deadzone, the
squaring curve, and the half-steering option - modern handsets do expo themselves.

Note `invert_left` was true by default on MK1 so both motors can be wired the same way.
That maps onto ELRS's per-output `inverted` flag rather than anything here. Check motor
direction before closing the bot up.

## AM32 serial passthrough

An AM32 configurator attached to a PWM pad can talk to the ESC behind it, with the
ESP32-C3 relaying over hardware UARTs. Implemented in `src/src/rx_am32_passthrough.cpp`;
a logical port of `Software/ATtiny1616/src/serial_passthrough.c` from the main Catalyst
repo, using hardware UARTs rather than bit-banging.

Detection runs **once at boot**, after `setupConfigAndPocCheck()` (which loads the
`AM32 config` gate) and before `setupTarget()` claims the PWM pins. That ordering isn't
a preference — the configurator attaches to the very pads ELRS drives as servo outputs,
so the two cannot coexist and the choice has to be made up front. If a configurator is
found, passthrough owns the board until power is removed. With the gate set to Blocked
the pads are never even sampled.

Sitting after the POC check has a trap worth knowing. `setupConfigAndPocCheck()`
increments the power-on counter and schedules its 2 s clear via `deferExecutionMillis`,
which only runs from the main loop. `relay()` never returns to the loop, so that clear
would never fire - and three passthrough sessions would leave the counter at 3, dropping
the next ordinary boot into binding mode for no visible reason. The passthrough path
clears it explicitly instead.

- CH3 pad (GPIO19) → **left** ESC
- CH4 pad (GPIO18) → **right** ESC
- GPIO9 holds both ESCs in reset during setup, then releases them with the signal line
  high so they land in the bootloader rather than the application.

GPIO9 needs isolation diodes (anode at each AM32 NRST, cathode on the shared node).
Without them an ESC asserting its own reset drags the shared net low — and since GPIO9
is also the BOOT strap, that traps the ESP32 in ROM download mode at its next reset.

Two ESP32 quirks this depends on, both of which fail *silently*, are documented at
length in the file header. Read them before touching that code:

1. `Serial.begin(baud, cfg, pin, pin)` does not produce a working single-wire pad — the
   core's two-call pin attach leaves the pad with no input route, so you transmit and
   never receive, not even your own echo. Both signals are routed by hand instead.
2. Never `gpio_set_direction()` to a mode lacking `OUTPUT`. `gpio_ll_output_disable()`
   permanently unroutes UART TX rather than merely releasing the driver.

## USB behaviour

PWM channels 3/4 share the native USB D+/D− pins, so they conflict by construction.
With a host attached, `devServoOutput` leaves those two pins as USB instead of claiming
them for PWM. That costs channels 3/4, but keeps the CDC port alive across ELRS boot, so
reflashing needs no BOOT-strap dance. Bench and production-fixture case only — neither is
driving motors — and with USB unplugged the channels behave normally.

Logging goes out over the native USB-Serial-JTAG rather than UART0, because on this board
UART0's pins (GPIO20/21) are a motor output and the status LED.

## Rebasing onto a newer ExpressLRS

```bash
git remote add upstream https://github.com/ExpressLRS/ExpressLRS.git
git fetch upstream --tags
git rebase v4.2.0        # or whichever tag
```

Expect conflicts only in the files listed above. Some of these commits may cease to
exist over time — the `PWM_ESP32` fixes in particular are upstream bugs and should
disappear from this fork if they're accepted upstream.
