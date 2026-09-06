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
| `HACK: cap WiFi TX power at 11dBm…` | **Workaround for an EVB power fault. Revert once the rail redesign lands.** |

That last one is isolated deliberately: the EVB's AP7375 LDO can't source the current a
19.5 dBm transmit draws, so the rail sags and WiFi drops. It caps TX power for *every*
ESP32 target, not just this board. `git revert` it and retest before believing a new
supply is good — leaving it in place hides the fault the redesign is meant to fix.

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

VSENSE on GPIO3 is not in the layout JSON; ELRS support for reading it is unconfirmed.
`serial_rx`/`serial_tx` are omitted entirely — valid for a PWM-only receiver, the WebUI
just won't offer a serial pin.

Mixing is assumed to happen on the transmitter, matching the ATtiny1616 firmware's own
`enable_mixing = false` default. Upstream's PWM output is a fixed 1:1 channel-to-pin
mapper and cannot reproduce that firmware's differential-drive mix, deadzone, or
exponential response curve. Doing it receiver-side would need real work here.

> **Naming caveat:** the target is called *Catalyst ELRS EVB*, but the EVB and the
> form-factor board now share this exact pinout and configuration, so the one target
> serves both and there is no reason to split it. Only the name is stale. Renaming it
> would change `board_config`, the layout filename and the product name shown in the
> configurator, so it is not free — worth doing deliberately, not incidentally.

## AM32 serial passthrough

An AM32 configurator attached to a PWM pad can talk to the ESC behind it, with the
ESP32-C3 relaying over hardware UARTs. Implemented in `src/src/rx_am32_passthrough.cpp`;
a logical port of `Software/ATtiny1616/src/serial_passthrough.c` from the main Catalyst
repo, using hardware UARTs rather than bit-banging.

Detection runs **once at boot**, before `setupTarget()` claims the PWM pins. That
ordering isn't a preference — the configurator attaches to the very pads ELRS drives as
servo outputs, so the two cannot coexist and the choice has to be made up front. If a
configurator is found, passthrough owns the board until power is removed.

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
