/**
 * AM32 serial passthrough for the Catalyst board.
 *
 * Lets an AM32 configurator attached to a PWM output pad talk to the ESC behind
 * it, with the ESP32 relaying between them. Logical port of
 * Software/ATtiny1616/src/serial_passthrough.c, using hardware UARTs rather than
 * bit-banging.
 *
 * This is checked ONCE at boot, before ELRS claims the PWM pins, because the
 * host pads are the same pins ELRS drives as servo outputs in normal operation -
 * there is no way to share them, so the decision has to be made up front.
 *
 * Pin roles come from the hardware JSON's pwm_outputs array so they stay in sync
 * with the target definition:
 *   [0] LEFT ESC signal   [2] CH3 host pad -> relays to LEFT
 *   [1] RIGHT ESC signal  [3] CH4 host pad -> relays to RIGHT
 *
 * TWO ESP32 QUIRKS THIS DEPENDS ON, both of which fail silently:
 *
 * 1. Serial.begin(baud, cfg, pin, pin) does NOT give a working single-wire pad.
 *    The core attaches RX then TX via separate uart_set_pin() calls and the
 *    second leaves the pad with no input route, so you transmit but never
 *    receive - not even your own echo. We route both signals by hand instead.
 *
 * 2. Never gpio_set_direction() to a mode lacking OUTPUT. gpio_ll_output_disable()
 *    rewrites GPIO_FUNC0_OUT_SEL_CFG_REG to SIG_GPIO_OUT_IDX, permanently
 *    unrouting UART TX rather than merely releasing the driver. The pad stays in
 *    INPUT_OUTPUT_OD for its whole life; open-drain plus a pullup is inherently
 *    bidirectional and needs no direction switching.
 */

#if defined(PLATFORM_ESP32_C3)

#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <soc/uart_periph.h>

#include "targets.h"
#include "logging.h"
#include "config.h"

// GPIO9 drives the shared AM32 /RST net. Not in the hardware JSON: it is the
// boot strap, already pulled high by the board, which is the correct inactive
// state for an active-low reset. Isolation diodes (anode at each ESC NRST,
// cathode on the shared node) stop an ESC asserting its own reset from dragging
// this net - without them a self-resetting ESC pulls our strap low and traps the
// ESP32 in ROM download mode at its next reset.
#define PIN_AM32_RST 9

static constexpr uint32_t PASSTHRU_BAUD = 19200;
static constexpr uint32_t BIT_TIME_US   = 1000000UL / PASSTHRU_BAUD;
// AM32 spaces its reply bytes in software, far wider than the 5 bit times the
// ATtiny used; too small a gap truncates the reply after its first byte.
static constexpr uint32_t PACKET_GAP_US = 30 * BIT_TIME_US;

static constexpr uint32_t TIMEOUT_HOST_US = 1000000;
static constexpr uint32_t TIMEOUT_ESC_US  = 200000;
static constexpr size_t   RX_BUFFER_SIZE  = 258;  // matches AM32 bootloader

// A bare pad reads low through its 100k pulldown; an attached configurator idles
// its UART high. Only a fraction of samples need be high, since the configurator
// may be mid-transmission when we look.
static constexpr uint16_t DETECT_SAMPLES         = 1000;
static constexpr uint16_t DETECT_HIGH_THRESHOLD  = 200;
static constexpr uint32_t DETECT_SAMPLE_DELAY_US = 10;

static constexpr uint32_t HOST_ABSENT_MS = 3000;

static uint8_t rxBuffer[RX_BUFFER_SIZE];

struct Side {
    HardwareSerial *port;
    int pin;
};

static void am32HoldReset() {
    pinMode(PIN_AM32_RST, OUTPUT);
    digitalWrite(PIN_AM32_RST, LOW);
}

/**
 * Release the ESCs. Whatever the signal line is doing at this instant decides
 * their fate - AM32's checkForSignal() samples it within ~17ms of coming out of
 * reset. Held high (our idle state) means bootloader; low or toggling means it
 * jumps to the application. Setting the line first makes that deterministic
 * instead of a power-up race.
 */
static void am32ReleaseReset() {
    pinMode(PIN_AM32_RST, INPUT);  // board pullup brings it high
    delay(60);                     // ESC boot + its checkForSignal window
}

static bool configuratorPresent(int pin) {
    pinMode(pin, INPUT);  // no pullup - let the board's pulldown win
    uint16_t high = 0;
    for (uint16_t i = 0; i < DETECT_SAMPLES; i++) {
        if (digitalRead(pin) == HIGH) {
            high++;
        }
        delayMicroseconds(DETECT_SAMPLE_DELAY_US);
    }
    return high >= DETECT_HIGH_THRESHOLD;
}

static void bindSingleWire(HardwareSerial &port, int pin, int uartNum) {
    port.setRxBufferSize(RX_BUFFER_SIZE);
    port.begin(PASSTHRU_BAUD, SERIAL_8N1, pin, pin);

    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode((gpio_num_t)pin, GPIO_PULLUP_ONLY);

    // Quirk 1: route both directions onto the pad ourselves.
    esp_rom_gpio_connect_out_signal(pin, UART_PERIPH_SIGNAL(uartNum, SOC_UART_TX_PIN_IDX), false, false);
    esp_rom_gpio_connect_in_signal(pin, UART_PERIPH_SIGNAL(uartNum, SOC_UART_RX_PIN_IDX), false);
}

/** Read a packet, framed by an inter-byte gap. Returns 0 on timeout. */
static size_t readPacket(HardwareSerial &port, uint8_t *buf, uint32_t firstTimeoutUs) {
    uint32_t start = micros();
    while (!port.available()) {
        if (micros() - start > firstTimeoutUs) {
            return 0;
        }
    }

    size_t len = 0;
    uint32_t lastByte = micros();
    while (len < RX_BUFFER_SIZE) {
        if (port.available()) {
            buf[len++] = (uint8_t)port.read();
            lastByte = micros();
        } else if (micros() - lastByte > PACKET_GAP_US) {
            break;
        }
    }
    return len;
}

/** Write a packet and swallow the echo it creates on our own RX. */
static void writePacket(HardwareSerial &port, const uint8_t *buf, size_t len) {
    // Clear stale bytes first, or the echo drain counts leftovers and every
    // subsequent frame reads offset by however many were pending.
    while (port.available()) {
        port.read();
    }

    port.write(buf, len);
    port.flush();

    size_t discarded = 0;
    uint32_t budget = (uint32_t)(len + 4) * 10 * BIT_TIME_US;
    uint32_t start = micros();
    while (discarded < len && micros() - start < budget) {
        if (port.available()) {
            port.read();
            discarded++;
        }
    }
}

/**
 * Relay until the configurator goes away. Packet-at-a-time with a direction
 * swap - both sides are single-wire, so only one may drive at once. A plain
 * packet relay is sufficient: the ATtiny needed CMD_SET_BUFFER and
 * CMD_READ_FLASH_SIL handled specially because bit-banging made store-and-forward
 * slow enough to blow the host's timeout, but hardware UARTs turn around fast
 * enough that the configurator never notices.
 */
static void relay(const Side &host, const Side &esc) {
    const Side *in  = &host;
    const Side *out = &esc;
    uint32_t firstTimeout = TIMEOUT_HOST_US;
    uint32_t lastActivity = millis();

    while (true) {
        size_t len = readPacket(*in->port, rxBuffer, firstTimeout);

        if (len > 0) {
            writePacket(*out->port, rxBuffer, len);
            lastActivity = millis();

            const Side *tmp = in;
            in  = out;
            out = tmp;
            firstTimeout = (in == &host) ? TIMEOUT_HOST_US : TIMEOUT_ESC_US;
        } else {
            // Fall back to listening on the host side; resynchronises us if we
            // guessed the direction wrong. We never give up and return: there is
            // no way back to ELRS (the pins are committed), and re-running the
            // pad detection would call pinMode(), whose output-disable path
            // unroutes UART TX for good - which silently killed the link a few
            // seconds after entry.
            in  = &host;
            out = &esc;
            firstTimeout = TIMEOUT_HOST_US;
            (void)lastActivity;
        }
    }
}

/**
 * Called once at boot, after options_init() has loaded the pin map but before
 * anything claims the PWM pins. Returns false to continue normal ELRS startup.
 * If a configurator is found this does not return - passthrough owns the board
 * until power is removed, because the pins cannot be handed back to ELRS.
 */
bool am32Passthrough_checkAndRun()
{
    if (GPIO_PIN_PWM_OUTPUTS_COUNT < 4) {
        return false;  // not a Catalyst-shaped target
    }

    // Checked before the pads are sampled, so a blocked board never drives them.
    if (config.GetCatalystAm32Blocked()) {
        DBGLN("AM32 passthrough: disabled in config, skipping detection");
        return false;
    }

    // CH3's pad is also USB D+, which a connected host holds high - that reads
    // identically to a configurator idling its UART. So if a USB host is present,
    // never enter passthrough; it is either a developer or a production fixture,
    // and both want normal startup. (USB still goes away later when ELRS claims
    // these pins for PWM - this guard is about not misreading it, not saving it.)
    if (HWCDC::isPlugged()) {
        DBGLN("AM32 passthrough: USB host present, skipping detection");
        return false;
    }

    const int escLeft  = GPIO_PIN_PWM_OUTPUTS[0];
    const int escRight = GPIO_PIN_PWM_OUTPUTS[1];
    const int hostCh3  = GPIO_PIN_PWM_OUTPUTS[2];
    const int hostCh4  = GPIO_PIN_PWM_OUTPUTS[3];

    int hostPin;
    int escPin;

    if (configuratorPresent(hostCh3)) {
        hostPin = hostCh3;
        escPin  = escLeft;
        DBGLN("AM32 passthrough: configurator on CH3 -> LEFT ESC");
    } else if (configuratorPresent(hostCh4)) {
        hostPin = hostCh4;
        escPin  = escRight;
        DBGLN("AM32 passthrough: configurator on CH4 -> RIGHT ESC");
    } else {
        return false;  // normal ELRS startup
    }

    // setupConfigAndPocCheck() incremented the power-on counter on the way in and
    // scheduled the 2s clear as a deferred callback. relay() never returns to the
    // main loop, so that callback never runs - three passthrough sessions would
    // leave the counter at 3 and drop the next ordinary boot into binding mode.
    // Clear it here instead: entering passthrough is a deliberate boot, not a
    // half-finished one.
    if (config.GetPowerOnCounter() != 0) {
        config.SetPowerOnCounter(0);
        config.Commit();
    }

    // Park the ESCs before touching anything, so they cannot be running their
    // application while we set up to talk to their bootloader.
    am32HoldReset();

    bindSingleWire(Serial1, hostPin, 1);
    bindSingleWire(Serial, escPin, 0);

    // Both pads now idle high. Releasing reset with the ESC line high is what
    // lands it in the bootloader rather than the application.
    am32ReleaseReset();

    Side host = {&Serial1, hostPin};
    Side esc  = {&Serial, escPin};
    relay(host, esc);  // never returns

    return true;  // unreachable, keeps the compiler happy
}

#endif  // PLATFORM_ESP32_C3
