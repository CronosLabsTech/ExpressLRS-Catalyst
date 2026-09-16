/**
 * LSM6DS3TR-C bring-up proof of concept for the Catalyst board.
 *
 * Proves three things on real hardware, in order of how likely they are to bite:
 *
 *   1. The part answers at all      - WHO_AM_I reads 0x6A.
 *   2. It shares the radio's SPI bus with a live SX1280, rather than the two
 *      corrupting each other.
 *   3. It reports sane gravity      - a board sitting flat reads ~1000 mg on one
 *      axis and ~0 on the other two.
 *
 * Build-flag gated, so a normal build is byte-identical to one without this file:
 *
 *     pio run -e Unified_ESP32C3_2400_RX_via_UART        # untouched
 *     pio run -e Unified_ESP32C3_2400_RX_IMU_POC         # streams samples
 *
 * This is deliberately a POC and not the shape of the eventual feature. It only
 * reads and prints. No orientation reference is stored anywhere - that baseline
 * belongs to runtime sampling, not to config.
 *
 * WIRING (EVB, hand-wired)
 *   CS    GPIO0    driven by hand here; also ADC1_CH0, unused by the layout JSON
 *                  since radio_rst moved to GPIO9 to share the AM32 reset
 *   SCK   GPIO5  }
 *   MISO  GPIO6  }  shared with the SX1280, straight off the hardware JSON
 *   MOSI  GPIO7  }
 *   INT1/INT2     not connected - this polls STATUS_REG instead
 *
 * SHARING THE BUS WITH THE RADIO
 *
 * The SX1280 hal owns this bus and configures it for itself: 17.5 MHz, SPI mode
 * 0, and *hardware* chip-select, with spiEnableSSPins() arming the radio's NSS.
 * A naive SPIClass::transfer() here would therefore assert the RADIO's NSS and
 * feed IMU traffic straight into the SX1280.
 *
 * SPIEx::_transfer() is the way out. It re-selects which CS pins the SPI module
 * drives on every single transfer:
 *
 *     spiDisableSSPins(bus(), ~cs_mask);
 *     spiEnableSSPins(bus(), cs_mask);
 *
 * so passing cs_mask 0 means the hardware drives no CS at all and the radio's
 * NSS stays high while we toggle GPIO0 ourselves. The LSM6 also wants a slower
 * clock (10 MHz ceiling) and the opposite SPI mode (3, not 0), so each transfer
 * sets clock and mode, moves its bytes, and puts both back the way the radio
 * left them.
 *
 * Two consequences worth knowing, both acceptable for a POC and both reasons
 * this is not yet product code:
 *
 *   - Radio SPI happens from the DIO1 ISR. If it fires inside one of our
 *     transfers, that radio transfer runs at 10 MHz in mode 3. The SX1280
 *     tolerates the slower clock, but mode 3 is wrong for it, so a corrupted
 *     radio transaction is possible. Rare, harmless to a bench test, and the
 *     real fix is a mutex or moving the IMU off the ISR's path entirely.
 *   - SPIEx buffers must be word-aligned and word-padded, since _transfer()
 *     copies them to the FIFO a uint32_t at a time. Hence alignas(4) and the
 *     rounded-up lengths below.
 *
 * LOGGING: debugPrintf() is ELRS's own parser, not libc. It understands exactly
 * %s %d %u %x %f and has no width or precision. An unsupported specifier such as
 * %02X prints nothing AND does not consume its vararg, so every argument after it
 * silently shifts by one and the line comes out wrong rather than merely ugly.
 * Pad hex by hand if it ever matters; do not reach for %02X.
 *
 * FIRST TRANSFER IS GARBAGE, BY DESIGN OF THE DRIVER: measured on hardware, the
 * first read after we switch the bus to the LSM6's clock and mode returns 0xFF
 * and every read after it returns 0x6A. setFrequency()/setDataMode() do not take
 * effect until the driver starts the next transaction, so that first transfer
 * still runs in the radio's mode 0 and simply samples the pulled-up MISO line.
 * One throwaway transfer lands the configuration; do not delete it thinking it
 * is a superstition.
 *
 * TRANSFER LENGTH IS NOT COSMETIC: pass the exact number of bytes that should go
 * on the wire, never the padded buffer size. With IF_INC set, every byte clocked
 * after a written value lands in the NEXT register. A write padded to 4 bytes
 * therefore also zeroes reg+1 and reg+2 - which is how the CTRL2_G write silently
 * cleared CTRL3_C, turning IF_INC off and making burst reads return the same
 * register six times. The buffers stay word-sized for _transfer()'s FIFO copy;
 * ms_data_bitlen is what decides how much the part actually sees.
 */

#if defined(PLATFORM_ESP32_C3) && defined(CATALYST_IMU_POC)

#include <Arduino.h>
#include <SPIEx.h>

#include "targets.h"
#include "logging.h"
#include "device.h"

// GPIO0. Hardcoded rather than read from the hardware JSON: the IMU is not part
// of the target definition, and adding a key to the layout would change the
// public target schema for a bench test.
#define IMU_CS_PIN              0

#define IMU_SPI_HZ              10000000    // LSM6DS3TR-C tops out at 10 MHz
#define RADIO_SPI_HZ            17500000    // what SX1280_hal.cpp set, restored after

#define IMU_READ                0x80        // OR into a register address to read

#define REG_WHO_AM_I            0x0F
#define REG_CTRL1_XL            0x10
#define REG_CTRL2_G             0x11
#define REG_CTRL3_C             0x12
#define REG_CTRL4_C             0x13
#define REG_STATUS              0x1E
#define REG_OUTX_L_XL           0x28

#define WHOAMI_LSM6DS3TR_C      0x6A        // the part on the BOM
#define WHOAMI_LSM6DS3          0x69        // older sibling, accepted so a wrong
                                            // part reports itself instead of "dead"

#define CTRL3_C_BDU_IFINC       0x44        // block data update + auto address increment
#define CTRL4_C_I2C_DISABLE     0x04        // SPI only, so a stray I2C edge cannot wake it
#define CTRL1_XL_416HZ_2G       0x60
#define CTRL2_G_416HZ_245DPS    0x60

#define STATUS_XLDA             0x01        // new accelerometer sample ready

#define MG_PER_LSB_2G           0.061f      // datasheet sensitivity at +-2g

#define POC_SAMPLE_INTERVAL     250         // ms between printed samples
#define IMU_WHOAMI_ATTEMPTS     4           // after the throwaway, in case the bus is marginal

static bool imuPresent = false;

/**
 * Full-duplex transfer with the IMU, in place. Takes the bus at the LSM6's
 * clock and mode, drives CS by hand, and hands the bus back exactly as the
 * radio configured it. len must be the whole frame: address byte first, then
 * one placeholder byte per byte to be read back.
 */
static void imuTransfer(uint8_t *buf, uint32_t len)
{
    SPIEx.setFrequency(IMU_SPI_HZ);
    SPIEx.setDataMode(SPI_MODE3);

    digitalWrite(IMU_CS_PIN, LOW);
    SPIEx.read(0, buf, len);            // cs_mask 0 - no hardware CS, radio NSS stays high
    digitalWrite(IMU_CS_PIN, HIGH);

    SPIEx.setDataMode(SPI_MODE0);
    SPIEx.setFrequency(RADIO_SPI_HZ);
}

static uint8_t imuReadReg(uint8_t reg)
{
    // Two bytes on the wire: address out, one byte back.
    alignas(4) uint8_t buf[4] = { (uint8_t)(reg | IMU_READ), 0, 0, 0 };
    imuTransfer(buf, 2);
    return buf[1];
}

static void imuWriteReg(uint8_t reg, uint8_t value)
{
    // Exactly two bytes. Anything longer writes the padding into reg+1, reg+2 -
    // see TRANSFER LENGTH in the header.
    alignas(4) uint8_t buf[4] = { reg, value, 0, 0 };
    imuTransfer(buf, 2);
}

static bool initialize()
{
    // Claim CS early and park it inactive. This runs before setupRadio() brings
    // the bus up, so no transfer can happen yet - but an undriven CS floating
    // low during radio init would leave the IMU listening to radio traffic.
    pinMode(IMU_CS_PIN, OUTPUT);
    digitalWrite(IMU_CS_PIN, HIGH);
    return true;
}

static int start()
{
    // Runs after setupRadio(), so the bus is already begun - configured for the
    // radio, not for us. Burn one transfer to land our clock and mode (see the
    // header), then read for real.
    imuReadReg(REG_WHO_AM_I);

    uint8_t whoami = 0;
    for (uint8_t attempt = 0; attempt < IMU_WHOAMI_ATTEMPTS; attempt++)
    {
        whoami = imuReadReg(REG_WHO_AM_I);
        if (whoami == WHOAMI_LSM6DS3TR_C || whoami == WHOAMI_LSM6DS3)
            break;
    }

    if (whoami == WHOAMI_LSM6DS3TR_C || whoami == WHOAMI_LSM6DS3)
    {
        imuPresent = true;
        DBGLN("IMU POC: WHO_AM_I 0x%x (%s), bus shared with radio OK",
              whoami, whoami == WHOAMI_LSM6DS3TR_C ? "LSM6DS3TR-C" : "LSM6DS3");

        imuWriteReg(REG_CTRL3_C, CTRL3_C_BDU_IFINC);
        imuWriteReg(REG_CTRL4_C, CTRL4_C_I2C_DISABLE);
        imuWriteReg(REG_CTRL1_XL, CTRL1_XL_416HZ_2G);
        imuWriteReg(REG_CTRL2_G, CTRL2_G_416HZ_245DPS);

        // Read CTRL3_C back. Without IF_INC a burst read returns the same
        // register six times, so X, Y and Z come out identical - data that looks
        // plausible and means nothing. Worth proving rather than assuming.
        const uint8_t ctrl3 = imuReadReg(REG_CTRL3_C);
        DBGLN("IMU POC: CTRL3_C 0x%x, IF_INC %s", ctrl3,
              (ctrl3 & 0x04) ? "on" : "OFF - burst reads will not increment");

        return POC_SAMPLE_INTERVAL;
    }

    // Read it twice more. Three identical wrong values point at wiring - 0x00 is
    // MISO stuck low or CS never releasing, 0xFF is MISO floating high. Values
    // that differ run by run point at the bus itself: clock, mode, or the radio
    // interleaving with us.
    const uint8_t retry1 = imuReadReg(REG_WHO_AM_I);
    const uint8_t retry2 = imuReadReg(REG_WHO_AM_I);

    DBGLN("IMU POC: WHO_AM_I 0x%x 0x%x 0x%x - expected 0x%x",
          whoami, retry1, retry2, WHOAMI_LSM6DS3TR_C);
    DBGLN("IMU POC: CS GPIO%d, bus SCK %d MISO %d MOSI %d",
          IMU_CS_PIN, GPIO_PIN_SCK, GPIO_PIN_MISO, GPIO_PIN_MOSI);
    return DURATION_NEVER;
}

static int timeout()
{
    if (!imuPresent)
        return DURATION_NEVER;

    if ((imuReadReg(REG_STATUS) & STATUS_XLDA) == 0)
        return POC_SAMPLE_INTERVAL;

    // 7 bytes on the wire - address plus X, Y, Z as little-endian pairs. The
    // buffer is 8 so _transfer() can copy it as whole words.
    alignas(4) uint8_t buf[8] = { (uint8_t)(REG_OUTX_L_XL | IMU_READ), 0, 0, 0, 0, 0, 0, 0 };
    imuTransfer(buf, 7);

    const int16_t rawX = (int16_t)(buf[1] | (buf[2] << 8));
    const int16_t rawY = (int16_t)(buf[3] | (buf[4] << 8));
    const int16_t rawZ = (int16_t)(buf[5] | (buf[6] << 8));

    const int mgX = (int)(rawX * MG_PER_LSB_2G);
    const int mgY = (int)(rawY * MG_PER_LSB_2G);
    const int mgZ = (int)(rawZ * MG_PER_LSB_2G);

    // Magnitude is the sanity check that matters: whatever way up the board is,
    // a still board must read about 1000 mg total. A number far off that means
    // the axes are being misread, not that the bot is accelerating.
    const int mag = (int)sqrtf((float)mgX * mgX + (float)mgY * mgY + (float)mgZ * mgZ);

    DBGLN("IMU POC: X %d mg  Y %d mg  Z %d mg  |g| %d mg", mgX, mgY, mgZ, mag);

    return POC_SAMPLE_INTERVAL;
}

device_t IMU_POC_device = {
    .initialize = initialize,
    .start = start,
    .event = nullptr,
    .timeout = timeout,
    .subscribe = EVENT_NONE
};

#endif
