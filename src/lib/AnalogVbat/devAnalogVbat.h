#pragma once

#if defined(TARGET_RX)

#include "common.h"
#include "device.h"

void Vbat_enableSlowUpdate(bool enable);
void Vbat_setCalibrationActive(bool active);

#if defined(CATALYST_DIAG)
// Raw per-sample VBAT excursion tracking (Catalyst diagnostics build only).
// Telemetry is fed by a 5-deep median-average filter whose entire purpose is
// rejecting outliers - which is exactly the regen/sag transient we are hunting.
// These report the UNSMOOTHED extremes instead, in millivolts. Resolution is
// still bounded by the 20ms (DEBUG_VBAT_ADC) sample tick, so this catches an
// excursion lasting tens of ms - a visible LED pulse - not a microsecond spike.
void Vbat_diagGetExtremes(int32_t *loMv, int32_t *hiMv, int32_t *lastMv);
void Vbat_diagResetExtremes();
#endif

extern device_t AnalogVbat_device;

#endif
