/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"
#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/dshot_command.h"
#include "drivers/pwm_output.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/motors.h"
#include "flight/setpoint.h"
#include "flight/gps_rescue.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "pid.h"

typedef enum {
    LEVEL_MODE_OFF = 0,
    LEVEL_MODE_R,
    LEVEL_MODE_RP,
} levelMode_e;

const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL;"
    "MAG;";

FAST_RAM_ZERO_INIT uint32_t targetPidLooptime;
FAST_RAM_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT float dT;
static FAST_RAM_ZERO_INIT float pidFrequency;

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 2);

#if defined(STM32F1)
#define PID_PROCESS_DENOM_DEFAULT       8
#elif defined(STM32F3)
#define PID_PROCESS_DENOM_DEFAULT       4
#elif defined(STM32F411xE)
#define PID_PROCESS_DENOM_DEFAULT       2
#else
#define PID_PROCESS_DENOM_DEFAULT       1
#endif

PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT
);

#ifdef USE_ACRO_TRAINER
#define ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT 500.0f  // Max gyro rate for lookahead time scaling
#define ACRO_TRAINER_SETPOINT_LIMIT       1000.0f // Limit the correcting setpoint
#endif // USE_ACRO_TRAINER

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 15);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] =  { 42, 85, 35, 90 },
            [PID_PITCH] = { 46, 90, 38, 95 },
            [PID_YAW] =   { 45, 90, 0, 90 },
            [PID_LEVEL] = { 50, 50, 75, 0 },
            [PID_MAG] =   { 40, 0, 0, 0 },
        },
        .pidSumLimit = PIDSUM_LIMIT,
        .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
        .yaw_lowpass_hz = 0,
        .dterm_notch_hz = 0,
        .dterm_notch_cutoff = 0,
        .itermWindupPointPercent = 100,
        .levelAngleLimit = 55,
        .feedForwardTransition = 0,
        .yawRateAccelLimit = 0,
        .rateAccelLimit = 0,
        .horizon_tilt_effect = 75,
        .horizon_tilt_expert_mode = false,
        .itermLimit = 400,
        .iterm_rotation = false,
        .iterm_relax = ITERM_RELAX_RP,
        .iterm_relax_cutoff = ITERM_RELAX_CUTOFF_DEFAULT,
        .iterm_relax_type = ITERM_RELAX_SETPOINT,
        .acro_trainer_angle_limit = 20,
        .acro_trainer_lookahead_ms = 50,
        .acro_trainer_debug_axis = FD_ROLL,
        .acro_trainer_gain = 75,
        .abs_control_gain = 0,
        .abs_control_limit = 90,
        .abs_control_error_limit = 20,
        .abs_control_cutoff = 11,
        .dterm_lowpass_hz = 150,    // NOTE: dynamic lpf is enabled by default so this setting is actually
                                    // overridden and the static lowpass 1 is disabled. We can't set this
                                    // value to 0 otherwise Configurator versions 10.4 and earlier will also
                                    // reset the lowpass filter type to PT1 overriding the desired BIQUAD setting.
        .dterm_lowpass2_hz = 150,   // second Dterm LPF ON by default
        .dterm_filter_type = FILTER_PT1,
        .dterm_filter2_type = FILTER_PT1,
        .dyn_lpf_dterm_min_hz = 70,
        .dyn_lpf_dterm_max_hz = 170,
        .thrustLinearization = 0,
        .motor_output_limit = 100,
        .auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY,
        .profileName = { 0 },
        .ff_interpolate_sp = FF_INTERPOLATE_AVG2,
        .ff_spike_limit = 60,
        .ff_max_rate_limit = 100,
        .ff_smooth_factor = 37,
        .ff_boost = 15,
        .dyn_lpf_curve_expo = 5,
        .vbat_sag_compensation = 0,
        .yawColKf = 300,
        .yawColPulseKf = 300,
        .yawCycKf = 0,
        .yawBaseThrust = 900,
        .collective_ff_impulse_freq = 100,
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

static void pidSetTargetLooptime(uint32_t pidLooptime)
{
    targetPidLooptime = pidLooptime;
    dT = targetPidLooptime * 1e-6f;
    pidFrequency = 1.0f / dT;
#ifdef USE_DSHOT
    dshotSetPidLoopTime(targetPidLooptime);
#endif
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

typedef union dtermLowpass_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
} dtermLowpass_t;

static FAST_RAM_ZERO_INIT float previousPidSetpoint[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT float collectiveDeflectionAbs;
static FAST_RAM_ZERO_INIT float collectiveDeflectionAbsLPF;
static FAST_RAM_ZERO_INIT float collectiveDeflectionAbsHPF;
static FAST_RAM_ZERO_INIT float collectivePulseFilterGain;

static FAST_RAM_ZERO_INIT filterApplyFnPtr dtermNotchApplyFn;
static FAST_RAM_ZERO_INIT biquadFilter_t dtermNotch[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT filterApplyFnPtr dtermLowpassApplyFn;
static FAST_RAM_ZERO_INIT dtermLowpass_t dtermLowpass[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT filterApplyFnPtr dtermLowpass2ApplyFn;
static FAST_RAM_ZERO_INIT dtermLowpass_t dtermLowpass2[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT filterApplyFnPtr ptermYawLowpassApplyFn;
static FAST_RAM_ZERO_INIT pt1Filter_t ptermYawLowpass;

#if defined(USE_ITERM_RELAX)
static FAST_RAM_ZERO_INIT pt1Filter_t windupLpf[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT uint8_t itermRelax;
static FAST_RAM_ZERO_INIT uint8_t itermRelaxType;
static uint8_t itermRelaxCutoff;
#endif

#if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED FAST_RAM_ZERO_INIT float axisError[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float acGain;
static FAST_RAM_ZERO_INIT float acLimit;
static FAST_RAM_ZERO_INIT float acErrorLimit;
static FAST_RAM_ZERO_INIT float acCutoff;
static FAST_RAM_ZERO_INIT pt1Filter_t acLpf[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float oldSetpointCorrection[XYZ_AXIS_COUNT];
#endif

#ifdef USE_RC_SMOOTHING_FILTER
static FAST_RAM_ZERO_INIT pt1Filter_t setpointDerivativePt1[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT biquadFilter_t setpointDerivativeBiquad[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT bool setpointDerivativeLpfInitialized;
static FAST_RAM_ZERO_INIT uint8_t rcSmoothingDebugAxis;
static FAST_RAM_ZERO_INIT uint8_t rcSmoothingFilterType;
#endif // USE_RC_SMOOTHING_FILTER

static FAST_RAM_ZERO_INIT float ffBoostFactor;
static FAST_RAM_ZERO_INIT float ffSmoothFactor;
static FAST_RAM_ZERO_INIT float ffSpikeLimitInverse;

float pidGetSpikeLimitInverse()
{
    return ffSpikeLimitInverse;
}

float pidGetFfBoostFactor()
{
    return ffBoostFactor;
}

float pidGetFfSmoothFactor()
{
    return ffSmoothFactor;
}


void pidInitFilters(const pidProfile_t *pidProfile)
{
    STATIC_ASSERT(FD_YAW == 2, FD_YAW_incorrect); // ensure yaw axis is 2

    if (targetPidLooptime == 0) {
        // no looptime set, so set all the filters to null
        dtermNotchApplyFn = nullFilterApply;
        dtermLowpassApplyFn = nullFilterApply;
        ptermYawLowpassApplyFn = nullFilterApply;
        return;
    }

    const uint32_t pidFrequencyNyquist = pidFrequency / 2; // No rounding needed

    uint16_t dTermNotchHz;
    if (pidProfile->dterm_notch_hz <= pidFrequencyNyquist) {
        dTermNotchHz = pidProfile->dterm_notch_hz;
    } else {
        if (pidProfile->dterm_notch_cutoff < pidFrequencyNyquist) {
            dTermNotchHz = pidFrequencyNyquist;
        } else {
            dTermNotchHz = 0;
        }
    }

    if (dTermNotchHz != 0 && pidProfile->dterm_notch_cutoff != 0) {
        dtermNotchApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(dTermNotchHz, pidProfile->dterm_notch_cutoff);
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            biquadFilterInit(&dtermNotch[axis], dTermNotchHz, targetPidLooptime, notchQ, FILTER_NOTCH);
        }
    } else {
        dtermNotchApplyFn = nullFilterApply;
    }

    //1st Dterm Lowpass Filter
    uint16_t dterm_lowpass_hz = pidProfile->dterm_lowpass_hz;

#ifdef USE_DYN_LPF
    if (pidProfile->dyn_lpf_dterm_min_hz) {
        dterm_lowpass_hz = pidProfile->dyn_lpf_dterm_min_hz;
    }
#endif

    if (dterm_lowpass_hz > 0 && dterm_lowpass_hz < pidFrequencyNyquist) {
        switch (pidProfile->dterm_filter_type) {
        case FILTER_PT1:
            dtermLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt1FilterInit(&dtermLowpass[axis].pt1Filter, pt1FilterGain(dterm_lowpass_hz, dT));
            }
            break;
        case FILTER_BIQUAD:
#ifdef USE_DYN_LPF
            dtermLowpassApplyFn = (filterApplyFnPtr)biquadFilterApplyDF1;
#else
            dtermLowpassApplyFn = (filterApplyFnPtr)biquadFilterApply;
#endif
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                biquadFilterInitLPF(&dtermLowpass[axis].biquadFilter, dterm_lowpass_hz, targetPidLooptime);
            }
            break;
        default:
            dtermLowpassApplyFn = nullFilterApply;
            break;
        }
    } else {
        dtermLowpassApplyFn = nullFilterApply;
    }

    //2nd Dterm Lowpass Filter
    if (pidProfile->dterm_lowpass2_hz == 0 || pidProfile->dterm_lowpass2_hz > pidFrequencyNyquist) {
    	dtermLowpass2ApplyFn = nullFilterApply;
    } else {
        switch (pidProfile->dterm_filter2_type) {
        case FILTER_PT1:
            dtermLowpass2ApplyFn = (filterApplyFnPtr)pt1FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt1FilterInit(&dtermLowpass2[axis].pt1Filter, pt1FilterGain(pidProfile->dterm_lowpass2_hz, dT));
            }
            break;
        case FILTER_BIQUAD:
            dtermLowpass2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                biquadFilterInitLPF(&dtermLowpass2[axis].biquadFilter, pidProfile->dterm_lowpass2_hz, targetPidLooptime);
            }
            break;
        default:
            dtermLowpass2ApplyFn = nullFilterApply;
            break;
        }
    }

    if (pidProfile->yaw_lowpass_hz == 0 || pidProfile->yaw_lowpass_hz > pidFrequencyNyquist) {
        ptermYawLowpassApplyFn = nullFilterApply;
    } else {
        ptermYawLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
        pt1FilterInit(&ptermYawLowpass, pt1FilterGain(pidProfile->yaw_lowpass_hz, dT));
    }

#if defined(USE_ITERM_RELAX)
    if (itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            pt1FilterInit(&windupLpf[i], pt1FilterGain(itermRelaxCutoff, dT));
        }
    }
#endif
#if defined(USE_ABSOLUTE_CONTROL)
    if (itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            pt1FilterInit(&acLpf[i], pt1FilterGain(acCutoff, dT));
        }
    }
#endif
    ffBoostFactor = (float)pidProfile->ff_boost / 10.0f;
    ffSpikeLimitInverse = pidProfile->ff_spike_limit ? 1.0f / ((float)pidProfile->ff_spike_limit / 10.0f) : 0.0f;

    // HF3D: Collective input impulse high-pass filter.
    // Setting is for cutoff frequency in Hz * 100.
    // Calculate similar to pt1FilterGain with cutoff frequency of 0.05Hz (20s)
    //   RC = 1 / ( 2 * M_PI_FLOAT * f_cut);  ==> RC = 3.183
    //   k = dT / (RC + dT);                  ==>  k = 0.0000393 for 8kHz
    collectivePulseFilterGain = dT / (dT + (1 / ( 2 * 3.14159f * (float)pidProfile->collective_ff_impulse_freq / 100.0f)));
}

#ifdef USE_RC_SMOOTHING_FILTER
void pidInitSetpointDerivativeLpf(uint16_t filterCutoff, uint8_t debugAxis, uint8_t filterType)
{
    rcSmoothingDebugAxis = debugAxis;
    rcSmoothingFilterType = filterType;
    if ((filterCutoff > 0) && (rcSmoothingFilterType != RC_SMOOTHING_DERIVATIVE_OFF)) {
        setpointDerivativeLpfInitialized = true;
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            switch (rcSmoothingFilterType) {
                case RC_SMOOTHING_DERIVATIVE_PT1:
                    pt1FilterInit(&setpointDerivativePt1[axis], pt1FilterGain(filterCutoff, dT));
                    break;
                case RC_SMOOTHING_DERIVATIVE_BIQUAD:
                    biquadFilterInitLPF(&setpointDerivativeBiquad[axis], filterCutoff, targetPidLooptime);
                    break;
            }
        }
    }
}

void pidUpdateSetpointDerivativeLpf(uint16_t filterCutoff)
{
    if ((filterCutoff > 0) && (rcSmoothingFilterType != RC_SMOOTHING_DERIVATIVE_OFF)) {
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            switch (rcSmoothingFilterType) {
                case RC_SMOOTHING_DERIVATIVE_PT1:
                    pt1FilterUpdateCutoff(&setpointDerivativePt1[axis], pt1FilterGain(filterCutoff, dT));
                    break;
                case RC_SMOOTHING_DERIVATIVE_BIQUAD:
                    biquadFilterUpdateLPF(&setpointDerivativeBiquad[axis], filterCutoff, targetPidLooptime);
                    break;
            }
        }
    }
}
#endif // USE_RC_SMOOTHING_FILTER

typedef struct pidCoefficient_s {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} pidCoefficient_t;

static FAST_RAM_ZERO_INIT pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT float maxVelocity[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT float feedForwardTransition;
static FAST_RAM_ZERO_INIT float levelGain, horizonGain, horizonTransition, horizonCutoffDegrees, horizonFactorRatio;
static FAST_RAM_ZERO_INIT float itermWindupPointInv;
static FAST_RAM_ZERO_INIT uint8_t horizonTiltExpertMode;
static FAST_RAM_ZERO_INIT float itermLimit;
static FAST_RAM_ZERO_INIT bool itermRotation;

void pidResetIterm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0.0f;
#if defined(USE_ABSOLUTE_CONTROL)
        axisError[axis] = 0.0f;
#endif
    }
}

#ifdef USE_ACRO_TRAINER
static FAST_RAM_ZERO_INIT float acroTrainerAngleLimit;
static FAST_RAM_ZERO_INIT float acroTrainerLookaheadTime;
static FAST_RAM_ZERO_INIT uint8_t acroTrainerDebugAxis;
static FAST_RAM_ZERO_INIT bool acroTrainerActive;
static FAST_RAM_ZERO_INIT int acroTrainerAxisState[2];  // only need roll and pitch
static FAST_RAM_ZERO_INIT float acroTrainerGain;
#endif // USE_ACRO_TRAINER

#ifdef USE_THRUST_LINEARIZATION
FAST_RAM_ZERO_INIT float thrustLinearization;
FAST_RAM_ZERO_INIT float thrustLinearizationReciprocal;
FAST_RAM_ZERO_INIT float thrustLinearizationB;
#endif

#ifdef USE_DYN_LPF
static FAST_RAM uint8_t dynLpfFilter = DYN_LPF_NONE;
static FAST_RAM_ZERO_INIT uint16_t dynLpfMin;
static FAST_RAM_ZERO_INIT uint16_t dynLpfMax;
static FAST_RAM_ZERO_INIT uint8_t dynLpfCurveExpo;
#endif

#ifdef USE_INTERPOLATED_SP
static FAST_RAM_ZERO_INIT ffInterpolationType_t ffFromInterpolatedSetpoint;
#endif

void pidInitConfig(const pidProfile_t *pidProfile)
{
    if (pidProfile->feedForwardTransition == 0) {
        feedForwardTransition = 0;
    } else {
        feedForwardTransition = 100.0f / pidProfile->feedForwardTransition;
    }

    // Roll axis
    pidCoefficient[FD_ROLL].Kp = ROLL_PTERM_SCALE * pidProfile->pid[FD_ROLL].P;
    pidCoefficient[FD_ROLL].Ki = ROLL_ITERM_SCALE * pidProfile->pid[FD_ROLL].I;
    pidCoefficient[FD_ROLL].Kd = ROLL_DTERM_SCALE * pidProfile->pid[FD_ROLL].D;
    pidCoefficient[FD_ROLL].Kf = ROLL_FF_SCALE    * pidProfile->pid[FD_ROLL].F;

    // Pitch axis
    pidCoefficient[FD_PITCH].Kp = PITCH_PTERM_SCALE * pidProfile->pid[FD_PITCH].P;
    pidCoefficient[FD_PITCH].Ki = PITCH_ITERM_SCALE * pidProfile->pid[FD_PITCH].I;
    pidCoefficient[FD_PITCH].Kd = PITCH_DTERM_SCALE * pidProfile->pid[FD_PITCH].D;
    pidCoefficient[FD_PITCH].Kf = PITCH_FF_SCALE    * pidProfile->pid[FD_PITCH].F;

    // Yaw axis
    pidCoefficient[FD_YAW].Kp = YAW_PTERM_SCALE * pidProfile->pid[FD_YAW].P;
    pidCoefficient[FD_YAW].Ki = YAW_ITERM_SCALE * pidProfile->pid[FD_YAW].I;
    pidCoefficient[FD_YAW].Kd = YAW_DTERM_SCALE * pidProfile->pid[FD_YAW].D;
    pidCoefficient[FD_YAW].Kf = YAW_FF_SCALE    * pidProfile->pid[FD_YAW].F;

    levelGain = pidProfile->pid[PID_LEVEL].P / 10.0f;

    horizonGain = pidProfile->pid[PID_LEVEL].I / 10.0f;
    horizonTransition = (float)pidProfile->pid[PID_LEVEL].D;
    horizonTiltExpertMode = pidProfile->horizon_tilt_expert_mode;
    horizonCutoffDegrees = (175 - pidProfile->horizon_tilt_effect) * 1.8f;
    horizonFactorRatio = (100 - pidProfile->horizon_tilt_effect) * 0.01f;

    maxVelocity[FD_ROLL] = maxVelocity[FD_PITCH] = pidProfile->rateAccelLimit * 100 * dT;
    maxVelocity[FD_YAW] = pidProfile->yawRateAccelLimit * 100 * dT;

    itermWindupPointInv = 1.0f;
    if (pidProfile->itermWindupPointPercent < 100) {
        const float itermWindupPoint = pidProfile->itermWindupPointPercent / 100.0f;
        itermWindupPointInv = 1.0f / (1.0f - itermWindupPoint);
    }
    itermLimit = pidProfile->itermLimit;
    itermRotation = pidProfile->iterm_rotation;

#if defined(USE_ITERM_RELAX)
    itermRelax = pidProfile->iterm_relax;
    itermRelaxType = pidProfile->iterm_relax_type;
    itermRelaxCutoff = pidProfile->iterm_relax_cutoff;
#endif

#ifdef USE_ACRO_TRAINER
    acroTrainerAngleLimit = pidProfile->acro_trainer_angle_limit;
    acroTrainerLookaheadTime = (float)pidProfile->acro_trainer_lookahead_ms / 1000.0f;
    acroTrainerDebugAxis = pidProfile->acro_trainer_debug_axis;
    acroTrainerGain = (float)pidProfile->acro_trainer_gain / 10.0f;
#endif // USE_ACRO_TRAINER

#if defined(USE_ABSOLUTE_CONTROL)
    acGain = (float)pidProfile->abs_control_gain;
    acLimit = (float)pidProfile->abs_control_limit;
    acErrorLimit = (float)pidProfile->abs_control_error_limit;
    acCutoff = (float)pidProfile->abs_control_cutoff;
    float rollCorrection  = -acGain * ROLL_PTERM_SCALE  / ROLL_ITERM_SCALE  * pidCoefficient[FD_ROLL].Kp;
    float pitchCorrection = -acGain * PITCH_PTERM_SCALE / PITCH_ITERM_SCALE * pidCoefficient[FD_PITCH].Kp;
    float yawCorrection   = -acGain * YAW_PTERM_SCALE   / YAW_ITERM_SCALE   * pidCoefficient[FD_YAW].Kp;
    pidCoefficient[FD_ROLL].Ki  = MAX(0.0f, pidCoefficient[FD_ROLL].Ki  + rollCorrection);
    pidCoefficient[FD_PITCH].Ki = MAX(0.0f, pidCoefficient[FD_PITCH].Ki + pitchCorrection);
    pidCoefficient[FD_YAW].Ki   = MAX(0.0f, pidCoefficient[FD_YAW].Ki   + yawCorrection);
#endif

#ifdef USE_DYN_LPF
    if (pidProfile->dyn_lpf_dterm_min_hz > 0) {
        switch (pidProfile->dterm_filter_type) {
        case FILTER_PT1:
            dynLpfFilter = DYN_LPF_PT1;
            break;
        case FILTER_BIQUAD:
            dynLpfFilter = DYN_LPF_BIQUAD;
            break;
        default:
            dynLpfFilter = DYN_LPF_NONE;
            break;
        }
    } else {
        dynLpfFilter = DYN_LPF_NONE;
    }
    dynLpfMin = pidProfile->dyn_lpf_dterm_min_hz;
    dynLpfMax = pidProfile->dyn_lpf_dterm_max_hz;
    dynLpfCurveExpo = pidProfile->dyn_lpf_curve_expo;
#endif

#ifdef USE_THRUST_LINEARIZATION
    thrustLinearization = pidProfile->thrustLinearization / 100.0f;
    if (thrustLinearization != 0.0f) {
        thrustLinearizationReciprocal = 1.0f / thrustLinearization;
        thrustLinearizationB = (1.0f - thrustLinearization) / (2.0f * thrustLinearization);
    }
#endif

#ifdef USE_INTERPOLATED_SP
    ffFromInterpolatedSetpoint = pidProfile->ff_interpolate_sp;
    ffSmoothFactor = 1.0f - ((float)pidProfile->ff_smooth_factor) / 100.0f;
    interpolatedSpInit(pidProfile);
#endif
}

void pidInit(const pidProfile_t *pidProfile)
{
    pidSetTargetLooptime(gyro.targetLooptime); // Initialize pid looptime
    pidInitFilters(pidProfile);
    pidInitConfig(pidProfile);
}

#ifdef USE_ACRO_TRAINER
void pidAcroTrainerInit(void)
{
    acroTrainerAxisState[FD_ROLL] = 0;
    acroTrainerAxisState[FD_PITCH] = 0;
}
#endif // USE_ACRO_TRAINER

#ifdef USE_THRUST_LINEARIZATION
float pidApplyThrustLinearization(float motorOutput)
{
    if (thrustLinearization != 0.0f) {
        if (motorOutput > 0.0f) {
            motorOutput = sqrtf(motorOutput * thrustLinearizationReciprocal +
                                thrustLinearizationB * thrustLinearizationB) - thrustLinearizationB;
        }
    }
    return motorOutput;
}
#endif

void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT
        && dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}

#if defined(USE_ACC)
// calculates strength of horizon leveling; 0 = none, 1.0 = most leveling
STATIC_UNIT_TESTED float calcHorizonLevelStrength(void)
{
    // start with 1.0 at center stick, 0.0 at max stick deflection:
    float horizonLevelStrength = 1.0f - MAX(getRcDeflectionAbs(FD_ROLL), getRcDeflectionAbs(FD_PITCH));

    // 0 at level, 90 at vertical, 180 at inverted (degrees):
    const float currentInclination = MAX(ABS(attitude.values.roll), ABS(attitude.values.pitch)) / 10.0f;

    // horizonTiltExpertMode:  0 = leveling always active when sticks centered,
    //                         1 = leveling can be totally off when inverted
    if (horizonTiltExpertMode) {
        if (horizonTransition > 0 && horizonCutoffDegrees > 0) {
                    // if d_level > 0 and horizonTiltEffect < 175
            // horizonCutoffDegrees: 0 to 125 => 270 to 90 (represents where leveling goes to zero)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations; 0.0 at horizonCutoffDegrees value:
            const float inclinationLevelRatio = constrainf((horizonCutoffDegrees-currentInclination) / horizonCutoffDegrees, 0, 1);
            // apply configured horizon sensitivity:
                // when stick is near center (horizonLevelStrength ~= 1.0)
                //  H_sensitivity value has little effect,
                // when stick is deflected (horizonLevelStrength near 0.0)
                //  H_sensitivity value has more effect:
            horizonLevelStrength = (horizonLevelStrength - 1) * 100 / horizonTransition + 1;
            // apply inclination ratio, which may lower leveling
            //  to zero regardless of stick position:
            horizonLevelStrength *= inclinationLevelRatio;
        } else  { // d_level=0 or horizon_tilt_effect>=175 means no leveling
          horizonLevelStrength = 0;
        }
    } else { // horizon_tilt_expert_mode = 0 (leveling always active when sticks centered)
        float sensitFact;
        if (horizonFactorRatio < 1.01f) { // if horizonTiltEffect > 0
            // horizonFactorRatio: 1.0 to 0.0 (larger means more leveling)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations, goes to 1.0 at inclination==level:
            const float inclinationLevelRatio = (180-currentInclination)/180 * (1.0f-horizonFactorRatio) + horizonFactorRatio;
            // apply ratio to configured horizon sensitivity:
            sensitFact = horizonTransition * inclinationLevelRatio;
        } else { // horizonTiltEffect=0 for "old" functionality
            sensitFact = horizonTransition;
        }

        if (sensitFact <= 0) {           // zero means no leveling
            horizonLevelStrength = 0;
        } else {
            // when stick is near center (horizonLevelStrength ~= 1.0)
            //  sensitFact value has little effect,
            // when stick is deflected (horizonLevelStrength near 0.0)
            //  sensitFact value has more effect:
            horizonLevelStrength = ((horizonLevelStrength - 1) * (100 / sensitFact)) + 1;
        }
    }
    return constrainf(horizonLevelStrength, 0, 1);
}

FAST_CODE_NOINLINE float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, float currentPidSetpoint)
{
    // calculate error angle and limit the angle to the max inclination
    // rcDeflection is in range [-1.0, 1.0]
    float angle = pidProfile->levelAngleLimit * getRcDeflection(axis);
#ifdef USE_GPS_RESCUE
    angle += gpsRescueAngle[axis] / 100; // ANGLE IS IN CENTIDEGREES
#endif
    angle = constrainf(angle, -pidProfile->levelAngleLimit, pidProfile->levelAngleLimit);
    const float errorAngle = angle - ((attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f);
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(GPS_RESCUE_MODE)) {
        // ANGLE mode - control is angle based
        currentPidSetpoint = errorAngle * levelGain;
    } else {
        // HORIZON mode - mix of ANGLE and ACRO modes
        // mix in errorAngle to currentPidSetpoint to add a little auto-level feel
        const float horizonLevelStrength = calcHorizonLevelStrength();
        currentPidSetpoint = currentPidSetpoint + (errorAngle * horizonGain * horizonLevelStrength);
    }
    return currentPidSetpoint;
}
#endif // USE_ACC

#ifdef USE_ACRO_TRAINER

int acroTrainerSign(float x)
{
    return x > 0 ? 1 : -1;
}

// Acro Trainer - Manipulate the setPoint to limit axis angle while in acro mode
// There are three states:
// 1. Current angle has exceeded limit
//    Apply correction to return to limit (similar to pidLevel)
// 2. Future overflow has been projected based on current angle and gyro rate
//    Manage the setPoint to control the gyro rate as the actual angle  approaches the limit (try to prevent overshoot)
// 3. If no potential overflow is detected, then return the original setPoint

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM. We accept the
// performance decrease when Acro Trainer mode is active under the assumption that user is unlikely to be
// expecting ultimate flight performance at very high loop rates when in this mode.
static FAST_CODE_NOINLINE float applyAcroTrainer(int axis, const rollAndPitchTrims_t *angleTrim, float setPoint)
{
    float ret = setPoint;

    if (!FLIGHT_MODE(ANGLE_MODE) && !FLIGHT_MODE(HORIZON_MODE) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
        bool resetIterm = false;
        float projectedAngle = 0;
        const int setpointSign = acroTrainerSign(setPoint);
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        const int angleSign = acroTrainerSign(currentAngle);

        if ((acroTrainerAxisState[axis] != 0) && (acroTrainerAxisState[axis] != setpointSign)) {  // stick has reversed - stop limiting
            acroTrainerAxisState[axis] = 0;
        }

        // Limit and correct the angle when it exceeds the limit
        if ((fabsf(currentAngle) > acroTrainerAngleLimit) && (acroTrainerAxisState[axis] == 0)) {
            if (angleSign == setpointSign) {
                acroTrainerAxisState[axis] = angleSign;
                resetIterm = true;
            }
        }

        if (acroTrainerAxisState[axis] != 0) {
            ret = constrainf(((acroTrainerAngleLimit * angleSign) - currentAngle) * acroTrainerGain, -ACRO_TRAINER_SETPOINT_LIMIT, ACRO_TRAINER_SETPOINT_LIMIT);
        } else {

        // Not currently over the limit so project the angle based on current angle and
        // gyro angular rate using a sliding window based on gyro rate (faster rotation means larger window.
        // If the projected angle exceeds the limit then apply limiting to minimize overshoot.
            // Calculate the lookahead window by scaling proportionally with gyro rate from 0-500dps
            float checkInterval = constrainf(fabsf(gyro.gyroADCf[axis]) / ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT, 0.0f, 1.0f) * acroTrainerLookaheadTime;
            projectedAngle = (gyro.gyroADCf[axis] * checkInterval) + currentAngle;
            const int projectedAngleSign = acroTrainerSign(projectedAngle);
            if ((fabsf(projectedAngle) > acroTrainerAngleLimit) && (projectedAngleSign == setpointSign)) {
                ret = ((acroTrainerAngleLimit * projectedAngleSign) - projectedAngle) * acroTrainerGain;
                resetIterm = true;
            }
        }

        if (resetIterm) {
            pidData[axis].I = 0;
        }

        if (axis == acroTrainerDebugAxis) {
            DEBUG_SET(DEBUG_ACRO_TRAINER, 0, lrintf(currentAngle * 10.0f));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 1, acroTrainerAxisState[axis]);
            DEBUG_SET(DEBUG_ACRO_TRAINER, 2, lrintf(ret));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 3, lrintf(projectedAngle * 10.0f));
        }
    }

    return ret;
}
#endif // USE_ACRO_TRAINER

static float accelerationLimit(int axis, float currentPidSetpoint)
{
    static float previousSetpoint[XYZ_AXIS_COUNT];
    const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];

    if (fabsf(currentVelocity) > maxVelocity[axis]) {
        currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + maxVelocity[axis] : previousSetpoint[axis] - maxVelocity[axis];
    }

    previousSetpoint[axis] = currentPidSetpoint;
    return currentPidSetpoint;
}

static void rotateVector(float v[XYZ_AXIS_COUNT], float rotation[XYZ_AXIS_COUNT])
{
    // rotate v around rotation vector rotation
    // rotation in radians, all elements must be small
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        int i_1 = (i + 1) % 3;
        int i_2 = (i + 2) % 3;
        float newV = v[i_1] + v[i_2] * rotation[i];
        v[i_2] -= v[i_1] * rotation[i];
        v[i_1] = newV;
    }
}

// Iterm Rotation:  Rotate the current iTerm vector properly as the craft rotates on other axes   (Pirouette Compensation)
// HF3D TODO:  Evaluate applicability of full iTerm rotation including Yaw axis.  Unsure if appropriate for helicopter use.
//      ArduCopter Piro Comp = Rotate integrator terms on X & Y axis based on pirouette rotation rate around the Z axis
//      Goal is to keep the swashplate tilted in the original direction of travel as the heli pirouettes
//      They do not rotate iTerm from the Z axis.
// HF3D TODO:  What is the lag from calculation of PID components to the actuation of the servos / change in blade pitch during fast rotations?
//      If the delay is significant, will the heli be in a slightly different z-axis orientation when the control output impacts flight?
//      Should look-forward prediction be used to rotate the error compensation to where it will actually occur in time for all PID gains on Roll & Pitch axis?
//      500 deg/s = 1 degree of rotation in 2ms.   dRonin measured response times are ~20ms.  So 10 degrees of rotation worst case?
//      2000rpm headspeed = 33Hz rotation of blades, and gyroscopic precession means that inputs take 90-degrees of rotation to occur
//        So worst case is that input determined while blade is at the point it needs to be controlled, rotates 90 degrees where CCPM mixing happens, then pitches 90 degrees later.
//        Total lag of 180-degrees of rotation worst case = 15ms @ 2000rpm   (After servos have been commanded and started to actually move)

//   Absolute control was made to address the same attitude issues as iterm_rotation, but without some of the downsides.
//   Absolute Control continuously measures the error of the quads path over stick input, properly rotated into the quads coordinate
//     system, and mixes a correction proportional to that error into the setpoint.  It's as if you noticed every tiny attitude
//     error the quad incurs and provided an instantaneous correction on your TX.  The result is significantly better tracking
//     to sticks, particularly during rotations involving yaw and other difficult situations like throttle blips.
//   Absolute Control will likely eventually replace iterm_rotation, but it is not yet enabled by default.

// YOU SHOULD NOT ENABLE ABSOLUTE CONTROL AND ITERM ROTATION AT THE SAME TIME!
STATIC_UNIT_TESTED void rotateItermAndAxisError()
{
    if (itermRotation
#if defined(USE_ABSOLUTE_CONTROL)
        || acGain > 0 || debugMode == DEBUG_AC_ERROR
#endif
        ) {
        const float gyroToAngle = dT * RAD;
        float rotationRads[XYZ_AXIS_COUNT];
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            // convert the deg/s rotation rate sensed by the gyro into the total radians of angle change during the last time step
            rotationRads[i] = gyro.gyroADCf[i] * gyroToAngle;
        }
#if defined(USE_ABSOLUTE_CONTROL)
        if (acGain > 0 || debugMode == DEBUG_AC_ERROR) {
            // Rotate the calculated absolute control error for each axis based on the 3d rotation sensed by the gryo during the last time step
            rotateVector(axisError, rotationRads);
        }
#endif
        // itermRotation not to be used with absolute control
        if (itermRotation) {
            float v[XYZ_AXIS_COUNT];
            // Grab the old iTerm for each axis
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                v[i] = pidData[i].I;
            }
            // Rotate the iTerm among each axis based on the 3d rotation sensed by the gyro during the last time step
            rotateVector(v, rotationRads );
            // Overwrite the old iTerms with the rotated iTerms
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                pidData[i].I = v[i];
            }
        }
    }
}

#ifdef USE_RC_SMOOTHING_FILTER
float FAST_CODE applyRcSmoothingDerivativeFilter(int axis, float pidSetpointDelta)
{
    float ret = pidSetpointDelta;
    if (axis == rcSmoothingDebugAxis) {
        DEBUG_SET(DEBUG_RC_SMOOTHING, 1, lrintf(pidSetpointDelta * 100.0f));
    }
    if (setpointDerivativeLpfInitialized) {
        switch (rcSmoothingFilterType) {
            case RC_SMOOTHING_DERIVATIVE_PT1:
                ret = pt1FilterApply(&setpointDerivativePt1[axis], pidSetpointDelta);
                break;
            case RC_SMOOTHING_DERIVATIVE_BIQUAD:
                ret = biquadFilterApplyDF1(&setpointDerivativeBiquad[axis], pidSetpointDelta);
                break;
        }
        if (axis == rcSmoothingDebugAxis) {
            DEBUG_SET(DEBUG_RC_SMOOTHING, 2, lrintf(ret * 100.0f));
        }
    }
    return ret;
}
#endif // USE_RC_SMOOTHING_FILTER


//  YOU SHOULD NOT ENABLE ABSOLUTE CONTROL AND ITERM ROTATION AT THE SAME TIME!
//    HF3D TODO:  At least not until I make them compatible with each other.  :)
//  Absolute Control needs to be used with iTermRelax to avoid bounce-backs due to the latency between stick movement and quad response.
//    iTermRelax will then suspend AbsoluteControl error accumulation as well during quick moves.  Finally, AbsoluteControl only kicks in
//    once the throttle minimum for airmode activation is exceeded to avoid undue corrections on the ground.
//  Absolute control was made to address the same attitude issues as iterm_rotation, but without some of the downsides.
//  Absolute Control continuously measures the error of the quads orientation versus stick input, properly rotated into the quads coordinate
//     system, and mixes a correction proportional to that error into the setpoint.  It's as if you noticed every tiny attitude
//     error the quad incurs and provided an instantaneous correction on your TX.  The result is significantly better tracking
//     to sticks, particularly during rotations involving yaw and other difficult situations like throttle blips.
#if defined(USE_ITERM_RELAX)
#if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED void applyAbsoluteControl(const int axis, const float gyroRate, float *currentPidSetpoint, float *itermErrorRate)
{
    if (acGain > 0 || debugMode == DEBUG_AC_ERROR) {
        // Apply low-pass filter that was initialized with the pidProfile->abs_control_cutoff frequency to the roll rate command on this axis
        const float setpointLpf = pt1FilterApply(&acLpf[axis], *currentPidSetpoint);
        // Create high-pass filter by subtracting the commanded roll rate from the low-pass filtered version of itself
        const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);
        float acErrorRate = 0;

        // NOTE:  This function runs on EACH AXIS INDEPENDENTLY.
        //   Even though one axis may have a fast stick movement, the other two axes may not.
        //   The other two axes will have full absolute control correction applied to them.

        // Create window around the low-pass filtered signal value
        //   No change in stick position ==> Lpf = commanded rate, Hpf = 0
        //      So, no stick movement = no window.  The window collapses to just the commanded rate on that axis.
        //   Fast stick movement ==>  Lpf = smoothed command rate, 1>Hpf>0
        // Note:  Commanded rate could also be a result of self-leveling or other modes
        // If roll rate sensed by Gyro is inside a window around the user's commanded setpoint,
        //   then we'll accumulate the full error.
        const float gmaxac = setpointLpf + 2 * setpointHpf;
        const float gminac = setpointLpf - 2 * setpointHpf;
        // Check to see if the roll rate sensed by the gyro is within the window of roll rates we created
        if (gyroRate >= gminac && gyroRate <= gmaxac) {
            const float acErrorRate1 = gmaxac - gyroRate;
            const float acErrorRate2 = gminac - gyroRate;
            // axisError is initialized to zero with pidInit and is only used by Absolute Control to accumulate error for each axis
            // Note:  axisError has already been compensated for the rotation of the aircraft that occurred during the last timestep dT
            if (acErrorRate1 * axisError[axis] < 0) {
                acErrorRate = acErrorRate1;
            } else {
                acErrorRate = acErrorRate2;
            }
            if (fabsf(acErrorRate * dT) > fabsf(axisError[axis]) ) {
                acErrorRate = -axisError[axis] * pidFrequency;
            }
        } else {
            // Roll rate sensed by gyro was outside of the window around the user's commanded Setpoint
            // Set the absolute control error rate to the maximum or minimum edges of the window and subtract the actual gyro rate
            // If no change in stick position, then this will simply be rcCommandRate - gyroRate   (so just the angle rate error)
            acErrorRate = (gyroRate > gmaxac ? gmaxac : gminac ) - gyroRate;
        }

        // Check to ensure we are spooled up at a reasonable level
        if (isHeliSpooledUp()) {
            // Integrate the angle rate error, which gives us the accumulated angle error for this axis
            //  Limit the total angle error to the range defined by pidProfile->abs_control_error_limit
            axisError[axis] = constrainf(axisError[axis] + acErrorRate * dT,
                -acErrorLimit, acErrorLimit);
            // Apply a proportional gain (abs_control_gain) to get a desired amount of correction
            //  Limit the total correction to the range defined by pidProfile->abs_control_limit
            const float acCorrection = constrainf(axisError[axis] * acGain, -acLimit, acLimit);
            // Manipulate the commanded roll rate on this axis by adding in the absolute control correction
            *currentPidSetpoint += acCorrection;
            *itermErrorRate += acCorrection;
            DEBUG_SET(DEBUG_AC_CORRECTION, axis, lrintf(acCorrection * 10));
            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 3, lrintf(acCorrection * 10));
            }
        }

        DEBUG_SET(DEBUG_AC_ERROR, axis, lrintf(axisError[axis] * 10));
    }
}
#endif

// Absolute Control needs to be used with iTermRelax to avoid bounce-backs due to the latency between stick movement and quad response.
// iTerm Relax:     https://www.youtube.com/watch?v=VhBL5u_g2LQ
//                  https://www.youtube.com/watch?v=QfiGTG5LfCk
//  iTerm accumulates error over the time (it looks into the past) and applies a correction for steady-state error
//  iTerm Relax accounts for the fact that fast rotation of the aircraft always has a slight lag versus the commanded rotation
//  This mechanical lag causes the iTerm to wind up over the course of a fast maneuver, which causes "bounceback"
//  iTerm Relax prevents iTerm from growing very fast during fast accelerations of the control stick.
//    This means that the iTerm will not wind up during those "transition" periods at the beginning or the end of a movement of the control stick.
//  When the control stick is in a fixed position iTerm Relax will not impact the growth of iTerm.

// The lower the cutoff frequency the more that iTerm Relax will limit the change of iTerm during stick accelerations
//   The higher the cutoff frequency the shorter the window of time will be where the limitation is occuring
//   Default is 20Hz which is good for 5" miniquads.  Bigger quads can probably be lowered to 15Hz.  Requires testing.

// RP = Roll, Pitch axis only
// RPY = Roll, Pitch, Yaw
// RP_INC = Inc versions are only limiting the GROWTH of the iterm, while lowering of iTerm is not constrained
// RPY_INC = Inc versions are only limiting the GROWTH of the iterm, while lowering of iTerm is not constrained
// Setpoint mode applies a high-pass filter to RC input, resulting in a value that gets higher whenever the sticks are moved quickly.
//   When the rate of change is zero (sticks are not moving), iTerm accumulation is normal.
//   Accumulation is then attenuated linearly as the stick movement approaches a threshold.  Above threshold, no iTerm accumulation occurs at all.
//   Tracks the actual movement on your stick, more suited for racer where smoothness is not required
//   Setpoint is when you want the craft to go exactly where you told it to go
// Gyro mode uses a high-pass filter based on rate of change of stick movement, and uses this to create a window either side of the gyro value inside
//   which the quad should be tracking.  While inside the window, no iTerm accumulation occurs.  If the sticks are held still, the window compresses
//   back to nothing, and iTerm accumulation becomes normal again.
//   Gyro algorithm gives you more freestyle feel where result will be slightly smoother flight


// With iTerm relax we can push iTerm much higher than before without unfortunately rollback/bounceback
//    Up to 50% higher iTerm can work when using iTermRelax.
// HF3D TODO:  Evaluate iTerm relax cutoff frequency for helicopter use
//   (stick - stick lpf) = stick hpf, nice explanation Pawel. Also for planes ? I know current implementation for planes.
//   "No, planes will still have current Iterm limiting in iNav, we are not changing that.
//      Planes are not that agile as drones and it would require very low cutoff frequency for Iterm relax to work"
STATIC_UNIT_TESTED void applyItermRelax(const int axis, const float iterm,
    const float gyroRate, float *itermErrorRate, float *currentPidSetpoint)
{
    // Apply low-pass filter that was initialized with the pidProfile->iterm_relax_cutoff frequency to the roll rate command on this axis
    const float setpointLpf = pt1FilterApply(&windupLpf[axis], *currentPidSetpoint);
    // High pass filter = original signal minus the low-pass filtered version of that signal
    const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);
    //   No change in stick position ==> Lpf = commanded rate, Hpf = 0
    //   Fast stick movement ==>  Lpf = smoothed command rate, 1>Hpf>0
    // Note:  Commanded rate could also be a result of self-leveling or other modes

    if (itermRelax) {
        if (axis < FD_YAW || itermRelax == ITERM_RELAX_RPY || itermRelax == ITERM_RELAX_RPY_INC) {
            const float itermRelaxFactor = MAX(0, 1 - setpointHpf / ITERM_RELAX_SETPOINT_THRESHOLD);
            const bool isDecreasingI =
                ((iterm > 0) && (*itermErrorRate < 0)) || ((iterm < 0) && (*itermErrorRate > 0));
            if ((itermRelax >= ITERM_RELAX_RP_INC) && isDecreasingI) {
                // Do Nothing and allow iTerm to decrease normally.  Use the precalculed itermErrorRate
            } else if (itermRelaxType == ITERM_RELAX_SETPOINT) {
                // Change iTerm accumulation factor based only on the speed of the stick movement
                *itermErrorRate *= itermRelaxFactor;
            } else if (itermRelaxType == ITERM_RELAX_GYRO ) {
                // Accumulate iTerm if our gyro movement rate is within a window defined by our stick movement rate
                // Otherwise don't allow iTerm accumulation if the aircraft isn't tracking the commanded roll rate yet.
                *itermErrorRate = fapplyDeadband(setpointLpf - gyroRate, setpointHpf);
            } else {
                *itermErrorRate = 0.0f;
            }

            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 0, lrintf(setpointHpf));
                DEBUG_SET(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor * 100.0f));
                DEBUG_SET(DEBUG_ITERM_RELAX, 2, lrintf(*itermErrorRate));
            }
        }

#if defined(USE_ABSOLUTE_CONTROL)
        applyAbsoluteControl(axis, gyroRate, currentPidSetpoint, itermErrorRate);
#endif
    }
}
#endif


// Betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
// Based on 2DOF reference design (matlab)
void FAST_CODE pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    static float previousGyroRateDterm[XYZ_AXIS_COUNT];
#ifdef USE_INTERPOLATED_SP
    static FAST_RAM_ZERO_INIT uint32_t lastFrameNumber;
#endif

#if defined(USE_ACC)
    static timeUs_t levelModeStartTimeUs = 0;
    static bool gpsRescuePreviousState = false;
#endif

#if defined(USE_ACC)
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
#else
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);
#endif

#if defined(USE_ACC)
    const bool gpsRescueIsActive = FLIGHT_MODE(GPS_RESCUE_MODE);
    levelMode_e levelMode;
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE) || gpsRescueIsActive) {
        levelMode = LEVEL_MODE_RP;
    } else {
        levelMode = LEVEL_MODE_OFF;
    }

    // Keep track of when we entered a self-level mode so that we can
    // add a guard time before crash recovery can activate.
    // Also reset the guard time whenever GPS Rescue is activated.
    if (levelMode) {
        if ((levelModeStartTimeUs == 0) || (gpsRescueIsActive && !gpsRescuePreviousState)) {
            levelModeStartTimeUs = currentTimeUs;
        }
    } else {
        levelModeStartTimeUs = 0;
    }

    gpsRescuePreviousState = gpsRescueIsActive;
#endif

    // Precalculate gyro delta for D-term here, this allows loop unrolling
    float gyroRateDterm[XYZ_AXIS_COUNT];
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        gyroRateDterm[axis] = gyro.gyroADCf[axis];
        gyroRateDterm[axis] = dtermNotchApplyFn((filter_t *) &dtermNotch[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = dtermLowpassApplyFn((filter_t *) &dtermLowpass[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = dtermLowpass2ApplyFn((filter_t *) &dtermLowpass2[axis], gyroRateDterm[axis]);
    }

    // HF3D:  iTermRotation acts as FFF Pirouette Compensation on a heli.
    //   Will not work properly unless the hover roll compensation is outside of the roll integral in the PID controller.
    rotateItermAndAxisError();

#ifdef USE_RPM_FILTER
    rpmFilterUpdate();
#endif

#ifdef USE_INTERPOLATED_SP
    bool newRcFrame = false;
    if (lastFrameNumber != getRcFrameNumber()) {
        lastFrameNumber = getRcFrameNumber();
        newRcFrame = true;
    }
#endif

    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {

        float currentPidSetpoint = getSetpointRate(axis);
        if (maxVelocity[axis]) {
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }

        // Yaw control is GYRO based, direct sticks control is applied to rate PID
#if defined(USE_ACC)
        switch (levelMode) {
        case LEVEL_MODE_OFF:
            break;
        case LEVEL_MODE_R:
            if (axis == FD_PITCH)
                break;
            FALLTHROUGH;
        case LEVEL_MODE_RP:
            if (axis == FD_YAW) {
                // HF3D:  No yaw input while corrections are occuring
                currentPidSetpoint = 0.0f;
                break;
            }
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
        }
#endif

#ifdef USE_ACRO_TRAINER
        if ((axis != FD_YAW) && acroTrainerActive) {
            currentPidSetpoint = applyAcroTrainer(axis, angleTrim, currentPidSetpoint);
        }
#endif // USE_ACRO_TRAINER

        // -----calculate error rate
        const float gyroRate = gyro.gyroADCf[axis];
        float errorRate = currentPidSetpoint - gyroRate;

        const float previousIterm = pidData[axis].I;
        float itermErrorRate = errorRate;

#ifdef USE_ABSOLUTE_CONTROL
        float uncorrectedSetpoint = currentPidSetpoint;
#endif

#if defined(USE_ITERM_RELAX)
        {
            applyItermRelax(axis, previousIterm, gyroRate, &itermErrorRate, &currentPidSetpoint);
            errorRate = currentPidSetpoint - gyroRate;
        }
#endif
#ifdef USE_ABSOLUTE_CONTROL
        float setpointCorrection = currentPidSetpoint - uncorrectedSetpoint;
#endif

        // --------low-level gyro-based PID based on 2DOF PID controller. ----------
        // 2-DOF PID controller with optional filter on derivative term.
        // b = 1 and only c (feedforward weight) can be tuned (amount derivative on measurement or error).

        // -----calculate P component
        pidData[axis].P = pidCoefficient[axis].Kp * errorRate;
        if (axis == FD_YAW) {
            pidData[axis].P = ptermYawLowpassApplyFn((filter_t *) &ptermYawLowpass, pidData[axis].P);
        }

        // -----calculate I component
        float Ki = pidCoefficient[axis].Ki;
        pidData[axis].I = constrainf(previousIterm + Ki * dT * itermErrorRate, -itermLimit, itermLimit);

        // -----calculate pidSetpointDelta
        float pidSetpointDelta = 0;
#ifdef USE_INTERPOLATED_SP
        if (ffFromInterpolatedSetpoint) {
            pidSetpointDelta = interpolatedSpApply(axis, newRcFrame, ffFromInterpolatedSetpoint);
        } else
#endif
        {
            pidSetpointDelta = currentPidSetpoint - previousPidSetpoint[axis];
        }
        previousPidSetpoint[axis] = currentPidSetpoint;


#ifdef USE_RC_SMOOTHING_FILTER
        pidSetpointDelta = applyRcSmoothingDerivativeFilter(axis, pidSetpointDelta);
#endif // USE_RC_SMOOTHING_FILTER

        // -----calculate D component
        if (pidCoefficient[axis].Kd > 0) {

            // Divide rate change by dT to get differential (ie dr/dt).
            // dT is fixed and calculated from the target PID loop time
            // This is done to avoid DTerm spikes that occur with dynamically
            // calculated deltaT whenever another task causes the PID
            // loop execution to be delayed.
            const float delta = (previousGyroRateDterm[axis] - gyroRateDterm[axis]) * pidFrequency;
            pidData[axis].D = pidCoefficient[axis].Kd * delta;
        } else {
            pidData[axis].D = 0;
        }
        previousGyroRateDterm[axis] = gyroRateDterm[axis];

        // -----calculate feedforward component
#ifdef USE_ABSOLUTE_CONTROL
        // include abs control correction in FF
        pidSetpointDelta += setpointCorrection - oldSetpointCorrection[axis];
        oldSetpointCorrection[axis] = setpointCorrection;
#endif

        // Only enable feedforward for rate mode (flightModeFlag=0 is acro/rate mode)
        const float feedforwardGain = (flightModeFlags) ? 0.0f : pidCoefficient[axis].Kf;

        if (feedforwardGain > 0) {
            // transition = 1 if feedForwardTransition == 0   (no transition)
            float transition = feedForwardTransition > 0 ? MIN(1.f, getRcDeflectionAbs(axis) * feedForwardTransition) : 1.0f;

            // HF3D:  Direct stick feedforward for roll and pitch.  Stick delta feedforward for yaw.
            // Let's do direct stick feedforward for roll & pitch, and let's AMP IT UP A LOT.
            // 0.013754 * 90 * 1 * 60 deg/s = 74 output for 100 feedForward gain
            float feedForward = feedforwardGain * 90.0f * transition * currentPidSetpoint;
            if (axis == FD_YAW) {
                feedForward = feedforwardGain * transition * pidSetpointDelta * pidFrequency;    //  Kf * 1 * 20 deg/s * 8000
            }

#ifdef USE_INTERPOLATED_SP
            // HF3D:  Only apply feedforward interpolation limits to the Yaw axis since we're using direct feedforward on the roll and pitch axes.
            if (axis == FD_YAW) {
                pidData[axis].F = shouldApplyFfLimits(axis) ?
                    applyFfLimit(axis, feedForward, pidCoefficient[axis].Kp, currentPidSetpoint) : feedForward;
            } else {
               pidData[axis].F = feedForward;
            }
#else
            pidData[axis].F = feedForward;
#endif
        } else {
            pidData[axis].F = 0;
        }

         // HF3D:  Calculate tail feedforward precompensation and add it to the pidSum on the Yaw channel
        if (axis == FD_YAW) {

            // Calculate absolute value of the percentage of collective stick throw
            if ((rcCommand[COLLECTIVE] >= 500) || (rcCommand[COLLECTIVE] <= -500)) {
                collectiveDeflectionAbs = 100.0f;
            } else {
                if (rcCommand[COLLECTIVE] >= 0) {
                    collectiveDeflectionAbs = (rcCommand[COLLECTIVE] * 100.0f) / (PWM_RANGE_MAX - rxConfig()->midrc);
                } else if (rcCommand[COLLECTIVE] < 0) {
                    collectiveDeflectionAbs = (rcCommand[COLLECTIVE] * -100.0f) / (rxConfig()->midrc - PWM_RANGE_MIN);
                }
            }

            // Collective pitch impulse feed-forward for the main motor
            // Run collectiveDeflectionAbs through a low pass filter
            collectiveDeflectionAbsLPF = collectiveDeflectionAbsLPF + collectivePulseFilterGain * (collectiveDeflectionAbs - collectiveDeflectionAbsLPF);
            // Subtract LPF from the original value to get a high pass filter
            // HPF value will be <60% or so of the collectiveDeflectionAbs, and will be smaller the slower the stick movement is.
            //  Cutoff frequency determines this action.
            collectiveDeflectionAbsHPF = collectiveDeflectionAbs - collectiveDeflectionAbsLPF;

            // HF3D TODO:  Negative because of clockwise main rotor spin direction -> CCW body torque on helicopter
            //   Implement a configuration parameter for rotor rotation direction
            float tailCollectiveFF = -1.0f * collectiveDeflectionAbs * pidProfile->yawColKf / 100.0f;
            float tailCollectivePulseFF = -1.0f * collectiveDeflectionAbsHPF * pidProfile->yawColPulseKf / 100.0f;
            float tailBaseThrust = -1.0f * pidProfile->yawBaseThrust / 10.0f;

            // Calculate absolute value of the percentage of cyclic stick throw (both combined... but swash ring is the real issue).
            float tailCyclicFF = -1.0f * getCyclicDeflection() * 100.0f * pidProfile->yawCycKf / 100.0f;

            // Main motor torque increase from the ESC is proportional to the absolute change in average voltage (NOT percent change in average voltage)
            //     and it is linear with the amount of change.
            // Main motor torque will always cause the tail to rotate in in the same direction, so we just need to apply this offset in the correct direction to counter-act that torque.
            // For CW rotation of the main rotor, the torque will turn the body of the helicopter CCW
            //   This means the leading edge of the tail blade needs to tip towards the left side of the helicopter to counteract it.
            //   Yaw stick right = clockwise rotation = tip of tail blade to left = POSITIVE servo values observed on the X3
            //      NOTE:  Servo values required to obtain a certain direction of change in the tail depends on which side the servo is mounted and if it's a leading or trailing link!!
            //      It is very important to set the servo reversal configuration up correctly so the servo moves in the same direction as the Preview model!
            //   Yaw stick right = positive control channel PWM values from the TX also.
            //   Smix on my rudder channel is setup for -100... so it will take NEGATIVE values in the pidSum to create positive servo movement!

            //   So my collective comp also needs to make positive tail servo values, which means that I need a NEGATIVE adder to the pidSum due to the channel rate reversal.
            //     But, this should always work for all helis that are also setup correctly, regardless of servo reversal...
            //     ... as long as it yaws the correct direction to match the Preview model.
            //     ... and as long as the main motor rotation direction is CW!  CCW rotation will require reversing this and generating POSITIVE pidSum additions!
            // HF3D TODO:  Add a "main rotor rotation direction" configuration parameter.

            // HF3D TODO:  Consider adding a delay in here to allow time for other things to occur...
            //  Tail servo can probably add pitch faster than the swash servos can add collective pitch
            //   and it's also probably faster than the ESC can add torque to the main motor (for gov impulse)
            //  But for motor-driven tails the delay may not be needed... depending on how fast the ESC+motor
            //   can spin up the tail blades relative to the other pieces of the puzzle.

            // Add our collective feedforward terms into the yaw axis pidSum as long as we don't have a motor driven tail
            // Only if we're armed and throttle > 15 use the tail feedforwards.  If we're auto-rotating then there's no use for all this stuff.  It will just screw up our tail position since there's no main motor torque!!
            // HF3D TODO:  Add a configurable override for this check in case someone wants to run an external governor without passing the throttle signal through the flight controller?
            if ((calculateThrottlePercentAbs() > 15) || (!ARMING_FLAG(ARMED))) {
                // if disarmed, show the user what they will get regardless of throttle value
                pidData[FD_YAW].F += tailCollectiveFF + tailCollectivePulseFF + tailBaseThrust + tailCyclicFF;
            }

            // HF3D TODO:  Do some integration of the motor driven tail code here for motorCount == 2...
            //   But have to be careful, because if main motor throttle goes near zero then we'll never get the tail back if we're
            //   adding feedforward that doesn't need to be there.  We can't create negative thrust with a motor driven fixed pitch tail.
        }

        // calculating the PID sum
        pidData[axis].Sum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;

        // Limited PID sum
        const float pidLimit = (axis == FD_YAW) ? currentPidProfile->pidSumLimitYaw : currentPidProfile->pidSumLimit;
        pidData[axis].SumLim = constrainf(pidData[axis].Sum, -pidLimit, pidLimit);
    }

    // Disable PID control if gyro overflow detected
    // This may look very inefficient, but it is done on purpose to always show real CPU usage as in flight
    if (gyroOverflowDetected()) {
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;
            pidData[axis].F = 0;
            pidData[axis].Sum = 0;
            pidData[axis].SumLim = 0;
        }
    }
}

#ifdef USE_ACRO_TRAINER
void pidSetAcroTrainerState(bool newState)
{
    if (acroTrainerActive != newState) {
        if (newState) {
            pidAcroTrainerInit();
        }
        acroTrainerActive = newState;
    }
}
#endif // USE_ACRO_TRAINER

#ifdef USE_DYN_LPF
void dynLpfDTermUpdate(float throttle)
{
    unsigned int cutoffFreq;

    if (dynLpfFilter != DYN_LPF_NONE) {
        if (dynLpfCurveExpo > 0) {
            cutoffFreq = dynDtermLpfCutoffFreq(throttle, dynLpfMin, dynLpfMax, dynLpfCurveExpo);
        } else {
            cutoffFreq = fmax(dynThrottle(throttle) * dynLpfMax, dynLpfMin);
        }

        if (dynLpfFilter == DYN_LPF_PT1) {
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&dtermLowpass[axis].pt1Filter, pt1FilterGain(cutoffFreq, dT));
            }
        } else if (dynLpfFilter == DYN_LPF_BIQUAD) {
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterUpdateLPF(&dtermLowpass[axis].biquadFilter, cutoffFreq, targetPidLooptime);
            }
        }
    }
}
#endif

float dynDtermLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo)
{
    float expof = expo / 10.0f;
    float curve = throttle * (1 - throttle) * expof + throttle;
    return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
}

float pidGetPreviousSetpoint(int axis)
{
    return previousPidSetpoint[axis];
}

float pidGetDT()
{
    return dT;
}

float pidGetPidFrequency()
{
    return pidFrequency;
}

float getCollectiveDeflectionAbs()
{
    return collectiveDeflectionAbs;
}

float getCollectiveDeflectionAbsHPF()
{
    return collectiveDeflectionAbsHPF;
}

