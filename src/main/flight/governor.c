/*
 * This file is part of Heliflight 3D.
 *
 * Heliflight 3D is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Heliflight 3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"
#include "config/config.h"

#include "pg/motor.h"

#include "drivers/dshot.h"
#include "drivers/motor.h"
#include "drivers/freq.h"
#include "drivers/time.h"
#include "drivers/io.h"

#include "io/motors.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/rc.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/esc_sensor.h"

#include "flight/rpm_filter.h"
#include "flight/governor.h"
#include "flight/motors.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#define DEBUG_GOV_COMPAT
//#define DEBUG_GOV_TAIL
//#define DEBUG_GOV_PARTS
//#define DEBUG_GOV_PIDSUM
//#define DEBUG_GOV_THROTTLE


PG_REGISTER_WITH_RESET_TEMPLATE(governorConfig_t, governorConfig, PG_GOVERNOR_CONFIG, 0);

PG_RESET_TEMPLATE(governorConfig_t, governorConfig,
    .gov_max_headspeed = 2000,
    .gov_spoolup_time = 10,
    .gov_gear_ratio = 1000,
    .gov_p_gain = 0,
    .gov_i_gain = 0,
    .gov_cyclic_ff_gain = 0,
    .gov_collective_ff_gain = 0,
    .gov_collective_ff_impulse_gain = 0,
    .gov_tailmotor_assist_gain = 0,
);


FAST_RAM_ZERO_INIT float govOutput[MAX_SUPPORTED_MOTORS];

static FAST_RAM_ZERO_INIT float govMaxHeadspeed;
static FAST_RAM_ZERO_INIT float govGearRatio;
static FAST_RAM_ZERO_INIT float govRampRate;

static FAST_RAM_ZERO_INIT float govKp;
static FAST_RAM_ZERO_INIT float govKi;
static FAST_RAM_ZERO_INIT float govCycKf;
static FAST_RAM_ZERO_INIT float govColKf;
static FAST_RAM_ZERO_INIT float govColPulseKf;

static FAST_RAM_ZERO_INIT float govI;
static FAST_RAM_ZERO_INIT float govP;
static FAST_RAM_ZERO_INIT float govPidSum;

static FAST_RAM_ZERO_INIT float govSetpoint;
static FAST_RAM_ZERO_INIT float govSetpointLimited;
static FAST_RAM_ZERO_INIT float govBaseThrottle;

static FAST_RAM_ZERO_INIT float lastSpoolThrottle;
static FAST_RAM_ZERO_INIT timeMs_t lastSpoolEndTime;

static FAST_RAM_ZERO_INIT bool  govSpooledUp;

static FAST_RAM_ZERO_INIT float headSpeed;


void governorInit(void)
{
    govMaxHeadspeed  = (float)governorConfig()->gov_max_headspeed;
    govGearRatio     = (float)governorConfig()->gov_gear_ratio / 1000.0;
    govRampRate      = (float)pidGetDT() / constrainf(governorConfig()->gov_spoolup_time, 1, 30);
    govKp            = (float)governorConfig()->gov_p_gain / 10.0;
    govKi            = (float)governorConfig()->gov_i_gain / 10.0;
    govCycKf         = (float)governorConfig()->gov_cyclic_ff_gain / 100.0;
    govColKf         = (float)governorConfig()->gov_collective_ff_gain / 100.0;
    govColPulseKf    = (float)governorConfig()->gov_collective_ff_impulse_gain / 100.0;
}


void governorUpdate(void)
{
    float throttle = mixerGetThrottle();
    float govMain = 0.0;
    float govTail = 0.0;

    // Handle main motor throttle output & spool-up
    if (getMotorCount() > 0) {

        // If ramp is set to 5s then this will allow 20% change in 1 second, or 10% headspeed in 0.5 seconds.
        float setPointRampRate = govRampRate * govMaxHeadspeed;

        // Calculate headspeed
        headSpeed = getMotorRPM(0) / govGearRatio;

        // Some logic to help us come back from a stage 1 failsafe / glitch / RPM loss / accidental throttle hold quickly
        // We're going to use this time to lock in our govSpooledUp state for a few seconds after throttle = 0 when we were just govSpooledUp on the last pass through
        // Also gives us a few second window if we lose headspeed signal... in that case we'll fall back to the commanded throttle value
        if (govSpooledUp && (throttle == 0.0 || headSpeed < 1000.0) && cmp32(millis(), lastSpoolEndTime) > 5500) {
            // Time check above must be set just a little longer than any of the lastSpoolEndTime checks below.
            // HF3D TODO:  Maybe change the throttle check above to something >0.0 or add some more logic to allow for a faster ramp for autorotation bailout of some kind?
            lastSpoolEndTime = millis();
        }

        // Governor active
        if (govMaxHeadspeed > 0 && throttle > 0.50) {

            // Set the user requested governor headspeed setting
            govSetpoint = throttle * govMaxHeadspeed;

            // If we don't have a non-zero rate limited setpoint yet, set it to the headspeed
            if (govSetpointLimited == 0) {
                govSetpointLimited = headSpeed;
            }

            // Check to see if we've been govSpooledUp recently .  If so, increase our setPointRampRate greatly to help recover from temporary loss of throttle signal.
            // HF3D TODO:  If someone immediately plugged in their heli, armed, and took off throttle hold we could accidentally hit this fast governor ramp.  That would be crazy... but possible I guess?
            if (govSpooledUp && cmp32(millis(),lastSpoolEndTime) < 5000) {
                setPointRampRate *= 7.0f;
            }

            // Increment or decrement the rate limited governor setpoint if needed
            if ((govSetpoint - govSetpointLimited) > setPointRampRate) {
                // Setpoint is higher than the rate limited setpoint, so increment limited setpoint higher
                govSetpointLimited = constrainf(govSetpointLimited + setPointRampRate, govSetpointLimited, govSetpoint);
            } else if ((govSetpointLimited - govSetpoint) > setPointRampRate)  {
                // Setpoint is lower than the rate limited setpoint, so decrement limited setpoint lower
                govSetpointLimited = constrainf(govSetpointLimited - setPointRampRate, govSetpoint, govSetpointLimited);
            } else {
                govSetpointLimited = govSetpoint;
            }
        }
        else {
            // Don't use governor.
            govSetpoint = 0;
            govSetpointLimited = 0;
        }

        // Also disable governor if main motor RPM is not available
        if (headSpeed == 0) {
            govSetpoint = 0;
            govSetpointLimited = 0;
        }


        //----------------- Spoolup Logic ----------------

        // MVP:  If Main Motor RPM <1000, then reset govSpooledUp flag and use govSpooledUp logic
        // HF3D TODO:  Add auto-rotation mode that ignores this check, or at least decreases the ramp time significantly.
        // HF3D TODO:  Clean-up spool-up logic that allows for 2 modes:  Spool on throttle % only (no RPM telemetry), and Spool on Governor (Headspeed)
        //    May be easier to just have 2 sets of code... 1 for spooling on governor and one for spooling without governor??
        // HF3D TODO:
        //  Idea for RPM-based spool-up logic when governor is active...
        //  Bring throttle up to ~25%.  If no RPM detected, then disarm.
        //  Once RPM is detected, start tracking PWM vs. RPM.
        //  If there becomes a large discrepency between PWM ramp and RPM ramp, then stop ramping PWM until ratio comes back in line.
        //  This will prevent the "lag" that sometimes occurs where we end up ramping PWM into oblivion and then the ESC "catches up" later on.
        //  See 4/9/20 log #7

        // Determine spoolup status
        if (headSpeed < 1000 && cmp32(millis(),lastSpoolEndTime) > 3000) {
            // Require heli to spin up slowly if it's been more than 3 seconds since we last had a headspeed above 1000 rpm
            govSpooledUp = false;

        } else if (govSetpoint == 0 && throttle < lastSpoolThrottle) {
            // Governor is disabled, running on throttle % only.
            // If user spools up above 1000rpm, then lowers throttle below the last spool target, allow the heli to be considered spooled up
            govSpooledUp = true;
            lastSpoolThrottle = throttle;        // Allow spool target to reduce with throttle freely even if already spooledUp.
            // HF3D TODO:  There's a bug with this block....
            //   If you spool up above 1000rpm and then come back below that throttle setting (normal mode)
            //   lastSpoolThrottle will ONLY go down since that's all it's allowed to do here.
            //   Then when the governor turns on it gets this really low lastSpoolThrottle value (maybe even zero)
            //      and then throttle drops to zero and the i-term has to wind up the entire amount.

        } else if (!govSpooledUp && govSetpoint > 0 && headSpeed > govSetpoint * 0.97f) {
            // consider the heli to be spooled up
            govSpooledUp = true;
            // Set the governor's base throttle % to our last spooled throttle value
            govBaseThrottle = lastSpoolThrottle;
            // Jump the rate limited Setpoint up to the setpoint.
            govSetpointLimited = govSetpoint * 0.97f;

        } else if (!govSpooledUp && govSetpoint > 0 && lastSpoolThrottle > 0.90f) {
            // Governor is enabled, and we've hit 90% throttle trying to get within 97% the requested headspeed.
            // HF3D TODO:  Flag and alert user in the logs and/or with beep tones after flight that govMaxHeadspeed is set too high.
            govSpooledUp = true;
            // Set the governor's base throttle % to our last spooled throttle value
            govBaseThrottle = lastSpoolThrottle;
            // Jump to the current headspeed
            govSetpointLimited = headSpeed;
        }

        // Handle ramping of throttle

        // HF3D TODO:  Eventually add a "govEnabled" user setting flag.  If the user doesn't have the governor enabled then we need to not use headspeed to perform our govSpooledUp check.
        // Right now we have governor enabled all of of the time, but eventually the code needs to support running with or without an RPM signal (gov or no gov)
        // Skip spooling logic if no throttle signal or if we're disarmed
        if (throttle == 0.0 || !ARMING_FLAG(ARMED)) {
            // Don't reset govSpooledUp flag because we want throttle to respond quickly if user drops throttle in normal mode or re-arms while blades are still spinning > 1000rpm
            //   govSpooledUp flag will reset anyway if RPM < 1000
            lastSpoolThrottle = 0.0;         // Require re-spooling to start from zero if RPM<1000 and throttle=0 or disarmed
        }

        // Governor disabled
        if (govSetpoint == 0.0) {
            // Running on throttle % only.
            if (throttle > lastSpoolThrottle) {
                // Throttle is higher than our last output, spool some more
                lastSpoolThrottle += govRampRate;
                govMain = lastSpoolThrottle;
            } else {
                // Allow spool target to reduce with throttle freely
                lastSpoolThrottle = throttle;
                govMain = lastSpoolThrottle;
            }
        }
        // Governor enabled
        else {
            // If not spooled up and headspeed is lower than our govSetpoint, spool some more
            if (!govSpooledUp && headSpeed < govSetpoint) {
                // Governor is enabled, running on headspeed.
                lastSpoolThrottle += govRampRate;
                govMain = lastSpoolThrottle;
            }
        }
        // --------------- End of Spoolup Logic --------------

#ifdef DEBUG_GOV_THROTTLE
        DEBUG_SET(DEBUG_GOVERNOR, 0, headSpeed);
        DEBUG_SET(DEBUG_GOVERNOR, 1, throttle * 1000);
        DEBUG_SET(DEBUG_GOVERNOR, 2, lastSpoolThrottle * 1000);
        DEBUG_SET(DEBUG_GOVERNOR, 3, govSpooledUp ? 1000 : 0);
#endif


        // --------------- Governor Logic --------------------
        if (govSpooledUp && govSetpoint > 0.0) {

            float govTailmotorAssist = 0.0f;

#ifdef USE_HF3D_ASSISTED_TAIL
            // If using a motor-driven tail, allow the governor to help the tail motor to yaw in the main motor torque direction
            //   It's more important that we maintain tail authority than it is to prevent overspeeds.
            //   Constrain the main motor throttle assist to 15%
            // HF3D TODO:  Using govSetpointLimited to handle the ramp back down after tailmotor assist is a bit of a hack...
            if ((motorCount > 1) && (pidData[FD_YAW].SumLim > 0.0f)) {
                // NOTE:  Make sure to update pidSumHighLimiYaw in pid.c if the 0.15f constraint is changed.
                govTailmotorAssist = constrainf((float)governorConfig()->gov_tailmotor_assist_gain / 100.0f * pidData[FD_YAW].SumLim * MIXER_PID_SCALING, 0.0f, 0.15f);
                // Don't allow the governor to regulate down while we're assisting the tail motor unless we're more than 15% over our governor's headspeed setpoint
                if (headSpeed > govSetpointLimited && headSpeed < govSetpoint*1.15f) {
                    // Increase the rate-limited setpoint so that it tracks the headspeed higher and prevents the govPgain from offsetting our tailmotor assist
                    //   5x ramp rate was chosen because that's about how fast you need to be able to track the headspeed increase for a fast burst of throttle
                    //   Need to increase govSetpointLimited 1x faster than desired ramp rate because it will be decremented by ramp rate in the governor spoolup code above
                    govSetpointLimited = constrainf(govSetpointLimited + 6.0f*setPointRampRate, govSetpointLimited, headSpeed);
                }
            }
#endif
            // Quick and dirty collective pitch linear feed-forward for the main motor
            // Calculate linear feedforward vs. collective stick position (always positive adder)
            //   Reasonable value would be 0.15 throttle addition for 12-degree collective throw..
            //   So gains in the 0.0015 - 0.0032 range depending on where max collective pitch is on the heli
            //   HF3D TODO:  Set this up so works off of a calibrated pitch value for the heli taken during setup
            float govCollectiveFF = govColKf * getCollectiveDeflectionAbs();

            // Collective pitch impulse feed-forward for the main motor
            float govCollectivePulseFF = govColPulseKf * getCollectiveDeflectionAbsHPF();

            // HF3D TODO:  Add a cyclic stick feedforward to the governor - linear gain should be fine.
            // Additional torque is required from the motor when adding cyclic pitch, just like collective (although less)
            // Maybe use this?:  float transition = feedForwardTransition > 0 ? MIN(1.f, getRcDeflectionAbs(axis) * feedForwardTransition) : 1;
            // It's calculated like this in the pid.c code:
            //   Calculate absolute value of the percentage of cyclic stick throw (both combined... but swash ring is the real issue).
            //   servosGetCyclicDeflection() is a 0..1.0f value that is a fraction of the total cyclic travel allowed (usually 10 degrees)
            float govCyclicFF = govCycKf * getCyclicDeflection();

            // Total FeedForward
            float govFeedForward = govCollectiveFF + govCollectivePulseFF + govCyclicFF;

            // Calculate error as a percentage of the max headspeed, since 100% throttle should be close to max headspeed
            // HF3D TODO:  Do we really want the governor to respond the same even if setpoint is only 60% of max?
            //   100 rpm error on 60% of max would "feel" a lot different than 100 rpm error on 90% of max headspeed.
            //   But would it really require any torque differences for the same response??  Maybe, since less inertia in head?
            float govError = (govSetpointLimited - headSpeed) / govMaxHeadspeed;

            // if gov_p_gain = 10 (govKp = 1), we will get 1% change in throttle for 1% error in headspeed
            govP = govKp * govError;

            // if gov_i_gain = 10 (govKi = 1), we will get 1% change in throttle for 1% error in headspeed after 1 second
            govI = constrainf(govI + govKi * govError * pidGetDT(), -50.0, 50.0);

            // Governor PID sum
            govPidSum = govP + govI;

            // HF3D TODO:  Scale the sums based on the average battery voltage?
            //  Note:  This should NOT apply to the tail feedforward compensations that go into the PID controller!
            //         Those compensations are related to the amount of TORQUE only... and this comp would be trying
            //            to keep torque equal, so those shouldn't have to change.

            // Generate our new governed throttle signal
            govMain = govBaseThrottle + govFeedForward + govPidSum + govTailmotorAssist;

            // Reset any wind-up due to excess control signal
            if (govMain > 1.0) {
                // Remove last addition to I-term to prevent further wind-up if it was moving us towards this over-control
                if (govError > 0.0) {
                    govI -= govKi * govError * pidGetDT();
                }
                govMain = 1.0;

            } else if (govMain < 0.0) {
                // Remove last addition to I-term to prevent further wind-up if it was moving us towards this over-control
                // HF3D TODO:  What if I-term was at contraints before we did this?
                if (govError < 0.0) {
                    govI -= govKi * govError * pidGetDT();
                }
                govMain = 0.0;
            }


            // pidGetdT() = targetPidLooptime * 1e-6f;  // 0.00125 for 8kHz
            // Calculate similar to pt1FilterGain with cutoff frequency of 0.05Hz (20s)
            //   RC = 1 / ( 2 * M_PI_FLOAT * f_cut);  ==> RC = 3.183
            //   k = dT / (RC + dT);                  ==>  k = 0.0000393
            // Slowly adapt our govBaseThrottle over time (LPF) as a fallback in case we lose RPM data.
            // HF3D TODO:  If always flying at high collective pitch, that govCollectiveFF will end up adding into the govBaseThrottle
            //    This means the base throttle will be ramped way up if then go back towards a lower average pitch... is that a problem?
            //float govBaseThrottleChange = (throttle - govBaseThrottle) * govBaseThrottleFilterGain;
            //govBaseThrottle += govBaseThrottleChange;

            // Adjust the I-term to account for the base throttle adjustment we just made
            // HF3D TODO:  What if I-term was at contraints before we did this?
            //govI -= govBaseThrottleChange;

            // Set lastSpoolThrottle to track the governor throttle signal when the governor is active
            lastSpoolThrottle = govMain;

#ifdef DEBUG_GOV_PIDSUM
            DEBUG_SET(DEBUG_GOVERNOR, 0, govError * 1000);
            DEBUG_SET(DEBUG_GOVERNOR, 1, govP * 1000);
            DEBUG_SET(DEBUG_GOVERNOR, 2, govI * 1000);
            DEBUG_SET(DEBUG_GOVERNOR, 3, govPidSum * 1000);
#endif
#ifdef DEBUG_GOV_PARTS
            DEBUG_SET(DEBUG_GOVERNOR, 0, govBaseThrottle * 1000);
            DEBUG_SET(DEBUG_GOVERNOR, 1, govFeedForward * 1000);
            DEBUG_SET(DEBUG_GOVERNOR, 2, govPidSum * 1000);
            DEBUG_SET(DEBUG_GOVERNOR, 3, govMain  * 1000);
#endif
        }

        // Enable pass-through of throttle signal if spoolup_time setting = 0 and we are armed.
        if (governorConfig()->gov_spoolup_time == 0 && governorConfig()->gov_max_headspeed == 0 && ARMING_FLAG(ARMED)) {
            govMain = throttle;
        }

#ifdef DEBUG_GOV_COMPAT
        DEBUG_SET(DEBUG_GOVERNOR, 0, govSetpointLimited);
        DEBUG_SET(DEBUG_GOVERNOR, 1, headSpeed);
        DEBUG_SET(DEBUG_GOVERNOR, 2, govPidSum * 1000);
        DEBUG_SET(DEBUG_GOVERNOR, 3, getMotorRPM(1));
#endif
    } // end of Main Motor handling


    // Handle the TAIL motor mixing & control
    // HF3D TODO:  Eventually need to support motor driven + variable pitch combination tails
    if (getMotorCount() > 1) {

        // motorMix for tail motor should be 100% of the inverted stabilized yaw channel
        //  Negative yaw pidSum ==> Tail motor spins
        //  Positive yaw pidSum ==> Main motor gov assist
        float pidSum = pidData[FD_YAW].SumLim * MIXER_PID_SCALING * -1.0f;

        //  For a tail motor.. we don't really want it spinning like crazy from base thrust anytime we're armed,
        //   so tone it down a bit using the main motor throttle as a gain until we're at half our throttle setting or something.
        if (!govSpooledUp) {
            // Track the main motor output while spooling up so that we don't have our tail motor going nuts at zero throttle
            govTail = pidSum * govMain;
        } else {
            govTail = pidSum;
        }

#ifdef USE_THRUST_LINEARIZATION
        // Scale PID sums and throttle to linearize the system (thrust varies with rpm^2)
        //   https://github.com/betaflight/betaflight/pull/7304
        govTail = pidApplyThrustLinearization(govTail);
#endif

        // Tail motor should never stop while flying in the air - if the ESC fails to restart it quickly bad things happen
        // Use dshot_idle_value to prevent tail from stopping when main motor is running
        // HF3D TODO:  Rename dshot_idle_value to tail_motor_idle if we're not using it for anything else?
        if (throttle > 0.0) {
            govTail = constrainf(govTail, motorConfig()->digitalIdleOffsetValue * 0.0001f, 1.0f);
        } else {
            // Allow the tail motor to come to a complete stop when throttle is zero.
            govTail = constrainf(govTail, 0.0, 1.0f);
        }

#ifdef DEBUG_GOV_TAIL
        DEBUG_SET(DEBUG_GOVERNOR, 0, throttle * 1000);
        DEBUG_SET(DEBUG_GOVERNOR, 1, pidSum * 1000);
        DEBUG_SET(DEBUG_GOVERNOR, 2, govTail * 1000);
        DEBUG_SET(DEBUG_GOVERNOR, 3, govMain * 1000);
#endif
    }  // end of tail motor handling

    govOutput[0] = govMain;
    govOutput[1] = govTail;
}


bool isHeliSpooledUp(void)
{
    return govSpooledUp;
}

float getHeadSpeed(void)
{
    return headSpeed;
}

