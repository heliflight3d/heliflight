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
FAST_RAM_ZERO_INIT uint8_t govState;

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

static FAST_RAM_ZERO_INIT float headSpeed;

static FAST_RAM_ZERO_INIT float govMainPrevious;
static FAST_RAM_ZERO_INIT timeMs_t stateEntryTime;

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

    govState = GS_THROTTLE_OFF;
}

void changeGovStateTo(uint8_t futureState)
{
    // Set the next governor state
    govState = futureState;
    stateEntryTime = millis();
}

bool headSpeedValid(void)
{
    // Check for valid headspeed signal
    if (headSpeed > 25) {
        return true;
    }
    return false;
}

void governorUpdate(void)
{
    // Other code looks to governor.c for headspeed, update it on every loop.
    headSpeed = getMotorRPM(0) / govGearRatio;

    float throttle = mixerGetThrottle();
    float govMain = 0.0;
    float govTail = 0.0;

    // If we're disarmed or no motors, reset the governor state and variables, then exit.
    //   This method does not preserve looptime while disarmed, but it is *safe*.
    // HF3D TODO:  Disarming in flight is catastrophic in many respects throughout the code.  Should it be?
    if (!ARMING_FLAG(ARMED) || getMotorCount() < 1) {
        changeGovStateTo(GS_THROTTLE_OFF);
        govMainPrevious = 0.0;
        govOutput[0] = 0.0;
        govOutput[1] = 0.0;
        return;
    }

    // Determine the main motor output based on the current governor state
    switch (govState)
    {
    case GS_THROTTLE_OFF:
        // -- State complete
        // Reset gov parameters
        govMainPrevious = 0.0;
        govMain = 0.0;
        // If we now have a throttle signal, transition to a new gov state on the next loop iteration
        // HF3D TODO:  min_check instead of 0 throttle?  Or no?
        if (throttle >= 0.20 && govMaxHeadspeed > 0) {
            // Governor only initiates at >20% throttle
            changeGovStateTo(GS_GOVERNOR_SPOOLING_UP);
        } else if (throttle > 0 && govMaxHeadspeed == 0 && governorConfig()->gov_spoolup_time == 0) {
            // Straight throttle pass-through if no governor and no soft spool-up time is set
            changeGovStateTo(GS_PASSTHROUGH_ACTIVE);
        } else if (throttle > 0 && govMaxHeadspeed == 0) {
            // Pass-through with slow spool-up if no governor but gov spoolup time is set
            changeGovStateTo(GS_PASSTHROUGH_SPOOLING_UP);
        }
        break;

    case GS_PASSTHROUGH_SPOOLING_UP:
        // -- State complete
        // State only entered if gov_spoolup_time > 0
        if (throttle > 0) {
            // spool slowly
            if (throttle > govMainPrevious) {
                // Throttle is higher than our last output, spool some more
                govMainPrevious += govRampRate;
                govMain = govMainPrevious;
            } else if (cmp32(millis(), stateEntryTime) > (pidGetDT()/govRampRate)*throttle) {
                // We're finished spooling up if throttle <= previousOutput and we completed our effective ramp time
                changeGovStateTo(GS_PASSTHROUGH_ACTIVE);
                govMainPrevious = throttle;
                govMain = govMainPrevious;
            } else {
                // Allow throttle output to follow throttle rcCommand down
                govMainPrevious = throttle;
                govMain = govMainPrevious;
            }
        } else {
            // GO BACK TO THROTTLE OFF STATE IF WE DROP OUT OF PASSTHROUGH SPOOL-UP!
            changeGovStateTo(GS_THROTTLE_OFF);
            govMainPrevious = 0.0;
            govMain = govMainPrevious;
        }
        break;

    case GS_PASSTHROUGH_ACTIVE:
        // -- State complete
        // Passthrough throttle signal if it is non-zero
        if (throttle > 0) {
            // HF3D TODO:  Probably want to give the user option for maximum ramp of some multiple of spoolup?
            govMainPrevious = throttle;
            govMain = govMainPrevious;
        } else {
            changeGovStateTo(GS_PASSTHROUGH_LOST_THROTTLE);
            govMainPrevious = 0.0;
            govMain = govMainPrevious;
        }
        break;

    case GS_PASSTHROUGH_LOST_THROTTLE:
        // -- State complete
        if (throttle > 0.0) {
            // regained throttle within timer, move back to passthrough state or bailout if set to slow spool
            if (governorConfig()->gov_spoolup_time == 0) {
                changeGovStateTo(GS_PASSTHROUGH_ACTIVE);
                govMainPrevious = throttle;
                govMain = govMainPrevious;
            } else {
                changeGovStateTo(GS_AUTOROTATION_BAILOUT);
                govMainPrevious = govRampRate * 5.0f;
                govMain = govMainPrevious;
            }
        } else {
            // throttle still zero
            if (cmp32(millis(), stateEntryTime) > 5000) {
                // We've been waiting for throttle signal over 5 seconds, fall back to throttle off state.
                changeGovStateTo(GS_THROTTLE_OFF);
            }
            govMainPrevious = 0.0;
            govMain = govMainPrevious;
        }
        break;

    case GS_PASSTHROUGH_LOST_HEADSPEED:
        // State to be implemented
        break;

    case GS_GOVERNOR_SPOOLING_UP:
        // Check to ensure throttle remains above 0.20 during spool-up, otherwise drop back to THROTTLE_OFF
        if (throttle >= 0.20) {
            // Check for lack of headspeed signal during spool-up when motor output > 20%
            //   If headspeed signal isn't working and governor is spooling up it will just cycle between 0-20% throttle forever or until it gets a headspeed reading eventually.
            // HF3D TODO:  May want to add a timeout here instead of relying on the ramp delay?
            if (!headSpeedValid() && govMainPrevious > 0.20) {
                changeGovStateTo(GS_THROTTLE_OFF);
                govMainPrevious = 0.0;
                govMain = govMainPrevious;
                break;
            }

            // Set the user requested governor headspeed setting
            govSetpoint = throttle * govMaxHeadspeed;

            // Check if headspeed is now within 3% of govSetpoint
            // HF3D TODO:  Should we add a timer to ensure we meet this headspeed for at least 0.1 seconds or so, or just rely on the low pass filter?
            if (headSpeed > govSetpoint * 0.97) {
                // consider the heli to be spooled up
                changeGovStateTo(GS_GOVERNOR_ACTIVE);
                govI = 0.0;
                // Set the governor's base throttle % to our last spooled throttle value
                govBaseThrottle = govMainPrevious;
                govMain = govBaseThrottle;
                // Jump the rate limited Setpoint up to the setpoint.
                govSetpointLimited = govSetpoint * 0.97;
            }
            // Or if we're already over 95% throttle
            else if (govMainPrevious > 0.95) {
                // HF3D TODO:  Flag and alert user in the logs and/or with beep tones after flight that govMaxHeadspeed is set too high.
                changeGovStateTo(GS_GOVERNOR_ACTIVE);
                govI = 0.0;
                // Set the governor's base throttle % to our last spooled throttle value
                govBaseThrottle = govMainPrevious;
                govMain = govBaseThrottle;
                // Jump the current headspeed
                govSetpointLimited = headSpeed;
            }
            // Otherwise continue spooling
            else {
                // Increase throttle another ramp step
                govMainPrevious += govRampRate;
                govMain = govMainPrevious;
            }
        }
        // throttle < 0.20
        else {
            // GO BACK TO THROTTLE OFF STATE IF WE DROP OUT OF GOVERNOR THROTTLE RANGE!
            changeGovStateTo(GS_THROTTLE_OFF);
            govMainPrevious = 0.0;
            govMain = govMainPrevious;
        }
        break;

    case GS_GOVERNOR_ACTIVE:
        // -- State complete.
        // Update headspeed and check for loss of headspeed signal
        if (!headSpeedValid()) {
            changeGovStateTo(GS_GOVERNOR_LOST_HEADSPEED);
            // Be lazy and just hold the last previous throttle output until the next looptime
            govMain = govMainPrevious;
            break;
        }

        ///---  MAIN MOTOR GOVERNOR CODE ---///
        if (throttle >= 0.20) {
            // Governor only active above 20% throttle

            // Set the user requested governor headspeed setting
            govSetpoint = throttle * govMaxHeadspeed;

            // Set the governor ramp rate for headspeed setting changes
            // If ramp time is set to 5s then this will allow 20% headspeed change over 1 second
            float setPointRampRate = govRampRate * govMaxHeadspeed;

            ///--- Adapt to changes in throttle (govSetpoint) by limiting the rate of headspeed change ---///
            // If we don't have a non-zero rate limited setpoint yet, set it to the headspeed
            if (govSetpointLimited == 0) {
                govSetpointLimited = headSpeed;
            }
            // Increment or decrement the rate limited governor setpoint if needed
            if ((govSetpoint - govSetpointLimited) > setPointRampRate) {
                // Setpoint is higher than the rate limited setpoint, so increment limited setpoint higher
                govSetpointLimited = constrainf(govSetpointLimited + setPointRampRate, govSetpointLimited, govSetpoint);
            } else if ((govSetpointLimited - govSetpoint) > setPointRampRate)  {
                // Setpoint is lower than the rate limited setpoint, so decrement limited setpoint lower
                govSetpointLimited = constrainf(govSetpointLimited - setPointRampRate, govSetpoint, govSetpointLimited);
            }

            float govTailmotorAssist = 0.0f;

#ifdef USE_HF3D_ASSISTED_TAIL
            // Allow main motor govern to assist motor driven tail to yaw in the main motor torque direction.
            //   It's more important that we maintain tail authority than it is to prevent overspeeds.
            //   Constrain the main motor throttle assist to 15% for now.
            if ((motorCount > 1) && (pidData[FD_YAW].SumLim > 0.0f)) {
                // NOTE:  Make sure to update pidSumHighLimiYaw in pid.c if the 0.15f constraint is changed.
                // Calculate the desired throttle adder
                govTailmotorAssist = constrainf((float)governorConfig()->gov_tailmotor_assist_gain / 100.0f * pidData[FD_YAW].SumLim * MIXER_PID_SCALING, 0.0f, 0.15f);
                // Now we must prevent the governor from regulating down while we're assisting the tail motor unless we're more than 15% over our governor's headspeed setpoint
                if (headSpeed > govSetpointLimited && headSpeed < govSetpoint*1.15f) {
                    // Increase the rate-limited setpoint so that it tracks the headspeed higher and prevents the govPgain from offsetting our tailmotor assist
                    //   5x ramp rate was chosen because that's about how fast you need to be able to track the headspeed increase for a fast burst of throttle
                    //   Need to increase govSetpointLimited 1x faster than desired ramp rate because it will be decremented by ramp rate in the governor setpoint change code above
                    govSetpointLimited = constrainf(govSetpointLimited + 6.0f*setPointRampRate, govSetpointLimited, headSpeed);
                }
            }
#endif
            // Quick and dirty collective pitch linear feed-forward for the main motor
            // Calculate linear feedforward vs. collective stick position (always positive adder)
            //   Reasonable value would be 0.15 throttle addition for 12-degree collective throw..
            //   So gains in the 0.0015 - 0.0032 range depending on where max collective pitch is on the heli
            //   HF3D TODO:  Set this up so it works off of a calibrated pitch value for the heli taken during setup
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
            //   Would govSetpoint make more sense than govMaxHeadspeed?
            float govError = (govSetpointLimited - headSpeed) / govMaxHeadspeed;

            // if gov_p_gain = 10 (govKp = 1), we will get 1% change in throttle for 1% error in headspeed
            govP = govKp * govError;

            // if gov_i_gain = 10 (govKi = 1), we will get 1% change in throttle for 1% error in headspeed after 1 second
            govI = constrainf(govI + govKi * govError * pidGetDT(), -1.0, 1.0);

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
            govMainPrevious = govMain;

        ///---  End of Main Motor Governor Code ---///

        } else if (throttle > 0) {
            // Switch to Autorotation mode if throttle between 0-20% and governor was already active
            changeGovStateTo(GS_AUTOROTATION_CLASSIC);
            govMainPrevious = 0.01;
            govMain = govMainPrevious;
        } else {
            // throttle == 0
            changeGovStateTo(GS_GOVERNOR_LOST_THROTTLE);
            govMainPrevious = 0.0;
            govMain = govMainPrevious;
        }
        break;

    case GS_GOVERNOR_LOST_THROTTLE:
        if (throttle > 0.0) {
            // regained throttle within timer
            // Check for loss of headspeed signal
            if (!headSpeedValid()) {
                changeGovStateTo(GS_GOVERNOR_LOST_HEADSPEED);
                govMainPrevious = 0;
                govMain = govMainPrevious;
            } else if (throttle > 0.20) {
                changeGovStateTo(GS_AUTOROTATION_BAILOUT);
                govMainPrevious = govRampRate * 5.0f;
                govMain = govMainPrevious;
            } else {
                // 0 > Throttle < 0.20
                changeGovStateTo(GS_AUTOROTATION_CLASSIC);
                govMainPrevious = 0.01;
                govMain = govMainPrevious;
            }
        } else {
            // throttle still zero
            if (cmp32(millis(), stateEntryTime) > 5000) {
                // We've been waiting for throttle signal over 5 seconds, fall back to throttle off state.
                changeGovStateTo(GS_THROTTLE_OFF);
            }
            govMainPrevious = 0;
            govMain = govMainPrevious;
        }
        break;

    case GS_GOVERNOR_LOST_HEADSPEED:
        // -- State complete.
        // Check for recovery of headspeed signal
        if (headSpeedValid()) {
            changeGovStateTo(GS_AUTOROTATION_BAILOUT);
            govMain = govMainPrevious;
            break;
        }

        // Run on throttle until headspeed recovers
        if (throttle >= 0.20) {
            float tempTarget = (throttle < govBaseThrottle) ? throttle : govBaseThrottle;
            // Determine if we need to ramp
            if (govMainPrevious >= tempTarget) {
                // Already ramped, set equal to throttle or govBaseThrottle
                govMainPrevious = tempTarget;
                govMain = govMainPrevious;
            } else {
                // Not ramped up, ramp to the lesser of throttle or govBaseThrottle
                govMainPrevious = constrainf(govMainPrevious + govRampRate * 5.0f, 0.0, tempTarget);
                govMain = govMainPrevious;
            }
        } else if (throttle > 0) {
            // 0 > throttle < 0.20
            changeGovStateTo(GS_AUTOROTATION_CLASSIC);
            govMainPrevious = 0.01;
            govMain = govMainPrevious;
        } else {
            // throttle == 0
            changeGovStateTo(GS_GOVERNOR_LOST_THROTTLE);
            govMainPrevious = 0.0;
            govMain = govMainPrevious;
        }
        break;

    case GS_AUTOROTATION_CLASSIC:
        // -- State complete.
        // Only governor sends us to this state for now
        // HF3D TODO:  Should the autorotation mode have a drop-out timer that kicks us back to THROTTLE_OFF after a minute or so?  What's the longest autorotation ever?
        if (throttle >= 0.20) {
            // Go to bailout
            changeGovStateTo(GS_AUTOROTATION_BAILOUT);
            govMainPrevious = govRampRate * 5.0f;
            govMain = govMainPrevious;
        } else if (throttle > 0) {
            // Try providing a really small throttle command to prevent losing RPM signal
            govMainPrevious = 0.01;
            govMain = govMainPrevious;
        } else {
            // throttle == 0
            changeGovStateTo(GS_GOVERNOR_LOST_THROTTLE);
            govMainPrevious = 0.0;
            govMain = govMainPrevious;
        }
        break;

    case GS_AUTOROTATION_ASSIST:
        // Not currently implemented.
        //  In the future, have it help with govTailMotorAssist during autorotations.
        //  Possibly track orientation and collective to add simulated energy with the motor for micro helis.
        break;

    case GS_AUTOROTATION_BAILOUT:
        // -- State complete.
        // Bailout state gives fast ramp up to setpoint
        // We can enter bailout from passthrough or governor entry points
        if (govMaxHeadspeed > 0) {
            // Governor bailout
            //  NOTE:  Only enter governor autorotation bailout if we've successfully been gov active in the past!
            if (throttle >= 0.20) {
                // Bailout quickly to previous govBaseThrottle, then check for headspeed
                if (govBaseThrottle > govMainPrevious) {
                    govMainPrevious = constrainf(govMainPrevious + govRampRate * 5.0f, 0.0, govBaseThrottle);
                    govMain = govMainPrevious;
                } else {
                    // Check for headspeed signal once we're at govBaseThrottle
                    if (headSpeedValid()) {
                        // Have headspeed signal at govBaseOutput --> Transition to active governor on a slow ramp
                        // HF3D TODO:  Consider bringing the normal governor spoolup code in here and increasing the throttle ramp rate so that it ramps fast all the way to the headspeed target.
                        changeGovStateTo(GS_GOVERNOR_ACTIVE);
                        govI = 0.0;
                        govSetpointLimited = headSpeed;
                        govMain = govMainPrevious;
                    }
                    // Run in bailout for up to 5 seconds waiting on headspeed signal
                    else if (cmp32(millis(), stateEntryTime) > 5000) {
                        // We've been waiting for headspeed signal over 5 seconds, fall back to a more appropriate state
                        changeGovStateTo(GS_GOVERNOR_LOST_HEADSPEED);
                        govMain = govMainPrevious;
                    }
                }
            } else if (throttle > 0) {
                // 0 > Throttle < 0.20
                // Throttle dropped back into autorotation range
                changeGovStateTo(GS_AUTOROTATION_CLASSIC);
                govMainPrevious = 0.01;
                govMain = govMainPrevious;
            } else {
                // throttle == 0
                changeGovStateTo(GS_GOVERNOR_LOST_THROTTLE);
                govMainPrevious = 0.0;
                govMain = govMainPrevious;
            }

        } else {
            // Passthrough bailout -- spool quickly
            if (throttle > govMainPrevious) {
                // Throttle is higher than our last output, spool some more
                govMainPrevious += govRampRate * 5.0f;
                govMain = govMainPrevious;
            } else {
                // Transition to active state
                changeGovStateTo(GS_PASSTHROUGH_ACTIVE);
                govMainPrevious = throttle;
                govMain = govMainPrevious;
            }
        }
        break;

    default:
        // default is bad!  abort!
        govState = GS_THROTTLE_OFF;
        govMainPrevious = 0.0;
        govOutput[0] = 0.0;
        govOutput[1] = 0.0;
        return;
    }

#ifdef DEBUG_GOV_THROTTLE
    DEBUG_SET(DEBUG_GOVERNOR, 0, headSpeed);
    DEBUG_SET(DEBUG_GOVERNOR, 1, throttle * 1000);
    DEBUG_SET(DEBUG_GOVERNOR, 2, govMainPrevious * 1000);
    DEBUG_SET(DEBUG_GOVERNOR, 3, govState);
#endif
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
#ifdef DEBUG_GOV_COMPAT
    DEBUG_SET(DEBUG_GOVERNOR, 0, govSetpointLimited);
    DEBUG_SET(DEBUG_GOVERNOR, 1, headSpeed);
    DEBUG_SET(DEBUG_GOVERNOR, 2, govPidSum * 1000);
    DEBUG_SET(DEBUG_GOVERNOR, 3, getMotorRPM(1));
#endif
    // end of Main Motor handling

    // Handle the TAIL motor mixing & control
    // HF3D TODO:  Eventually need to support motor driven + variable pitch combination tails
    if (getMotorCount() > 1) {

        // motorMix for tail motor should be 100% of the inverted stabilized yaw channel
        //  Negative yaw pidSum ==> Tail motor spins
        //  Positive yaw pidSum ==> Main motor gov assist
        float pidSum = pidData[FD_YAW].SumLim * MIXER_PID_SCALING * -1.0f;

        //  For a tail motor.. we don't really want it spinning like crazy from base thrust anytime we're armed,
        //   so tone it down a bit using the main motor throttle as a gain until we're at half our throttle setting or something.
        switch (govState)
        {
            case GS_THROTTLE_OFF:
            case GS_PASSTHROUGH_LOST_THROTTLE:
            case GS_GOVERNOR_LOST_THROTTLE:
                govTail = 0.0;
                break;
            case GS_PASSTHROUGH_SPOOLING_UP:
            case GS_GOVERNOR_SPOOLING_UP:
                // Track the main motor output while spooling up so that we don't have our tail motor going nuts at zero throttle
                govTail = pidSum * govMain;
                break;
            case GS_PASSTHROUGH_ACTIVE:
            case GS_PASSTHROUGH_LOST_HEADSPEED:
            case GS_GOVERNOR_ACTIVE:
            case GS_GOVERNOR_LOST_HEADSPEED:
            case GS_AUTOROTATION_ASSIST:
            case GS_AUTOROTATION_BAILOUT:
                govTail = pidSum;
                break;
            case GS_AUTOROTATION_CLASSIC:
                // When motor output is zero we need to be very laid back on our tail motor application!
                govTail = pidSum * 0.10;
                break;
            default:
                govTail = 0.0;
                break;
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
    switch (govState)
    {
        case GS_THROTTLE_OFF:
        case GS_PASSTHROUGH_SPOOLING_UP:
        case GS_GOVERNOR_SPOOLING_UP:
            return false;
        case GS_PASSTHROUGH_ACTIVE:
        case GS_PASSTHROUGH_LOST_THROTTLE:
        case GS_PASSTHROUGH_LOST_HEADSPEED:
        case GS_GOVERNOR_ACTIVE:
        case GS_GOVERNOR_LOST_THROTTLE:
        case GS_GOVERNOR_LOST_HEADSPEED:
        case GS_AUTOROTATION_CLASSIC:
        case GS_AUTOROTATION_ASSIST:
        case GS_AUTOROTATION_BAILOUT:
            return true;
        default:
            return false;
    }

    return false;
}

float getHeadSpeed(void)
{
    return headSpeed;
}

