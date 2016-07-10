/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */



#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/gtune.h"
#include "flight/navigation.h"
#include "flight/filter.h"


#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#define VBATT_HYSTERESIS 1 //TESTING NEW BATT FAILSAFE CONDITION

/*
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "debug.h"

#include "common/axis.h"

#include "rx/rx.h"
#include "drivers/system.h"
#include "io/beeper.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "sensors/battery.h"
#include "flight/failsafe.h"
#include "imu.h"
#include "common/maths.h"
#include "pid.h"
//#include "sensors/acceleration.h"
//#include "sensors/boardalignment.h"
//#include "drivers/accgyro.h"
//#include "drivers/sensor.h"
*/
/*
 * Usage:
 *
 * failsafeInit() and useFailsafeConfig() must be called before the other methods are used.
 *
 * failsafeInit() and useFailsafeConfig() can be called in any order.
 * failsafeInit() should only be called once.
 *
 * enable() should be called after system initialisation.
 */


extern uint16_t batteryCriticalVoltage; //test



static failsafeState_t failsafeState;

static failsafeConfig_t *failsafeConfig;

static rxConfig_t *rxConfig;

static uint16_t deadband3dThrottle;           // default throttle deadband from MIDRC

extern uint16_t vbat; //test
extern uint16_t vbatscaled;
throttleStatus_e throttleStatus;

void ErrorLed(void);
extern uint8_t Indicator;
//tern uint8_t ErrorStatus_Indicator;//

static void failsafeReset(void)
{
    failsafeState.rxDataFailurePeriod = PERIOD_RXDATA_FAILURE + failsafeConfig->failsafe_delay * MILLIS_PER_TENTH_SECOND;
    failsafeState.validRxDataReceivedAt = 0;
    failsafeState.validRxDataFailedAt = 0;
    failsafeState.throttleLowPeriod = 0;
    failsafeState.landingShouldBeFinishedAt = 0;
    failsafeState.receivingRxDataPeriod = 0;
    failsafeState.receivingRxDataPeriodPreset = 0;
    failsafeState.phase = FAILSAFE_IDLE;
    failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
}

/*
 * Should called when the failsafe config needs to be changed - e.g. a different profile has been selected.
 */
void useFailsafeConfig(failsafeConfig_t *failsafeConfigToUse)
{
    failsafeConfig = failsafeConfigToUse;
    failsafeReset();
}

void failsafeInit(rxConfig_t *intialRxConfig, uint16_t deadband3d_throttle)
{
    rxConfig = intialRxConfig;

    deadband3dThrottle = deadband3d_throttle;
    failsafeState.events = 0;
    failsafeState.monitoring = false;

    return;
}

failsafePhase_e failsafePhase()
{
    return failsafeState.phase;
}

bool failsafeIsMonitoring(void)
{
    return failsafeState.monitoring;
}

bool failsafeIsActive(void)
{
    return failsafeState.active;
}

void failsafeStartMonitoring(void)
{
    failsafeState.monitoring = true;
}

static bool failsafeShouldHaveCausedLandingByNow(void)
{
    return (millis() > failsafeState.landingShouldBeFinishedAt);
}

static void failsafeActivate(void)
{
    failsafeState.active = true;
    failsafeState.phase = FAILSAFE_LANDING;
    ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
    failsafeState.landingShouldBeFinishedAt = millis() + failsafeConfig->failsafe_off_delay * MILLIS_PER_TENTH_SECOND;

    failsafeState.events++;
}

static void failsafeApplyControlInput(void)
{
    /*for (int i = 0; i < 3; i++) {
        rcData[i] = rxConfig->midrc;
    }*/
    throttleStatus = calculateThrottleStatus(&masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);
    if(!(throttleStatus==THROTTLE_LOW)) {
        rcData[THROTTLE] = failsafeConfig->failsafe_throttle;
    }
    else{
        mwDisarm();
    }

}

    //Adding new stuff for battery (Varun)
void failsafeOnLowBattery(void)
{
    if (vbat > batteryCriticalVoltage) {
        DISABLE_ARMING_FLAG(PREVENT_ARMING);
    }
    else{
        ENABLE_ARMING_FLAG(PREVENT_ARMING);
       Indicator|=(1<<0);
    }
    if(vbatscaled<333){
        failsafeState.phase=FAILSAFE_LANDING;
        failsafeActivate();
        Indicator|=(1<<2);
        ENABLE_FLIGHT_MODE(ANGLE_MODE);
        ENABLE_ARMING_FLAG(ARMED);
    }

}


bool failsafeIsReceivingRxData(void)
{
    return (failsafeState.rxLinkState == FAILSAFE_RXLINK_UP);
}

void failsafeOnRxSuspend(uint32_t usSuspendPeriod)
{
    failsafeState.validRxDataReceivedAt += (usSuspendPeriod / 1000);    // / 1000 to convert micros to millis
}

void failsafeOnRxResume(void)
{
    failsafeState.validRxDataReceivedAt = millis();                     // prevent RX link down trigger, restart rx link up
    failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;                     // do so while rx link is up
}

void failsafeOnValidDataReceived(void)
{
    failsafeState.validRxDataReceivedAt = millis();
    if ((failsafeState.validRxDataReceivedAt - failsafeState.validRxDataFailedAt) > PERIOD_RXDATA_RECOVERY) {
        failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;
    }
}

void failsafeOnValidDataFailed(void)
{
    failsafeState.validRxDataFailedAt = millis();
    if ((failsafeState.validRxDataFailedAt - failsafeState.validRxDataReceivedAt) > failsafeState.rxDataFailurePeriod) {
        failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
    }
}

void failsafeUpdateState(void)
{
    if (!failsafeIsMonitoring()) {
        return;
    }

   bool receivingRxData = failsafeIsReceivingRxData();
    bool armed = ARMING_FLAG(ARMED);
    bool failsafeSwitchIsOn = IS_RC_MODE_ACTIVE(BOXFAILSAFE);
    beeperMode_e beeperMode = BEEPER_SILENCE;

    if (!receivingRxData) {
        beeperMode = BEEPER_RX_LOST;
    }

    bool reprocessState;
	//led1_op(true);
    do {
        reprocessState = false;
		//led0_op(true);
        switch (failsafeState.phase) {
            case FAILSAFE_IDLE:
			/*led0_op(true);
			led1_op(false);
			led2_op(false);*/
                if (armed) {
                    // Track throttle command below minimum time
                    if (THROTTLE_HIGH == calculateThrottleStatus(rxConfig, deadband3dThrottle)) {
                        failsafeState.throttleLowPeriod = millis() + failsafeConfig->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
						/*led0_op(true);
						led1_op(false);
						led2_op(false);*/
                    }
                    // Kill switch logic (must be independent of receivingRxData to skip PERIOD_RXDATA_FAILURE delay before disarming)
                    if (failsafeSwitchIsOn && failsafeConfig->failsafe_kill_switch) {
						/*led0_op(false);
						led1_op(true);
						led2_op(false);*/
                        // KillswitchEvent: failsafe switch is configured as KILL switch and is switched ON
                        failsafeActivate();
                        failsafeState.phase = FAILSAFE_LANDED;      // skip auto-landing procedure
                        failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_1_SECONDS;    // require 1 seconds of valid rxData
                        reprocessState = true;
                    } else if (!receivingRxData) {
						/*led2_op(true);
						led1_op(false);
						led0_op(false);*/
                        if (millis() > failsafeState.throttleLowPeriod) {
							//led2_op(true);
                            // JustDisarm: throttle was LOW for at least 'failsafe_throttle_low_delay' seconds
                            failsafeActivate();
                            failsafeState.phase = FAILSAFE_LANDED;      // skip auto-landing procedure
                            failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_3_SECONDS; // require 3 seconds of valid rxData//drona_failsafe 9 seconds
                        } else {
							
                            failsafeState.phase = FAILSAFE_RX_LOSS_DETECTED;
                        }
                        reprocessState = true;
                    }
                } else {
                    // When NOT armed, show rxLinkState of failsafe switch in GUI (failsafe mode)
                    if (failsafeSwitchIsOn) {
                        ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
                    } else {
                        DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
                    }
                    // Throttle low period expired (= low long enough for JustDisarm)
                    failsafeState.throttleLowPeriod = 0;
                }
                break;

            case FAILSAFE_RX_LOSS_DETECTED:
			/*led0_op(false);
			led2_op(false);
			led1_op(true);*/

                if (receivingRxData) {
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                } else {
                    // Stabilize, and set Throttle to specified level
                    failsafeActivate();
                    Indicator|=(1<<1);
                }
                reprocessState = true;
                Indicator|=(1<<1);
                break;

            case FAILSAFE_LANDING:
			/*led0_op(false);
			led1_op(false);
			led2_op(true);
                if (receivingRxData) {
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                    reprocessState = true;

                }*/{
               // Indicator|=(1<<2);
                DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
                ENABLE_ARMING_FLAG(ARMED);
                if (armed) {
                    failsafeApplyControlInput();
                    beeperMode = BEEPER_RX_LOST_LANDING;
                }
                if (failsafeShouldHaveCausedLandingByNow() || !armed || (ABS(accSmooth[0]) > 500) ||
                    (ABS(accSmooth[1]) > 500)) { //ABS value of acc in pipes here initial 2000
                    failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_30_SECONDS; // require 30 seconds of valid rxData
                    failsafeState.phase = FAILSAFE_LANDED;
                    reprocessState = true;
                }
                break;
            }

            case FAILSAFE_LANDED:
			/*led0_op(true);
			led2_op(true);
			led1_op(false);*/ Indicator|=(1<<1);
                //ENABLE_ARMING_FLAG(PREVENT_ARMING); // To prevent accidently rearming by an intermittent rx link
                mwDisarm();
                failsafeState.receivingRxDataPeriod = millis() + failsafeState.receivingRxDataPeriodPreset; // set required period of valid rxData
                failsafeState.phase = FAILSAFE_RX_LOSS_MONITORING;
                reprocessState = true;
                DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
                break;

            case FAILSAFE_RX_LOSS_MONITORING:
			/*led2_op(false);
			led0_op(true);
			led1_op(true);*/
                // Monitoring the rx link to allow rearming when it has become good for > `receivingRxDataPeriodPreset` time.
               // Indicator|=(1<<2);
                if (receivingRxData) {
                    if (millis() > failsafeState.receivingRxDataPeriod) {
                        // rx link is good now, when arming via ARM switch, it must be OFF first
                        if (!(!isUsingSticksForArming() && IS_RC_MODE_ACTIVE(BOXARM))) {
                            DISABLE_ARMING_FLAG(PREVENT_ARMING);
                            failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                            reprocessState = true;
                        }
                    }
                } else {
                    failsafeState.receivingRxDataPeriod = millis() + failsafeState.receivingRxDataPeriodPreset;
                }
                break;

            case FAILSAFE_RX_LOSS_RECOVERED:
			/*led0_op(false);
			led2_op(true);
			led1_op(true);*/
                // Entering IDLE with the requirement that throttle first must be at min_check for failsafe_throttle_low_delay period.
                // This is to prevent that JustDisarm is activated on the next iteration.
                // Because that would have the effect of shutting down failsafe handling on intermittent connections.
                failsafeState.throttleLowPeriod = millis() + failsafeConfig->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
                failsafeState.phase = FAILSAFE_IDLE;
                failsafeState.active = false;
                DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
                reprocessState = true;
                break;

            default:
                break;
        }
    } while (reprocessState);

    if (beeperMode != BEEPER_SILENCE) {
        beeper(beeperMode);
    }
}
