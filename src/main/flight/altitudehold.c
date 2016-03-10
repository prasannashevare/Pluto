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
#include <stdint.h>
#include <stdlib.h>
#include <math.h>


#include "platform.h"
#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"

#include "rx/rx.h"

#include "io/rc_controls.h"
#include "io/escservo.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "config/runtime_config.h"

int32_t setVelocity = 0;
uint8_t velocityControl = 1;
int32_t errorVelocityI = 0;
int32_t altHoldThrottleAdjustment = 0;
int32_t AltHold;
int32_t vario = 0;                      // variometer in cm/s

//extern uint16_t debug_d0;
static barometerConfig_t *barometerConfig;
static pidProfile_t *pidProfile;
static rcControlsConfig_t *rcControlsConfig;
static escAndServoConfig_t *escAndServoConfig;
barometerConfig_t *barometerConfig_tmp;

void configureAltitudeHold(
        pidProfile_t *initialPidProfile,
        barometerConfig_t *intialBarometerConfig,
        rcControlsConfig_t *initialRcControlsConfig,
        escAndServoConfig_t *initialEscAndServoConfig
)
{
    pidProfile = initialPidProfile;
    barometerConfig = intialBarometerConfig;
    rcControlsConfig = initialRcControlsConfig;
    barometerConfig_tmp=barometerConfig;
    escAndServoConfig = initialEscAndServoConfig;
}



#if defined(BARO) || defined(SONAR)

static int16_t initialThrottleHold; //DRONA
static int32_t EstAlt;                // in cm
int16_t initialThrottleHold_test; //DRONA
int16_t debug_e1; //DRONA

// 40hz update rate (20hz LPF on acc)
#define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25)

#define DEGREES_80_IN_DECIDEGREES 800
/** Drone should maintian its altitude which it attins at the throttle of '1500'*/
static void applyMultirotorAltHold(void)
{
    static uint8_t isAltHoldChanged = 0;
    static int16_t throttle_history=0; //drona
    static int16_t sensitivity_inv = 6;
    // multirotor alt hold
   if (rcControlsConfig->alt_hold_fast_change) {
        // rapid alt changes
        if (ABS(rcData[THROTTLE] - initialThrottleHold) > rcControlsConfig->alt_hold_deadband) {
            //errorVelocityI = 0; //drona pras
            isAltHoldChanged = 1;
            //rcCommand[THROTTLE] += (rcData[THROTTLE] > initialThrottleHold) ? -rcControlsConfig->alt_hold_deadband : rcControlsConfig->alt_hold_deadband; //drona
            rcCommand[THROTTLE] = throttle_history +  constrain((rcData[THROTTLE] - initialThrottleHold)/sensitivity_inv,-80,80); //drona
            if(rcData[THROTTLE]<1100)//Drona pras2
            {
                rcCommand[THROTTLE]= 1150;
            }
           //led2_op(true);//drona led
           //led1_op(false);//drona led
           //debug1 = rcCommand[THROTTLE]; //DRONA
        } else {
            if (isAltHoldChanged) {
                AltHold = EstAlt;
                isAltHoldChanged = 0;
                if (ARMING_FLAG(ARMED)){
                //altHoldThrottleAdjustment = throttle_history;//drona pras
                }
            }
            //led2_op(false);//drona led
            //led1_op(true);//drona led
            rcCommand[THROTTLE] = constrain(initialThrottleHold + altHoldThrottleAdjustment, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle);
            throttle_history = rcCommand[THROTTLE] ; //drona

        }
    } else {
        // slow alt changes, mostly used for aerial photography
        if (ABS(rcData[THROTTLE] - initialThrottleHold) > rcControlsConfig->alt_hold_deadband) {
            // set velocity proportional to stick movement +100 throttle gives ~ +50 cm/s
            setVelocity = (rcData[THROTTLE] - initialThrottleHold) / 6;
            setVelocity = constrain(setVelocity,-120,120);
           /* if (isAltHoldChanged)
               velocityControl = 0;
            else
               velocityControl = 1;*/
            velocityControl = 1;
            isAltHoldChanged=1;
        } else if(isAltHoldChanged){
            setVelocity = 0;
            velocityControl = 1;
            AltHold=EstAlt;
            isAltHoldChanged=0;
         }


         /*if((setVelocity>50))
            {led2_op(true);}
         else
            {led2_op(false);}*/

        rcCommand[THROTTLE] = constrain(initialThrottleHold + altHoldThrottleAdjustment, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle);
        if(rcData[THROTTLE]<1150)
            {
            rcCommand[THROTTLE]=rcData[THROTTLE];
            }
    }
}

static void applyFixedWingAltHold(airplaneConfig_t *airplaneConfig)
{
    // handle fixedwing-related althold. UNTESTED! and probably wrong
    // most likely need to check changes on pitch channel and 'reset' althold similar to
    // how throttle does it on multirotor

    rcCommand[PITCH] += altHoldThrottleAdjustment * airplaneConfig->fixedwing_althold_dir;
}

void applyAltHold(airplaneConfig_t *airplaneConfig)
{
    if (STATE(FIXED_WING)) {
        applyFixedWingAltHold(airplaneConfig);
    } else {
        applyMultirotorAltHold();
    }
}

void updateAltHoldState(void)
{
    // Baro alt hold activate
    if (!IS_RC_MODE_ACTIVE(BOXBARO)) {
        DISABLE_FLIGHT_MODE(BARO_MODE);
        return;
         }

    if (!FLIGHT_MODE(BARO_MODE)) {
        //led0_op(false);
        ENABLE_FLIGHT_MODE(BARO_MODE);
           AltHold = EstAlt;//+100;//drona pras                      //DRONA
        //initialThrottleHold = rcData[THROTTLE];//Drona pras     //
        initialThrottleHold = 1500;//Drona pras     //
        errorVelocityI = 0;                               //DRONA
        altHoldThrottleAdjustment = 0;                    //DRONA
    }
    else
    {
           //led0_op(true);//drona pras

    }
    initialThrottleHold_test=initialThrottleHold;           //DRONA
    //debug_d0 = pidProfile->D8[PIDALT];
    debug_e1 = rcCommand[THROTTLE];                      //DRONA
}

void updateSonarAltHoldState(void)
{
    // Sonar alt hold activate
    if (!IS_RC_MODE_ACTIVE(BOXSONAR)) {
        DISABLE_FLIGHT_MODE(SONAR_MODE);
        return;
    }

    if (!FLIGHT_MODE(SONAR_MODE)) {
        ENABLE_FLIGHT_MODE(SONAR_MODE);
        AltHold = EstAlt;
        initialThrottleHold = rcData[THROTTLE];
        errorVelocityI = 0;
        altHoldThrottleAdjustment = 0;
    }
}

bool isThrustFacingDownwards(rollAndPitchInclination_t *inclination)
{
    return ABS(inclination->values.rollDeciDegrees) < DEGREES_80_IN_DECIDEGREES && ABS(inclination->values.pitchDeciDegrees) < DEGREES_80_IN_DECIDEGREES;
}

/*
* This (poorly named) function merely returns whichever is higher, roll inclination or pitch inclination.
* //TODO: Fix this up. We could either actually return the angle between 'down' and the normal of the craft
* (my best interpretation of scalar 'tiltAngle') or rename the function.
*/
int16_t calculateTiltAngle(rollAndPitchInclination_t *inclination)
{
    return MAX(ABS(inclination->values.rollDeciDegrees), ABS(inclination->values.pitchDeciDegrees));
}

int32_t calculateAltHoldThrottleAdjustment(int32_t vel_tmp, float accZ_tmp, float accZ_old)
{
    int32_t result = 0;
    int32_t error;
    int32_t setVel;

    if (!isThrustFacingDownwards(&inclination)) {
        return result;
    }

    // Altitude P-Controller
    if(!ARMING_FLAG(ARMED))//Drona alt
        {AltHold= EstAlt + 100;
        //initialThrottleHold=1500;//Drona pras
        }//default alt = 100cm;

    if (!velocityControl) {
        error = constrain(AltHold - EstAlt, -500, 500);
        error = applyDeadband(error, 10); // remove small P parameter to reduce noise near zero position
        setVel = constrain((pidProfile->P8[PIDALT] * error / 128), -300, +300); // limit velocity to +/- 3 m/s
        //led2_op(false);
    } else {
        //led2_op(true);
        setVel = setVelocity;
    }

    /*if((setVelocity>50))
        {led1_op(true);}
    else
        {led1_op(false);}*/

    /*if(setVel>50)
         {
         led0_op(true);
         }
    else
         {
         led0_op(false);
         }
    */
    // Velocity PID-Controller

    // P
    error = setVel - vel_tmp;
    result = constrain((pidProfile->P8[PIDVEL] * error / 32), -300, +300);

    // I
    if(ARMING_FLAG(ARMED))/*//Drona alt*/
    {
        errorVelocityI += (pidProfile->I8[PIDVEL] * error);
    }
    else
    {
        errorVelocityI = 0;
    }/*//Drona alt*/


    errorVelocityI = constrain(errorVelocityI, -(8192 * 300), (8192 * 300));
    result += errorVelocityI / 8192;     // I in range +/-200

    // D
    result -= constrain(pidProfile->D8[PIDVEL] * (accZ_tmp + accZ_old) / 512, -150, 150);

    return result;
}
int16_t accalttemp;
float Temp;
void calculateEstimatedAltitude(uint32_t currentTime)
{
    static uint32_t previousTime;
    uint32_t dTime;
    int32_t baroVel;
    float dt;
    float vel_acc;
    int32_t vel_tmp;
    float accZ_tmp;
    int32_t sonarAlt = -1;
    static float accZ_old = 0.0f;
    static float vel = 0.0f;
    static float accAlt = 0.0f;
    static int32_t lastBaroAlt;

    static int32_t baroAlt_offset = 0;
    float sonarTransition;

#ifdef SONAR
    int16_t tiltAngle;
#endif

    dTime = currentTime - previousTime;
    if (dTime < BARO_UPDATE_FREQUENCY_40HZ)
        return;

    previousTime = currentTime;

#ifdef BARO
    if (!isBaroCalibrationComplete()) {
        performBaroCalibrationCycle();
        vel = 0;
        accAlt = 0;
    }

    BaroAlt = baroCalculateAltitude();
#else
    BaroAlt = 0;
#endif

#ifdef SONAR
    tiltAngle = calculateTiltAngle(&inclination);
    sonarAlt = sonarRead();
    sonarAlt = sonarCalculateAltitude(sonarAlt, tiltAngle);
#endif

    if (sonarAlt > 0 && sonarAlt < 200) {
        baroAlt_offset = BaroAlt - sonarAlt;
        BaroAlt = sonarAlt;
    } else {
        BaroAlt -= baroAlt_offset;
        if (sonarAlt > 0  && sonarAlt <= 300) {
            sonarTransition = (300 - sonarAlt) / 100.0f;
            BaroAlt = sonarAlt * sonarTransition + BaroAlt * (1.0f - sonarTransition);
        }
    }

    dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

    // Integrator - velocity, cm/sec
    if (accSumCount) {
        accZ_tmp = (float)accSum[2] / (float)accSumCount;
    } else {
        accZ_tmp = 0;
    }
    vel_acc = accZ_tmp * accVelScale * (float)accTimeSum;

    // Integrator - Altitude in cm
    accAlt += (vel_acc * 0.5f) * dt + vel * dt; 
    accalttemp=lrintf(100*accAlt); //Checking how acc measures height                                                                // integrate velocity to get distance (x= a/2 * t^2)
    accAlt = accAlt * barometerConfig->baro_cf_alt + (float)BaroAlt * (1.0f - barometerConfig->baro_cf_alt);    // complementary filter for altitude estimation (baro & acc)
    vel += vel_acc;

#ifdef DEBUG_ALT_HOLD
    debug[1] = accSum[2] / accSumCount; // acceleration
    debug[2] = vel;                     // velocity
    debug[3] = accAlt;                  // height
#endif

    imuResetAccelerationSum();

#ifdef BARO
    if (!isBaroCalibrationComplete()) {
        return;
    }
#endif

    if (sonarAlt > 0 && sonarAlt < 200) {
        // the sonar has the best range
        EstAlt = BaroAlt;
    } else {
        EstAlt = accAlt;
    }

    baroVel = (BaroAlt - lastBaroAlt) * 1000000.0f / dTime;
    lastBaroAlt = BaroAlt;

    baroVel = constrain(baroVel, -1500, 1500);  // constrain baro velocity +/- 1500cm/s
    baroVel = applyDeadband(baroVel, 10);       // to reduce noise near zero

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * barometerConfig->baro_cf_vel + baroVel * (1.0f - barometerConfig->baro_cf_vel);
    vel_tmp = lrintf(vel);

    // set vario
    vario = applyDeadband(vel_tmp, 5);
     if (1)//!(ABS(rcData[THROTTLE] - initialThrottleHold) > rcControlsConfig->alt_hold_deadband))
     {
        altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment(vel_tmp, accZ_tmp, accZ_old);
     }//dronadrona_1200am
    Temp=pidProfile->I8[PIDALT];
    barometerConfig->baro_cf_alt=1-Temp/1000 ;
    accZ_old = accZ_tmp;
}

int32_t altitudeHoldGetEstimatedAltitude(void)
{
    return EstAlt;
}

#endif

