/*
    LittleFoot Elegance Photo Driver
    Copyright (C) 2018 Emanuele Pavoni

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

//--------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
//--------------------------------------
#include "indicom.h"
#include "indilogger.h"
#include "indidevapi.h"
#include "lx200driver.h"
#include "lx200lfep.h"
#include <indidevapi.h>
//--------------------------------------
#include "indifocuserinterface.h"
#include "indibase/indifocuser.h"
#include "indifocuser.h"
#include "inditelescope.h"
//--------------------------------------
#include <libnova/sidereal_time.h>
#include <libnova/transform.h>

//------------------------------------------------------------------------------------------

#define LFEP_MAX_CMD   9
#define LFEP_TIMEOUT   4
#define LFEP_STEP_RES  5
//#define POLLMS  1000
#define LFEP_MAX_TRIES    3
#define LFEP_MAX_DELAY    100000

#define BACKLASH_FOCUS_READOUT  99999
#define MAXTRAVEL_READOUT       99999
#define MAX_N_STEP              65534

#define currentFocusSpeed           FocusSpeedN[0].value
#define currentFocusPosition        FocusAbsPosN[0].value
#define currentFocusTemperature     TemperatureN[0].value
#define currentFocusBacklash        SetBacklashN[0].value
//#define currentDuty             SettingsN[0].value
//#define currentDelay            SettingsN[1].value
//#define currentTicks            SettingsN[2].value
#define currentFocusRelativeMovement FocusRelPosN[0].value
#define currentFocusAbsoluteMovement FocusAbsPosN[0].value
#define currentFocusSetBacklash      SetBacklashN[0].value
#define currentFocusMinPosition      MinMaxPositionN[0].value
#define currentFocusMaxPosition      MinMaxPositionN[1].value
#define currentFocusMaxTravel        MaxTravelN[0].value
//------------------------------------------------------
#define MIN_HA_FLIP -2
#define MAX_HA_FLIP 2
//------------------------------------------------------
#define FOCUS_SETTINGS_TAB            "Focus Settings"
#define FOCUS_PRESETS_TAB             "Focus Presets"
#define MERIDIAN_FLIP_TAB             "Meridian Flip"
//-----------------------------------------------------
LX200LFEP::LX200LFEP() : LX200Generic()
{
    DBG_FOCUS = INDI::Logger::getInstance().addDebugLevel("Focus Verbose", "FOCUS");
    setVersion(1, 2);
       //setLX200Capability(LX200_HAS_TRACKING_FREQ |LX200_HAS_SITES | LX200_HAS_ALIGNMENT_TYPE | LX200_HAS_PULSE_GUIDING);
    SetTelescopeCapability(TELESCOPE_CAN_PARK | TELESCOPE_CAN_CONTROL_TRACK | TELESCOPE_CAN_SYNC | TELESCOPE_CAN_GOTO | TELESCOPE_CAN_ABORT | TELESCOPE_HAS_TIME | TELESCOPE_HAS_LOCATION | TELESCOPE_HAS_PIER_SIDE, 4);
    timerID = -1;
    locationUpdated = false;
    mountInitialized = false;
}

const char *LX200LFEP::getDefaultName()
{
    return (const char *) "LFEP";
}

bool LX200LFEP::initProperties()
{
    LX200Generic::initProperties();


    // Track mode

    //  AddTrackMode("TRACK_SIDEREAL", "Sidereal", true);
    //  AddTrackMode("TRACK_SOLAR", "Solar");
    //  AddTrackMode("TRACK_LUNAR", "Lunar");
    //  AddTrackMode("TRACK_CUSTOM", "Custom");
    //  AddTrackMode("TRACK_OFF", "Stationary");

    IUFillSwitch(&TrackModeS[0], "TRACK_SIDEREAL", "Sidereal", ISS_ON);
    IUFillSwitch(&TrackModeS[1], "TRACK_SOLAR", "Solar", ISS_OFF);
    IUFillSwitch(&TrackModeS[2], "TRACK_LUNAR", "Lunar", ISS_OFF);
    IUFillSwitch(&TrackModeS[3], "TRACK_CUSTOM", "Custom", ISS_OFF);
    IUFillSwitch(&TrackModeS[4], "TRACK_OFF", "Stationary", ISS_OFF); //introduced for testing
    IUFillSwitchVector(&TrackModeSP, TrackModeS, 5, getDeviceName(), "TELESCOPE_TRACK_MODE", "Track Mode", MOTION_TAB, IP_RW, ISR_1OFMANY, 0, IPS_OK);

    // Track State
    IUFillSwitch(&TrackStateS[TRACK_ON], "TRACK_ON", "On", ISS_OFF);
    IUFillSwitch(&TrackStateS[TRACK_OFF], "TRACK_OFF", "Off", ISS_ON);
    IUFillSwitchVector(&TrackStateSP, TrackStateS, 2, getDeviceName(), "TELESCOPE_TRACK_STATE", "Tracking", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0,
                       IPS_IDLE);

    // Swap King Rate
    IUFillSwitch(&SwapKingRateS[0], "KING_RATE_OFF", "Off", ISS_ON);
    IUFillSwitch(&SwapKingRateS[1], "KING_RATE_ON", "On", ISS_OFF);
    IUFillSwitchVector(&SwapKingRateSP, SwapKingRateS, 2, getDeviceName(), "TELESCOPE_KING_RATE", "King Rate", MOTION_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Slew threshold
    IUFillNumber(&SlewAccuracyN[0], "SlewRA", "RA (arcmin)", "%10.6m", 0., 60., 1., 3.0);
    IUFillNumber(&SlewAccuracyN[1], "SlewDEC", "Dec (arcmin)", "%10.6m", 0., 60., 1., 3.0);
    IUFillNumberVector(&SlewAccuracyNP, SlewAccuracyN, NARRAY(SlewAccuracyN), getDeviceName(), "Slew Accuracy", "", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    // guide speed
    IUFillSwitch(&GuideSpeedArS[0], "0.33", "0.33x", ISS_OFF);
    IUFillSwitch(&GuideSpeedArS[1], "0.67", "0.67x", ISS_ON);
    IUFillSwitch(&GuideSpeedArS[2], "1.0", "1.0x", ISS_OFF);
    IUFillSwitchVector(&GuideSpeedArSP, GuideSpeedArS, 3, getDeviceName(), "Guide Rate AR", "", GUIDE_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);
    IUFillSwitch(&GuideSpeedDecS[0], "0.33", "0.33x", ISS_OFF);
    IUFillSwitch(&GuideSpeedDecS[1], "0.67", "0.67x", ISS_ON);
    IUFillSwitch(&GuideSpeedDecS[2], "1.0", "1.0x", ISS_OFF);
    IUFillSwitchVector(&GuideSpeedDecSP, GuideSpeedDecS, 3, getDeviceName(), "Guide Rate DE", "", GUIDE_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Meridian Flip
    IUFillSwitch(&FlipEnableS[0], "FLIPPING_OFF", "Off", ISS_OFF);
    IUFillSwitch(&FlipEnableS[1], "FLIPPING_ON", "On", ISS_ON);
    IUFillSwitchVector(&FlipEnableSP, FlipEnableS, 2, getDeviceName(), "Meridian Flip", "", MERIDIAN_FLIP_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Flip Through north south
    IUFillSwitch(&FlipThroughS[0], "FLIPPING_NORTH", "North", ISS_ON);
    IUFillSwitch(&FlipThroughS[1], "FLIPPING_SOUTH", "South", ISS_OFF);
    IUFillSwitchVector(&FlipThroughSP, FlipThroughS, 2, getDeviceName(), "Flip through", "", MERIDIAN_FLIP_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // meridian flip should stay within these limits
    IUFillNumber(&MinMaxPositionFlipN[0], "MAX_HA_FLIP", "Max HA Target", "%10.6m", MIN_HA_FLIP, MAX_HA_FLIP, 0.05, 0. );
    IUFillNumberVector(&MinMaxPositionFlipNP, MinMaxPositionFlipN, 1, getDeviceName(), "Limit Flip", "", MERIDIAN_FLIP_TAB, IP_RW, 0, IPS_IDLE);


    //----------------------------FOCUS----------------------------------------
    DefaultDevice::initProperties();   //  let the base class flesh in what it wants

    //initFocuserProperties(getDeviceName(),  MAIN_CONTROL_TAB);

    //Focus 
    IUFillNumber(&FocusAbsPosN[0],"FOCUS_ABSOLUTE_POSITION","Ticks","%4.0f",0.0,MAX_N_STEP,1000.0,0);
    IUFillNumberVector(&FocusAbsPosNP,FocusAbsPosN,1,getDeviceName(),"ABS_FOCUS_POSITION","Absolute Position",FOCUS_TAB,IP_RW,60,IPS_OK);

    IUFillNumber(&FocusRelPosN[0],"FOCUS_RELATIVE_POSITION","Ticks","%4.0f",0.0,MAX_N_STEP,1000.0,0);
    IUFillNumberVector(&FocusRelPosNP,FocusRelPosN,1,getDeviceName(),"REL_FOCUS_POSITION","Relative Position",FOCUS_TAB,IP_RW,60,IPS_OK);

    IUFillSwitch(&AbortFocusS[0],"ABORTFOCUS","Abort Focus",ISS_OFF);
    IUFillSwitchVector(&AbortFocusSP,AbortFocusS,1,getDeviceName(),"FOCUS_ABORT_MOTION","Abort Motion",FOCUS_TAB,IP_RW,ISR_ATMOST1,60,IPS_IDLE);

    IUFillSwitch(&FocusModeS[0], "LX200_HALTFOCUS", "Focus Speed OFF", ISS_OFF);
    IUFillSwitch(&FocusModeS[1], "LX200_FOCUSSLOW", "Focus Speed MIN", ISS_OFF);
    IUFillSwitch(&FocusModeS[2], "LX200_FOCUSFAST", "Focus Speed MAX", ISS_ON);
    IUFillSwitchVector(&FocusModeSP, FocusModeS, 3, getDeviceName(), "FOCUS_LX200_SPEED", "Speed", FOCUS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_OK);

    /* Focus should stay within these limits */
    IUFillNumber(&MinMaxPositionN[0], "MINPOS", "Minimum Tick", "%6.0f", 1., MAX_N_STEP, 0., 1. );
    IUFillNumber(&MinMaxPositionN[1], "MAXPOS", "Maximum Tick", "%6.0f", 1., MAX_N_STEP, 0., 50000.);
    IUFillNumberVector(&MinMaxPositionNP, MinMaxPositionN, 2, getDeviceName(), "FOCUS_MINMAXPOSITION", "Extrema", FOCUS_SETTINGS_TAB, IP_RW, 0, IPS_IDLE);

    IUFillNumber(&MaxTravelN[0], "MAXTRAVEL", "Maximum travel", "%6.0f", 1., MAX_N_STEP, 0., 65500.);
    IUFillNumberVector(&MaxTravelNP, MaxTravelN, 1, getDeviceName(), "FOCUS_MAXTRAVEL", "Max. travel", FOCUS_SETTINGS_TAB, IP_RW, 0, IPS_IDLE );
    /* Set Focus position register to this position */
    IUFillNumber(&SetRegisterPositionN[0], "SETPOS", "Position", "%6.0f", 0, MAX_N_STEP, 0., 0. );
    IUFillNumberVector(&SetRegisterPositionNP, SetRegisterPositionN, 1, getDeviceName(), "FOCUS_REGISTERPOSITION", "Sync", FOCUS_SETTINGS_TAB, IP_RW, 0, IPS_IDLE);

    /* Backlash */
    IUFillNumber(&SetBacklashN[0], "SETBACKLASH", "Backlash", "%6.0f", -255., 255., 0., 0.);
    IUFillNumberVector(&SetBacklashNP, SetBacklashN, 1, getDeviceName(), "FOCUS_BACKLASH", "Set Register", FOCUS_SETTINGS_TAB, IP_RW, 0, IPS_IDLE);

    // Presets
    IUFillNumber(&PresetN[0], "Preset 1", "", "%6.2f", 0.0, MAX_N_STEP, 1000.0, 0);
    IUFillNumber(&PresetN[1], "Preset 2", "", "%6.2f", 0.0, MAX_N_STEP, 1000.0, 0);
    IUFillNumber(&PresetN[2], "Preset 3", "", "%6.2f", 0.0, MAX_N_STEP, 1000.0, 0);
    IUFillNumber(&PresetN[3], "Preset 4", "", "%6.2f", 0.0, MAX_N_STEP, 1000.0, 0);
    IUFillNumberVector(&PresetNP, PresetN, 4, getDeviceName(), "FOCUS_PRESETS", "", FOCUS_PRESETS_TAB, IP_RW, 0, IPS_IDLE);

    //Preset GOTO
    IUFillSwitch(&PresetGotoS[0], "Preset 1", "", ISS_OFF);
    IUFillSwitch(&PresetGotoS[1], "Preset 2", "", ISS_OFF);
    IUFillSwitch(&PresetGotoS[2], "Preset 3", "", ISS_OFF);
    IUFillSwitch(&PresetGotoS[3], "Preset 4", "", ISS_OFF);
    IUFillSwitchVector(&PresetGotoSP, PresetGotoS, 4, getDeviceName(), "FOCUS_GOTO", "", FOCUS_PRESETS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);


    TrackState = SCOPE_PARKED;

    /* Relative and absolute movement */
    FocusRelPosN[0].min = -5000.;
    FocusRelPosN[0].max = 5000.;
    FocusRelPosN[0].value = 100;
    FocusRelPosN[0].step = 100;

    FocusAbsPosN[0].min = 0.;
    FocusAbsPosN[0].max = MAX_N_STEP;
    FocusAbsPosN[0].value = 0;
    FocusAbsPosN[0].step = 500;

    //simulatedTemperature=600.0;
    simulatedPosition=20000;

    //--------------------------------------------------------------------
//    initGuiderProperties(getDeviceName(), GUIDE_TAB);
//    setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);
    //--------------------------------------------------------------------
    SetParkDataType(PARK_AZ_ALT);
    PierSideSP.p = IP_WO;   //Write Only for LFEP
    return true;

}

void LX200LFEP::ISGetProperties(const char *dev)
{
    LX200Generic::ISGetProperties(dev);

    if (isConnected())
    {
        //defineSwitch(&StartUpSP);
        //defineText(&DeclinationAxisTP);
        /* Motion group */
        defineProperty(&SwapKingRateSP);
        defineProperty(&TrackModeSP);
        defineProperty(&TrackStateSP);
        defineProperty(&SlewAccuracyNP);
        defineProperty(&GuideSpeedArSP);
        defineProperty(&GuideSpeedDecSP);
        defineProperty(&FlipEnableSP);
        defineProperty(&FlipThroughSP);
        defineProperty(&FocusMotionSP);
        defineProperty(&FocusModeSP);
        //defineSwitch(&UsePulseCmdSP);
        //defineSwitch(&AbortFocusSP);
        defineProperty(&PresetNP);
        defineProperty(&PresetGotoSP);

        //defineSwitch(&SwapSP);
        //defineSwitch (&SlewSpeedSP);
        defineProperty(&MaxTravelNP);
        defineProperty(&MinMaxPositionNP);
        defineProperty(&SetRegisterPositionNP);
        defineProperty(&SetBacklashNP);
        defineProperty(&MinMaxPositionFlipNP);
        defineProperty(&FocusRelPosNP);
        defineProperty(&FocusAbsPosNP);
        //defineProperty(&FocusSpeedSP);

    }
}

bool LX200LFEP::updateProperties()
{
    INDI::Telescope::updateProperties();

    if (isConnected())
    {
        defineProperty(&SlewRateSP);
        defineProperty(&SlewAccuracyNP);
        defineProperty(&SwapKingRateSP);
        defineProperty(&TrackModeSP);
        defineProperty(&TrackStateSP);
        defineProperty(&GuideSpeedArSP);
        defineProperty(&GuideSpeedDecSP);
        defineProperty(&FlipEnableSP);
        defineProperty(&FlipThroughSP);
        defineProperty(&FocusMotionSP);
        defineProperty(&FocusModeSP);
        //defineSwitch(&AbortFocusSP);
        defineProperty(&UsePulseCmdSP);
        defineProperty(&PresetNP);
        defineProperty(&PresetGotoSP);

        //defineSwitch(&SwapSP);
        defineProperty(&MaxTravelNP);
        defineProperty(&MinMaxPositionNP);
        defineProperty(&SetRegisterPositionNP);
        defineProperty(&SetBacklashNP);
        defineProperty(&MinMaxPositionFlipNP);
        defineProperty(&FocusRelPosNP);
        defineProperty(&FocusAbsPosNP);
        //defineProperty(&FocusSpeedSP);
        defineProperty(&GuideNSNP);
        defineProperty(&GuideWENP);


        GetFocusParams();
        GetTelescopeLFEParams();

        if (InitPark())
        {
            // If loading parking data is successful, we just set the default parking values.
            SetAxis1ParkDefault(0);
            SetAxis2ParkDefault(LocationN[LOCATION_LATITUDE].value);
        }
        else
        {
            // Otherwise, we set all parking data to default in case no parking data is found.
            SetAxis1Park(0);
            SetAxis2Park(LocationN[LOCATION_LATITUDE].value);
            SetAxis1ParkDefault(0);
            SetAxis2ParkDefault(LocationN[LOCATION_LATITUDE].value);
        }
    }
    else
    {
        deleteProperty(SlewRateSP.name);
        deleteProperty(SlewAccuracyNP.name);
        deleteProperty(TrackModeSP.name);
        deleteProperty(TrackStateSP.name);
        deleteProperty(SwapKingRateSP.name);
        deleteProperty(GuideSpeedArSP.name);
        deleteProperty(GuideSpeedDecSP.name);
        deleteProperty(FlipEnableSP.name);
        deleteProperty(FlipThroughSP.name);
        deleteProperty(FocusMotionSP.name);
        deleteProperty(FocusModeSP.name);

        deleteProperty(MinMaxPositionNP.name);
        deleteProperty(MaxTravelNP.name);
        deleteProperty(SetRegisterPositionNP.name);
        deleteProperty(SetBacklashNP.name);
        deleteProperty(MinMaxPositionFlipNP.name);
        deleteProperty(FocusRelPosNP.name);
        deleteProperty(FocusAbsPosNP.name);
        //deleteProperty(FocusSpeedSP.name);
        deleteProperty(PresetNP.name);
        deleteProperty(PresetGotoSP.name);
        //deleteProperty(AbortFocusSP.name);
        //deleteProperty(SwapSP.name);
    }
    return true;
}


//void LX200LFEP::set_getDefaultName()(const char *deviceName, unsigned int debug_level)
//{
//    strncpy(lx200lfep_name, deviceName, MAXINDIDEVICE);
//    LFEP_DBG_SCOPE = debug_level;
//}

//void LX200LFEP::debugTriggered(bool enable)
//{
//    INDI_UNUSED(enable);
//    LX200Generic::debugTriggered(enable);
//    set_lx200lfep_name(getDeviceName(), DBG_SCOPE);
//}

//TODO: to implement
//bool LX200LFEP::Handshake()
//{
//    char firmeware[]="FV0000000";

//    if (isSimulation())
//    {
//        timerID = SetTimer(getCurrentPollingPeriod());
//        DEBUG(INDI::Logger::DBG_SESSION, "Simulated Focus is online. Getting focus parameters...");
//        FocusAbsPosN[0].value = simulatedPosition;
//        //updateLFEPFirmware(firmeware);
//        return true;
//    }

//    if((updateLFEPFirmware(firmeware)) < 0)
//      {
//        /* This would be the end*/
//        DEBUG(INDI::Logger::DBG_ERROR, "Unknown error while reading Focus firmware.");
//        return false;
//      }

//    return true;
//}

bool LX200LFEP::initMountLFEP()
{
    int err=0;

    DEBUG(INDI::Logger::DBG_DEBUG, "Mount is not yet initialized. Initializing it...");
    if (isSimulation () == false && (err = selectLFEPTrackingMode (PortFD, 4) < 0))
    {
        DEBUGF (INDI::Logger::DBG_ERROR, "Mount is not initialized: (%d).", err);
        return false;
    }
    mountInitialized = true;
    DEBUG(INDI::Logger::DBG_DEBUG, "Mount is initialized.");
    return true;

}

bool LX200LFEP::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    int err = 0;

    if ((strcmp(getDeviceName(), dev)) == 0)
    {

        //              // ============================================================
        //              // Satisfy LFEP initialization
        //              // ============================================================
        //              if (!strcmp(name, StartUpSP.name))
        //              {
        //                  int switch_nr;

        //                  IUUpdateSwitch(&StartUpSP, states, names, n);

        //                  // Make sure that the mount is setup according to the properties
        //                  switch_nr = IUFindOnSwitchIndex(&TrackModeSP);

        //                        if (isSimulation () == false && (err = selectLFEPTrackingMode (PortFD, switch_nr) < 0))
        //                          {
        //                            DEBUGF (INDI::Logger::DBG_ERROR, "StartUpSP: Error setting tracking mode (%d).", err);
        //                            return false;
        //                          }

        //                  TrackModeSP.s = IPS_OK;
        //                  IDSetSwitch(&TrackModeSP, NULL);
        //                  return true;
        //              }

        // =======================================
        // Tracking Mode
        // =======================================
        if (!strcmp(name, TrackModeSP.name))
        {
            IUResetSwitch(&TrackModeSP);
            IUUpdateSwitch(&TrackModeSP, states, names, n);
            trackingMode = IUFindOnSwitchIndex(&TrackModeSP);
            DEBUGF(INDI::Logger::DBG_DEBUG, "trackingMode val: (%d)", trackingMode);
            TrackState = (trackingMode == 4) ? SCOPE_IDLE : SCOPE_TRACKING ;
            if (isSimulation() == false && ( err = selectLFEPTrackingMode(PortFD, trackingMode) < 0) )
            {
                DEBUGF(INDI::Logger::DBG_ERROR, "Error setting tracking mode (%d).", err);
                return false;
            }
            TrackModeSP.s = IPS_OK;
            IDSetSwitch(&TrackModeSP, NULL);

            return true;
        }
        // =======================================
        // Swap King rate
        // =======================================
        if (!strcmp(name, SwapKingRateSP.name))
        {
            int currentSwapKingRate;

            IUResetSwitch(&SwapKingRateSP);
            IUUpdateSwitch(&SwapKingRateSP, states, names, n);
            currentSwapKingRate = IUFindOnSwitchIndex(&SwapKingRateSP);
            if ((isSimulation() == false && (err = swapLFEPKingButtons(PortFD, currentSwapKingRate)) < 0))
            {
                DEBUGF(INDI::Logger::DBG_ERROR, "Error swapping king rate buttons (%d).", err);
                return false;
            }
            SwapKingRateS[0].s = ISS_OFF;
            SwapKingRateS[1].s = ISS_OFF;
            SwapKingRateSP.s = IPS_OK;
            IDSetSwitch(&SwapKingRateSP, NULL);
            return true;
        }
        // =======================================
        // Meridian Flipping
        // =======================================
        if (!strcmp(name, FlipEnableSP.name))
        {
            IUResetSwitch(&FlipEnableSP);
            IUUpdateSwitch(&FlipEnableSP, states, names, n);
            FlipEnableSP.s = IPS_OK;
            IDSetSwitch(&FlipEnableSP, NULL);
            return true;
        }
        // =======================================
        // Flipping Through Pole N/S
        // =======================================
        if (!strcmp(name, FlipThroughSP.name))
        {
            IUResetSwitch(&FlipThroughSP);
            IUUpdateSwitch(&FlipThroughSP, states, names, n);
            FlipThroughSP.s = IPS_OK;
            IDSetSwitch(&FlipThroughSP, NULL);
            return true;
        }
        // =======================================
        // Swap Pier Side
        // =======================================
        if (!strcmp(name, PierSideSP.name))
        {
            int currentPierSideFlag;

            IUResetSwitch(&PierSideSP);
            if (IUUpdateSwitch(&PierSideSP, states, names, n) < 0)
                return false;
            currentPierSideFlag = IUFindOnSwitchIndex(&PierSideSP);
            if ((isSimulation() == false && (err = setLFEPpierSide(PortFD, currentPierSideFlag)) < 0))
            {
                DEBUGF(INDI::Logger::DBG_ERROR, "Error swapping pier side buttons (%d).", err);
                PierSideSP.s = IPS_ALERT;
                IDSetSwitch(&PierSideSP, "Could not set side of mount");
                return false;
            }
            //  PierSideS[0].s = ISS_OFF;
            //  PierSideS[1].s = ISS_OFF;
            PierSideSP.s = IPS_OK;
            IDSetSwitch(&PierSideSP, NULL);//client update
            return true;
        }
        // ========================================
        // Guide Speed
        // ========================================
        if (!strcmp(name, GuideSpeedArSP.name) || !strcmp(name, GuideSpeedDecSP.name))
        {
            if (!strcmp(name, GuideSpeedArSP.name))
                IUUpdateSwitch(&GuideSpeedArSP, states, names, n);
            else
                IUUpdateSwitch(&GuideSpeedDecSP, states, names, n);
            int guideRateAr = IUFindOnSwitchIndex(&GuideSpeedArSP);
            int guideRateDec = IUFindOnSwitchIndex(&GuideSpeedDecSP);
            if (!isSimulation() && (err = selectLFEPguideRateRaDec(PortFD, guideRateAr, guideRateDec) < 0))
            {
                DEBUGF(INDI::Logger::DBG_ERROR, "Error setting guiding to rate: (%d).", err);
                return false;
            }
            GuideSpeedArSP.s = IPS_OK;
            IDSetSwitch(&GuideSpeedArSP, nullptr);
            GuideSpeedDecSP.s = IPS_OK;
            IDSetSwitch(&GuideSpeedDecSP, nullptr);
            return true;
        }

        // =======================================
        // Swap Preset Goto Focus
        // =======================================
        if (!strcmp(PresetGotoSP.name, name))
        {
            IUUpdateSwitch(&PresetGotoSP, states, names, n);
            int index = IUFindOnSwitchIndex(&PresetGotoSP);
            if (PresetN[index].value == FocusAbsPosN[0].value)
            {
                PresetGotoSP.s = IPS_OK;
                IDSetSwitch(&PresetGotoSP, NULL);
                DEBUG(INDI::Logger::DBG_DEBUG, "Moving to Preset ok.");
                return true;
            }
            if (PresetN[index].value < FocusAbsPosN[0].min)
            {
                PresetGotoSP.s = IPS_ALERT;
                IDSetSwitch(&PresetGotoSP, NULL);
                DEBUGFDEVICE(dev, INDI::Logger::DBG_ERROR, "Requested position out of bound. Focus minimum position is %g", FocusAbsPosN[0].min);
                return false;
            }
            else if (PresetN[index].value > FocusAbsPosN[0].max)
            {
                PresetGotoSP.s = IPS_ALERT;
                IDSetSwitch(&PresetGotoSP, NULL);
                DEBUGFDEVICE(dev, INDI::Logger::DBG_ERROR, "Requested position out of bound. Focus maximum position is %g", FocusAbsPosN[0].max);
                return false;
            }
            int rc = MoveAbsFocuser(PresetN[index].value);
            if (rc >= 0)
            {
                if (rc == IPS_OK)
                    return true;
                else if (rc == IPS_BUSY)
                {
                    FocusAbsPosNP.s=IPS_BUSY;
                    return true;
                }
            }
            PresetGotoSP.s = IPS_ALERT;
            IDSetSwitch(&PresetGotoSP, NULL);
            FocusAbsPosNP.s = IPS_ALERT;
            IDSetNumber(&FocusAbsPosNP, "Focuser failed to move to new requested position.");
            return false;
        }
        if (strstr(name, "FOCUS_"))
            return processFocuserSwitch(name, states, names, n);
    }

    return LX200Generic::ISNewSwitch(dev, name, states, names, n);
}

bool LX200LFEP::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (strcmp(dev, getDeviceName()) == 0)
    {
        //Slew Accuracy
        if (!strcmp(name, SlewAccuracyNP.name))
        {
            if (IUUpdateNumber(&SlewAccuracyNP, values, names, n) < 0)
                return false;
            SlewAccuracyNP.s = IPS_OK;
            if (SlewAccuracyN[0].value < 3 || SlewAccuracyN[1].value < 3)
                IDSetNumber(&SlewAccuracyNP, "Warning: Setting the slew accuracy too low may result in a dead lock");
            IDSetNumber(&SlewAccuracyNP, NULL);
            return true;
        }
        //Meridia Flip
        if (!strcmp(name, MinMaxPositionFlipNP.name))
        {
            if (IUUpdateNumber(&MinMaxPositionFlipNP, values, names, n) < 0)
                return false;
            MinMaxPositionFlipNP.s = IPS_OK;
            if (MinMaxPositionFlipN[0].value < MIN_HA_FLIP)
                IDSetNumber(&MinMaxPositionFlipNP, "Warning: Setting the MinMaxPositionFlip too low");
            if (MinMaxPositionFlipN[0].value > MAX_HA_FLIP)
                IDSetNumber(&MinMaxPositionFlipNP, "Warning: Setting the MinMaxPositionFlip too high");
            IDSetNumber(&MinMaxPositionFlipNP, NULL);
            return true;
        }
        if (strstr(name, "FOCUS_"))
            return processFocuserNumber(dev, name, values, names, n);
    }

    return LX200Generic::ISNewNumber(dev, name, values, names, n);
}

bool LX200LFEP::isSlewComplete()
{
    const double dx = targetRA - currentRA;
    const double dy = targetDEC - currentDEC;
    return fabs(dx) <= (SlewAccuracyN[0].value / (900.0)) && fabs(dy) <= (SlewAccuracyN[1].value / 60.0);
}

bool LX200LFEP::checkConnection()
{
    return true;
}

bool LX200LFEP::saveConfigItems(FILE *fp)
{
    INDI::Telescope::saveConfigItems(fp);
    IUSaveConfigNumber(fp, &SlewAccuracyNP);
    IUSaveConfigNumber(fp, &SettingsNP);
    IUSaveConfigNumber(fp, &SetBacklashNP);
    IUSaveConfigSwitch(fp, &GuideSpeedArSP);
    IUSaveConfigSwitch(fp, &GuideSpeedDecSP);
    IUSaveConfigNumber(fp, &PresetNP);
    IUSaveConfigNumber(fp, &MinMaxPositionFlipNP);
    IUSaveConfigSwitch(fp, &FlipEnableSP);
    IUSaveConfigSwitch(fp, &FlipThroughSP);
    //IUSaveConfigNumber(fp, &FocusAbsPosNP);
    IUSaveConfigSwitch(fp, &FocusModeSP);
    IUSaveConfigNumber(fp, &MinMaxPositionNP);
    //IUSaveConfigSwitch(fp, &FocusSpeedSP);
    //IUSaveConfigNumber(fp, &MaxTravelNP);

    return true;
}

bool LX200LFEP::updateLocation(double latitude, double longitude, double elevation)
{
    char la[32], lo[32];

    INDI_UNUSED(elevation);
    if (!isSimulation())
    {
        if(setLFEPSiteLongitude(PortFD, longitude) < 0)
        {
            DEBUG(INDI::Logger::DBG_ERROR, "Error setting site longitude coordinates");
            return false;
        }
        if (setLFEPSiteLatitude(PortFD, latitude) < 0)
        {
            DEBUG(INDI::Logger::DBG_ERROR, "Error setting site latitude coordinates");
            return false;
        }

        fs_sexa (la, latitude, 3, 3600);
        fs_sexa (lo, longitude, 4, 3600);

        lnobserver.lat = latitude;
        lnobserver.lng = longitude;
        if (lnobserver.lng > 180)
            lnobserver.lng -= 360;

        DEBUGF(INDI::Logger::DBG_DEBUG, "Site location updated to Lat %s - Long %s", la, lo);
        DEBUG(INDI::Logger::DBG_SESSION, "Location updated.");
    }
    locationUpdated=true;
    return true;
}

bool LX200LFEP::updateTime(ln_date *utc, double utc_offset)
{
    if (!isSimulation())
    {
        struct ln_zonedate ltm;
        ln_date_to_zonedate(utc,&ltm,utc_offset*3600.0);
        JD = ln_get_julian_day(utc);
        DEBUGF(INDI::Logger::DBG_DEBUG, "New JD is %f",static_cast<float>(JD));

        // Set time zone
        if (setUTCOffset(utc_offset) == false)//  if (setUTCOffset(PortFD, fabs(utc_offset)) < 0)
        {
            DEBUG(INDI::Logger::DBG_ERROR, "Error setting time zone");
            return false;
        }
        DEBUGF(INDI::Logger::DBG_DEBUG, "Set UTC Offset %g is successful.", fabs(utc_offset));
        // Set Local Time
        if (setLocalTime(PortFD, ltm.hours, ltm.minutes, (int) ltm.seconds) < 0)
        {
            DEBUG(INDI::Logger::DBG_ERROR, "Error setting local time.");
            return false;
        }
        DEBUGF(INDI::Logger::DBG_DEBUG, "Set Local Time %02d:%02d:%02d is successful.", ltm.hours, ltm.minutes, (int) ltm.seconds);
        // Set Date
        if (setCalenderDateLFEP(PortFD, ltm.days, ltm.months, ltm.years) < 0)
        {
            DEBUG(INDI::Logger::DBG_ERROR, "Error setting local date.");
            return false;
        }
        DEBUGF(INDI::Logger::DBG_DEBUG, "Set Local Date %02d/%02d/%02d is successful.", ltm.days, ltm.months, ltm.years);
        DEBUG(INDI::Logger::DBG_SESSION, "Time updated.");
    }
    if (locationUpdated && mountInitialized == false)
        initMountLFEP();
    return true;
}

bool LX200LFEP::Park()
{
    double parkAZ = GetAxis1Park();
    double parkAlt = GetAxis2Park();

    char AzStr[16], AltStr[16];
    fs_sexa(AzStr, parkAZ, 2, 3600);
    fs_sexa(AltStr, parkAlt, 2, 3600);
    DEBUGF(INDI::Logger::DBG_DEBUG, "Parking to Az (%s) Alt (%s)...", AzStr, AltStr);
    ln_hrz_posn horizontalPos;
    // Libnova south = 0, west = 90, north = 180, east = 270
    horizontalPos.az = parkAZ + 180;
    if (horizontalPos.az >= 360)
        horizontalPos.az -= 360;
    horizontalPos.alt = parkAlt;
    ln_lnlat_posn observer;
    observer.lat = LocationN[LOCATION_LATITUDE].value;
    observer.lng = LocationN[LOCATION_LONGITUDE].value;
    if (observer.lng > 180)
        observer.lng -= 360;
    ln_equ_posn equatorialPos;
    ln_get_equ_from_hrz(&horizontalPos, &observer, ln_get_julian_from_sys(), &equatorialPos);
    char RAStr[16], DEStr[16];
    fs_sexa(RAStr, equatorialPos.ra / 15.0, 2, 3600);
    fs_sexa(DEStr, equatorialPos.dec, 2, 3600);
    DEBUGF(INDI::Logger::DBG_DEBUG, "Parking to RA (%s) DEC (%s)...", RAStr, DEStr);
    if (Goto(equatorialPos.ra / 15.0, equatorialPos.dec))
    {
        DEBUG(INDI::Logger::DBG_SESSION, "Parking is in progress...");
        TrackState = SCOPE_PARKING;
        return true;
    }
    else
        return false;
}

bool LX200LFEP::UnPark()
{
    double parkAZ = GetAxis1Park();
    double parkAlt = GetAxis2Park();

    char AzStr[16], AltStr[16];
    fs_sexa(AzStr, parkAZ, 2, 3600);
    fs_sexa(AltStr, parkAlt, 2, 3600);
    DEBUGF(INDI::Logger::DBG_DEBUG, "Unparking from Az (%s) Alt (%s)...", AzStr, AltStr);
    ln_hrz_posn horizontalPos;
    // Libnova south = 0, west = 90, north = 180, east = 270
    horizontalPos.az = parkAZ + 180;
    if (horizontalPos.az >= 360)
        horizontalPos.az -= 360;
    horizontalPos.alt = parkAlt;
    ln_lnlat_posn observer;
    observer.lat = LocationN[LOCATION_LATITUDE].value;
    observer.lng = LocationN[LOCATION_LONGITUDE].value;
    if (observer.lng > 180)
        observer.lng -= 360;
    ln_equ_posn equatorialPos;
    ln_get_equ_from_hrz(&horizontalPos, &observer, ln_get_julian_from_sys(), &equatorialPos);
    char RAStr[16], DEStr[16];
    fs_sexa(RAStr, equatorialPos.ra / 15.0, 2, 3600);
    fs_sexa(DEStr, equatorialPos.dec, 2, 3600);
    DEBUGF(INDI::Logger::DBG_DEBUG, "Syncing to parked coordinates RA (%s) DEC (%s)...", RAStr, DEStr);

    if (Sync(equatorialPos.ra / 15.0, equatorialPos.dec))
    {
        //NOTE: if after UnPark you want to enable tracking, then below is to be decommentad
        //SetTrackEnabled(true);
        SetParked(false);
        //TrackState = SCOPE_TRACKING;
        //  IUResetSwitch(&TrackModeSP);
        //  trackingMode = 0; //Setting tracking mode to sidereal;
        //        if (isSimulation() == false && (selectLFEPTrackingMode(PortFD, trackingMode) < 0) )
        //          {
        //            DEBUG(INDI::Logger::DBG_ERROR, "Error setting tracking mode sidereal.");
        //            return false;
        //          }
        //        TrackModeS[trackingMode].s = ISS_ON;
        //        TrackModeSP.s = IPS_OK;
        //        IDSetSwitch(&TrackModeSP, NULL);

        return true;
    }
    else
        return false;
}

bool LX200LFEP::SetCurrentPark()
{
    ln_hrz_posn horizontalPos;
    // Libnova south = 0, west = 90, north = 180, east = 270

    ln_lnlat_posn observer;
    observer.lat = LocationN[LOCATION_LATITUDE].value;
    observer.lng = LocationN[LOCATION_LONGITUDE].value;
    if (observer.lng > 180)
        observer.lng -= 360;
    ln_equ_posn equatorialPos;
    equatorialPos.ra = currentRA * 15;
    equatorialPos.dec = currentDEC;
    ln_get_hrz_from_equ(&equatorialPos, &observer, ln_get_julian_from_sys(), &horizontalPos);
    double parkAZ = horizontalPos.az - 180;
    if (parkAZ < 0)
        parkAZ += 360;
    double parkAlt = horizontalPos.alt;
    char AzStr[16], AltStr[16];
    fs_sexa(AzStr, parkAZ, 2, 3600);
    fs_sexa(AltStr, parkAlt, 2, 3600);
    DEBUGF(INDI::Logger::DBG_DEBUG, "Setting current parking position to coordinates Az (%s) Alt (%s)...", AzStr, AltStr);
    SetAxis1Park(parkAZ);
    SetAxis2Park(parkAlt);

    return true;
}

bool LX200LFEP::SetDefaultPark()
{
    // By defualt azimuth 0
    SetAxis1Park(0);
    // Altitude = latitude of observer
    SetAxis2Park(LocationN[LOCATION_LATITUDE].value);

    return true;
}

//-----Get controller's internal settings--------------
bool LX200LFEP::GetInternalSettingsLFEP()
{
    char cmd[] = "#0:VX#";
    char data[22];
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD <%s>", cmd);
    if ((error_type = tty_write_string(PortFD, cmd, &nbytes_write)) != TTY_OK)
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "Error write info internal settings status (%d).", error_type);
        return false;
    }
    if ((error_type = tty_read(PortFD, data, 22, LFEP_TIMEOUT, &nbytes_read))!= TTY_OK)
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "TTY error detected: %d", error_type);
        return false;
    }
    if (nbytes_read < 0)
    {
        return false;
    }
    DEBUGF(INDI::Logger::DBG_DEBUG, "internal settings : %s", data);

    // read backlash setting
    //TODO: to implement

    // read sync state
    getLFEPinfo.SyncArDec = (bool)((data[6] & 0x05) == 0x05);

    // read slewing state
    if ((data[6] & 0x05) != 0x05)
        getLFEPinfo.SlewArDec = !((data[6] & 0x0A) == 0x0A);
    else
        getLFEPinfo.SlewArDec = false;

    // read tracking state
    if ((data[7] & 255) != 0)
        getLFEPinfo.TrackingS = false;
    else
        getLFEPinfo.TrackingS = true;

    // store current tracking rate
    //TODO: to implement

    // read pier side
    if ((data[6] & 32) == 0)
    {
        getLFEPinfo.PierSide = false;//pierWest
    }
    else
    {
        getLFEPinfo.PierSide = true;//pierEast
    }

    // read ramp value
    //TODO: to implement

    // read Motor Direction setting
    if ((data[6] & 16) == 0)
        getLFEPinfo.MotorDirectionRA = false; //normal
    else
        getLFEPinfo.MotorDirectionRA = true;  //Reversed

    if ((data[6] & 128) == 0)
        getLFEPinfo.MotorDirectionDE = false; //Normal
    else
        getLFEPinfo.MotorDirectionDE = true;  //Reverse

    // read guiding speeds
    getLFEPinfo.GuideSpeedRA = (int)data[9];  //[1=0.33x, 2=0.67x, 3=1x]
    getLFEPinfo.GuideSpeedDE = (int)data[15];  //[1=0.33x, 2=0.67x, 3=1x]
    getLFEPinfo.GuideSpeedRange = (int)(data[21] & 0x03);

    // read speeds
    //TODO: to implement

    return true;
}

bool LX200LFEP::ReadScopeStatus()
{
    if (!isConnected())
        return false;
    if (isSimulation())
    {
        mountSim();
        return true;
    }

    //----------------------------------
    if ( getLX200RA(PortFD, &currentRA) < 0 || getLX200DEC(PortFD, &currentDEC) < 0)
    {
        EqNP.s = IPS_ALERT;
        IDSetNumber(&EqNP, "Error reading RA/DEC.");
        return false;
    }
    if (TrackState == SCOPE_SLEWING)
    {
        // Check if LFEP is done slewing
        if (isSlewComplete())
        {
            //************ MERIDIAN FLIP ***********
            if (forceMeridianFlip)
            {
                //                if(doubleMovement)
                //                {
                //                    doubleMovement=false;
                //                    MeridianFlip(false);
                //                }
                //                else
                MeridianFlip(true);
                return true;
                //DEBUG(INDI::Logger::DBG_SESSION, "Ultimate cross meridian.");
            }
            //*****************************************
            TrackState = SCOPE_TRACKING;
            // Turn on tracking. default tracking=off
            if (SetTrackEnabled(true)==false)
            {
                DEBUG(INDI::Logger::DBG_ERROR, "Error setting tracking stop.");
                return false;
            }
            IDMessage(getDeviceName(),"Slew is complete. Tracking...");
        }
    }
    else if (TrackState == SCOPE_PARKING)
    {
        if(isSlewComplete())
        {
            //************ MERIDIAN FLIP ***********
            if (forceMeridianFlip)
            {
                //                if(doubleMovement)
                //                {
                //                    doubleMovement=false;
                //                    MeridianFlip(false);
                //                }
                //                else
                MeridianFlip(true);

                return true;
            }
            //*****************************************
            // Turn off tracking.
            if ((isSimulation() == false) && (SetTrackEnabled(false)==false))
            {
                DEBUG(INDI::Logger::DBG_ERROR, "Error setting tracking stop.");
                return false;
            }
            IDMessage(getDeviceName(),"Parking slew is complete.");
            DEBUG(INDI::Logger::DBG_DEBUG, "Parking slew is complete.");
            SetParked(true);
        }
    }
    NewRaDec(currentRA, currentDEC);
    if(PierSideSensor)
        syncSideOfPier();

    return true;
}

void LX200LFEP::syncSideOfPier()
{
    //   DEBUG(INDI::Logger::DBG_DEBUG, "Query internal settings info...");
    if (!GetInternalSettingsLFEP())
        DEBUG(INDI::Logger::DBG_ERROR, "Error reading info internal settings status");
    //currentPierSide=getLFEPinfo.PierSide;
    setPierSide(toupper(getLFEPinfo.PierSide) ? INDI::Telescope::PIER_EAST : INDI::Telescope::PIER_WEST);
}



////---------------------------------------------------------------------------------
////------------------------------------ FLIP ---------------------------------------
////---------------------------------------------------------------------------------

////*********************************************************************************
/*FLIP : param name=FlipStatus "false= first initial step meridianFlip"
                               "True = second step meridianFlip" */
bool LX200LFEP::MeridianFlip(bool FlipStatus)
{
    char RAStr[64]={0}, DecStr[64]={0};
    bool parkstate = false;
    double flipDEover;
    double Meridian;
    int currentPierSideFlag;
    int err = 0;


    //    if (targetaltaz.alt < 0)  //TODO: to implement
    //    {
    //        DEBUG(INDI::Logger::DBG_SESSION, "Meridian flip aborted, target below the horizon");
    //        return true;
    //    }
    if (TrackState == SCOPE_PARKING)
        parkstate=true;

    if(!FlipStatus)
    {
        oldTargetRA = targetRA;
        oldTargetDEC = targetDEC;
        if(PierSideSensor)
            flipDEover= 89.9;   //flip Through Pole = Nord
        else
            flipDEover= 90.0;

        if(IUFindOnSwitchIndex(&FlipThroughSP))    //flip Through Pole = South
        {
            DEBUGDEVICE(getDefaultName(), INDI::Logger::DBG_WARNING, "Meridian Flip started through pole South....");
            flipDEover = -flipDEover;
        }else
            DEBUGDEVICE(getDefaultName(), INDI::Logger::DBG_WARNING, "Meridian Flip started through pole Nord....");
        /********************
        *   Point To Pole   *
        *********************/
        //  first initial step meridianFlip; GOTO east or west meridian and near +/-90 DEC

        Meridian = get_local_sidereal_time(lnobserver.lng);

        if(!IUFindOnSwitchIndex(&PierSideSP))   //West
        {
            //            if(((targetaltaz.az > 270 && targetaltaz.az < 360 && flipDEover < 0) || (targetaltaz.az > 180 && targetaltaz.az < 270 && flipDEover > 0)))
            //            {
            //                doubleMovement = true;
            //                targetRA = Meridian;
            //            }
            Meridian += 6;
        }
        else    //East
        {
            //            if(((targetaltaz.az > 0 && targetaltaz.az < 90 && flipDEover < 0) || (targetaltaz.az > 90 && targetaltaz.az < 180 && flipDEover > 0)))
            //            {
            //                doubleMovement = true;
            //                targetRA = Meridian;
            //            }
            Meridian -= 6;
        }
        Meridian=range24(Meridian);

//        if(doubleMovement)
//        {
//            fs_sexa(RAStr, targetRA, 2, 3600);
//            fs_sexa(DecStr, targetDEC, 2, 3600);
//            //---------- Slew to meridian ------------
//            if (setObjectRA(PortFD, targetRA) < 0 || (setObjectDEC(PortFD, targetDEC)) < 0)
//            {
//                EqNP.s = IPS_ALERT;
//                IDSetNumber(&EqNP, "Error setting RA/DEC.");
//                return false;
//            }
//            /* Slew reads the '0', that is not the end of the slew */
//            if ((err = Slew(PortFD)))
//            {
//                DEBUGF(INDI::Logger::DBG_ERROR, "Error Slewing to JNow RA %s - DEC %s", RAStr, DecStr);
//                slewError(err);
//                return false;
//            }
//            TrackState = SCOPE_SLEWING;
//            EqNP.s     = IPS_BUSY;
//            DEBUGF(INDI::Logger::DBG_SESSION, "Slewing to RA: %s - DEC: %s", RAStr, DecStr);

//            return true;
//        }

        targetRA=Meridian;
        targetDEC=flipDEover;
        //---------- Slew to pole ------------
        fs_sexa(RAStr, Meridian, 2, 3600);
        fs_sexa(DecStr, flipDEover, 2, 3600);
        if (setObjectRA(PortFD, targetRA) < 0 || (setObjectDEC(PortFD, targetDEC)) < 0)
        {
            EqNP.s = IPS_ALERT;
            IDSetNumber(&EqNP, "Error setting RA/DEC.");
            return false;
        }

        if ((err = Slew(PortFD))) /* Slew reads the '0', that is not the end of the slew */
        {
            DEBUGF(INDI::Logger::DBG_ERROR, "Error Slewing to JNow RA %s - DEC %s", RAStr, DecStr);
            slewError(err);
            return false;
        }
        if (!parkstate)
            TrackState = SCOPE_SLEWING;
        EqNP.s     = IPS_BUSY;
        DEBUGF(INDI::Logger::DBG_SESSION, "Slewing to RA: %s - DEC: %s", RAStr, DecStr);

        /********************
        *   Cross Meridian  *
        *********************/
        //second step meridianFlip

        /*  TODO: to be activated in next version */
        if(PierSideSensor)
        {   //For mount equipped with Pierside position sensor;
            // moving slowly over Cross meridian (+/-90 DEC) and monitor the controller's change of the orientation

            //lastPierSide = IUFindOnSwitchIndex(&PierSideSP);
            lastPierSide = currentPierSide;
            // Wait until controller changes the pier side.
            DEBUGF(INDI::Logger::DBG_SESSION, "Crossing meridian. Current side: %d ", lastPierSide);

            //SetSpeed("#:R9#");  //TODO: to implement , Set move speed to 16x for Cross meridian

            if (flipDEover > 0)
                MoveNS(DIRECTION_NORTH, MOTION_START);
            else
                MoveNS(DIRECTION_SOUTH, MOTION_START);

            while (currentPierSide == lastPierSide)
            {
                usleep(300000); //300msec
                syncSideOfPier();
            }

            DEBUGF(INDI::Logger::DBG_SESSION, "Cross meridian. Current side: %d ", currentPierSide);
            if (flipDEover > 0)
                MoveNS(DIRECTION_NORTH, MOTION_STOP);
            else
                MoveNS(DIRECTION_SOUTH, MOTION_STOP);

            //restore guideRate
            int guideRateAr = IUFindOnSwitchIndex(&GuideSpeedArSP);
            int guideRateDec = IUFindOnSwitchIndex(&GuideSpeedDecSP);
            if (!isSimulation() && (err = selectLFEPguideRateRaDec(PortFD, guideRateAr, guideRateDec) < 0))
            {
                DEBUGF(INDI::Logger::DBG_ERROR, "Error setting guiding to rate AR (%d).", err);
                return false;
            }
            GuideSpeedArSP.s = IPS_OK;
            IDSetSwitch(&GuideSpeedArSP, nullptr);
            GuideSpeedDecSP.s = IPS_OK;
            IDSetSwitch(&GuideSpeedDecSP, nullptr);
        }

        return true;    //end step 1
    }
    else
    {
        if(!PierSideSensor)
        {
            if (IUFindOnSwitchIndex(&PierSideSP)==0)    //is_west
            {
                lastPierSide=PIER_WEST;
                setPierSide(INDI::Telescope::PIER_EAST);
                currentPierSideFlag = 1;  //TODO:  remove
            }
            else
            {
                lastPierSide=PIER_EAST;
                setPierSide(INDI::Telescope::PIER_WEST);
                currentPierSideFlag = 0;
            }
            if ((isSimulation() == false && (err = setLFEPpierSide(PortFD, currentPierSideFlag)) < 0))//FIXME: modify the function, deleting the int parametern ( use INDI::Telescope:: )
            {
                DEBUGF(INDI::Logger::DBG_ERROR, "Error swapping pier side (%d).", err);
                PierSideSP.s = IPS_ALERT;
                IDSetSwitch(&PierSideSP, "Could not set side of mount");
                return false;
            }
            DEBUG(INDI::Logger::DBG_SESSION,"switched Pier Side.");
        }

        forceMeridianFlip=false; //completed meridian flip

        if ( getLX200RA(PortFD, &currentRA) < 0)
        {
            EqNP.s = IPS_ALERT;
            IDSetNumber(&EqNP, "Error reading RA.");
            return false;
        }

        if(!IUFindOnSwitchIndex(&PierSideSP))   //West
            currentRA += 12;    //6(meridian) + 6(new position)
        else
            currentRA -= 12;
        currentRA = range24(currentRA);
        Sync(currentRA, targetDEC);
        targetRA = oldTargetRA;
        targetDEC = oldTargetDEC;

        DEBUG(INDI::Logger::DBG_SESSION,"Slewing target...");

        Goto(targetRA, targetDEC);
        if (parkstate)
            TrackState = SCOPE_PARKING;

        return true;    //end step 2
    }
}

////*********************************************************************************

bool LX200LFEP::Goto(double ra, double dec)
{
    targetRA  = ra;
    targetDEC = dec;
    char RAStr[64]={0}, DecStr[64]={0};
    int fracbase = 3600;
    double targetHA, lst;

// il controllo dim formato rimosso 21/2/21
/*    switch (getLX200Format())
    {
    case LX200_LONGER_FORMAT:
        fracbase = 360000;
        break;
    case LX200_LONG_FORMAT:
    case LX200_SHORT_FORMAT:
    default:
        fracbase = 3600;
        break;
    }
*/
    fs_sexa(RAStr, targetRA, 2, fracbase);
    fs_sexa(DecStr, targetDEC, 2, fracbase);

    // If moving, let's stop it first.
    if (EqNP.s == IPS_BUSY)
    {
        if (!isSimulation() && abortSlew(PortFD) < 0)
        {
            AbortSP.s = IPS_ALERT;
            IDSetSwitch(&AbortSP, "Abort slew failed.");
            return false;
        }
        AbortSP.s = IPS_OK;
        EqNP.s    = IPS_IDLE;
        IDSetSwitch(&AbortSP, "Slew aborted.");
        IDSetNumber(&EqNP, nullptr);

        if (MovementNSSP.s == IPS_BUSY || MovementWESP.s == IPS_BUSY)
        {
            MovementNSSP.s = MovementWESP.s = IPS_IDLE;
            EqNP.s                          = IPS_IDLE;
            IUResetSwitch(&MovementNSSP);
            IUResetSwitch(&MovementWESP);
            IDSetSwitch(&MovementNSSP, nullptr);
            IDSetSwitch(&MovementWESP, nullptr);
        }
        // sleep for 100 mseconds
        usleep(100000);
    }
    if (!isSimulation())
    {
        //--------------------- Check Meridian FLIP -------------------
        if (IUFindOnSwitchIndex(&FlipEnableSP))
        {
            lst = get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value);
            targetHA = get_local_hour_angle(lst, targetRA);

            if((targetHA > MinMaxPositionFlipN[0].value && !IUFindOnSwitchIndex(&PierSideSP)) || (targetHA < MinMaxPositionFlipN[0].value && IUFindOnSwitchIndex(&PierSideSP)))
            {
                DEBUG(INDI::Logger::DBG_SESSION, "Meridian Flip started ....");
                forceMeridianFlip = true;
                MeridianFlip(false);
                EqNP.s = IPS_BUSY;
                return true;
            }
        }
        //-------------------------------------------------------------

        DEBUGF(INDI::Logger::DBG_SESSION, "Slewing to RA: %s - DEC: %s", RAStr, DecStr);
        if (setObjectRA(PortFD, targetRA) < 0 || (setObjectDEC(PortFD, targetDEC)) < 0)
        {
            EqNP.s = IPS_ALERT;
            IDSetNumber(&EqNP, "Error setting RA/DEC.");
            return false;
        }
        int err = 0;
        /* Slew reads the '0', that is not the end of the slew */
        if ((err = Slew(PortFD)))
        {
            DEBUGF(INDI::Logger::DBG_ERROR, "Error Slewing to JNow RA %s - DEC %s", RAStr, DecStr);
            slewError(err);
            return false;
        }
    }
    TrackState = SCOPE_SLEWING;
    EqNP.s     = IPS_BUSY;
    DEBUGF(INDI::Logger::DBG_SESSION, "Slewing to RA: %s - DEC: %s", RAStr, DecStr);

    return true;
}

//----------------------------------------------------------


bool LX200LFEP::SetTrackEnabled(bool enabled)
{
    TrackState = (trackingMode == 4)|(!enabled) ? SCOPE_IDLE : SCOPE_TRACKING ;
    return SetTrackMode(enabled ? IUFindOnSwitchIndex(&TrackModeSP) : 4);//if(enable==0) TRACKING_OFF
}

bool LX200LFEP::SetTrackMode(uint8_t mode)
{
    if (isSimulation())
        return true;
    bool rc = (selectLFEPTrackingMode(PortFD, mode) == 0);
    // NOTE: Only update tracking frequency if it is defined and not deleted by child classes
    if (rc &&  (genericCapability & LX200_HAS_TRACKING_FREQ))
    {
        getTrackFreq(PortFD, &TrackFreqN[0].value);
        IDSetNumber(&TrackFreqNP, nullptr);	//&TrackingFreqNP
    }
    return rc;
}

void LX200LFEP::GetTelescopeLFEParams ()
{
    if (!isSimulation())
    {
        char response[24];
        int lfep_rc ;
        int nbytes_write = 0;
        int nbytes_read  = 0;
        char LFEP_cmd_KingRate[]="#-K#";

        /*************************************
         * Get controller's internal settings
         *************************************/
        //DEBUG(INDI::Logger::DBG_DEBUG, "Query internal settings info...");

        if(!PierSideSensor)
            syncSideOfPier();
        /************************
        * king Rate
        ************************/
        DEBUG(INDI::Logger::DBG_DEBUG, "Query King Rate info...");
        char kingON[2] = "1";

        if ((lfep_rc = tty_write_string(PortFD, LFEP_cmd_KingRate, &nbytes_write)) != TTY_OK)
        {
            DEBUGF(INDI::Logger::DBG_ERROR, "Error write info king rate status (%d).", lfep_rc);
            return;
        }
        if ((lfep_rc = tty_read(PortFD, response, 1, LFEP_TIMEOUT, &nbytes_read))!= TTY_OK)
        {
            DEBUGF(INDI::Logger::DBG_ERROR, "TTY error detected: %d", lfep_rc);
            return;
        }
        DEBUGF(INDI::Logger::DBG_DEBUG, "King Rate [OFF/ON]: %s", response);
        if ( !strncmp(response, kingON, 1))
        {
            if ((lfep_rc = swapLFEPKingButtons(PortFD, atoi(kingON))) < 0)
            {
                DEBUGF(INDI::Logger::DBG_ERROR, "Error swapping king rate buttons (%d).", lfep_rc);
                return;
            }
            SwapKingRateS[0].s = ISS_OFF;
            SwapKingRateS[1].s = ISS_ON;
        }
    }
    SwapKingRateSP.s = IPS_OK;
    IDSetSwitch(&SwapKingRateSP, NULL);

    return;
}

int LX200LFEP::setCalenderDateLFEP(int fd, int dd, int mm, int yy)
{
    char read_buffer[64];
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;
    yy = yy % 100;

    snprintf(read_buffer, sizeof(read_buffer), ":SC %02d/%02d/%02d#", mm, dd, yy);
    //DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", read_buffer);
    DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD <%s>", read_buffer);
    tcflush(fd, TCIFLUSH);
    if ((error_type = tty_write_string(fd, read_buffer, &nbytes_write)) != TTY_OK)
        return error_type;
    error_type = tty_read_section(fd, read_buffer, '#', LFEP_TIMEOUT, &nbytes_read);
    tcflush(fd, TCIFLUSH);
    if (nbytes_read < 1)
    {
        DEBUG(INDI::Logger::DBG_ERROR, "Unable to parse response set CalenderDate");
        //DEBUGFDEVICE(getDefaultName(), INDI::Logger::DBG_DEBUG, "Unable to parse response set CalenderDate");
        return error_type;
    }
    read_buffer[1] = '\0';
    //DEBUGF(INDI::Logger::DBG_DEBUG, "RES <%s>", read_buffer);
    DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "RES <%s>", read_buffer);
    if (read_buffer[0] == '0')
        return -1;
    /* Sleep 10ms before flushing. This solves some issues with LX200 compatible devices. */
    usleep(10000);
    tcflush(fd, TCIFLUSH);

    return 0;
}


bool LX200LFEP::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{

    int current_move = (dir == DIRECTION_NORTH) ? LX200_NORTH : LX200_SOUTH;
    if(IUFindOnSwitchIndex(&PierSideSP)==PIER_WEST)  /* solves LFEP BUG ver 6.20 */
    {
        switch(dir)
        {
        case DIRECTION_NORTH:
            current_move = LX200_SOUTH;
            break;
        case DIRECTION_SOUTH:
            current_move = LX200_NORTH;
            break;
        }
    }
    switch (command)
    {
    case MOTION_START:
        if (!isSimulation() && MoveTo(PortFD, current_move) < 0)
        {
            DEBUG(INDI::Logger::DBG_ERROR, "Error setting N/S motion direction.");
            return false;
        }
        else
            DEBUGF(INDI::Logger::DBG_SESSION, "Moving toward %s.",
                   ((current_move == LX200_NORTH) ^ (IUFindOnSwitchIndex(&PierSideSP)==PIER_WEST)) ? "North" : "South");
        break;

    case MOTION_STOP:
        if (!isSimulation() && HaltMovement(PortFD, current_move) < 0)
        {
            DEBUG(INDI::Logger::DBG_ERROR, "Error stopping N/S motion.");
            return false;
        }
        else
            DEBUGF(INDI::Logger::DBG_SESSION, "Movement toward %s halted.",
                   ((current_move == LX200_NORTH) ^ (IUFindOnSwitchIndex(&PierSideSP)==PIER_WEST)) ? "North" : "South");
        break;
    }

    return true;
}

//FIXME: check SendPulse if it's okay.   gira correto? prende questo o quello di lx200driver?

//int LX200LFEP::SendPulseCmd(int direction, int duration_msec)
//{
//    int nbytes_write = 0;
//    char cmd[20];

//    if(IUFindOnSwitchIndex(&PierSideSP)==PIER_WEST)  /* solves LFEP BUG ver 6.20 */
//    {
//        switch(direction)
//        {
//        case LX200_NORTH:
//            direction = LX200_SOUTH;
//            break;
//        case LX200_SOUTH:
//            direction = LX200_NORTH;
//            break;
//        }
//    }
//    switch (direction)
//    {
//    case LX200_NORTH:
//        sprintf(cmd, ":Mgn%04d#", duration_msec);
//        break;
//    case LX200_SOUTH:
//        sprintf(cmd, ":Mgs%04d#", duration_msec);
//        break;
//    case LX200_EAST:
//        sprintf(cmd, ":Mge%04d#", duration_msec);
//        break;
//    case LX200_WEST:
//        sprintf(cmd, ":Mgw%04d#", duration_msec);
//        break;
//    default:
//        return 1;
//    }
//    tty_write_string(PortFD, cmd, &nbytes_write);
//    tcflush(PortFD, TCIFLUSH);

//    return 0;
//}

int LX200LFEP::SendPulseCmd(int direction, int duration_msec)
{
    int nbytes_write = 0;
    char cmd[20];

    if(IUFindOnSwitchIndex(&PierSideSP)==PIER_WEST)  /* solves LFEP BUG ver 6.20 */
    {
        switch(direction)
        {
        case LX200_NORTH:
            direction = LX200_SOUTH;
            break;
        case LX200_SOUTH:
            direction = LX200_NORTH;
            break;
        }
    }
    switch (direction)
    {
    case LX200_NORTH:
        sprintf(cmd, ":Mgn%04d#", duration_msec);
        break;
    case LX200_SOUTH:
        sprintf(cmd, ":Mgs%04d#", duration_msec);
        break;
    case LX200_EAST:
        sprintf(cmd, ":Mge%04d#", duration_msec);
        break;
    case LX200_WEST:
        sprintf(cmd, ":Mgw%04d#", duration_msec);
        break;
    default:
        return 1;
    }
    tty_write_string(PortFD, cmd, &nbytes_write);
    tcflush(PortFD, TCIFLUSH);

    return 0;
}




/****************************************************************************
************************** basic functions **********************************
*****************************************************************************/


int LX200LFEP::selectLFEPTrackingMode(int fd, int trackMode)
{
    DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "<%s>", __FUNCTION__);
    int error_type;
    int nbytes_write = 0;

    switch (trackMode)
    {
    /* Sidereal */
    case 0:
        DEBUG(INDI::Logger::DBG_SESSION, "Setting tracking mode to sidereal.");
        DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD <%s>", "#0T");
        //DEBUGDEVICE(getDefaultName(), INDI::Logger::DBG_DEBUG, "Setting tracking mode to sidereal.");
        if ((error_type = tty_write_string(fd, "#0T#", &nbytes_write)) != TTY_OK)
            return error_type;
        break;

        /* Solar */
    case 1:
        DEBUG(INDI::Logger::DBG_SESSION, "Setting tracking mode to solar.");
        DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD <%s>", "#1T");
        //DEBUGDEVICE(getDefaultName(), INDI::Logger::DBG_DEBUG, "Setting tracking mode to solar.");
        if ((error_type = tty_write_string(fd, "#1T#", &nbytes_write)) != TTY_OK)
            return error_type;
        break;

        /* Lunar */
    case 2:
        DEBUG(INDI::Logger::DBG_SESSION, "Setting tracking mode to lunar.");
        //DEBUGDEVICE(getDefaultName(), INDI::Logger::DBG_DEBUG, "Setting tracking mode to lunar.");
        DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD <%s>", "#2T");
        if ((error_type = tty_write_string(fd, "#2T#", &nbytes_write)) != TTY_OK)
            return error_type;
        break;

        /* Custom */
    case 3:
        DEBUG(INDI::Logger::DBG_SESSION, "Setting tracking mode to Custom.");
        //DEBUGDEVICE(getDefaultName(), INDI::Logger::DBG_DEBUG, "Setting tracking mode to Custom.");
        DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD <%s>", "#3T#");
        if ((error_type = tty_write_string(fd, "#3T#", &nbytes_write)) != TTY_OK)
            return error_type;
        break;

        /* Zero */
    case 4:
        DEBUG(INDI::Logger::DBG_SESSION, "Setting tracking mode to off.");
        DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD <%s>", "#-1T");
        //DEBUGDEVICE(getDefaultName(), INDI::Logger::DBG_DEBUG, "Setting tracking mode to off.");
        if ((error_type = tty_write_string(fd, "#-1T#", &nbytes_write)) != TTY_OK)
            return error_type;
        break;

    default:
        return -1;
        break;
    }
    return 0;

}

int LX200LFEP::setLFEPpierSide(int fd, int currentSwap)
{
    int error_type;
    int nbytes_write = 0;

    switch (currentSwap)
    {
    case 0:
        DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD (%s)", "#<D#"); //is WEAST
        if ((error_type = tty_write_string(fd, "#<D#", &nbytes_write)) != TTY_OK)//
            return error_type;
        DEBUG(INDI::Logger::DBG_SESSION,"set Pier Side: West.");
        break;

    case 1:
        DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD (%s)", "#>D#"); //is EAST
        if ((error_type = tty_write_string(fd, "#>D#", &nbytes_write)) != TTY_OK)//
            return error_type;
        DEBUG(INDI::Logger::DBG_SESSION,"set Pier Side: East.");
        break;

    default:
        return -1;
        break;
    }
    return 0;
}

int LX200LFEP::swapLFEPKingButtons(int fd, int currentSwap)
{
    int error_type;
    int nbytes_write = 0;

    switch (currentSwap)
    {
    case 0:
        DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD (%s)", "#<K#");
        if ((error_type = tty_write_string(fd, "#<K#", &nbytes_write)) != TTY_OK)
            return error_type;
        DEBUG(INDI::Logger::DBG_SESSION,"set King Rate: OFF.");
        break;

    case 1:
        DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD (%s)", "#>K#");
        if ((error_type = tty_write_string(fd, "#>K#", &nbytes_write)) != TTY_OK)
            return error_type;
        DEBUG(INDI::Logger::DBG_SESSION,"set King Rate: ON.");
        break;

    default:
        return -1;
        break;
    }
    return 0;
}

//Note: long. and lat. precision =0.01
int LX200LFEP::setLFEPSiteLongitude(int fd, double Long)
{
    int d, m;
    char temp_string[32];

    if (Long>180)
        Long-=360.0;
    d = (int32_t) fabs(Long);
    m = (int32_t) ((fabs(Long) - d) * 100.0);

    if (Long < 0)
        d *= -1;
    snprintf(temp_string, sizeof( temp_string ), "#:Sg%04d %02d#", d, m);
    DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD (%s)", temp_string);

    return (setStandardProcedure(fd, temp_string));
}

int LX200LFEP::setLFEPSiteLatitude(int fd, double Lat)
{
    int d, m;
    char temp_string[32];

    d = (int32_t) fabs(Lat);
    m = (int32_t) ((fabs(Lat) - d) * 100.0);
    if (Lat < 0)
        d *= -1;
    snprintf(temp_string, sizeof( temp_string ), "#:St%03d %02d#", d, m);
    DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD (%s)", temp_string);

    return (setStandardProcedure(fd, temp_string));
}

int LX200LFEP::selectLFEPguideRateRaDec(int fd, int guideRateAr, int guideRateDec)
{
    int error_type;
    int nbytes_write = 0;

    switch (guideRateAr)
    {
    /* 0.33x */
    case 0:
        //DEBUGDEVICE(getDefaultName(), INDI::Logger::DBG_DEBUG, "selectLFEPguideRate: Setting guide rate RA to 0.33x");
        DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD <%s>", "#>g#");
        if ((error_type = tty_write_string(fd, "#>g#", &nbytes_write)) != TTY_OK)
            return error_type;
        DEBUG(DBG_SCOPE,"set guideRat AR: 0.33x");
        break;
    /* 0.67x */
    case 1:
        //DEBUGDEVICE(getDefaultName(), INDI::Logger::DBG_DEBUG, "selectLFEPguideRate: Setting guide rate RA to 0.67x");
        DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD <%s>", "#-<g#");
        if ((error_type = tty_write_string(fd, "#-<g#", &nbytes_write)) != TTY_OK)
            return error_type;
        DEBUG(DBG_SCOPE,"set guideRat AR: 0.67x");
        break;
    /* 1.00x */
    case 2:
        //DEBUGDEVICE(getDefaultName(), INDI::Logger::DBG_DEBUG, "selectLFEPguideRate: Setting guide rate RA to 1.00x");
        DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD <%s>", "#<g#");
        if ((error_type = tty_write_string(fd, "#<g#", &nbytes_write)) != TTY_OK)
            return error_type;
        DEBUG(DBG_SCOPE,"set guideRat AR: 1.00x");
        break;
    default:
        return -1;
        break;
    }
    switch (guideRateDec)
    {
    /* 0.33x */
    case 0:
        //DEBUGDEVICE(getDefaultName(), INDI::Logger::DBG_DEBUG, "selectLFEPguideRate: Setting guide rate DEC to 0.33x");
        DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD <%s>", "#>G#");
        if ((error_type = tty_write_string(fd, "#>G#", &nbytes_write)) != TTY_OK)
            return error_type;
        DEBUG(DBG_SCOPE,"set guideRat DEC: 0.33x");
        break;
    /* 0.67x */
    case 1:
        //DEBUGDEVICE(getDefaultName(), INDI::Logger::DBG_DEBUG, "selectLFEPguideRate: Setting guide rate DEC to 0.67x");
        DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD <%s>", "#-<G#");
        if ((error_type = tty_write_string(fd, "#-<G#", &nbytes_write)) != TTY_OK)
            return error_type;
        DEBUG(DBG_SCOPE,"set guideRat DEC: 0.67x");
        break;
    /* 1.00x */
    case 2:
        //DEBUGDEVICE(getDefaultName(), INDI::Logger::DBG_DEBUG, "selectLFEPguideRate: Setting guide rate DEC to 1.00x");
        DEBUGFDEVICE(getDefaultName(), DBG_SCOPE, "CMD <%s>", "#<G#");
        if ((error_type = tty_write_string(fd, "#<G#", &nbytes_write)) != TTY_OK)
            return error_type;
        DEBUG(DBG_SCOPE,"set guideRat DEC: 1.00x");
        break;
    default:
        return -1;
        break;
    }
    return 0;
}



//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//---------------                     FOCUS LFEP                      -----------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

unsigned char LX200LFEP::CheckSum(char *LFEP_cmd)
{
    char substr[255] ;
    unsigned char val= 0 ;
    int i= 0 ;

    for(i=0; i < 8; i++)
        substr[i]=LFEP_cmd[i] ;
    val = CalculateSum( substr) ;
    if( val !=  (unsigned char) LFEP_cmd[8])
        DEBUGF(INDI::Logger::DBG_WARNING, "Checksum: Wrong (%s,%ld), %x != %x",  LFEP_cmd, strlen(LFEP_cmd), val, (unsigned char) LFEP_cmd[8]) ;

    return val ;
}

unsigned char LX200LFEP::CalculateSum(char *LFEP_cmd)
{
    unsigned char val= 0 ;
    int i=0 ;

    for(i=0; i < 8; i++)
        val = val + (unsigned char) LFEP_cmd[i];

    return val % 256 ;
}

int LX200LFEP::SendCommand(char *LFEP_cmd)
{
    int nbytes_written=0, err_code=0;
    char LFEP_cmd_cks[32],lfepFocus_error[MAXRBUF];

    unsigned char val= 0 ;
    val = CalculateSum( LFEP_cmd );
    for(int i=0; i < 8; i++)
        LFEP_cmd_cks[i]= LFEP_cmd[i] ;
    LFEP_cmd_cks[8]=  (unsigned char) val ;
    LFEP_cmd_cks[9]= 0 ;
    if (isSimulation())
        return 0;
    tcflush(PortFD, TCIOFLUSH);
    DEBUGFDEVICE(getDefaultName(), DBG_FOCUS, "CMD (%#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X)", LFEP_cmd_cks[0], LFEP_cmd_cks[1], LFEP_cmd_cks[2], LFEP_cmd_cks[3], LFEP_cmd_cks[4], LFEP_cmd_cks[5], LFEP_cmd_cks[6], LFEP_cmd_cks[7], LFEP_cmd_cks[8]);
    //DEBUGF(INDI::Logger::DBG_DEBUG, "CMD (%#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X)", LFEP_cmd_cks[0], LFEP_cmd_cks[1], LFEP_cmd_cks[2], LFEP_cmd_cks[3], LFEP_cmd_cks[4], LFEP_cmd_cks[5], LFEP_cmd_cks[6], LFEP_cmd_cks[7], LFEP_cmd_cks[8]);
    if  ( (err_code = tty_write(PortFD, LFEP_cmd_cks, LFEP_MAX_CMD, &nbytes_written) != TTY_OK))
    {
        tty_error_msg(err_code, lfepFocus_error, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "TTY error detected: %s", lfepFocus_error);
        return -1;
    }

    return nbytes_written;
}

int LX200LFEP::ReadResponseFocus(char *buf)
{
    char lfepFocus_error[MAXRBUF];
    char lfepFocus_char[1];
    int bytesRead = 0;
    int err_code;
    char motion = 0;
    bool externalMotion=false;

    if (isSimulation())
        return LFEP_MAX_CMD;
    while (1)
    {
        if ( (err_code = tty_read(PortFD, lfepFocus_char, 1, LFEP_TIMEOUT, &bytesRead)) != TTY_OK)
        {
            tty_error_msg(err_code, lfepFocus_error, MAXRBUF);
            DEBUGF(INDI::Logger::DBG_ERROR, "TTY error detected: %s", lfepFocus_error);
            return -1;
        }
        switch (lfepFocus_char[0])
        {
        // Catch 'I'
        case 0x49:
            if (motion != 0x49)
            {
                motion = 0x49;
                DEBUG(INDI::Logger::DBG_SESSION, "Moving inward...");
                if (FocusAbsPosNP.s != IPS_BUSY)
                {
                    externalMotion = true;
                    FocusAbsPosNP.s = IPS_BUSY;
                    IDSetNumber(&FocusAbsPosNP, NULL);
                }
            }
            //usleep(100000);
            break;

            // catch 'O'
        case 0x4F:
            if (motion != 0x4F)
            {
                motion = 0x4F;
                DEBUG(INDI::Logger::DBG_SESSION, "Moving outward...");

                if (FocusAbsPosNP.s != IPS_BUSY)
                {
                    externalMotion = true;
                    FocusAbsPosNP.s = IPS_BUSY;
                    IDSetNumber(&FocusAbsPosNP, NULL);
                }
            }
            //usleep(100000);
            break;

            // Start of frame 'F'
        case 0x46:
            buf[0]=0x46;
            // Read rest of frame
            if ( (err_code = tty_read(PortFD, buf+1, LFEP_MAX_CMD-1, LFEP_TIMEOUT, &bytesRead)) != TTY_OK)
            {
                tty_error_msg(err_code, lfepFocus_error, MAXRBUF);
                DEBUGF(INDI::Logger::DBG_ERROR, "TTY error detected: %s", lfepFocus_error);
                return -1;
            }
            if (motion != 0)
            {
                DEBUG(INDI::Logger::DBG_SESSION, "Stopped.");
                // If we set it busy due to external motion, let's set it to OK
                if (externalMotion)
                {
                    FocusAbsPosNP.s = IPS_OK;
                    IDSetNumber(&FocusAbsPosNP, NULL);
                }
            }
            tcflush(PortFD, TCIOFLUSH);
            return (bytesRead+1);
            break;

        default:
            break;
        }
    }
    //        // Abort lfepFocus
    //        if (AbortFocusSP.s == IPS_BUSY && motion != 0)
    //        {
    //            AbortFocuser();
    //          }

    return -1;
}

int LX200LFEP::getLFEPPositionFocus(double *value)
{
    float temp ;
    char LFEP_cmd[LFEP_MAX_CMD] ;
    int lfepFocus_rc ;

    DEBUG(INDI::Logger::DBG_DEBUG, "Querying Position...");
    if (isSimulation())
    {
        *value = simulatedPosition;
        return 0;
    }
    strcpy(LFEP_cmd, "FG000000" ) ;
    DEBUGFDEVICE(getDefaultName(), DBG_FOCUS, "CMD <%s>", LFEP_cmd);
    if ((lfepFocus_rc= SendCommand(LFEP_cmd)) < 0)
        return lfepFocus_rc;
    if ((lfepFocus_rc= ReadResponseFocus(LFEP_cmd)) < 0)
        return lfepFocus_rc;
    if (sscanf(LFEP_cmd, "FD%6f", &temp) < 1)
        return -1;
    *value = (double) temp;
    DEBUGF(INDI::Logger::DBG_DEBUG, "Focus Position: %g", *value);

    return 0;

}
//TODO: to implement
//int LX200LFEP::updateLFEPTemperatureFocus(double *value)
//{
//    DEBUGF(INDI::Logger::DBG_DEBUG, "Focus Update Temperature: %g", value);
//    float temp ;
//    char LFEP_cmd[32] ;
//    int lfepFocus_rc ;

//    strcpy(LFEP_cmd, "FT000000" ) ;
//    DEBUGFDEVICE(getDefaultName(), DBG_FOCUS, "CMD <%s>", LFEP_cmd ) ;
//    if ((lfepFocus_rc= SendCommand( LFEP_cmd)) < 0)
//        return lfepFocus_rc;
//    if (isSimulation())
//        snprintf(LFEP_cmd, 32, "FT%6g", simulatedTemperature);
//    else  if ((lfepFocus_rc= ReadResponseFocus(LFEP_cmd)) < 0)
//            return lfepFocus_rc;
//    if (sscanf(LFEP_cmd, "FT%6f", &temp) < 1)
//        return -1;
//    *value = (double) temp/2.- 273.15;

//    return 0;
//}

int LX200LFEP::updateLFEPBacklashFocus(double *value)
{
    DEBUGF(INDI::Logger::DBG_DEBUG, "Focus Update Backlash: %g", value);
    float temp ;
    char LFEP_cmd[32] ;
    char vl_tmp[4] ;
    int lfepFocus_rc ;
    int sign= 0 ;

    if (isSimulation())
        return false;
    if(*value== BACKLASH_FOCUS_READOUT)
    {
        strcpy(LFEP_cmd, "FB000000" ) ;
    }
    else
    {
        LFEP_cmd[0]=  'F' ;
        LFEP_cmd[1]=  'B' ;
        if( *value > 0) {
            LFEP_cmd[2]= '3' ;
        } else {
            *value= - *value ;
            LFEP_cmd[2]= '2' ;
        }
        LFEP_cmd[3]= '0' ;
        LFEP_cmd[4]= '0' ;
        if(*value > 99) {
            sprintf( vl_tmp, "%3d", (int) *value) ;
        } else if(*value > 9) {
            sprintf( vl_tmp, "0%2d", (int) *value) ;
        } else {
            sprintf( vl_tmp, "00%1d", (int) *value) ;
        }
        LFEP_cmd[5]= vl_tmp[0] ;
        LFEP_cmd[6]= vl_tmp[1] ;
        LFEP_cmd[7]= vl_tmp[2] ;
    }
    DEBUGFDEVICE(getDefaultName(), DBG_FOCUS, "CMD <%s>", LFEP_cmd);
    if ((lfepFocus_rc= SendCommand( LFEP_cmd)) < 0)
        return lfepFocus_rc;
    if ((lfepFocus_rc= ReadResponseFocus(LFEP_cmd)) < 0)
        return lfepFocus_rc;
    if (sscanf(LFEP_cmd, "FB%1d%5f", &sign, &temp) < 1)
        return -1;
    *value = (double) temp  ;
    if(( sign== 2) && ( *value > 0))
    {
        *value = - (*value) ;
    }

    return false;
}

//NOTE: to be implemented in the future o remove?
//int LX200LFEP::updateLFEPMotorSettings(double *duty, double *delay, double *ticks)
//{

//    DEBUGF(INDI::Logger::DBG_DEBUG, "Update Motor Settings: Duty (%g), Delay (%g), Ticks(%g)", *duty, *delay, *ticks);
//    char LFEP_cmd[32] ;
//    int lfepFocus_rc ;

//    if (isSimulation())
//    {
//        *duty = 100;
//        *delay = 0;
//        *ticks = 0;
//        return 0;
//    }
//    if(( *duty== 0 ) && (*delay== 0) && (*ticks== 0) ){
//        strcpy(LFEP_cmd, "FC000000" ) ;
//    } else {
//        LFEP_cmd[0]=  'F' ;
//        LFEP_cmd[1]=  'C' ;
//        LFEP_cmd[2]= (char) *duty ;
//        LFEP_cmd[3]= (char) *delay ;
//        LFEP_cmd[4]= (char) *ticks ;
//        LFEP_cmd[5]= '0' ;
//        LFEP_cmd[6]= '0' ;
//        LFEP_cmd[7]= '0' ;
//        LFEP_cmd[8]=  0 ;
//    }
//    if ((lfepFocus_rc= SendCommand( LFEP_cmd)) < 0)
//        return lfepFocus_rc;
//    if ((lfepFocus_rc= ReadResponseFocus(LFEP_cmd)) < 0)
//            return lfepFocus_rc;
//    *duty=  (float) LFEP_cmd[2] ;
//    *delay= (float) LFEP_cmd[3] ;
//    *ticks= (float) LFEP_cmd[4] ;

//    return 0;
//}

int LX200LFEP::updateLFEPPositionFocusRelativeInward(double value)
{
    char LFEP_cmd[32] ;
    int lfepFocus_rc ;
    //float temp ;
    LFEP_cmd[0]= 0 ;

    DEBUGF(INDI::Logger::DBG_DEBUG, "Focus Update Relative Position Inward: %g", value);
    if (isSimulation())
    {
        simulatedPosition+= value;
        //value = simulatedPosition;
        return 0;
    }
    if(value > 9999) {
        sprintf( LFEP_cmd, "FI0%5d", (int) value) ;
    } else if(value > 999) {
        sprintf( LFEP_cmd, "FI00%4d", (int) value) ;
    } else if(value > 99) {
        sprintf( LFEP_cmd, "FI000%3d", (int) value) ;
    } else if(value > 9) {
        sprintf( LFEP_cmd, "FI0000%2d", (int) value) ;
    } else {
        sprintf( LFEP_cmd, "FI00000%1d", (int) value) ;
    }
    DEBUGFDEVICE(getDefaultName(), DBG_FOCUS, "CMD <%s>", LFEP_cmd);
    if ((lfepFocus_rc= SendCommand( LFEP_cmd)) < 0)
        return lfepFocus_rc;

    return 0;
}

int LX200LFEP::updateLFEPPositionFocusRelativeOutward(double value)
{

    char LFEP_cmd[32] ;
    int lfepFocus_rc ;
    //float temp ;

    DEBUGF(INDI::Logger::DBG_DEBUG, "Focus Update Relative Position Outward: %g", value);
    if (isSimulation())
    {
        simulatedPosition-= value;
        //value = simulatedPosition;
        return 0;
    }
    LFEP_cmd[0]= 0 ;
    if(value > 9999) {
        sprintf( LFEP_cmd, "FO0%5d", (int) value) ;
    } else if(value > 999) {
        sprintf( LFEP_cmd, "FO00%4d", (int) value) ;
    } else if(value > 99) {
        sprintf( LFEP_cmd, "FO000%3d", (int) value) ;
    } else if(value > 9) {
        sprintf( LFEP_cmd, "FO0000%2d", (int) value) ;
    } else {
        sprintf( LFEP_cmd, "FO00000%1d", (int) value) ;
    }
    DEBUGFDEVICE(getDefaultName(), DBG_FOCUS, "CMD <%s>", LFEP_cmd);
    if ((lfepFocus_rc= SendCommand( LFEP_cmd)) < 0)
        return lfepFocus_rc;

    return 0;
}

int LX200LFEP::updateLFEPPositionFocusAbsolute(double value)
{

    char LFEP_cmd[32] ;
    int lfepFocus_rc ;

    DEBUGF(INDI::Logger::DBG_DEBUG, "Focus Moving Absolute Position: %g", value);
    if (isSimulation())
    {
        simulatedPosition = value;
        return 0;
    }
    LFEP_cmd[0]= 0 ;
    if(value > 9999) {
        sprintf( LFEP_cmd, "FG0%5d", (int) value) ;
    } else if(value > 999) {
        sprintf( LFEP_cmd, "FG00%4d", (int) value) ;
    } else if(value > 99) {
        sprintf( LFEP_cmd, "FG000%3d", (int) value) ;
    } else if(value > 9) {
        sprintf( LFEP_cmd, "FG0000%2d", (int) value) ;
    } else {
        sprintf( LFEP_cmd, "FG00000%1d", (int) value) ;
    }
    DEBUGFDEVICE(getDefaultName(), DBG_FOCUS, "CMD <%s>", LFEP_cmd);
    if ((lfepFocus_rc= SendCommand( LFEP_cmd)) < 0)
        return lfepFocus_rc;

    return 0;
}

int LX200LFEP::updateLFEPMaxPosition(double *value)
{

    DEBUG(INDI::Logger::DBG_DEBUG, "Query max position...");

    float temp ;
    char LFEP_cmd[32] ;
    int lfepFocus_rc ;
    char waste[1] ;

    if (isSimulation())
    {
        return 0;
    }
    if(*value== MAXTRAVEL_READOUT) { //MAXTRAVEL_READOUT = 99999
        strcpy(LFEP_cmd, "FL000000" ) ;
    } else {
        if(*value > 9999) {
            sprintf( LFEP_cmd, "FL0%5d", (int) *value) ;
        } else if(*value > 999) {
            sprintf( LFEP_cmd, "FL00%4d", (int) *value) ;
        } else if(*value > 99) {
            sprintf( LFEP_cmd, "FL000%3d", (int) *value);
        } else if(*value > 9) {
            sprintf( LFEP_cmd, "FL0000%2d", (int) *value) ;
        } else {
            sprintf( LFEP_cmd, "FL00000%1d", (int) *value) ;
        }
    }
    DEBUGFDEVICE(getDefaultName(), DBG_FOCUS, "CMD <%s>", LFEP_cmd);
    if ((lfepFocus_rc= SendCommand( LFEP_cmd)) < 0)
        return lfepFocus_rc;
    if ((lfepFocus_rc= ReadResponseFocus(LFEP_cmd)) < 0)
        return lfepFocus_rc;
    if (sscanf(LFEP_cmd, "FL%1c%5f", waste, &temp) < 1)
        return -1;
    *value = (double) temp;
    DEBUGF(INDI::Logger::DBG_DEBUG, "Focus Max position: %g", *value);

    return 0;
}

int LX200LFEP::updateLFEPFocusSetPosition(double *value)
{
    char LFEP_cmd[32] ;
    int lfepFocus_rc ;

    if (isSimulation())
    {
        simulatedPosition= *value;
        return 0;
    }
    LFEP_cmd[0]= 0 ;
    if(*value > 9999) {
        sprintf( LFEP_cmd, "FS0%5d", (int) *value) ;
    } else if(*value > 999) {
        sprintf( LFEP_cmd, "FS00%4d", (int) *value) ;
    } else if(*value > 99) {
        sprintf( LFEP_cmd, "FS000%3d",(int)  *value) ;
    } else if(*value > 9) {
        sprintf( LFEP_cmd, "FS0000%2d", (int) *value) ;
    } else {
        sprintf( LFEP_cmd, "FS00000%1d",(int) *value) ;
    }
    DEBUGFDEVICE(getDefaultName(), DBG_FOCUS, "CMD <%s>", LFEP_cmd);
    if ((lfepFocus_rc= SendCommand( LFEP_cmd)) < 0)
        return lfepFocus_rc;
    DEBUGF(INDI::Logger::DBG_DEBUG, "Focus Set position: %g", *value);

    return 0;
}

void LX200LFEP::GetFocusParams ()
{
    int ret = -1 ;

    if((ret= getLFEPPositionFocus(&currentFocusPosition)) < 0)
    {
        FocusAbsPosNP.s = IPS_ALERT;
        DEBUGF(INDI::Logger::DBG_ERROR, "Unknown error while reading Focus position: %d", ret);
        IDSetNumber(&FocusAbsPosNP, NULL);
        return;
    }

    FocusAbsPosNP.s = IPS_OK;
    IDSetNumber(&FocusAbsPosNP, NULL);

    //    if(( ret= updateLFEPTemperatureFocus(&currentFocusTemperature)) < 0)
    //    {
    //        TemperatureNP.s = IPS_ALERT;
    //        DEBUG(INDI::Logger::DBG_ERROR, "Unknown error while reading lfepFocus temperature.");
    //        IDSetNumber(&TemperatureNP, NULL);
    //        return;
    //    }

    //    TemperatureNP.s = IPS_OK;
    //    IDSetNumber(&TemperatureNP, NULL);

    currentFocusBacklash= BACKLASH_FOCUS_READOUT ;
    if(( ret= updateLFEPBacklashFocus(&currentFocusBacklash)) < 0)
    {
        SetBacklashNP.s = IPS_ALERT;
        DEBUG(INDI::Logger::DBG_ERROR, "Unknown error while reading Focus backlash.");
        IDSetNumber(&SetBacklashNP, NULL);
        return;
    }
    SetBacklashNP.s = IPS_OK;
    IDSetNumber(&SetBacklashNP, NULL);

    //    currentDuty= currentDelay= currentTicks=0 ;
    //
    //    if(( ret= updateLFEPMotorSettings(&currentDuty, &currentDelay, &currentTicks )) < 0)
    //    {
    //        SettingsNP.s = IPS_ALERT;
    //        DEBUG(INDI::Logger::DBG_ERROR, "Unknown error while reading Focus motor settings.");
    //        IDSetNumber(&SettingsNP, NULL);
    //        return;
    //    }

    //    SettingsNP.s = IPS_OK;
    //    IDSetNumber(&SettingsNP, NULL);

    currentFocusMaxTravel = MAXTRAVEL_READOUT;
    if(( ret= updateLFEPMaxPosition(&currentFocusMaxTravel)) < 0)
    {
        MaxTravelNP.s = IPS_ALERT;
        DEBUG(INDI::Logger::DBG_ERROR, "Unknown error while reading Focus maximum travel");
        IDSetNumber(&MaxTravelNP, NULL);
        return;
    }
    MaxTravelNP.s = IPS_OK;
    IDSetNumber(&MaxTravelNP, NULL);

}

IPState LX200LFEP::MoveAbsFocuser(uint32_t targetTicks)
{
    int ret= -1 ;
    targetPos = targetTicks;

    if (targetTicks < FocusAbsPosN[0].min || targetTicks > FocusAbsPosN[0].max)
    {
        DEBUG(INDI::Logger::DBG_DEBUG, "Error, requested Focus position is out of range.");
        return IPS_ALERT;
    }
    if(( ret= updateLFEPPositionFocusAbsolute(targetPos)) < 0)
    {
        DEBUGF(INDI::Logger::DBG_DEBUG, "Read out of the Focus absolute movement failed %3d", ret);
        return IPS_ALERT;
    }
    RemoveTimer(timerID);
    timerID = SetTimer(250);
    return IPS_BUSY;
}

IPState LX200LFEP::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    return MoveAbsFocuser(FocusAbsPosN[0].value + (ticks * (dir == FOCUS_INWARD ? -1 : 1)));
}

void LX200LFEP::TimerHit()
{
    bool rc;

    if (isConnected() == false)
    {
        SetTimer(getCurrentPollingPeriod());
        return;
    }
    if (FocusAbsPosNP.s == IPS_OK || FocusAbsPosNP.s == IPS_IDLE)
    {
        rc=ReadScopeStatus();
        if(!rc)
        {
            DEBUG(INDI::Logger::DBG_ERROR, "Read error! ReadScopeStatus.");
            EqNP.s= lastEqState = IPS_ALERT;
            IDSetNumber(&EqNP, nullptr);
        }
    }
    rc=ReadFocusStatus();//if (FocusAbsPosNP.s == IPS_BUSY)
    if(!rc)
    {
        DEBUG(INDI::Logger::DBG_ERROR, "Read error! ReadFocusStatus.");
        EqNP.s= lastEqState = IPS_ALERT;
        IDSetNumber(&EqNP, nullptr);
    }
    SetTimer(getCurrentPollingPeriod());
}

bool LX200LFEP::ReadFocusStatus()
{
    //    double prevPos=currentFocusPosition;
    //    double newPos=0;
    //    int rc=0;

    if (FocusAbsPosNP.s == IPS_OK || FocusAbsPosNP.s == IPS_IDLE)
    {
        //        -----  Get a temperature  -----
        //                if(( rc= updateLFEPTemperatureFocus(&currentFocusTemperature)) < 0)
        //        {
        //            TemperatureNP.s = IPS_ALERT;
        //            DEBUG(INDI::Logger::DBG_ERROR, "Unknown error while reading lfepFocus temperature.");
        //            IDSetNumber(&TemperatureNP, NULL);
        //            return;
        //        }
        //        lastTemperature = TemperatureN[0].value;
        //        TemperatureNP.s = IPS_OK;
        //        IDSetNumber(&TemperatureNP, NULL);
        //        -----  Get Focus position  -----
        //                rc = getLFEPPositionFocus(&newPos);
        //        if (rc >= 0)
        //        {
        //            currentFocusPosition = newPos;
        //            if (prevPos != currentFocusPosition)
        //                IDSetNumber(&FocusAbsPosNP, NULL);
        //        }
    }
    if (FocusAbsPosNP.s == IPS_BUSY)
    {
        float newPos=0;
        int nbytes_read=0;
        char LFEP_cmd[LFEP_MAX_CMD] = {0};

        //nbytes_read= ReadUntilComplete(LFEP_cmd, LFEP_TIMEOUT) ;
        nbytes_read= ReadResponseFocus(LFEP_cmd);
        LFEP_cmd[nbytes_read - 1] = 0 ;

        if (nbytes_read != 9 || (sscanf(LFEP_cmd, "FD0%5f", &newPos) <= 0))
        {
            DEBUGF(INDI::Logger::DBG_WARNING, "Bogus Focus position: (%#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X) - Bytes read: %d", LFEP_cmd[0], LFEP_cmd[1], LFEP_cmd[2], LFEP_cmd[3],
                    LFEP_cmd[4], LFEP_cmd[5], LFEP_cmd[6], LFEP_cmd[7], LFEP_cmd[8], nbytes_read);
            timerID = SetTimer(getCurrentPollingPeriod());
            return false;
        }
        else if (nbytes_read < 0)
        {
            FocusAbsPosNP.s = IPS_ALERT;
            DEBUG(INDI::Logger::DBG_ERROR, "Read error! Reconnect and try again.");
            IDSetNumber(&FocusAbsPosNP, NULL);
            return false;
        }
        currentFocusPosition = newPos;
        if (currentFocusPosition == targetPos)
        {
            FocusAbsPosNP.s = IPS_OK;
            if (FocusRelPosNP.s == IPS_BUSY)
            {
                FocusRelPosNP.s = IPS_OK;
                IDSetNumber(&FocusRelPosNP, NULL);
            }
            PresetGotoSP.s = IPS_OK;
//            DEBUGF(INDI::Logger::DBG_SESSION, "Moving to Preset %d with position %g.", index+1, PresetN[index].value);
            IDSetSwitch(&PresetGotoSP, NULL);
            FocusAbsPosNP.s=IPS_OK;
        }
        IDSetNumber(&FocusAbsPosNP, NULL);
        if (FocusAbsPosNP.s == IPS_BUSY)
        {
            timerID = SetTimer(250);
        }
    }
    return true;

}

bool LX200LFEP::AbortFocuser()
{
    DEBUG(INDI::Logger::DBG_DEBUG, "Aborting focuser...");

    int nbytes_written;
    const char *buf = "FV0000000";
    if (tty_write(PortFD, buf, strlen(buf), &nbytes_written) == TTY_OK)
        return true;
    else
        return false;
}

bool LX200LFEP::processFocuserNumber (const char * dev, const char * name, double values[], char * names[], int n)
{
    int nset=0,i=0;

    //    if(strcmp(name,"FOCUS_TIMER")==0)
    //    {
    //        FocusDirection dir;
    //        int speed;
    //        int t;

    //        //  first we get all the numbers just sent to us
    //        IUUpdateNumber(&FocusTimerNP,values,names,n);

    //        //  Now lets find what we need for this move
    //        speed=FocusSpeedN[0].value;

    //        if(FocusMotionS[0].s==ISS_ON)
    //            dir=FOCUS_INWARD;
    //        else
    //            dir=FOCUS_OUTWARD;

    //        t=FocusTimerN[0].value;
    //        //*************************************************************************************************
    //        //        lastTimerValue = t;
    //        //TODO: da ri inserire
    //        //        FocusTimerNP.s = MoveFocuser(dir,speed,t);
    //        IDSetNumber(&FocusTimerNP,NULL);
    //        return true;
    //    }


    //    if(strcmp(name,"FOCUS_SPEED")==0)
    //    {
    //        FocusSpeedNP.s=IPS_OK;
    //        int current_speed = FocusSpeedN[0].value;
    //        IUUpdateNumber(&FocusSpeedNP,values,names,n);
    //        //TODO: da ri inserire --------------------------------------------------------
    //        //        if (SetFocuserSpeed(FocusSpeedN[0].value) == false)
    //        //        {
    //        //            FocusSpeedN[0].value = current_speed;
    //        //            FocusSpeedNP.s = IPS_ALERT;
    //        //        }

    //        //  Update client display
    //        IDSetNumber(&FocusSpeedNP,NULL);
    //        return true;
    //    }


    // =======================================
    // Focus  Settings
    // =======================================

    //          if (!strcmp (name, SettingsNP.name))
    //          {
    //              /* new speed */
    //              double new_duty = 0 ;
    //              double new_delay = 0 ;
    //              double new_ticks = 0 ;
    //              int ret = -1 ;

    //              for (nset = i = 0; i < n; i++)
    //              {
    //                  /* Find numbers with the passed names in the SettingsNP property */
    //                  INumber *eqp = IUFindNumber (&SettingsNP, names[i]);
    //                  /* If the number found is  (SettingsN[0]) then process it */
    //                  if (eqp == &SettingsN[0])
    //                  {
    //                      new_duty = (values[i]);
    //                      nset += new_duty >= 0 && new_duty <= 255;
    //                  } else if  (eqp == &SettingsN[1])
    //                  {
    //                      new_delay = (values[i]);
    //                      nset += new_delay >= 0 && new_delay <= 255;
    //                  } else if  (eqp == &SettingsN[2])
    //                  {
    //                      new_ticks = (values[i]);
    //                      nset += new_ticks >= 0 && new_ticks <= 255;
    //                  }
    //              }
    //              /* Did we process the three numbers? */
    //              if (nset == 3)
    //              {
    //                  /* Set the lfepFocus state to BUSY */
    //                  SettingsNP.s = IPS_BUSY;
    //                  IDSetNumber(&SettingsNP, NULL);
    //                  if(( ret= updateLFEPMotorSettings(&new_duty, &new_delay, &new_ticks))< 0)
    //                  {
    //                      IDSetNumber(&SettingsNP, "Changing to new settings failed");
    //                      return false;
    //                  }
    //                  currentDuty = new_duty ;
    //                  currentDelay= new_delay ;
    //                  currentTicks= new_ticks ;
    //                  SettingsNP.s = IPS_OK;
    //                  IDSetNumber(&SettingsNP, "Motor settings are now  %3.0f %3.0f %3.0f", currentDuty, currentDelay, currentTicks);
    //                  return true;
    //              } else
    //              {
    //                  /* Set property state to idle */
    //                  SettingsNP.s = IPS_IDLE;
    //                  IDSetNumber(&SettingsNP, "Settings absent or bogus.");
    //                  return false ;
    //              }
    //          }

    // -------------------SetBacklash---------------------------------------------
    if (!strcmp (name, SetBacklashNP.name))
    {
        double new_back = 0 ;
        int nset = 0;
        int ret= -1 ;

        for (nset = i = 0; i < n; i++)
        {
            /* Find numbers with the passed names in the SetBacklashNP property */
            INumber *eqp = IUFindNumber (&SetBacklashNP, names[i]);
            /* If the number found is SetBacklash (SetBacklashN[0]) then process it */
            if (eqp == &SetBacklashN[0]){
                new_back = (values[i]);
                /* limits */
                nset += new_back >= -0xff && new_back <= 0xff;
            }
            if (nset == 1) {
                /* Set the lfepFocus state to BUSY */
                SetBacklashNP.s = IPS_BUSY;
                IDSetNumber(&SetBacklashNP, NULL);
                if(( ret= updateLFEPBacklashFocus(&new_back)) < 0) {
                    SetBacklashNP.s = IPS_IDLE;
                    IDSetNumber(&SetBacklashNP, "Setting new backlash failed.");
                    return false ;
                }
                currentFocusSetBacklash = new_back ;
                SetBacklashNP.s = IPS_OK;
                IDSetNumber(&SetBacklashNP, "Backlash is now  %3.0f", currentFocusSetBacklash) ;
                return true;
            } else {
                SetBacklashNP.s = IPS_IDLE;
                IDSetNumber(&SetBacklashNP, "Need exactly one parameter.");
                return false ;
            }
        }
    }

    // -------------------------MinMaxPosition--------------------------------------
    if (!strcmp (name, MinMaxPositionNP.name))
    {
        /* new positions */
        double new_min = 0 ;
        double new_max = 0 ;

        for (nset = i = 0; i < n; i++)
        {
            /* Find numbers with the passed names in the MinMaxPositionNP property */
            INumber *mmpp = IUFindNumber (&MinMaxPositionNP, names[i]);
            /* If the number found is  (MinMaxPositionN[0]) then process it */
            if (mmpp == &MinMaxPositionN[0])
            {
                new_min = (values[i]);
                nset += new_min >= 1 && new_min <= MAX_N_STEP;
            } else if  (mmpp == &MinMaxPositionN[1])
            {
                new_max = (values[i]);
                nset += new_max >= 1 && new_max <= MAX_N_STEP;
            }
        }
        if (nset == 2)
        {
            /* Set the lfepFocus state to BUSY */
            MinMaxPositionNP.s = IPS_BUSY;
            FocusAbsPosN[0].min = new_min ;
            //          currentFocusMinPosition = FocusAbsPosN[0].min ;
            FocusAbsPosN[0].max= new_max ;
            //          currentFocusMaxPosition = FocusAbsPosN[0].max ;
            MinMaxPositionNP.s = IPS_OK;
            IDSetNumber(&MinMaxPositionNP, "Minimum and Maximum settings are now  %3.0f %3.0f", FocusAbsPosN[0].min, FocusAbsPosN[0].max);
            return true;
        } else
        {
            /* Set property state to idle */
            MinMaxPositionNP.s = IPS_IDLE;
            IDSetNumber(&MinMaxPositionNP, "Minimum and maximum limits absent or bogus.");
            return false;
        }
    }

    // -----------------------max Travel---------------------------------------------
    if (!strcmp (name, MaxTravelNP.name))   //=max Tick
    {
        double new_maxt = 0 ;
        int ret = -1 ;

        for (nset = i = 0; i < n; i++)
        {
            /* Find numbers with the passed names in the MinMaxPositionNP property */
            INumber *mmpp = IUFindNumber (&MaxTravelNP, names[i]);
            /* If the number found is  (MaxTravelN[0]) then process it */
            if (mmpp == &MaxTravelN[0])
            {
                new_maxt = (values[i]);
                nset += new_maxt >= 1 && new_maxt <= MAX_N_STEP;
            }
        }
        if (nset == 1) {
            IDSetNumber(&MinMaxPositionNP, NULL);
            if(( ret= updateLFEPMaxPosition(&new_maxt))< 0 )
            {
                MaxTravelNP.s = IPS_IDLE;
                IDSetNumber(&MaxTravelNP, "Changing to new maximum travel failed");
                return false ;
            }
            currentFocusMaxTravel=  new_maxt ;
            MaxTravelNP.s = IPS_OK;
            IDSetNumber(&MaxTravelNP, "Maximum travel is now  %3.0f", currentFocusMaxTravel) ;
            return true;
        } else {
            /* Set property state to idle */
            MaxTravelNP.s = IPS_IDLE;
            IDSetNumber(&MaxTravelNP, "Maximum travel absent or bogus.");
            return false ;
        }
    }

    // ------------------Sync Focus ----------------------------------------
    if (!strcmp (name, SetRegisterPositionNP.name))
    {
        double new_apos = 0 ;
        int nset = 0;
        int ret= -1 ;

        for (nset = i = 0; i < n; i++)
        {
            /* Find numbers with the passed names in the SetRegisterPositionNP property */
            INumber *srpp = IUFindNumber (&SetRegisterPositionNP, names[i]);
            /* If the number found is SetRegisterPosition (SetRegisterPositionN[0]) then process it */
            if (srpp == &SetRegisterPositionN[0])
            {
                new_apos = (values[i]);
                /* limits are absolute */
                nset += new_apos >= 0 && new_apos <= MAX_N_STEP;
            }
            if (nset == 1)
            {
                if((new_apos < FocusAbsPosN[0].min) || (new_apos > FocusAbsPosN[0].max))
                {
                    SetRegisterPositionNP.s = IPS_ALERT ;
                    IDSetNumber(&SetRegisterPositionNP, "Value out of limits  %5.0f", new_apos);
                    return false ;
                }
                /* Set the lfepFocus state to BUSY ----  Sync */
                SetRegisterPositionNP.s = IPS_BUSY;
                IDSetNumber(&SetRegisterPositionNP, NULL);
                if(( ret= updateLFEPFocusSetPosition(&new_apos)) < 0)// set position lfepFocus
                {
                    SetRegisterPositionNP.s = IPS_OK;
                    IDSetNumber(&SetRegisterPositionNP, "Read out of the set position to %3d failed. Trying to recover the position", ret);
                    if((ret= getLFEPPositionFocus( &currentFocusPosition)) < 0)// get position lfepFocus
                    {
                        FocusAbsPosNP.s = IPS_ALERT;
                        IDSetNumber(&FocusAbsPosNP, "Unknown error while reading Focus position: %d", ret);
                        SetRegisterPositionNP.s = IPS_IDLE;
                        IDSetNumber(&SetRegisterPositionNP, "Relative movement failed.");
                    }
                    SetRegisterPositionNP.s = IPS_OK;
                    IDSetNumber(&SetRegisterPositionNP, NULL);
                    FocusAbsPosNP.s = IPS_OK;
                    IDSetNumber(&FocusAbsPosNP, "Focus position recovered %5.0f", currentFocusPosition);
                    DEBUG(INDI::Logger::DBG_DEBUG, "Focus position recovered resuming normal operation");
                    /* We have to leave here, because new_apos is not set */
                    return true ;
                }
                currentFocusPosition = new_apos ;
                FocusAbsPosNP.s = IPS_OK;
                IDSetNumber(&FocusAbsPosNP, "Focus position is now %5.0f", currentFocusPosition);
                SetRegisterPositionNP.s = IPS_OK;
                SetRegisterPositionN[0].value = currentFocusPosition;
                IDSetNumber(&SetRegisterPositionNP, "Focus register set to %5.0f", currentFocusPosition);
                return true ;
            } else
            {
                SetRegisterPositionNP.s = IPS_IDLE;
                IDSetNumber(&SetRegisterPositionNP, "Need exactly one parameter.");
                return false;
            }
            if((ret= getLFEPPositionFocus(&currentFocusPosition)) < 0)
            {
                FocusAbsPosNP.s = IPS_ALERT;
                DEBUGF(INDI::Logger::DBG_ERROR, "Unknown error while reading Focus position: %d", ret);
                IDSetNumber(&FocusAbsPosNP, NULL);
                return false ;
            }
            SetRegisterPositionNP.s = IPS_OK;
            SetRegisterPositionN[0].value = currentFocusPosition;
            IDSetNumber(&SetRegisterPositionNP, "Focus has accepted new register setting" ) ;
            FocusAbsPosNP.s = IPS_OK;
            DEBUGF(INDI::Logger::DBG_SESSION, "Focus new position %5.0f", currentFocusPosition);
            IDSetNumber(&FocusAbsPosNP, NULL);

            return true;
        }
    }

    // ------------------------Preset position-------------------------------------
    if (!strcmp(name, PresetNP.name))
    {
        IUUpdateNumber(&PresetNP, values, names, n);
        PresetNP.s = IPS_OK;
        IDSetNumber(&PresetNP, NULL);
        //TODO: saveConfig();

        return true;
    }

    // ------------------------ABS_FOCUS_POSITION-----------------------------------
    if(strcmp(name,"ABS_FOCUS_POSITION")==0)
    {
        int newPos = (int) values[0];

        if (newPos < FocusAbsPosN[0].min)
        {
            FocusAbsPosNP.s = IPS_ALERT;
            IDSetNumber(&FocusAbsPosNP, NULL);
            DEBUGFDEVICE(dev, INDI::Logger::DBG_ERROR, "Requested position absolute out of bound. Focus minimum position is %g", FocusAbsPosN[0].min);
            return false;
        }
        else if (newPos > FocusAbsPosN[0].max)
        {
            FocusAbsPosNP.s = IPS_ALERT;
            IDSetNumber(&FocusAbsPosNP, NULL);
            DEBUGFDEVICE(dev, INDI::Logger::DBG_ERROR, "Requested position absolute out of bound. Focus maximum position is %g", FocusAbsPosN[0].max);
            return false;
        }
        IPState ret;
        if ( (ret = MoveAbsFocuser(newPos)) == IPS_OK)
        {
            FocusAbsPosNP.s=IPS_OK;
            IUUpdateNumber(&FocusAbsPosNP,values,names,n);
            IDSetNumber(&FocusAbsPosNP, "Focuser moved to position %d", newPos);
            return true;
        }
        else if (ret == IPS_BUSY)
        {
            FocusAbsPosNP.s=IPS_BUSY;
            IDSetNumber(&FocusAbsPosNP, "Focuser is moving to position %d", newPos);
            return true;
        }
        FocusAbsPosNP.s = IPS_ALERT;
        IDSetNumber(&FocusAbsPosNP, "Focuser failed to move to new requested position.");
        return false;
    }

    // ------------------------REL_FOCUS_POSITION-----------------------------------
    if(strcmp(name,"REL_FOCUS_POSITION")==0)
    {
        int newPos = (int) values[0];

        if (newPos <= 0)
        {
            DEBUGDEVICE(dev, INDI::Logger::DBG_ERROR, "Relative ticks value must be greater than zero.");
            FocusRelPosNP.s = IPS_ALERT;
            IDSetNumber(&FocusRelPosNP, NULL);
            return false;
        }
        IPState ret;
        if (FocusMotionS[0].s == ISS_ON)
        {
            if (FocusAbsPosN[0].value - newPos < FocusAbsPosN[0].min)
            {
                FocusRelPosNP.s = IPS_ALERT;
                IDSetNumber(&FocusRelPosNP, NULL);
                DEBUGFDEVICE(dev, INDI::Logger::DBG_ERROR, "Requested position relative out of bound. Focus minimum position is %g", FocusAbsPosN[0].min);
                return false;
            }
        }
        else
        {
            if (FocusAbsPosN[0].value + newPos > FocusAbsPosN[0].max)
            {
                FocusRelPosNP.s = IPS_ALERT;
                IDSetNumber(&FocusRelPosNP, NULL);
                DEBUGFDEVICE(dev, INDI::Logger::DBG_ERROR, "Requested position relative out of bound. Focus maximum position is %g", FocusAbsPosN[0].max);
                return false;
            }
        }
        if ( (ret=MoveRelFocuser( (FocusMotionS[0].s == ISS_ON ? FOCUS_INWARD : FOCUS_OUTWARD), newPos)) == IPS_OK)
        {
            FocusRelPosNP.s=FocusAbsPosNP.s=IPS_OK;
            IUUpdateNumber(&FocusRelPosNP,values,names,n);
            IDSetNumber(&FocusRelPosNP, "Focuser moved %d steps %s", newPos, FocusMotionS[0].s == ISS_ON ? "inward" : "outward");
            IDSetNumber(&FocusAbsPosNP, NULL);
            return true;
        }
        else if (ret == IPS_BUSY)
        {
            IUUpdateNumber(&FocusRelPosNP,values,names,n);
            FocusRelPosNP.s=FocusAbsPosNP.s=IPS_BUSY;
            IDSetNumber(&FocusAbsPosNP, "Focuser is moving %d steps %s...", newPos, FocusMotionS[0].s == ISS_ON ? "inward" : "outward");
            IDSetNumber(&FocusAbsPosNP, NULL);
            return true;
        }
        FocusRelPosNP.s = IPS_ALERT;
        IDSetNumber(&FocusRelPosNP, "Focuser failed to move to new requested position.");
        return false;
    }
    return false;

}

bool LX200LFEP::processFocuserSwitch (const char * name, ISState * states, char * names[], int n)
{
    int index;

    // =======================================
    // Focus Motion
    // =======================================

    if(!strcmp(name,FocusMotionSP.name))
    {
        //  client is telling us what to do with focus direction
        FocusMotionSP.s=IPS_OK;
        IUUpdateSwitch(&FocusMotionSP,states,names,n);
        IDSetSwitch(&FocusMotionSP,NULL);

        return true;
    }

    // Focus Motion
    //    if (!strcmp(name, FocusMotionSP.name))
    //    {
    //        // If mode is "halt"
    //        if (FocusModeS[0].s == ISS_ON)
    //        {
    //            FocusMotionSP.s = IPS_IDLE;
    //            IDSetSwitch(&FocusMotionSP, "Focus mode is halt. Select slow or fast mode");
    //            return true;
    //        }

    //        int last_motion = IUFindOnSwitchIndex(&FocusMotionSP);

    //        if (IUUpdateSwitch(&FocusMotionSP, states, names, n) < 0)
    //            return false;

    //        index = IUFindOnSwitchIndex(&FocusMotionSP);
    //        // If same direction and we're busy, stop
    //        if (last_motion == index && FocusMotionSP.s == IPS_BUSY)
    //        {
    //            IUResetSwitch(&FocusMotionSP);
    //            FocusMotionSP.s = IPS_IDLE;
    //            setFocuserSpeedMode(PortFD, 0);
    //            IDSetSwitch(&FocusMotionSP, nullptr);
    //            return true;
    //        }
    //        if (!isSimulation() && setFocuserMotion(PortFD, index) < 0)
    //        {
    //            FocusMotionSP.s = IPS_ALERT;
    //            IDSetSwitch(&FocusMotionSP, "Error setting focuser speed.");
    //            return false;
    //        }
    //        // with a timer
    //        if (FocusTimerN[0].value > 0)
    //        {
    //            FocusTimerNP.s  = IPS_BUSY;
    //            FocusMotionSP.s = IPS_BUSY;
    //            IEAddTimer(50, LX200Generic::updateFocusHelper, this);
    //        }
    //        FocusMotionSP.s = IPS_OK;
    //        IDSetSwitch(&FocusMotionSP, nullptr);
    //        return true;
    //    }

    // =======================================
    // Focus Speed
    // =======================================
    if (!strcmp(name, FocusModeSP.name))
    {
        IUResetSwitch(&FocusModeSP);
        IUUpdateSwitch(&FocusModeSP, states, names, n);
        index = IUFindOnSwitchIndex(&FocusModeSP);
        /* disable timer and motion */
        if (index == 0)
        {
            IUResetSwitch(&FocusMotionSP);
            FocusMotionSP.s = IPS_IDLE;
            //           FocusTimerNP.s  = IPS_IDLE;
            IDSetSwitch(&FocusMotionSP, nullptr);
            //           IDSetNumber(&FocusTimerNP, nullptr);
        }
        if (!isSimulation())
            setFocuserSpeedMode(PortFD, index);
        FocusModeSP.s = IPS_OK;
        IDSetSwitch(&FocusModeSP, nullptr);
        return true;
    }

    // =======================================
    // Focus Abort
    // =======================================

    if(strcmp(name,"FOCUS_ABORT_MOTION")==0)
    {
        IUResetSwitch(&AbortFocusSP);

        if (AbortFocuser())
        {
            AbortFocusSP.s = IPS_OK;
            if (FocusAbsPosNP.s != IPS_IDLE)
            {
                FocusAbsPosNP.s = IPS_IDLE;
                IDSetNumber(&FocusAbsPosNP, NULL);
            }
            if (FocusRelPosNP.s != IPS_IDLE)
            {
                FocusRelPosNP.s = IPS_IDLE;
                IDSetNumber(&FocusRelPosNP, NULL);
            }
        }
        else
            AbortFocusSP.s = IPS_ALERT;

        IDSetSwitch(&AbortFocusSP, NULL);
        return true;
    }

    return false;

}



















