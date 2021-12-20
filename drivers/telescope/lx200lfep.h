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
//---------------------------------------------------------------------
#ifndef LX200LFEP_H
#define LX200LFEP_H

#include "lx200generic.h"
#include "indibase/indifocuser.h"


class LX200LFEP : public LX200Generic
{
public:

    LX200LFEP();
    ~LX200LFEP() {}
    //      virtual bool Handshake();
    virtual bool initProperties();
    virtual bool updateProperties();
    virtual bool ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n);
    virtual bool ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n);
    virtual void ISGetProperties(const char *dev);

    //focus
    enum FocusDirection { FOCUS_INWARD, FOCUS_OUTWARD };
    virtual IPState MoveAbsFocuser(uint32_t ticks);
    virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks);
    virtual bool AbortFocuser();
    virtual void TimerHit();

protected:
    //--------------------------------------------------------------------------
    void set_lx200lfep_name(const char *deviceName, unsigned int debug_level);

    int selectLFEPTrackingMode(int fd, int trackMode);
    int setLFEPpierSide(int fd, int currentSwap) ;
    int swapLFEPKingButtons(int fd, int currentSwap) ;
    int setLFEPSiteLongitude(int fd, double Long) ;
    int setLFEPSiteLatitude(int fd, double Lat) ;
    int selectLFEPguideRateRaDec(int fd, int guideRateAr, int guideRateDec) ;

    //void GetFocusParams () ;
    //--------------------------------------------------------------------------

    virtual const char *getDefaultName();
    virtual bool isSlewComplete();
    virtual bool checkConnection();
//    virtual void debugTriggered(bool enable) override;
    virtual bool saveConfigItems(FILE *fp);
    virtual bool ReadScopeStatus();
    void GetTelescopeLFEParams ();

    // Time and Location
    virtual bool updateLocation(double latitude, double longitude, double elevation);
    virtual bool updateTime(ln_date *utc, double utc_offset);
    int setCalenderDateLFEP(int fd, int dd, int mm, int yy);
    //int ReadUntilComplete(char *buf, int timeout);

    // Moving
    virtual bool MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) override;
    virtual bool Goto(double, double) override;

    // Guide Pulse Commands
    virtual int SendPulseCmd(int direction, int duration_msec) ;//override;

    // Parking
    virtual bool Park();
    virtual bool UnPark();
    virtual bool SetCurrentPark();
    virtual bool SetDefaultPark();

    // Tracking
    virtual bool SetTrackMode(uint8_t mode) override;
    virtual bool SetTrackEnabled(bool enabled) override;
    //virtual bool SetTrackRate(double raRate, double deRate) override;

    //Meridian Flip
    bool MeridianFlip(bool FlipStatus);

    // Focus
    bool processFocuserNumber (const char * dev, const char * name, double values[], char * names[], int n);
    bool processFocuserSwitch (const char * name, ISState * states, char * names[], int n);
    int getLFEPPositionFocus(double *value);
    int updateLFEPTemperatureFocus(double *value) ;
    int updateLFEPBacklashFocus(double *value);
    int updateLFEPMotorSettings(double *duty, double *delay, double *ticks);
    int updateLFEPPositionFocusRelativeInward(double value);
    int updateLFEPPositionFocusRelativeOutward(double value);
    int updateLFEPPositionFocusAbsolute(double value);
    int updateLFEPPowerSwitches(int s, int  new_sn, int *cur_s1LL, int *cur_s2LR, int *cur_s3RL, int *cur_s4RR) ;
    int updateLFEPMaxPosition(double *value);
    int updateLFEPFocusSetPosition(double *value);

    // State InternalSettings LFEP
    bool GetInternalSettingsLFEP();
    unsigned char CheckSum(char *LFEP_cmd);
    unsigned char CalculateSum(char *LFEP_cmd);
    bool ReadFocusStatus();
    int SendCommand(char *LFEP_cmd);
    int ReadResponseFocus(char *buf);
    void GetFocusParams();

    // Variables
//    char lx200lfep_name[MAXINDIDEVICE];
//    unsigned int LFEP_DBG_SCOPE;
    unsigned int DBG_FOCUS;
    int timerID;
    double targetPos;
    double simulatedTemperature;
    double simulatedPosition;

    //--------------------------------------

    INumber SlewAccuracyN[2];
    INumberVectorProperty SlewAccuracyNP;

    //    ISwitch SwapS[2];
    //    ISwitchVectorProperty SwapSP;

    ISwitch SwapKingRateS[2];
    ISwitchVectorProperty SwapKingRateSP;

    ISwitch TrackModeS[5];
    ISwitchVectorProperty TrackModeSP;

    ISwitch GuideSpeedArS[3];
    ISwitchVectorProperty GuideSpeedArSP;

    ISwitch GuideSpeedDecS[3];
    ISwitchVectorProperty GuideSpeedDecSP;

    ISwitch FlipEnableS[2];
    ISwitchVectorProperty FlipEnableSP;

    ISwitch FlipThroughS[2];
    ISwitchVectorProperty FlipThroughSP;
    //------------------------------------------------------

    INumber TemperatureN[1];
    INumberVectorProperty TemperatureNP;

    INumber SettingsN[3];
    INumberVectorProperty SettingsNP;

    INumber MinMaxPositionN[2];
    INumberVectorProperty MinMaxPositionNP;

    INumber MaxTravelN[1];
    INumberVectorProperty MaxTravelNP;

    INumber SetRegisterPositionN[1];
    INumberVectorProperty SetRegisterPositionNP;

    INumber RelMovementN[1];
    INumberVectorProperty RelMovementNP;

    INumber AbsMovementN[1];
    INumberVectorProperty AbsMovementNP;

    INumber SetBacklashN[1];
    INumberVectorProperty SetBacklashNP;

    INumber MinMaxPositionFlipN[1];
    INumberVectorProperty MinMaxPositionFlipNP;

    INumberVectorProperty FocusSpeedNP;
    INumber FocusSpeedN[1];

    INumberVectorProperty FocusTimerNP;
    INumber FocusTimerN[1];
    INumberVectorProperty FocusAbsPosNP;
    INumber FocusAbsPosN[1];
    INumberVectorProperty FocusRelPosNP;
    INumber FocusRelPosN[1];
    ISwitchVectorProperty AbortFocusSP;
    ISwitch AbortFocusS[1];
 //   ISwitchVectorProperty FocusSpeedSP;
 //   ISwitch FocusSpeedS[2];
    IPState lastEqState;

    INumber PresetN[4];
    INumberVectorProperty PresetNP;
    ISwitch PresetGotoS[4];
    ISwitchVectorProperty PresetGotoSP;
    //------------------------------------------------------

    double Elevation, Latitud, Longitude;

private:
    bool initMountLFEP();
    void syncSideOfPier();
    bool PierSideSensor {false};
    bool mountInitialized=false, locationUpdated=false;
    bool forceMeridianFlip {false};
    //bool doubleMovement {false};
    double oldTargetRA, oldTargetDEC;
    ln_lnlat_posn lnobserver { 0, 0 };

    struct _getLFEPinfo
    {
        /** RBacklash RA **/
        int BacklashRA =-1;
        /** RBacklash DEC **/
        int BacklashDE =-1;
        /** Sync AR & DEC **/
        bool SyncArDec;
        /** Slew AR & DEC **/
        bool SlewArDec;
        /** Tracking State **/
        bool TrackingS;
        /** Motor Direction RA (0=normal, 1=inverse)**/
        bool MotorDirectionRA;
        /** Motor Direction DEC (0=normal, 1=inverse)**/
        bool MotorDirectionDE;
        /** PierSide / Invert DEC **/
        bool PierSide;
        /** Sideral Speed [0=SID,1=SOL,2=LUN,3=USR,255=OFF]**/
        int SideralSpeed =-1;
        /** Guide Speed RA [1=0.33x, 2=0.67x, 3=1x]**/
        int GuideSpeedRA =-1;
        /** Guide Speed DEC [1=0.33x, 2=0.67x, 3=1x]**/
        int GuideSpeedDE =-1;
        /** Guide Speed Range **/
        int GuideSpeedRange=-1;
    } getLFEPinfo;
};


#endif
