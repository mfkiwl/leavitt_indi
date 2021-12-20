/*******************************************************************************
  INDI DAEnetIP3 Relay I/O Module Driver
  Copyright (C) 2019 Emanuele Pavoni

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.

 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#include "DAEnetIP3.h"
#include "defaultdevice.h"
#include "indicom.h"

#include <cstring>
#include <memory>
//TODO: da rimuovere qui sotto?
#include "indicontroller.h"
#include "connectionplugins/connectionserial.h"
#include "connectionplugins/connectiontcp.h"
//--------------
#include "indifilterinterface.h"
#include <unistd.h>
#include <termios.h>
//#include "indifilterwheel.h"
#include <iostream>
#include <string>
#include<fstream>

//---------------------------------------------------
#define DAENET_TIMEOUT  4
#define CMD_SIZE_A      12
#define BUF_SIZE_A      11
#define BUF_SIZE_B      9
#define BUF_SIZE_C      17
#define MAX_BUF_LABEL   17
#define MAX_LABEL       13
#define D_OUT_N         16
#define D_INP_N         8
#define A_INP_N         8
//----------------------------------------------------
#define SETTINGS_TAB         "Out Settings" //TODO:da rivedere
#define DIGITAL_OUTPUT_TAB   "Digital Output"
#define DIGITAL_INPUT_TAB    "Digital Input"
#define ANALOG_INPUT_TAB     "Analog Input"
//#define _TAB             "Meridian Flip"
//----------------------------------------------------
int salto = 0;

// We declare an auto pointer to DAEnetIP3.
static std::unique_ptr<DAEnetIP3> daenet(new DAEnetIP3());

//void ISPoll(void *p);

void ISGetProperties(const char *dev)
{
    daenet->ISGetProperties(dev);

}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    daenet->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    daenet->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    daenet->ISNewNumber(dev, name, values, names, n);
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
               char *names[], int n)
{
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}
void ISSnoopDevice(XMLEle *root)
{
    daenet->ISSnoopDevice(root);
}

DAEnetIP3::DAEnetIP3()
{
    setVersion(1, 0);
    //   DAEConnection(CONNECTION_SERIAL | CONNECTION_TCP);
}

const char *DAEnetIP3::getDefaultName()
{
    return static_cast<const char *>("DAEnetIP3");
}

bool DAEnetIP3::initProperties()
{
    int i;
    char propName[16]={0}, propLabel[16]={0};
    INDI::DefaultDevice::initProperties();

    //    IUFillSwitch(&HomeS[0], "Find", "Find", ISS_OFF);
    //    IUFillSwitchVector(&HomeSP, HomeS, 1, getDeviceName(), "HOME", "Home", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 60,
    //                       IPS_IDLE);

    //-------------------------------------------------
    if (DAEnetConnection & CONNECTION_SERIAL)
    {
        serialConnection = new Connection::Serial(this);
        serialConnection->registerHandshake([&]() { return callHandshake(); });
        registerConnection(serialConnection);
    }

    if (DAEnetConnection & CONNECTION_TCP)
    {
        tcpConnection = new Connection::TCP(this);
        tcpConnection->registerHandshake([&]() { return callHandshake(); });
        registerConnection(tcpConnection);
    }
    //-------------------------------------------------

    // Digital Settings
    for (i = 0; i < D_OUT_N; i++)
    {
        snprintf(propName, 16, "PIN_S%d", i + 1);
        IUFillSwitch(&DigitalSettingS[i], propName, propLabel, ISS_OFF);
    }
    IUFillSwitchVector(&DigitalSettingSP, DigitalSettingS, D_OUT_N, getDeviceName(), "PORT_A", "Digital Output", DIGITAL_OUTPUT_TAB, IP_RW, ISR_NOFMANY, 60, IPS_IDLE);

    // Digital Readings
    for (i = 0; i < D_INP_N; i++)
    {
        snprintf(propName, 16, "PIN_D%d", i + 1);
        IUFillLight(&DigitalReadingL[i], propName, propLabel, IPS_IDLE);
    }
    IUFillLightVector(&DigitalReadingLP, DigitalReadingL, D_INP_N, getDeviceName(), "PORT_B", "Digital Inputs", DIGITAL_INPUT_TAB, IPS_IDLE);

    // Analog Readings
    for (i = 0; i < A_INP_N; i++)
    {
        snprintf(propName, 16, "PIN_A%d", i + 1);
        IUFillNumber(&AnalogReadingN[i], propName, propLabel, "%7.4f", -20, 30, 0, 0);
    }
    IUFillNumberVector(&AnalogReadingNP, AnalogReadingN, A_INP_N, getDeviceName(), "PORT_C", "Analog Inputs", ANALOG_INPUT_TAB, IP_RO, 0, IPS_IDLE);

    addAuxControls();
    //addDebugControl();
    return true;

}

bool DAEnetIP3::updateProperties()
{
    int i;
    char propLabel[20]={0};

    INDI::DefaultDevice::updateProperties();
    if (isConnected())
    {
       
        
        for (i = 0; i < D_OUT_N; i++)
        {
            snprintf(propLabel, 16, "%d: ", i + 1);
            getLabelPort('A', i, propLabel);
            strncpy(DigitalSettingS[i].label, propLabel, MAX_LABEL);
        }

        for (i = 0; i < D_INP_N; i++)
        {
            snprintf(propLabel, 16, "%d: ", i + 1);
            getLabelPort('B', i, propLabel);
            strncpy(DigitalReadingL[i].label, propLabel, MAX_LABEL);
        }

        for (i = 0; i < A_INP_N; i++)
        {
            snprintf(propLabel, 16, "%d: ", i + 1);
            getLabelPort('C', i, propLabel);
            getReadUnitLabelPortC(i, propLabel);
            strncpy(AnalogReadingN[i].label, propLabel, MAX_LABEL + 9);           
        }

        defineProperty(&DigitalSettingSP); //  defineSwitch(&DigitalSettingSP);
        defineProperty(&DigitalReadingLP); //  defineLight(&DigitalReadingLP);
        defineProperty(&AnalogReadingNP);  //  defineNumber(&AnalogReadingNP);


    }
    else
    {
        deleteProperty(DigitalSettingSP.name);
        deleteProperty(DigitalReadingLP.name);
        deleteProperty(AnalogReadingNP.name);
    }
    return true;

}

bool DAEnetIP3::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    int err = 0;
    int i;
    uint16_t portA_val=0;

    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // =======================================
        // Digital Outputs Setting
        // =======================================
        if (!strcmp(name, DigitalSettingSP.name))
        {
            IUResetSwitch(&DigitalSettingSP);
            if (IUUpdateSwitch(&DigitalSettingSP, states, names, n) < 0)
                return false;
            for (i = 0 ; i < D_OUT_N ; i++)
            {
                if(DigitalSettingS[i].s == ISS_ON)
                    portA_val |= 1 << i;
            }
            DEBUGF(INDI::Logger::DBG_SESSION, "Settings dig.out (%#04X).", portA_val);
            if ((isSimulation() == false && (err = setDAEnetPortA(portA_val)) < 0))
            {
                DEBUGF(INDI::Logger::DBG_ERROR, "Error set DaenetIP3 dig.out (%d).", err);
                DigitalSettingSP.s = IPS_ALERT;
                IDSetSwitch(&DigitalSettingSP, "Could not set DaenetIP3 Digital Outputs ");
                return false;
            }

            DigitalSettingSP.s = IPS_OK;
            IDSetSwitch(&DigitalSettingSP, nullptr);//client update
            return true;
        }
    }

    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool DAEnetIP3::Handshake()
{
    //TODO: DA FINIRE:  How do we do handshake with DAEnetIP3? We return true for now

    if (isSimulation())
    {
        LOGF_INFO("Connected successfully to simulated %s. Retrieving startup data...",
               getDeviceName());

        SetTimer(getCurrentPollingPeriod());
        return true;
    }

//    PortFD = serialConnection->getPortFD();

//    /* Drop RTS */
//    int i = 0;
//    i |= TIOCM_RTS;
//    if (ioctl(PortFD, TIOCMBIC, &i) != 0)
//    {
//        LOGF_ERROR("IOCTL error %s.", strerror(errno));
//        return false;
//    }

//    i |= TIOCM_RTS;
//    if (ioctl(PortFD, TIOCMGET, &i) != 0)
//    {
//        LOGF_ERROR("IOCTL error %s.", strerror(errno));
//        return false;
//    }

//    if (!ping())
//    {
//        LOG_ERROR("Device ping failed.");
//        return false;
//    }

    return true;

}

bool DAEnetIP3::callHandshake()
{
    if (DAEnetConnection > 0)
    {
        if (getActiveConnection() == serialConnection)
            PortFD = serialConnection->getPortFD();
        else if (getActiveConnection() == tcpConnection)
            PortFD = tcpConnection->getPortFD();
    }

    return Handshake();
}

void DAEnetIP3::TimerHit()
{
    bool rc;

    if (!isConnected())
        return;
    rc = getReadingsDAEnet();
    //TODO: update info
    //    AnalogReadingNP.s = rc ? IPS_OK : IPS_ALERT;
    //    IDSetNumber(&AnalogReadingNP, nullptr);
    SetTimer(getCurrentPollingPeriod());
}
//-------------------------------------------------------------------------------

void DAEnetIP3::DAEConnection(const uint8_t &value)
{
    uint8_t mask = CONNECTION_SERIAL | CONNECTION_TCP | CONNECTION_NONE;

    if (value == 0 || (mask & value) == 0)
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "Invalid connection mode %d", value);
        return;
    }

    DAEnetConnection = value;
}

bool DAEnetIP3::getReadPortA()
{
    int error_type;
    char bufferA[BUF_SIZE_A]={0};
    const char *cmdA = "00ASG=?;";
    int nbytes_write = 0, nbytes_read = 0, i;
    int port_A;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD: %s", cmdA);
    //DEBUGFDEVICE(getDefaultName(), DBG_DEBUG, "CMD (%s)", cmdA);
    tcflush(PortFD, TCIOFLUSH);
    if ((error_type = tty_write_string(PortFD, cmdA, &nbytes_write)) != TTY_OK)
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "Error getting device readings, error type: %d", error_type);
        return false;
    }
    if ((error_type = tty_nread_section(PortFD, bufferA, BUF_SIZE_A, ';', DAENET_TIMEOUT, &nbytes_read))!= TTY_OK)
    {
        //DEBUGF(INDI::Logger::DBG_ERROR, "TTY error detected PortA: %d", error_type);
        return false;
    }

    tcflush(PortFD, TCIOFLUSH);
    if (nbytes_read < 0)
        return false;

    DEBUGF(INDI::Logger::DBG_DEBUG, "Port A, digital Outputs: %s", bufferA);

    // ------------- Update Port A -----------------

    if(sscanf(bufferA, "00ASG=%04X;", &port_A))
    {
        if(port_A != old_port_A)
        {
            DEBUGF(INDI::Logger::DBG_DEBUG, "PortA: %d , Old_PortA: %d", port_A , old_port_A);

            for (i = 0 ; i < D_OUT_N ; i++)
            {
                if(port_A & (0b1 << i))
                {
                    //DEBUG(INDI::Logger::DBG_SESSION, "ON");
                    DigitalSettingS[i].s = ISS_ON;
                }
                else
                {
                    //DEBUG(INDI::Logger::DBG_SESSION, "OFF");
                    DigitalSettingS[i].s = ISS_OFF;
                }
            }
            DigitalSettingSP.s = IPS_OK;
            IDSetSwitch(&DigitalSettingSP, nullptr);//client update
            old_port_A = port_A;
        }
    }
    return true;

}

bool DAEnetIP3::getReadPortB()
{
    int error_type;
    char bufferB[BUF_SIZE_B]={0};
    const char *cmdB = "00BVG=?;";
    int nbytes_write = 0, nbytes_read = 0, i;
    int port_B;
    DigitalReadingLP.s = IPS_IDLE;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD: %s", cmdB);
    //DEBUGFDEVICE(getDefaultName(), DBG_DEBUG, "CMD (%s)", cmdB);
    tcflush(PortFD, TCIOFLUSH);
    if ((error_type = tty_write_string(PortFD, cmdB, &nbytes_write)) != TTY_OK)
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "Error getting device readings, error type: %d", error_type);
        return false;
    }

    if ((error_type = tty_nread_section(PortFD, bufferB, BUF_SIZE_B, ';', DAENET_TIMEOUT, &nbytes_read))!= TTY_OK)
    {
        //DEBUGF(INDI::Logger::DBG_ERROR, "TTY error detected PortB: %d", error_type);
        return false;
    }
    tcflush(PortFD, TCIOFLUSH);
    if (nbytes_read < 0)
        return false;

    DEBUGF(INDI::Logger::DBG_DEBUG, "Port B, digital Inputs: %s", bufferB);

    // ------------- Update Port B -----------------

    if(sscanf(bufferB, "00BVG=%04X;", &port_B))
    {
        if(port_B != old_port_B)
        {
            DEBUGF(INDI::Logger::DBG_DEBUG, "PortB: %d , Old_PortB: %d", port_B , old_port_B);

            for (i = 0 ; i < D_INP_N ; i++)
            {
                if(port_B & (0b1 << i))
                {
                    //DEBUG(INDI::Logger::DBG_SESSION, "ON");
                    DigitalReadingL[i].s = IPS_OK;
                }
                else
                {
                    //DEBUG(INDI::Logger::DBG_SESSION, "OFF");
                    DigitalReadingL[i].s = IPS_IDLE;
                }
            }
            DigitalReadingLP.s = IPS_OK;
            IDSetLight(&DigitalReadingLP, nullptr);//client update
            old_port_B = port_B;
        }
    }

    return true;

}

bool DAEnetIP3::getReadPortC()
{
    int error_type;
    char bufferC[BUF_SIZE_C]={0};
    char cmdC[CMD_SIZE_A];
    int nbytes_write = 0, nbytes_read = 0, i;
    float analogValue;

    AnalogReadingNP.s = IPS_IDLE;
    for(i=0; i<A_INP_N; i++)
    {
        sprintf(cmdC, "00CD%01d=?;", i);
        DEBUGF(INDI::Logger::DBG_DEBUG, "CMD_C: %s", cmdC);
        //DEBUGFDEVICE(getDefaultName(), DBG_DEBUG, "CMD (%s)", cmdC);
        tcflush(PortFD, TCIOFLUSH);
        if ((error_type = tty_write_string(PortFD, cmdC, &nbytes_write)) != TTY_OK)
        {
            DEBUGF(INDI::Logger::DBG_ERROR, "Error getting device readings, error type: %d", error_type);
            return false;
        }

        if ((error_type = tty_read_section(PortFD, bufferC, ';', DAENET_TIMEOUT, &nbytes_read))!= TTY_OK)
        {
            //DEBUGF(INDI::Logger::DBG_ERROR, "TTY error detected PortC: %d", error_type);
            return false;
        }
        tcflush(PortFD, TCIOFLUSH);
        if (nbytes_read < 0)
            return false;
        bufferC[nbytes_read-1] = '\0';

        // ------------- Update Port C -----------------
        if(sscanf(bufferC, "00CD%*c=%f", &analogValue))
        {
            DEBUGF(INDI::Logger::DBG_DEBUG, "AnalogValue : %d - %f", i, analogValue);
            AnalogReadingN[i].value = analogValue;
        }
    }
    AnalogReadingNP.s = IPS_OK;
    IDSetNumber(&AnalogReadingNP, nullptr);//client update

    return true;

}

bool DAEnetIP3::getReadingsDAEnet()
{
    //---------Reading PORT_A----------------
    getReadPortA();
    //usleep(1000000);
    //---------Reading PORT_B----------------
    getReadPortB();
    //---------Reading PORT_C----------------
    getReadPortC();

	//update file Property in /tmp/indi_daenetip3
	salto +=1;	//TODO: usare SetTimer
	if (salto >= 6)
	{
	std::ofstream fout("/tmp/indi_daenetip3.txt");
	for (int i = 0 ; i < D_INP_N ; i++)
		fout<<DigitalReadingL[i].label << "=" << DigitalReadingL[i].s << "\n";
	for(int i=0; i<A_INP_N; i++)
		fout<<AnalogReadingN[i].label <<  "=" << AnalogReadingN[i].value << "\n";
	fout.close();
	salto=0;
	}
    return true;
}

int DAEnetIP3::setDAEnetPortA(uint16_t port_A)
{
    int error_type;
    int nbytes_write = 0;
    char cmdA[CMD_SIZE_A];

    snprintf(cmdA, CMD_SIZE_A, "00ASG=%04X;", port_A);
    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD: %s ", cmdA);
    //DEBUGFDEVICE(getDefaultName(), DBG_DEBUG, "CMD (%s)", cmd);
    if ((error_type = tty_write_string(PortFD, cmdA, &nbytes_write)) != TTY_OK)
        return error_type;
    old_port_A = port_A;
    DEBUG(INDI::Logger::DBG_SESSION,"Set DAEnetIP3 PortA.");

    return 0;

}

bool DAEnetIP3::getLabelPort(char port, int n_label, char *l_label)
{
    char buffer[MAX_BUF_LABEL]={0};
    char label[MAX_LABEL]={0};
    char cmd[CMD_SIZE_A];
    int error_type, nbytes_write = 0, nbytes_read = 0;

    tcflush(PortFD, TCIOFLUSH);
    snprintf(cmd, CMD_SIZE_A, "00%cC%01X=?;", port, n_label);
    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD: %s", cmd);
    //DEBUGFDEVICE(getDefaultName(), DBG_DEBUG, "CMD (%s)", cmdB);
    if ((error_type = tty_write_string(PortFD, cmd, &nbytes_write)) != TTY_OK)
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "Error getting device readings, error type: %d", error_type);
        return error_type;
    }

    if ((error_type = tty_read_section(PortFD, buffer, ';', DAENET_TIMEOUT, &nbytes_read)!= TTY_OK))    // tty_read(PortFD, bufferB, 12, DAENET_TIMEOUT, &nbytes_read))!= TTY_OK)
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "TTY error detected: %d", error_type);
        return false;
    }
    if (nbytes_read < 0)
        return false;
    buffer[nbytes_read-1] = '\0';

    // ------------- Update Label -----------------
    if(sscanf(buffer, "00%*cC%*c=%s", label))
    {
        DEBUGF(INDI::Logger::DBG_DEBUG, "Label : %d - %s", n_label, label);
        strcat (l_label, label);
    }

    return true;

}

bool DAEnetIP3::getReadUnitLabelPortC(int n_label, char *l_label)
{
    char label[MAX_LABEL]={0};
    int error_type, nbytes_write = 0, nbytes_read = 0;
    char bufferC[BUF_SIZE_C]={0};
    char cmdC[CMD_SIZE_A];

    sprintf(cmdC, "00CS%01d=?;", n_label);
    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD_C: %s", cmdC);
    //DEBUGFDEVICE(getDefaultName(), DBG_DEBUG, "CMD (%s)", cmdC);
    tcflush(PortFD, TCIOFLUSH);
    if ((error_type = tty_write_string(PortFD, cmdC, &nbytes_write)) != TTY_OK)
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "Error getting device readings, error type: %d", error_type);
        return false;
    }

    if ((error_type = tty_read_section(PortFD, bufferC, ';', DAENET_TIMEOUT, &nbytes_read))!= TTY_OK)
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "TTY error detected PortC: %d", error_type);
        return false;
    }
    tcflush(PortFD, TCIOFLUSH);
    if (nbytes_read < 0)
        return false;
    bufferC[nbytes_read-1] = '\0';

    // ------------- Update Label Unit -----------------
    if(sscanf(bufferC, "00CS%*c=%s", label))
    {
        DEBUGF(INDI::Logger::DBG_DEBUG, "Label : %d - %s", n_label, label);
        sprintf(bufferC, "  [%s]", label);
        strcat (l_label, bufferC);
    }

    return true;

}

