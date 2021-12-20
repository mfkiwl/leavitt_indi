/*******************************************************************************
  INDI DAEnetIP3 Relay I/O Module Driver
  Copyright (C) 2019 Emanuele Pavoni

  This program is free software; you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free
  Software Foundation; either version 2 of the License, or (at your option)
  any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; see the file COPYING.LIB.  If not, write to
  the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
  Boston, MA 02110-1301, USA.

  The full GNU General Public License is included in this distribution in the
  file called LICENSE.
*******************************************************************************/

#pragma once

#include "defaultdevice.h"

class DAEnetIP3 : public INDI::DefaultDevice
{
  public:
    DAEnetIP3();
    virtual ~DAEnetIP3() = default;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n);
    void DAEConnection(const uint8_t &value);//TODO: da spostare

    /**
     * @struct DAEnetConnection
     * @brief Holds the connection mode of the device.
     */
    enum
    {
        CONNECTION_NONE   = 1 << 0, /** Do not use any connection plugin */
        CONNECTION_SERIAL = 1 << 1, /** For regular serial connections */
        CONNECTION_TCP    = 1 << 2  /** For TCP connections */
    }  DaenetConnection;

  protected:
    const char *getDefaultName();
    bool initProperties();
    bool updateProperties();
    bool Handshake();
    void TimerHit();
    int setDAEnetPortA(uint16_t port_A);

 private:
    // Settings
    ISwitchVectorProperty DigitalSettingSP;
    ISwitch DigitalSettingS[16];
    // Readings
    ILightVectorProperty DigitalReadingLP;
    ILight DigitalReadingL[8];
    INumberVectorProperty AnalogReadingNP;
    INumber AnalogReadingN[8];

    Connection::Serial *serialConnection { nullptr };
    Connection::TCP *tcpConnection { nullptr };
    /// For Serial & TCP connections

    int PortFD { -1 };
    uint16_t old_port_A { 0 };
    uint16_t old_port_B { 0 };
    //char cmd_A[CMD_SIZE_A];
    uint8_t DAEnetConnection { CONNECTION_SERIAL | CONNECTION_TCP };
    bool callHandshake();
    bool getReadingsDAEnet();
    bool getReadPortA();
    bool getReadPortB();
    bool getReadPortC();
    bool getLabelPort(char port, int n_label, char *l_label);
    bool getReadUnitLabelPortC(int n_label, char *l_label);
};

