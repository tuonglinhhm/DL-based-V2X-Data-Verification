//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

/**
 * F2MDBaseMode4Layer is a new application developed to be used with Mode 4 based simulation
 * Author: Brian McCarthy
 * Email: b.mccarthy@cs.ucc.ie
 */

#ifndef _LTE_F2MDBaseMode4Layer_H_
#define _LTE_F2MDBaseMode4Layer_H_

#include "apps/mode4App/Mode4BaseApp.h"
#include "apps/alert/AlertPacket_m.h"
#include "corenetwork/binder/LteBinder.h"

#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/base/utils/FindModule.h"

#include "veins_inet/VeinsInetMobility.h"
#include "veins_inet/VeinsInetManager.h"

#include "veins/modules/application/f2md/mdMessages/BasicSafetyMessage_m.h"

using veins::AnnotationManager;
using veins::AnnotationManagerAccess;
using veins::TraCICommandInterface;
using veins::TraCIMobility;
using veins::TraCIMobilityAccess;
using veins::FindModule;

using veins::VeinsInetMobility;
using veins::VeinsInetManager;

using veins::BasicSafetyMessage;

class F2MDBaseMode4Layer : public Mode4BaseApp, public cListener {

public:
    ~F2MDBaseMode4Layer() override;

    void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) override;

    void initialize(int stage);
    void finish();

    enum DemoApplMessageKinds {
        SEND_ALERT_EVT,
        SEND_BEACON_EVT
    };

protected:

   int numInitStages() const { return inet::NUM_INIT_STAGES; }

   void handleLowerMessage(cMessage* msg);

   /**
    * Main loop of the Mac level, calls the scheduler
    * and every other function every TTI : must be reimplemented
    * by derivate classes
    */
   void handleSelfMessage(cMessage* msg);

   /**
    * sendLowerPackets() is used
    * to send packets to lower layer
    *
    * @param pkt Packet to send
    */
   void sendLowerPackets(cPacket* pkt);

   /** @brief this function is called every time the vehicle receives a position update signal */
   virtual void handlePositionUpdate(cObject* obj);
   /** @brief this function is called upon receiving a DemoSafetyMessage, also referred to as a beacon  */
   virtual void onBSM(BasicSafetyMessage* bsm){};
   /** @brief sets all the necessary fields in the WSM, BSM, or WSA. */
   virtual void populateBSM(BasicSafetyMessage* bsm);

protected:
    //sender
    int size_;
    int nextSno_;
    int priority_;
    int duration_;
    simtime_t period_;

    simsignal_t sentMsg_;
    simsignal_t delay_;
    simsignal_t rcvdMsg_;
    simsignal_t cbr_;

    /* BSM (beacon) settings */
    uint32_t beaconLengthBits;
    uint32_t beaconUserPriority;
    simtime_t beaconInterval;
    bool sendBeacons;

    cMessage* sendBeaconEvt;
    cMessage* sendAlertEvt;
    bool sendAlerts;

    LteBinder* binder_;
    MacNodeId nodeId_;

protected:
    /* pointers ill be set when used with TraCIMobility */
    VeinsInetMobility* mobility;
    TraCICommandInterface* traci;
    TraCICommandInterface::Vehicle* traciVehicle;

    veins::Coord curPosition;
    veins::Coord curSpeed;
    veins::Coord curHeading;
    veins::Coord curAccel;

    veins::LAddress::L2Type myId = 0;



};

#endif
