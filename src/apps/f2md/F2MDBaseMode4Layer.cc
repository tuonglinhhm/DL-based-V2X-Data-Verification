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

#include "F2MDBaseMode4Layer.h"

#include "common/LteControlInfo.h"
#include "stack/phy/packet/cbr_m.h"

Define_Module(F2MDBaseMode4Layer);

void F2MDBaseMode4Layer::initialize(int stage)
{
    Mode4BaseApp::initialize(stage);
    if (stage==inet::INITSTAGE_LOCAL){
        // initialize pointers to other modules
        if (FindModule<VeinsInetMobility*>::findSubModule(getParentModule())) {
            mobility = FindModule<VeinsInetMobility*>::findSubModule(getParentModule());
            ASSERT(mobility);
            VeinsInetManager * manager = FindModule<VeinsInetManager*>::findSubModule(getParentModule()->getParentModule());
            traci = manager->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();
        }
        else {
            traci = nullptr;
            mobility = nullptr;
            traciVehicle = nullptr;
        }

        FindModule<>::findHost(this)->subscribe(VeinsInetMobility::mobilityStateChangedSignal, this);

        // Register the node with the binder
        // Issue primarily is how do we set the link layer address

        // Get the binder
        binder_ = getBinder();

        // Get our UE
        cModule *ue = getParentModule();

        //Register with the binder
        nodeId_ = binder_->registerNode(ue, UE, 0);

        // Register the nodeId_ with the binder.
        binder_->setMacNodeId(nodeId_, nodeId_);

        // store MAC address for quick access
        myId = nodeId_;

    } else if (stage==inet::INITSTAGE_APPLICATION_LAYER) {

        sendBeaconEvt = NULL;
        sendAlertEvt = NULL;

        // read parameters
        headerLength = par("headerLength");
        sendBeacons = par("sendBeacons").boolValue();
        beaconLengthBits = par("beaconLengthBits");
        beaconUserPriority = par("beaconUserPriority");
        beaconInterval = par("beaconInterval");

        sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
        sendAlertEvt = new cMessage("alert evt", SEND_ALERT_EVT);

        nextSno_ = 0;
        sendAlerts = par("sendAlerts").boolValue();
        size_ = par("packetSize");
        period_ = par("period");
        priority_ = par("priority");
        duration_ = par("duration");

        sentMsg_ = registerSignal("sentMsg");
        delay_ = registerSignal("delay");
        rcvdMsg_ = registerSignal("rcvdMsg");
        cbr_ = registerSignal("cbr");

        if(sendAlerts){
            double delay = 0.001 * intuniform(0, 1000, 0);
            scheduleAt((simTime() + delay).trunc(SIMTIME_MS), sendAlertEvt);
        }

        if(sendBeacons){
            double delay = 0.001 * intuniform(0, 1000, 0);
            scheduleAt((simTime() + delay).trunc(SIMTIME_MS), sendBeaconEvt);
        }

    }
}

void F2MDBaseMode4Layer::handleLowerMessage(cMessage* msg)
{
    if (msg->isName("CBR")) {
        Cbr* cbrPkt = check_and_cast<Cbr*>(msg);
        double channel_load = cbrPkt->getCbr();
        delete cbrPkt;
    } else {
        if(BasicSafetyMessage* bsm = dynamic_cast<BasicSafetyMessage*>(msg)){
            onBSM(bsm);
        }else if(AlertPacket* pkt = check_and_cast<AlertPacket*>(msg)){
            if (pkt == 0)
                throw cRuntimeError("F2MDBaseMode4Layer::handleMessage - FATAL! Error when casting to AlertPacket");
            // emit statistics
            simtime_t delay = simTime() - pkt->getTimestamp();
            emit(delay_, delay);
            emit(rcvdMsg_, (long)1);

            EV << "F2MDBaseMode4Layer::handleMessage - Packet received: SeqNo[" << pkt->getSno() << "] Delay[" << delay << "]" << endl;

            delete msg;
        }

    }
}

void F2MDBaseMode4Layer::handleSelfMessage(cMessage* msg)
{
    switch (msg->getKind()) {
    case SEND_BEACON_EVT: {
        BasicSafetyMessage* bsm = new BasicSafetyMessage();
        populateBSM(bsm);

        auto lteControlInfo = new FlowControlInfoNonIp();

        lteControlInfo->setSrcAddr(nodeId_);
        lteControlInfo->setDirection(D2D_MULTI);
        lteControlInfo->setPriority(priority_);
        lteControlInfo->setDuration(duration_);
        lteControlInfo->setCreationTime(simTime());

        bsm->setControlInfo(lteControlInfo);

        Mode4BaseApp::sendLowerPackets(bsm);
        emit(sentMsg_, (long)1);

        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        break;
    }
    case SEND_ALERT_EVT: {
        // Replace method
        AlertPacket* packet = new AlertPacket("Alert");
        packet->setTimestamp(simTime());
        packet->setByteLength(size_);
        packet->setSno(nextSno_);

        nextSno_++;

        auto lteControlInfo = new FlowControlInfoNonIp();

        lteControlInfo->setSrcAddr(nodeId_);
        lteControlInfo->setDirection(D2D_MULTI);
        lteControlInfo->setPriority(priority_);
        lteControlInfo->setDuration(duration_);
        lteControlInfo->setCreationTime(simTime());

        packet->setControlInfo(lteControlInfo);

        Mode4BaseApp::sendLowerPackets(packet);
        emit(sentMsg_, (long)1);

        scheduleAt(simTime() + period_, sendAlertEvt);
        break;
    }
    default: {
        throw cRuntimeError("F2MDBaseMode4Layer::handleMessage - Unrecognized self message");
        break;
    }
    }


}

void F2MDBaseMode4Layer::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
    Enter_Method_Silent();
    if (signalID == VeinsInetMobility::mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
}

void F2MDBaseMode4Layer::handlePositionUpdate(cObject* obj)
{
    curPosition = veins::Coord(mobility->getCurrentPosition().x,mobility->getCurrentPosition().y,mobility->getCurrentPosition().z);
    curSpeed = veins::Coord(mobility->getCurrentSpeed().x,mobility->getCurrentSpeed().y,mobility->getCurrentSpeed().z);
    curAccel = veins::Coord(mobility->getCurrentAcceleration().x,mobility->getCurrentAcceleration().y,mobility->getCurrentAcceleration().z);
    curHeading = veins::Coord(cos(-mobility->getCurrentAngularPosition().alpha), -sin(-mobility->getCurrentAngularPosition().alpha));
}

void F2MDBaseMode4Layer::populateBSM(BasicSafetyMessage* bsm)
{
    bsm->setRecipientAddress(veins::LAddress::L2BROADCAST());
    bsm->setBitLength(headerLength);

    bsm->setSenderPos(curPosition);
    bsm->setSenderSpeed(curSpeed);
    bsm->setSenderHeading(curHeading);
    bsm->setSenderAccel(curAccel);
    bsm->setPsid(-1);
    bsm->addBitLength(beaconLengthBits);
    bsm->setUserPriority(beaconUserPriority);
}


void F2MDBaseMode4Layer::finish()
{
    cancelAndDelete(sendAlertEvt);
}

F2MDBaseMode4Layer::~F2MDBaseMode4Layer()
{
    binder_->unregisterNode(nodeId_);
}
