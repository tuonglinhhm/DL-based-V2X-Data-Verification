//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//
// This file is an extension of SimuLTE
// Author: Brian McCarthy
// email : b.mccarthy@cs.ucc.ie

#include "stack/phy/layer/LtePhyUeD2D.h"
#include "stack/phy/packet/SidelinkControlInformation_m.h"
#include "stack/mac/packet/LteSchedulingGrant.h"
#include "stack/mac/allocator/LteAllocationModule.h"
#include "stack/phy/layer/Subchannel.h"
#include <unordered_map>

class LtePhyVUeMode4 : public LtePhyUeD2D
{
  protected:

    // D2D Tx Power
    double d2dTxPower_;

    bool adjacencyPSCCHPSSCH_;
    int pStep_;
    int numSubchannels_;
    int subchannelSize_ ;
    int selectionWindowStartingSubframe_;
    int thresholdRSSI_;

    bool transmitting_;

    std::map<MacNodeId, simtime_t> previousTransmissionTimes_;

    std::vector<int> ThresPSSCHRSRPvector_;

    std::vector<LteAirFrame*> tbFrames_; // airframes received in the current TTI. Only one will be decoded
    cMessage* d2dDecodingTimer_; // timer for triggering decoding at the end of the TTI. Started when the first airframe is received

    std::vector<std::vector<double>> tbRsrpVectors_;
    std::vector<std::vector<double>> tbRssiVectors_;

    std::vector<std::vector<Subchannel*>> sensingWindow_;
    int sensingWindowFront_;
    LteMode4SchedulingGrant* sciGrant_;
    std::vector<std::vector<double>> sciRsrpVectors_;
    std::vector<std::vector<double>> sciRssiVectors_;
    std::vector<LteAirFrame*> sciFrames_;
    std::vector<cPacket*> scis_;

    simsignal_t cbr;
    simsignal_t sciReceived;
    simsignal_t sciDecoded;
    simsignal_t sciNotDecoded;
    simsignal_t sciSent;
    simsignal_t tbSent;
    simsignal_t tbReceived;
    simsignal_t tbDecoded;
    simsignal_t tbFailedDueToNoSCI;
    simsignal_t tbFailedButSCIReceived;
    simsignal_t tbAndSCINotReceived;
    simsignal_t sciFailedHalfDuplex;
    simsignal_t tbFailedHalfDuplex;
    simsignal_t threshold;
    simsignal_t txRxDistanceSCI;
    simsignal_t txRxDistanceTB;
    simsignal_t subchannelReceived;
    simsignal_t subchannelsUsed;
    simsignal_t senderID;
    simsignal_t subchannelSent;
    simsignal_t subchannelsUsedToSend;
    simsignal_t interPacketDelay;
    simsignal_t posX;
    simsignal_t posY;

    int sciReceived_;
    int sciDecoded_;
    int sciNotDecoded_;
    int sciFailedHalfDuplex_;
    int tbReceived_;
    int tbDecoded_;
    int tbFailedDueToNoSCI_;
    int tbFailedButSCIReceived_;
    int tbAndSCINotReceived_;
    int tbFailedHalfDuplex_;
    int subchannelReceived_;
    int subchannelsUsed_;

    RbMap availableRBs_;

    LteAllocationModule* allocator_;

    void storeAirFrame(LteAirFrame* newFrame);
    LteAirFrame* extractAirFrame();
    void decodeAirFrame(LteAirFrame* frame, UserControlInfo* lteInfo, std::vector<double> &rsrpVector, std::vector<double> &rssiVector);
    // ---------------------------------------------------------------- //

    virtual void initialize(int stage);
    virtual void finish();
    virtual void handleAirFrame(cMessage* msg);
    virtual void handleUpperMessage(cMessage* msg);
    virtual void handleSelfMessage(cMessage *msg);

    // Helper function which prepares a frame for sending
    virtual LteAirFrame* prepareAirFrame(cMessage* msg, UserControlInfo* lteInfo);

    // Generate an SCI message corresponding to a Grant
    virtual SidelinkControlInformation* createSCIMessage();

    // Compute Candidate Single Subframe Resources which the MAC layer can use for transmission
    virtual void computeCSRs(LteMode4SchedulingGrant* &grant);

    virtual void updateSubframe();

    virtual std::vector<std::tuple<double, int, int>> selectBestRSSIs(std::unordered_map<int, std::set<int>> possibleCSRs, LteMode4SchedulingGrant* &grant, int totalPossibleCSRs);

    virtual std::tuple<int,int> decodeRivValue(SidelinkControlInformation* sci, UserControlInfo* sciInfo);

    virtual void updateCBR();

    virtual RbMap sendSciMessage(cMessage* sci, UserControlInfo* lteInfo);

    virtual void initialiseSensingWindow();

    virtual int translateIndex(int index);

  public:
    LtePhyVUeMode4();
    virtual ~LtePhyVUeMode4();

    virtual double getTxPwr(Direction dir = UNKNOWN_DIRECTION)
    {
        if (dir == D2D)
            return d2dTxPower_;
        return txPower_;
    }
};

