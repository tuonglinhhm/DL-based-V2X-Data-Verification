/*******************************************************************************
 * @author  Joseph Kamel
 * @email   josephekamel@gmail.com
 * @date    28/11/2018
 * @version 2.0
 *
 * SCA (Secure Cooperative Autonomous systems)
 * Copyright (c) 2013, 2018 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#include "F2MDVeinsMode4.h"

Define_Module(JosephVeinsMode4);

void JosephVeinsMode4::initialize(int stage) {

    F2MDBaseMode4Layer::initialize(stage);

    if (!linkInit) {

        params.serialNumber = par("serialNumber").stdstringValue();
        params.savePath = par("savePath").stdstringValue();

        params.veremiConf = par("veremiConf");
        params.randomConf = par("randomConf");
        params.variableConf = par("variableConf");
        params.confPos = par("confPos");
        params.confSpd = par("confSpd");
        params.confHea = par("confHea");
        params.confAcc = par("confAcc");
        params.confPrec = par("confPrec");
        params.minConf = par("minConf");

        params.SAVE_PERIOD = par("SAVE_PERIOD");
        params.PRINT_PERIOD = par("PRINT_PERIOD");

        params.START_SAVE = par("START_SAVE");
        params.START_ATTACK = par("START_ATTACK");

        params.START_SAVE = par("START_SAVE");
        params.START_ATTACK = par("START_ATTACK");

        params.REPORT_VERSION =
                reportTypes::intReport[par("REPORT_VERSION").intValue()];

        params.UseAttacksServer = par("UseAttacksServer");
        params.MixLocalAttacks = par("MixLocalAttacks");
        params.RandomLocalMix = par("RandomLocalMix");

        params.LOCAL_ATTACKER_PROB = par("LOCAL_ATTACKER_PROB");
        params.LOCAL_ATTACK_TYPE = attackTypes::intAttacks[par(
                "LOCAL_ATTACK_TYPE").intValue()];

        params.GLOBAL_ATTACKER_PROB = par("GLOBAL_ATTACKER_PROB");
        params.GLOBAL_ATTACK_TYPE = attackTypes::intAttacks[par(
                "GLOBAL_ATTACK_TYPE")];

        params.EnablePC = par("EnablePC");
        params.PC_TYPE =
                pseudoChangeTypes::intPseudoChange[par("PC_TYPE").intValue()];

        params.EnableV1 = par("EnableV1");
        params.EnableV2 = par("EnableV2");
        params.SaveStatsV1 = par("SaveStatsV1");
        params.SaveStatsV2 = par("SaveStatsV2");

        params.checksVersionV1 = mdChecksVersionTypes::intChecksVersion[par(
                "checksVersionV1").intValue()];
        params.checksVersionV2 = mdChecksVersionTypes::intChecksVersion[par(
                "checksVersionV2").intValue()];

        params.appTypeV1 = mdAppTypes::intApp[par("appTypeV1").intValue()];
        params.appTypeV2 = mdAppTypes::intApp[par("appTypeV2").intValue()];

        params.writeSelfMsg = par("writeSelfMsg");
        params.writeListSelfMsg = par("writeListSelfMsg");

        params.writeBsmsV1 = par("writeBsmsV1");
        params.writeBsmsV2 = par("writeBsmsV2");
        params.writeListBsmsV1 = par("writeListBsmsV1");
        params.writeListBsmsV2 = par("writeListBsmsV2");

        params.writeReportsV1 = par("writeReportsV1");
        params.writeReportsV2 = par("writeReportsV2");
        params.writeListReportsV1 = par("writeListReportsV1");
        params.writeListReportsV2 = par("writeListReportsV2");

        params.writeVeReMi = par("writeVeReMi");
        params.VeReMiSliceTime = par("VeReMiSliceTime");

        params.sendReportsV1 = par("sendReportsV1");
        params.sendReportsV2 = par("sendReportsV2");

        params.maHostV1 = par("maHostV1").stdstringValue();
        params.maHostV2 = par("maHostV2").stdstringValue();
        params.maPortV1 = par("maPortV1");
        params.maPortV2 = par("maPortV2");
        params.enableVarThreV1 = par("enableVarThreV1");
        params.enableVarThreV2 = par("enableVarThreV2");
        //Simulation Parameters

        // ------ Detection Parameters -- Start
        params.MAX_PROXIMITY_RANGE_L = par("MAX_PROXIMITY_RANGE_L");
        params.MAX_PROXIMITY_RANGE_W = par("MAX_PROXIMITY_RANGE_W");
        params.MAX_PROXIMITY_DISTANCE = par("MAX_PROXIMITY_DISTANCE");

        params.MAX_CONFIDENCE_RANGE = par("MAX_CONFIDENCE_RANGE");
        params.MAX_PLAUSIBLE_RANGE = par("MAX_PLAUSIBLE_RANGE");
        params.MAX_TIME_DELTA = par("MAX_TIME_DELTA");
        params.MAX_DELTA_INTER = par("MAX_DELTA_INTER");
        params.MAX_SA_RANGE = par("MAX_SA_RANGE");
        params.MAX_SA_TIME = par("MAX_SA_TIME");
        params.MAX_KALMAN_TIME = par("MAX_KALMAN_TIME");
        params.KALMAN_POS_RANGE = par("KALMAN_POS_RANGE");
        params.KALMAN_SPEED_RANGE = par("KALMAN_SPEED_RANGE");
        params.KALMAN_MIN_POS_RANGE = par("KALMAN_MIN_POS_RANGE");
        params.KALMAN_MIN_SPEED_RANGE = par("KALMAN_MIN_SPEED_RANGE");
        params.MIN_MAX_SPEED = par("MIN_MAX_SPEED");
        params.MIN_MAX_ACCEL = par("MIN_MAX_ACCEL");
        params.MIN_MAX_DECEL = par("MIN_MAX_DECEL");
        params.MAX_MGT_RNG = par("MAX_MGT_RNG");
        params.MAX_MGT_RNG_DOWN = par("MAX_MGT_RNG_DOWN");
        params.MAX_MGT_RNG_UP = par("MAX_MGT_RNG_UP");
        params.MAX_BEACON_FREQUENCY = par("MAX_BEACON_FREQUENCY");
        params.MAX_DISTANCE_FROM_ROUTE = par("MAX_DISTANCE_FROM_ROUTE");
        params.MAX_NON_ROUTE_SPEED = par("MAX_NON_ROUTE_SPEED");
        params.MAX_HEADING_CHANGE = par("MAX_HEADING_CHANGE");
        params.DELTA_BSM_TIME = par("DELTA_BSM_TIME");
        params.DELTA_REPORT_TIME = par("DELTA_REPORT_TIME");
        params.POS_HEADING_TIME = par("POS_HEADING_TIME");
        // ------ Detection Parameters -- End

        // ------ Storage Parameters -- Start
        params.MAX_TARGET_TIME = par("MAX_TARGET_TIME");
        params.MAX_ACCUSED_TIME = par("MAX_ACCUSED_TIME");
        // ------ Storage Parameters -- End

        // ------ Attacks Parameters -- Start
        params.parVar = par("parVar");
        params.RandomPosOffsetX = par("RandomPosOffsetX");
        params.RandomPosOffsetY = par("RandomPosOffsetY");
        params.RandomSpeedX = par("RandomSpeedX");
        params.RandomSpeedY = par("RandomSpeedY");
        params.RandomSpeedOffsetX = par("RandomSpeedOffsetX");
        params.RandomSpeedOffsetY = par("RandomSpeedOffsetY");
        params.RandomAccelX = par("RandomAccelX");
        params.RandomAccelY = par("RandomAccelY");
        params.StopProb = par("StopProb");
        params.StaleMessages_Buffer = par("StaleMessages_Buffer");
        params.DosMultipleFreq = par("DosMultipleFreq");
        params.DosMultipleFreqSybil = par("DosMultipleFreqSybil");
        params.ReplaySeqNum = par("ReplaySeqNum");
        params.SybilVehNumber = par("SybilVehNumber");
        params.SelfSybil = par("SelfSybil");
        params.SybilDistanceX = par("SybilDistanceX");
        params.SybilDistanceY = par("SybilDistanceY");
        // ------ Attacks Parameters -- End

        // ------ Pseudonym Parameters -- Start
        params.Period_Change_Time = par("Period_Change_Time");
        params.Tolerance_Buffer = par("Tolerance_Buffer");
        params.Period_Change_Distance = par("Period_Change_Distance");
        params.Random_Change_Chance = par("Random_Change_Chance");
        // ------ Pseudonym Parameters -- End

        //------ Report Parameters -- Start
        params.InitialHistory = par("InitialHistory");
        params.CollectionPeriod = par("CollectionPeriod");
        params.UntolerancePeriod = par("UntolerancePeriod");
        //------ Report Parameters -- End

        linkControl.initialize(&params, traci);

        linkInit = true;
    }

    if (stage == 0) {
        // F2MD init
        curPositionConfidenceOrig = veins::Coord(0, 0, 0);
        curSpeedConfidenceOrig = veins::Coord(0, 0, 0);
        curHeadingConfidenceOrig = veins::Coord(0, 0, 0);
        curAccelConfidenceOrig = veins::Coord(0, 0, 0);

        curPositionConfidence = veins::Coord(0, 0, 0);
        curSpeedConfidence = veins::Coord(0, 0, 0);
        curHeadingConfidence = veins::Coord(0, 0, 0);
        curAccelConfidence = veins::Coord(0, 0, 0);
        myWidth = 0;
        myLength = 0;
        curHeading = veins::Coord(0, 0, 0);
        curAccel = veins::Coord(0, 0, 0);
        myMdType = mbTypes::Genuine;
        myAttackType = attackTypes::Attacks::Genuine;
        //attackBsm.setSenderAddress(0);
        //nextAttackBsm.setSenderAddress(0);

        reportProtocolEnforcerV1.setParams(&params);
        reportProtocolEnforcerV2.setParams(&params);

        //mkdir savePath
        struct stat info;
        if ((stat(params.savePath.c_str(), &info) != 0) || !(info.st_mode & S_IFDIR)) {
            mkdir(params.savePath.c_str(), 0777);
        }

        // F2MD init

    } else if (stage == 1) {
        EV << "Initializing " << par("appName").stringValue() << std::endl;

        myVType = traciVehicle->getVType();

        MaxRandomPosX = mobility->getConstraintAreaMax().x;
        MaxRandomPosY = mobility->getConstraintAreaMax().y;

        setMDApp(params.appTypeV1, params.appTypeV2);

        MAX_PLAUSIBLE_ACCEL = traciVehicle->getAccel()/0.9;
        MAX_PLAUSIBLE_DECEL = traciVehicle->getDeccel()/0.9;
        MAX_PLAUSIBLE_SPEED = traciVehicle->getMaxSpeed()/0.9;

        myWidth = traciVehicle->getWidth();
        myLength = traciVehicle->getLength();

        myMdType = induceMisbehavior(params.LOCAL_ATTACKER_PROB,
                params.GLOBAL_ATTACKER_PROB);

        if (params.UseAttacksServer) {
            localAttackServer.pingAttackServer();

            if (localAttackServer.isQueuedAttack()) {
                if (localAttackServer.isInstantAttack()) {
                    if (myMdType == mbTypes::Genuine) {
                        myMdType = mbTypes::LocalAttacker;
                    }
                }
            } else {
                if (myMdType == mbTypes::LocalAttacker) {
                    myMdType = mbTypes::Genuine;
                }
            }
        }

        //pseudonym-------------------------------
        myPcType = params.PC_TYPE;
        pseudoNum = 0;

        pcPolicy = PCPolicy(veins::Coord(mobility->getCurrentPosition().x,mobility->getCurrentPosition().y,mobility->getCurrentPosition().z), &params);

        pcPolicy.setMbType(myMdType);
        pcPolicy.setMdAuthority(&mdStats);
        pcPolicy.setCurPosition(&curPosition);
        pcPolicy.setMyId(&myId);
        pcPolicy.setMyPseudonym(&myPseudonym);
        pcPolicy.setPseudoNum(&pseudoNum);

        myPseudonym = pcPolicy.getNextPseudonym();

        //pseudonym-------------------------------

        if (params.randomConf) {
            double randConfPos = genLib.RandomDouble(
                    params.confPos * params.minConf, params.confPos);
            double randConfSpeed = genLib.RandomDouble(
                    params.confSpd * params.minConf, params.confSpd);
            double randConfHeading = genLib.RandomDouble(
                    params.confHea * params.minConf, params.confHea);
            double randConfAccel = genLib.RandomDouble(
                    params.confAcc * params.minConf, params.confAcc);

            if (params.variableConf) {
                ConfPosMax = veins::Coord(randConfPos * params.minConf,
                        randConfPos * params.minConf, 0);
                ConfSpeedMax = veins::Coord(randConfSpeed * params.minConf,
                        randConfSpeed * params.minConf, 0);
                ConfHeadMax = veins::Coord(randConfHeading * params.minConf,
                        randConfHeading * params.minConf, 0);
                ConfAccelMax = veins::Coord(randConfAccel * params.minConf,
                        randConfAccel * params.minConf, 0);
            } else {
                ConfPosMax = veins::Coord(0.0, 0.0, 0.0);
                ConfSpeedMax = veins::Coord(0.0, 0.0, 0.0);
                ConfHeadMax = veins::Coord(0.0, 0.0, 0.0);
                ConfAccelMax = veins::Coord(0.0, 0.0, 0.0);
            }

            randConfPos = ((int) (randConfPos * params.confPrec + .5)
                    / params.confPrec);
            randConfSpeed = ((int) (randConfSpeed * params.confPrec + .5)
                    / params.confPrec);
            randConfHeading = ((int) (randConfHeading * params.confPrec + .5)
                    / params.confPrec);
            randConfAccel = ((int) (randConfAccel * params.confPrec + .5)
                    / params.confPrec);

            curPositionConfidenceOrig = veins::Coord(randConfPos, randConfPos, 0);
            curSpeedConfidenceOrig = veins::Coord(randConfSpeed, randConfSpeed, 0);
            curHeadingConfidenceOrig = veins::Coord(randConfHeading, randConfHeading,
                    0);
            curAccelConfidenceOrig = veins::Coord(randConfAccel, randConfAccel, 0);

            curPositionConfidence = veins::Coord(randConfPos, randConfPos, 0);
            curSpeedConfidence = veins::Coord(randConfSpeed, randConfSpeed, 0);
            curHeadingConfidence = veins::Coord(randConfHeading, randConfHeading, 0);
            curAccelConfidence = veins::Coord(randConfAccel, randConfAccel, 0);

        } else if (params.veremiConf) {
            double posE0 = genLib.RandomDouble(3, 5);
            double speedE0 = genLib.GaussianRandomDouble(0, 0.0016);
            double headingE0 = genLib.RandomDouble(0, 20);

            curPositionConfidenceOrig = veins::Coord(posE0, posE0, 0);
            curSpeedConfidenceOrig = veins::Coord(speedE0, speedE0, 0);
            curHeadingConfidenceOrig = veins::Coord(headingE0, headingE0, 0);
            curAccelConfidenceOrig = veins::Coord(0, 0, 0);

            curPositionConfidence = veins::Coord(posE0, posE0, 0);
            curSpeedConfidence = veins::Coord(speedE0, speedE0, 0);
            curHeadingConfidence = veins::Coord(headingE0, headingE0, 0);
            curAccelConfidence = veins::Coord(0, 0, 0);

            lastPositionUpdate = simTime().dbl() - 1.0;
        } else {
            if (params.variableConf) {
                ConfPosMax = veins::Coord(params.confPos * params.minConf,
                        params.confPos * params.minConf, 0);
                ConfSpeedMax = veins::Coord(params.confSpd * params.minConf,
                        params.confSpd * params.minConf, 0);
                ConfHeadMax = veins::Coord(params.confHea * params.minConf,
                        params.confHea * params.minConf, 0);
                ConfAccelMax = veins::Coord(params.confAcc * params.minConf,
                        params.confAcc * params.minConf, 0);
            } else {
                ConfPosMax = veins::Coord(0.0, 0.0, 0.0);
                ConfSpeedMax = veins::Coord(0.0, 0.0, 0.0);
                ConfHeadMax = veins::Coord(0.0, 0.0, 0.0);
                ConfAccelMax = veins::Coord(0.0, 0.0, 0.0);
            }

            double ConfPosR = ((int) (params.confPos * params.confPrec + .5)
                    / params.confPrec);
            double ConfSpeedR = ((int) (params.confSpd * params.confPrec + .5)
                    / params.confPrec);
            double ConfHeadingR = ((int) (params.confHea * params.confPrec + .5)
                    / params.confPrec);
            double ConfAccelR = ((int) (params.confAcc * params.confPrec + .5)
                    / params.confPrec);

            curPositionConfidenceOrig = veins::Coord(ConfPosR, ConfPosR, 0);
            curSpeedConfidenceOrig = veins::Coord(ConfSpeedR, ConfSpeedR, 0);
            curHeadingConfidenceOrig = veins::Coord(ConfHeadingR, ConfHeadingR, 0);
            curAccelConfidenceOrig = veins::Coord(ConfAccelR, ConfAccelR, 0);

            curPositionConfidence = veins::Coord(ConfPosR, ConfPosR, 0);
            curSpeedConfidence = veins::Coord(ConfSpeedR, ConfSpeedR, 0);
            curHeadingConfidence = veins::Coord(ConfHeadingR, ConfHeadingR, 0);
            curAccelConfidence = veins::Coord(ConfAccelR, ConfAccelR, 0);
        }

        myReportType = params.REPORT_VERSION;

        switch (myMdType) {
        case mbTypes::Genuine: {
            traciVehicle->setColor(TraCIColor(0, 255, 0, 255));
            myAttackType = attackTypes::Genuine;
        }
            break;
        case mbTypes::GlobalAttacker: {
            traciVehicle->setColor(TraCIColor(0, 255, 0, 255));
            myAttackType = params.GLOBAL_ATTACK_TYPE;

            mdGlobalAttack = MDGlobalAttack();

            mdGlobalAttack.setMyPseudonym(&myPseudonym);
            mdGlobalAttack.setCurHeading(&curHeading);
            mdGlobalAttack.setCurHeadingConfidence(&curHeadingConfidence);
            mdGlobalAttack.setCurPosition(&curPosition);
            mdGlobalAttack.setCurPositionConfidence(&curPositionConfidence);
            mdGlobalAttack.setCurSpeed(&curSpeed);
            mdGlobalAttack.setCurSpeedConfidence(&curSpeedConfidence);
            mdGlobalAttack.setCurAccel(&curAccel);
            mdGlobalAttack.setCurAccelConfidence(&curAccelConfidence);
            mdGlobalAttack.setTraci(traci);

            mdGlobalAttack.init(myAttackType);

        }
            break;
        case mbTypes::LocalAttacker: {
            //attack-------------------------------
            if (params.UseAttacksServer) {
                myAttackType = localAttackServer.getNextAttack();
            } else if (params.MixLocalAttacks) {
                int AtLiSize = sizeof(params.MixLocalAttacksList)
                        / sizeof(params.MixLocalAttacksList[0]);
                int localAttackIndex = 0;
                if (params.RandomLocalMix) {
                    localAttackIndex = genLib.RandomInt(0, AtLiSize - 1);
                } else {
                    if (LastLocalAttackIndex < (AtLiSize - 1)) {
                        localAttackIndex = LastLocalAttackIndex + 1;
                        LastLocalAttackIndex = localAttackIndex;
                    } else {
                        localAttackIndex = 0;
                        LastLocalAttackIndex = 0;
                    }
                }
                myAttackType = params.MixLocalAttacksList[localAttackIndex];
            } else {
                myAttackType = params.LOCAL_ATTACK_TYPE;
            }

            std::cout
                    << "=+#=+#=+#=+#=+#=+#=+#=+#+#=+#=+#=+#=+#=+#=+#=+#=+#=+#=+#=+#=+# "
                    << "\n";
            std::cout
                    << "=+#=+#=+#=+#=+#=+#=+#=+# NEW ATTACKER =+#=+#=+#=+#=+#=+#=+#=+# "
                    << myPseudonym << " : "
                    << attackTypes::AttackNames[myAttackType] << "\n";
            std::cout
                    << "=+#=+#=+#=+#=+#=+#=+#=+#+#=+#=+#=+#=+#=+#=+#=+#=+#=+#=+#=+#=+# "
                    << "\n";

            mdAttack = MDAttack();

            mdAttack.setBeaconInterval(&beaconInterval);
            mdAttack.setCurHeading(&curHeading);
            mdAttack.setCurHeadingConfidence(&curHeadingConfidence);
            mdAttack.setCurPosition(&curPosition);
            mdAttack.setCurPositionConfidence(&curPositionConfidence);
            mdAttack.setCurSpeed(&curSpeed);
            mdAttack.setCurSpeedConfidence(&curSpeedConfidence);
            mdAttack.setCurAccel(&curAccel);
            mdAttack.setCurAccelConfidence(&curAccelConfidence);
            mdAttack.setDetectedNodes(&detectedNodes);
            mdAttack.setMyBsm(myBsm);
            mdAttack.setMyBsmNum(&myBsmNum);
            mdAttack.setMyLength(&myLength);
            mdAttack.setMyPseudonym(&myPseudonym);
            mdAttack.setMyWidth(&myWidth);
            mdAttack.setPcPolicy(&pcPolicy);

            mdAttack.init(myAttackType, MaxRandomPosX, MaxRandomPosY, &params);

            //attack-------------------------------
            traciVehicle->setColor(TraCIColor(255, 0, 0, 255));

        }
            break;
        default:
            traciVehicle->setColor(TraCIColor(0, 0, 0, 0));
            break;
        }

        if (!setDate) {
            char dateBuffer[50];
            time_t t = time(NULL);
            struct tm tm = *localtime(&t);
            sprintf(dateBuffer, "%d-%d-%d_%d:%d:%d", tm.tm_year + 1900,
                    tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,
                    tm.tm_sec);
            std::string curDateTemp(dateBuffer);
            curDate = curDateTemp;
            setDate = true;
        }

        if (params.writeVeReMi) {
            VeReMi.initVeReMiPrintable(params.savePath, params.serialNumber,
                    myId, getParentModule()->getId(), myAttackType, curDate,
                    params.VeReMiSliceTime, VeReMiSliceStartTime,
                    simTime().dbl());
        }

    }
}

void JosephVeinsMode4::finish() {

    realDynamicMap.erase(myId);

    if (myReportType == reportTypes::ProtocolReport) {
        handleReportProtocol(true);
    }

    F2MDBaseMode4Layer::finish();

    //statistics recording goes here
}

void JosephVeinsMode4::setMDApp(mdAppTypes::App appTypeV1,
        mdAppTypes::App appTypeV2) {
    switch (appTypeV1) {
    case mdAppTypes::ThresholdApp:
        AppV1 = &ThreV1;
        break;
    case mdAppTypes::AggrigationApp:
        AppV1 = &AggrV1;
        break;
    case mdAppTypes::BehavioralApp:
        AppV1 = &BehaV1;
        break;
    case mdAppTypes::CooperativeApp:
        AppV1 = &CoopV1;
        break;
    case mdAppTypes::ExperiApp:
        AppV1 = &ExperV1;
        break;
    case mdAppTypes::MachineLearningApp:
        AppV1 = &PybgV1;
        PybgV1.setMyId(myId);
        break;
    default:
        AppV1 = &ThreV1;
        break;
    }
    switch (appTypeV2) {
    case mdAppTypes::ThresholdApp:
        AppV2 = &ThreV2;
        break;
    case mdAppTypes::AggrigationApp:
        AppV2 = &AggrV2;
        break;
    case mdAppTypes::BehavioralApp:
        AppV2 = &BehaV2;
        break;
    case mdAppTypes::CooperativeApp:
        AppV2 = &CoopV2;
        break;
    case mdAppTypes::ExperiApp:
        AppV2 = &ExperV2;
        break;
    case mdAppTypes::MachineLearningApp:
        AppV2 = &PybgV2;
        PybgV2.setMyId(myId);
        break;
    default:
        AppV2 = &ThreV2;
        break;
    }
}

static double totalGenuine = 0;
static double totalLocalAttacker = 0;
static double totalGlobalAttacker = 0;
mbTypes::Mbs JosephVeinsMode4::induceMisbehavior(double localAttacker,
        double globalAttacker) {

    if (simTime().dbl() < params.START_ATTACK) {
        return mbTypes::Genuine;
    }

    if ((totalLocalAttacker + totalGenuine) == 0) {
        totalGenuine++;
        return mbTypes::Genuine;
    }

    double realFactor = totalLocalAttacker
            / (totalGenuine + totalLocalAttacker);
    if (localAttacker > realFactor) {
        totalLocalAttacker++;
        return mbTypes::LocalAttacker;
    } else {
        double realGFactor = totalGlobalAttacker
                / (totalGenuine + totalGlobalAttacker);
        if (globalAttacker > realGFactor) {
            totalGlobalAttacker++;
            return mbTypes::GlobalAttacker;
        } else {
            totalGenuine++;
            return mbTypes::Genuine;
        }
    }
}

void JosephVeinsMode4::onBSM(BasicSafetyMessage *bsm) {

    if (params.writeVeReMi) {
        VeReMi.serializeBeacon(bsm);
    }

    unsigned long senderPseudonym = bsm->getSenderPseudonym();

    if (params.EnableV1) {
        LocalMisbehaviorDetection(bsm, 1);
    }

    if (params.EnableV2) {
        LocalMisbehaviorDetection(bsm, 2);
    }

    if (!detectedNodes.includes(senderPseudonym)) {
        NodeHistory newNode(senderPseudonym);
        newNode.addBSM(*bsm);
        MDMHistory newMDM(senderPseudonym);
        if (params.EnableV1) {
            newMDM.addBsmCheck(bsmCheckV1, 1);
            newMDM.initKalman(bsm, 1);
        }
        if (params.EnableV2) {
            newMDM.addBsmCheck(bsmCheckV2, 2);
            newMDM.initKalman(bsm, 2);
        }
        detectedNodes.put(senderPseudonym, newNode, newMDM,
                &reportProtocolEnforcerV1, &reportProtocolEnforcerV2);
    } else {
        NodeHistory *existingNode = detectedNodes.getNodeHistoryAddr(
                senderPseudonym);
        existingNode->addBSM(*bsm);
        MDMHistory *existingMDM = detectedNodes.getMDMHistoryAddr(
                senderPseudonym);
        if (params.EnableV1) {
            existingMDM->addBsmCheck(bsmCheckV1, 1);
        }
        if (params.EnableV2) {
            existingMDM->addBsmCheck(bsmCheckV2, 2);
        }
        //detectedNodes.put(senderPseudonym, *existingNode, *existingMDM);
    }

//Your application has received a beacon message from another car or RSU
//code for handling the message goes here
}
void JosephVeinsMode4::treatAttackFlags() {

    if (myMdType == mbTypes::LocalAttacker) {
        attackBsm = mdAttack.launchAttack(myAttackType, &linkControl);

        if (mdAttack.getTargetNode() >= 0) {
            addTargetNode(mdAttack.getTargetNode());
        }

        if (isAccusedNode(myPseudonym)) {
            traciVehicle->setColor(TraCIColor(0, 0, 0, 255));
        } else {
            traciVehicle->setColor(TraCIColor(255, 0, 0, 255));
        }

    } else {
        if (isTargetNode(myPseudonym)) {
            traciVehicle->setColor(TraCIColor(255, 255, 0, 255));
        } else {
            traciVehicle->setColor(TraCIColor(0, 255, 0, 255));
        }
        if (isAccusedNode(myPseudonym)) {
            traciVehicle->setColor(TraCIColor(0, 0, 255, 255));
        }
    }

    if ((simTime().dbl() - targetClearTime) > params.MAX_TARGET_TIME) {
        targetClearTime = simTime().dbl();
        clearTargetNodes();
    }

    if ((simTime().dbl() - accusedClearTime) > params.MAX_ACCUSED_TIME) {
        accusedClearTime = simTime().dbl();
        clearAccusedNodes();
    }

}

void JosephVeinsMode4::LocalMisbehaviorDetection(BasicSafetyMessage *bsm,
        int version) {

    unsigned long senderPseudo = bsm->getSenderPseudonym();
    bsm->getArrivalTime();

    switch (version) {
    case 1: {
        std::string mdv = "V1";
        switch (params.checksVersionV1) {

        case mdChecksVersionTypes::LegacyChecks: {
            LegacyChecks mdm(version, myPseudonym, curPosition, curSpeed,
                    curHeading, veins::Coord(myWidth, myLength),
                    veins::Coord(MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL,
                            MAX_PLAUSIBLE_DECEL), &linkControl, &realDynamicMap, myId, &params);
            bsmCheckV1 = mdm.CheckBSM(bsm, &detectedNodes);
        }
            break;
        case mdChecksVersionTypes::CatchChecks: {
            CaTChChecks mdm(version, myPseudonym, curPosition,
                    curPositionConfidence, curHeading, curHeadingConfidence,
                    veins::Coord(myWidth, myLength),
                    veins::Coord(MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL,
                            MAX_PLAUSIBLE_DECEL), &linkControl, &realDynamicMap, myId, &params);
            bsmCheckV1 = mdm.CheckBSM(bsm, &detectedNodes);
        }
            break;
        case mdChecksVersionTypes::ExperiChecks: {
            ExperiChecks mdm(version, myPseudonym, curPosition,
                    curPositionConfidence, curHeading, curHeadingConfidence,
                    veins::Coord(myWidth, myLength),
                    veins::Coord(MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL,
                            MAX_PLAUSIBLE_DECEL), &linkControl, &realDynamicMap, myId, &params);
            bsmCheckV1 = mdm.CheckBSM(bsm, &detectedNodes);
        }
            break;
        default: {
            LegacyChecks mdm(version, myPseudonym, curPosition, curSpeed,
                    curHeading, veins::Coord(myWidth, myLength),
                    veins::Coord(MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL,
                            MAX_PLAUSIBLE_DECEL), &linkControl, &realDynamicMap, myId, &params);
            bsmCheckV1 = mdm.CheckBSM(bsm, &detectedNodes);
        }
            break;
        }

        bool result = false;
        auto startV1 = high_resolution_clock::now();
        result = AppV1->CheckNodeForReport(myPseudonym, bsm, &bsmCheckV1,
                &detectedNodes);
        auto endV1 = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(endV1 - startV1);
        meanTimeV1 = ((double) numTimeV1 * meanTimeV1 + (duration.count()))
                / ((double) numTimeV1 + 1.0);
        numTimeV1 = numTimeV1 + 1;

        if (params.enableVarThreV1) {
            varThrePrintableV1.registerMessage(
                    mbTypes::intMbs[bsm->getSenderMbType()],
                    AppV1->getMinFactor());
        }

        if (result && (myMdType == mbTypes::Genuine)) {
            MDReport reportBase;
            reportBase.setGenerationTime(simTime().dbl());
            reportBase.setSenderPseudo(myPseudonym);
            reportBase.setReportedPseudo(senderPseudo);

            reportBase.setSenderRealId(myId);
            reportBase.setReportedRealId(bsm->getSenderRealId());

            reportBase.setMbType(mbTypes::mbNames[bsm->getSenderMbType()]);
            reportBase.setAttackType(
                    attackTypes::AttackNames[bsm->getSenderAttackType()]);
            std::pair<double, double> currLonLat = traci->getLonLat(
                    curPosition);
            reportBase.setSenderGps(veins::Coord(currLonLat.first, currLonLat.second));
            reportBase.setReportedGps(bsm->getSenderGpsCoordinates());

            char nameV1[32] = "mdaV1";
            mdStats.getReport(nameV1, reportBase);
            if (params.writeReportsV1 && myBsmNum > 0) {
                writeReport(reportBase, version, mdv, bsmCheckV1, bsm);
            }

            if (params.writeListReportsV1 && myBsmNum > 0) {
                writeListReport(reportBase, version, mdv, bsmCheckV1, bsm);
            }

            if (params.sendReportsV1 && myBsmNum > 0) {
                sendReport(reportBase, version, mdv, bsmCheckV1, bsm);
            }
        } else if (myMdType == mbTypes::GlobalAttacker) {
            MDReport reportBase = mdGlobalAttack.launchAttack(myAttackType,
                    bsm);

            if (params.writeReportsV1) {
                writeReport(reportBase, version, mdv, bsmCheckV1, bsm);
            }

            if (params.writeListReportsV1) {
                writeListReport(reportBase, version, mdv, bsmCheckV1, bsm);
            }

            if (params.sendReportsV1) {
                sendReport(reportBase, version, mdv, bsmCheckV1, bsm);
            }
        }

        if (params.writeBsmsV1) {
            writeMdBsm(mdv, bsmCheckV1, bsm);
        }
        if (params.writeListBsmsV1) {
            writeMdListBsm(mdv, bsmCheckV1, bsm);
        }

        if (!initV1) {
            AppV1->resetAllFlags();
            //mdAuthority.resetAll();
            initV1 = true;
        }

        if ((simTime().dbl() - deltaTV1) > params.SAVE_PERIOD) {
            bool printOut = false;
            if ((simTime().dbl() - deltaTVS1) > params.PRINT_PERIOD) {
                deltaTVS1 = simTime().dbl();
                printOut = true;
                std::cout << "-_-_-_-_-_-_-_-_-_-_-_-_-" << " meanTimeV1:"
                        << meanTimeV1 << " μs " << numTimeV1 << "\n";
            }

            deltaTV1 = simTime().dbl();

            if ((simTime().dbl() > params.START_SAVE) && params.SaveStatsV1) {

                AppV1->saveLine(params.savePath, params.serialNumber,
                        mobility->getManager()->getManagedHosts().size(),
                        deltaTV1, printOut);

                mdStats.saveLine(params.savePath, params.serialNumber, deltaTV1,
                        printOut);
                if (params.enableVarThreV1) {
                    varThrePrintableV1.saveFile(params.savePath,
                            params.serialNumber, printOut);
                }

            }
            AppV1->resetInstFlags();
        }

        if (result) {
            addAccusedNode(senderPseudo);
        }

        break;
    }
    case 2: {

        std::string mdv = "V2";

        switch (params.checksVersionV2) {
        case mdChecksVersionTypes::LegacyChecks: {
            LegacyChecks mdm(version, myPseudonym, curPosition, curSpeed,
                    curHeading, veins::Coord(myWidth, myLength),
                    veins::Coord(MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL,
                            MAX_PLAUSIBLE_DECEL), &linkControl, &realDynamicMap, myId, &params);
            bsmCheckV2 = mdm.CheckBSM(bsm, &detectedNodes);
        }
            break;
        case mdChecksVersionTypes::CatchChecks: {
            CaTChChecks mdm(version, myPseudonym, curPosition,
                    curPositionConfidence, curHeading, curHeadingConfidence,
                    veins::Coord(myWidth, myLength),
                    veins::Coord(MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL,
                            MAX_PLAUSIBLE_DECEL), &linkControl, &realDynamicMap, myId, &params);
            bsmCheckV2 = mdm.CheckBSM(bsm, &detectedNodes);
        }
            break;
        case mdChecksVersionTypes::ExperiChecks: {
            ExperiChecks mdm(version, myPseudonym, curPosition,
                    curPositionConfidence, curHeading, curHeadingConfidence,
                    veins::Coord(myWidth, myLength),
                    veins::Coord(MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL,
                            MAX_PLAUSIBLE_DECEL), &linkControl, &realDynamicMap, myId, &params);
            bsmCheckV2 = mdm.CheckBSM(bsm, &detectedNodes);
        }
            break;
        default: {
            LegacyChecks mdm(version, myPseudonym, curPosition, curSpeed,
                    curHeading, veins::Coord(myWidth, myLength),
                    veins::Coord(MAX_PLAUSIBLE_SPEED, MAX_PLAUSIBLE_ACCEL,
                            MAX_PLAUSIBLE_DECEL), &linkControl, &realDynamicMap, myId, &params);
            bsmCheckV2 = mdm.CheckBSM(bsm, &detectedNodes);
        }
            break;
        }

        bool result = false;
        auto startV2 = high_resolution_clock::now();
        result = AppV2->CheckNodeForReport(myPseudonym, bsm, &bsmCheckV2,
                &detectedNodes);
        auto endV2 = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(endV2 - startV2);
        meanTimeV2 = ((double) numTimeV2 * meanTimeV2 + (duration.count()))
                / ((double) numTimeV2 + 1.0);
        numTimeV2 = numTimeV2 + 1;

        if (params.enableVarThreV2) {
            varThrePrintableV2.registerMessage(
                    mbTypes::intMbs[bsm->getSenderMbType()],
                    AppV2->getMinFactor());
        }

        if (result && (myMdType == mbTypes::Genuine)) {
            MDReport reportBase;
            reportBase.setGenerationTime(simTime().dbl());
            reportBase.setSenderPseudo(myPseudonym);
            reportBase.setReportedPseudo(senderPseudo);

            reportBase.setSenderRealId(myId);
            reportBase.setReportedRealId(bsm->getSenderRealId());

            reportBase.setMbType(mbTypes::mbNames[bsm->getSenderMbType()]);
            reportBase.setAttackType(
                    attackTypes::AttackNames[bsm->getSenderAttackType()]);
            std::pair<double, double> currLonLat = traci->getLonLat(
                    curPosition);
            reportBase.setSenderGps(veins::Coord(currLonLat.first, currLonLat.second));
            reportBase.setReportedGps(bsm->getSenderGpsCoordinates());

            char nameV2[32] = "mdaV2";
            mdStats.getReport(nameV2, reportBase);

            if (params.writeReportsV2 && myBsmNum > 0) {
                writeReport(reportBase, version, mdv, bsmCheckV2, bsm);
            }

            if (params.writeListReportsV2 && myBsmNum > 0) {
                writeListReport(reportBase, version, mdv, bsmCheckV2, bsm);
            }

            if (params.sendReportsV2 && myBsmNum > 0) {
                sendReport(reportBase, version, mdv, bsmCheckV2, bsm);
            }

        } else if (myMdType == mbTypes::GlobalAttacker) {
            MDReport reportBase = mdGlobalAttack.launchAttack(myAttackType,
                    bsm);

            if (params.writeReportsV2) {
                writeReport(reportBase, version, mdv, bsmCheckV2, bsm);
            }

            if (params.writeListReportsV2) {
                writeListReport(reportBase, version, mdv, bsmCheckV2, bsm);
            }

            if (params.sendReportsV2) {
                sendReport(reportBase, version, mdv, bsmCheckV2, bsm);
            }

        }

        if (params.writeBsmsV2) {
            writeMdBsm(mdv, bsmCheckV2, bsm);
        }

        if (params.writeListBsmsV2) {
            writeMdListBsm(mdv, bsmCheckV2, bsm);
        }

        if (!initV2) {
            AppV2->resetAllFlags();
            //mdAuthority.resetAll();
            initV2 = true;
        }

        if ((simTime().dbl() - deltaTV2) > params.SAVE_PERIOD) {
            bool printOut = false;
            if ((simTime().dbl() - deltaTVS2) > params.PRINT_PERIOD) {
                deltaTVS2 = simTime().dbl();
                printOut = true;
                std::cout << "-_-_-_-_-_-_-_-_-_-_-_-_-" << " meanTimeV2:"
                        << meanTimeV2 << " μs " << numTimeV2 << "\n";
            }

            deltaTV2 = simTime().dbl();

            if ((simTime().dbl() > params.START_SAVE) && params.SaveStatsV2) {
                AppV2->saveLine(params.savePath, params.serialNumber,
                        mobility->getManager()->getManagedHosts().size(),
                        deltaTV2, printOut);

                mdStats.saveLine(params.savePath, params.serialNumber, deltaTV2,
                        printOut);
                if (params.enableVarThreV2) {
                    varThrePrintableV2.saveFile(params.savePath,
                            params.serialNumber, printOut);
                }

            }
            AppV2->resetInstFlags();
        }

        if (result) {
            addAccusedNode(senderPseudo);
        }

        break;
    }

    default:
        break;
    }

}

void JosephVeinsMode4::writeReport(MDReport reportBase, int version,
        std::string maversion, BsmCheck bsmCheck, BasicSafetyMessage *bsm) {
    switch (myReportType) {
    case reportTypes::BasicCheckReport: {
        BasicCheckReport bcr = BasicCheckReport(reportBase);
        bcr.setReportedCheck(bsmCheck);
        bcr.writeStrToFile(params.savePath, params.serialNumber, maversion,
                bcr.getReportPrintableJson(), curDate);
    }
        break;

    case reportTypes::OneMessageReport: {
        OneMessageReport omr = OneMessageReport(reportBase);
        omr.setReportedBsm(*bsm);
        omr.setReportedCheck(bsmCheck);
        omr.writeStrToFile(params.savePath, params.serialNumber, maversion,
                omr.getReportPrintableJson(), curDate);
    }
        break;
    case reportTypes::EvidenceReport: {
        EvidenceReport evr = EvidenceReport(reportBase);
        if (myBsmNum > 0) {
            evr.addEvidence(myBsm[0], bsmCheck, *bsm, &detectedNodes);
            evr.writeStrToFile(params.savePath, params.serialNumber, maversion,
                    evr.getReportPrintableJson(), curDate);
        } else {
            OneMessageReport omr = OneMessageReport(reportBase);
            omr.setReportedBsm(*bsm);
            omr.setReportedCheck(bsmCheck);
            omr.writeStrToFile(params.savePath, params.serialNumber, maversion,
                    omr.getReportPrintableJson(), curDate);
        }
    }
        break;
    case reportTypes::ProtocolReport: {
        bool newNode = false;
        switch (version) {
        case 1: {
            newNode = reportProtocolEnforcerV1.addMisbehavingPseudo(
                    bsm->getSenderPseudonym(), simTime().dbl());
        }
            break;
        case 2: {
            newNode = reportProtocolEnforcerV2.addMisbehavingPseudo(
                    bsm->getSenderPseudonym(), simTime().dbl());
        }
            break;
        }
        if (newNode) {
            ProtocolReport hir = ProtocolReport(reportBase);
            hir.addEvidence(myBsm[0], bsmCheck, *bsm, &detectedNodes,
                    simTime().dbl(), params.InitialHistory, version);
            hir.writeStrToFile(params.savePath, params.serialNumber, maversion,
                    hir.getReportPrintableJson(), curDate);
        }
    }
        break;
    default:
        break;
    }
}

void JosephVeinsMode4::writeListReport(MDReport reportBase, int version,
        std::string maversion, BsmCheck bsmCheck, BasicSafetyMessage *bsm) {

    switch (myReportType) {
    case reportTypes::BasicCheckReport: {
        BasicCheckReport bcr = BasicCheckReport(reportBase);
        bcr.setReportedCheck(bsmCheck);
        bcr.writeStrToFileList(params.savePath, params.serialNumber, maversion,
                bcr.getReportPrintableJson(), curDate);
    }
        break;

    case reportTypes::OneMessageReport: {
        OneMessageReport omr = OneMessageReport(reportBase);
        omr.setReportedBsm(*bsm);
        omr.setReportedCheck(bsmCheck);
        omr.writeStrToFileList(params.savePath, params.serialNumber, maversion,
                omr.getReportPrintableJson(), curDate);
    }
        break;
    case reportTypes::EvidenceReport: {
        EvidenceReport evr = EvidenceReport(reportBase);
        if (myBsmNum > 0) {
            evr.addEvidence(myBsm[0], bsmCheck, *bsm, &detectedNodes);
            evr.writeStrToFileList(params.savePath, params.serialNumber,
                    maversion, evr.getReportPrintableJson(), curDate);
        } else {
            OneMessageReport omr = OneMessageReport(reportBase);
            omr.setReportedBsm(*bsm);
            omr.setReportedCheck(bsmCheck);
            omr.writeStrToFileList(params.savePath, params.serialNumber,
                    maversion, omr.getReportPrintableJson(), curDate);
        }
    }
        break;
    case reportTypes::ProtocolReport: {
        bool newNode = false;
        switch (version) {
        case 1: {
            newNode = reportProtocolEnforcerV1.addMisbehavingPseudo(
                    bsm->getSenderPseudonym(), simTime().dbl());
        }
            break;
        case 2:
            newNode = reportProtocolEnforcerV2.addMisbehavingPseudo(
                    bsm->getSenderPseudonym(), simTime().dbl());
            break;
        }
        if (newNode) {
            ProtocolReport hir = ProtocolReport(reportBase);
            hir.addEvidence(myBsm[0], bsmCheck, *bsm, &detectedNodes,
                    simTime().dbl(), params.InitialHistory, version);
            hir.writeStrToFileList(params.savePath, params.serialNumber,
                    maversion, hir.getReportPrintableJson(), curDate);
        }
    }
        break;
    default:
        break;
    }

}

void JosephVeinsMode4::sendReport(MDReport reportBase, int version,
        std::string maversion, BsmCheck bsmCheck, BasicSafetyMessage *bsm) {

    std::string reportStr = "";

    switch (myReportType) {
    case reportTypes::BasicCheckReport: {
        BasicCheckReport bcr = BasicCheckReport(reportBase);
        bcr.setReportedCheck(bsmCheck);
        reportStr = bcr.getReportPrintableJson();
    }
        break;

    case reportTypes::OneMessageReport: {
        OneMessageReport omr = OneMessageReport(reportBase);
        omr.setReportedBsm(*bsm);
        omr.setReportedCheck(bsmCheck);
        reportStr = omr.getReportPrintableJson();
    }
        break;
    case reportTypes::EvidenceReport: {
        EvidenceReport evr = EvidenceReport(reportBase);
        if (myBsmNum > 0) {
            evr.addEvidence(myBsm[0], bsmCheck, *bsm, &detectedNodes);
            reportStr = evr.getReportPrintableJson();
        } else {
            OneMessageReport omr = OneMessageReport(reportBase);
            omr.setReportedBsm(*bsm);
            omr.setReportedCheck(bsmCheck);
            reportStr = omr.getReportPrintableJson();
        }
    }
        break;
    case reportTypes::ProtocolReport: {
        bool newNode = false;
        switch (version) {
        case 1: {

            newNode = reportProtocolEnforcerV1.addMisbehavingPseudo(
                    bsm->getSenderPseudonym(), simTime().dbl());
        }
            break;
        case 2:

            newNode = reportProtocolEnforcerV2.addMisbehavingPseudo(
                    bsm->getSenderPseudonym(), simTime().dbl());
            break;
        }

        if (newNode) {
            ProtocolReport hir = ProtocolReport(reportBase);
            hir.addEvidence(myBsm[0], bsmCheck, *bsm, &detectedNodes,
                    simTime().dbl(), params.InitialHistory, version);
            reportStr = hir.getReportPrintableJson();

        } else {
            //do not report now
            return;
        }
    }
        break;
    default:
        reportStr = "ERROR myReportType";
        break;
    }

    //std::cout << reportStr << "\n";
    //exit(0);

    if (!maversion.compare("V1")) {
        HTTPRequest httpr = HTTPRequest(params.maPortV1, params.maHostV1);
        std::string response = httpr.Request(reportStr);
    }

    if (!maversion.compare("V2")) {
        HTTPRequest httpr = HTTPRequest(params.maPortV2, params.maHostV2);
        std::string response = httpr.Request(reportStr);
    }

}

void JosephVeinsMode4::writeMdBsm(std::string version, BsmCheck bsmCheck,
        BasicSafetyMessage *bsm) {
    BsmPrintable bsmPrint = BsmPrintable();
    bsmPrint.setReceiverId(myId);
    bsmPrint.setReceiverPseudo(myPseudonym);
    bsmPrint.setBsm(*bsm);
    bsmPrint.setBsmCheck(bsmCheck);
    bsmPrint.writeStrToFile(params.savePath, params.serialNumber, version,
            bsmPrint.getBsmPrintableJson(), curDate);
}

void JosephVeinsMode4::writeMdListBsm(std::string version, BsmCheck bsmCheck,
        BasicSafetyMessage *bsm) {
    BsmPrintable bsmPrint = BsmPrintable();
    bsmPrint.setReceiverId(myId);
    bsmPrint.setReceiverPseudo(myPseudonym);
    bsmPrint.setBsm(*bsm);
    bsmPrint.setBsmCheck(bsmCheck);
    bsmPrint.writeStrToFileList(params.savePath, params.serialNumber, version,
            bsmPrint.getBsmPrintableJson(), curDate);
}

void JosephVeinsMode4::writeSelfBsm(BasicSafetyMessage bsm) {
    BsmPrintable bsmPrint = BsmPrintable();
    bsmPrint.setReceiverId(myId);
    bsmPrint.setReceiverPseudo(myPseudonym);
    bsmPrint.setBsm(bsm);
    bsmPrint.writeSelfStrToFile(params.savePath, params.serialNumber,
            bsmPrint.getSelfBsmPrintableJson(myVType), curDate);
}

void JosephVeinsMode4::writeSelfListBsm(BasicSafetyMessage bsm) {
    BsmPrintable bsmPrint = BsmPrintable();
    bsmPrint.setReceiverId(myId);
    bsmPrint.setReceiverPseudo(myPseudonym);
    bsmPrint.setBsm(bsm);
    bsmPrint.writeSelfStrToFileList(params.savePath, params.serialNumber,
            bsmPrint.getSelfBsmPrintableJson(myVType), curDate);
}

void JosephVeinsMode4::handleSelfMessage(cMessage *msg) {

    treatAttackFlags();

    F2MDBaseMode4Layer::handleSelfMessage(msg);

    if (params.sendReportsV1) {
        std::string reportStr = "curTime:";
        reportStr.append(std::to_string(simTime().dbl()));
        HTTPRequest httpr = HTTPRequest(params.maPortV1, params.maHostV1);
        std::string response = httpr.Request(reportStr);
    }
    if (params.sendReportsV2) {
        std::string reportStr = "curTime:";
        reportStr.append(std::to_string(simTime().dbl()));
        HTTPRequest httpr = HTTPRequest(params.maPortV2, params.maHostV2);
        std::string response = httpr.Request(reportStr);
    }

    if (params.writeSelfMsg) {
        writeSelfBsm(myBsm[0]);
    }

    if (params.writeListSelfMsg) {
        writeSelfListBsm(myBsm[0]);
    }

    if (myReportType == reportTypes::ProtocolReport) {
        handleReportProtocol(false);
    }

    if (params.EnablePC) {
        pcPolicy.checkPseudonymChange(myPcType);
    }

//this method is for self messages (mostly timers)
//it is important to call the F2MDBaseMode4Layer function for BSM and WSM transmission

}

void JosephVeinsMode4::handleReportProtocol(bool lastTimeStep) {

    unsigned long pseudosList[MAX_REP_PSEUDOS];
    if (params.EnableV1) {
        int pseudoNum = 0;
        if (lastTimeStep) {
            pseudoNum = reportProtocolEnforcerV1.getAllReportPseudoes(
                    simTime().dbl(), pseudosList);
        } else {
            pseudoNum = reportProtocolEnforcerV1.getReportPseudoes(
                    simTime().dbl(), pseudosList);
        }

        for (int var = 0; var < pseudoNum; ++var) {
            if (!detectedNodes.includes(pseudosList[var])) {
                BasicSafetyMessage *reportedBsm =
                        detectedNodes.getNodeHistoryAddr(pseudosList[var])->getLatestBSMAddr();

                BsmCheck *reportedCheck = detectedNodes.getMDMHistoryAddr(
                        pseudosList[var])->getLatestBsmCheckAddr(1);

                MDReport reportBase;
                reportBase.setGenerationTime(simTime().dbl());
                reportBase.setSenderPseudo(myPseudonym);
                reportBase.setReportedPseudo(pseudosList[var]);

                reportBase.setSenderRealId(myId);
                reportBase.setReportedRealId(reportedBsm->getSenderRealId());

                reportBase.setMbType(
                        mbTypes::mbNames[reportedBsm->getSenderMbType()]);
                reportBase.setAttackType(
                        attackTypes::AttackNames[reportedBsm->getSenderAttackType()]);
                std::pair<double, double> currLonLat = traci->getLonLat(
                        curPosition);
                reportBase.setSenderGps(
                        veins::Coord(currLonLat.first, currLonLat.second));
                reportBase.setReportedGps(
                        reportedBsm->getSenderGpsCoordinates());

                ProtocolReport hir = ProtocolReport(reportBase);
                hir.addEvidence(myBsm[0], *reportedCheck, *reportedBsm,
                        &detectedNodes, simTime().dbl(),
                        params.CollectionPeriod, 1);

                if (params.writeReportsV1 && myBsmNum > 0) {
                    hir.writeStrToFile(params.savePath, params.serialNumber,
                            "V1", hir.getReportPrintableJson(), curDate);
                }

                if (params.writeListReportsV1 && myBsmNum > 0) {
                    hir.writeStrToFileList(params.savePath, params.serialNumber,
                            "V1", hir.getReportPrintableJson(), curDate);
                }

                if (params.sendReportsV1 && myBsmNum > 0) {
                    HTTPRequest httpr = HTTPRequest(params.maPortV1,
                            params.maHostV1);
                    std::string response = httpr.Request(
                            hir.getReportPrintableJson());
                }
            } else {
                reportProtocolEnforcerV1.removeReportedPseudo(pseudosList[var]);
            }
        }

    }

    if (params.EnableV2) {

        int pseudoNum = 0;
        if (lastTimeStep) {
            pseudoNum = reportProtocolEnforcerV2.getAllReportPseudoes(
                    simTime().dbl(), pseudosList);
        } else {
            pseudoNum = reportProtocolEnforcerV2.getReportPseudoes(
                    simTime().dbl(), pseudosList);
        }

        for (int var = 0; var < pseudoNum; ++var) {
            if (detectedNodes.includes(pseudosList[var])) {
                BasicSafetyMessage *reportedBsm =
                        detectedNodes.getNodeHistoryAddr(pseudosList[var])->getLatestBSMAddr();
                BsmCheck *reportedCheck = detectedNodes.getMDMHistoryAddr(
                        pseudosList[var])->getLatestBsmCheckAddr(2);
                if (reportedBsm->getSenderPseudonym() == 0) {
                    std::cout << pseudosList[var] << "\n";
                    std::cout << "Problem Found \n";
                    exit(0);
                }

                MDReport reportBase;
                reportBase.setGenerationTime(simTime().dbl());
                reportBase.setSenderPseudo(myPseudonym);
                reportBase.setReportedPseudo(pseudosList[var]);

                reportBase.setSenderRealId(myId);
                reportBase.setReportedRealId(reportedBsm->getSenderRealId());

                reportBase.setMbType(
                        mbTypes::mbNames[reportedBsm->getSenderMbType()]);
                reportBase.setAttackType(
                        attackTypes::AttackNames[reportedBsm->getSenderAttackType()]);
                std::pair<double, double> currLonLat = traci->getLonLat(
                        curPosition);
                reportBase.setSenderGps(
                        veins::Coord(currLonLat.first, currLonLat.second));
                reportBase.setReportedGps(
                        reportedBsm->getSenderGpsCoordinates());

                ProtocolReport hir = ProtocolReport(reportBase);

                hir.addEvidence(myBsm[0], *reportedCheck, *reportedBsm,
                        &detectedNodes, simTime().dbl(),
                        params.CollectionPeriod, 2);

                if (params.writeReportsV2 && myBsmNum > 0) {
                    hir.writeStrToFile(params.savePath, params.serialNumber,
                            "V2", hir.getReportPrintableJson(), curDate);
                }

                if (params.writeListReportsV2 && myBsmNum > 0) {
                    hir.writeStrToFileList(params.savePath, params.serialNumber,
                            "V2", hir.getReportPrintableJson(), curDate);
                }

                if (params.sendReportsV2 && myBsmNum > 0) {
                    HTTPRequest httpr = HTTPRequest(params.maPortV2,
                            params.maHostV2);
                    std::string response = httpr.Request(
                            hir.getReportPrintableJson());
                }
            } else {
                reportProtocolEnforcerV2.removeReportedPseudo(pseudosList[var]);
            }
        }

    }

}
void JosephVeinsMode4::handlePositionUpdate(cObject *obj) {
    F2MDBaseMode4Layer::handlePositionUpdate(obj);

    realDynamicMap[myId] = veins::Coord(mobility->getCurrentPosition().x,mobility->getCurrentPosition().y,mobility->getCurrentPosition().z);

    RelativeOffsetConf relativeOffsetConfidence = RelativeOffsetConf(
            &ConfPosMax, &ConfSpeedMax, &ConfHeadMax, &ConfAccelMax,
            &deltaConfPos, &deltaConfSpeed, &deltaConfHead, &deltaConfAccel);

    veins::Coord tempPosition = veins::Coord(mobility->getCurrentPosition().x,mobility->getCurrentPosition().y,mobility->getCurrentPosition().z);
    veins::Coord tempSpeed = veins::Coord(mobility->getCurrentSpeed().x,mobility->getCurrentSpeed().y,mobility->getCurrentSpeed().z);
    veins::Coord tempAccel = veins::Coord(mobility->getCurrentAcceleration().x,mobility->getCurrentAcceleration().y,mobility->getCurrentAcceleration().z);
    veins::Coord tempHeading = veins::Coord(cos(-mobility->getCurrentAngularPosition().alpha), -sin(-mobility->getCurrentAngularPosition().alpha));


    if (params.veremiConf) {
        double posEtx = genLib.GaussianRandomDouble(
                (curPositionConfidence.x + curPositionConfidenceOrig.x) / 2,
                0.03 * curPositionConfidenceOrig.x);
        double posEty = genLib.GaussianRandomDouble(
                (curPositionConfidence.y + curPositionConfidenceOrig.y) / 2,
                0.03 * curPositionConfidenceOrig.y);

        curPositionConfidence = veins::Coord(posEtx, posEty, 0);

        curHeadingConfidence = veins::Coord(
                curHeadingConfidenceOrig.x
                        * exp(-0.1 * mobility->getCurrentSpeed().x),
                curHeadingConfidenceOrig.y
                        * exp(-0.1 * mobility->getCurrentSpeed().y), 0);

        veins::Coord oldSpeedConfidence = veins::Coord(curSpeedConfidence.x,
                curSpeedConfidence.y, curSpeedConfidence.z);
        curSpeedConfidence = veins::Coord(curSpeed.x * curSpeedConfidenceOrig.x,
                curSpeed.y * curSpeedConfidenceOrig.y, 0);

        curAccelConfidence = veins::Coord(
                fabs((curSpeedConfidence.x - oldSpeedConfidence.x))
                        / (simTime().dbl() - lastPositionUpdate),
                fabs((curSpeedConfidence.y - oldSpeedConfidence.y))
                        / (simTime().dbl() - lastPositionUpdate),
                fabs((curSpeedConfidence.z - oldSpeedConfidence.z))
                        / (simTime().dbl() - lastPositionUpdate));

        lastPositionUpdate = simTime().dbl();

        //curHeadingConfidence = veins::Coord(0, 0, 0);

        RelativeOffset relativeOffset = RelativeOffset(&curPositionConfidence,
                &curSpeedConfidence, &curHeadingConfidence, &curAccelConfidence,
                &deltaRPosition, &deltaThetaPosition, &deltaSpeed,
                &deltaHeading, &deltaAccel);

        curPosition = relativeOffset.OffsetPosition(tempPosition);
        curSpeed = relativeOffset.OffsetSpeed(tempSpeed);
        curHeading = relativeOffset.OffsetHeading(tempHeading);
        curAccel = relativeOffset.OffsetAccel(tempAccel);

    } else {
        curPositionConfidence = relativeOffsetConfidence.OffsetPosConf(
                curPositionConfidenceOrig);
        curSpeedConfidence = relativeOffsetConfidence.OffsetSpeedConf(
                curSpeedConfidenceOrig);
        curHeadingConfidence = relativeOffsetConfidence.OffsetHeadingConf(
                curHeadingConfidenceOrig);
        curAccelConfidence = relativeOffsetConfidence.OffsetAccelConf(
                curAccelConfidenceOrig);

        RelativeOffset relativeOffset = RelativeOffset(&curPositionConfidence,
                &curSpeedConfidence, &curHeadingConfidence, &curAccelConfidence,
                &deltaRPosition, &deltaThetaPosition, &deltaSpeed,
                &deltaHeading, &deltaAccel);

        //    GaussianRandom relativeOffset = GaussianRandom(&curPositionConfidence,
        //            &curSpeedConfidence, &curHeadingConfidence);

        curPosition = relativeOffset.OffsetPosition(tempPosition);
        curSpeed = relativeOffset.OffsetSpeed(tempSpeed);
        curHeading = relativeOffset.OffsetHeading(tempHeading);
        curAccel = relativeOffset.OffsetAccel(tempAccel);
    }

    if (params.writeVeReMi) {
        VeReMi.serializeRawData(&curPosition, &curPositionConfidence, &curSpeed,
                &curSpeedConfidence, &curHeading, &curHeadingConfidence,
                &curAccel, &curAccelConfidence);
    }

//the vehicle has moved. Code that reacts to new positions goes here.
//member variables such as currentPosition and currentSpeed are updated in the parent class
}

// code is based on TracingApp::populateWSM(WaveShortMessage* wsm, int rcvId, int serial)
void JosephVeinsMode4::populateBSM(BasicSafetyMessage *bsm) {
    F2MDBaseMode4Layer::populateBSM(bsm);

    //F2MD
    bsm->setSenderPseudonym(myPseudonym);
    bsm->setSenderMbType(myMdType);
    bsm->setSenderAttackType(myAttackType);
    // Genuine
    bsm->setSenderPos(curPosition);
    bsm->setSenderPosConfidence(curPositionConfidence);

    std::pair<double, double> currLonLat = traci->getLonLat(curPosition);
    bsm->setSenderGpsCoordinates(veins::Coord(currLonLat.first, currLonLat.second));

    bsm->setSenderSpeed(curSpeed);
    bsm->setSenderSpeedConfidence(curSpeedConfidence);

    bsm->setSenderHeading(curHeading);
    bsm->setSenderHeadingConfidence(curHeadingConfidence);

    bsm->setSenderAccel(curAccel);
    bsm->setSenderAccelConfidence(curAccelConfidence);

    bsm->setSenderWidth(myWidth);
    bsm->setSenderLength(myLength);

    bsm->setSenderRealId(myId);
    addMyBsm(*bsm);
    // Genuine

    if (myMdType == mbTypes::LocalAttacker) {
        if (attackBsm.getSenderPseudonym() != 0) {
            bsm->setSenderPseudonym(attackBsm.getSenderPseudonym());

            bsm->setSenderPos(attackBsm.getSenderPos());
            bsm->setSenderPosConfidence(attackBsm.getSenderPosConfidence());

            std::pair<double, double> currLonLat = traci->getLonLat(
                    attackBsm.getSenderPos());
            bsm->setSenderGpsCoordinates(
                    veins::Coord(currLonLat.first, currLonLat.second));

            bsm->setSenderSpeed(attackBsm.getSenderSpeed());
            bsm->setSenderSpeedConfidence(attackBsm.getSenderSpeedConfidence());

            bsm->setSenderHeading(attackBsm.getSenderHeading());
            bsm->setSenderHeadingConfidence(
                    attackBsm.getSenderHeadingConfidence());

            bsm->setSenderAccel(attackBsm.getSenderAccel());
            bsm->setSenderAccelConfidence(attackBsm.getSenderAccelConfidence());

            bsm->setSenderWidth(attackBsm.getSenderWidth());
            bsm->setSenderLength(attackBsm.getSenderLength());
        } else {
            bsm->setSenderPos(curPosition);
            bsm->setSenderPosConfidence(curPositionConfidence);

            std::pair<double, double> currLonLat = traci->getLonLat(
                    curPosition);
            bsm->setSenderGpsCoordinates(
                    veins::Coord(currLonLat.first, currLonLat.second));

            bsm->setSenderSpeed(curSpeed);
            bsm->setSenderSpeedConfidence(curSpeedConfidence);

            bsm->setSenderHeading(curHeading);
            bsm->setSenderHeadingConfidence(curHeadingConfidence);

            bsm->setSenderAccel(curAccel);
            bsm->setSenderAccelConfidence(curAccelConfidence);

            bsm->setSenderWidth(myWidth);
            bsm->setSenderLength(myLength);

            bsm->setSenderMbType(mbTypes::Genuine);
            //bsm->setSenderAttackType(attackTypes::Genuine);
        }
    }

//        GeneralLib genLib = GeneralLib();
//        Coord posConf = bsm->getSenderPosConfidence();
//        posConf = veins::Coord(posConf.x + genLib.RandomDouble(0, 0.1*posConf.x) ,posConf.y + genLib.RandomDouble(0, 0.1*posConf.y),posConf.z);
//        bsm->setSenderPosConfidence(posConf);
//
//        Coord spdConf = bsm->getSenderSpeedConfidence();
//        spdConf = veins::Coord(spdConf.x + genLib.RandomDouble(0, 0.1*spdConf.x) ,spdConf.y + genLib.RandomDouble(0, 0.1*spdConf.y),spdConf.z);
//        bsm->setSenderSpeedConfidence(spdConf);
//
//        Coord accConf = bsm->getSenderAccelConfidence();
//        accConf = veins::Coord(accConf.x + genLib.RandomDouble(0, 0.1*accConf.x) ,accConf.y + genLib.RandomDouble(0, 0.1*accConf.y),accConf.z);
//        bsm->setSenderAccelConfidence(accConf);
//
//        Coord headConf = bsm->getSenderHeadingConfidence();
//        headConf = veins::Coord(headConf.x + genLib.RandomDouble(0, 0.1*headConf.x) ,headConf.y + genLib.RandomDouble(0, 0.1*headConf.y),headConf.z);
//        bsm->setSenderHeadingConfidence(headConf);

    if (params.writeVeReMi) {
        VeReMi.serializeGroundTruth(bsm);
    }

}

void JosephVeinsMode4::addTargetNode(unsigned long id) {
    bool found = false;
    for (int var = 0; var < targetNodesLength; ++var) {
        if (targetNodes[var] == id) {
            found = true;
        }
    }

    if (!found) {
        targetNodes[targetNodesLength] = id;
        targetNodesLength++;
    }
}
void JosephVeinsMode4::removeTargetNode(unsigned long id) {
    int index = -1;
    for (int var = 0; var < targetNodesLength; ++var) {
        if (targetNodes[var] == id) {
            index = var;
            break;
        }
    }

    for (int var = index; var < targetNodesLength - 1; ++var) {
        targetNodes[var] = targetNodes[var + 1];
    }
    targetNodesLength--;
}
void JosephVeinsMode4::clearTargetNodes() {
    targetNodesLength = 0;
}
bool JosephVeinsMode4::isTargetNode(unsigned long id) {
    for (int var = 0; var < targetNodesLength; ++var) {
        if (targetNodes[var] == id) {
            return true;
        }
    }
    return false;
}

void JosephVeinsMode4::addAccusedNode(unsigned long id) {
    bool found = false;
    for (int var = 0; var < accusedNodesLength; ++var) {
        if (accusedNodes[var] == id) {
            found = true;
        }
    }

    if (!found) {
        accusedNodes[accusedNodesLength] = id;
        accusedNodesLength++;
    }
}
void JosephVeinsMode4::removeAccusedNode(unsigned long id) {
    int index = -1;
    for (int var = 0; var < accusedNodesLength; ++var) {
        if (accusedNodes[var] == id) {
            index = var;
            break;
        }
    }

    for (int var = index; var < accusedNodesLength - 1; ++var) {
        accusedNodes[var] = accusedNodes[var + 1];
    }
    accusedNodesLength--;
}

void JosephVeinsMode4::clearAccusedNodes() {
    accusedNodesLength = 0;
}

bool JosephVeinsMode4::isAccusedNode(unsigned long id) {
    for (int var = 0; var < accusedNodesLength; ++var) {
        if (accusedNodes[var] == id) {
            return true;
        }
    }
    return false;
}

//F2MD
void JosephVeinsMode4::addMyBsm(BasicSafetyMessage bsm) {
    if (myBsmNum < MYBSM_SIZE) {
        myBsmNum++;
    }
    for (int var = myBsmNum - 1; var > 0; --var) {
        myBsm[var] = myBsm[var - 1];
    }

    myBsm[0] = bsm;
}
