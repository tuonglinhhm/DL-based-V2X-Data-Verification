///*******************************************************************************
//* @author  Joseph Kamel
//* @email   josephekamel@gmail.com
//* @date    28/11/2018
//* @version 1.1
//*
//* SCA (Secure Cooperative Autonomous systems)
//* Copyright (c) 2013, 2018 Institut de Recherche Technologique SystemX
//* All rights reserved.
//*******************************************************************************/

package lte.apps.f2md;
import lte.apps.f2md.F2MDBaseMode4Layer;

//
// network description file for your Veins Application. Add parameters here
//
simple JosephVeinsMode4 extends F2MDBaseMode4Layer
{
    @class(JosephVeinsMode4);
    string appName = default("Joseph Mb Veins App");
    

    //Simulation Parameters
    string serialNumber = default("IRT-DEMO");
    string savePath = default("../../../../mdmSave/");

    bool veremiConf = default(true);
    bool randomConf = default(false);
    bool variableConf = default(false);
    double confPos = default(10.0);
    double confSpd = default(0.05);
    double confHea = default(20.0);
    double confAcc = default(0.01);
    double confPrec = default(100.0);
    double minConf = default(0.20); 

    double SAVE_PERIOD = default(10); 
    double PRINT_PERIOD = default(60);

    double START_SAVE = default(0); 
    double START_ATTACK = default(10);

    int REPORT_VERSION = default(2);

    bool UseAttacksServer = default(false);
    bool MixLocalAttacks = default(true);
    bool RandomLocalMix = default(false);

    double LOCAL_ATTACKER_PROB = default(0.05);

    int LOCAL_ATTACK_TYPE = default(7);

    double GLOBAL_ATTACKER_PROB = default(0.0);
    int GLOBAL_ATTACK_TYPE = default(20);

    bool EnablePC = default(false);
    int PC_TYPE = default(5);

    bool EnableV1 = default(true);
    bool EnableV2 = default(true);
    bool SaveStatsV1 = default(true);
    bool SaveStatsV2 = default(true);

    int checksVersionV1 = default(2);
    int checksVersionV2 = default(2);

    int appTypeV1 = default(0);
    int appTypeV2 = default(2);

    bool writeSelfMsg = default(false);
    bool writeListSelfMsg = default(false);

    bool writeBsmsV1 = default(false);
    bool writeBsmsV2 = default(false);
    bool writeListBsmsV1 = default(false);
    bool writeListBsmsV2 = default(false);

    bool writeReportsV1 = default(false);
    bool writeReportsV2 = default(false);
    bool writeListReportsV1 = default(false);
    bool writeListReportsV2 = default(false);


    bool writeVeReMi = default(false);
    double VeReMiSliceTime = default(3600);

    bool sendReportsV1 = default(false);
    bool sendReportsV2 = default(false);
    string maHostV1 = default("localhost");
    string maHostV2 = default("localhost");
    int maPortV1 = default(9980);
    int maPortV2 = default(9981);

    bool enableVarThreV1 = default(false);
    bool enableVarThreV2 = default(false);

    // ------ Detection Parameters -- Start 
    double MAX_CONFIDENCE_RANGE = default(10);
    double MAX_PLAUSIBLE_RANGE = default(420);
   	double MAX_TIME_DELTA = default(3.1);
	double MAX_DELTA_INTER = default(2.0);
	double MAX_SA_RANGE = default(210);
	double MAX_SA_TIME = default(2.1);
	double MAX_KALMAN_TIME = default(3.1);
	double KALMAN_POS_RANGE = default(1.0);
	double KALMAN_SPEED_RANGE = default(4.0);
	double KALMAN_MIN_POS_RANGE = default(4.0);
	double KALMAN_MIN_SPEED_RANGE = default(1.0);
	double MIN_MAX_SPEED = default(40);
	double MIN_MAX_ACCEL = default(3);
	double MIN_MAX_DECEL = default(4.5);
	double MAX_MGT_RNG = default(4);
	double MAX_MGT_RNG_DOWN = default(6.2);
	double MAX_MGT_RNG_UP = default(2.1);

	double MAX_BEACON_FREQUENCY = default(0.99);
	double MAX_DISTANCE_FROM_ROUTE = default(2);
	double MAX_NON_ROUTE_SPEED = default(-1);
	double MAX_HEADING_CHANGE = default(90);
	double DELTA_BSM_TIME = default(5);
	double DELTA_REPORT_TIME = default(5);
	double POS_HEADING_TIME = default(1.1);
	// ------ Detection Parameters -- End
	
	// ------ Storage Parameters -- Start
	double MAX_TARGET_TIME = default(2);
	double MAX_ACCUSED_TIME = default(2);
	// ------ Storage Parameters -- End

	// ------ Attacks Parameters -- Start
	double parVar = default(0.55);
	double RandomPosOffsetX = default(70.0);
	double RandomPosOffsetY = default(70.0);
	double RandomSpeedX = default(40.0);
	double RandomSpeedY = default(40.0);
	double RandomSpeedOffsetX = default(7.0);
	double RandomSpeedOffsetY = default(7.0);
	double RandomAccelX = default(2.0);
	double RandomAccelY = default(2.0);
	double StopProb = default(0.05);
	int StaleMessages_Buffer = default(60);
	int DosMultipleFreq = default(4);
	int DosMultipleFreqSybil = default(2);
	int ReplaySeqNum = default(6);
	int SybilVehNumber = default(5);
	bool SelfSybil = default(false);
	double SybilDistanceX = default(5);
	double SybilDistanceY = default(2);
	// ------ Attacks Parameters -- End

	// ------ Pseudonym Parameters -- Start
	double Period_Change_Time = default(240);
	int Tolerance_Buffer = default(10);
	double Period_Change_Distance = default(80);
	double Random_Change_Chance = default(0.1);
	// ------ Pseudonym Parameters -- End

	//------ Report Parameters -- Start
	int InitialHistory = default(5);
	double CollectionPeriod = default(10);
	double UntolerancePeriod = default(5);
	//------ Report Parameters -- End
    
}
