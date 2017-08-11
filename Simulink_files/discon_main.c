/*
 * Copyright 1994-2012 The MathWorks, Inc.
 *
 * File    : classic_main.c
 *
 * Abstract:
 *      A Generic "Real-Time (single tasking or pseudo-multitasking,
 *      statically allocated data)" main that runs under most
 *      operating systems.
 *
 *      This file may be a useful starting point when targeting a new
 *      processor or microcontroller.
 *
 *
 * Compiler specified defines:
 *	RT              - Required.
 *      MODEL=modelname - Required.
 *	NUMST=#         - Required. Number of sample times.
 *	NCSTATES=#      - Required. Number of continuous states.
 *      TID01EQ=1 or 0  - Optional. Only define to 1 if sample time task
 *                        id's 0 and 1 have equal rates.
 *      MULTITASKING    - Optional. (use MT for a synonym).
 *	SAVEFILE        - Optional (non-quoted) name of .mat file to create.
 *			  Default is <MODEL>.mat
 */

#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
FILE *pFile; //Added by JW to read the init file

#include "rtwtypes.h"
# include "rtmodel.h"
#include "rt_sim.h"
#include "rt_logging.h"
#ifdef UseMMIDataLogging
#include "rt_logging_mmi.h"
#endif

#include "ext_work.h"



/*=========*
 * Defines *
 *=========*/

#ifndef TRUE
#define FALSE (0)
#define TRUE  (1)
#endif

#ifndef EXIT_FAILURE
#define EXIT_FAILURE  1
#endif
#ifndef EXIT_SUCCESS
#define EXIT_SUCCESS  0
#endif

#define QUOTE1(name) #name
#define QUOTE(name) QUOTE1(name)    /* need to expand name    */

#ifndef RT
# error "must define RT"
#endif

#ifndef MODEL
# error "must define MODEL"
#endif

#ifndef NUMST
# error "must define number of sample times, NUMST"
#endif

#ifndef NCSTATES
# error "must define NCSTATES"
#endif

#ifndef SAVEFILE
# define MATFILE2(file) #file ".mat"
# define MATFILE1(file) MATFILE2(file)
# define MATFILE MATFILE1(MODEL)
#else
# define MATFILE QUOTE(SAVEFILE)
#endif

#define RUN_FOREVER -1.0

#define EXPAND_CONCAT(name1,name2) name1 ## name2
#define CONCAT(name1,name2) EXPAND_CONCAT(name1,name2)
#define RT_MODEL            CONCAT(MODEL,_rtModel)

#define EXPAND_CONCAT3(name1,name2,name3) name1 ## _ ## name2 ## . ## name3
#define CONCAT3(name1,name2,name3) EXPAND_CONCAT3(name1,name2,name3)
#define SIG_MODEL(suffix,name)     CONCAT3(MODEL,suffix,name)

#define NINT(a) ((a) >= 0.0 ? (int)((a)+0.5) : (int)((a)-0.5))
#define MIN(a,b) ((a)>(b)?(b):(a))

/*====================*
 * External functions *
 *====================*/
#ifdef __cplusplus

extern "C" {

#endif

extern RT_MODEL *MODEL(void);

extern void MdlInitializeSizes(void);
extern void MdlInitializeSampleTimes(void);
extern void MdlStart(void);
extern void MdlOutputs(int_T tid);
extern void MdlUpdate(int_T tid);
extern void MdlTerminate(void);

extern void __declspec(dllexport) __cdecl DISCON(float *avrSwap, int *aviFail, char *accInfile, char *avcOutname, char *avcMsg); 

#ifdef __cplusplus

}
#endif

#if NCSTATES > 0
#ifdef __cplusplus

extern "C" {

#endif
  extern void rt_ODECreateIntegrationData(RTWSolverInfo *si);
  extern void rt_ODEUpdateContinuousStates(RTWSolverInfo *si);
#ifdef __cplusplus

}
#endif

# define rt_CreateIntegrationData(S) \
    rt_ODECreateIntegrationData(rtmGetRTWSolverInfo(S));
# define rt_UpdateContinuousStates(S) \
    rt_ODEUpdateContinuousStates(rtmGetRTWSolverInfo(S));
# else
# define rt_CreateIntegrationData(S)  \
      rtsiSetSolverName(rtmGetRTWSolverInfo(S),"FixedStepDiscrete");
# define rt_UpdateContinuousStates(S) /* Do Nothing */
#endif


/*==================================*
 * Global data local to this module *
 *==================================*/

static struct {
  int_T    stopExecutionFlag;
  int_T    isrOverrun;
  int_T    overrunFlags[NUMST];
  int_T    eventFlags[NUMST];
  const    char_T *errmsg;
} GBLbuf;


#ifdef EXT_MODE
#  define rtExtModeSingleTaskUpload(S)                          \
   {                                                            \
        int stIdx;                                              \
        rtExtModeUploadCheckTrigger(rtmGetNumSampleTimes(S));   \
        for (stIdx=0; stIdx<NUMST; stIdx++) {                   \
            if (rtmIsSampleHit(S, stIdx, 0 /*unused*/)) {       \
                rtExtModeUpload(stIdx,rtmGetTaskTime(S,stIdx)); \
            }                                                   \
        }                                                       \
   }
#else
#  define rtExtModeSingleTaskUpload(S) /* Do nothing */
#endif

/*=================*
 * Local functions *
 *=================*/

/* Function: initiateController ===========================================
 *
 * Abstract:
 *      Initialize the controller of the compiled Matlab Simulink block.
 */
static RT_MODEL *S;
int initiateController(char *errorMsg) {
    const char *status;
	
    /****************************
     * Initialize global memory *
     ****************************/
    (void)memset(&GBLbuf, 0, sizeof(GBLbuf));

    /************************
     * Initialize the model *
     ************************/

    S = MODEL();
    if (rtmGetErrorStatus(S) != NULL) {
        (void)fprintf(stderr,"Error during model registration: %s\n",
                      rtmGetErrorStatus(S));
        exit(EXIT_FAILURE);
    }
    rtmSetTFinal(S,RUN_FOREVER);

    MdlInitializeSizes();
    MdlInitializeSampleTimes();
    
    status = rt_SimInitTimingEngine(rtmGetNumSampleTimes(S),
                                    rtmGetStepSize(S),
                                    rtmGetSampleTimePtr(S),
                                    rtmGetOffsetTimePtr(S),
                                    rtmGetSampleHitPtr(S),
                                    rtmGetSampleTimeTaskIDPtr(S),
                                    rtmGetTStart(S),
                                    &rtmGetSimTimeStep(S),
                                    &rtmGetTimingData(S));

    if (status != NULL) {
        (void)fprintf(stderr,
                "Failed to initialize sample time engine: %s\n", status);
        exit(EXIT_FAILURE);
    }
    rt_CreateIntegrationData(S);

#ifdef UseMMIDataLogging
    rt_FillStateSigInfoFromMMI(rtmGetRTWLogInfo(S),&rtmGetErrorStatus(S));
#endif
    GBLbuf.errmsg = rt_StartDataLogging(rtmGetRTWLogInfo(S),
                                        rtmGetTFinal(S),
                                        rtmGetStepSize(S),
                                        &rtmGetErrorStatus(S));
    if (GBLbuf.errmsg != NULL) {
        (void)fprintf(stderr,"Error starting data logging: %s\n",GBLbuf.errmsg);
        return(EXIT_FAILURE);
    }

    rtExtModeCheckInit(rtmGetNumSampleTimes(S));
    rtExtModeWaitForStartPkt(rtmGetRTWExtModeInfo(S),
                             rtmGetNumSampleTimes(S),
                             (boolean_T *)&rtmGetStopRequested(S));

    (void)printf("\n** Starting the controller **\n");

    MdlStart();
    if (rtmGetErrorStatus(S) != NULL) {
      GBLbuf.stopExecutionFlag = 1;
    }
    
    return 0;
}  /* end initiateController */

#if !defined(MULTITASKING)  /* SINGLETASKING */
int calcOutputController(float rUserVar1, float rUserVar2, float rUserVar3, float rUserVar4, float rUserVar5,float rUserVar6, float rUserVar7, float rUserVar8, float rUserVar9, float rUserVar10,
						 float rUserVar11,float rUserVar12,float rUserVar13,float rUserVar14,float rUserVar15,float rUserVar16,float rUserVar17,float rUserVar18,float rUserVar19,float rUserVar20,
		float rInit, float rGeneratorSpeed, float rRatedSpeed,
        float rBelowRatedPitch, float rForeAftTower, float rSideTower,
        float rRotorAzimuth, float rOP1RootMoment, float rOP2RootMoment,
        float rOP3RootMoment,float rIP1RootMoment, float rIP2RootMoment,
        float rIP3RootMoment, float rMeasuredPitch, float rMeasuredTorque, float rShaftTorque,
        float rModeGain, float rYawError, float rYawBearingRate, float rElectricalPower, float *rTorqueDemand, float *rBlade1Pitch, 
        float *rBlade2Pitch, float *rBlade3Pitch, float *rPitchDemand, char *errorMsg, float* rYawRate,
        float* rLog1, float* rLog2,  float* rLog3,  float* rLog4,  float* rLog5,  float* rLog6,  float* rLog7,  float* rLog8,  float* rLog9,  float* rLog10,
		float* rLog11, float* rLog12,  float* rLog13,  float* rLog14,  float* rLog15,  float* rLog16,  float* rLog17,  float* rLog18,  float* rLog19,  float* rLog20) {
    real_T tnext;
    
    SIG_MODEL(U,Generator_Speed) = rGeneratorSpeed;
    SIG_MODEL(U,Rated_Speed) = rRatedSpeed;
    SIG_MODEL(U,Below_Rated_Pitch_Angle) = rBelowRatedPitch;
    SIG_MODEL(U,Fore_Aft_Tower_Accel) = rForeAftTower;
    SIG_MODEL(U,Sidewards_Tower_Accel) = rSideTower;
    SIG_MODEL(U,Measured_Pitch) = rMeasuredPitch;
    SIG_MODEL(U,Measured_Torque) = rMeasuredTorque;
	SIG_MODEL(U,Shaft_Torque) = rShaftTorque;
    SIG_MODEL(U,Mode_Gain) = rModeGain;
    SIG_MODEL(U,Rotor_Azimuth_Angle) = rRotorAzimuth;
    SIG_MODEL(U,Blade1_OP_Root_Moment) = rOP1RootMoment;
    SIG_MODEL(U,Blade2_OP_Root_Moment) = rOP2RootMoment;
    SIG_MODEL(U,Blade3_OP_Root_Moment) = rOP3RootMoment;
    SIG_MODEL(U,Blade1_IP_Root_Moment) = rIP1RootMoment;
    SIG_MODEL(U,Blade2_IP_Root_Moment) = rIP2RootMoment;
    SIG_MODEL(U,Blade3_IP_Root_Moment) = rIP3RootMoment;
	
	SIG_MODEL(U,Init) = rInit;
	SIG_MODEL(U,userVar1) = rUserVar1;
	SIG_MODEL(U,userVar2) = rUserVar2;
	SIG_MODEL(U,userVar3) = rUserVar3;
	SIG_MODEL(U,userVar4) = rUserVar4;
	SIG_MODEL(U,userVar5) = rUserVar5;
	SIG_MODEL(U,userVar6) = rUserVar6;
	SIG_MODEL(U,userVar7) = rUserVar7;
	SIG_MODEL(U,userVar8) = rUserVar8;
	SIG_MODEL(U,userVar9) = rUserVar9;
	SIG_MODEL(U,userVar10) = rUserVar10;
	SIG_MODEL(U,userVar11) = rUserVar11;
	SIG_MODEL(U,userVar12) = rUserVar12;
	SIG_MODEL(U,userVar13) = rUserVar13;
	SIG_MODEL(U,userVar14) = rUserVar14;
	SIG_MODEL(U,userVar15) = rUserVar15;
	SIG_MODEL(U,userVar16) = rUserVar16;
	SIG_MODEL(U,userVar17) = rUserVar17;
	SIG_MODEL(U,userVar18) = rUserVar18;
	SIG_MODEL(U,userVar19) = rUserVar19;
	SIG_MODEL(U,userVar20) = rUserVar20;
	SIG_MODEL(U,YawError) = rYawError;
	SIG_MODEL(U,YawBearingRate) = rYawBearingRate;
	SIG_MODEL(U,ElectricalPower) = rElectricalPower;
	/***********************************************
     * Check and see if base step time is too fast *
     ***********************************************/
    if (GBLbuf.isrOverrun++) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }

    /***********************************************
     * Check and see if error status has been set  *
     ***********************************************/
    if (rtmGetErrorStatus(S) != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }
    
    /* enable interrupts here */
    
    
    tnext = rt_SimGetNextSampleHit();
    rtsiSetSolverStopTime(rtmGetRTWSolverInfo(S),tnext);

    MdlOutputs(0);

    rtExtModeSingleTaskUpload(S);

    GBLbuf.errmsg = rt_UpdateTXYLogVars(rtmGetRTWLogInfo(S),
                                        rtmGetTPtr(S));
    if (GBLbuf.errmsg != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }

    MdlUpdate(0);
    rt_SimUpdateDiscreteTaskSampleHits(rtmGetNumSampleTimes(S),
                                       rtmGetTimingData(S),
                                       rtmGetSampleHitPtr(S),
                                       rtmGetTPtr(S));

    if (rtmGetSampleTime(S,0) == CONTINUOUS_SAMPLE_TIME) {
        rt_UpdateContinuousStates(S);
    }

    GBLbuf.isrOverrun--;

    rtExtModeCheckEndTrigger();

    
    rTorqueDemand[0] = SIG_MODEL(Y,Generator_Torque);
    rBlade1Pitch[0] = SIG_MODEL(Y,Blade1_Pitch_Angle);
    rBlade2Pitch[0] = SIG_MODEL(Y,Blade2_Pitch_Angle);
    rBlade3Pitch[0] = SIG_MODEL(Y,Blade3_Pitch_Angle);
	rPitchDemand[0] = SIG_MODEL(Y, Collective_Pitch_Angle);
	rYawRate[0] = SIG_MODEL(Y,Yaw_Rate);
    rLog1[0] = SIG_MODEL(Y,Log1);
    rLog2[0] = SIG_MODEL(Y,Log2);
    rLog3[0] = SIG_MODEL(Y,Log3);
    rLog4[0] = SIG_MODEL(Y,Log4);
    rLog5[0] = SIG_MODEL(Y,Log5);
    rLog6[0] = SIG_MODEL(Y,Log6);
    rLog7[0] = SIG_MODEL(Y,Log7);
    rLog8[0] = SIG_MODEL(Y,Log8);
    rLog9[0] = SIG_MODEL(Y,Log9);
    rLog10[0] = SIG_MODEL(Y,Log10);
	rLog11[0] = SIG_MODEL(Y,Log11);
	rLog12[0] = SIG_MODEL(Y,Log12);
	rLog13[0] = SIG_MODEL(Y,Log13);
	rLog14[0] = SIG_MODEL(Y,Log14);
	rLog15[0] = SIG_MODEL(Y,Log15);
	rLog16[0] = SIG_MODEL(Y,Log16);
	rLog17[0] = SIG_MODEL(Y,Log17);
	rLog18[0] = SIG_MODEL(Y,Log18);
	rLog19[0] = SIG_MODEL(Y,Log19);
	rLog20[0] = SIG_MODEL(Y,Log20);


    return 0;
}  /* end calcOutputController */

#else /* MULTITASKING */

# if TID01EQ == 1
#  define FIRST_TID 1
# else
#  define FIRST_TID 0
# endif

int calcOutputController(float rUserVar1, float rUserVar2, float rUserVar3, float rUserVar4, float rUserVar5,float rUserVar6, float rUserVar7, float rUserVar8, float rUserVar9, float rUserVar10,
						 float rUserVar11,float rUserVar12,float rUserVar13,float rUserVar14,float rUserVar15,float rUserVar16,float rUserVar17,float rUserVar18,float rUserVar19,float rUserVar20,
		float rInit, float rGeneratorSpeed, float rRatedSpeed,
        float rBelowRatedPitch, float rForeAftTower, float rSideTower,
        float rRotorAzimuth, float rOP1RootMoment, float rOP2RootMoment,
        float rOP3RootMoment,float rIP1RootMoment, float rIP2RootMoment,
        float rIP3RootMoment, float rMeasuredPitch, float rMeasuredTorque, float rShaftTorque,
        float rModeGain, float rYawError, float rYawBearingRate, float rElectricalPower, float *rTorqueDemand, float *rBlade1Pitch, 
        float *rBlade2Pitch, float *rBlade3Pitch, float *rPitchDemand, char *errorMsg, float* rYawRate,
        float* rLog1, float* rLog2,  float* rLog3,  float* rLog4,  float* rLog5,  float* rLog6,  float* rLog7,  float* rLog8,  float* rLog9,  float* rLog10,
		float* rLog11, float* rLog12,  float* rLog13,  float* rLog14,  float* rLog15,  float* rLog16,  float* rLog17,  float* rLog18,  float* rLog19,  float* rLog20) {
    int_T  i;
    real_T tnext;
    int_T  *sampleHit = rtmGetSampleHitPtr(S);
    
    SIG_MODEL(U,Generator_Speed) = rGeneratorSpeed;
    SIG_MODEL(U,Rated_Speed) = rRatedSpeed;
    SIG_MODEL(U,Below_Rated_Pitch_Angle) = rBelowRatedPitch;
    SIG_MODEL(U,Fore_Aft_Tower_Accel) = rForeAftTower;
    SIG_MODEL(U,Sidewards_Tower_Accel) = rSideTower;
    SIG_MODEL(U,Measured_Pitch) = rMeasuredPitch;
    SIG_MODEL(U,Measured_Torque) = rMeasuredTorque;
	SIG_MODEL(U,Shaft_Torque) = rShaftTorque;
    SIG_MODEL(U,Mode_Gain) = rModeGain;
    SIG_MODEL(U,Rotor_Azimuth_Angle) = rRotorAzimuth;
    SIG_MODEL(U,Blade1_OP_Root_Moment) = rOP1RootMoment;
    SIG_MODEL(U,Blade2_OP_Root_Moment) = rOP2RootMoment;
    SIG_MODEL(U,Blade3_OP_Root_Moment) = rOP3RootMoment;
    SIG_MODEL(U,Blade1_IP_Root_Moment) = rIP1RootMoment;
    SIG_MODEL(U,Blade2_IP_Root_Moment) = rIP2RootMoment;
    SIG_MODEL(U,Blade3_IP_Root_Moment) = rIP3RootMoment;
	
	SIG_MODEL(U,Init) = rInit;
	SIG_MODEL(U,userVar1) = rUserVar1;
	SIG_MODEL(U,userVar2) = rUserVar2;
	SIG_MODEL(U,userVar3) = rUserVar3;
	SIG_MODEL(U,userVar4) = rUserVar4;
	SIG_MODEL(U,userVar5) = rUserVar5;
	SIG_MODEL(U,userVar6) = rUserVar6;
	SIG_MODEL(U,userVar7) = rUserVar7;
	SIG_MODEL(U,userVar8) = rUserVar8;
	SIG_MODEL(U,userVar9) = rUserVar9;
	SIG_MODEL(U,userVar10) = rUserVar10;
	SIG_MODEL(U,userVar11) = rUserVar11;
	SIG_MODEL(U,userVar12) = rUserVar12;
	SIG_MODEL(U,userVar13) = rUserVar13;
	SIG_MODEL(U,userVar14) = rUserVar14;
	SIG_MODEL(U,userVar15) = rUserVar15;
	SIG_MODEL(U,userVar16) = rUserVar16;
	SIG_MODEL(U,userVar17) = rUserVar17;
	SIG_MODEL(U,userVar18) = rUserVar18;
	SIG_MODEL(U,userVar19) = rUserVar19;
	SIG_MODEL(U,userVar20) = rUserVar20;
	SIG_MODEL(U,YawError) = rYawError;
	SIG_MODEL(U,YawBearingRate) = rYawBearingRate;
	SIG_MODEL(U,ElectricalPower) = rElectricalPower;
	/***********************************************
     * Check and see if base step time is too fast *
     ***********************************************/
    if (GBLbuf.isrOverrun++) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }

    /***********************************************
     * Check and see if error status has been set  *
     ***********************************************/
    if (rtmGetErrorStatus(S) != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }
    /* enable interrupts here */

    /***********************************************
     * Update discrete events                      *
     ***********************************************/
    tnext = rt_SimUpdateDiscreteEvents(rtmGetNumSampleTimes(S),
                                       rtmGetTimingData(S),
                                       rtmGetSampleHitPtr(S),
                                       rtmGetPerTaskSampleHitsPtr(S));
    rtsiSetSolverStopTime(rtmGetRTWSolverInfo(S),tnext);
    for (i=FIRST_TID+1; i < NUMST; i++) {
        if (sampleHit[i] && GBLbuf.eventFlags[i]++) {
            GBLbuf.isrOverrun--; 
            GBLbuf.overrunFlags[i]++;    /* Are we sampling too fast for */
            GBLbuf.stopExecutionFlag=1;  /*   sample time "i"?           */
            return;
        }
    }
    /*******************************************
     * Step the model for the base sample time *
     *******************************************/
    MdlOutputs(FIRST_TID);

    rtExtModeUploadCheckTrigger(rtmGetNumSampleTimes(S));
    rtExtModeUpload(FIRST_TID,rtmGetTaskTime(S, FIRST_TID));

    GBLbuf.errmsg = rt_UpdateTXYLogVars(rtmGetRTWLogInfo(S),
                                        rtmGetTPtr(S));
    if (GBLbuf.errmsg != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }

    MdlUpdate(FIRST_TID);

    if (rtmGetSampleTime(S,0) == CONTINUOUS_SAMPLE_TIME) {
        rt_UpdateContinuousStates(S);
    }
     else {
        rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(S), 
                                     rtmGetTimingData(S), 0);
    }

#if FIRST_TID == 1
    rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(S), 
                                 rtmGetTimingData(S),1);
#endif


    /************************************************************************
     * Model step complete for base sample time, now it is okay to          *
     * re-interrupt this ISR.                                               *
     ************************************************************************/

    GBLbuf.isrOverrun--;


    /*********************************************
     * Step the model for any other sample times *
     *********************************************/
    for (i=FIRST_TID+1; i<NUMST; i++) {
        /* If task "i" is running, don't run any lower priority task */
        if (GBLbuf.overrunFlags[i]) return;

        if (GBLbuf.eventFlags[i]) {
            GBLbuf.overrunFlags[i]++;

            MdlOutputs(i);
 
            rtExtModeUpload(i, rtmGetTaskTime(S,i));

            MdlUpdate(i);

            rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(S), 
                                         rtmGetTimingData(S),i);

            /* Indicate task complete for sample time "i" */
            GBLbuf.overrunFlags[i]--;
            GBLbuf.eventFlags[i]--;
        }
    }

    rtExtModeCheckEndTrigger();

    
    rTorqueDemand[0] = SIG_MODEL(Y,Generator_Torque);
    rBlade1Pitch[0] = SIG_MODEL(Y,Blade1_Pitch_Angle);
    rBlade2Pitch[0] = SIG_MODEL(Y,Blade2_Pitch_Angle);
    rBlade3Pitch[0] = SIG_MODEL(Y,Blade3_Pitch_Angle);
	rPitchDemand[0] = SIG_MODEL(Y, Collective_Pitch_Angle);
	rYawRate[0] = SIG_MODEL(Y,Yaw_Rate);
    rLog1[0] = SIG_MODEL(Y,Log1);
    rLog2[0] = SIG_MODEL(Y,Log2);
    rLog3[0] = SIG_MODEL(Y,Log3);
    rLog4[0] = SIG_MODEL(Y,Log4);
    rLog5[0] = SIG_MODEL(Y,Log5);
    rLog6[0] = SIG_MODEL(Y,Log6);
    rLog7[0] = SIG_MODEL(Y,Log7);
    rLog8[0] = SIG_MODEL(Y,Log8);
    rLog9[0] = SIG_MODEL(Y,Log9);
    rLog10[0] = SIG_MODEL(Y,Log10);
	rLog11[0] = SIG_MODEL(Y,Log11);
	rLog12[0] = SIG_MODEL(Y,Log12);
	rLog13[0] = SIG_MODEL(Y,Log13);
	rLog14[0] = SIG_MODEL(Y,Log14);
	rLog15[0] = SIG_MODEL(Y,Log15);
	rLog16[0] = SIG_MODEL(Y,Log16);
	rLog17[0] = SIG_MODEL(Y,Log17);
	rLog18[0] = SIG_MODEL(Y,Log18);
	rLog19[0] = SIG_MODEL(Y,Log19);
	rLog20[0] = SIG_MODEL(Y,Log20);


    return 0;
}  /* end calcOutputController */

#endif /* MULTITASKING */

/* Function: performCleanup ===============================================
 *
 * Abstract:
 *      Execute model on a generic target such as a workstation.
 */
int performCleanup(char *errorMsg) {
    
	#ifdef UseMMIDataLogging
		rt_CleanUpForStateLogWithMMI(rtmGetRTWLogInfo(S));
	#endif
    rt_StopDataLogging(MATFILE, rtmGetRTWLogInfo(S));
    
    rtExtModeShutdown(rtmGetNumSampleTimes(S));
    
    if (GBLbuf.errmsg) {
        (void)fprintf(stderr,"%s\n",GBLbuf.errmsg);
        exit(EXIT_FAILURE);
    }
    
    if (rtmGetErrorStatus(S) != NULL) {
        (void)fprintf(stderr,"ErrorStatus set: \"%s\"\n", rtmGetErrorStatus(S));
        exit(EXIT_FAILURE);
    }
    
    if (GBLbuf.isrOverrun) {
        (void)fprintf(stderr,
                      "%s: ISR overrun - base sampling rate is too fast\n",
                      QUOTE(MODEL));
        exit(EXIT_FAILURE);
    }
	
	#ifdef MULTITASKING
    else {
        int_T i;
        for (i=1; i<NUMST; i++) {
            if (GBLbuf.overrunFlags[i]) {
                (void)fprintf(stderr,
                        "%s ISR overrun - sampling rate is too fast for "
                        "sample time index %d\n", QUOTE(MODEL), i);
                exit(EXIT_FAILURE);
            }
        }
    }
	#endif
    
    sprintf(errorMsg, "** Stopping the controller **");
    MdlTerminate();
    
    return 0;
}  /* end performCleanup */

static void displayUsage (void)
{
    (void) printf("usage: %s -tf <finaltime> -w -port <TCPport>\n",QUOTE(MODEL));
    (void) printf("arguments:\n");
    (void) printf("  -tf <finaltime> - overrides final time specified in "
                  "Simulink (inf for no limit).\n");
    (void) printf("  -w              - waits for Simulink to start model "
                  "in External Mode.\n");
    (void) printf("  -port <TCPport> - overrides 17725 default port in "
                  "External Mode, valid range 256 to 65535.\n");
}

/* This function is added by JW to read the external inputs from Bladed */
float* SetParams(float *avrSwap) 
{
	char mystring [200];
	int iStatus;

	iStatus	=	NINT(avrSwap[0]);

	if (iStatus == 0)
	{
		pFile = fopen ("discon.in","r");
		if (pFile == NULL) {avrSwap[128]=1;}
		else {
			fgets (mystring , 200 , pFile);
			avrSwap[119]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[120]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[121]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[122]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[123]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[124]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[125]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[126]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[127]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[128]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[129]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[130]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[131]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[132]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[133]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[134]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[135]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[136]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[137]=atof(mystring);
			fgets (mystring , 200 , pFile);
			avrSwap[138]=atof(mystring);
			fclose (pFile);
			}
	}
	return(avrSwap);
}

/*===================*
 * Visible functions *
 *===================*/


/* Function: main =============================================================
 *
 * Abstract:
 *      Execute model on a generic target such as a workstation.
 */
void __declspec(dllexport) __cdecl DISCON(float *avrSwap, int *aviFail, char *accInfile, char *avcOutname, char *avcMsg) 
{
	int iStatus, iFirstLog;
	char errorMsg[257], OutName[1025];// inFile[257]; 
	float rTime, rSample, rGeneratorSpeed, rRatedSpeed, rBelowRatedPitch, 
			rRotorAzimuth, rOP1RootMoment, rOP2RootMoment, rOP3RootMoment, rIP1RootMoment, rIP2RootMoment, rIP3RootMoment,
			rForeAftTower, rSideTower, rMeasuredPitch, rMeasuredTorque, rShaftTorque,
			rModeGain, rInit, rUserVar1, rUserVar2, rUserVar3, rUserVar4, rUserVar5, rUserVar6, rUserVar7, rUserVar8, rUserVar9, rUserVar10,
			rUserVar11,rUserVar12,rUserVar13,rUserVar14,rUserVar15,rUserVar16,rUserVar17,rUserVar18,rUserVar19,rUserVar20,rYawError,rYawBearingRate,rElectricalPower;
	static float rTorqueDemand, rPitchDemand, rBlade1Pitch, rBlade2Pitch, 
			rBlade3Pitch, rYawRate, rLog1,rLog2,rLog3,rLog4,rLog5,rLog6,rLog7,rLog8,rLog9,rLog10,rLog11,rLog12,rLog13,rLog14,rLog15,rLog16,rLog17,rLog18,rLog19,rLog20;
	
	/* Take local copies of strings */
	//memcpy(inFile, accInfile, NINT(avrSwap[49]));
	//inFile[NINT(avrSwap[49])+1] = '\0';
	//memcpy(outName, avcOutname, NINT(avrSwap[50]));
	//outName[NINT(avrSwap[50])+1] = '\0';
	
	/* Set message to blank */
	memset(errorMsg, ' ', 257);
	
	/* Set constants JW turned this on, see function just above this call*/ 
	SetParams(avrSwap); /*PF disable this call for Labview's sake*/
	
	/* Load variables from Bladed (See Appendix A) */
	iStatus          = NINT(avrSwap[0]);
	rInit			= avrSwap[0];
	rUserVar1		= avrSwap[119];
	rUserVar2		= avrSwap[120];
	rUserVar3		= avrSwap[121];
	rUserVar4		= avrSwap[122];
	rUserVar5		= avrSwap[123];
	rUserVar6		= avrSwap[124];
	rUserVar7		= avrSwap[125];
	rUserVar8		= avrSwap[126];
	rUserVar9		= avrSwap[127];
	rUserVar10		= avrSwap[128];
	rUserVar11		= avrSwap[129];
	rUserVar12		= avrSwap[130];
	rUserVar13		= avrSwap[131];
	rUserVar14		= avrSwap[132];
	rUserVar15		= avrSwap[133];
	rUserVar16		= avrSwap[134];
	rUserVar17		= avrSwap[135];
	rUserVar18		= avrSwap[136];
	rUserVar19		= avrSwap[137];
	rUserVar20		= avrSwap[138];
	rTime            = avrSwap[1];
	rSample          = avrSwap[2];
	rMeasuredPitch   = avrSwap[3];
	rBelowRatedPitch = avrSwap[4];
	rModeGain        = avrSwap[15];
	rRatedSpeed      = avrSwap[18];
	rGeneratorSpeed  = avrSwap[19];
	rMeasuredTorque  = avrSwap[22]; /* this was number 22 but I believe it should be 108 but that is the LSS Torque */
	rShaftTorque	 = avrSwap[108];
	rOP1RootMoment   = avrSwap[29]; 
	rOP2RootMoment   = avrSwap[30];
	rOP3RootMoment   = avrSwap[31];
	rIP1RootMoment   = avrSwap[68]; 
	rIP2RootMoment   = avrSwap[69];
	rIP3RootMoment   = avrSwap[70];
	rForeAftTower    = avrSwap[52];
	rSideTower       = avrSwap[53];
	rRotorAzimuth    = avrSwap[59];
	rYawError		 = avrSwap[23];
	rYawBearingRate  = avrSwap[162];
	rElectricalPower = avrSwap[14];
	
    /* determine iStatus */
    aviFail[0] = 0;
    if (iStatus == 0) {
        
                /* Initialize Controller */
        aviFail[0] = initiateController(errorMsg);
        
                aviFail[0] = calcOutputController(rUserVar1, rUserVar2, rUserVar3, rUserVar4, rUserVar5,rUserVar6, rUserVar7, rUserVar8, rUserVar9, rUserVar10,
						rUserVar11,rUserVar12,rUserVar13,rUserVar14,rUserVar15,rUserVar16,rUserVar17,rUserVar18,rUserVar19,rUserVar20, rInit, rGeneratorSpeed, rRatedSpeed,
                        rBelowRatedPitch, rForeAftTower, rSideTower,
                        rRotorAzimuth, rOP1RootMoment, rOP2RootMoment,
                        rOP3RootMoment, rIP1RootMoment, rIP2RootMoment,
                        rIP3RootMoment, rMeasuredPitch, rMeasuredTorque, rShaftTorque,
                        rModeGain, rYawError, rYawBearingRate, rElectricalPower, &rTorqueDemand, &rBlade1Pitch, 
                        &rBlade2Pitch, &rBlade3Pitch, &rPitchDemand, &errorMsg, &rYawRate,
                        &rLog1,&rLog2,&rLog3,&rLog4,&rLog5,&rLog6,&rLog7,&rLog8,&rLog9,&rLog10,&rLog11,&rLog12,&rLog13,&rLog14,&rLog15,&rLog16,&rLog17,&rLog18,&rLog19,&rLog20);
        

        rPitchDemand = rMeasuredPitch;    
        rTorqueDemand = rMeasuredTorque;
		sprintf(errorMsg, "Controller initialization complete");
    }
    else if (iStatus >= 0) {
        /* Main calculation */
        aviFail[0] = calcOutputController(rUserVar1, rUserVar2, rUserVar3, rUserVar4, rUserVar5,rUserVar6, rUserVar7, rUserVar8, rUserVar9, rUserVar10,
						rUserVar11,rUserVar12,rUserVar13,rUserVar14,rUserVar15,rUserVar16,rUserVar17,rUserVar18,rUserVar19,rUserVar20, rInit, rGeneratorSpeed, rRatedSpeed,
                        rBelowRatedPitch, rForeAftTower, rSideTower,
                        rRotorAzimuth, rOP1RootMoment, rOP2RootMoment,
                        rOP3RootMoment, rIP1RootMoment, rIP2RootMoment,
                        rIP3RootMoment, rMeasuredPitch, rMeasuredTorque, rShaftTorque,
                        rModeGain, rYawError, rYawBearingRate, rElectricalPower, &rTorqueDemand, &rBlade1Pitch, 
                        &rBlade2Pitch, &rBlade3Pitch, &rPitchDemand, &errorMsg, &rYawRate,
                        &rLog1,&rLog2,&rLog3,&rLog4,&rLog5,&rLog6,&rLog7,&rLog8,&rLog9,&rLog10,&rLog11,&rLog12,&rLog13,&rLog14,&rLog15,&rLog16,&rLog17,&rLog18,&rLog19,&rLog20);
    }
    else if (iStatus == -1) {
        /* Main calculation */
        aviFail[0] = calcOutputController(rUserVar1, rUserVar2, rUserVar3, rUserVar4, rUserVar5,rUserVar6, rUserVar7, rUserVar8, rUserVar9, rUserVar10,
						rUserVar11,rUserVar12,rUserVar13,rUserVar14,rUserVar15,rUserVar16,rUserVar17,rUserVar18,rUserVar19,rUserVar20, rInit, rGeneratorSpeed, rRatedSpeed,
                        rBelowRatedPitch, rForeAftTower, rSideTower,
                        rRotorAzimuth, rOP1RootMoment, rOP2RootMoment,
                        rOP3RootMoment, rIP1RootMoment, rIP2RootMoment,
                        rIP3RootMoment, rMeasuredPitch, rMeasuredTorque, rShaftTorque,
                        rModeGain, rYawError, rYawBearingRate, rElectricalPower, &rTorqueDemand, &rBlade1Pitch, 
                        &rBlade2Pitch, &rBlade3Pitch, &rPitchDemand, &errorMsg, &rYawRate,
                        &rLog1,&rLog2,&rLog3,&rLog4,&rLog5,&rLog6,&rLog7,&rLog8,&rLog9,&rLog10,&rLog11,&rLog12,&rLog13,&rLog14,&rLog15,&rLog16,&rLog17,&rLog18,&rLog19,&rLog20);
        
        /* Perform Cleanup */
        aviFail[0] = performCleanup(errorMsg);
    }
    else {
        aviFail[0] = -1;
        sprintf(errorMsg, "iStatus is not recognized: %d", iStatus);
    }
    
    /* Store variables to Bladed (See Appendix A) */
    avrSwap[9] = 0; /* Pitch Angle */
	avrSwap[27] = 1; /* Individual Pitch control */
    avrSwap[34] = 1; /* Generator contactor status */
    avrSwap[35] = 0; /* Shaft brake status: 0=off */
    avrSwap[40] = 0; /* Demanded yaw actuator torque */
    avrSwap[41] = rBlade1Pitch;  /* Blade 1 pitch angle demand */
    avrSwap[42] = rBlade2Pitch;  /* Blade 2 pitch angle demand */
    avrSwap[43] = rBlade3Pitch;  /* Blade 3 pitch angle demand */
	avrSwap[44] = rPitchDemand;  /* Pitch angle demand CPC*/
    avrSwap[46] = rTorqueDemand; /* Generator torque demand */
    avrSwap[47] = rYawRate; /* Demanded nacelle yaw rate */
    avrSwap[54] = 0; /* Pitch override */
    avrSwap[55] = 0; /* Torque override */
	avrSwap[71] = 0; /* Generator start-up resistance */
    avrSwap[78] = 1; /* Request for loads: 0=none */
    avrSwap[79] = 0; /* Variable slip current status */
    avrSwap[80] = 0; /* Variable slip current demand */
	
	// To read the log variables in bladed (JW)
	avrSwap[64] =0; /* Number of variables returned for logging */
	iFirstLog = NINT(avrSwap[62])-1; //added also this iFirstLog as an integer
	strcpy(OutName, "Log1:-;Log2:-;Log3:-;Log4:-;Log5:-;Log6:-;Log7:-;Log8:-;Log9:-;Log10:-;Log11:-;Log12:-;Log13:-;Log14:-;Log15:-;Log16:-;Log17:-;Log18:-;Log19:-;Log20:-;"); //Names and units
    avrSwap[iFirstLog] =rLog1; /*avrSwap[144] = rLog1;*/
	avrSwap[iFirstLog+1] =rLog2; /*avrSwap[144] = rLog2;*/
	avrSwap[iFirstLog+2] =rLog3; /*avrSwap[144] = rLog3;*/
	avrSwap[iFirstLog+3] =rLog4; /*avrSwap[144] = rLog4;*/
	avrSwap[iFirstLog+4] =rLog5; /*avrSwap[144] = rLog5;*/
	avrSwap[iFirstLog+5] =rLog6; /*avrSwap[144] = rLog6;*/
	avrSwap[iFirstLog+6] =rLog7; /*avrSwap[144] = rLog7;*/
	avrSwap[iFirstLog+7] =rLog8; /*avrSwap[144] = rLog8;*/
	avrSwap[iFirstLog+8] =rLog9; /*avrSwap[144] = rLog9;*/
	avrSwap[iFirstLog+9] =rLog10; /*avrSwap[144] = rLog10;*/
	avrSwap[iFirstLog+10] =rLog11; /*avrSwap[144] = rLog11;*/
	avrSwap[iFirstLog+11] =rLog12; /*avrSwap[144] = rLog12;*/
	avrSwap[iFirstLog+12] =rLog13; /*avrSwap[144] = rLog13;*/
	avrSwap[iFirstLog+13] =rLog14; /*avrSwap[144] = rLog14;*/
	avrSwap[iFirstLog+14] =rLog15; /*avrSwap[144] = rLog15;*/
	avrSwap[iFirstLog+15] =rLog16; /*avrSwap[144] = rLog16;*/
	avrSwap[iFirstLog+16] =rLog17; /*avrSwap[144] = rLog17;*/
	avrSwap[iFirstLog+17] =rLog18; /*avrSwap[144] = rLog18;*/
	avrSwap[iFirstLog+18] =rLog19; /*avrSwap[144] = rLog19;*/
	avrSwap[iFirstLog+19] =rLog20; /*avrSwap[144] = rLog20;*/

    //Return strings
	memcpy(avcOutname,OutName, NINT(avrSwap[63]));
	memcpy(avcMsg,errorMsg,MIN(256,NINT(avrSwap[48])));
	
  return;
}
		  /* end DISON */



/* EOF: discon_main.c */
