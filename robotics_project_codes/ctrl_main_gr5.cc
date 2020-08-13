/*! 
 * \file ctrl_main_gr5.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

#include "ctrl_main_gr5.h"
#include "namespace_ctrl.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//------------ FSM ---------------
#define START_CALIBRATION 0
#define TARGET1           1
#define TARGET2           2
#define TARGET3           3
#define TARGET4           4
#define GOTOBASE          5
#define WAIT              6
#define AVOIDANCE         7
//--------------------------------


NAMESPACE_INIT(ctrlGr5);



/*! \brief initialize controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */
void controller_init(CtrlStruct *cvs)
{
    printf("Hello gr5 \n");
    
    CtrlIn *ivs;
    CtrlOut *ovs;
    CtrlState *svs;
    
    ivs = cvs->inputs;
    ovs = cvs->outputs;
    svs = cvs->states;
    
    ovs-> wheel_commands[R_ID]  = 0;//right wheel speed output
    ovs-> wheel_commands[L_ID]  = 0;//left wheel speed output
    
    svs-> Rwheel                       = 0.03;
    svs-> b                            = 0.225;
    
    svs-> integralR                    = 0 ;//KI for right
    svs-> integralL                    = 0; //KI for left
    svs-> eR_Old                       = 0; //old error right wheel
    svs-> eL_Old                       = 0; //old error left wheel
    svs-> eR_New                       = 0; //new error right wheel
    svs-> eL_New                       = 0 ;//new error left wheel

    svs-> time                         = ivs->t ;// present time
    svs-> deltaTime                    = 0 ;//new time - old time
    svs-> calibrationIsDone            = false;
    svs-> Xcalibration                 = false;
    svs-> Ycalibration                 = false;
    svs-> YstopCalibration             = false;
    svs-> XstopCalibration             = false;
	svs-> Tcalibration                 = false;
    svs-> omegaRrefCalibration         = -4;
    svs-> omegaLrefCalibration         = -4;
    svs-> countCalibration             = 0;
	svs-> rho                          = 14.0; //
    
	svs-> errorRold                    = 0.0;   // Old error in speed of right wheel
	svs-> errorLold                    = 0.0;   // Old error in speed of left wheel
	svs-> intErrorR                    = 0.0;
	svs-> intErrorR                    = 0.0;
	svs-> timeOld                      = 0.0;  // Old time
	svs-> dt                           = 0.0;  // Time 
	svs-> pFileSpeed                   = fopen("speed.txt", "w");
	svs-> pFileTime                    = fopen("time.txt", "w");
	svs-> pFileXodo                    = fopen("Xodo.txt","w");
	svs-> pFileYodo                    = fopen("Yodo.txt","w");
	svs-> pFileThetaOdo                = fopen("ThetaOdo.txt","w");
	svs-> pFileXtri                    = fopen("Xtri.txt","w");
	svs-> pFileYtri                    = fopen("Ytri.txt","w");
	svs-> pFileThetaTri                = fopen("ThetaTri.txt","w");
	svs-> pFileDisF                    = fopen("DisF.txt","w");
	svs-> pFileDis                     = fopen("Dis.txt","w");
	
    
    svs->pos_theta_odo                 = 0;
	svs->pos_theta_odoNW               = 0;
    svs->pos_x_odo                     = 0;
    svs->pos_y_odo                     = 0;
    
	svs->countDiscontinuity            = 0;
	svs->RobotState                    = START_CALIBRATION; //Start FSM
	
	svs->onTarget                      = false;       
	svs->indexPath                     = 1;
	svs->onNode                        = false;
	svs->t0                            = 0.0;
	svs->waitIsDone                    = false;
	svs->indexWait                     = 0;
	svs->previousState                 = 0;
	
	svs->distoPponentold               = 0.0;
	svs->distoPponent                  = 0.0;
	svs->cntOpp                        = 0.0;
	svs->sumOpp                        = 0.0;
	
	//-------------------------------------------KALMAN---------------------------
    //initialization of p at t=0
    svs->P[0][0]                        = 0.2*0.2;
    svs->P[0][1]                        = 0;
    svs->P[0][2]                        = 0;
    svs->P[1][0]                        = 0;
    svs->P[1][1]                        = 0.2*0.2;
    svs->P[1][2]                        = 0;
    svs->P[2][0]                        = 0;
    svs->P[2][1]                        = 0;
    svs->P[2][2]                        = 10*10;

    svs->Odo_kal[0][0]                        = 0;
    svs->Odo_kal[1][0]                        = 0;
    svs->Odo_kal[2][0]                        = 0;

    svs->Fp_PFpt[0][0]                        = 0;
    svs->Fp_PFpt[0][1]                        = 0;
    svs->Fp_PFpt[0][2]                        = 0;
    svs->Fp_PFpt[1][0]                        = 0;
    svs->Fp_PFpt[1][1]                        = 0;
    svs->Fp_PFpt[1][2]                        = 0;
    svs->Fp_PFpt[2][0]                        = 0;
    svs->Fp_PFpt[2][1]                        = 0;
    svs->Fp_PFpt[2][2]                        = 0;

    svs->PFpt[0][0]                        = 0;
    svs->PFpt[0][1]                        = 0;
    svs->PFpt[0][2]                        = 0;
    svs->PFpt[1][0]                        = 0;
    svs->PFpt[1][1]                        = 0;
    svs->PFpt[1][2]                        = 0;
    svs->PFpt[2][0]                        = 0;
    svs->PFpt[2][1]                        = 0;
    svs->PFpt[2][2]                        = 0;

    svs->QFut[0][0]                        = 0;
    svs->QFut[0][1]                        = 0;
    svs->QFut[0][2]                        = 0;
    svs->QFut[1][0]                        = 0;
    svs->QFut[1][1]                        = 0;
    svs->QFut[1][2]                        = 0;


     svs->Fu_QFut[0][0]                        = 0;
     svs->Fu_QFut[0][1]                        = 0;
     svs->Fu_QFut[0][2]                        = 0;
     svs->Fu_QFut[1][0]                        = 0;
     svs->Fu_QFut[1][1]                        = 0;
     svs->Fu_QFut[1][2]                        = 0;
     svs->Fu_QFut[2][0]                        = 0;
     svs->Fu_QFut[2][1]                        = 0;
     svs->Fu_QFut[2][2]                        = 0;

     svs->PHt[0][0]                        = 0;
     svs->PHt[0][1]                        = 0;
     svs->PHt[0][2]                        = 0;
     svs->PHt[1][0]                        = 0;
     svs->PHt[1][1]                        = 0;
     svs->PHt[1][2]                        = 0;
     svs->PHt[2][0]                        = 0;
     svs->PHt[2][1]                        = 0;
     svs->PHt[2][2]                        = 0;

     svs->HPHt[0][0]                        = 0;
     svs->HPHt[0][1]                        = 0;
     svs->HPHt[0][2]                        = 0;
     svs->HPHt[1][0]                        = 0;
     svs->HPHt[1][1]                        = 0;
     svs->HPHt[1][2]                        = 0;
     svs->HPHt[2][0]                        = 0;
     svs->HPHt[2][1]                        = 0;
     svs->HPHt[2][2]                        = 0;

     svs->S[0][0]                        = 0;
     svs->S[0][1]                        = 0;
     svs->S[0][2]                        = 0;
     svs->S[1][0]                        = 0;
     svs->S[1][1]                        = 0;
     svs->S[1][2]                        = 0;
     svs->S[2][0]                        = 0;
     svs->S[2][1]                        = 0;
     svs->S[2][2]                        = 0;

     svs->invS[0][0]                        = 0;
     svs->invS[0][1]                        = 0;
     svs->invS[0][2]                        = 0;
     svs->invS[1][0]                        = 0;
     svs->invS[1][1]                        = 0;
     svs->invS[1][2]                        = 0;
     svs->invS[2][0]                        = 0;
     svs->invS[2][1]                        = 0;
     svs->invS[2][2]                        = 0;

     svs->K[0][0]                        = 0;
     svs->K[0][1]                        = 0;
     svs->K[0][2]                        = 0;
     svs->K[1][0]                        = 0;
     svs->K[1][1]                        = 0;
     svs->K[1][2]                        = 0;
     svs->K[2][0]                        = 0;
     svs->K[2][1]                        = 0;
     svs->K[2][2]                        = 0;

     svs->HtinvS[0][0]                        = 0;
     svs->HtinvS[0][1]                        = 0;
     svs->HtinvS[0][2]                        = 0;
     svs->HtinvS[1][0]                        = 0;
     svs->HtinvS[1][1]                        = 0;
     svs->HtinvS[1][2]                        = 0;
     svs->HtinvS[2][0]                        = 0;
     svs->HtinvS[2][1]                        = 0;
     svs->HtinvS[2][2]                        = 0;

     svs->error[0][0]                        = 0;
     svs->error[1][0]                        = 0;
     svs->error[2][0]                        = 0;

     svs->Kerror[0][0]                        = 0;
     svs->Kerror[1][0]                        = 0;
     svs->Kerror[2][0]                        = 0;

     svs->newX[0][0]                        = 0;
     svs->newX[1][0]                        = 0;
     svs->newX[2][0]                        = 0;

     svs->KH[0][0]                        = 0;
     svs->KH[0][1]                        = 0;
     svs->KH[0][2]                        = 0;
     svs->KH[1][0]                        = 0;
     svs->KH[1][1]                        = 0;
     svs->KH[1][2]                        = 0;
     svs->KH[2][0]                        = 0;
     svs->KH[2][1]                        = 0;
     svs->KH[2][2]                        = 0;

     svs->I_KH[0][0]                        = 0;
     svs->I_KH[0][1]                        = 0;
     svs->I_KH[0][2]                        = 0;
     svs->I_KH[1][0]                        = 0;
     svs->I_KH[1][1]                        = 0;
     svs->I_KH[1][2]                        = 0;
     svs->I_KH[2][0]                        = 0;
     svs->I_KH[2][1]                        = 0;
     svs->I_KH[2][2]                        = 0;

     svs->newP[0][0]                        = 0;
     svs->newP[0][1]                        = 0;
     svs->newP[0][2]                        = 0;
     svs->newP[1][0]                        = 0;
     svs->newP[1][1]                        = 0;
     svs->newP[1][2]                        = 0;
     svs->newP[2][0]                        = 0;
     svs->newP[2][1]                        = 0;
     svs->newP[2][2]                        = 0;

//----------------------------------------------------------------------
	
   
}

/*! \brief controller loop (called every timestep)
 * 
 * \param[in] cvs controller main structure
 */
//double distanceOfOponents(CtrlStruct *cvs);
 
void controller_loop(CtrlStruct *cvs)
{
    

    
    
    CtrlIn *ivs;
    CtrlOut *ovs;
    CtrlState *svs;
    
    ivs = cvs->inputs;
    ovs = cvs->outputs;
    svs = cvs->states;
	
	map* Map = init_map();
	//int waitTARGET[5] = {TARGET1,TARGET2,TARGET3,TARGET4,GOTOBASE};
	int waitTARGET[6] = {TARGET1,TARGET2,GOTOBASE,TARGET3,TARGET4,GOTOBASE};
	ovs->flag_release = 0;
	
	//--------------------------------------------------
	//               DETECTION OPPONENT
	//--------------------------------------------------
	
	//printf("distance : %f\n",svs->distoPponent);
	distanceOfOponents(cvs);
	//printf("d: %f\n",svs->distoPponent);
	if((svs->distoPponent<0.4) && (svs->Xcalibration && svs->Ycalibration))
	{
		svs->RobotState = AVOIDANCE;	
		
	}	
	//triangulation(cvs);

	
	//--------------------------------------------------
	//               		FSM
	//--------------------------------------------------
	switch(svs->RobotState)
	{
		case START_CALIBRATION:
			if(svs->Xcalibration && svs->Ycalibration)
			{
				svs->RobotState = TARGET1;
				
			}
			else
			{
				calibration(cvs);
			}
			break;
			
		case TARGET1:
			if(svs->onTarget)
			{
				svs->RobotState = WAIT;
				svs->onTarget = false;
				svs->indexPath = 1;
				svs->t0 = ivs->t;
				
			}
			else
			{
				svs->start = 1;
				svs->goal = 12;
				target(cvs,Map,1,12);
			}
			
			svs->previousState = svs->RobotState;
			break;
		
		case TARGET2:
			
			if(svs->onTarget)
			{
				
				svs->RobotState = WAIT;
				svs->onTarget = false;
				svs->indexPath = 1;
				svs->t0 = ivs->t;
			}
			else
			{
				
				svs->start = 12;
				svs->goal = 4;
				target(cvs,Map,12,4);
			}
			svs->previousState = svs->RobotState;
			break;
			
		case TARGET3:
			if(svs->onTarget)
			{
				svs->RobotState = WAIT;
				svs->onTarget = false;
				svs->indexPath = 1;
				svs->t0 = ivs->t;
			}
			else
			{
				svs->start = 1;
				svs->goal = 17;
				target(cvs,Map,1,17);
			}
			svs->previousState = svs->RobotState;
			break;	
		
		case TARGET4:
			if(svs->onTarget)
			{
				svs->RobotState = WAIT;
				svs->onTarget = false;
				svs->indexPath = 1;
				svs->t0 = ivs->t;
			}
			else
			{
				svs->start = 17;
				svs->goal = 18;
				target(cvs,Map,17,18);
			}
			svs->previousState = svs->RobotState;
			break;	
		
		case GOTOBASE:
			if(svs->onTarget)
			{
				svs->RobotState = WAIT;
				ovs->flag_release = 1;
				svs->onTarget = false;
				svs->indexPath = 1;
				svs->t0 = ivs->t;
				svs->cntBase++;
			}
			else
			{
				if(svs->cntBase == 1)
				{
					svs->start = 18;
					svs->goal = 1;
					target(cvs,Map,18,1);
				}
					
				else
				{
					svs->start = 4;
					svs->goal = 1;
					target(cvs,Map,4,1);
				}
			}
			
			svs->previousState = svs->RobotState;
			break;
		
		case WAIT:
			if(svs->waitIsDone)
			{
				svs->indexWait++;
				svs->RobotState = waitTARGET[svs->indexWait];
				svs->waitIsDone = false;
				
			}
			else
			{
				wait(cvs);
			}
			break;
			
		case AVOIDANCE:
			
			
			int result = Astar(svs->start,svs->goal,Map);
			double x = Map->nodes[Map->path[svs->indexPath-1]].x;
			double y = Map->nodes[Map->path[svs->indexPath-1]].y;
			int index = Map->nodes[Map->path[svs->indexPath-1]].index;
			
			navigation(cvs,x,y);
			getOdometry(cvs);
			if(svs->onNode)
				svs->RobotState = WAIT;
			
			
			break;
	}
	
}




/*! \brief last controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */
void controller_finish(CtrlStruct *cvs)
{
	//
}


NAMESPACE_CLOSE();
