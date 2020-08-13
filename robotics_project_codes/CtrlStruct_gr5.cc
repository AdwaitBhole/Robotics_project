#include "CtrlStruct_gr5.h"
#include "namespace_ctrl.h"
#include "ctrl_main_gr5.h"
#include "math.h"

NAMESPACE_INIT(ctrlGr5);

/*! \brief initialize the controller structure
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] outputs outputs of the controller
 * \return controller main structure
 */


CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs, CtrlOut *py_outputs)
{
	CtrlStruct *cvs;

	cvs = (CtrlStruct*) malloc(sizeof(CtrlStruct));
    cvs->states  = (CtrlState*) malloc(sizeof(CtrlState));
	cvs->inputs  = inputs;
	cvs->outputs = outputs;
    cvs->py_outputs = py_outputs;
    


	return cvs;
}

// Our external functions.

// LIMITER
double limiter(double value, double min, double max)
{
    
    if(value > max)
    {
        return max;
    }
    else if (value < min)
    {
        return min;
    }
    else
    {
        return value;
    }
}

void controller_iterate(CtrlStruct *cvs, double omegaRref, double omegaLref)
{
    CtrlIn *ivs;
    CtrlOut *ovs;
    CtrlState *svs;
    
    
    double commandR, commandL;
    double commandRbackEMF;
    double commandLbackEMF;
    
    ivs = cvs-> inputs;
    ovs = cvs-> outputs;
    svs = cvs-> states;
    
    
    fprintf(svs-> pFileSpeed, "%f\n", ivs-> r_wheel_speed);
    fprintf(svs-> pFileTime, "%f\n", ivs->t);
    
    
    svs-> omegaRref      = omegaRref;
    svs-> omegaLref      = omegaLref;
    svs-> omegaRrefMotor = svs-> rho * omegaRref;
    svs-> omegaLrefMotor = svs-> rho * omegaLref;
    
    if(ivs->t>-13)
	{  
        svs-> errorR    = svs-> omegaRrefMotor - svs-> rho * ivs-> r_wheel_speed;
        svs-> errorL    = svs-> omegaLrefMotor - svs-> rho * ivs-> l_wheel_speed;
        svs-> dt        = ivs-> t - svs-> timeOld;
        svs-> intErrorR = svs-> intErrorR + ((svs-> errorR + svs-> errorRold)*(svs-> dt/2.0));
        svs-> intErrorL = svs-> intErrorL + ((svs-> errorL + svs-> errorLold)*(svs-> dt/2.0));
        
        commandR             = KP*svs-> errorR + KI*svs-> intErrorR;
        commandL             = KP*svs-> errorL + KI*svs-> intErrorL;
        commandRbackEMF      = commandR  + KPHI * svs-> rho * ivs-> r_wheel_speed;
        commandLbackEMF      = commandL  + KPHI * svs-> rho * ivs-> l_wheel_speed;
        ovs-> wheel_commands[R_ID] = limiter(commandRbackEMF, -100, 100);
        ovs-> wheel_commands[L_ID] = limiter(commandLbackEMF, -100, 100);
       
        svs-> errorRold = svs-> errorR;
        svs-> errorLold = svs-> errorL;  
    }
    svs-> timeOld   = ivs-> t;
}

void calibration(CtrlStruct *cvs)
{
    
    CtrlIn *ivs;
    CtrlOut *ovs;
    CtrlState *svs;
	
	ivs = cvs-> inputs;
    ovs = cvs-> outputs;
    svs = cvs-> states;
	
	bool   switchesPressed  = ((ivs->u_switch[R_ID] && ivs->u_switch[L_ID]));
    bool   switchesRpressed = ivs->u_switch[R_ID];
    bool   switchesLpressed = ivs->u_switch[L_ID];
    double omegaRrefCalibration = svs->omegaRrefCalibration;
    double omegaLrefCalibration = svs->omegaLrefCalibration;
    double xAxis, yAxis;
    
    if((!svs->Xcalibration || !svs->Ycalibration))
    {
        
        
        if(!svs->Ycalibration)
        {
            controller_iterate(cvs, omegaRrefCalibration, omegaLrefCalibration);
            getOdometry(cvs);
            if(switchesRpressed && !switchesLpressed)
            { 
                controller_iterate(cvs, 0, omegaLrefCalibration); 
            }
            if(switchesLpressed && !switchesRpressed)
            {
                controller_iterate(cvs, omegaRrefCalibration, 0);
            }
            if(switchesPressed)
            {
                controller_iterate(cvs, 0, 0);
                svs->Ycalibration  = true;
				printf("id : %i \n",ivs->robot_id );
				//if(ivs->robot_id == BLUE || ivs->robot_id == RED)
				//{
					svs->pos_y_odo     = -1.5 + 0.06;
					svs->pos_theta_odo = M_PI/2;
				//}
                //else if(ivs->robot_id == YELLOW || ivs->robot_id == WHITE)
				//{
				//	svs->pos_y_odo     = 1.5 - 0.06;
				//	svs->pos_theta_odo = -M_PI/2;
				//}
                getOdometry(cvs);
                printf("Ycalibration is done\n");
            }
        }
        if(!svs->Xcalibration && svs->Ycalibration)
        {
            controller_iterate(cvs, 4, 4);
            getOdometry(cvs);
            if(svs->pos_y_odo>-0.75)
            {
				controller_iterate(cvs, -3, 3);
                getOdometry(cvs);
                if(svs->pos_theta_odo <0.1 && svs->pos_theta_odo>-0.1)
                {
                    controller_iterate(cvs, -8, -8);
                    getOdometry(cvs);
                    if(switchesPressed)
                    {
                        controller_iterate(cvs, 0, 0);
                        svs->Xcalibration = true;
                        svs->pos_x_odo        = -1.0 + 0.06;
                        svs->pos_theta_odo    = 0.0;
                        getOdometry(cvs);
                        printf("Xcalibration is done\n");	
                    }
                    
                }
                
            }
            
        }
        
        
    }
    
}


void triangulation(CtrlStruct *cvs)
{
	CtrlIn *ivs;
    CtrlOut *ovs;
    CtrlState *svs;
	
	ivs = cvs-> inputs;
    ovs = cvs-> outputs;
    svs = cvs-> states;
	
	fprintf(svs-> pFileXtri, "%f\n", svs->pos_x_tri);
	fprintf(svs-> pFileYtri, "%f\n", svs->pos_y_tri);
	fprintf(svs-> pFileThetaTri, "%f\n", svs->pos_theta_tri);
	
	double  rising_fixed[3];
	double  falling_fixed[3];
	int     index;
	double xBeacon[3] = {-1.062,  1.062, 0.0};
	double yBeacon[3] = {-1.562, -1.562, 1.562};
	double alpha_beacon_estimation[3];
	int i,j;
	
	if((ivs->nb_rising_fixed ==3) & (ivs->nb_falling_fixed ==3 ))
	{
		index = ivs->rising_index_fixed;
		if(ivs->rising_index_fixed > ivs->falling_index_fixed) index--;
		
	     // Extract angles of rising and falling edges in tabulars of length 3
		if(ivs->rising_index_fixed == 0)
		{
			rising_fixed[0]  = ivs->last_rising_fixed[NB_STORE_EDGE-2];
			rising_fixed[1]  = ivs->last_rising_fixed[NB_STORE_EDGE-1];
			rising_fixed[2]  = ivs->last_rising_fixed[0];
			falling_fixed[0] = ivs->last_falling_fixed[NB_STORE_EDGE-2];
			falling_fixed[1] = ivs->last_falling_fixed[NB_STORE_EDGE-1];
			falling_fixed[2] = ivs->last_falling_fixed[0];
			
		}
		else if(ivs->rising_index_fixed == 1)
		{
			rising_fixed[0]  = ivs->last_rising_fixed[NB_STORE_EDGE-1];
			rising_fixed[1]  = ivs->last_rising_fixed[0];
			rising_fixed[2]  = ivs->last_rising_fixed[1];
			falling_fixed[0] = ivs->last_falling_fixed[NB_STORE_EDGE-1];
			falling_fixed[1] = ivs->last_falling_fixed[0];
			falling_fixed[2] = ivs->last_falling_fixed[1];
		
		}
		else
		{
			rising_fixed[0]  = ivs->last_rising_fixed[index-2];
			rising_fixed[1]  = ivs->last_rising_fixed[index-1];
			rising_fixed[2]  = ivs->last_rising_fixed[index];
			falling_fixed[0] = ivs->last_falling_fixed[index-2];
			falling_fixed[1] = ivs->last_falling_fixed[index-1];
			falling_fixed[2] = ivs->last_falling_fixed[index];
			
		}
		
		for(i=0;i<3;i++)
		{
			if(rising_fixed[i] == falling_fixed[i])
			{
				svs-> alpha_beacon[i] = 0.0;
				printf("Fuck\n");
				break;
			}
			else if(rising_fixed[i] > falling_fixed[i])
			{
				falling_fixed[i] += 2*M_PI;
				svs-> alpha_beacon[i] = (rising_fixed[i] + falling_fixed[i])/2.0;
				printf("Fuck\n");
			}	
			else
			{
				svs-> alpha_beacon[i] = (rising_fixed[i] + falling_fixed[i])/2.0;
			}
		}
		
	}
		// Estimate angles of each beacon
	for(i=0;i<3;i++)
	{
		double epsilon = 5.0*M_PI/180.0;
		alpha_beacon_estimation[i] = atan2((yBeacon[i]-svs->pos_y_odo),(xBeacon[i]-svs->pos_x_odo))-svs->pos_theta_odo;
		// Discontinuity traitement
		bool QUADRANT_23  = alpha_beacon_estimation[i]*svs->alpha_beacon[i]<0;
		bool ESTI_3_MES_2 = (alpha_beacon_estimation[i]<-M_PI+epsilon) && (svs->alpha_beacon[i]>M_PI-epsilon);
		bool ESTI_2_MES_3 = (svs->alpha_beacon[i]<-M_PI+epsilon) && (alpha_beacon_estimation[i]>M_PI-epsilon);
		if(QUADRANT_23 && (ESTI_3_MES_2 || ESTI_2_MES_3))
		{
			svs->countDiscontinuity++;
			if(alpha_beacon_estimation[i] < 0)
			{
				alpha_beacon_estimation[i] += 2*M_PI;
			}
			else
			{
				alpha_beacon_estimation[i] -= 2*M_PI;
			}
		}
	}
	
	// Compare angles 
			
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			svs->err[i][j] = fabs(svs->alpha_beacon[i]-alpha_beacon_estimation[j]);
		}
	}
	mappingAnglesBeacons(cvs);
	triangulationCompute(cvs);
	/*printf("============================================================================\n");
	printf("Xodo = %f -- Yodo = %f -- THETAodo = %f\n", svs->pos_x_odo, svs->pos_y_odo, svs->pos_theta_odo*180.0/M_PI);
	printf("Xtri = %f -- Ytri = %f -- THETAtri = %f\n", svs->pos_x_tri, svs->pos_y_tri, svs->pos_theta_tri*180.0/M_PI);
	printf("============================================================================\n");*/
}

void triangulationCompute(CtrlStruct *cvs)
{
	CtrlIn *ivs;
    CtrlOut *ovs;
    CtrlState *svs;
	
	ivs = cvs-> inputs;
    ovs = cvs-> outputs;
    svs = cvs-> states;
	
    double xBeacon[3] = {-1.062,  1.062, 0.0};
	double yBeacon[3] = {-1.562, -1.562, 1.562};
	
	double x1P, y1P, x3P, y3P, T12, T23, T31, x12P, y12P, x23P, y23P, x31P, y31P, k31P, D;
	
	x1P = xBeacon[svs->map[0]]-xBeacon[svs->map[1]];  
	y1P = yBeacon[svs->map[0]]-yBeacon[svs->map[1]];  
	x3P = xBeacon[svs->map[2]]-xBeacon[svs->map[1]];  
	y3P = yBeacon[svs->map[2]]-yBeacon[svs->map[1]];
	
	T12  = cot(svs->alpha_beacon[1]-svs->alpha_beacon[0]);
	T23  = cot(svs->alpha_beacon[2]-svs->alpha_beacon[1]);
	T31  = (1-T12*T23)/(T12+T23);
	
	x12P = x1P + T12*y1P;
	y12P = y1P - T12*x1P;
	x23P = x3P - T23*y3P;
	y23P = y3P + T23*x3P;
	x31P = (x3P + x1P) + T31*(y3P - y1P);
	y31P = (y3P + y1P) - T31*(x3P - x1P);
	
	k31P = x1P*x3P + y1P*y3P + T31*(x1P*y3P - x3P*y1P);
	
	D    = (x12P - x23P)*(y23P - y31P) - (y12P - y23P)*(x23P - x31P);
	
	svs->pos_x_tri     = xBeacon[svs->map[1]] + ((k31P*(y12P - y23P))/(D)) - 0.083;
	svs->pos_y_tri     = yBeacon[svs->map[1]] + ((k31P*(x23P - x12P))/(D));
	svs->pos_theta_tri = wrapAngle(atan2((yBeacon[svs->map[1]] - svs->pos_y_tri),(xBeacon[svs->map[1]] - svs->pos_x_tri)) - svs->alpha_beacon[1]);
}

double wrapAngle(double angle)
{
	return (angle - 2*M_PI*floor((angle + M_PI) / (2*M_PI)));
}

void mappingAnglesBeacons(CtrlStruct *cvs)
{
	CtrlIn *ivs;
    CtrlOut *ovs;
    CtrlState *svs;
	
	ivs = cvs-> inputs;
    ovs = cvs-> outputs;
    svs = cvs-> states;
	
	int i,j;
	double min;
	int mapp=0;
	for(i=0;i<3;i++)
	{
		min = svs->err[i][0];
		for(j=0;j<3;j++)
		{
			if(svs->err[i][j] > 6.2 && svs->err[i][j] < 6.4)
			{
				svs->map[i]= j;
				break;
			}
			else
			{
				if(svs->err[i][j] <= min)
				{
					svs->map[i] = j;
					min         = svs->err[i][j];
				}
			}
		}
		
	}
	
}

void getOdometry(CtrlStruct *cvs)
{
    CtrlIn *ivs;
    CtrlOut *ovs;
    CtrlState *svs;
	
	ivs = cvs-> inputs;
    ovs = cvs-> outputs;
    svs = cvs-> states;
	
	
	
		fprintf(svs-> pFileXodo, "%f\n", svs->pos_x_odo);
		fprintf(svs-> pFileYodo, "%f\n", svs->pos_y_odo);
		fprintf(svs-> pFileThetaOdo, "%f\n", svs->pos_theta_odo);
	
	
	double v_r, v_l, delta_Sl, delta_Sr, delta_S, b, dt, theta, delta_Theta, thetaNotWraped;
    
    dt          = svs->dt;
    b           = svs->b;
    //printf("X = %f -- Y = %f -- theta = %f\n", cvs->states->pos_x_odo, cvs->states->pos_y_odo, cvs->states->pos_theta_odo);
    
    
   //------------------------------------Kalman---------------------------------------//

    /// implimentation of the kalman measurment in the odometer
    v_r = ivs->r_wheel_speed * cvs->states->Rwheel; // speed w r
    v_l = ivs->l_wheel_speed * cvs->states->Rwheel; // speed w l
    delta_Sr = v_r * dt;//
    delta_Sl = v_l * dt;//
    delta_S  = ((delta_Sr + delta_Sl)/2.0); //mean
    delta_Theta            = ((delta_Sr - delta_Sl)/b); // diff
    theta                  = svs->pos_theta_odo;
		//printf("v_r:%f, V_l:%f , delta_Sr:%f , delta_Sl:%f , delta_S: %f , delta_Theta: %f , theta:%d \n", v_r, v_l, delta_Sr,delta_Sl,delta_S, delta_Theta, theta);

    svs->pos_x_odo       += delta_S * cos(theta+(delta_Theta/2.0));
    svs->pos_y_odo       += delta_S * sin(theta+(delta_Theta/2.0));
    svs->pos_theta_odo   += delta_Theta;
	//svs->pos_theta_odo    = wrapAngle(svs->pos_theta_odoNW);
	//printf("pos_x_odo: %f , pos_y_odo: %f , pos_theta_odo: %f\n", svs->pos_x_odo, svs->pos_y_odo, svs->pos_theta_odo );

    svs->Odo_kal[0][0]=svs->pos_x_odo;
    svs->Odo_kal[1][0]=svs->pos_y_odo;
    svs->Odo_kal[2][0]=svs->pos_theta_odo;
    //this is the X(k+1/k)



    double co = cos(theta +0.5*delta_Theta); // Simplification of the notation cos()
    double si = sin(theta +0.5*delta_Theta); // Simplification of the notation sin()
		//printf("cos: %f, sin: %f \n", co, si);


    if(svs->pos_theta_odo > M_PI)
    {
        svs->pos_theta_odo -= 2*M_PI;
    }
    else if(svs->pos_theta_odo < -M_PI)
    {
        svs->pos_theta_odo += 2*M_PI;
    }
    svs->timeOld = cvs->inputs->t;


// Jacobian matrixes:
    //jacobian of F_odo x y theta
    double Fp[3][3] = {{ 1.0 , 0.0, -(delta_S)*si},
                       {0.0 , 1.0, (delta_S)*co},
                       {0.0 , 0.0 , 1.0}}; //ok
		//printf("data: ..... %f\n", -(delta_S)*si);
/*
		printf("Fp1.1 : %f  Fp1.2 : %f  Fp1.3 : %f\n", Fp[0][0],Fp[0][1],Fp[0][2]);
		printf("Fp2.1 : %f  Fp2.2 : %f  Fppt2.3 : %f\n", Fp[1][0],Fp[1][1],Fp[1][2]);
		printf("Fp3.1 : %f  Fp3.2 : %f  Fp3.3 : %f\n", Fp[2][0],Fp[2][1],Fp[2][2]);
		printf("\n");
	*/

    //traspose of fp
    double Fp_t[3][3] = {{Fp[0][0], Fp[1][0], Fp[2][0]},
                         {Fp[0][1], Fp[1][1], Fp[2][1]},
												 {Fp[0][2], Fp[1][2], Fp[2][2]}};//ok
/*
		printf("Fp_t1.1 : %f  Fp_t1.2 : %f  Fp_t1.3 : %f\n", Fp_t[0][0],Fp_t[0][1],Fp_t[0][2]);
		printf("Fp_t2.1 : %f  Fp_t2.2 : %f  Fp_t2.3 : %f\n", Fp_t[1][0],Fp_t[1][1],Fp_t[1][2]);
		printf("Fp_t3.1 : %f  Fp_t3.2 : %f  Fp_t3.3 : %f\n", Fp_t[2][0],Fp_t[2][1],Fp_t[2][2]);
		printf("\n");
		*/

    //jacobian of F_odo by x y theta
    double Fu[3][2] = {{0.5*co - (0.5*(delta_S)*si)/(BETWEEN_WHEELS) , 0.5*co + (0.5*(delta_S)*si)/(BETWEEN_WHEELS)},
                       {(0.5*si + 0.5*(delta_S)*co)/(BETWEEN_WHEELS) , (0.5*si - 0.5*(delta_S)*co)/(BETWEEN_WHEELS)},
                       {1.0/(BETWEEN_WHEELS), -1.0/(BETWEEN_WHEELS)}};
    //transpose of Fu
    double Fu_T[2][3] = {{Fu[0][0] , Fu[1][0], Fu[2][0]},
                        {Fu[0][1],Fu[1][1],Fu[2][1]}};
    //Q
    double Q[2][2] = {{KOR *fabs(delta_Sr), 0.0},
                      {0.0, KOL*fabs(delta_Sl)}};


    // mathematical operation of P(k+1/k)
		//P =Fp_PFpt + Fu_QFut



    //mult(3,3, 3,3, svs->P, Fp_t, PFpt); //function 2 internet

		mult_mat_33_33(svs->PFpt,svs->P,Fp_t);
		/*
		printf("PFpt1.1 : %f  PFpt1.2 : %f  PFpt1.3 : %f\n", PFpt[0][0],PFpt[0][1],PFpt[0][2]);
		printf("PFpt2.1 : %f  PFpt2.2 : %f  PFpt2.3 : %f\n", PFpt[1][0],PFpt[1][1],PFpt[1][2]);
		printf("PFpt3.1 : %f  PFpt3.2 : %f  PFpt3.3 : %f\n", PFpt[2][0],PFpt[2][1],PFpt[2][2]);
		printf("\n" );
		*/
    mult_mat_33_33(svs->Fp_PFpt,Fp,svs->PFpt);
		/*
		printf("%f\n",((0.2)*(0.2)+(-(delta_S)*si)*(-(delta_S)*si)*(10)*(10)) );
		printf("Fp_PFpt1.1 : %f  Fp_PFpt1.2 : %f  Fp_PFpt1.3 : %f\n", Fp_PFpt[0][0],Fp_PFpt[0][1],Fp_PFpt[0][2]);
		printf("Fp_PFpt2.1 : %f  Fp_PFpt2.2 : %f  Fp_PFpt2.3 : %f\n", Fp_PFpt[1][0],Fp_PFpt[1][1],Fp_PFpt[1][2]);
		printf("Fp_PFpt3.1 : %f  Fp_PFpt3.2 : %f  Fp_PFpt3.3 : %f\n", Fp_PFpt[2][0],Fp_PFpt[2][1],Fp_PFpt[2][2]);
		printf("\n" );
		*/
    mult_mat_22_23(svs->QFut,Q,Fu_T);
    mult_mat_32_23(svs->Fu_QFut,Fu,svs->QFut);
		/*
		printf("p1.1 : %f  p1.2 : %f  p1.3 : %f\n", svs->QFut[0][0],svs->QFut[0][1],svs->QFut[0][2]);
		printf("p2.1 : %f  p2.2 : %f  p2.3 : %f\n", svs->QFut[1][0],svs->QFut[1][1],svs->QFut[1][2]);
		printf("p3.1 : %f  p3.2 : %f  p3.3 : %f\n", svs->QFut[2][0],svs->QFut[2][1],svs->QFut[2][2]);
		printf("\n");
		*/
    add_mat_33_33(svs->P,svs->Fp_PFpt,svs->Fu_QFut);
		/*
		printf("p1.1 : %f  p1.2 : %f  p1.3 : %f\n", svs->P[0][0],svs->P[0][1],svs->P[0][2]);
		printf("p2.1 : %f  p2.2 : %f  p2.3 : %f\n", svs->P[1][0],svs->P[1][1],svs->P[1][2]);
		printf("p3.1 : %f  p3.2 : %f  p3.3 : %f\n", svs->P[2][0],svs->P[2][1],svs->P[2][2]);
		printf("\n");
		*/

    /*
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            printf("value of the matrix is : %f \n",svs->P[i][j] );
        }
        printf("\n");
    }*/

    KalmanFilter(cvs);

}

void KalmanFilter(CtrlStruct *cvs)
{

    CtrlIn *ivs;
    CtrlOut *ovs;
    CtrlState *svs;

    ivs = cvs-> inputs;
    ovs = cvs-> outputs;
    svs = cvs-> states;

    int i;
    double xBeacon[3] = {-1.062,  1.062, 0.0};
    double yBeacon[3] = {-1.562, -1.562, 1.562};
    double alpha_estimated[3];
    for(i=0;i<3;i++)
    {
        alpha_estimated[0] = atan2((yBeacon[0]-svs->pos_y_odo),(xBeacon[0]-svs->pos_x_odo))-(svs->pos_theta_odo);
        alpha_estimated[1] = atan2((yBeacon[1]-svs->pos_y_odo),(xBeacon[1]-svs->pos_x_odo))-(svs->pos_theta_odo);
        alpha_estimated[2] = atan2((yBeacon[2]-svs->pos_y_odo),(xBeacon[2]-svs->pos_x_odo))-(svs->pos_theta_odo);
    }
		//h is the measurment function
		double h[3][1] = {{alpha_estimated[0]},{alpha_estimated[1]},{alpha_estimated[2]}};
		//printf("h1 : %f  h2 : %f  h3 : %f\n", h[0][0],h[0][1],h[0][2]);

    double x = svs->pos_x_odo;
    double y = svs->pos_y_odo;
		//printf("x_odo: %f,  y_odo: %f \n", x, y);
		//H is the jacobian of h
    //double H[3][3]= //ignore this one!!
		//{{((yBeacon[0]-y)/(x*x -2*x*xBeacon[0]+xBeacon[0]*xBeacon[0]+(yBeacon[0]-y)*(yBeacon[0]-y))),((xBeacon[0]-x)/(x*x -2*x*xBeacon[0]+xBeacon[0]*xBeacon[0]+(yBeacon[0]-y)*(yBeacon[0]-y))),(-1)},
		//{((yBeacon[1]-y)/(x*x -2*x*xBeacon[1]+xBeacon[1]*xBeacon[1]+(yBeacon[1]-y)*(yBeacon[1]-y))),(((xBeacon[1]-x)/(x*x -2*x*xBeacon[1]+xBeacon[1]*xBeacon[1]+(yBeacon[1]-y)*(yBeacon[1]-y)))),(-1)},
		//{((yBeacon[2]-y)/(x*x -2*x*xBeacon[1]+xBeacon[1]*xBeacon[1]+(yBeacon[1]-y)*(yBeacon[1]-y))),((xBeacon[2]-x)/(x*x -2*x*xBeacon[1]+xBeacon[1]*xBeacon[1]+(yBeacon[1]-y)*(yBeacon[1]-y))),(-1)}};

		double H[3][3]=
         {{((yBeacon[0]-y)/(((xBeacon[0]-x)*(xBeacon[0]-x))-(((yBeacon[0]-y)*(yBeacon[0]-y)/((xBeacon[0]-x)*(xBeacon[0]-x)))+1))),
            (-1)/(((xBeacon[0]-x)*(xBeacon[0]-x))-(((yBeacon[0]-y)*(yBeacon[0]-y)/((xBeacon[0]-x)*(xBeacon[0]-x)))+1)),
            (-1)},
		 {(yBeacon[1]-y)/(((xBeacon[1]-x)*(xBeacon[1]-x))-(((yBeacon[1]-y)*(yBeacon[1]-y)/((xBeacon[1]-x)*(xBeacon[1]-x)))+1)),
			 (-1)/(((xBeacon[1]-x)*(xBeacon[1]-x))-(((yBeacon[1]-y)*(yBeacon[1]-y)/((xBeacon[1]-x)*(xBeacon[1]-x)))+1)),(-1)},
            {(yBeacon[2]-y)/(((xBeacon[2]-x)*(xBeacon[2]-x))-(((yBeacon[2]-y)*(yBeacon[2]-y)/((xBeacon[2]-x)*(xBeacon[2]-x)))+1)),
							(-1)/(((xBeacon[2]-x)*(xBeacon[2]-x))-(((yBeacon[2]-y)*(yBeacon[2]-y)/((xBeacon[2]-x)*(xBeacon[2]-x)))+1)),(-1)}};


		//verified on wolframealfa
		/*
		printf("H1.1 : %f  H1.2 : %f  H1.3 : %f\n", H[0][0],H[0][1],H[0][2]);
		printf("H2.1 : %f  H2.2 : %f  H2.3 : %f\n", H[1][0],H[1][1],H[1][2]);
		printf("H3.1 : %f  H3.2 : %f  H3.3 : %f\n", H[2][0],H[2][1],H[2][2]);
		printf("\n");
		*/


		double Ht[3][3]= {{H[0][0], H[1][0], H[2][0]},
                     {H[0][1], H[1][1], H[2][1]},
                     {H[0][2], H[1][2], H[2][2]}};
	 /*
	 printf("Ht1.1 : %f  Ht1.2 : %f  Ht1.3 : %f\n", Ht[0][0],Ht[0][1],Ht[0][2]);
	 printf("Ht2.1 : %f  Ht2.2 : %f  Ht2.3 : %f\n", Ht[1][0],Ht[1][1],Ht[1][2]);
	 printf("Ht3.1 : %f  Ht3.2 : %f  Ht3.3 : %f\n", Ht[2][0],Ht[2][1],Ht[2][2]);
	 printf("\n");
	 */


		// sigma_m2 -> laser noise variance
    double R[3][3]={{0.015*100, 0, 0},{0, 0.015*100, 0},{0, 0, 0.015*100}}; // need to verify if it is fine


    mult_mat_33_33(svs->PHt, svs->P, Ht);
/*
		printf("PHt1.1 : %f  PHt1.2 : %f  PHt1.3 : %f\n", svs->PHt[0][0],svs->PHt[0][1],svs->PHt[0][2]);
		printf("PHt2.1 : %f  PHt2.2 : %f  PHt2.3 : %f\n", svs->PHt[1][0],svs->PHt[1][1],svs->PHt[1][2]);
		printf("PHt3.1 : %f  PHt3.2 : %f  PHt3.3 : %f\n", svs->PHt[2][0],svs->PHt[2][1],svs->PHt[2][2]);
		printf("\n");
	*/


    mult_mat_33_33(svs->HPHt,H,svs->PHt);
/*
		printf("HPHt1.1 : %f  HPHt1.2 : %f  HPHt1.3 : %f\n", svs->HPHt[0][0],svs->HPHt[0][1],svs->HPHt[0][2]);
		printf("HPHt2.1 : %f  HPHt2.2 : %f  HPHt2.3 : %f\n", svs->HPHt[1][0],svs->HPHt[1][1],svs->HPHt[1][2]);
		printf("HPHt3.1 : %f  HPHt3.2 : %f  HPHt3.3 : %f\n", svs->HPHt[2][0],svs->HPHt[2][1],svs->HPHt[2][2]);
		printf("\n");
	*/


		//s = HPHt + R
    add_mat_33_33(svs->S, svs->HPHt , R);
/*
		printf("S1.1 : %f  S1.2 : %f  S1.3 : %f\n", svs->S[0][0],svs->S[0][1],svs->S[0][2]);
		printf("S2.1 : %f  S2.2 : %f  S2.3 : %f\n", svs->S[1][0],svs->S[1][1],svs->S[1][2]);
		printf("S3.1 : %f  S3.2 : %f  S3.3 : %f\n", svs->S[2][0],svs->S[2][1],svs->S[2][2]);
		printf("\n");
	*/

    inv_mat_33(svs->invS, svs->S);


    mult_mat_33_33(svs->HtinvS,Ht,svs->invS);
    mult_mat_33_33(svs->K, svs->P, svs->HtinvS);
/*
		printf("K1.1 : %f  K1.2 : %f  K1.3 : %f\n", svs->K[0][0],svs->K[0][1], svs->K[0][2]);
		printf("K2.1 : %f  K2.2 : %f  K2.3 : %f\n", svs->K[1][0],svs->K[1][1],svs->K[1][2]);
		printf("K3.1 : %f  K3.2 : %f  K3.3 : %f\n", svs->K[2][0],svs->K[2][1],svs->K[2][2]);
		printf("\n");
*/


svs->triang_kal[0][0] = svs->pos_x_tri;
svs->triang_kal[1][0] = svs->pos_y_tri;
svs->triang_kal[2][0] = svs->pos_theta_tri;


    sous_mat_31_31(svs->error, svs->triang_kal, h);
/*
		printf("error1.1 : %f  \n", svs->error[0][0]);
		printf("error2.1 : %f  \n", svs->error[1][0]);
		printf("error3.1 : %f  \n", svs->error[2][0]);
		printf("\n");
*/

    mult_mat_33_31(svs->Kerror, svs->K, svs->error);
/*
		printf("error1.1 : %f  \n", svs->Kerror[0][0]);
		printf("error2.1 : %f  \n", svs->Kerror[1][0]);
		printf("error3.1 : %f  \n", svs->Kerror[2][0]);
		printf("\n");
*/
    add_mat_31_31(svs->newX, svs->triang_kal, svs->Kerror);
	/*
    printf("kalmanX: %f  kalmanY: %f  kalmanT: %f\n", svs->newX[0][0],svs->newX[1][0],svs->newX[2][0]);
		printf("odo_x: %f , odo_y: %f , odo_theta: %f \n ", svs->pos_x_odo, svs->pos_y_odo, svs->pos_theta_odo);
		*/


   	//P= (I -KH) P
   mult_mat_33_33(svs->KH, svs->K, H);
/*
		printf("H1.1 : %f  H1.2 : %f  H1.3 : %f\n", H[0][0],H[0][1],H[0][2]);
		printf("H2.1 : %f  H2.2 : %f  H2.3 : %f\n", H[1][0],H[1][1],H[1][2]);
		printf("H3.1 : %f  H3.2 : %f  H3.3 : %f\n", H[2][0],H[2][1],H[2][2]);
		printf("\n");

		printf("KH1.1 : %f  KH1.2 : %f  KH1.3 : %f\n", svs->KH[0][0],svs->KH[0][1],svs->KH[0][2]);
		printf("KH2.1 : %f  KH2.2 : %f  KH2.3 : %f\n", svs->KH[1][0],svs->KH[1][1],svs->KH[1][2]);
		printf("KH3.1 : %f  KH3.2 : %f  KH3.3 : %f\n", svs->KH[2][0],svs->KH[2][1],svs->KH[2][2]);
		printf("\n");
*/
		double I[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    sous_mat_33_33(svs->I_KH, I,svs->KH);
/*
		printf("I_KH1.1 : %f  I_KH1.2 : %f  I_KH1.3 : %f\n", svs->I_KH[0][0],svs->I_KH[0][1],svs->I_KH[0][2]);
		printf("I_KH2.1 : %f  I_KH2.2 : %f  I_KH2.3 : %f\n", svs->I_KH[1][0],svs->I_KH[1][1],svs->I_KH[1][2]);
		printf("I_KH3.1 : %f  I_KH3.2 : %f  I_KH3.3 : %f\n", svs->I_KH[2][0],svs->I_KH[2][1],svs->I_KH[2][2]);
		printf("\n");
	*/

    mult_mat_33_33(svs->newP,svs->I_KH, svs->P );
		/*
		printf("p1.1 : %f  p1.2 : %f  p1.3 : %f\n", svs->P[0][0],svs->P[0][1],svs->P[0][2]);
		printf("p2.1 : %f  p2.2 : %f  p2.3 : %f\n", svs->P[1][0],svs->P[1][1],svs->P[1][2]);
		printf("p3.1 : %f  p3.2 : %f  p3.3 : %f\n", svs->P[2][0],svs->P[2][1],svs->P[2][2]);
		printf("\n");
		printf("pnew1.1 : %f  pnew1.2 : %f  pnew1.3 : %f\n", svs->newP[0][0],svs->newP[0][1],svs->newP[0][2]);
		printf("pnew2.1 : %f  pnew2.2 : %f  pnew2.3 : %f\n", svs->newP[1][0],svs->newP[1][1],svs->newP[1][2]);
		printf("pnew3.1 : %f  pnew3.2 : %f  pnew3.3 : %f\n", svs->newP[2][0],svs->newP[2][1],svs->newP[2][2]);
		printf("\n");
		*/

		svs->P[0][0]= svs->newP[0][0];svs->P[0][1]= svs->newP[0][1];svs->P[0][2]= svs->newP[0][2];
		svs->P[1][0]= svs->newP[1][0];svs->P[1][1]= svs->newP[1][1];svs->P[1][2]= svs->newP[1][2];
		svs->P[2][0]= svs->newP[2][0];svs->P[2][1]= svs->newP[2][1];svs->P[2][2]= svs->newP[2][2];

//------------------------------------------------------------------------------------------------
//don't forget to add the matrix manipilation !!

}

void distanceOfOponents(CtrlStruct *cvs)
{
	
	CtrlIn *ivs;
	CtrlOut *ovs;
    CtrlState *svs;
	
	ivs = cvs-> inputs;
    ovs = cvs-> outputs;
    svs = cvs-> states;
	
	ovs->tower_command = 15.0;
	double alpha, d;
	
	alpha = fabs(ivs->last_rising[ivs->rising_index] - ivs->last_falling[ivs->falling_index])/2.0;
	if(alpha > M_PI)
		alpha -= 2*M_PI;
	else if(alpha < -M_PI)
		alpha += 2*M_PI;
	d    = fabs(RADIUS_BEACON/tan(alpha));
	fprintf(svs->pFileDis,"%f\n",d);
	svs->cntOpp++;
	svs->sumOpp += d;
	if(svs->cntOpp >= 500)
	{
		svs->distoPponent = svs->sumOpp / svs->cntOpp;
		svs->cntOpp = 0;
		svs->sumOpp = 0.0;
	}
	fprintf(svs->pFileDisF,"%f\n",svs->distoPponent);
   	
}

double angleOfOponents(CtrlStruct *cvs)
{ 
	CtrlIn *ivs;
	
	ivs = cvs-> inputs;
	
	double alpha;
	
	if(ivs->last_rising[ivs->rising_index] == ivs->last_falling[ivs->falling_index])
		alpha = 0.0;
	else if(ivs->last_rising[ivs->rising_index] > ivs->last_falling[ivs->falling_index])
	{
		ivs->last_falling[ivs->falling_index] += 2*M_PI;
		alpha = (ivs->last_rising[ivs->rising_index] + ivs->last_falling[ivs->falling_index])/2.0;
	}
	else
		alpha = (ivs->last_rising[ivs->rising_index] + ivs->last_falling[ivs->falling_index])/2.0;
	/*
	if(alpha > M_PI)
		alpha -= 2*M_PI;
	else if(alpha < -M_PI)
		alpha += 2*M_PI;*/
	return alpha;
}

void navigation(CtrlStruct *cvs, double X_target, double Y_target)
{
  
	double Krho   = 0.25;
	double Kalpha = 1;
	double Lrobot = 0.225;
	double R      = 0.03;
    
	double delta_x, delta_y,beta,rho,alpha,vref,omega,vLeft,vRight;
   
    delta_x = X_target - cvs->states->pos_x_odo; //pos_x_odo;// target exprimer en coordonner robot
    delta_y = Y_target - cvs->states->pos_y_odo; //pos_y_odo; 
	

	rho   = sqrt(delta_x*delta_x + delta_y*delta_y);
	alpha = -(cvs->states->pos_theta_odo) + atan2(delta_y,delta_x);//cvs->states->pos_theta_odo
    beta  = -(cvs->states->pos_theta_odo) - alpha; //cvs->states->pos_theta_odo
	/*
	
	delta_x = X_target - cvs->states->newX[0][0]; //pos_x_odo;// target exprimer en coordonner robot
    delta_y = Y_target - cvs->states->newX[1][0]; //pos_y_odo; 
	

	rho   = sqrt(delta_x*delta_x + delta_y*delta_y);
	alpha = -(cvs->states->newX[2][0]) + atan2(delta_y,delta_x);//cvs->states->pos_theta_odo
    beta  = -(cvs->states->newX[2][0]) - alpha; //cvs->states->pos_theta_odo
*/
	//printf("angle_avant=%f\n",alpha*180/M_PI);
	if(alpha>M_PI)
	{
		alpha = alpha - 2*M_PI;
	}
	if(alpha<-M_PI)
	{
		alpha = alpha + 2*M_PI;
	}
	//double angle = alpha*180.0 / M_PI;
	//printf("angle_apres=%f\n",alpha*180/M_PI);
	if(fabs(alpha*180.0 / M_PI)<15.0)
	{   
		vref = 0.3;
	//	printf("vref 02\n");
	}
	else 
	{
		vref = 0;
	//	printf("vref 00\n");
	}
	omega = Kalpha*alpha;//vitesse angulaire du robot
	//printf("Omega=%f\n",omega);
	if(rho>=0.06)//on va s arreter a une certaine distance 
	{ 
		vLeft  = vref - (Lrobot)*omega;
		vRight = vref + (Lrobot)*omega;
	}
    else
	{
            vLeft  = 0;
            vRight = 0;
			cvs->states->onNode = true;
	}
	//printf("rho=%f\n", rho);
	//printf("vRight=%f\n",vRight);
	controller_iterate(cvs, vRight / R, vLeft / R);
	
}

//-----------------------------------------------------------------------------------
//                                    Astar 
//-----------------------------------------------------------------------------------

int Astar(int start, int goal, map* map)
{
    int result,current,j,mapAdjacent;

    map->nodes[start].open = 1;
    while(!openIsEmpty(map))
    {
        current = minIndexFopen(map);
        if(current == goal) break;
        map->nodes[current].open = 0;

        setParent(map,current);

        for(j=0;j<8;j++)
        {
            mapAdjacent = map->nodes[current].adjacent[j];
			if(!(map->nodes[mapAdjacent].walkable))
				mapAdjacent = -1;
            if(mapAdjacent != -1)
            {
                map->nodes[mapAdjacent].g = map->nodes[current].g ;//+ g(map->nodes[current],map->nodes[mapAdjacent]);
                map->nodes[mapAdjacent].h = heuristique(map->nodes[mapAdjacent], map->nodes[goal]);
                map->nodes[mapAdjacent].f = map->nodes[mapAdjacent].g + map->nodes[mapAdjacent].h;

                if(map->nodes[mapAdjacent].closed) continue;
                if(map->nodes[mapAdjacent].open) continue;
                map->nodes[mapAdjacent].open = 1; // Add node to the openlist
            }
        }

        map->nodes[current].closed = 1;
        map->path[map->lengthPath] = map->nodes[current].index;
        map->lengthPath++;
    }
    map->path[map->lengthPath] = map->nodes[goal].index;
    map->lengthPath++;

    result = 1;

    return result;
}



double g(node a, node b)
{
    double cost;
    cost = sqrt((b.x-a.x)*(b.x-a.x)+(b.y-a.y)*(b.y-a.y));
    //cost = (b.x-a.x)*(b.x-a.x)+(b.y-a.y)*(b.y-a.y);
    return cost;
}


void printClosed(map* map)
{
    int i;
    for(i=0;i<map->number;i++)
    {
        printf("%i\n", map->nodes[i].closed);
    }
}

void printPath(map* Map)
{
    int i;
    printf("===============================================================\n");
    printf("Path found :\t");
    for(i=0;i<Map->lengthPath;i++)
    {
        printf("%i | ", Map->path[i]);
    }
    printf("\n");
    printf("===============================================================\n");
}

void setParent(map* map, int n)
{
    int i,mapAdjacent;

    for(i=0;i<8;i++)
    {
        mapAdjacent = map->nodes[n].adjacent[i];
        if(mapAdjacent != -1)
        {
            map->nodes[mapAdjacent].parent = &(map->nodes[n]);
        }
    }
}

int minIndexFopen(map* map)
{
    int i,index;
    double min = 10000.0;
    index = -1;
    for(i=0;i<map->number;i++)
    {
        if(map->nodes[i].open)
        {
            if(map->nodes[i].f <= min)
            {
                min = map->nodes[i].f;
                index = i;
            }
        }
    }
    return index;
}

int openIsEmpty(map* map)
{
    int i;
    for(i=0;i<map->number;i++)
    {
        if((map->nodes[i].open) == 1)
            return 0;
    }
    return 1;
}

double heuristique(node n, node goal)
{
   return sqrt((goal.y - n.y)*(goal.y - n.y) + (goal.x - n.x)*(goal.x - n.x));
   //return fabs(goal.y - n.y) + fabs(goal.x - n.x);
}

map* init_map()
{
	FILE *pFile = NULL;
	pFile = fopen("nodes2.txt", "r+");
	map *Map = (map*)malloc(sizeof(map));
	Map->path = (int*)malloc(sizeof(int)*MAX_LIST_ELEMENTS);
	Map->lengthPath = 0;

	if (pFile != NULL)
	{
		fscanf(pFile, "Number of nodes : %d\n", &Map->number);

		Map->nodes = (node*)malloc(sizeof(node)*Map->number);

		int i;
		for (i = 0; i<Map->number; i++)
		{
			fscanf(pFile, "%d: %lf %lf %d %d %d %d %d %d %d %d %d\n", &(Map->nodes[i].index), &(Map->nodes[i].x), &(Map->nodes[i].y), &(Map->nodes[i].walkable),
				&(Map->nodes[i].adjacent[0]), &(Map->nodes[i].adjacent[1]), &(Map->nodes[i].adjacent[2]), &(Map->nodes[i].adjacent[3]),
				&(Map->nodes[i].adjacent[4]), &(Map->nodes[i].adjacent[5]), &(Map->nodes[i].adjacent[6]), &(Map->nodes[i].adjacent[7]));

			Map->nodes[i].open = 0;
			Map->nodes[i].closed = 0;
			Map->nodes[i].g = 0.0;
			Map->nodes[i].h = 0.0;
			Map->nodes[i].f = 0.0;
			Map->nodes[i].parent = NULL;
		}
	}
	else
	{
		exit(0);
	}
	fclose(pFile);

	return Map;
}

void printMap()
{
    printf("==========================\n");
    map* map;
    map = init_map();
    int i,j;
    for(i=0;i<map->number;i++)
    {
        printf("Node[%d] : \n",i);
        for(j=0;j<8;j++)
        {
            printf("\tadjacent[%d] = %d\n",j,map->nodes[i].adjacent[j]);
        }
        printf("\n");
    }
    printf("==========================\n");
    freeMap(map);
}

void freeMap(map* map)
{
    free(map->nodes);
    free(map->path);
    free(map);
}


//-----------------------------------------------------------------------------------

void target(CtrlStruct *cvs, map* Map, int start, int goal)
{
			
	
	CtrlIn *ivs;
	CtrlOut *ovs;
	CtrlState *svs;

	ivs = cvs->inputs;
	ovs = cvs->outputs;
	svs = cvs->states;
	
	
	int result;
	
	result = Astar(start,goal,Map);
	
	if(svs->indexPath == Map->lengthPath)
	{
		svs->onTarget = true;
		printf("target true\n");
	}
		
	else if(svs->onNode)
	{
		printf("Node %i\n",Map->nodes[Map->path[svs->indexPath]].index);
		svs->indexPath++;
		
		navigation(cvs,Map->nodes[Map->path[svs->indexPath]].x, Map->nodes[Map->path[svs->indexPath]].y);
		getOdometry(cvs);
		
		svs->onNode = false;
	}
	else
	{
		navigation(cvs,Map->nodes[Map->path[svs->indexPath]].x, Map->nodes[Map->path[svs->indexPath]].y);
		getOdometry(cvs);
	}
			
			
			
	freeMap(Map);
}

void wait(CtrlStruct* cvs)
{
	CtrlIn *ivs;
	CtrlOut *ovs;
	CtrlState *svs;

	ivs = cvs->inputs;
	ovs = cvs->outputs;
	svs = cvs->states;
	
	controller_iterate(cvs,0.0,0.0);
	
	getOdometry(cvs);
	if((ivs->t - svs->t0) > 2.0)
	{
		svs->waitIsDone = true;
		
	}
	
}

void XYoponents(CtrlStruct* cvs)
{
	CtrlIn *ivs;
	CtrlOut *ovs;
	CtrlState *svs;

	ivs = cvs->inputs;
	ovs = cvs->outputs;
	svs = cvs->states;
	
	ovs->tower_command = 15;
	double alphaOponents,Xop,Yop;
	
	distanceOfOponents(cvs);
	alphaOponents    = angleOfOponents(cvs);
	
	if(alphaOponents > M_PI)
		alphaOponents -= 2*M_PI;
	else if(alphaOponents < -M_PI)
		alphaOponents += 2*M_PI;
	
	svs->filterCntOp++; 
	svs->Xoponents = svs->pos_x_odo + svs->distoPponent*cos(svs->pos_theta_odo + alphaOponents);
	svs->Yoponents = svs->pos_y_odo + svs->distoPponent*sin(svs->pos_theta_odo + alphaOponents);
	svs->XoponentsSum += svs->Xoponents;
	svs->YoponentsSum += svs->Yoponents;
	
	if(svs->filterCntOp >= 500)
	{
		svs->XoponentsFiltered = svs->XoponentsSum / svs->filterCntOp;
		svs->YoponentsFiltered = svs->YoponentsSum / svs->filterCntOp;
		svs->filterCntOp = 0;
		svs->XoponentsSum = 0.0;
		svs->YoponentsSum = 0.0;
	}
	
	
	
	
	fprintf(svs-> pFileXoponent, "%f\n", svs->Xoponents);
	fprintf(svs-> pFileYoponent, "%f\n", svs->Yoponents);
	fprintf(svs->pFileXopFiltered, "%f\n",svs->XoponentsFiltered);
	fprintf(svs->pFileYopFiltered, "%f\n",svs->YoponentsFiltered);
	
	//printf("==============================================================\n");
	//printf("X of oponent : %f -- Y of oponent : %f -- Angle : %f\n",svs->Xoponents,svs->Yoponents,alphaOponents);
	//printf("distance : %f\n",distanceOponents);
	//printf("==============================================================\n");
}


////--------------------------------KALMAN-------------------------------------------
//mathematical manipulation of matrix

void inv_mat_33(double result[3][3], double A[3][3]){

    double det = A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1]) - A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0]) + A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);
    double invDet = 1.0/det;
    double A_t[3][3];
    int i,j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            A_t[i][j] = A[j][i];
        }
    }
    double M_00 = A_t[1][1]*A_t[2][2] - A_t[2][1]*A_t[1][2];
    double M_01 = A_t[1][0]*A_t[2][2] - A_t[2][0]*A_t[1][2];
    double M_02 = A_t[1][0]*A_t[2][1] - A_t[1][1]*A_t[2][0];
    double M_10 = A_t[0][1]*A_t[2][2] - A_t[2][1]*A_t[0][2];
    double M_11 = A_t[0][0]*A_t[2][2] - A_t[2][0]*A_t[0][2];
    double M_12 = A_t[0][0]*A_t[2][1] - A_t[2][0]*A_t[0][1];
    double M_20 = A_t[0][1]*A_t[1][2] - A_t[1][1]*A_t[0][2];
    double M_21 = A_t[0][0]*A_t[1][2] - A_t[1][0]*A_t[0][2];
    double M_22 = A_t[0][0]*A_t[1][1] - A_t[1][0]*A_t[0][1];

    result[0][0] =   M_00 * invDet;   result[0][1] = - M_01 * invDet;   result[0][2] =   M_02 * invDet;
    result[1][0] = - M_10 * invDet;   result[1][1] =   M_11 * invDet;   result[1][2] = - M_12 * invDet;
    result[2][0] =   M_20 * invDet;   result[2][1] = - M_21 * invDet;   result[2][2] =   M_22 * invDet;
}

void sous_mat_33_33(double result[3][3], double A[3][3], double B[3][3]){
    int i,j;
    for(i = 0; i < 3; i++){
        for( j = 0; j < 3; j++){
            result[i][j] = A[i][j] - B[i][j];
        }
    }
}
void sous_mat_31_31(double result[3][1], double A[3][1], double B[3][1]){
    int i,j;
    for(i = 0; i < 3; i++){
        for( j = 0; j < 1; j++){
            result[i][j] = A[i][j] - B[i][j];
        }
    }
}

void add_mat_33_33(double result[3][3], double A[3][3], double B[3][3]){
    int i,j;
    for(i = 0; i < 3; i++){
        for( j = 0; j < 3; j++){
            result[i][j] = A[i][j] + B[i][j];
        }
    }
}
void add_mat_31_31(double result[3][1], double A[3][1], double B[3][1]){
    int i,j;
    for(i = 0; i < 3; i++){
        for( j = 0; j < 1; j++){
            result[i][j] = A[i][j] + B[i][j];
        }
    }
}

void mult_mat_22_23(double result[2][3], double A[2][2], double B[2][3]){
    int i,j;
    for(i = 0; i < 2; i++){
        for( j = 0; j < 3; j++){
            result[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j];
        }
    }
}

void mult_mat_32_23(double result[3][3], double A[3][2], double B[2][3]){
    int i,j;
    for(i = 0; i < 3; i++){
        for( j = 0; j < 3; j++){
            result[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j];
        }
    }
}

void mult_mat_33_33(double result[3][3], double A[3][3], double B[3][3]){
    int i,j;
    for(i = 0; i < 3; i++){
        for( j = 0; j < 3; j++){
            result[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
        }
    }
}
void mult_mat_33_31(double result[3][1], double A[3][3], double B[3][1]){
    int i,j,k;
    for (i = 0; i <= 2; i++) {
        for (j = 0; j <= 1; j++) {
            for (k = 0; k <= 2; k++) {
                result[i][j] = A[i][k] * B[k][j];
            }
        }
    }
}

//-----------------------------------------------------------------------------------

/*! \brief release controller main structure memory
 * 
 * \param[in] cvs controller main structure
 */
void free_CtrlStruct(CtrlStruct *cvs)
{
	
	free(cvs);
}

NAMESPACE_CLOSE();
