#include "RobotAPI.h"
#include "Utils.h"

float sensorFrontPos[3] = {0.06, 0, 0};
float sensorLeftPos[3]  = {0.06, 0, PI/2.0};
float sensorRightPos[3] = {0.06, 0, -PI/2.0};

#define MAP_SIZE 4.0
#define BEL_NXY 81.0
#define BEL_NTHETA ((360.0/5)+1)
#define XY_STEP (MAP_SIZE/BEL_NXY)
#define TH_STEP (2*M_PI/BEL_NTHETA)

#define SEARCH_SQR 1.0
#define MINPROB 10/(BEL_NXY*BEL_NXY*BEL_NTHETA)

//Robot parameters:
#define r 0.02 //0.0325
#define l 0.1 //0.075

#define SIGMA_SONAR 0.05
#define SONAR_ANGLE (7.5*2*PI/360)
#define SONAR_DELTA (SONAR_ANGLE*2.0/10) 

#define Kr 1
#define Kl 1

#define k_rho 0.1
#define k_alpha 2
#define k_beta -0.5

float mapLines[20][4];//at most 20 lines
int nLines=0;

#define NUM_GOALS 8
float goals[NUM_GOALS][3] =
{
	{-1.0, 1.5, 3.14/2},
	{-0.25, 0.55, 0},
	{1, -0.3, -3.14/2},
	{1.0, -1.5, -3.14/2},
	{1.0, 1.5, 3.14/2},
	{0.25, -0.5, -3.14},
	{-1.0, -1.5, -3.14/2},
	{0.35, 0, 3.14/2}
};
int g = 0;
float* goal = goals[g];

void sensorToRobot(float dist, float* sensorPos, float *out)
{
    out[0] = sensorPos[0] + dist*cos(sensorPos[2]);
    out[1] = sensorPos[1] + dist*sin(sensorPos[2]);
}

void robotToWorld(float* vetin, float* robotPos, float *out)
{
    float sintheta = sin(robotPos[2]);
    float costheta = cos(robotPos[2]);
    
    out[0] = robotPos[0] + (vetin[0]*costheta - vetin[1]*sintheta);
    out[1] = robotPos[1] + (vetin[0]*sintheta + vetin[1]*costheta);
    out[2] = robotPos[2] + vetin[2];
}

void worldToMap(float *vetin, float *out)
{
    
    out[0] = round((vetin[0]+MAP_SIZE/2)*(BEL_NXY-1)/MAP_SIZE);
    out[1] = round((vetin[1]+MAP_SIZE/2)*(BEL_NXY-1)/MAP_SIZE);
    out[2] = round((vetin[2]+M_PI)*(BEL_NTHETA-1)/(2*M_PI));
}

void mapToWorld(float *vetin, float *out)
{
    
    out[0] = vetin[0]*MAP_SIZE/(BEL_NXY-1) - MAP_SIZE/2;
    out[1] = vetin[1]*MAP_SIZE/(BEL_NXY-1) - MAP_SIZE/2;
    out[2] = vetin[2]*2*M_PI/(BEL_NTHETA-1) - M_PI;
}

int nearesPointInLine(float *p, float* pA, float* pB, float *npl)
{
  float AP[2] = {p[0] - pA[0], p[1] - pA[1]}; // Storing vector A->P
  float AB[2] = {pB[0] - pA[0], pB[1] - pA[1]}; // Storing vector A->B

  float ab2 = AB[0]*AB[0] + AB[1]*AB[1];  //Basically finding the squared magnitude of AB

  float AP_dot_AB = AP[0]*AB[0] + AP[1]*AB[1]; // The dot product of AP and AB                                     

  float t = AP_dot_AB / ab2; // The normalized "distance" from a to
  
  if (t<0) t = 0;
  if (t>1) t = 1;
  
  npl[0] = pA[0] + AB[0]*t;
  npl[1] = pA[1] + AB[1]*t;
  
//  printf("\rp: [%.2f, %.2f]\n", p[0], p[1]);
//  printf("\rpA: [%.2f, %.2f]\n", pA[0], pA[1]);
//  printf("\rpB: [%.2f, %.2f]\n", pB[0], pB[1]);
//  printf("\rAP: [%.2f, %.2f]\n", AP[0], AP[1]);
//  printf("\rAB: [%.2f, %.2f]\n", AB[0], AB[1]);
//  printf("\rnpl: [%.2f, %.2f]\n", npl[0], npl[1]);
//  printf("\rt: %.2f]\n", t);

  return 1; //point is inside AB
}


//Find nearest point disregarding sonar wave crossing walls
float nearesPointInMap(float *p, float *npl)
{
    float pl[2], d, best_d=99999;
    
    for (int i=0; i<nLines; i++) {
        if (nearesPointInLine(p, &mapLines[i][0], &mapLines[i][2], pl))
        {
            d = SquareDistance(pl[0], pl[1], p[0], p[1]);
            if (d<best_d) {
                best_d = d;
                npl[0] = pl[0];
                npl[1] = pl[1];
            }
//            printf("\r\np: [%.2f, %.2f] ", p[0], p[1]);
//            printf("\rpl: [%.2f, %.2f] ", pl[0], pl[1]);
//            printf("\rd:%.4f bd: %.4f ", d, best_d);
//            printf("\rnpl: [%.2f, %.2f]\n", npl[0], npl[1]);
        }
    }
    
    return best_d;
}

int segmentsIntersect(float *a, float *b, float *c, float *d)
{
    float den = ((d[1]-c[1])*(b[0]-a[0])-(d[0]-c[0])*(b[1]-a[1]));
    float num1 = ((d[0] - c[0])*(a[1]-c[1]) - (d[1]- c[1])*(a[0]-c[0]));
    float num2 = ((b[0]-a[0])*(a[1]-c[1])-(b[1]-a[1])*(a[0]-c[0]));

    if (den == 0 && num1  == 0 && num2 == 0)
        return -1; // The two lines are coincidents
    if (den == 0)
        return -2; // The two lines are parallel

    float u1 = num1/den;
    float u2 = num2/den;

    if (u1 <0 || u1 > 1 || u2 < 0 || u2 > 1)
        
        return -3; // Lines do not collide 
    
    return 1; // Lines DO collide
}

// Find nearest point considering intersections
// since sonar wave can't go through walls
float nearesPointInMapInter(float *robotPos, float *p, float *npl)
{
    float pl[2], d=-1, best_d=99999;
    
    //TODO: check lines de intesect first
    
    for (int i=0; i<nLines; i++) {
        if (nearesPointInLine(p, &mapLines[i][0], &mapLines[i][2], pl))
        {
            d = SquareDistance(pl[0], pl[1], p[0], p[1]);
            if (d<best_d) {
                best_d = d;
                npl[0] = pl[0];
                npl[1] = pl[1];
            }
//            printf("\r\np: [%.2f, %.2f] ", p[0], p[1]);
//            printf("\rpl: [%.2f, %.2f] ", pl[0], pl[1]);
//            printf("\rd:%.4f bd: %.4f ", d, best_d);
//            printf("\rnpl: [%.2f, %.2f]\n", npl[0], npl[1]);
        }
    }
    
    return best_d;
}

void robotPosFromWall(float d, float *npl, float *pRobot, float *newRobot)
{
    float NPLR[2] = {pRobot[0] - npl[0], pRobot[1] - npl[1]};
    
    float norm = sqrt(NPLR[0]*NPLR[0] + NPLR[1]*NPLR[1]);
    newRobot[0] = npl[0] + d*NPLR[0]/norm;
    newRobot[1] = npl[1] + d*NPLR[1]/norm;
    
    //printf("\rNPLR: [%.2f, %.2f]\n", NPLR[0], NPLR[1]);
    //printf("\rnewPos: [%.2f, %.2f]\n", newRobot[0], newRobot[1]);
}

int loadMap(const char *filename)
{
    FILE *fp = fopen(filename, "r");
    if (fp !=NULL) {
        nLines = 0;
        int res;
        while (1) {
            res = fscanf(fp, "%f%f%f%f", &mapLines[nLines][0], &mapLines[nLines][1], &mapLines[nLines][2], &mapLines[nLines][3]);
            if (feof(fp)) break;
            printf("\rline loaded: (%.2f, %.2f) (%.2f, %.2f)\n", mapLines[nLines][0], mapLines[nLines][1], mapLines[nLines][2], mapLines[nLines][3]);
            nLines++;
        }
        fclose(fp);
        return 1;
    } else
        printf("\rCould not open file: %s\n", filename);

    return 0;
}

void robotToSensorPoint(float *pRobot, float *pSensor, float dist, float* point)
{
    float pSR[2];
    sensorToRobot(dist, pSensor, pSR);
    robotToWorld(pSR, pRobot, point);
}

double z(double x) //normal pdf
{
    // constants
    double a1 =  0.254829592;
    double a2 = -0.284496736;
    double a3 =  1.421413741;
    double a4 = -1.453152027;
    double a5 =  1.061405429;
    double p  =  0.3275911;

    // Save the sign of x
    int sign = 1;
    if (x < 0)
        sign = -1;
    x = fabs(x)/sqrt(2.0);

    // A&S formula 7.1.26
    double t = 1.0/(1.0 + p*x);
    double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

    return 0.5*(1.0 + sign*y);
}

float pGaussian(float dist, float sigma, float step)
{
    float x = fabs(dist);
    float halfstep = step/2.0;    
    float zmax = (x+halfstep)/sigma;
    float zmin = (x-halfstep)/sigma;
    return z(zmax) - z(zmin);
}

float pTrvGaussian(cv::Mat &m, cv::Mat &x, cv::Mat E)
{
    /*
     * "Approximates" a trivariate gaussian by multiplying tree univariate gaussians with 
     * sigma equal to the main diagonal. It seems to be fast and precise enought for our purposes.
     * 
    */
    return (pGaussian(x.at<float>(0,0)-m.at<float>(0,0), E.at<float>(0,0), XY_STEP)*
            pGaussian(x.at<float>(0,1)-m.at<float>(0,1), E.at<float>(1,1), XY_STEP)*
            pGaussian(smallestAngleDiff(x.at<float>(0,2),m.at<float>(0,2)), E.at<float>(2,2), TH_STEP));///3;
}

void perceptionUpdate(cv::Mat &bel, float distF, float distL, float distR)
{
    float robotPos[3]; //[x,y,theta]
    int x,y,t;
    float sensorPoint[3], mapPoint[3];
    float p, sum=0;
        
    for(robotPos[0]=-2, x=0; x<BEL_NXY; robotPos[0]+=4.0/(BEL_NXY-1), x++)
    for(robotPos[1]=-2, y=0; y<BEL_NXY; robotPos[1]+=4.0/(BEL_NXY-1), y++)
    for(robotPos[2]=-M_PI, t=0; t<BEL_NTHETA; robotPos[2]+=2*M_PI/(BEL_NTHETA-1), t++)
    {
        
        float b = bel.at<float>(x,y,t);
        
        if (b>0) {
            float stmin, stmax, d;
            float dF=999, dL=999, dR=999;
            float sensorDir[3];
            
//            robotToSensorPoint(robotPos, sensorFrontPos, distF, sensorPoint);
//            dF = nearesPointInMap(sensorPoint, mapPoint);
            
            sensorDir[0] = sensorFrontPos[0];
            sensorDir[1] = sensorFrontPos[1];
            stmin = sensorFrontPos[2]-SONAR_ANGLE;
            stmax = sensorFrontPos[2]+SONAR_ANGLE;
            for (sensorDir[2] = stmin; sensorDir[2]<=stmax; sensorDir[2]+=SONAR_DELTA) {
                robotToSensorPoint(robotPos, sensorDir, distF, sensorPoint);
                d = nearesPointInMap(sensorPoint, mapPoint);
                if (d<dF) dF=d;
            }
            
//            robotToSensorPoint(robotPos, sensorLeftPos, distL, sensorPoint);
//            dL = nearesPointInMap(sensorPoint, mapPoint);
            sensorDir[0] = sensorLeftPos[0];
            sensorDir[1] = sensorLeftPos[1];
            stmin = sensorLeftPos[2]-SONAR_ANGLE;
            stmax = sensorLeftPos[2]+SONAR_ANGLE;
            for (sensorDir[2] = stmin; sensorDir[2]<=stmax; sensorDir[2]+=SONAR_DELTA) {
                robotToSensorPoint(robotPos, sensorDir, distL, sensorPoint);
                d = nearesPointInMap(sensorPoint, mapPoint);
                if (d<dL) dL=d;
            }

//            robotToSensorPoint(robotPos, sensorRightPos, distR, sensorPoint);
//            dR = nearesPointInMap(sensorPoint, mapPoint);
            sensorDir[0] = sensorRightPos[0];
            sensorDir[1] = sensorRightPos[1];
            stmin = sensorRightPos[2]-SONAR_ANGLE;
            stmax = sensorRightPos[2]+SONAR_ANGLE;
            for (sensorDir[2] = stmin; sensorDir[2]<=stmax; sensorDir[2]+=SONAR_DELTA) {
                robotToSensorPoint(robotPos, sensorDir, distR, sensorPoint);
                d = nearesPointInMap(sensorPoint, mapPoint);
                if (d<dR) dR=d;
            }

            p = pGaussian(dF, SIGMA_SONAR, XY_STEP)*pGaussian(dL, SIGMA_SONAR, XY_STEP)*pGaussian(dR, SIGMA_SONAR, XY_STEP)*b;
            bel.at<float>(x,y,t) = p;
            sum+=p;
        }
    }
    
    if (sum>0)
        bel = bel/sum;
//    printf("\rrobot bel(%d,%d,%d): %f\n", xr,yr,tr, bel.at<float>(xr,yr,tr));
//    printf("\rmax p bel(%d,%d,%d): %f\n", xp,yp,tp, bel.at<float>(xp,yp,tp));
}

void fdeltaRL(float theta, float ds, float dtheta, cv::Mat &FDrl)
{
    //ds = fabs(ds);
    float costdt2 = cos(theta+dtheta/2);
    float sintdt2 = sin(theta+dtheta/2);
    float b = 2*l;
    
    FDrl.at<float>(0,0) = costdt2/2 - (ds/(2*b))*sintdt2;//1
    FDrl.at<float>(0,1) = costdt2/2 + (ds/(2*b))*sintdt2;//2
    
    FDrl.at<float>(1,0) = sintdt2/2 + (ds/(2*b))*costdt2;//3
    FDrl.at<float>(1,1) = sintdt2/2 - (ds/(2*b))*costdt2;//4

    FDrl.at<float>(2,0) = 1/b;//5
    FDrl.at<float>(2,1) = -1/b;//6
}

void odomError(cv::Mat &FDrl, cv::Mat &Ed, cv::Mat &Ep)
{
    Ep = FDrl*Ed*FDrl.t();

	//cout << "Ed:\n" << Ed.at<float>(0,0) << " | "  << Ed.at<float>(0,1) << endl;
	//cout << Ed.at<float>(1,0) << " | "  << Ed.at<float>(1,1) << endl;
	    
	//cout << "FDrl:\n" << FDrl.at<float>(0,0) << " | "  << FDrl.at<float>(0,1) << endl;
	//cout << FDrl.at<float>(1,0) << " | "  << FDrl.at<float>(1,1) << endl;
	//cout << FDrl.at<float>(2,0) << " | "  << FDrl.at<float>(2,1) << endl;
	    
	//cout << "Ep:\n" << Ep.at<float>(0,0) << " | "  << Ep.at<float>(0,1) << " | "  << Ep.at<float>(0,2) << endl;
	//cout << Ep.at<float>(1,0) << " | "  << Ep.at<float>(1,1) << " | "  << Ep.at<float>(1,2) << endl;
	//cout << Ep.at<float>(2,0) << " | "  << Ep.at<float>(2,1) << " | "  << Ep.at<float>(2,2) << endl; 
}

int actionUpdate(cv::Mat &bel, float dsl, float dsr)
{
    static float old_dsl = 0;
    static float old_dsr = 0;

    //printf("\r\ndsr: %f, dsl: %f\n", dsr, dsl);
    dsr = old_dsr = (old_dsr+dsr);
    dsl = old_dsl = (old_dsl+dsl);
    float ds = (dsr + dsl)/2;
    float dtheta = (dsr - dsl)/(2*l);
    //printf("\rndsr: %f, ndsl: %f, abs ds:%f<%f, dt: %f<%f\n", dsr, dsl, fabs(ds), XY_STEP, fabs(dtheta), TH_STEP);
    
    if (fabs(ds) < XY_STEP && fabs(dtheta) < TH_STEP) {
        return -1;
    } else {
        old_dsl = 0;
        old_dsr = 0;        
    }
    
    int x1,y1,t1; //x,y,theta indexes on bel map
    int x0,y0,t0; //x,y,theta indexes on tempbel map
    
    cv::Mat X1(1,3, CV_32FC1), //current position [x,y,theta]
            X0(1,3, CV_32FC1), //previous position [x,y,theta]
            M(1,3, CV_32FC1),  //new expected position [x,y,theta]
            Ed(2,2, CV_32FC1),  //covar(dsr, dsl)
            FDrl(3,2, CV_32FC1),//Jacobian
            Ep(3,3, CV_32FC1);  //motion error Sigmap
    
    const int belSizes[3]={BEL_NXY, BEL_NXY, BEL_NTHETA};
    cv::Mat sumbel(3, belSizes, CV_32FC1, 0.0); //somatory belief mat init with zeroes
        
    Ed.at<float>(0,0) = Kr*fabs(dsr);
    Ed.at<float>(1,1) = Kl*fabs(dsl);
    Ed.at<float>(0,1) = 0;
    Ed.at<float>(1,0) = 0;
    
    int cells=0;
    float sum=0;
    float costdt2=0;
    float sintdt2=0;
    
    for(X0.at<float>(0,2)=-M_PI, t0=0; t0<BEL_NTHETA; X0.at<float>(0,2)+=2*M_PI/(BEL_NTHETA-1), t0++) {//for each old theta
        bool newTheta = true;
        
        for(X0.at<float>(0,0)=-2, x0=0; x0<BEL_NXY; X0.at<float>(0,0)+=4.0/(BEL_NXY-1), x0++) { //for each old x
            for(X0.at<float>(0,1)=-2, y0=0; y0<BEL_NXY; X0.at<float>(0,1)+=4.0/(BEL_NXY-1), y0++) {  //for each old y

                float b = bel.at<float>(x0,y0,t0);
                if (b>MINPROB) {
                    
                    if (newTheta) {
                        costdt2 = cos(X0.at<float>(0,2)+dtheta/2);
                        sintdt2 = sin(X0.at<float>(0,2)+dtheta/2);
                        fdeltaRL(X0.at<float>(0,2), ds, dtheta, FDrl);
                        odomError(FDrl, Ed, Ep);
                        
                        M.at<float>(0,2) = to_pi_range(X0.at<float>(0,2)+dtheta);
                        newTheta = false;
                    }
                    
                    M.at<float>(0,1) = X0.at<float>(0,1)+ds*sintdt2;
                    M.at<float>(0,0) = X0.at<float>(0,0)+ds*costdt2;
                    
                    //Update only a rectangle around X0
                    float dsrect = SEARCH_SQR*(fabs(ds)+XY_STEP);
                    float dtrect = SEARCH_SQR*(fabs(dtheta)+TH_STEP);//2*(dtheta + 1 step) since dtheta can be 0
                    float x1min[3], x1max[3];
                    //x1min
                    x1min[0] = M.at<float>(0,0)-dsrect;
                    if (x1min[0]<-2) x1min[0] = -2;                    
                    x1min[1] = M.at<float>(0,1)-dsrect;
                    if (x1min[1]<-2) x1min[1] = -2;
                    x1min[2] = M.at<float>(0,2)-dtrect;
                    if (x1min[2]<-M_PI) x1min[2] = -M_PI;
                    
                    //x1max
                    x1max[0] = M.at<float>(0,0)+dsrect;
                    if (x1max[0]>2) x1max[0] = 2;                    
                    x1max[1] = M.at<float>(0,1)+dsrect;
                    if (x1max[1]>2) x1max[1] = 2;
                    x1max[2] = M.at<float>(0,2)+dtrect;
                    if (x1max[2]>M_PI) x1max[2] = M_PI;
                    
                    //get map coordinates
                    float mapmin[3], mapmax[3];
                    worldToMap(x1min, mapmin);
                    worldToMap(x1max, mapmax);     
                    
                    mapToWorld(mapmin, x1min);
                    mapToWorld(mapmax, x1max);
                    
                    
//                    printf("\rX0 = (%.2f,%.2f,%.2f) M = (%.2f,%.2f,%.2f)\n", X0.at<float>(0,0), X0.at<float>(0,1), X0.at<float>(0,2), M.at<float>(0,0), M.at<float>(0,1), M.at<float>(0,2));
                    
                    for(X1.at<float>(0,2) = x1min[2], t1 =  mapmin[2]; t1<=mapmax[2]; X1.at<float>(0,2)+=2*M_PI/(BEL_NTHETA-1), t1++) {//for each new theta
                        for(X1.at<float>(0,0) = x1min[0], x1 =  mapmin[0]; x1<=mapmax[0]; X1.at<float>(0,0)+=4.0/(BEL_NXY-1), x1++) { //for each new x        
                            for(X1.at<float>(0,1) = x1min[1], y1 =  mapmin[1]; y1<=mapmax[1]; X1.at<float>(0,1)+=4.0/(BEL_NXY-1), y1++) { //for each new y           

                                float px1_u1x0 = pTrvGaussian(M, X1, Ep);
                                sumbel.at<float>(x1,y1,t1) += b*px1_u1x0;
                                sum += b*px1_u1x0;
                                cells++;
//                                if (px1_u1x0>0) {
//                                    printf("\rpx1_u1x0(%.2f,%.2f,%.2f | %.2f,%.2f,%.2f): p:%f  b:%f pb:%f\n", X1.at<float>(0,0), X1.at<float>(0,1), X1.at<float>(0,2), M.at<float>(0,0), M.at<float>(0,1), M.at<float>(0,2), px1_u1x0, b, px1_u1x0*b);
//                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    if (sum>0) {
        bel = sumbel/sum;   
    }
    else { 
        printf("\rI'm lost!!!\n");
        bel = 1.0; //start over with perception data only
//        sum = cv::sum(bel)[0];
//        bel = bel/sum;
        return BEL_NXY*BEL_NXY*BEL_NTHETA;
    }
  
//    double testMaxval;
//    int maxIdx[3];
//    cv::minMaxIdx(sumbel, 0, &testMaxval, 0, maxIdx);
//    printf("\rMax bel: %f (%d,%d,%d) Sum: %f\n", testMaxval, maxIdx[0], maxIdx[1], maxIdx[2], sum); 
//    
    return cells;
}

int readOdometers(int clientID, simxFloat &dwL, simxFloat &dwR)
{
    static bool first = true;
    
    //old joint angle position
    static simxFloat lwprev=0; 
    static simxFloat rwprev=0;
    
    //current joint angle position
    simxFloat lwcur=0;
    simxFloat rwcur=0;
    
    if (first)
    {
        simxInt ret = simxGetJointPosition(clientID, leftMotorHandle, &lwprev, simx_opmode_streaming);
        if(ret > 0)
        {
        	printf("\rCondition A - %d\n", ret);
        	return -1;
     	}
    
        ret = simxGetJointPosition(clientID, rightMotorHandle, &rwprev, simx_opmode_streaming);
        if(ret > 0)
        {
        	printf("\rCondition B - %d\n", ret);
        	return -1;
     	}
        
        dwR = dwL = 0;
        first = false;
    }

    simxInt ret = simxGetJointPosition(clientID, leftMotorHandle, &lwcur, simx_opmode_buffer);
    if(ret > 0)
    {
        printf("\rCondition C - %d\n", ret);
        return -1;
    }
    
    ret = simxGetJointPosition(clientID, rightMotorHandle, &rwcur, simx_opmode_buffer);
    if(ret > 0)
    {
        printf("\rCondition D - %d\n", ret);
        return -1;
    }
    
    dwL = smallestAngleDiff(lwcur, lwprev);
    dwR = smallestAngleDiff(rwcur, rwprev);
    
	//dwR = rwcur - rwprev;
	//dwL = lwcur - lwprev;
        
    if (fabs(dwR)>PI || fabs(dwL)>PI)
    {
        printf("\rwL: %f - (%f) = %f\n", lwcur, lwprev, dwL);
        printf("\rwR: %f - (%f) = %f\n", rwcur, rwprev, dwR);
        lwprev = lwcur;
        rwprev = rwcur;
        return -1;
    }
    
    lwprev = lwcur;
    rwprev = rwcur;
    return 0;
}



void nextGoal()
{
	g = g+1;
    
    if (g >= NUM_GOALS)
        g = 0;
   
    goal = goals[g];
}

void ExecuteMotionControl(float* pos)
{	
	static float K_RHO = 0.1;
	static float K_ALPHA = 2.0;
	static float K_BETA = -0.5;

    float dtheta = smallestAngleDiff(goal[2], pos[2]);
 
    float deltax = goal[0] - pos[0];
    float deltay = goal[1] - pos[1];
    float rho = sqrt(deltax*deltax + deltay*deltay);
 
    float atg = to_pi_range(atan2(deltay, deltax));
    float alpha = to_pi_range(smallestAngleDiff(atg, pos[2]));
    float beta = to_pi_range(goal[2] - pos[2] - alpha);
 
    float v = K_RHO * rho;
    
    if(v < 0.025) 
    	v = 0.025;

    float w = (K_ALPHA * alpha + K_BETA * beta);
 
    float wR = v + l*w; //2*v + l*w;
    float wL = v - l*w; //wR - 2*l*w;
 
    float phiL = wL / r;
    float phiR = wR / r;

    if(rho < 0.05)
    {
		phiR = 2 * dtheta;
		phiL = -2 * dtheta;
		
		if(fabs(dtheta) >= 0.5)
		{
			phiR = 2 * dtheta;
			phiL = -2 * dtheta;
		}
		
		else
		{
			phiL = 0.0;
			phiR = 0.0;
			nextGoal();
		}
    }

    APISetRobotSpeed(phiL, phiR);
}

int main(int argc, char* argv[])
{
	//Se conseguiu conectar com a API
    if (APIInitConection())
    {
        printf("\rConexão efetuada.\n");

        if (!loadMap("ini/envmapBig.ini"))
        {
            printf("\rNão foi possível carregar o mapa.\n");
            return -1;
        }
       
       	//Começa a simulação
        if(APIStartSimulation())
        {
        	printf("\rSimulação iniciada.\n");

	        APIReadSonarLeft();
	        APIReadSonarFront();
			APIReadSonarRight();

	        const int belSizes[3] = {BEL_NXY, BEL_NXY, BEL_NTHETA};
	        cv::Mat bel(3, belSizes, CV_32FC1);

	       /*
	        * Initialize bel with 100% certainty at real robot position so
	        * that the tracking starts correctly.
	        */ 

			//Get real robot position from v-rep API:
	        float pos[3] = {0,0,0};
	        APIGetTrueRobotPosition(pos);      
	        float firstMapPos[3];
	        worldToMap(pos, firstMapPos);
	        bel = 0.0;
	        bel.at<float>(firstMapPos[0],firstMapPos[1], firstMapPos[2]) = 1;      
	        int cells = 1;
        	
        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning())
		    {    
		    	// Perception update
	            float distF = -1, distR = -1, distL = -1;
	            distL = APIReadSonarLeft();
	            distF = APIReadSonarFront();
	            distR = APIReadSonarRight();

	            if (cells > 0 && distF >= 0 && distL >= 0 && distR >= 0)
	            {
	                //Do only if action step was successfull and sonar data is good
	                //printf("\rstart perception update...");fflush(stdout);
	                APIStopRobot();
                    perceptionUpdate(bel, distF, distL, distR);
	                printf("\rperception update done\n");
	            }

	            //Action update
	            //Read current wheels angle variation:
	            float dwL, dwR; //rad
	            if (readOdometers(clientID, dwL, dwR) == 0)
	            {
	                //printf("\rstart action update width: %.4f %.4f...", dwL*r, dwR*r);fflush(stdout);
	                cells = actionUpdate(bel, dwL*r, dwR*r);
	                if (cells > 0)
	                    printf("\rAction Update %d cells\n", cells);
	            } else
	                printf("\rError reading odometers\n");

	            //Set robot target speed if you wish:
				ExecuteMotionControl(pos);

		        //Espera um tempo antes da próxima atualização
		        APIWait();
		    }
		    //---------------------------------------------------------
		   
		   	printf("\rFim da simulação.\n\r");
		   
		    //Para o robô e desconecta
		    APIFinishSimulation();
        } 
        
        else
        	printf("\rNão foi possível iniciar a simulação.\n");
    } 
    
    else
        printf("\rNão foi possível conectar.\n");
   
    return 0;
}