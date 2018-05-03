

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../include/contrller.h"
#include "motionInterfaceBuffer.h"
#include "motionParameterType.h"
#include "motionParameter.h"
#include "motionIner.h"
#include "motionConfig.h"
#include "motionCommandType.h"
#include "Common.h"
#include "posemath.h"
#include "kinematics.h"
#include "motionFeedback.h"
#include "rtos.h"
//#include "MotionApi.h"
#include "tp.h"
#include "motionErrorType.h"
#define AROUND_ZERO 0.00001

MotionParameter  parameterBuf;
MotionCommandBuffer commandBuf;
MotionStatus motionStatusBuf;
MotionConfig motConfigBuf;
MotionDebug motionDebugBuf;
MotionJointParameter jointBuf[MAX_JOINTS];
MotionJointParameter *joints = jointBuf;
MotionStatus *emcmotStatus = &motionStatusBuf;
MotionConfig *emcmotConfig = &motConfigBuf;
MotionDebug *emcmotDebug = &motionDebugBuf;

MotionCommandBuffer *commandShmem = &commandBuf;
VRobotMotionCommand  *motCmd = &(commandBuf.motionCmd);
MotionCommandQueue *interpList = &(commandBuf.motionCmdQ);
VRobotMotionCommand *emcmotCommand;
PmRotationMatrix Rtmp;
PmRotationVector Vector;
MotionFeedback motionFb;
t_lock Mutex;


//t_servo* g_servo[6]={NULL,NULL,NULL,NULL,NULL,NULL};
double PUMA_A1 = 100.0;
double PUMA_A2 = 300.0;
double PUMA_A3 = 20.0;
double PUMA_D3 = 10.0;
double PUMA_D4 = 300.0;
double PUMA_D6 = 100.0;
double SCARA_L1=325;
double SCARA_L2=275;
 int num_joints = 6;

 int errorCode = 0;


// 这是导出变量的一个示例
double robotJ0;
double robotJ1;
double robotJ2;
double robotJ3;
double robotJ4;
double robotJ5;
double x;
double y;
double z;
extern void reportErrorType(int type);
// 这是导出函数的一个示例。
 int fncontrller(void)
{
    return 42;
}

// 这是已导出类的构造函数。
// 有关类定义的信息，请参阅 contrller.h
/*Ccontrller::Ccontrller()
NN
    return;
}
*/
/*
This fuction shoud calculate the usr coordiante information based on the position that is indicated by 
the pointer jp
the pointer jp shoud address at leat 3 JointPoint variables,otherwise the outcome should be an error.
the pointer out pointed to a memory that stores the outcome value


CONTRLLER_API int CTRL_UserCalibration(JointPoint *jp, JointPoint *out)
{
	int i = 0;
	for (i = 0; i < 3; i++)
	{
		out[i].j1 = jp[i].j1;
		out[i].j2 = jp[i].j2;
		out[i].j3 = jp[i].j3;
		out[i].j4 = jp[i].j4;
		out[i].j5 = jp[i].j5;
		out[i].j6 = jp[i].j6;
	}


	return 0;
}

*/

void matrixToRpy(PmRotationMatrix rot, PmRpy *rpy)
{
	double  T[3][3];
	double Pitch;
	double Roll;
	double Yaw;
	T[0][0] = rot.x.x;
	T[0][1] = rot.x.y;
	T[0][2] = rot.x.z;

	T[1][0] = rot.y.x;
	T[1][1] = rot.y.y;
	T[1][2] = rot.y.z;

	T[2][0] = rot.z.x;
	T[2][1] = rot.z.y;
	T[2][2] = rot.z.z;

	Pitch = asin(-T[2][0]);
	Roll = atan(T[2][1] / T[2][2]);
	Yaw = atan(T[1][0] / T[0][0]);

	if (T[2][2]<0)
	{
		if (Roll < 0)
		{
			Roll = Roll + 3.1416;
		}
		else
		{
			Roll = Roll - 3.1416;
		}
	}

	if (T[0][0]<0)
	{
		if (T[1][0]>0)
		{
			Yaw = Yaw + 3.1416;
		}
		else
		{
			Yaw = Yaw - 3.1416;
		}
	}

	rpy->p = Pitch;
	rpy->r = Roll;
	rpy->y = Yaw;


	return;
}

void rpyToMatrix(PmRpy rpy,PmRotationMatrix *rot)
{
	double theta_z, theta_y, theta_x;

	double cos_z_2;
	double cos_y_2;
	double cos_x_2;

	double sin_z_2;
	double sin_y_2;
	double sin_x_2;

	double Q[4];
	double T[3][3];

	theta_z = rpy.r;
	theta_y = rpy.y;
	theta_x = rpy.p;


	theta_z = theta_z*PM_PI / 180;
	theta_y = theta_y*PM_PI / 180;
	theta_x = theta_x*PM_PI / 180;

	cos_z_2 = cos(0.5*theta_z);
	cos_y_2 = cos(0.5*theta_y);
	cos_x_2 = cos(0.5*theta_x);

	sin_z_2 = sin(0.5*theta_z);
	sin_y_2 = sin(0.5*theta_y);
	sin_x_2 = sin(0.5*theta_x);

	Q[0] = cos_z_2*cos_y_2*cos_x_2 + sin_z_2*sin_y_2*sin_x_2;
	Q[1] = cos_z_2*cos_y_2*sin_x_2 - sin_z_2*sin_y_2*cos_x_2;
	Q[2] = cos_z_2*sin_y_2*cos_x_2 + sin_z_2*cos_y_2*sin_x_2;
	Q[3] = sin_z_2*cos_y_2*cos_x_2 - cos_z_2*sin_y_2*sin_x_2;

	T[0][0] = Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2] - Q[3] * Q[3];
	T[0][1] = 2 * (Q[1] * Q[2] - Q[0] * Q[3]);
	T[0][2] = 2 * (Q[1] * Q[3] + Q[0] * Q[2]);

	T[1][0] = 2 * (Q[1] * Q[2] + Q[0] * Q[3]);
	T[1][1] = Q[0] * Q[0] - Q[1] * Q[1] + Q[2] * Q[2] - Q[3] * Q[3];
	T[1][2] = 2 * (Q[2] * Q[3] - Q[0] * Q[1]);

	T[2][0] = 2 * (Q[1] * Q[3] - Q[0] * Q[2]);
	T[2][1] = 2 * (Q[2] * Q[3] + Q[0] * Q[1]);
	T[2][2] = Q[0] * Q[0] - Q[1] * Q[1] - Q[2] * Q[2] + Q[3] * Q[3];

	rot->x.x = T[0][0];
	rot->x.y = T[0][1];
	rot->x.z = T[0][2];

	rot->y.x = T[1][0];
	rot->y.y = T[1][1];
	rot->y.z = T[1][2];

	rot->z.x = T[2][0];
	rot->z.y = T[2][1];
	rot->z.z = T[2][2];

	return;
}

int CTRL_SetOriginOffset(s64 offset,u32 axis)
{
	
	if(1==emcmotStatus->enble)
	{
		printf("can not change the origin offset while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_ORGOFFSET_WHILE_ENABLED_TYPE;
	}
	else
	{
		if(axis>6||axis<1)
		{
			printf("input axis number error axis=%d \n",axis);		
			return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
		}
		else
		{
			emcmotConfig->encodeOriginOffset[axis-1]=offset;
			printf("emcmotConfig->encodeOriginOffset[%d]=%lld \r\n",axis-1,emcmotConfig->encodeOriginOffset[axis-1]);
		}
	}
	return 0;
}

int CTRL_GetOriginOffset(s64 * offset,u32 axis)
{
	
	if(axis>6||axis<1)
	{
		printf("input axis number error axis=%d \n",axis);		
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	else
	{
		*offset=emcmotConfig->encodeOriginOffset[axis-1];
	}
	return 0;
}

int CTRL_SetPUMA(f64 length,u32 num)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the puma while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_PUMA_WHILE_ENABLED_TYPE;
	}
	else
	{
		switch(num)
		{
			case 1:
	 			PUMA_A1 =length ;
				break;
			case 2:
				PUMA_A2 =length ;
				break;
			case 3:
				 PUMA_A3 = length;
				break;
			case 4:
				 PUMA_D4 = length;
				break;
			case 5:
 				PUMA_D6 = length;	
				break;
			default:
				printf("can not edit puma that is not exist \r\n");
				return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
				break;
		}
	}
	return 0;
}

int CTRL_GetPUMA(f64 *length,u32 num)
{
	switch(num)
	{
		case 1:
	 		*length=PUMA_A1 ;
			break;
		case 2:
			*length=PUMA_A2  ;
			break;
		case 3:
			*length=PUMA_A3 ;
			break;
		case 4:
			*length=PUMA_D4 ;
			break;
		case 5:
 			*length=PUMA_D6 ;	
			break;
		default:
			printf("can not get puma that is not exist \r\n");
			return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
			break;
	}	
	return 0;
}

int CTRL_SetTeachLinearVelocity(f64 velocity)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the Teach Linear Velocity while servo is enabled \n");
		return MOTION_ERROR_CANNOT_MODIFY_TEACHV_WHILE_ENABLED_TYPE;
	}
	emcmotConfig->teachLinearV=velocity;
	return 0;
}

int CTRL_GetTeachLinearVelocity(f64 *velocity)
{
	*velocity=emcmotConfig->teachLinearV;
	return 0;
}

int CTRL_SetLinearMaxVelocity(f64 velocity)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the  Maximum Linear Velocity while servo is enabled \n");
		return MOTION_ERROR_CANNOT_MODIFY_MAX_LINEARV_WHILE_ENABLED_TYPE;
	}
	emcmotConfig->linearMaxV=velocity;
	return 0;

}
 int CTRL_GetLinearMaxVelocity(f64 *velocity)
{
	*velocity=emcmotConfig->linearMaxV;
	return 0;
}

int CTRL_SetLinearMoveAcc(f64 acc)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the  Maximum Linear acceleration while servo is enabled \n");
		return MOTION_ERROR_CANNOT_MODIFY_MAX_LINEARACC_WHILE_ENABLED_TYPE;
	}
	emcmotConfig->linearMoveAcc=acc;
	return 0;
}

 int CTRL_GetLinearMoveAcc(f64 *acc)
{
	*acc=emcmotConfig->linearMoveAcc;
	return 0;
}

int CTRL_SetRotateMoveVelocity(f64 velocity)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the  Maximum rotation velocity while servo is enabled \n");
		return MOTION_ERROR_CANNOT_MODIFY_MAX_ROTATEV_WHILE_ENABLED_TYPE;
	}
	emcmotConfig->rotateMoveV=velocity;
	return 0;
}

 int CTRL_GetRotateMoveVelocity(f64 *velocity)
{
	*velocity=emcmotConfig->rotateMoveV;
	return 0;
}
int CTRL_SetRotateMoveAcc(f64 acc)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the  Maximum rotation move acceleration while servo is enabled \n");
		return -1;
	}
	emcmotConfig->rotateMoveAcc=acc;
	return 0;
}

int CTRL_GetRotateMoveAcc(f64 *acc)
{
	*acc=emcmotConfig->rotateMoveAcc;
	return 0;
}
int CTRL_SetLinearPositiveLimit(f64 limit,u32 axis)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the soft limit while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_SOFT_LIMIT_WHILE_ENABLED_TYPE;
	}
	if(axis>=1&&axis<=6)
	{
		emcmotConfig->linearPosLimit[axis-1]=limit;
		joints[axis-1].max_pos_limit=limit;
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}
int CTRL_GetLinearPositiveLimit(f64 *limit,u32 axis)
{
	if(axis>=1&&axis<=6)
	{
		*limit=emcmotConfig->linearPosLimit[axis-1];
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}

int CTRL_SetLinearNegativeLimit(f64 limit,u32 axis)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the soft limit while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_SOFT_LIMIT_WHILE_ENABLED_TYPE;
	}
	if(axis>=1&&axis<=6)
	{
		emcmotConfig->linearNegaLimit[axis-1]=limit;
		joints[axis-1].min_pos_limit=limit;
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}
int CTRL_GetLinearNegativeLimit(f64 *limit,u32 axis)
{
	if(axis>=1&&axis<=6)
	{
		*limit=emcmotConfig->linearNegaLimit[axis-1];
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}



int CTRL_SetJointPositiveLimit(f64 limit,u32 axis)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the soft limit while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_SOFT_LIMIT_WHILE_ENABLED_TYPE;
	}
	if(axis>=1&&axis<=6)
	{
		emcmotConfig->jointPosLimit[axis-1]=limit;
		joints[axis-1].max_jog_limit=limit;

	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}
int CTRL_GetJointPositiveLimit(f64 *limit,u32 axis)
{

	if(axis>=1&&axis<=6)
	{
		*limit=emcmotConfig->jointPosLimit[axis-1];
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}

int CTRL_SetJointNegativeLimit(f64 limit,u32 axis)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the soft limit while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_SOFT_LIMIT_WHILE_ENABLED_TYPE;
	}
	if(axis>=1&&axis<=6)
	{
		emcmotConfig->jointNegaLimit[axis-1]=limit;
		joints[axis-1].min_jog_limit=limit;
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}
int CTRL_GetJointNegativeLimit(f64 *limit,u32 axis)
{
	if(axis>=1&&axis<=6)
	{
		*limit=emcmotConfig->jointNegaLimit[axis-1];
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}


int CTRL_SetJointMaxVelocity(f64 velocity,u32 axis)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change set joint velocity while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_JOINT_VELOCITY_WHILE_ENABLED_TYPE;
	}

	if(axis>=1&&axis<=6)
	{
		emcmotConfig->jointMoveVel[axis-1]=velocity;
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}
int CTRL_GetJointMaxVelocity(f64 *velocity,u32 axis)
{
	if(axis>=1&&axis<=6)
	{
		*velocity=emcmotConfig->jointMoveVel[axis-1];
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}

int CTRL_SetJointMaxAcc(f64 acc,u32 axis)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change set joint ACC while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_JOINT_ACC_WHILE_ENABLED_TYPE;
	}
	if(axis>=1&&axis<=6)
	{
		emcmotConfig->jointMoveAcc[axis-1]=acc;
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}
int CTRL_GetJointMaxAcc(f64 *acc,u32 axis)
{
	if(axis>=1&&axis<=6)
	{
		*acc=emcmotConfig->jointMoveAcc[axis-1];
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}



int CTRL_SetReductionRate(s64 demoninator, s64 numerator, u32 axis)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change reduction rate while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_RATIO_WHILE_ENABLED_TYPE;
	}
	if(axis>=1&&axis<=6)
	{
		emcmotConfig->mechTransRatio[axis-1]=(f64)numerator/(f64)demoninator;
		
		emcmotConfig->mechTransDenominator[axis-1]=demoninator;
		emcmotConfig->mechTransNumerator[axis-1]=numerator;
		printf("aixs:%d: Denominator=%lld\r\n，Numerator=%lld ",axis,demoninator,numerator);
		
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}
int CTRL_GetReductionRate(s64 *demoninator, s64 *numerator,u32 axis)
{
	if(axis>=1&&axis<=6)
	{
		*demoninator=emcmotConfig->mechTransDenominator[axis-1];
		*numerator=emcmotConfig->mechTransNumerator[axis-1];	
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}

int CTRL_SetCoupleRate(s64 rate)
{
	emcmotConfig->coupleRate=rate;
	return 0;
}

int CTRL_GetCoupleRate(s64 *rate)
{
	*rate=emcmotConfig->coupleRate;
	return 0;
}



int CTRL_UserCalibration(ThreeJointPoints *jp, ToolCoordianteInformation *tool,UserCoordianteInformation *out)
{
	double tempJoints[6];
	PmCartesian origin,ox,oy;
	PmCartesian VecX,VecY,VecZ;
	RobotPose world;
	PmRotationMatrix  Ehom;
	PmQuaternion tempQuaternion;
	KINEMATICS_FORWARD_FLAGS fflags;
	KINEMATICS_INVERSE_FLAGS iflags;
	int i=0;
	double angle;
	PmRpy tempRpy;
	PmCartesian tcf;
	PmPose  orgT,EXT,EYT,TTool,TORG,TXT,TYT;
	PmEulerZyx tempZyx;
	PmRotationMatrix rotT;
	tempZyx.x=tool->rx*PM_PI/180.0;
	tempZyx.y=tool->ry*PM_PI/180.0;
	tempZyx.z=tool->rz*PM_PI/180.0;
	pmZyxQuatConvert(tempZyx,&tempQuaternion);
	TTool.rot=tempQuaternion;
	TTool.tran.x=tool->x;
	TTool.tran.y=tool->y;
	TTool.tran.z=tool->z;
	
	tempJoints[0]=jp->org.j1;
	tempJoints[1]=jp->org.j2;
	tempJoints[2]=jp->org.j3;
	tempJoints[3]=jp->org.j4;
	tempJoints[4]=jp->org.j5;
	tempJoints[5]=jp->org.j6;
	kinematicsForward(tempJoints,&world,&Ehom,&fflags,&iflags);
	origin=world.tran;
	orgT.tran=world.tran;
	pmMatQuatConvert(Ehom,&tempQuaternion);
	orgT.rot=tempQuaternion;
	pmPosePoseMult(orgT,TTool,&TORG);

	tempJoints[0]=jp->ox.j1;
	tempJoints[1]=jp->ox.j2;
	tempJoints[2]=jp->ox.j3;
	tempJoints[3]=jp->ox.j4;
	tempJoints[4]=jp->ox.j5;
	tempJoints[5]=jp->ox.j6;
	kinematicsForward(tempJoints,&world,&Ehom,&fflags,&iflags);
	ox=world.tran;
	EXT.tran=world.tran;
	pmMatQuatConvert(Ehom,&tempQuaternion);
	EXT.rot=tempQuaternion;
	pmPosePoseMult(EXT,TTool,&EXT);
	
	tempJoints[0]=jp->oy.j1;
	tempJoints[1]=jp->oy.j2;
	tempJoints[2]=jp->oy.j3;
	tempJoints[3]=jp->oy.j4;
	tempJoints[4]=jp->oy.j5;
	tempJoints[5]=jp->oy.j6;
	kinematicsForward(tempJoints,&world,&Ehom,&fflags,&iflags);
	oy=world.tran;

	EYT.tran=world.tran;
	pmMatQuatConvert(Ehom,&tempQuaternion);
	EYT.rot=tempQuaternion;
	pmPosePoseMult(EYT,TTool,&TYT);
	

/*
	for(i=0;i<3;i++)
	{
		tempJoints[0]=jp[i].j1;
		tempJoints[1]=jp[i].j2;
		tempJoints[2]=jp[i].j3;
		tempJoints[3]=jp[i].j4;
		tempJoints[4]=jp[i].j5;
		tempJoints[5]=jp[i].j6;
		kinematicsForward(tempJoints,&world,&Ehom,&fflags,&iflags);
		switch(i)
		{
		case 0:
			origin=world.tran;
			break;
		case 1:
			ox=world.tran;
			break;
		case 2:
			oy=world.tran;
			break;
		default:
			break;
		}
	}
	*/
	pmCartCartSub(ox,origin,&VecX);
	pmCartUnit(VecX,&VecX);
	pmCartCartSub(oy,origin,&oy);
	pmCartUnit(oy,&oy);
	pmCartCartDot(VecX,oy,&angle);
	if((1-angle)<AROUND_ZERO)
	{
		printf("userCalibaration failed \n");	
		return MOTION_ERROR_USR_CALIBARATION_FAILED_TYPE;
	}
	pmCartCartCross(VecX,oy,&VecZ);
	pmCartCartCross(VecZ, VecX,&VecY);


	PmHomogeneous hom;
	PmPose worldPose;



	Ehom.x=VecX;
	Ehom.y=VecY;
	Ehom.z=VecZ;

	

	printf("x.x=%f , x.y=%f ,x.z =%f \n", Ehom.x.x, Ehom.x.y, Ehom.x.z);
	printf("y.x=%f , y.y=%f ,y.z =%f \n", Ehom.y.x, Ehom.y.y, Ehom.y.z);
	printf("z.x=%f , z.y=%f ,z.z =%f \n", Ehom.z.x, Ehom.z.y, Ehom.z.z);

	//matrixToRpy(Ehom,&tempRpy);
	//PmQuaternion tempVec;
	////pmMatQuatConvert(Ehom,&tempVec);
	
	pmMatZyxConvert(Ehom, &tempZyx);
	//pmMatRpyConvert(Ehom,&tempRpy);
	out->x=TORG.tran.x;
	out->y=TORG.tran.y;
	out->z=TORG.tran.z;
	//out->a= tempRpy.p*180.0/PM_PI;
	//out->b= tempRpy.y*180.0/PM_PI;
	//out->c= tempRpy.r*180.0/PM_PI;
	out->a= tempZyx.x*180.0/PM_PI;
	out->b= tempZyx.y*180.0/PM_PI;
	out->c= tempZyx.z*180.0/PM_PI;
	printf("user calibration x=%f,y=%f,z=%f,a=%f, b=%f ,c=%f \n",out->x,out->y,out->z,out->a,out->b,out->c);

	//pmRpyQuatConvert(tempRpy, &worldPose.rot);
	//pmPoseHomConvert(worldPose, &hom);

	//Ehom = hom.rot;

	//pmQuatMatConvert(tempVec, &Ehom);
	//pmRpyMatConvert(tempRpy, &Ehom);


	//rpyToMatrix(tempRpy,&Ehom);
	pmZyxMatConvert(tempZyx,&Ehom);
	printf("x.x=%f , x.y=%f ,x.z =%f \n", Ehom.x.x, Ehom.x.y, Ehom.x.z);
	printf("y.x=%f , y.y=%f ,y.z =%f \n", Ehom.y.x, Ehom.y.y, Ehom.y.z);
	printf("z.x=%f , z.y=%f ,z.z =%f \n", Ehom.z.x, Ehom.z.y, Ehom.z.z);

	//matrixToRpy(Ehom, &tempRpy);

	//printf("user calibration x=%f,y=%f,z=%f,a=%f, b=%f ,c=%f \n", out->x, out->y, out->z, out->a, out->b, out->c);

	return 0;
}


/*
 this function shoud calculate the Tool coordinate information based on the position that was indicated by 
 the pointer jp.
 the jp shoud address at least 6 JointPoint variables, otherwise the outcome would be nonsense.
 the pointNumber shoud be 6 right now. In the futuer it should be able to be any integer that is greater than 4.
 And the value of the pointNumber should math the number of the JointPoint type array 
 that is input in this function.
At last, the coordinate information calculated would be put in the pointer out.
*/
 int CTRL_ToolCalibration(JointPoint *jp,int pointNumber,ToolCoordianteInformation *out)
{
	double tempJoints[6];
	PmCartesian descartesPoints[6],center,temp,PT;
	PmCartesian VecX,VecY,VecZ;
	RobotPose world;
	PmRotationMatrix  Ehom,Inv;
	KINEMATICS_FORWARD_FLAGS fflags;
	KINEMATICS_INVERSE_FLAGS iflags;
	double a11,a12,a13,a21,a22,a23,a31,a32,a33,b1,b2,b3,d,d1,d2,d3;
	double xt,yt,zt;
	int i=0;
	double angle;
	PmRpy tempRpy;
	PmEulerZyx tempZyx;
	PmRotationMatrix AE,AEInv,AT;
	if(pointNumber<6)
	{
		return -1;
	}
	
//get descarte coordinate positions
	for(i=0;i<6;i++)
	{
		tempJoints[0]=jp[i].j1;
		tempJoints[1]=jp[i].j2;
		tempJoints[2]=jp[i].j3;
		tempJoints[3]=jp[i].j4;
		tempJoints[4]=jp[i].j5;
		tempJoints[5]=jp[i].j6;	
		kinematicsForward(tempJoints,&world,&Ehom,&fflags,&iflags);
		if(i==3)
		{
			AE=Ehom;
		}
		descartesPoints[i]=world.tran;
	}
	a11=2*(descartesPoints[1].x-descartesPoints[0].x); 
	a12=2*(descartesPoints[1].y-descartesPoints[0].y);
	a13=2*(descartesPoints[1].z-descartesPoints[0].z);  
	a21=2*(descartesPoints[2].x-descartesPoints[1].x);
	a22=2*(descartesPoints[2].y-descartesPoints[1].y);
	a23=2*(descartesPoints[2].z-descartesPoints[1].z);  
	a31=2*(descartesPoints[3].x-descartesPoints[2].x);
	a32=2*(descartesPoints[3].y-descartesPoints[2].y);
	a33=2*(descartesPoints[3].z-descartesPoints[2].z);  
	b1=descartesPoints[1].x*descartesPoints[1].x-descartesPoints[0].x*descartesPoints[0].x+descartesPoints[1].y*descartesPoints[1].y-descartesPoints[0].y*descartesPoints[0].y+descartesPoints[1].z*descartesPoints[1].z-descartesPoints[0].z*descartesPoints[0].z;  
	b2=descartesPoints[2].x*descartesPoints[2].x-descartesPoints[1].x*descartesPoints[1].x+descartesPoints[2].y*descartesPoints[2].y-descartesPoints[1].y*descartesPoints[1].y+descartesPoints[2].z*descartesPoints[2].z-descartesPoints[1].z*descartesPoints[1].z;   
	b3=descartesPoints[3].x*descartesPoints[3].x-descartesPoints[2].x*descartesPoints[2].x+descartesPoints[3].y*descartesPoints[3].y-descartesPoints[2].y*descartesPoints[2].y+descartesPoints[3].z*descartesPoints[3].z-descartesPoints[2].z*descartesPoints[2].z;     
	d=a11*a22*a33+a12*a23*a31+a13*a21*a32-a11*a23*a32-a12*a21*a33-a13*a22*a31;  
	d1=b1*a22*a33+a12*a23*b3+a13*b2*a32-b1*a23*a32-a12*b2*a33-a13*a22*b3;  
	d2=a11*b2*a33+b1*a23*a31+a13*a21*b3-a11*a23*b3-b1*a21*a33-a13*b2*a31;  
	d3=a11*a22*b3+a12*b2*a31+b1*a21*a32-a11*b2*a32-a12*a21*b3-b1*a22*a31; 
	if(fabs(d)>AROUND_ZERO)
	{
		xt=d1/d;  
		yt=d2/d;  
		zt=d3/d;  
	d=Ehom.x.x*Ehom.y.y*Ehom.z.z+Ehom.x.y*Ehom.y.z*Ehom.z.x+Ehom.x.z*Ehom.y.x*Ehom.z.y
		-Ehom.x.x*Ehom.y.z*Ehom.z.y-Ehom.x.y*Ehom.y.x*Ehom.z.z-Ehom.x.z*Ehom.y.y*Ehom.z.x;
		center.x=xt/d;
		center.y=yt/d;
		center.z=zt/d;
		pmCartCartSub(center,descartesPoints[3],&temp);
		pmMatInv(AE, &Inv);
		pmMatCartMult(Inv,temp,&PT);
		out->x=PT.x;
		out->y=PT.y;
		out->z=PT.z;
	}
	emcmotDebug->robot_tcp.x = out->x;
	emcmotDebug->robot_tcp.y = out->y;
	emcmotDebug->robot_tcp.z = out->z;
	



//get the vectors for three axises
//	descartesPoints pmCartCartCross
	pmCartCartSub(descartesPoints[4],descartesPoints[3],&VecX);
	pmCartUnit(VecX,&VecX);

	pmCartCartSub(descartesPoints[5],descartesPoints[3],&VecY);
	pmCartUnit(VecY,&VecY);

	pmCartCartDot(VecX, VecY, &angle);
	if((1-angle)<AROUND_ZERO)
	{		
		return MOTION_ERROR_TOOL_CALIBARATION_FAILED_TYPE;
	}
	pmCartCartCross(VecX, VecY,&VecZ);
	pmCartUnit(VecZ,&VecZ);

//	pmCartCartCross(VecX, VecZ,&VecY);
//	pmCartUnit(VecY,&VecY);

	Ehom.x=VecX;
	Ehom.y=VecY;
	Ehom.z=VecZ;
	pmMatInv(AE,&AEInv);
	pmMatMatMult(AEInv, Ehom, &AT);
	pmMatZyxConvert(AT,&tempZyx);
	out->rx= tempZyx.x*180.0/PM_PI;
	out->ry= tempZyx.y*180.0/PM_PI;
	out->rz= tempZyx.z*180.0/PM_PI;
	
	return 0;

}

/*
 void CTRL_SetSocket(char * IP,int port ,int cycleTime)
{
	
	motConfigBuf.cycleTime = cycleTime;
	motConfigBuf.socketPortNumber = port;
	strcpy(motConfigBuf.socketIP, IP);

}
*/
 int CTRL_GetMotionStatus(MotionFeedback * fb)
{
	long int ret = 0;
	//ret = LOCK(Mutex);
	LOCK(Mutex);
	if (1)	// (0==ret)
	{	
		//printf("B");
		*fb = motionFb;
		if(motionFb.motionState==COMMAND_ERROR||motionFb.motionState==COMMAND_EXEC)
		{
			fb->motionState=motionFb.motionState;
		}
		else if(interpList->start!=interpList->end)
		{
			fb->motionState=COMMAND_EXEC;
			printf("temporay display \r\n");
		}
		else
		{
			fb->motionState=COMMAND_DONE;
		}
		UNLOCK(Mutex); 
	}
	else
	{
		printf("D");
		return ret;
	}
	return 0;
}



 void CTRL_GetJointValue(JointPoint * joint)
{

	joint->j1= robotJ0;
	joint->j2= robotJ1;
	joint->j3= robotJ2;
	joint->j4= robotJ3;
	joint->j5= robotJ4;
	joint->j6= robotJ5;

}

 double CTRL_GetJointValueFor3DTest(int jointNumber)
{
	switch (jointNumber)
	{
	case 1:
		return robotJ0;
		break;
	case 2:
		return robotJ1;
		break;
	case 3:
		return robotJ2;
		break;
	case 4:
		return robotJ3;
		break;
	case 5:
		return robotJ4;
		break;
	case 6:
		return robotJ5;
		break;
	default:
		printf("input joint number error , the input number is %d \n",jointNumber);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
		break;
	}
	return -500.0;
}

 double CTRL_GetPositionValue(int axis)
{
	switch (axis)
	{
	case 1:
		return x;
		break;
	case 2:
		return y;
		break;
	case 3:
		return z;
		break;
	default:
		printf("input axis number error , the input number is %d \n", axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
		break;
	}


	return -1000.0;
}






 int CTRL_ServoEnable(int enable)
{
	int i=0;
	int ret=0;
	if(enable!=0&&enable!=1)
	{
		//reportErrorType(MOTION_ERROR_INABLE_INPUT_ERROR_TYPE);
		printf("input enable error %d \n", enable);
		return MOTION_ERROR_INPUT_ENABLE_VALUE_ERROR_TYPE;
	}
	//printf("enter the enable function \n");
	//for(i=0;i<6;i++)
	//{
	//	if(g_servo[i]!=NULL)
	//	{
	//		if(enable!=emcmotStatus->enble)
	//		{
	//			ret=Servo_Enable(g_servo[i],enable);
	//			//printf("the return value of servo enable is %d ret\n",ret);
	//			if(0==ret)
	//			{
	//				//continue;
	//			}
	//			else
	//			{
	//				//printf("failed to enable the servo ret=%d\n",ret);
	//				return MOTION_ERROR_MOTION_ENABLE_FAILED_TYPE;
	//				//return ret;
	//			}		
	//		}
	//	}
	//	else
	//	{
	//		printf("can not get access to all servo\n");
	//		return MOTION_ERROR_CANNOT_ACCESS_SERVO_FAILED_TYPE;
	//	}
	//}
	//if(1==enable)
	//{
	//	emcmotDebug->enabling=1;
	//	tpSetJPos(&emcmotDebug->queue, emcmotStatus->joint_cmd);
	//}
	//else
	//{
	//	emcmotDebug->enabling=0;
	//}


	//return 0;

	motCmd->head++;
	motCmd->serialNumber++;
	motCmd->type = MOTION_COMMAND_SERVO_ENABLE_TYPE;
	if (enable != 0 && enable != 1)
	{
		printf("input enable error %d \n", enable);
		return -1;
	}
	motCmd->motionCmdParameter.svEable.enable = enable;

	motCmd->tail = motCmd->head;
	return 0;

}

 int CTRL_ServoReset(void)
{
	int i=0;
	int ret=0;
	//for(i=0;i<6;i++)
	//{
	//	if(g_servo[i]!=NULL)
	//	{
	//		ret=Servo_Reset(g_servo[i]);
	//		//printf("the return value of servo enable is %d ret\n",ret);
	//		if(0==ret)
	//		{
	//			continue;
	//		}
	//		else
	//		{
	//			printf("failed to reset the servo ret=%d\n",ret);
	//			return MOTION_ERROR_MOTION_RESET_FAILED_TYPE;
	//		}		
	//	}
	//	else
	//	{
	//		printf("can not get access to all servo\n");
	//		return MOTION_ERROR_CANNOT_ACCESS_SERVO_FAILED_TYPE;	//-1;
	//	}
	//}
	emcmotStatus->linkErr=0;

	motCmd->head++;
	motCmd->serialNumber++;
	motCmd->type = MOTION_COMMAND_SERVO_RESET_TYPE;
	
	motCmd->tail = motCmd->head;
	return 0;	// 0;

	
}



 void CTRL_MovementStop()
{
	motCmd->head++;
	motCmd->serialNumber++;
	motCmd->type = MOTION_COMMAND_MOVEMENT_STOP_TYPE;
	motCmd->tail = motCmd->head;
	return ;
}


 int CTRL_AddSingleAxisMove(SingleAxisMove *singleMove)
{
	
	motCmd->head++;
	motCmd->serialNumber++;
	motCmd->type = MOTION_COMMAND_SINGLEAXISMOVE_TYPE;
	if (singleMove->vel>1.0 || singleMove->vel<0.001)
	{
		//reportErrorType(MOTION_ERROR_SINGLE_AXIS_MOVE_VELOCITY_OFR_TYPE);
		printf("input velocity out of range %f \n", singleMove->vel);
		return MOTION_ERROR_SINGLE_AXIS_MOVE_VELOCITY_OFR_TYPE;
	}
	motCmd->motionCmdParameter.singleMove.vel = singleMove->vel;
	if (singleMove->axis>6 || singleMove->axis<1)
	{
		//reportErrorType(MOTION_ERROR_SINGLE_AXIS_MOVE_AXIS_OFR_TYPE);
		printf("axis number not in the rang axis=%d\n", singleMove->axis);
		return MOTION_ERROR_SINGLE_AXIS_MOVE_AXIS_OFR_TYPE;
	}
	motCmd->motionCmdParameter.singleMove.axis = singleMove->axis;
	if (MOVE_DIR_POSITIVE != singleMove->direction&&MOVE_DIR_NEGATIVE != singleMove->direction)
	{
		//reportErrorType(MOTION_ERROR_SINGLE_AXIS_MOVE_DIR_ERROR_TYPE);
		printf("direction error %d \n", singleMove->direction);
		return MOTION_ERROR_SINGLE_AXIS_MOVE_DIR_ERROR_TYPE;
	}
	//printf("add single axis move\n");
	motCmd->motionCmdParameter.singleMove = *singleMove;
	//add coordinate information later
	motCmd->tail = motCmd->head;
	motionFb.motionState=COMMAND_EXEC;
	return 0;
}
 int CTRL_AddJointMove(JointMoveInformation *jointMove,u32 lineNumber,u32 fileNumber)
{
	Command_PTR command;
	if ((interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE == interpList->start)
	{
		//reportErrorType(MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE );
		printf("command buffer is full \n");
		return MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE;
	}
	//interpList->end++;
	if (interpList->end>MOTION_COMMAD_QUEUE_SIZE)
	{
		interpList->end = interpList->end%MOTION_COMMAD_QUEUE_SIZE;
	}
	else if (interpList->end<0)
	{
		interpList->end = 0;
	}
	command = &(interpList->motionCmd[interpList->end]);
	command->head++;
	command->serialNumber++;
	command->currentLineNumber=lineNumber;
	command->currentFileNumber=fileNumber;
	command->type = MOTION_COMMAND_JOINTMOVE_TYPE;
	if (jointMove->vJ<0.001 || jointMove->vJ>1.0)
	{
		//reportErrorType(MOTION_ERROR_JOINT_MOVE_VELOCITY_ERROR_TYPE);
		printf("joint move velocity scale out of range %f", jointMove->vJ);
		return MOTION_ERROR_JOINT_MOVE_VELOCITY_ERROR_TYPE;
	}
	else
	{
		command->motionCmdParameter.movJ.vJ = jointMove->vJ;
	}
	if (jointMove->acc<0.0001 || jointMove->acc>1.0)
	{
		//reportErrorType(MOTION_ERROR_JOINT_MOVE_ACC_ERROR_TYPE);
		printf("joint move acceleration scale out of range %f", jointMove->acc);
		return MOTION_ERROR_JOINT_MOVE_ACC_ERROR_TYPE;
	}
	else
	{
		command->motionCmdParameter.movJ.acc = jointMove->acc;
	}
	if (jointMove->dec<0.0001 || jointMove->dec>1.0)
	{
		//reportErrorType(MOTION_ERROR_JOINT_MOVE_DEC_ERROR_TYPE);
		printf("joint move acceleration scale out of range %f", jointMove->acc);
		return MOTION_ERROR_JOINT_MOVE_DEC_ERROR_TYPE;
	}
	else
	{
		command->motionCmdParameter.movJ.dec = jointMove->dec;
	}
	command->motionCmdParameter.movJ.endPoint = jointMove->endPoint;

	command->tail = command->head;
	interpList->end = (interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE;
	motionFb.motionState=COMMAND_EXEC;
	return 0;
}



 int CTRL_AddLinearMove(LinearMoveInformation *movl,u32 lineNumber,u32 fileNumber)
		
 {
	Command_PTR command;
	if ((interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE == interpList->start)
	{
		//reportErrorType(MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE);
		printf("command buffer is full \n");
		return MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE;
	}
	//interpList->end++;
	if (interpList->end>MOTION_COMMAD_QUEUE_SIZE)
	{
		interpList->end = interpList->end%MOTION_COMMAD_QUEUE_SIZE;
	}
	else if (interpList->end<0)
	{
		interpList->end = 0;
	}
	command = &(interpList->motionCmd[interpList->end]);
	command->head++;
	command->serialNumber++;
	command->currentLineNumber=lineNumber;
	command->currentFileNumber=fileNumber;
	command->type = MOTION_COMMAND_LINEARMOVE_TYPE;
	if(movl->vL<0.0)
	{
		//reportErrorType(MOTION_ERROR_LINEAR_VELOCITY_OFR_TYPE);
		printf("linear move velocity out of range %f", movl->acc);
		return MOTION_ERROR_LINEAR_VELOCITY_OFR_TYPE;
	}
	if(movl->vR<0.0)
	{
		//reportErrorType(MOTION_ERROR_ROTATION_VELOCITY_OFR_TYPE);
		printf("linear move rotate velocity out of range %f", movl->acc);
		return MOTION_ERROR_ROTATION_VELOCITY_OFR_TYPE;
	}
	if (movl->acc<0.0001 || movl->acc>1.0)
	{
		//reportErrorType(MOTION_ERROR_LINEAR_MOVE_ACC_ERROR_TYPE);
		printf("linear move acceleration scale out of range %f", movl->acc);
		return MOTION_ERROR_LINEAR_MOVE_ACC_ERROR_TYPE;
	}
	else
	{
		command->motionCmdParameter.movL.acc = movl->acc;
	}
	if (movl->dec<0.0001 || movl->dec>1.0)
	{
		
		//reportErrorType(MOTION_ERROR_LINEAR_MOVE_DEC_ERROR_TYPE);
		printf("linear move acceleration scale out of range %f", movl->acc);
		return MOTION_ERROR_LINEAR_MOVE_DEC_ERROR_TYPE;
	}
	else
	{
		command->motionCmdParameter.movL.dec = movl->dec;
	}


	command->motionCmdParameter.movL.vL = movl->vL;
	command->motionCmdParameter.movL.endPoint = movl->endPoint;

	command->tail = command->head;
	interpList->end = (interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE;
	motionFb.motionState=COMMAND_EXEC;
	return 0;
}

 int CTRL_AddCircularMove(CircularMoveInformation *movC,u32 lineNumber,u32 fileNumber)
{
	Command_PTR command;
	if ((interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE == interpList->start)
	{
		//reportErrorType(MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE);
		printf("command buffer is full \n");
		return MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE;
	}
	//interpList->end++;
	if (interpList->end>MOTION_COMMAD_QUEUE_SIZE)
	{
		interpList->end = interpList->end%MOTION_COMMAD_QUEUE_SIZE;
	}
	else if (interpList->end<0)
	{
		interpList->end = 0;
	}
	command = &(interpList->motionCmd[interpList->end]);
	command->head++;
	command->serialNumber++;
	command->currentLineNumber=lineNumber;
	command->currentFileNumber=fileNumber;
	command->type = MOTION_COMMAND_CIRCULARMOVE_TYPE;

	if(movC->vL<0.0)
	{
		//reportErrorType(MOTION_ERROR_CIRCULAR_VELOCITY_OFR_TYPE);
		printf("circular move velocity out of range %f", movC->acc);
		return MOTION_ERROR_CIRCULAR_VELOCITY_OFR_TYPE;
	}
	if(movC->vR<0.0)
	{
		//reportErrorType(MOTION_ERROR_CIRCULAR_ROTATION_VELOCITY_OFR_TYPE);
		printf("circular move rotate velocity out of range %f", movC->acc);
		return MOTION_ERROR_CIRCULAR_ROTATION_VELOCITY_OFR_TYPE;
	}
	if (movC->acc<0.0001 || movC->acc>1.0)
	{
		//reportErrorType(MOTION_ERROR_CIRCULAR_MOVE_ACC_ERROR_TYPE);
		printf("circular move acceleration scale out of range %f", movC->acc);
		return MOTION_ERROR_CIRCULAR_MOVE_ACC_ERROR_TYPE;
	}
	else
	{
		command->motionCmdParameter.movC.acc = movC->acc;
	}
	if (movC->dec<0.0001 || movC->dec>1.0)
	{
		//reportErrorType(MOTION_ERROR_CIRCULAR_MOVE_DEC_ERROR_TYPE);
		printf("circular move acceleration scale out of range %f", movC->acc);
		return MOTION_ERROR_CIRCULAR_MOVE_DEC_ERROR_TYPE;
	}
	else
	{
		command->motionCmdParameter.movC.dec = movC->dec;
	}


	command->motionCmdParameter.movC.vL = movC->vL;
	command->motionCmdParameter.movC.endPoint[0] = movC->endPoint[0];
	command->motionCmdParameter.movC.endPoint[1] = movC->endPoint[1];
	command->motionCmdParameter.movC.endPoint[2] = movC->endPoint[2];
	command->motionCmdParameter.movC = *movC;
	command->tail = command->head;
	interpList->end = (interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE;
	motionFb.motionState=COMMAND_EXEC;
	return 0;
}
#ifdef TEST
 int CTRL_AddDescartesLinearMove(double x,double y,double z,int type)
{
	Command_PTR command;
	if ((interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE == interpList->start)
	{
		printf("command buffer is full \n");
		return -1;
	}
	if (interpList->end>MOTION_COMMAD_QUEUE_SIZE)
	{
		interpList->end = interpList->end%MOTION_COMMAD_QUEUE_SIZE;
	}
	else if (interpList->end<0)
	{
		interpList->end = 0;
	}
	command = &(interpList->motionCmd[interpList->end]);
	command->head++;
	command->serialNumber++;
	command->type = MOTION_COMMAND_DESCARTES_LINEARMOVE_TYPE;
	switch(type)
	{
		case 0:
			command->motionCmdParameter.movDL.free=1;
			break;
		case 1:
			command->motionCmdParameter.movDL.rot.x.x=0;
			command->motionCmdParameter.movDL.rot.x.y=0;
			command->motionCmdParameter.movDL.rot.x.z=1;

			command->motionCmdParameter.movDL.rot.y.x=0;
			command->motionCmdParameter.movDL.rot.y.y=-1;
			command->motionCmdParameter.movDL.rot.y.z=0;
			
			command->motionCmdParameter.movDL.rot.z.x=1;
			command->motionCmdParameter.movDL.rot.z.y=0;
			command->motionCmdParameter.movDL.rot.z.z=0;
			break;
		case 2:
			command->motionCmdParameter.movDL.rot.x.x=0;
			command->motionCmdParameter.movDL.rot.x.y=0;
			command->motionCmdParameter.movDL.rot.x.z=-1;

			command->motionCmdParameter.movDL.rot.y.x=0;
			command->motionCmdParameter.movDL.rot.y.y=-1;
			command->motionCmdParameter.movDL.rot.y.z=0;
			
			command->motionCmdParameter.movDL.rot.z.x=-1;
			command->motionCmdParameter.movDL.rot.z.y=0;
			command->motionCmdParameter.movDL.rot.z.z=0;
			break;
		case 3:
			command->motionCmdParameter.movDL.rot.x.x=-1;
			command->motionCmdParameter.movDL.rot.x.y=0;
			command->motionCmdParameter.movDL.rot.x.z=0;

			command->motionCmdParameter.movDL.rot.y.x=0;
			command->motionCmdParameter.movDL.rot.y.y=-1;
			command->motionCmdParameter.movDL.rot.y.z=0;
			
			command->motionCmdParameter.movDL.rot.z.x=0;
			command->motionCmdParameter.movDL.rot.z.y=0;
			command->motionCmdParameter.movDL.rot.z.z=1;
			break;
		case 4:
			command->motionCmdParameter.movDL.rot.x.x=1;
			command->motionCmdParameter.movDL.rot.x.y=0;
			command->motionCmdParameter.movDL.rot.x.z=0;

			command->motionCmdParameter.movDL.rot.y.x=0;
			command->motionCmdParameter.movDL.rot.y.y=-1;
			command->motionCmdParameter.movDL.rot.y.z=0;
			
			command->motionCmdParameter.movDL.rot.z.x=0;
			command->motionCmdParameter.movDL.rot.z.y=0;
			command->motionCmdParameter.movDL.rot.z.z=-1;
			break;
		case 5:
			command->motionCmdParameter.movDL.rot.x.x=1;
			command->motionCmdParameter.movDL.rot.x.y=0;
			command->motionCmdParameter.movDL.rot.x.z=0;

			command->motionCmdParameter.movDL.rot.y.x=0;
			command->motionCmdParameter.movDL.rot.y.y=0;
			command->motionCmdParameter.movDL.rot.y.z=-1;
			
			command->motionCmdParameter.movDL.rot.z.x=0;
			command->motionCmdParameter.movDL.rot.z.y=1;
			command->motionCmdParameter.movDL.rot.z.z=0;
			break;
		case 6:
			command->motionCmdParameter.movDL.rot.x.x=1;
				command->motionCmdParameter.movDL.rot.x.y=0;
			command->motionCmdParameter.movDL.rot.x.z=0;

			command->motionCmdParameter.movDL.rot.y.x=0;
			command->motionCmdParameter.movDL.rot.y.y=0;
			command->motionCmdParameter.movDL.rot.y.z=1;
			
			command->motionCmdParameter.movDL.rot.z.x=0;
			command->motionCmdParameter.movDL.rot.z.y=-1;
			command->motionCmdParameter.movDL.rot.z.z=0;
			break;
		default:
			printf("can not deal with this kind of tcf type %d \n",type);
			break;
	}
	command->motionCmdParameter.movDL.vL=100;
	command->motionCmdParameter.movDL.vR=50;
	command->motionCmdParameter.movDL.endPoint.x=x;
	command->motionCmdParameter.movDL.endPoint.y=y;
	command->motionCmdParameter.movDL.endPoint.z=z;

	command->tail = command->head;
	interpList->end = (interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE;
	motionFb.motionState=COMMAND_EXEC;
	return 0;
}
 int CTRL_AddDescartesCircularMove(double x1,double y1,double z1,double x2,double y2,double z2,double x3,double y3,double z3)
{
	int i=0;
	Command_PTR command;
	if ((interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE == interpList->start)
	{
		printf("command buffer is full \n");
		return -1;
	}
	if (interpList->end>MOTION_COMMAD_QUEUE_SIZE)
	{
		interpList->end = interpList->end%MOTION_COMMAD_QUEUE_SIZE;
	}
	else if (interpList->end<0)
	{
		interpList->end = 0;
	}
	command = &(interpList->motionCmd[interpList->end]);
	command->head++;
	command->type = MOTION_COMMAND_DESCARTES_CIRCULARMOVE_TYPE;
	//command->motionCmdParameter.movDC.endPoint[1]=movDC.endPoint[1];
	//command->motionCmdParameter.movDC.endPoint[2]=movDC.endPoint[2];
	command->motionCmdParameter.movDC.free=1;
	command->motionCmdParameter.movDC.endPoint[0].x=x1;
	command->motionCmdParameter.movDC.endPoint[0].y=y1;
	command->motionCmdParameter.movDC.endPoint[0].z=z1;

	command->motionCmdParameter.movDC.endPoint[1].x=x2;
	command->motionCmdParameter.movDC.endPoint[1].y=y2;
	command->motionCmdParameter.movDC.endPoint[1].z=z2;

	command->motionCmdParameter.movDC.endPoint[2].x=x3;
	command->motionCmdParameter.movDC.endPoint[2].y=y3;
	command->motionCmdParameter.movDC.endPoint[2].z=z3;

	for(i=0;i<3;i++)
		{
		//command->motionCmdParameter.movDC.rot[i].x=0.0;
		//command->motionCmdParameter.movDC.rot[i].y=0.0;
		//command->motionCmdParameter.movDC.rot[i].z=0.0;
		}	
	command->motionCmdParameter.movDC.vL=100;
	command->motionCmdParameter.movDC.vR=100;
	command->motionCmdParameter.movDC.acc=1.0;
	command->motionCmdParameter.movDC.dec=1.0;
	command->tail=command->head;
	interpList->end = (interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE;
	motionFb.motionState=COMMAND_EXEC;
	return 0;
}
#else
 int CTRL_AddDescartesLinearMove(LinearDescartesMoveInformation *movDL, u32 lineNumber, u32 fileNumber)
{
	Command_PTR command;
	if ((interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE == interpList->start)
	{
		//reportErrorType(MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE);
		printf("command buffer is full \n");
		return MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE;
	}
	if (interpList->end>MOTION_COMMAD_QUEUE_SIZE)
	{
		interpList->end = interpList->end%MOTION_COMMAD_QUEUE_SIZE;
	}
	else if (interpList->end<0)
	{
		interpList->end = 0;
	}
	if(movDL->vL<0.0)
	{
		//reportErrorType(MOTION_ERROR_LINEAR_VELOCITY_OFR_TYPE);
		printf("linear move velocity out of range %f", movDL->acc);
		return MOTION_ERROR_LINEAR_VELOCITY_OFR_TYPE;
	}
	if(movDL->vR<0.0)
	{
		//reportErrorType(MOTION_ERROR_ROTATION_VELOCITY_OFR_TYPE);
		printf("linear move rotate velocity out of range %f", movDL->acc);
		return MOTION_ERROR_ROTATION_VELOCITY_OFR_TYPE;
	}
	if (movDL->acc<0.0001 || movDL->acc>1.0)
	{
		//reportErrorType(MOTION_ERROR_LINEAR_MOVE_ACC_ERROR_TYPE);
		printf("linear move acceleration scale out of range %f", movDL->acc);
		return MOTION_ERROR_LINEAR_MOVE_ACC_ERROR_TYPE;
	}	
	if (movDL->dec<0.0001 || movDL->dec>1.0)
	{
		//reportErrorType(MOTION_ERROR_LINEAR_MOVE_DEC_ERROR_TYPE);
		printf("linear move acceleration scale out of range %f", movDL->acc);
		return MOTION_ERROR_LINEAR_MOVE_DEC_ERROR_TYPE;
	}
	
	command = &(interpList->motionCmd[interpList->end]);
	command->head++;
	command->serialNumber++;
	command->currentLineNumber=lineNumber;
	command->currentFileNumber=fileNumber;
	command->type = MOTION_COMMAND_DESCARTES_LINEARMOVE_TYPE;
	command->motionCmdParameter.movDL=*movDL;

	command->tail=command->head;
	interpList->end = (interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE;
	motionFb.motionState=COMMAND_EXEC;
	return 0;
}


 int CTRL_AddDescartesCircularMove(CircularDescartesMoveInformation *movDC,u32 lineNumber,u32 fileNumber)
{
	
	int i=0;
	Command_PTR command;
	if ((interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE == interpList->start)
	{
		//reportErrorType(MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE);
		printf("command buffer is full \n");
		return MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE;
	}
	if (interpList->end>MOTION_COMMAD_QUEUE_SIZE)
	{
		interpList->end = interpList->end%MOTION_COMMAD_QUEUE_SIZE;
	}
	else if (interpList->end<0)
	{
		interpList->end = 0;
	}
	if(movDC->vL<0.0)
	{
		//reportErrorType(MOTION_ERROR_CIRCULAR_VELOCITY_OFR_TYPE);
		printf("circular move velocity out of range %f", movDC->acc);
		return MOTION_ERROR_CIRCULAR_VELOCITY_OFR_TYPE;
	}
	if(movDC->vR<0.0)
	{
		//reportErrorType(MOTION_ERROR_CIRCULAR_ROTATION_VELOCITY_OFR_TYPE);
		printf("circular move rotate velocity out of range %f", movDC->acc);
		return MOTION_ERROR_CIRCULAR_ROTATION_VELOCITY_OFR_TYPE;
	}
	if (movDC->acc<0.0001 || movDC->acc>1.0)
	{
		//reportErrorType(MOTION_ERROR_CIRCULAR_MOVE_ACC_ERROR_TYPE);
		printf("circular move acceleration scale out of range %f", movDC->acc);
		return MOTION_ERROR_CIRCULAR_MOVE_ACC_ERROR_TYPE;
	}	
	if (movDC->dec<0.0001 || movDC->dec>1.0)
	{
		//reportErrorType(MOTION_ERROR_CIRCULAR_MOVE_DEC_ERROR_TYPE);
		printf("circular move acceleration scale out of range %f", movDC->acc);
		return MOTION_ERROR_CIRCULAR_MOVE_DEC_ERROR_TYPE;
	}

	command = &(interpList->motionCmd[interpList->end]);
	command->head++;
	command->type = MOTION_COMMAND_DESCARTES_CIRCULARMOVE_TYPE;
	command->serialNumber++;
	command->currentLineNumber=lineNumber;
	command->currentFileNumber=fileNumber;
	//command->motionCmdParameter.movDC.endPoint[1]=movDC.endPoint[1];
	//command->motionCmdParameter.movDC.endPoint[2]=movDC.endPoint[2];
	command->motionCmdParameter.movDC=*movDC;
	//command->motionCmdParameter.movDC.free=movDC->free;
	//for(i=0;i<3;i++)
	//{
	//	command->motionCmdParameter.movDC.endPoint[i]=movDC->endPoint[i];
	//	command->motionCmdParameter.movDC.rot[i]=movDC->rot[i];
	//}
	//command->motionCmdParameter.movDC.vL=movDC->vL;
	//command->motionCmdParameter.movDC.vR=movDC->vR;
	
	//command->motionCmdParameter.movDC.acc=movDC->acc;
	//command->motionCmdParameter.movDC.dec=movDC->dec;
	
	command->tail=command->head;
	interpList->end = (interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE;
	motionFb.motionState=COMMAND_EXEC;
	return 0;
}

#endif


 int CTRL_Init(long us)
{
	INIT_LOCK(Mutex);
	
	//获取伺服信息
	int i=0;	
	int ret1,ret2,ret3,ret4,ret5,ret6;
	double period = (double)us;

	/*g_servo[0]=MOTION_GetServo("J1");
	g_servo[1]=MOTION_GetServo("J2");
	g_servo[2]=MOTION_GetServo("J3");
	g_servo[3]=MOTION_GetServo("J4");
	g_servo[4]=MOTION_GetServo("J5");
	g_servo[5]=MOTION_GetServo("J6");*/
	/*for(i=0;i<6;i++)
	{
		if(NULL==g_servo[i])
		{
			printf("can not get the servo information \n");
			return MOTION_ERROR_CANNOT_ACCESS_SERVO_FAILED_TYPE;
		}
		if(Servo_Init(g_servo[i]))
		{
			printf("can not init the servo struct \n");
			return MOTION_ERROR_CANNOT_INIT_SERVO_FAILED_TYPE;			
		}
		Servo_Refresh(g_servo[i]);
	}*/
	if (us <= 0)
	{
		printf("cycle time error %ld", us);
		return MOTION_ERROR_CYCLE_TIME_ERROR_TYPE;
	}
	//initialize some motion state....
	//first put some default value in to the inner variable
	// motion status
	PmJoint temp;

	emcmotStatus->commandEcho = -1;
	emcmotStatus->commandNumEcho = 0;
	emcmotStatus->commandStatus = EMCMOT_COMMAND_OK;
	emcmotStatus->motion_state = EMCMOT_MOTION_DISABLED;
	emcmotStatus->IObyte = -1; //do we still need it now?
	emcmotStatus->rapidMode = -1; //delete it later
	emcmotStatus->joint_type = 0; // 0 1 2 blending? , so now we do not need it.
	emcmotStatus->config_num = 0; // configuration changed? we don't need it now.

	emcmotStatus->net_feed_scale = 0; // freed? 
	emcmotStatus->linkErr = 0; //  we do not have any error now.
	emcmotStatus->distance_was_gone = 0; //come from tc.progress, i don't think it is necessary now
	emcmotStatus->distance_to_go = 0; //  samo samo
	emcmotStatus->motionFlag = 0x00000002; // if we are now just considering the algorithm this flag  should be a stable status and let it be
	emcmotStatus->motErrorFlag = 0; // we're not considering about any faults here at this moment, so....., let it be no faults.
	emcmotStatus->id = 0; //
	(void)emcmotStatus->fileName; //just don't touch it, or maybe i should just delete it.
	//emcmotStatus->coordinate_type = 0; //
	emcmotStatus->on_soft_limit = 0; //just as its name, when we are passing the limit, we need to stop,but...,do we need do something more?

	emcmotStatus->estop = 0; //useless, now
	emcmotStatus->pause = 0; //useless, now
	emcmotStatus->playing = 0; //now we never care about whether we are playing or what,right?
	emcmotStatus->enble = 0; //always enabled, right? what about joints flag?

	emcmotStatus->running = 0; //
	emcmotStatus->queueFull = 0; //
	emcmotStatus->linkAlmFlag = 0; //
	emcmotStatus->linkAlmReport = 0; //
	emcmotStatus->speed_scale = 150.0; //  i don't think we need it now, now we are running under a specific velocity environment

	emcmotStatus->joint_cmd.j0 = 0.0;
	emcmotStatus->joint_cmd.j1 = 0.0;
	emcmotStatus->joint_cmd.j2 = 0.0;
	emcmotStatus->joint_cmd.j3 = 0.0;
	emcmotStatus->joint_cmd.j4 = 90.0;
	emcmotStatus->joint_cmd.j5 = 0.0;


	//motion configuration
	emcmotConfig->PUMA_A[0] = PUMA_A1;
	emcmotConfig->PUMA_A[1] = PUMA_A2;
	emcmotConfig->PUMA_A[2] = PUMA_A3;
	emcmotConfig->PUMA_A[3] = PUMA_D4;
	emcmotConfig->PUMA_A[4] = PUMA_D6;

	emcmotConfig->teachLinearV = 100.0;
	emcmotConfig->playLinearV = 7000.0;
	emcmotConfig->linearMaxV = 7000.0;
	emcmotConfig->linearMoveAcc = 35000.0;
	emcmotConfig->rotateMoveAcc = 100.0;
	emcmotConfig->rotateMoveV = 100.0;
	emcmotConfig->linearPosLimit[0] = 1000000.0;
	emcmotConfig->linearPosLimit[1] = 1000000.0;
	emcmotConfig->linearPosLimit[2] = 1000000.0;
	emcmotConfig->linearPosLimit[3] = 180.0;
	emcmotConfig->linearPosLimit[4] = 180.0;
	emcmotConfig->linearPosLimit[5] = 180.0;

	emcmotConfig->linearNegaLimit[0] = -1000.0;
	emcmotConfig->linearNegaLimit[1] = -1000.0;
	emcmotConfig->linearNegaLimit[2] = -1000.0;
	emcmotConfig->linearNegaLimit[3] = -180.0;
	emcmotConfig->linearNegaLimit[4] = -180.0;
	emcmotConfig->linearNegaLimit[5] = -180.0;
	emcmotDebug->linkSimTest = 1;
	emcmotDebug->playing = 0;// never play again
	emcmotDebug->enabling = 0;
	for (int i = 0; i<6; i++)
	{
		emcmotConfig->jointPosLimit[i] = 180.0;
		emcmotConfig->jointNegaLimit[i] = -180;
		emcmotConfig->jointMoveVel[i] = 180.0;
		emcmotConfig->jointMoveAcc[i] = 300.0;
		emcmotConfig->jointMoveMaxVel[i] = 180.0;
		emcmotConfig->mechTransRatio[i] = 1.0;
		//emcmotConfig->servoPulsePerRound[i] = g_servo[i]->plusPerRound;
		emcmotConfig->servoPulsePerRound[i] = 36000;
		emcmotConfig->ServoMaxVelocity[i] = 1000;
		//emcmotConfig->encodeOriginOffset[i] = 0;
		//emcmotConfig->servoPolarity[i]=g_servo[i]->servoPolarity;
		//emcmotConfig->encoderType[i]=g_servo[i]->encoderType;
	}


	emcmotConfig->mechTransRatio[0] = 101.0;
	emcmotConfig->mechTransRatio[1] = -101.0;
	emcmotConfig->mechTransRatio[2] = 101.0;
	emcmotConfig->mechTransRatio[3] = -173.142857;
	emcmotConfig->mechTransRatio[4] = 101.0;
	emcmotConfig->mechTransRatio[5] = -51.0;
	emcmotConfig->coupleRate=51;
	emcmotConfig->mechTransDenominator[0]=1;
	emcmotConfig->mechTransDenominator[1]=-1;
	emcmotConfig->mechTransDenominator[2]=1;
	emcmotConfig->mechTransDenominator[3]=14;
	emcmotConfig->mechTransDenominator[4]=1;
	emcmotConfig->mechTransDenominator[5]=-1;
	
	emcmotConfig->mechTransNumerator[0]=101;
	emcmotConfig->mechTransNumerator[1]=101;
	emcmotConfig->mechTransNumerator[2]=101;
	emcmotConfig->mechTransNumerator[3]=2424;
	emcmotConfig->mechTransNumerator[4]=101;
	emcmotConfig->mechTransNumerator[5]=51;

	joints[0].pos_cmd = 0.0;
	joints[1].pos_cmd = 0.0;
	joints[2].pos_cmd = 0.0;
	joints[3].pos_cmd = 0.0;
	joints[4].pos_cmd = 90.0;
	joints[5].pos_cmd = 0.0;
	
	//joints
	for (int i = 0; i<MAX_JOINTS; i++)
	{
		joints[i].motorInputScale = 1.0;	//we don't care about the motor now
		joints[i].motorInverseInputScale = 1.0;
		joints[i].svtype = 0;
		joints[i].not_first_input = 0;
		joints[i].max_pos_limit = 1000.0;
		joints[i].min_pos_limit = -1000.0;
		joints[i].min_jog_limit = -3600.0;
		joints[i].max_jog_limit = 3600.0;
		joints[i].Jvel_limit = 360.0;
		joints[i].vel_limit = 360.0;
		joints[i].acc_limit = 1000.0;
		joints[i].min_ferror = 50.0;
		joints[i].max_ferror = 50.0;
		//joints[i].servoEncoderPolarity = (emcmotConfig->servoPolarity[i]== eServoPolarityAnticlockwise)?1:0;
		joints[i].servoEncoderPolarity = 1;
		/* init status info */
		joints[i].motorInputOffset = 0;
		joints[i].offsetInit = 0;
		joints[i].flag = 2;
		joints[i].errorFlag = 0;
		joints[i].coarse_pos = 0.0;
		//joints[i].pos_cmd = 0.0;
		joints[i].vel_cmd = 0.0;
		joints[i].pos_fb = 0.0;
		joints[i].ferror = 0.0;
		joints[i].ferror_limit = joints[i].min_ferror;
		joints[i].ferror_high_mark = 0.0;
		joints[i].encoder_counts = emcmotConfig->servoPulsePerRound[i];
	}
	joints[0].max_jog_limit=170;
	joints[0].min_jog_limit=-170;
	joints[1].max_jog_limit=160;
	joints[1].min_jog_limit=-60;
	joints[2].max_jog_limit=210;
	joints[2].min_jog_limit=-65;
	joints[3].max_jog_limit=180;
	joints[3].min_jog_limit=-180;
	
	joints[4].max_jog_limit=135;
	joints[4].min_jog_limit=-135;
	joints[5].max_jog_limit=360;
	joints[5].min_jog_limit=-359.9;

	for (int i = 0; i<6; i++)
	{
		emcmotConfig->jointPosLimit[i] = joints[i].max_jog_limit;
		emcmotConfig->jointNegaLimit[i] = joints[i].min_jog_limit;
	}



	if (-1 == tpCreate(&emcmotDebug->queue, DEFAULT_TC_QUEUE_SIZE, emcmotDebug->queueTcSpace))
	{
		printf("can not init the motion tp\n");
		return MOTION_ERROR_CANNOT_CREAT_TP_TYPE;
	}
	tpSetCycleTime(&emcmotDebug->queue, period/1000000);
	//    tpSetPos(&emcmotDebug->queue, emcmotStatus->carte_pos_cmd);
	tpSetJPos(&emcmotDebug->queue, emcmotStatus->joint_cmd);

}
 void CTRL_Exit(void)
{
	// pthread_mutexattr_destroy(&Mutex);
}
 int CTRL_GetTCPInUserSpace(PointPose *pos,JointPoint *joint,UserCoordianteInformation* user,ToolCoordianteInformation* tool)
{
	PmHomogeneous TBE;
	PmHomogeneous TET,TBU,TUT,tempT;
	PmHomogeneous TBUIN;
	PmPose PBE,PET,PBU,PUT,PBUIN;
	RobotPose world;
	PmRotationMatrix  Ehom;
	double tempJoints[6];
	
	KINEMATICS_FORWARD_FLAGS fflags;
	KINEMATICS_INVERSE_FLAGS iflags;
	int i = 0;
	PmEulerZyx toolZyx,userZyx,tempZyx;
	tempJoints[0]=joint->j1;
	tempJoints[1]=joint->j2;
	tempJoints[2]=joint->j3;
	tempJoints[3]=joint->j4;
	tempJoints[4]=joint->j5;
	tempJoints[5]=joint->j6;
	kinematicsForward(tempJoints,&world,&Ehom,&fflags,&iflags);
	
	
	TBE.tran.x=world.tran.x;
	TBE.tran.y=world.tran.y;
	TBE.tran.z=world.tran.z;
	
	TBE.rot=Ehom;

	TET.tran.x=tool->x;
	TET.tran.y=tool->y;
	TET.tran.z=tool->z;

	toolZyx.x=tool->rx*PM_PI/180.0;
	toolZyx.y=tool->ry*PM_PI/180.0;
	toolZyx.z=tool->rz*PM_PI/180.0;

	pmZyxMatConvert(toolZyx,&(TET.rot));

	TBU.tran.x=user->x;
	TBU.tran.y=user->y;
	TBU.tran.z=user->z;

	userZyx.x=user->a*PM_PI/180.0;
	userZyx.y=user->b*PM_PI/180.0;
	userZyx.z=user->c*PM_PI/180.0;

	pmZyxMatConvert(userZyx,&(TBU.rot));
	


	pmHomInv(TBU,&TBUIN);



	pmMatMatMult(TBUIN.rot, TBE.rot,&(tempT.rot));
	pmMatCartMult(TBUIN.rot,TBE.tran,&(tempT.tran));
	pmCartCartAdd(tempT.tran,TBUIN.tran,&(tempT.tran));


	pmMatMatMult(tempT.rot, TET.rot,&(TUT.rot));
	pmMatCartMult(tempT.rot,TET.tran,&(TUT.tran));
	pmCartCartAdd(TUT.tran, tempT.tran,&(TUT.tran));



/*	
	pmMatMatMult(TBE.rot, TET.rot,&(tempT.rot));
	pmMatCartMult(TBE.rot,TET.tran,&(tempT.tran));
	pmCartCartAdd(tempT.tran,TBE.tran,&(tempT.tran));
	
	pmMatMatMult(tempT.rot, TBUIN.rot,&(TUT.rot));
	pmMatCartMult(tempT.rot,TBUIN.tran,&(TUT.tran));
	pmCartCartAdd(TUT.tran, tempT.tran,&(TUT.tran));
*/
	pmMatZyxConvert(TUT.rot,&tempZyx);
	
	pos->x=TUT.tran.x;
	pos->y=TUT.tran.y;
	pos->z=TUT.tran.z;
	pos->a=tempZyx.x*180.0/PM_PI;
	pos->b=tempZyx.y*180.0/PM_PI;
	pos->c=tempZyx.z*180.0/PM_PI;

	return 0 ;

}
 int CTRL_GetTCPInUserSpaceInMatrix(PmHomogeneous *pos,JointPoint *joint,UserCoordianteInformation* user,ToolCoordianteInformation* tool)
 {
 	PmHomogeneous TBE;
	PmHomogeneous TET,TBU,TUT,tempT;
	PmHomogeneous TBUIN;
	PmPose PBE,PET,PBU,PUT,PBUIN;
	RobotPose world;
	PmRotationMatrix  Ehom;
	double tempJoints[6];
	
	KINEMATICS_FORWARD_FLAGS fflags;
	KINEMATICS_INVERSE_FLAGS iflags;
	int i = 0;
	PmEulerZyx toolZyx,userZyx,tempZyx;
	tempJoints[0]=joint->j1;
	tempJoints[1]=joint->j2;
	tempJoints[2]=joint->j3;
	tempJoints[3]=joint->j4;
	tempJoints[4]=joint->j5;
	tempJoints[5]=joint->j6;
	kinematicsForward(tempJoints,&world,&Ehom,&fflags,&iflags);
	
	
	TBE.tran.x=world.tran.x;
	TBE.tran.y=world.tran.y;
	TBE.tran.z=world.tran.z;
	
	TBE.rot=Ehom;

	TET.tran.x=tool->x;
	TET.tran.y=tool->y;
	TET.tran.z=tool->z;

	toolZyx.x=tool->rx*PM_PI/180.0;
	toolZyx.y=tool->ry*PM_PI/180.0;
	toolZyx.z=tool->rz*PM_PI/180.0;

	pmZyxMatConvert(toolZyx,&(TET.rot));

	TBU.tran.x=user->x;
	TBU.tran.y=user->y;
	TBU.tran.z=user->z;

	userZyx.x=user->a*PM_PI/180.0;
	userZyx.y=user->b*PM_PI/180.0;
	userZyx.z=user->c*PM_PI/180.0;

	pmZyxMatConvert(userZyx,&(TBU.rot));
	


	pmHomInv(TBU,&TBUIN);



	pmMatMatMult(TBUIN.rot, TBE.rot,&(tempT.rot));
	pmMatCartMult(TBUIN.rot,TBE.tran,&(tempT.tran));
	pmCartCartAdd(tempT.tran,TBUIN.tran,&(tempT.tran));


	pmMatMatMult(tempT.rot, TET.rot,&(TUT.rot));
	pmMatCartMult(tempT.rot,TET.tran,&(TUT.tran));
	pmCartCartAdd(TUT.tran, tempT.tran,&(TUT.tran));



/*	
	pmMatMatMult(TBE.rot, TET.rot,&(tempT.rot));
	pmMatCartMult(TBE.rot,TET.tran,&(tempT.tran));
	pmCartCartAdd(tempT.tran,TBE.tran,&(tempT.tran));
	
	pmMatMatMult(tempT.rot, TBUIN.rot,&(TUT.rot));
	pmMatCartMult(tempT.rot,TBUIN.tran,&(TUT.tran));
	pmCartCartAdd(TUT.tran, tempT.tran,&(TUT.tran));
*/
	
	*pos=TUT;

	return 0 ;
 }
 int CTRL_GetTCPInJoint(JointPoint *joint,PointPose *point,UserCoordianteInformation* user,ToolCoordianteInformation* tool)
{
	PmHomogeneous TBE,TET,TBU,TUT,tempT;
	PmHomogeneous TETIN;
	RobotPose world;
	PmRotationMatrix  Ehom;
	double tempJoints[6];
	double pre_joints[6];
	JointPoint startJoint;
	PointPose startPoint;
	int ret=0;
	KINEMATICS_FORWARD_FLAGS fflags;
	KINEMATICS_INVERSE_FLAGS iflags;

	PmEulerZyx toolZyx,userZyx,tempZyx;
	
	startJoint.j1=emcmotDebug->queue.goalJPos.j0;
	startJoint.j2=emcmotDebug->queue.goalJPos.j1;
	startJoint.j3=emcmotDebug->queue.goalJPos.j2;
	startJoint.j4=emcmotDebug->queue.goalJPos.j3;
	startJoint.j5=emcmotDebug->queue.goalJPos.j4;
	startJoint.j6=emcmotDebug->queue.goalJPos.j5;

	CTRL_GetTCPInUserSpace(&startPoint,&startJoint,user,tool);
	if(point->x>=1e99)
	{
		TUT.tran.x=startPoint.x;
	}
	else
	{
		TUT.tran.x=point->x;
	}
	if(point->y>=1e99)
	{
		TUT.tran.y=startPoint.y;
	}
	else
	{
		TUT.tran.y=point->y;
	}
	if(point->z>=1e99)
	{
		TUT.tran.z=startPoint.z;
	}
	else
	{
		TUT.tran.z=point->z;
	}
	if(point->a>=1e99)
	{
		toolZyx.x=startPoint.a*PM_PI/180.0;
	}
	else
	{
		toolZyx.x=point->a*PM_PI/180.0;
	}
	if(point->b>=1e99)
	{
		toolZyx.y=startPoint.b*PM_PI/180.0;
	}
	else
	{
		toolZyx.y=point->b*PM_PI/180.0;
	}
	if(point->c>=1e99)
	{
		toolZyx.z=startPoint.c*PM_PI/180.0;
	}
	else
	{
		toolZyx.z=point->c*PM_PI/180.0;
	}


	TBU.tran.x=user->x;
	TBU.tran.y=user->y;
	TBU.tran.z=user->z;
	
	userZyx.x=user->a*PM_PI/180.0;
	userZyx.y=user->b*PM_PI/180.0;
	userZyx.z=user->c*PM_PI/180.0;
	pmZyxMatConvert(userZyx, &(TBU.rot));

//	TUT.tran.x=point->x;
//	TUT.tran.y=point->y;
//	TUT.tran.z=point->z;

//	toolZyx.r=point->a*PM_PI/180.0;
//	toolZyx.p=point->b*PM_PI/180.0;
//	toolZyx.y=point->c*PM_PI/180.0;

	pmZyxMatConvert(toolZyx,&(TUT.rot));

	TET.tran.x=tool->x;
	TET.tran.y=tool->y;
	TET.tran.z=tool->z;

	toolZyx.x=tool->rx*PM_PI/180.0;
	toolZyx.y=tool->ry*PM_PI/180.0;
	toolZyx.z=tool->rz*PM_PI/180.0;

	pmZyxMatConvert(toolZyx, &(TET.rot));
	pmHomInv(TET,&TETIN);

	pmMatMatMult(TBU.rot, TUT.rot,&(tempT.rot));
	pmMatCartMult(TBU.rot,TUT.tran,&(tempT.tran));
	pmCartCartAdd(tempT.tran,TBU.tran,&(tempT.tran));


	pmMatMatMult(tempT.rot, TETIN.rot,&(TBE.rot));
	pmMatCartMult(tempT.rot,TETIN.tran,&(TBE.tran));
	pmCartCartAdd(TBE.tran,tempT.tran,&(TBE.tran));
	world.tran=TBE.tran;
	pre_joints[0]= emcmotDebug->queue.goalJPos.j0;
	pre_joints[1]= emcmotDebug->queue.goalJPos.j1;
	pre_joints[2]= emcmotDebug->queue.goalJPos.j2;
	pre_joints[3]= emcmotDebug->queue.goalJPos.j3;
	pre_joints[4]= emcmotDebug->queue.goalJPos.j4;
	pre_joints[5]= emcmotDebug->queue.goalJPos.j5;

	ret=kinematicsInverse( &world,&TBE,tempJoints,pre_joints,0);
	joint->j1=tempJoints[0];
	joint->j2=tempJoints[1];
	joint->j3=tempJoints[2];
	joint->j4=tempJoints[3];
	joint->j5=tempJoints[4];
	joint->j6=tempJoints[5];
	if(ret)
	{
		return MOTION_ERROR_KINEMATICSINVERSE_FAILED_TYPE;
	}
	return 0;
}

 int CTRL_GetTCPInJointInMatrix(JointPoint *joint,JointPoint *preJoint,PmHomogeneous *point,UserCoordianteInformation* user,ToolCoordianteInformation* tool)
 {
	PmHomogeneous TBE,TET,TBU,TUT,tempT;
	PmHomogeneous TETIN;
	RobotPose world;
	PmRotationMatrix  Ehom;
	double tempJoints[6];
	double pre_joints[6];
	int ret=0;
	KINEMATICS_FORWARD_FLAGS fflags;
	KINEMATICS_INVERSE_FLAGS iflags;

	PmEulerZyx toolZyx,userZyx,tempZyx;

	TBU.tran.x=user->x;
	TBU.tran.y=user->y;
	TBU.tran.z=user->z;

	userZyx.x=user->a*PM_PI/180.0;
	userZyx.y=user->b*PM_PI/180.0;
	userZyx.z=user->c*PM_PI/180.0;

	pmZyxMatConvert(userZyx, &(TBU.rot));

	TUT.tran=point->tran;
	TUT.rot=point->rot;

//	toolRpy.r=point->a*PM_PI/180.0;
//	toolRpy.p=point->b*PM_PI/180.0;
//	toolRpy.y=point->c*PM_PI/180.0;
//	pmRpyMatConvert(toolRpy,&(TUT.rot));

	TET.tran.x=tool->x;
	TET.tran.y=tool->y;
	TET.tran.z=tool->z;

	toolZyx.x=tool->rx*PM_PI/180.0;
	toolZyx.y=tool->ry*PM_PI/180.0;
	toolZyx.z=tool->rz*PM_PI/180.0;

	pmZyxMatConvert(toolZyx, &(TET.rot));
	pmHomInv(TET,&TETIN);

	pmMatMatMult(TBU.rot, TUT.rot,&(tempT.rot));
	pmMatCartMult(TBU.rot,TUT.tran,&(tempT.tran));
	pmCartCartAdd(tempT.tran,TBU.tran,&(tempT.tran));


	pmMatMatMult(tempT.rot, TETIN.rot,&(TBE.rot));
	pmMatCartMult(tempT.rot,TETIN.tran,&(TBE.tran));
	pmCartCartAdd(TBE.tran,tempT.tran,&(TBE.tran));
	world.tran=TBE.tran;
	pre_joints[0]=preJoint->j1;
	pre_joints[1]=preJoint->j2;
	pre_joints[2]=preJoint->j3;
	pre_joints[3]=preJoint->j4;
	pre_joints[4]=preJoint->j5;
	pre_joints[5]=preJoint->j6;

	ret=kinematicsInverse( &world,&TBE,tempJoints,pre_joints,0);
	joint->j1=tempJoints[0];
	joint->j2=tempJoints[1];
	joint->j3=tempJoints[2];
	joint->j4=tempJoints[3];
	joint->j5=tempJoints[4];
	joint->j6=tempJoints[5];
	if(ret)
	{
		return MOTION_ERROR_KINEMATICSINVERSE_FAILED_TYPE;
	}
	return 0;
 }


 int CTRL_SCARA_KineticForward(JointPoint *jp,PmHomogeneous *pos)
 {
 	double s[3],c[3];
 	s[0]=sin(180/3.1415926*jp->j1);
 	c[0]=cos(180/3.1415926*jp->j1);
 	s[1]=sin(180/3.1415926*jp->j2);
 	c[1]=cos(180/3.1415926*jp->j2);
 	s[2]=sin(180/3.1415926*jp->j3);
 	c[2]=cos(180/3.1415926*jp->j3);
 		
 	
 	pos->tran.x=(c[0]*c[1]-s[0]*s[1])*SCARA_L2+c[0]*SCARA_L1;
 	pos->tran.y=(s[0]*c[1]+c[0]*s[1])*SCARA_L2+s[0]*SCARA_L1;
 	pos->tran.z=jp->j4;

	pos->rot.x.x=c[0]*c[1]*c[2]-s[0]*s[1]*c[2]-c[1]*s[1]*s[2]-s[0]*c[1]*s[2];
	pos->rot.x.y=s[0]*c[1]*c[2]+c[0]*s[1]*c[3]-s[0]*s[1]*s[2]+c[0]*c[1]*s[2];
	pos->rot.x.z=0.0;
	pos->rot.y.x=-c[0]*c[1]*s[2]+s[0]*s[1]*s[2]-c[0]*s[1]*s[2]-s[0]*c[1]*c[2];
	pos->rot.y.y=-s[0]*c[1]*s[2]-c[0]*s[1]*s[2]-s[0]*s[1]*c[2]+c[0]*c[1]*c[2];
	pos->rot.y.z=0.0;
	pos->rot.z.x=0.0;
	pos->rot.z.y=0.0;
	pos->rot.z.z=1;
 	return 0;
 }
 

int CTRL_SCARA_KineticInverse(PmHomogeneous *pos,JointPoint *jp)
{
	double th1,th2,th3,th23;
	double x,y;
	double c1,c2,s1,c23;
	x=pos->tran.x;
	y=pos->tran.y;
	c2=(x*x+y*y-SCARA_L1*SCARA_L1-SCARA_L2*SCARA_L2)/SCARA_L1*SCARA_L2;
	th2=acos(c2);
	s1=sin(th2);
	c1=(c2*SCARA_L2*x+SCARA_L1*x+s1*SCARA_L2*y)/(x*x+y*y);
	th1=acos(c1);

	c23=c1*pos->rot.x.x+s1*pos->rot.x.y;
	th23=acos(c23);

	th3=th23-th2;
	jp->j1=th1;
	jp->j2=th2;
	jp->j3=th3;
	jp->j4=pos->tran.z;


	return 0;
}

 int CTRL_SCARA_KineticForwardInABC(JointPoint *jp,PointPose *point)
 {
	PmHomogeneous a;
	PmHomogeneous *pos=&a;
	PmEulerZyx zyx;


 	double s[3],c[3];
	double th123,c123,s123;
 	s[0]=sin(3.1415926*jp->j1/180.0);
 	c[0]=cos(3.1415926*jp->j1/180.0);
 	s[1]=sin(3.1415926*jp->j2/180.0);
 	c[1]=cos(3.1415926*jp->j2 / 180.0);
 	s[2]=sin(3.1415926*jp->j3 / 180.0);
 	c[2]=cos(3.1415926*jp->j3 / 180.0);
 		
	th123 = jp->j1 + jp->j2 + jp->j3;
	c123 = cos(th123*3.1415926/180.0);
	s123 = sin(th123*3.1415926 / 180.0);
 	pos->tran.x=(c[0]*c[1]-s[0]*s[1])*SCARA_L2+c[0]*SCARA_L1;
 	pos->tran.y=(s[0]*c[1]+c[0]*s[1])*SCARA_L2+s[0]*SCARA_L1;
 	pos->tran.z=jp->j4;

	pos->rot.x.x=c123;
	pos->rot.x.y=s123;
	pos->rot.x.z=0.0;
	pos->rot.y.x=-s123;
	pos->rot.y.y=c123;
	pos->rot.y.z=0.0;
	pos->rot.z.x=0.0;
	pos->rot.z.y=0.0;
	pos->rot.z.z=1;	

	point->x=pos->tran.x;
	point->y=pos->tran.y;
	point->z=pos->tran.z;
	
	pmMatZyxConvert(pos->rot,&zyx);
	point->a=zyx.x*180.0/3.1415926;
	point->b=zyx.y*180.0 / 3.1415926;
	point->c=zyx.z*180.0 / 3.1415926;
	
 
 	return 0;
 }
 int CTRL_SCARA_KineticInverseInABC(PointPose *point,JointPoint *jp)
 {
	PmHomogeneous a;
	PmHomogeneous *pos=&a;
	PmEulerZyx zyx;
	double th1,th2,th3,th23;
	double x,y;
	double c1,c2,s1,s2,c23;
	zyx.x=point->a/180.0*3.1415926;
	zyx.y=point->b / 180.0*3.1415926;
	zyx.z=point->c / 180.0*3.1415926;
	
	pmZyxMatConvert(zyx, &(pos->rot));
	pos->tran.x=point->x;
	pos->tran.y=point->y;
	pos->tran.z=point->z;


	x=pos->tran.x;
	y=pos->tran.y;
	c2=(x*x+y*y-SCARA_L1*SCARA_L1-SCARA_L2*SCARA_L2)/(2*SCARA_L1*SCARA_L2);
	th2=acos(c2);
	s2=sin(th2);
	c1=(c2*SCARA_L2*x+SCARA_L1*x+s2*SCARA_L2*y)/(x*x+y*y);
	th1=acos(c1);
	s1 = sin(th1);
	c23=c1*pos->rot.x.x+s1*pos->rot.x.y;
	th23=acos(c23);
	th23 = th23*180.0 / 3.1415926;
	jp->j1=th1/3.1415926*180.0;
	jp->j2=th2/3.1415926*180.0;
	jp->j3=th23-jp->j2;
	jp->j4=pos->tran.z;	
	


	
 	return 0;
 }


