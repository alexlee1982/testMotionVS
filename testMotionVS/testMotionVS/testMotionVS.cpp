// testMotionVS.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "libCtrl\include\contrller.h"
#include <stdio.h>  
#include <windows.h>  
#include "libCtrl\src\tp.h"

DWORD WINAPI CallBack(LPVOID lpParam)
{
	int i = 0;
	time_t timer;
	while (1)
	{	
		CTRL_Callback(1000);
		Sleep(1);
	}
}

int main()
{
	//PmCartesian center;
	//PmCartesian A,B,C;
	//A.x = 0;
	//A.y = 0;
	//A.z = 0;

	//B.x = 5.0;
	//B.y = 5.0;
	//B.z = 5;

	//C.x = 10.0;
	//C.y = 0;
	//C.z = 0;

	//center = threePointCircle(A,B,C);

	//double lcA, lcB, lcC;
	//PmCartesian temp;
	//pmCartCartSub(center, A, &temp);
	//pmCartMag(temp, &lcA);

	//pmCartCartSub(center, B, &temp);
	//pmCartMag(temp, &lcB);

	//pmCartCartSub(center, C, &temp);
	//pmCartMag(temp, &lcC);



	CircularMoveInformation movC;
	CircularMoveInformation movCInv;
	JointMoveInformation mj;
	mj.acc = 1;
	mj.dec = 1;
	mj.vJ = 1.0;
	mj.endPoint.j1 = 0;
	mj.endPoint.j2 = 0;
	mj.endPoint.j3 = 0;
	mj.endPoint.j4 = 0;
	mj.endPoint.j5 = 90;
	mj.endPoint.j6 = 0;

	movC.acc = 1.0;
	movC.dec = 1.0;
	movC.vL = 100.0;
	movC.vR = 10.0;
	movC.endPoint[0].j1 = 0.0;
	movC.endPoint[0].j2 = 0.0;
	movC.endPoint[0].j3 = 0.0;
	movC.endPoint[0].j4 = 0.0;
	movC.endPoint[0].j5 = 90.0;
	movC.endPoint[0].j6 = 0.0;

	movC.endPoint[1].j1 = 0.0;
	movC.endPoint[1].j2 = 20.0;
	movC.endPoint[1].j3 = 30.0;
	movC.endPoint[1].j4 = 0.0;
	movC.endPoint[1].j5 = -90.0;
	movC.endPoint[1].j6 = 0.0;

	movC.endPoint[2].j1 = 10.0;
	movC.endPoint[2].j2 = 0.0;
	movC.endPoint[2].j3 = 0.0;
	movC.endPoint[2].j4 = 0.0;
	movC.endPoint[2].j5 = 45.0;
	movC.endPoint[2].j6 = 0.0;





	CTRL_Init(100);

	HANDLE pid_fun = CreateThread(
		NULL,              // default security attributes
		0,                 // use default stack size  
		CallBack,        // thread function 
		NULL,             // argument to thread function 
		0,                 // use default creation flags 
		NULL);           // returns the thread identifier 
	//create thread to exe callback	


	Sleep(1000);

	CTRL_ServoEnable(1);
	printf("enabled \n");


	CTRL_AddJointMove(&mj,0,0);

	CTRL_AddCircularMove(&movC, 1, 0);
	//CTRL_AddCircularMove(&movCInv, 1, 0);


	for (;;)
	{
		;
	}
	CloseHandle(pid_fun);
    return 0;
}

