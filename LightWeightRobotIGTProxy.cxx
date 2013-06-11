/*=========================================================================

  Program:   Open IGT Link -- Example for Tracker Server Program
  Module:    $RCSfile: $
  Language:  C++
  Date:      $Date: $
  Version:   $Revision: $

  Copyright (c) Insight Software Consortium. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include <iostream>
#include <math.h>
#include <cstdlib>

#include "igtlOSUtil.h"
#include "igtlTransformMessage.h"
#include "igtlServerSocket.h"


int mState;
int mRcvState; 
int mVFtype;
int RcvData_size;
int SendData_size;
int pointstoregister=4;
unsigned char RcvDataType=0;


//This Functions reads a User Input 

float ReadDouble(double fDefault)
{
    char  pStr[128];
    double fVal;

    gets(pStr);
    if (sscanf(pStr, "%lf", &fVal) != 1)
    {
        printf("%lf\n", fDefault);
        return fDefault;
    }
    return fVal;
}


int ReadInt(int nDefault)
{
    char pStr[128];
    int  nVal;

    gets(pStr);
    if (sscanf(pStr, "%d", &nVal) != 1)
    {
        printf("%d\n", nDefault);
        return nDefault;
    }
    return nVal;
}

//--------------end--------------------------------------------

int main(int argc, char* argv[])
{

 //------------------------------------------------------------
  // Parse Arguments

	if (argc != 2) // check number of arguments
	{
		// If not correct, print usage
		std::cerr << "Usage: " << argv[0] << " <port> <fps>"    << std::endl;
		std::cerr << "    <port>     : Port # (18944 in Slicer default)"   << std::endl;
		std::cerr << "    <fps>      : Frequency (fps) to send coordinate" << std::endl;
		exit(0);
	}

	int    port     = 22222;
	double fps      = atof(argv[1]);
	int    interval = (int) (1000.0 / fps);

	unsigned char tmpArr[8];
	unsigned char bufferSendtoRobot[128];
	unsigned char bufferRecievefrmRobot [128];

	double Transform[12];
	double ap[3], n[3], phi;
	double RegPoints[6][3];
	int j=0;

	igtl::TransformMessage::Pointer transMsg;
	transMsg = igtl::TransformMessage::New();
	transMsg->SetDeviceName("Tracker");
	std::cerr<< "Size of Double" << sizeof(double) <<std::endl;

	//Initializing the ServerSocket for the robot Communication
	igtl::ServerSocket::Pointer ProxyserverSocket;
	ProxyserverSocket = igtl::ServerSocket::New();
	int r = ProxyserverSocket->CreateServer(port);

	

	if (r < 0)
	{
		std::cerr << "Cannot create the robot server socket." << std::endl;
		exit(0);
	}

	igtl::ServerSocket::Pointer SlicerProxySocket;
	SlicerProxySocket = igtl::ServerSocket::New();
	int r2 = SlicerProxySocket->CreateServer(18944);
	if (r < 0)
	{
		std::cerr << "Cannot create the Slicer server socket." << std::endl;
		exit(0);
	}
	igtl::Socket::Pointer RobotSocket;
	igtl::Socket::Pointer SlicerSocket;

	//Reading the Virtual Fixture Definition from Terminal
	//later one this will be send by the 3D Slicer Module
	printf("\nDefine Virtual Fixtures \n");
	printf("VF typ (0 = plane 1=cone): ");
	mVFtype = (ReadInt('0') );
	printf("\n");
	printf("Aufpunkt ap x: ");
	ap[0] = (ReadDouble('0.0') );
	printf("\n");
	printf("y :");
	ap[1] = (ReadDouble('0.0') );
	printf("\n");
	printf("z :");
	ap[2] = (ReadDouble('0.0') );
	printf("\n");
	printf("Normalenvektor n x: ");
	n[0] = (ReadDouble('0.0') );
	printf("\n");
	printf("y :");
	n[1] = (ReadDouble('0.0') );
	printf("\n");
	printf("z :");
	n[2] = (ReadDouble('1.0') );
	printf("\n");
	
	if(mVFtype ==1){  //If the Virtual Fixture is a cone read the ankle
		printf("phi :");
		phi = (ReadDouble('90') );
		printf("\n");
	}

	std::cerr<< "Defined Virtual Fixtures!!"<<std::endl;

	printf("nNumber of Points for Registration: ");
		pointstoregister = (ReadInt('4') );
		printf("\n");

		 //------------------------------------------------------------
	// Waiting for Connection
	RobotSocket = ProxyserverSocket->WaitForConnection(1000);
	if(RobotSocket.IsNotNull()){
		std::cerr<<"RobotController is connected "<<std::endl;
	}else{
		std::cerr<<"ERROR: RobotController is not connected"<<std::endl;
	}
	SlicerSocket = SlicerProxySocket->WaitForConnection(1000);
	if(SlicerSocket.IsNotNull()){
		std::cerr<<"SlicerModule is connected"<<std::endl;
	}else{
		std::cerr<<"ERROR: SlicerModule is not connected"<<std::endl;
	}
  while (1)
    {
   

	//Set the Current State - Later this will be recieved from the Slicer Module
	 printf("\nSelect State Current State is %d ): ",
        mState);
	mState = (ReadInt('1') );

    printf("\n");
     
    if (RobotSocket.IsNotNull()) // if client connected
      {
			//Depending on the State send the data according to an easy Protocol
			//First byte Datatype : S= State, T=Transform and R=Registration Points
			//Second byte Requested/CurrentState
			//The Rest depends on the Datatyp
			switch (mState){
				case 0: //State IDLE
					bufferSendtoRobot[0] = 'S';
					bufferSendtoRobot[1] = mState;
					RcvData_size = 3;
					SendData_size = 2;
					break;
				case 1: //State Registration
					bufferSendtoRobot[0] = 'S';
					bufferSendtoRobot[1] = mState;
					bufferSendtoRobot[2] = pointstoregister;
					SendData_size = 3;
					RcvData_size = 4;
					break;
				case 2: //Recieving Data from Robot
					bufferSendtoRobot[0] = 'S';
					bufferSendtoRobot[1] = mState;
					SendData_size = 2;
					RcvData_size = 3 + pointstoregister*sizeof(double)*3;
					break;
				case 3: //Send T_CT_Base to robot
					bufferSendtoRobot[0] = 'S';
					bufferSendtoRobot[1] = mState;

					//Reading the Transform from User
					printf("\n");
					printf("Transformation R11: ");
					Transform[0] = (ReadDouble('0.0') );
					printf("\n");
					printf("R12: :\n");
					Transform[1] = (ReadDouble('0.0') );
					printf("\n");
					printf("R13: :\n");
					Transform[2] = (ReadDouble('0.0') );
					printf("\n");
					printf(" t x: ");
					Transform[3] = (ReadDouble('0.0') );
					printf("\n");

					printf(" R21: ");
					Transform[4] = (ReadDouble('0.0') );
					printf("\n");
					printf("R22: :\n");
					Transform[5] = (ReadDouble('0.0') );
					printf("\n");
					printf("R23: :\n");
					Transform[6] = (ReadDouble('0.0') );
					printf("\n");
					printf(" t y: ");
					Transform[7] = (ReadDouble('0.0') );
					printf("\n");

							 printf(" R31: ");
					Transform[8] = (ReadDouble('0.0') );
					printf("\n");
					printf("R32: :\n");
					Transform[9] = (ReadDouble('0.0') );
					printf("\n");
					printf("R33: :\n");
					Transform[10] = (ReadDouble('0.0') );
					printf("\n");
					 printf(" t z: ");
					Transform[11] = (ReadDouble('0.0') );
					printf("\n");
					
					//Storing double data into unsigned char buffer for Socket Communication
					for(int q=0; q<12; q++){
						memcpy(&tmpArr,&Transform[q],sizeof(double));
						for (j=0; j<sizeof(double); j++) bufferSendtoRobot[2+q*sizeof(double)+j] = tmpArr[j];
					}
					SendData_size = 2+12*sizeof(double);
					RcvData_size = 3;

					break;
				case 4: //GravComp
					bufferSendtoRobot[0] = 'S';
					bufferSendtoRobot[1] = mState;
					SendData_size = 2;
					RcvData_size = 3;
					break;
				case 5: // VirtualFixtures
					bufferSendtoRobot[0] = 'S';
					bufferSendtoRobot[1] = mState;
					bufferSendtoRobot[2] = mVFtype;
					
					//Storing the Virtual Fixtures definition in a unsigned char array for Socket Communication
					memcpy(&tmpArr,&ap[0],sizeof(double));
					for (j=0; j<sizeof(double); j++) bufferSendtoRobot[3+j] = tmpArr[j];

					memcpy(&tmpArr,&ap[1],sizeof(double));
					for (j=0; j<sizeof(double); j++) bufferSendtoRobot[3+1*sizeof(double)+j] = tmpArr[j];

					memcpy(&tmpArr,&ap[2],sizeof(double));
					for (j=0; j<sizeof(double); j++) bufferSendtoRobot[3+2*sizeof(double)+j] = tmpArr[j];

					memcpy(&tmpArr,&n[0],sizeof(double));
					for (j=0; j<sizeof(double); j++) bufferSendtoRobot[3+3*sizeof(double)+j] = tmpArr[j];

					memcpy(&tmpArr,&n[1],sizeof(double));
					for (j=0; j<sizeof(double); j++) bufferSendtoRobot[3+4*sizeof(double)+j] = tmpArr[j];

					memcpy(&tmpArr,&n[2],sizeof(double));
					for (j=0; j<sizeof(double); j++) bufferSendtoRobot[3+5*sizeof(double)+j] = tmpArr[j];
					if(mVFtype==1){
						memcpy(&tmpArr,&phi,sizeof(double));
						for (j=0; j<sizeof(double); j++) bufferSendtoRobot[3+6*sizeof(double)+j] = tmpArr[j];
					}
					SendData_size = 3+(7+mVFtype)*sizeof(double);
					RcvData_size = 3;
					break;
				case 6: //NavGravComp
					bufferSendtoRobot[0] = 'S';
					bufferSendtoRobot[1] = mState;
					SendData_size = 2;
					RcvData_size = 2 + 12*sizeof(double);
					break;
				case 7: // NavGravCompVF
					bufferSendtoRobot[0] = 'S';
					bufferSendtoRobot[1] = mState;
					bufferSendtoRobot[2] = mVFtype;


					//Storing the Virtual Fixtures definition in a unsigned char array for Socket Communication
					memcpy(&tmpArr,&ap[0],sizeof(double));
					for (j=0; j<sizeof(double); j++) bufferSendtoRobot[3+j] = tmpArr[j];

					memcpy(&tmpArr,&ap[1],sizeof(double));
					for (j=0; j<sizeof(double); j++) bufferSendtoRobot[3+1*sizeof(double)+j] = tmpArr[j];

					memcpy(&tmpArr,&ap[2],sizeof(double));
					for (j=0; j<sizeof(double); j++) bufferSendtoRobot[3+2*sizeof(double)+j] = tmpArr[j];

					memcpy(&tmpArr,&n[0],sizeof(double));
					for (j=0; j<sizeof(double); j++) bufferSendtoRobot[3+3*sizeof(double)+j] = tmpArr[j];

					memcpy(&tmpArr,&n[1],sizeof(double));
					for (j=0; j<sizeof(double); j++) bufferSendtoRobot[3+4*sizeof(double)+j] = tmpArr[j];

					memcpy(&tmpArr,&n[2],sizeof(double));
					for (j=0; j<sizeof(double); j++) bufferSendtoRobot[3+5*sizeof(double)+j] = tmpArr[j];
					if(mVFtype==1){
						memcpy(&tmpArr,&phi,sizeof(double));
						for (j=0; j<sizeof(double); j++) bufferSendtoRobot[3+6*sizeof(double)+j] = tmpArr[j];
					}
					SendData_size = 3+7*sizeof(double);
					RcvData_size = 2 + 12*sizeof(double);
					break;
				default:
					break;
			}

			// Send the data to the Programm on the robot Control
			RobotSocket->Send(bufferSendtoRobot, SendData_size );

			//Recieving Data from the Robot 
			int r =RobotSocket->Receive(bufferRecievefrmRobot, 128);// RcvData_size);

			RcvDataType = bufferRecievefrmRobot[0];
			std::cerr <<"Datatype Recieved"<< RcvDataType<< std::endl;
			mRcvState = bufferRecievefrmRobot[1];
			
			if(mState!=mRcvState){
					//Some Debugging
					std::cerr << "Cannot change State to " << mState << std::endl;
					std::cerr <<"Recieved State: " <<bufferRecievefrmRobot[1]<<" Transfered to: " <<mRcvState << std::endl;

			}
			//Datatype is Transform
			if (RcvDataType == 'T'){
				
				//IGT Matrix type to send the data to 3D Slicer
				igtl::Matrix4x4 matrix;
				
				//reinterpret the data
				for( int m = 0; m<3; m++){
					for(int n =0; n<4; n++){
						memcpy(&matrix[m][n],&bufferRecievefrmRobot[2+(m+n)*sizeof(double)],sizeof(double));
					}
				}
				matrix[3][0] = 0;
				matrix[3][1] = 0;
				matrix[3][2] = 0;
				matrix[3][3] = 1;
				transMsg->SetMatrix(matrix);
				transMsg->Pack();
				std::cerr<< "T_Current Pose:" <<std::endl;
				std::cerr<<matrix[0][0] <<" Y " << matrix[0][1] << "Z" <<matrix[0][2] << std::endl;
				std::cerr <<  matrix[1][0] <<" Y " << RegPoints[1][1] << "Z" <<matrix[1][2] << std::endl;
				std::cerr <<  matrix[2][0] <<" Y " << RegPoints[2][1] << "Z" <<matrix[2][2] << std::endl;
				std::cerr <<  matrix[3][0] <<" Y " << RegPoints[3][1] << "Z" <<matrix[3][2] << std::endl;
				//SlicerSocket->Send(transMsg->GetPackPointer(), transMsg->GetPackSize());

			}else if (RcvDataType == 'R'){ //Registration Points from Robot

				//reinterpret the data
				for( int m = 0; m<pointstoregister; m++){
					for(int n =0; n<3; n++){
						memcpy(&RegPoints[m][n],&bufferRecievefrmRobot[2+(m+n)*sizeof(double)],sizeof(double));
						//std::cerr<<bufferRecievefrmRobot[2+m*sizeof(double) + n];
					}
				}
				//std::cerr<<endl;
				 // If not correct, print usage
				std::cerr<< "Point 1: X " << RegPoints[0][0] <<" Y " << RegPoints[0][1] << "Z" <<RegPoints[0][2] << std::endl;
				std::cerr << "Point 2: X " << RegPoints[1][0] <<" Y " << RegPoints[1][1] << "Z" <<RegPoints[1][2] << std::endl;
				std::cerr << "Point 3: X " << RegPoints[2][0] <<" Y " << RegPoints[2][1] << "Z" <<RegPoints[2][2] << std::endl;
				std::cerr << "Point 4: X " << RegPoints[3][0] <<" Y " << RegPoints[3][1] << "Z" <<RegPoints[3][2] << std::endl;

			}
        }
      }
    
  //------------------------------------------------------------
  // Close connection (The example code never reachs to this section ...)
  
 ProxyserverSocket->CloseSocket();
 RobotSocket->CloseSocket();

}


