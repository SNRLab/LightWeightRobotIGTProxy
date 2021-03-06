 /*Module:    $RCSfile: $
  Language:  C++
  Date:      $Date: $
  Version:   $Revision: $

  Copyright (c) Insight Software Consortium. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#include <iostream>
#include <iomanip>
#include <math.h>
#include <cstdlib>
#include <cstring>

#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlTransformMessage.h"
#include "igtlImageMessage.h"
#include "igtlServerSocket.h"
#include "igtlStatusMessage.h"
#include "igtlPositionMessage.h"

#if OpenIGTLink_PROTOCOL_VERSION >= 2
#include "igtlPointMessage.h"
#include "igtlStringMessage.h"
#include "igtlBindMessage.h"
#endif //OpenIGTLink_PROTOCOL_VERSION >= 2

int ReceiveTransform(igtl::Socket * socket, igtl::MessageHeader * header);

#if OpenIGTLink_PROTOCOL_VERSION >= 2
int ReceivePoint(igtl::Socket * socket, igtl::MessageHeader * header);
int ReceiveString(igtl::Socket * socket, igtl::MessageHeader * header);
int SendRegistrationPoints(igtl::Socket * socket, double Points[6][3], std::string ID);
int SendString(igtl::Socket * socket, unsigned char buffer[], std::string ID);
#endif //OpenIGTLink_PROTOCOL_VERSION >= 2

double T_CT_Base[12];
double ap[3], n[3], phi;
int mState;
int mRcvState; 
int mOldState;
int mVFtype;
int RcvData_size;
int SendData_size;
int pointstoregister=4;
unsigned char RcvDataType=0;
bool mError;
std::string mUID;
std::string mErrorString;
bool mReceivedData=false;
bool RecievedTransform=false;

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



	double RegPoints[6][3];
	int j=0;

	
	igtl::TransformMessage::Pointer transMsg;
	transMsg = igtl::TransformMessage::New();
	transMsg->SetDeviceName("CurrentPose in Imagespace");

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
	
	 // Create a message buffer to receive header
      igtl::MessageHeader::Pointer headerMsg;
      headerMsg = igtl::MessageHeader::New();

		// Waiting for Connection
	RobotSocket = ProxyserverSocket->WaitForConnection(10000);
	if(RobotSocket.IsNotNull()){
		std::cerr<<"RobotController is connected "<<std::endl;
	}else{
		std::cerr<<"ERROR: RobotController is not connected"<<std::endl;
	}

	SlicerSocket = SlicerProxySocket->WaitForConnection(10000);
	if(SlicerSocket.IsNotNull()){
		std::cerr<<"SlicerModule is connected"<<std::endl;
	}else{
		std::cerr<<"ERROR: SlicerModule is not connected"<<std::endl;
	}
	SlicerSocket->SetReceiveTimeout(20);
	if(SlicerSocket.IsNull()){
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
	}
		 //------------------------------------------------------------

  while (1)
    {
	mOldState=mState;
	if(SlicerSocket.IsNull()){
		//Set the Current State - Later this will be recieved from the Slicer Module
		 printf("\nSelect State Current State is %d ): ",
			mState);
		mState = (ReadInt('1') );

		printf("\n");
	}else if(SlicerSocket.IsNotNull()){
		


		 // Initialize receive buffer
        headerMsg->InitPack();
		   //------------------------------------------------------------
		  // Allocate a time stamp 
		  igtl::TimeStamp::Pointer ts;
		  ts = igtl::TimeStamp::New();
			// Receive generic header from the socket
        int r =SlicerSocket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize());
        if (r == 0)
          {
		  mReceivedData=false;
          //SlicerSocket->CloseSocket();
          }
        if (r != headerMsg->GetPackSize())
          {
		  mReceivedData=false;
          continue;
		}else{
			mReceivedData=true;
		}
		if(mReceivedData){

			// Deserialize the header
			headerMsg->Unpack();

			// Get time stamp
			igtlUint32 sec;
			igtlUint32 nanosec;
	        
			headerMsg->GetTimeStamp(ts);
			ts->GetTimeStamp(&sec, &nanosec);

			// Check data type and receive data body
			if (strcmp(headerMsg->GetDeviceType(), "TRANSFORM") == 0)
			  {
			  ReceiveTransform(SlicerSocket, headerMsg);
			  }
		#if OpenIGTLink_PROTOCOL_VERSION >= 2
			else if (strcmp(headerMsg->GetDeviceType(), "STRING") == 0)
			  {
			  ReceiveString(SlicerSocket, headerMsg);
			  }
			else
			  {
				  std::cerr << "Unknown Datatype received from Slicer!"<<std::endl;
			  }
		#endif //OpenIGTLink_PROTOCOL_VERSION >= 2
		}

	}
	     


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
					if(RecievedTransform==true){
					//Dataset is includingTransform
					bufferSendtoRobot[2]= 1;
					if(SlicerSocket.IsNull()){
							//Reading the Transform from User
							for (int f =0; f<12; f++){
								printf("\n");
								printf("Transformation T_CT_Base[ %d]: ", f);
								T_CT_Base[f] = (ReadDouble('0.0') );
								printf("\n");
							}
						}
						//Storing double data into unsigned char buffer for Socket Communication
						for(int q=0; q<12; q++){
							memcpy(&tmpArr,&T_CT_Base[q],sizeof(double));
							for (j=0; j<sizeof(double); j++) bufferSendtoRobot[3+q*sizeof(double)+j] = tmpArr[j];
						}
						SendData_size = 3+12*sizeof(double);
					}else{
						//Dataset without Transform
						bufferSendtoRobot[2]= 0;
					}
					RcvData_size = 4;

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
					RcvData_size = 3 + 12*sizeof(double);
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
					RcvData_size = 3 + 12*sizeof(double);
					break;
				default:
					break;
			}

			// Send the data to the Programm on the robot Control
			if(mReceivedData||mState==6)RobotSocket->Send(bufferSendtoRobot, SendData_size );

			//Recieving Data from the Robot if new State was send or the Current Pose is send
			if(mReceivedData || mState==6){
				int r =RobotSocket->Receive(bufferRecievefrmRobot, 128);// RcvData_size);

				RcvDataType = bufferRecievefrmRobot[0];
				std::cerr <<"Datatype Recieved from Robot: "<< RcvDataType<< std::endl;
				
				mRcvState = bufferRecievefrmRobot[1];
				std::cerr <<"RecievedState from Robot: "<< mRcvState<< std::endl;
				
				if(mRcvState!=mState){
						//Some Debugging
						std::cerr << "Cannot change Robot State to " << mState << std::endl;
						mState=mOldState;
						std::cerr << "Reseted Robot State to " << mState << std::endl;
				}
				//Datatype is Transform
				if (RcvDataType == 'T'){
					
					//IGT Matrix type to send the data to 3D Slicer
					igtl::Matrix4x4 matrix;
					double tmpDouble;
					//reinterpret the data
					for( int m = 0; m<3; m++){
						for(int n =0; n<4; n++){
							memcpy(&tmpDouble,&bufferRecievefrmRobot[3+(m*4+n)*sizeof(double)],sizeof(double));
							matrix[m][n] = tmpDouble;
						}
					}
					matrix[3][0] = 0;
					matrix[3][1] = 0;
					matrix[3][2] = 0;
					matrix[3][3] = 1;
					transMsg->SetMatrix(matrix);
					transMsg->Pack();
					std::cerr<< "T_Current Pose from Robot:" <<std::endl;
					std::cerr<<matrix[0][0] <<"	" << matrix[0][1] << "	"     <<matrix[0][2] <<"	" <<matrix[0][3] << std::endl;
					std::cerr <<  matrix[1][0] <<"	" << matrix[1][1] << "	" <<matrix[1][2] <<"	" <<matrix[1][3] << std::endl;
					std::cerr <<  matrix[2][0] <<"	" << matrix[2][1] << "	" <<matrix[2][2] <<"	" << matrix[2][3] <<std::endl;
					std::cerr <<  matrix[3][0] <<"	" << matrix[3][1] << "	" <<matrix[3][2] <<"	" <<matrix[3][3] << std::endl;
					SlicerSocket->Send(transMsg->GetPackPointer(), transMsg->GetPackSize());
					SendString(SlicerSocket, bufferRecievefrmRobot, mUID);
				}else if (RcvDataType == 'R'){ //Registration Points from Robot
					std::string NameStr;

					//reinterpret the data
					for( int m = 0; m<pointstoregister; m++){
						for(int n =0; n<3; n++){
							memcpy(&RegPoints[m][n],&bufferRecievefrmRobot[3+(m*3+n)*sizeof(double)],sizeof(double));
						}
					}
					 // If not correct, print usage
					std::cerr<< "Point 1: X " << RegPoints[0][0] <<" Y " << RegPoints[0][1] << "Z" <<RegPoints[0][2] << std::endl;
					std::cerr << "Point 2: X " << RegPoints[1][0] <<" Y " << RegPoints[1][1] << "Z" <<RegPoints[1][2] << std::endl;
					std::cerr << "Point 3: X " << RegPoints[2][0] <<" Y " << RegPoints[2][1] << "Z" <<RegPoints[2][2] << std::endl;
					std::cerr << "Point 4: X " << RegPoints[3][0] <<" Y " << RegPoints[3][1] << "Z" <<RegPoints[3][2] << std::endl;


					SendRegistrationPoints(SlicerSocket, RegPoints, mUID);
					

				}else if(RcvDataType == 'S'){
					
					SendString(SlicerSocket, bufferRecievefrmRobot, mUID);
				}
			}
		}
      }
    
  //------------------------------------------------------------
  // Close connection (The example code never reachs to this section ...)
  
 ProxyserverSocket->CloseSocket();
 RobotSocket->CloseSocket();

}




int ReceiveTransform(igtl::Socket * socket, igtl::MessageHeader * header)
{
  std::cerr << "Receiving TRANSFORM data type." << std::endl;

  std::string NameString;
  // Create a message buffer to receive transform data
  igtl::TransformMessage::Pointer transMsg;
  transMsg = igtl::TransformMessage::New();
  transMsg->SetMessageHeader(header);
  transMsg->AllocatePack();

  mState=3;
  // Receive transform data from the socket
  socket->Receive(transMsg->GetPackBodyPointer(), transMsg->GetPackBodySize());

  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = transMsg->Unpack(1);
  if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
    // Retrive the transform data
    igtl::Matrix4x4 matrix;
    transMsg->GetMatrix(matrix);
    igtl::PrintMatrix(matrix);
	NameString=transMsg->GetDeviceName();


	unsigned int pos =NameString.find("_");
	mUID = NameString.substr(pos+1,NameString.length());

	//Saving the Transform into T_CT_Case
		for( int g=0; g<3; g++){
			for(int h =0; h<4; h++){
				T_CT_Base[h + g*4] = matrix[g][h];
			}
		}
	RecievedTransform=true;

    return 1;
    }

  return 0;

}


#if OpenIGTLink_PROTOCOL_VERSION >= 2
int ReceiveString(igtl::Socket * socket, igtl::MessageHeader * header)
{
  std::string RecvString;
  std::string Command;
  std::string TmpString;
  std::string NameString;
  std::string tmpParam[12];
  int numberofparam=0;
  std::cerr << "Receiving STRING data type." << std::endl;
  // Create a message buffer to receive transform data
  igtl::StringMessage::Pointer stringMsg;
  stringMsg = igtl::StringMessage::New();
  stringMsg->SetMessageHeader(header);
  stringMsg->AllocatePack();

  // Receive transform data from the socket
  socket->Receive(stringMsg->GetPackBodyPointer(), stringMsg->GetPackBodySize());

  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = stringMsg->Unpack(1);
	
  if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
		std::cerr << "Encoding: " << stringMsg->GetEncoding() << "; "
				  << "String: " << stringMsg->GetString() << std::endl;
		RecvString = stringMsg->GetString();
		NameString = stringMsg->GetDeviceName();
		unsigned int pos =NameString.find("_");
		mUID = NameString.substr(pos+1,NameString.length());
		TmpString = RecvString;
		std::cerr<<"Received String: " <<RecvString<<std::endl;
		pos = RecvString.find(";");
		Command = TmpString.substr(0, pos);
		while((pos+1)<TmpString.length()){
			TmpString = TmpString.substr(pos+1);
			std::cerr<<"TmpString: "<<TmpString <<std::endl;
			pos = TmpString.find(";");
			tmpParam[numberofparam] = TmpString.substr(0, pos);
			std::cerr<<"Parameter: "<<tmpParam[numberofparam]<<std::endl;
			numberofparam++;
		}

		if(strcmp(Command.c_str(), "IDLE")==0){
			std::cerr<< "State: "<<mState<<std::endl;
			mState = 0;
			if(numberofparam!=0){
				std::cerr << "To many Parameters recieved - the sent parameters are ignored" << std::endl;
			}

		}else if(strcmp(Command.c_str(), "Registration")==0){
			mState = 1;
			pointstoregister=atoi(tmpParam[0].c_str());
			if(numberofparam>1){
				std::cerr << "To many Parameters recieved - the sent parameters are ignored" << std::endl;
			}
		}else if(strcmp(Command.c_str(), "SendData")==0){
			mState= 2;
			if(numberofparam!=0){
				std::cerr << "To many Parameters recieved - the sent parameters are ignored" << std::endl;
			}

		}else if(strcmp(Command.c_str(), "WaitforTCTBase")==0){
			mState = 3;
			if(numberofparam!=0){
				std::cerr << "To many Parameters recieved - the sent parameters are ignored" << std::endl;
			}

		}else if(strcmp(Command.c_str(), "GravComp")==0){
			mState= 4;
		}else if(strcmp(Command.c_str(), "VirtualFixtures")==0){
			mState= 5;
			if(numberofparam>=6){
				ap[0]=atof(tmpParam[1].c_str());
				ap[1]=atof(tmpParam[2].c_str());
				ap[2]=atof(tmpParam[3].c_str());

				n[0]=atof(tmpParam[4].c_str());
				n[1]=atof(tmpParam[5].c_str());
				n[2]=atof(tmpParam[6].c_str());

				if(strcmp(tmpParam[0].c_str(), "cone")==0){
					phi=atof(tmpParam[7].c_str());
					if(numberofparam>8){
						std::cerr << "To many Parameters recieved - the sent parameters are ignored" << std::endl;
					}
				}else if (strcmp(tmpParam[0].c_str(), "plane")==0){
					if(numberofparam>7){
						std::cerr << "To many Parameters recieved - the sent parameters are ignored" << std::endl;
					}
				}else{
					std::cerr << "Unknown VirtualFixtures Type" << std::endl;
				}
			}else{
				std::cerr << "Not enough Parameters" << std::endl;
			}
		}else if(strcmp(Command.c_str(), "NavGravComp")==0){
			mState= 6;
			if(numberofparam!=0){
				std::cerr << "To many Parameters recieved - the sent parameters are ignored" << std::endl;
			}
		}else if(strcmp(Command.c_str(), "NavGravCompVF")==0){
			mState= 7;
			if(numberofparam>=6){
				ap[0]=atof(tmpParam[1].c_str());
				ap[1]=atof(tmpParam[2].c_str());
				ap[2]=atof(tmpParam[3].c_str());

				n[0]=atof(tmpParam[4].c_str());
				n[1]=atof(tmpParam[5].c_str());
				n[2]=atof(tmpParam[6].c_str());

				if(strcmp(tmpParam[0].c_str(), "cone")==0){
					phi=atof(tmpParam[7].c_str());
					if(numberofparam>8){
						std::cerr << "To many Parameters recieved - the sent parameters are ignored" << std::endl;
					}
				}else if (strcmp(tmpParam[0].c_str(), "plane")==0){
					if(numberofparam>7){
						std::cerr << "To many Parameters recieved - the sent parameters are ignored" << std::endl;
					}
				}else{
					std::cerr << "Unknown VirtualFixtures Type" << std::endl;
				}
			}else{
				std::cerr << "Not enough Parameters" << std::endl;
			}
		}else{
			std::cerr << "Unknown Command String Recieved from Slicer" << std::endl;
		}
		std::cerr<< "State: "<<mState<<std::endl;
  }
  return 1;
}
#endif //OpenIGTLink_PROTOCOL_VERSION >= 2

int SendRegistrationPoints(igtl::Socket * socket, double Points[6][3], std::string mUID){
	std::string NameStr;
	 NameStr= "ACK" +mUID;


	      //---------------------------
	  // Create a point message
	  igtl::PointMessage::Pointer pointMsg;
	  pointMsg = igtl::PointMessage::New();
	  pointMsg->SetDeviceName(NameStr.data());
      
	  //---------------------------
	  // Create 1st point
	  igtl::PointElement::Pointer RegPoint0;
	  RegPoint0 = igtl::PointElement::New();
	  RegPoint0->SetName("POINT_0");
	  RegPoint0->SetGroupName("Registration");
	  RegPoint0->SetRGBA(0x00, 0x00, 0xFF, 0xFF);
	  RegPoint0->SetPosition(Points[0][0], Points[0][1], Points[0][2]);
	  RegPoint0->SetRadius(8.0);
	  RegPoint0->SetOwner("IMAGE_0");
      
	  //---------------------------
	  // Create 2nd point
	  igtl::PointElement::Pointer RegPoint1;
	  RegPoint1 = igtl::PointElement::New();
	  RegPoint1->SetName("POINT_1");
	  RegPoint1->SetGroupName("Registration");
	  RegPoint1->SetRGBA(0x00, 0x00, 0xFF, 0xFF);
	  RegPoint1->SetPosition(Points[1][0], Points[1][1], Points[1][2]);
	  RegPoint1->SetRadius(8.0);
	  RegPoint1->SetOwner("IMAGE_0");
      
	  //---------------------------
	  // Create 3rd point
	  igtl::PointElement::Pointer RegPoint2;
	  RegPoint2 = igtl::PointElement::New();
	  RegPoint2->SetName("POINT_2");
	  RegPoint2->SetGroupName("Registration");
	  RegPoint2->SetRGBA(0x00, 0x00, 0xFF, 0xFF);
	  RegPoint2->SetPosition(Points[2][0], Points[2][1], Points[2][2]);
	  RegPoint2->SetRadius(8.0);
	  RegPoint2->SetOwner("IMAGE_0");
      
	   //---------------------------
	  // Pack into the point message
	  pointMsg->AddPointElement(RegPoint0);
	  pointMsg->AddPointElement(RegPoint1);
	  pointMsg->AddPointElement(RegPoint2);

	  if(pointstoregister>3){
			  //---------------------------
		  // Create 4th point
		  igtl::PointElement::Pointer RegPoint3;
		  RegPoint3 = igtl::PointElement::New();
		  RegPoint3->SetName("POINT_3");
		  RegPoint3->SetGroupName("Registration");
		  RegPoint3->SetRGBA(0x00, 0x00, 0xFF, 0xFF);
		  RegPoint3->SetPosition(Points[3][0], Points[3][1], Points[3][2]);
		  RegPoint3->SetRadius(8.0);
		  RegPoint3->SetOwner("IMAGE_0");
		  pointMsg->AddPointElement(RegPoint3);
      
		  if(pointstoregister>4){
			  //---------------------------
			  // Create 5th point
			  igtl::PointElement::Pointer RegPoint4;
			  RegPoint4 = igtl::PointElement::New();
			  RegPoint4->SetName("POINT_4");
			  RegPoint4->SetGroupName("Registration");
			  RegPoint4->SetRGBA(0x00, 0x00, 0xFF, 0xFF);
			  RegPoint4->SetPosition(Points[4][0], Points[4][1], Points[4][2]);
			  RegPoint4->SetRadius(8.0);
			  RegPoint4->SetOwner("IMAGE_0");
			  pointMsg->AddPointElement(RegPoint4);
      
			  if(pointstoregister>5){
							  //---------------------------
				  // Create 6th point
				  igtl::PointElement::Pointer RegPoint5;
				  RegPoint5 = igtl::PointElement::New();
				  RegPoint5->SetName("POINT_5");
				  RegPoint5->SetGroupName("Registration");
				  RegPoint5->SetRGBA(0x00, 0x00, 0xFF, 0xFF);
				  RegPoint5->SetPosition(Points[5][0], Points[5][1], Points[5][2]);
				  RegPoint5->SetRadius(8.0);
				  RegPoint5->SetOwner("IMAGE_0");
				  pointMsg->AddPointElement(RegPoint5);
      
				  if(pointstoregister>6){
					  std::cerr<<"more than 6 Points are not supported!!"<<std::endl;
				  }
			  }
		  }
	  }
	 
	  pointMsg->Pack();
      
	  //---------------------------
	  // Send
	  int r = socket->Send(pointMsg->GetPackPointer(), pointMsg->GetPackSize());
	  return r;
}

int SendString(igtl::Socket * socket, unsigned char buffer[],std::string mUID){

		igtl::StringMessage::Pointer stringMsg;
		stringMsg = igtl::StringMessage::New();

		std::string tmpStringMsg;
		std::string NameStr;
		switch(mState){
			case 0://IDLE
				tmpStringMsg="IDLE;";
				break;
			case 1://Registration
				if(buffer[2] ==0){
					tmpStringMsg = "Registration;false;";
				}else if(buffer[2] ==1){
					tmpStringMsg = "Registration;true;";
				}
				NameStr = "ACK_" + mUID;
				break;
			case 2://SendData
				tmpStringMsg = "SendData;";
				NameStr = "ACK_" + mUID;
				break;
			case 3://WaitforTCTBase
				//Mirroring State + if the Transform is received
				if(buffer[2] ==0){
					tmpStringMsg = "WaitforTCTBase;false;";
				}else if(buffer[2] ==1){
					tmpStringMsg = "WaitforTCTBase;true;";
				}
				NameStr = "ACK_" + mUID;
				break;
			case 4://GravComp
				tmpStringMsg = "GravComp;";
				NameStr = "ACK_" + mUID;
				break;
			case 5://VirtualFixtures
				//Mirroring State + the type
				if(mVFtype ==0){
					tmpStringMsg = "VirtualFixtures;plane;";
				}else if(mVFtype ==1){
					tmpStringMsg = "VirtualFixtures;cone;";
				}else{
					tmpStringMsg = "VirtualFixtures;";
				}
				NameStr = "ACK_" + mUID;
				break;
			case 6://NavGravComp
				tmpStringMsg = "NavGravComp;0;";
				NameStr = "CMD_" + mUID;
				break;
			case 7://NavGravCompVF
				tmpStringMsg = "NavGravCompVF;";
				NameStr = "ACK_" + mUID;
				break;
			default:
				break;
		}

		stringMsg->SetString(tmpStringMsg);
		stringMsg->SetDeviceName(NameStr.data());
		stringMsg->Pack();
		int r= socket->Send(stringMsg->GetPackPointer(), stringMsg->GetPackSize());
		return r;
}