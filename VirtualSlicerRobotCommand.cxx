/*=========================================================================

  Program:   Open IGT Link -- Example for Data Receiving Client Program
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
#include <iomanip>
#include <math.h>
#include <cstdlib>
#include <cstring>
#include <windows.h>

#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlTransformMessage.h"
#include "igtlPositionMessage.h"
#include "igtlImageMessage.h"
#include "igtlClientSocket.h"
#include "igtlStatusMessage.h"

#if OpenIGTLink_PROTOCOL_VERSION >= 2
#include "igtlPointMessage.h"
#include "igtlStringMessage.h"
#include "igtlTrackingDataMessage.h"
#include "igtlQuaternionTrackingDataMessage.h"
#endif // OpenIGTLink_PROTOCOL_VERSION >= 2

int ReceiveTransform(igtl::Socket * socket, igtl::MessageHeader::Pointer& header);

#if OpenIGTLink_PROTOCOL_VERSION >= 2
  int ReceivePoint(igtl::Socket * socket, igtl::MessageHeader::Pointer& header);
  int ReceiveString(igtl::Socket * socket, igtl::MessageHeader::Pointer& header);
#endif //OpenIGTLink_PROTOCOL_VERSION >= 2


int SendString(igtl::Socket * socket );
int SendTransform(igtl::Socket * socket );
int mState=0;
int mOldState=0;
int mRcvState=0;

int RegistrationFinished=0;
int RegPointsReceived=0;
int TCTBase_send=0;


//Initialize the Variables and set default Values
double T_CT_Base[12] = {1.0, 0.0, 0.0, 32.0, 0.0, 1.0, 0.0, 23.0, 0.0, 0.0, 1.0, 33.0};
int pointstoregister = 4;
double ap[3] = {0.0, 0.0, 100.0};
double n[3] = { 0.0, 0.0, 1.0};
double phi = 90;
int mVFtype= 0;

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
int main(int argc, char* argv[])
{
  //------------------------------------------------------------
  // Parse Arguments

  if (argc != 3) // check number of arguments
    {
    // If not correct, print usage
    std::cerr << "    <hostname> : IP or host name"                    << std::endl;
    std::cerr << "    <port>     : Port # (18944 in Slicer default)"   << std::endl;
    exit(0);
    }

  char*  hostname = argv[1];
  int    port     = atoi(argv[2]);

  

  //------------------------------------------------------------
  // Establish Connection

  igtl::ClientSocket::Pointer RobotProxyClient;
  RobotProxyClient = igtl::ClientSocket::New();
  int r = RobotProxyClient->ConnectToServer(hostname, port);

  if (r != 0)
    {
    std::cerr << "Cannot connect to the server." << std::endl;
    exit(0);
    }

  //------------------------------------------------------------
  // Create a message buffer to receive header
  igtl::MessageHeader::Pointer headerMsg;
  headerMsg = igtl::MessageHeader::New();
  
  //------------------------------------------------------------
  // Allocate a time stamp 
  igtl::TimeStamp::Pointer ts;
  ts = igtl::TimeStamp::New();


  while (1)
    {      
		//if(GetKeyState(0x4E)){//Check for left Mouse-> when it was pressed read new State
			 printf("\nSelect State Current State is %d ): ", mState);
			mState = (ReadInt('0') );
			printf("\n");

		//}
		if(mState!=3){//if State is anything but WaitforTCTBase send State
			SendString(RobotProxyClient);
		}else if(mState=3){
			SendTransform(RobotProxyClient);
		}else{
			mState = mOldState;
			std::cerr<<"Unknown State Request!! State reseted to"<< mState<<std::endl;
		}

		 
      // Initialize receive buffer
      headerMsg->InitPack();
      
      // Receive generic header from the socket
      int r = RobotProxyClient->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize());
      if (r == 0)
        {
        RobotProxyClient->CloseSocket();
        exit(0);
        }
      if (r != headerMsg->GetPackSize())
        {
        continue;
        }
      
      // Deserialize the header
      headerMsg->Unpack();

      // Get time stamp
      igtlUint32 sec;
      igtlUint32 nanosec;

      headerMsg->GetTimeStamp(ts);
      ts->GetTimeStamp(&sec, &nanosec);

      std::cerr << "Time stamp: "
                << sec << "." << std::setw(9) << std::setfill('0') 
                << nanosec << std::endl;
      
      // Check data type and receive data body
      if (strcmp(headerMsg->GetDeviceType(), "TRANSFORM") == 0)
        {
        ReceiveTransform(RobotProxyClient, headerMsg);
        }
#if OpenIGTLink_PROTOCOL_VERSION >= 2
      else if (strcmp(headerMsg->GetDeviceType(), "POINT") == 0)
        {
        ReceivePoint(RobotProxyClient, headerMsg);
        }
      else if (strcmp(headerMsg->GetDeviceType(), "STRING") == 0)
        {
        ReceiveString(RobotProxyClient, headerMsg);
        }
#endif //OpenIGTLink_PROTOCOL_VERSION >= 2
      else
        {
        std::cerr << "Receiving unsupported Message type: " << headerMsg->GetDeviceType() << std::endl;
        RobotProxyClient->Skip(headerMsg->GetBodySizeToRead(), 0);
        }
	  if(mState==mRcvState){
		 mOldState = mState;
	  }else{
		  std::cerr<< "Couldn't Change State from " << mOldState <<" to "<< mState<<"!!"<<std::endl;
		  mState = mOldState;

	  }
    }

  //------------------------------------------------------------
  // Close connection (The example code never reaches this section ...)
  
  RobotProxyClient->CloseSocket();

}


int ReceiveTransform(igtl::Socket * socket, igtl::MessageHeader::Pointer& header)
{
  mRcvState = 6; // State is NavGravComp
  std::cerr << "Receiving TRANSFORM data type." << std::endl;
  
  // Create a message buffer to receive transform data
  igtl::TransformMessage::Pointer transMsg;
  transMsg = igtl::TransformMessage::New();
  transMsg->SetMessageHeader(header);
  transMsg->AllocatePack();
  
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
    std::cerr << std::endl;
    return 1;
    }

  return 0;
}


#if OpenIGTLink_PROTOCOL_VERSION >= 2
int ReceivePoint(igtl::Socket * socket, igtl::MessageHeader::Pointer& header)
{
  mRcvState = 2;//State DataSend because the RegPoints are send
  std::cerr << "Receiving POINT data type." << std::endl;

  // Create a message buffer to receive transform data
  igtl::PointMessage::Pointer pointMsg;
  pointMsg = igtl::PointMessage::New();
  pointMsg->SetMessageHeader(header);
  pointMsg->AllocatePack();

  // Receive transform data from the socket
  socket->Receive(pointMsg->GetPackBodyPointer(), pointMsg->GetPackBodySize());

  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = pointMsg->Unpack(1);

  if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
    int nElements = pointMsg->GetNumberOfPointElement();
    for (int i = 0; i < nElements; i ++)
      {
      igtl::PointElement::Pointer pointElement;
      pointMsg->GetPointElement(i, pointElement);

      igtlUint8 rgba[4];
      pointElement->GetRGBA(rgba);

      igtlFloat32 pos[3];
      pointElement->GetPosition(pos);

      std::cerr << "========== Element #" << i << " ==========" << std::endl;
      std::cerr << " Name      : " << pointElement->GetName() << std::endl;
      std::cerr << " GroupName : " << pointElement->GetGroupName() << std::endl;
      std::cerr << " RGBA      : ( " << (int)rgba[0] << ", " << (int)rgba[1] << ", " << (int)rgba[2] << ", " << (int)rgba[3] << " )" << std::endl;
      std::cerr << " Position  : ( " << std::fixed << pos[0] << ", " << pos[1] << ", " << pos[2] << " )" << std::endl;
      std::cerr << " Radius    : " << std::fixed << pointElement->GetRadius() << std::endl;
      std::cerr << " Owner     : " << pointElement->GetOwner() << std::endl;
      std::cerr << "================================" << std::endl << std::endl;
      }
    }

  return 1;
}

int ReceiveString(igtl::Socket * socket, igtl::MessageHeader::Pointer& header)
{
   std::string RecvString;
  std::string Command;
  std::string TmpString;
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
		RecvString = stringMsg->GetString();
		TmpString = RecvString;
		std::cerr<<"Received String: " <<RecvString<<std::endl;
		unsigned int pos = RecvString.find(";");
		Command = TmpString.substr(0, pos);
		while((pos+1)<TmpString.length()){
			TmpString = TmpString.substr(pos+1);
			pos = TmpString.find(";");
			tmpParam[numberofparam] = TmpString.substr(0, pos);
			numberofparam++;
		}

		if(strcmp(Command.c_str(), "IDLE")==0){
			mRcvState = 0;
			if(numberofparam!=0){
				std::cerr << "To many Parameters recieved - the sent parameters are ignored" << std::endl;
			}

		}else if(strcmp(Command.c_str(), "Registration")==0){
			mRcvState = 1;
			RegistrationFinished=atoi(tmpParam[0].c_str());
			if(numberofparam>1){
				std::cerr << "To many Parameters recieved - the sent parameters are ignored" << std::endl;
			}
		}else if(strcmp(Command.c_str(), "SendData")==0){
			mRcvState= 2;
			if(numberofparam!=0){
				std::cerr << "To many Parameters recieved - the sent parameters are ignored" << std::endl;
			}

		}else if(strcmp(Command.c_str(), "WaitforTCTBase")==0){
			mRcvState = 3;
			TCTBase_send=atoi(tmpParam[0].c_str());
			if(numberofparam>1){
				std::cerr << "To many Parameters recieved - the sent parameters are ignored" << std::endl;
			}

		}else if(strcmp(Command.c_str(), "GravComp")==0){
			mRcvState= 4;
		}else if(strcmp(Command.c_str(), "VirtualFixtures")==0){
			mRcvState= 5;
			if(strcmp(tmpParam[0].c_str(), "cone")==0){
				mVFtype=1;
				if(numberofparam>1){
					std::cerr << "To many Parameters recieved - the sent parameters are ignored" << std::endl;
				}
			}else if (strcmp(tmpParam[0].c_str(), "plane")==0){
				mVFtype=0;
				if(numberofparam>1){
					std::cerr << "To many Parameters recieved - the sent parameters are ignored" << std::endl;
				}
			}else{
				std::cerr << "Unknown VirtualFixtures Type" << std::endl;
			}
		}else if(strcmp(Command.c_str(), "NavGravComp")==0){
			mRcvState= 0;
			std::cerr << "Recieved Data Should be Transform not State" << std::endl;
	
		}else if(strcmp(Command.c_str(), "NavGravCompVF")==0){
			mRcvState= 0;
			std::cerr << "Recieved Data Should be Transform not State" << std::endl;
			}else{
				std::cerr << "Not enough Parameters" << std::endl;
			}
		}else{
			std::cerr << "Unknown Command String Recieved from Slicer" << std::endl;
		}
  return 1;
}

#endif 
int SendTransform(igtl::Socket * socket ){

		 // Create a message buffer to receive header
      igtl::MessageHeader::Pointer header;
      header = igtl::MessageHeader::New();
	  header->InitPack();

	// Create a message buffer to receive transform data
	  igtl::TransformMessage::Pointer transMsg;
	  transMsg = igtl::TransformMessage::New();
	  transMsg->SetMessageHeader(header);
	  transMsg->AllocatePack();
	
	
	//------------------------------------------------------------
	// Allocate a time stamp 
	igtl::TimeStamp::Pointer ts;
	ts = igtl::TimeStamp::New();

	igtl::Matrix4x4 matrix;
	for(int j=0; j<3; j++){
		for(int i=0; i<4; i++){
			matrix[j][i] = T_CT_Base[i+j*4];
		}
	}
    ts->GetTime();
    transMsg->SetMatrix(matrix);
    transMsg->SetTimeStamp(ts);
    transMsg->Pack();
    socket->Send(transMsg->GetPackPointer(), transMsg->GetPackSize());

	return 1;

}

int SendString(igtl::Socket * socket ){
 // Create a message buffer to receive header
	igtl::MessageHeader::Pointer header;
	header = igtl::MessageHeader::New();
	header->InitPack();

	igtl::StringMessage::Pointer stringMsg;
	stringMsg = igtl::StringMessage::New();
	stringMsg->SetMessageHeader(header);
	stringMsg->AllocatePack();

	std::ostringstream tmpStream;
	std::string tmpStringMsg;

	std::cerr<<"State: "<<mState<<std::endl;
	switch(mState){
		case 0://IDLE
			tmpStringMsg="IDLE;";
			break;
		case 1://Registration
			tmpStream << pointstoregister<<";";
			tmpStringMsg = "Registration;" ;
			tmpStringMsg += tmpStream.str();
			break;
		case 2://SendData
			tmpStringMsg = "SendData;";
			break;
		case 3://WaitforTCTBase
			//Mirroring State + if the Transform is received
			tmpStringMsg = "WaitforTCTBase;";
			break;
		case 4://GravComp
			tmpStringMsg = "GravComp;";
			break;
		case 5://VirtualFixtures
			//Mirroring State + the type
			if(mVFtype ==0){
				tmpStringMsg = "VirtualFixtures;plane;";
				tmpStream << ap[0]<<";"<<ap[1]<<";"<<ap[2]<<";"<<n[0]<<";"<<n[1]<<";"<<n[2]<<";";
			}else if(mVFtype ==1){
				tmpStringMsg = "VirtualFixtures;cone;";
				tmpStream << ap[0]<<";"<<ap[1]<<";"<<ap[2]<<";"<<n[0]<<";"<<n[1]<<";"<<n[2]<<";"<<phi<<";";
			}
			tmpStringMsg += tmpStream.str();
			break;
		case 6://NavGravComp
			tmpStringMsg = "NavGravComp;";
			break;
		case 7://NavGravCompVF
			tmpStringMsg = "NavGravCompVF;";
			break;
		default:
			break;
	}

	stringMsg->SetString(tmpStringMsg);
	stringMsg->Pack();
	socket->Send(stringMsg->GetPackPointer(), stringMsg->GetPackSize());

	return 1;
}