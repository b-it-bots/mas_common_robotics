/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Initial WIN32 port by Hozefa Indorewala, Robotics Equipment Corporation GmbH, www.servicerobotics.eu */
#include "ros/ros.h"
#include <say.h> //include service definition
#include <ConfigFileReader.h>

#include "std_msgs/Bool.h"

#include "ros/network.h"
#include <StringOperations.h>

#include <windows.h>
#include <iostream>
#include <string>
#include <sapi.h>

#define DEFAULT_ROS_MASTER "http://cob3-1-pc1:11311/"
#define DEFAULT_LOCALIP "cob3-1-pc3"

std::string CONFIG_FILE = "config.cfg";
std::string rosMaster="";
std::string localIP="";
ISpVoice * pVoice = NULL;

ros::Publisher pub;

/**
* Method to say something and publish it on ROS info
**/
void say(std::string data)
{
	data = StringOperations::replaceAll(data, "_", " ");

  ROS_INFO("Say: [%s]", data.c_str());
  std::wstring stemp = StringOperations::stdstring2LPW(data);

  bool robot_is_talking = true;
  pub.publish(robot_is_talking);
	
  pVoice->Speak(stemp.c_str(), 0, NULL);
  pVoice->WaitUntilDone(30000); //30sec timeout

  robot_is_talking = false;
  pub.publish(robot_is_talking);
}

bool sayService(brsu_srvs::say::Request  &req,
		brsu_srvs::say::Response &res )
{
	say(req.phrase);
	return TRUE;
}

bool initVoice()
{
    if (FAILED(::CoInitialize(NULL)))
        return FALSE;

    HRESULT hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void **)&pVoice);
    if( FAILED( hr ) )
    {
		return FALSE;
	}
	
	return TRUE;
}

//Initialize data required from file
void readConfigFile()
{
	ConfigFileReader reader;
	reader.setConfigFile(CONFIG_FILE);
	
	if(reader.isFileExisting())
	{
		rosMaster=reader.getROSMaster();
		localIP=reader.getLocalIP();
	}
	else
	{
		std::cout<<"!!!Config file not found, using default values!!!" <<std::endl;
		rosMaster=DEFAULT_ROS_MASTER;
		localIP=DEFAULT_LOCALIP;
	}
}

int main(int argc, char **argv)
{
	SetPriorityClass(GetCurrentProcess(),REALTIME_PRIORITY_CLASS);
	//parse arguments, argv[1] should be config file path
	if(argc > 1)
	{
		CONFIG_FILE = argv[1];
	}
	else
	{
		std::cout<<"*** No config file passed as argument, please use \"SpeechSynthesisROS.exe configfile.cfg\" !!!" << std::endl << "Searching in current folder..." << std::endl;
	}

	readConfigFile();
	
	initVoice();
	pVoice->Speak(L" ", 0, NULL);
	//set master URI
//	ros::master::setURI(rosMaster);
//	std::cout<< "Connect to ROS master: " << rosMaster << std::endl;

   //set ros ip
//	ros::network::setHost(localIP);
//	std::cout<< "local ROS IP: " << localIP << std::endl;

   /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "brsu_speech_synthesis");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::ServiceServer serviceSay = n.advertiseService("say", sayService);

  ROS_INFO("Speech Synthesis started");
  ros::spin();

    pVoice->Release();
    pVoice = NULL;

    ::CoUninitialize();

  return 0;
}
