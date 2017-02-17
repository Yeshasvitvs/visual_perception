/*
 * Copyright (c) 2016, <copyright holder> <email>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#ifndef VISUAL_MODULE_H
#define VISUAL_MODULE_H

#include <iostream>
#include <fstream>
#include <iomanip>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include<pose_estimate.h>
#include <boost/shared_ptr.hpp>

namespace visual_perception
{
    class VisualModule:public yarp::os::RFModule
    {
        yarp::os::RpcServer rpc_port;
        int count;
        std::string robotName;
        std::string partName;
        std::string sideName;
        
        cv::Mat image;
        
        visual_perception::PoseEstimation *pose;
    
    public:
        
        double getPeriod();
        bool updateModule();
    
        bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
        {
           
            std::string cmd = command.get(0).asString();
            std::string cmd1 = command.get(1).asString();
            
            std::cout << "Received command : " << cmd << " " << cmd1 << std::endl;
            
            if (cmd == "quit")
                return false;
            else
            {
                if(cmd=="log")
                {
                    //set log data flag
                    if(cmd1=="start")
                    {
                        if(pose->log_data_==true)
                        {
                            reply.addString("Data Logging: [INPROGRESS]");
                        }
                        else
                        {
                             if(command.size() < 3)
                                 reply.addString("In correct arguments! Correct usage : log start filename");
                            else
                            {
                                std::string cmd2 = command.get(2).asString(); //This is the filename
                                cmd2 = pose->data_directory_ + "/" + cmd2;
                                std::cout << "filename : " << cmd2 << std::endl;
                                pose->file_name_.open(cmd2);
                                pose->log_data_ = true;
                                std::string dummy = " Data Logging: [START] to " + cmd2;
                                reply.addString(dummy);
                            }
                        }
                    }
                    if(cmd1=="stop")
                    {
                        if(pose->log_data_==false)
                        {
                            reply.addString("Data Logging: [NOT IN PROGRESS]");
                        }
                        else
                        {
                            pose->log_data_ = false;
                            pose->file_name_.close();
                            reply.addString("Data Logging: [STOP]");
                        }
                    }
                         
                }
            }
            return true;
        }
     

        
        bool configure(yarp::os::ResourceFinder &rf)
        {
            
            count=0;
            if(!yarp::os::Network::initialized())
            {
                yarp::os::Network::init();
            }
            if(rpc_port.open("/visualModule/rpc:i"))
            {
                std::cout << "Opened the port " << rpc_port.getName() << std::endl;
                attach(rpc_port);
            }

            robotName = rf.find("robot").asString();
            partName = rf.find("part").asString();
            sideName = rf.find("side").asString();
            
            std::cout << "Initializing pose estimation object from visual module" << std::endl;
            pose = new visual_perception::PoseEstimation(this->robotName,this->sideName);
            pose->saveMarkerImages();

            return true;
            
        }
        
        bool interruptModule();
        bool close();
};
}

#endif // VISUAL_MODULE_H
