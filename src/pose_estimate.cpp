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

#include "pose_estimate.h"

visual_perception::PoseEstimation::PoseEstimation(std::string robot,std::string eye)
{
    //Initialize the FrameGrabber 
    frame_grabber_ = new FrameGrabber(robot);
    robot_name_ = robot;
    eye_name_ = eye+"_v2";
    std::cout << "Selected eye is " << eye_name_ << std::endl;
    
    eye_chain_ = new iCub::iKin::iCubEye(eye_name_); 
    
    //Releasing trunk links 
    eye_chain_->releaseLink(0);
    eye_chain_->releaseLink(1);
    eye_chain_->releaseLink(2);
    
    if(!yarp::os::Network::initialized())
    {
        yarp::os::Network::init();    
    }
    head_state_input_port_ = new yarp::os::BufferedPort<yarp::sig::Vector>;
    if(head_state_input_port_->open("/visual_perception/head/state:i"))
    {
        std::cout << "Opened the port " << head_state_input_port_->getName() << std::endl;
    }
}

void visual_perception::PoseEstimation::eyePoseCompute()
{
    if(!yarp::os::Network::checkNetwork())
    {
        std::cout << "Yarp network is not detected!" << std::endl;
    }
    else
    {   
        std::string head_port_name = "/"+robot_name_+"/head/state:o";
        bool port_connection = yarp::os::Network::connect(head_port_name,head_state_input_port_->getName());
        if(!port_connection)
        {
            std::cout << "Error in connecting port " << head_port_name << \
            " to " << head_state_input_port_ << std::endl;
        }
        else 
        {   
            std::cout << "Successfully connected port " << head_port_name << \
            " to " << head_state_input_port_->getName() << std::endl;
        }
    }
    head_state_ = head_state_input_port_->read();
    if(head_state_->length()!=0)
    {
        std::cout << "Received head state encoder values : " << head_state_->toString() << std::endl;
        eye_pose_ = eye_chain_->EndEffPose(iCub::ctrl::CTRL_DEG2RAD**head_state_);
        if(eye_pose_.length()!=0)
        {
            std::cout << "Computed eye pose : " << eye_pose_.toString() << std::endl;
            
        } 
        
    }
    else std::cout << "Error while reading head encoder values!" << std::endl;     
}

visual_perception::PoseEstimation::~PoseEstimation()
{
    //delete frame_grabber_;
    delete eye_chain_;
    delete head_state_;
    delete head_state_input_port_;
}

