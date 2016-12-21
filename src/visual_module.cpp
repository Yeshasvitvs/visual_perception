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

#include <visual_module.h>

double visual_perception::VisualModule::getPeriod()
{
    return 0.01;
}

bool visual_perception::VisualModule::updateModule()
{
    //std::cout << "Calling VisualModule update" << std::endl;
    
    if(robotName == "opencv")
    {
        pose->frame_grabber_->cvMatCapture();
        image =pose->frame_grabber_->getCVMat();
    }
    else
    {
        pose->frame_grabber_->yarpFrameCapture();
        image =pose->frame_grabber_->getYarpMat();
    }
    
    pose->detectMarkersAndComputePose(image);
    pose->getTrajectoryInfo();
    if(!image.empty())
    {
        if(!pose->displayImage(image))
        {
        }
        else{
            if(!close())
            {
                stopModule();
            }
        }
    }
}

bool visual_perception::VisualModule::interruptModule()
{
    std::cout << "Interrupting VisualModule for port cleanup" << std::endl;
    return true;
}

bool visual_perception::VisualModule::close()
{
    std::cout << "Calling VisualModule close function" << std::endl;
    handlePort.close();
    return true;
}