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
 *  This modules takes input argument of which camera device to  access, either a stand alone camera using opencv or 
 * eyes on the icub robot or icub gazebo sim
 * 
 * The output of this module gives images in cv::Mat format for doing further data processing using only opencv
 * 
 */

#ifndef FRAME_GRABBER_H
#define FRAME_GRABBER_H

#include "utils.h"

namespace visual_perception
{  
    class FrameGrabber
    {
        private:
            cv::VideoCapture* cv_capture_ptr_;
            cv::Mat input_cv_mat_; 
            
            yarp::os::BufferedPort<yarp::sig::ImageOf< PixelBgr > > *input_frame_port_ptr_;
            yarp::sig::ImageOf< PixelBgr > *input_yarp_frame_;
            cv::Mat input_yarp_mat_;
            std::string cam_port_name_;
            
        public:
            //Default Constructor
            FrameGrabber()
            {
                std::cout << "Frame grabber default Constructor" << std::endl;
            };
            
            //Constructor 
            FrameGrabber(std::string);
            
            //Capturing Images using cvCapture
            void cvMatCapture();
            
            void yarpFrameCapture(); 
            
            cv::Mat getCVMat(){return input_cv_mat_;};
            
            cv::Mat getYarpMat(){return input_yarp_mat_;};
            
            yarp::sig::ImageOf< PixelBgr > getYarpImg(){return *input_yarp_frame_;};
            
            //Destructor 
            ~FrameGrabber();
    };

}
#endif // FRAME_GRABBER_H
