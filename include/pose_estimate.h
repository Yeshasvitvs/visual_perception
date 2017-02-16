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

#ifndef POSE_ESTIMATE_H
#define POSE_ESTIMATE_H

#include "frame_grabber.h"
#include <string>
#include <unistd.h>
#include <errno.h>
#include <iostream>
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/format.hpp>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

namespace visual_perception
{
    class PoseEstimation
    {
        private:
            
            std::string eye_name_; 
            std::string robot_name_;
            iCub::iKin::iCubEye *eye_chain_;
            yarp::sig::Vector *head_state_;
            yarp::os::BufferedPort<yarp::sig::Vector> *head_state_input_port_;
            
            yarp::sig::Vector eye_pose_;
            
            //ArUco Markers
            int number_of_markers_ = 6;
            int marker_dimension_ = 4;
            cv::aruco::Dictionary marker_dictionary_;
            std::vector<cv::Mat> *marker_images_;
            std::string dir_location_ = "/home/yeshi/projects/visual_perception/markers/";
            
            
            std::vector<int> marker_ids_,sorted_marker_ids;
            std::vector<cv::Vec3d> rvecs, tvecs, sorted_rvecs, sorted_tvecs;
            std::vector<std::vector<cv::Point2f>> marker_corners_, rejected_candidates_;
            cv::aruco::DetectorParameters detection_params_; 
            
            cv::Mat camera_matrix_, dist_coeffs_;
            std::string calibration_file_ = "/home/yeshi/projects/visual_perception/out_camera_data.yml";
            cv::FileStorage fs;
            std::vector<cv::Vec3d> rot_vector_,trans_vector_;
            bool calib_success_;
            boost::posix_time::ptime time_;
            
            
        public:
            
            //Flags to be set by rpc commands
            bool log_data_;
            std::string data_directory_;                        
            std::ofstream file_name_;
            
            bool marker_detect_success_;
            visual_perception::FrameGrabber* frame_grabber_;
            
            //Containers for storing the trajectory information
            //Each observation contains a relative tranformation between two links
            struct Observation{
                boost::posix_time::ptime time;
                std::vector<Eigen::Vector3d> links_rel_transformation;
            };
            boost::shared_ptr<Observation> observation_sptr {new Observation};
            
            //Each Track contains a sequence of observations between two links
            struct Track
            {
                //std::vector<Observation> obs;
                std::vector< boost::shared_ptr<Observation> > obs;
                std::string id;
                bool modified;
            };
                        
            //This is vector of tracks between all the links
            std::vector<Track> tracks;
            
            //Default Constructor 
            PoseEstimation()
            {
                std::cout << "Pose estimation default constructor" << std::endl;
            }
            
            PoseEstimation(std::string,std::string);
            int loadCameraCalibParams();
            void eyePoseCompute();
            yarp::sig::Vector getEyePose(){return eye_pose_;};
            
            cv::Mat getMarkerImage(int);
            void saveMarkerImages();
            
            bool detectMarkersAndComputePose(cv::Mat&);
            void drawMarkers(cv::Mat&); 
            void computeMarkersPose();
            void drawMarkersPose(cv::Mat&);
            std::string timeConversion(const boost::posix_time::ptime&);
                                    
            void extractTrajectory(boost::posix_time::ptime&,std::vector<int>&,std::vector<cv::Vec3d>&,std::vector<cv::Vec3d>&);
            bool getTrajectoryInfo();
            bool logTrajectoryInfo();
            
            bool displayImage(cv::Mat&);
            //Destructor
            ~PoseEstimation();
    };
}

#endif // POSE_ESTIMATE_H
