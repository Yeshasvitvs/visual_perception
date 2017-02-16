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

bool visual_perception::PoseEstimation::displayImage(Mat& image)
{
    //std::cout << "Captured image" << std::endl;
    cv::namedWindow("Input Image",CV_WINDOW_AUTOSIZE);
    cv::imshow("Input Image",image);
    char key = (char) cv::waitKey(1);
    if (key == 27)
    {
        std::cout << "Esc key pressed!" << std::endl;
        return 1;
    }
    else
    {
        return 0;
    }   
}


visual_perception::PoseEstimation::PoseEstimation(std::string robot,std::string eye)
{
    char cwd[1024];
    if(getcwd(cwd,sizeof(cwd)) != NULL)
    {
        data_directory_ = cwd;
        data_directory_.replace(data_directory_.find("build"),5,"data");
        std::cout << "Current data directory : " << data_directory_ << std::endl;
    }
    else std::cerr << "getcwd() error!" << std::endl;
    
    log_data_ = false;
    //Initialize the FrameGrabber 
    frame_grabber_ = new FrameGrabber(robot);
    loadCameraCalibParams();
    
    if(robot!="opencv")
    {
        //Initialization for eye pose estimation
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
    
    //Initialization for ArUco markers 
    std::cout << "ArUCo Markers generation" << std::endl;
    marker_dictionary_ = cv::aruco::generateCustomDictionary(number_of_markers_,marker_dimension_);
    marker_images_ = new std::vector<cv::Mat>(number_of_markers_);
    for(int i = 0; i < number_of_markers_; i++)
    {
        cv::aruco::drawMarker(marker_dictionary_,i,200,marker_images_->at(i),1);
    }
}

int visual_perception::PoseEstimation::loadCameraCalibParams()
{
    fs.open(calibration_file_,cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix_;
    fs["distortion_coefficients"] >> dist_coeffs_;
    fs.release();
    
    if(camera_matrix_.empty() || dist_coeffs_.empty())
    {
        std::cout << "Error in loading camera calibration parameters!" << std::endl;
        calib_success_ = 0;
    }
    else
    {
        std::cout << "Camera calibration parameters loaded Successfully" << std::endl;
        std::cout << "Camera Matrix : " << camera_matrix_ << std::endl;
        std::cout << "Distortion Coefficients : " << dist_coeffs_ << std::endl;
        calib_success_ = 1;
    }
}

Mat visual_perception::PoseEstimation::getMarkerImage(int id)
{
    return marker_images_->at(id);
}

void visual_perception::PoseEstimation::saveMarkerImages()
{
    for(int i = 0; i < number_of_markers_; i++)
    {
        cv::Mat dummy = getMarkerImage(i);
        
        std::string file_name = boost::lexical_cast<std::string>(i) + ".jpg";
        std::cout << "File Name : " << file_name << std::endl;
        std::string location = dir_location_ + file_name;
        std::cout << "location" << location << std::endl;
        while(!cv::imwrite(location,dummy))
        {
            continue;
        }
    }
}

bool visual_perception::PoseEstimation::detectMarkersAndComputePose(cv::Mat& image)
{
    cv::aruco::detectMarkers(image,marker_dictionary_,marker_corners_,marker_ids_,detection_params_,rejected_candidates_);
    if(!marker_ids_.size() > 0)
    {
        //std::cout << "No markers detected!" << std::endl;
        marker_detect_success_=0;
    }
    else 
    {
        //std::cout << "Markers detected" << std::endl;
        marker_detect_success_=1;
        //Detects all the identifiable markers
        cv::aruco::drawDetectedMarkers(image,marker_corners_,marker_ids_);
        if(calib_success_ == 1)
        {
            cv::aruco::estimatePoseSingleMarkers(marker_corners_,0.05,camera_matrix_,dist_coeffs_,rvecs,tvecs);
            time_ = boost::posix_time::microsec_clock::local_time();
            //std::cout << "Rot : " << rvecs << std::endl;
            //std::cout << "Trans : " << tvecs << std::endl; 
            for(int i=0; i < marker_ids_.size(); i++)
            {
                //Pose values of each marker
                cv::aruco::drawAxis(image,camera_matrix_,dist_coeffs_,rvecs[i],tvecs[i],0.1);
                extractTrajectory(time_,marker_ids_,rvecs,tvecs);
            }
        }
        else
        {
            std::cout << "Cannot compute the pose of detected Markers, Error in loading camera calibration parameters!" << std::endl;
        }
    }
}
std::string visual_perception::PoseEstimation::timeConversion(const boost::posix_time::ptime& time)
{
    const boost::wformat f = boost::wformat(L"%02d.%02d.%s  %02d:%02d.%02d.%02d")
                % time.date().year_month_day().day.as_number()
                % time.date().year_month_day().month.as_number()
                % time.date().year_month_day().year
                % time.time_of_day().hours()
                % time.time_of_day().minutes()
                % time.time_of_day().seconds()
                % time.time_of_day().total_milliseconds();
                
                const std::wstring result = f.str();  // "21.06.2013  14:38"
                const std::string t(result.begin(),result.end());
                return t;

}

void visual_perception::PoseEstimation::extractTrajectory(boost::posix_time::ptime& time,std::vector<int>& marker_ids_,::vector<cv::Vec3d>& rvecs,std::vector<cv::Vec3d>& tvecs)
{
    std::cout << "Extracting trajectory" << std::endl;
    
    int dummy_track_length = marker_ids_.size()-1;
    if(dummy_track_length!=0)
    {
        std::cout << "Detected more than one marker" << std::endl;
        //std::cout << "Track length : " << dummy_track_length << std::endl;
        
         //Track structure array variables
        boost::shared_ptr<Track> track_sptr {new Track[dummy_track_length],std::default_delete<Track[]>() };
        tracks.resize(dummy_track_length);
    
        for(int i=1; i < marker_ids_.size(); i++)
        { 
            
            //Sorting the marker ids
            if( !std::is_sorted(marker_ids_.begin(),marker_ids_.end()) )
            {
                //std::cout << "Actual Marker ids : " << marker_ids_ << "--->";
                //std::cout << rvecs << " , " << tvecs;
                
                sorted_rvecs.resize(rvecs.size());
                sorted_tvecs.resize(tvecs.size());
                
                //SORT the order of marker ids
                sorted_marker_ids=marker_ids_;
                std::sort(sorted_marker_ids.begin(),sorted_marker_ids.end());
                //std::cout << "Sorted Marker ids : " << sorted_marker_ids << "--->";
     
                for(int s=0; s < sorted_marker_ids.size(); s++)
                {
                    for(int p=0; p < marker_ids_.size(); p++)
                    {
                        if(sorted_marker_ids.at(s)==marker_ids_.at(p))
                        {
                            sorted_rvecs.at(s) = rvecs.at(p);
                            sorted_tvecs.at(s) = tvecs.at(p);
                        }
                    }
                }
                
                //std::cout << sorted_rvecs << " , " << sorted_tvecs;
                marker_ids_ = sorted_marker_ids;
                rvecs = sorted_rvecs;
                tvecs = sorted_tvecs;
                
                //Clearing the sorted vectors
                sorted_rvecs.clear();
                sorted_tvecs.clear();
                sorted_marker_ids.clear();
            }
            
            
            //Clear if only one time instance of observation is            
            observation_sptr->time = time_;
            observation_sptr->links_rel_transformation.clear();

            
            //TODO Proper relative transformation implemented - Double check this
             //Translation vectors       
            Eigen::Vector3d P1,P2;
            cv::cv2eigen(tvecs[i-1],P1);
            cv::cv2eigen(tvecs[i],P2);
            
            //Rotational vectors in angels
            Eigen::Vector3d ang1,ang2;
            cv::cv2eigen(rvecs[i-1],ang1);
            cv::cv2eigen(rvecs[i],ang2);
            
            //This is differecen in orientation in RYP 
            //TODO Check this convention for ArUco markers
            Eigen::Matrix3f R1, R2;
            R1 = Eigen::AngleAxisf(ang1[0], Eigen::Vector3f::UnitZ())
                   * Eigen::AngleAxisf(ang1[1], Eigen::Vector3f::UnitY())
                   * Eigen::AngleAxisf(ang1[2], Eigen::Vector3f::UnitX());
           
            R2 = Eigen::AngleAxisf(ang2[0], Eigen::Vector3f::UnitZ())
                   * Eigen::AngleAxisf(ang2[1], Eigen::Vector3f::UnitY())
                   * Eigen::AngleAxisf(ang2[2], Eigen::Vector3f::UnitX());
           
            //Affine 3d Transformations
            Eigen::Affine3d T1,T2;
            T1.setIdentity();
            T1.translation() = P1;
            T1.rotate(Eigen::AngleAxisd(M_PI,ang1));
            
            T2.setIdentity();
            T2.translation() = P2;
            T2.rotate(Eigen::AngleAxisd(M_PI,ang2));
            
            //Computing relative transformation between marker poses
            Eigen::Affine3d rel_trans;
            rel_trans = T1.inverse()*T2;
            
            Eigen::Vector3d trans = rel_trans.translation();
            Eigen::Matrix3d rot = rel_trans.linear();
            Eigen::Vector3d euler_angels = rot.eulerAngles(0,1,2);
            
            observation_sptr->links_rel_transformation.push_back(trans);
            observation_sptr->links_rel_transformation.push_back(euler_angels);
            
        
            track_sptr.get()[i-1].obs.push_back(observation_sptr);
            std::string dummy_id = std::to_string(marker_ids_.at(i-1)) + std::to_string(marker_ids_.at(i));
            track_sptr.get()[i-1].id = dummy_id;
            //std::cout << "Track ID " << TRACK[i-1].id << std::endl;
            
            track_sptr.get()[i-1].modified = true;
        
            //This is of track length
            tracks.at(i-1) = track_sptr.get()[i-1];
            
    
            getTrajectoryInfo();
            
        }
    }
    else
    {
        std::cout << "Detected only single marker!!!" << std::endl;
        for(int i=0; i < tracks.size(); i++)
        {
            tracks.at(i).modified = false;
        }
    }
    
}

bool visual_perception::PoseEstimation::getTrajectoryInfo()
{
 
    //std::cout << "Getting trajectory information" << std::endl;
    //This should call extractTrajectory and display info
    if(marker_ids_.size()!=0)
    {        
        //This should be displayed only if new observations are added
        //std::cout << "track size : " << tracks.size() << std::endl; 
        for(int i=0; i < tracks.size(); i++)
        {
            if(tracks.at(i).modified==true)
            {
                for(int o = 0; o < tracks.at(i).obs.size(); o++)
                {
                    std::string t = timeConversion(tracks.at(i).obs.at(o)->time);
                    if(log_data_!=true)
                    {
                        std::cout << t << " , " << std::endl;
                        std::cout << "Track ID : " << tracks.at(i).id << " ---> " ;
                        Eigen::Vector3d dummy = tracks.at(i).obs.at(o)->links_rel_transformation.at(0);
                        std::cout << dummy[0] << " " << dummy[1] << " " << dummy[2] << " ";
                        dummy = tracks.at(i).obs.at(o)->links_rel_transformation.at(1);
                        std::cout << dummy[0] << " " << dummy[1] << " " << dummy[2] << " " << std::endl;
                        //std::cout << tracks.at(i).obs.at(o)->links_rel_transformation << std::endl;
                    }
                    else
                    {
                        if(file_name_.is_open())
                        {
                            std::cout << "file is open" << std::endl;
                            file_name_ << t << " ";
                            file_name_ << tracks.at(i).id << " ";
                            Eigen::Vector3d dummy = tracks.at(i).obs.at(o)->links_rel_transformation.at(0);
                            file_name_ << dummy[0] << " " << dummy[1] << " " << dummy[2] << " ";
                            dummy = tracks.at(i).obs.at(o)->links_rel_transformation.at(1);
                            file_name_ << dummy[0] << " " << dummy[1] << " " << dummy[2] << " " << std::endl;
                            //file_name_ << tracks.at(i).obs.at(o)->links_rel_transformation << std::endl;
                        }
                        else std::cerr << "file open error" << std::endl;
                    }
                }
            }
        }   
    
        
    }
    return true;
}

bool visual_perception::PoseEstimation::logTrajectoryInfo()
{
    //This should call extractTrajectory and log data
    std::cout << "Logging data" << std::endl;
    return true;
  
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

void visual_perception::PoseEstimation::drawMarkers(cv::Mat& image)
{
    cv::aruco::drawDetectedMarkers(image,marker_corners_,marker_ids_);
}

void visual_perception::PoseEstimation::computeMarkersPose()
{
    cv::aruco::estimatePoseSingleMarkers(marker_corners_,0.05,camera_matrix_,dist_coeffs_,rot_vector_,trans_vector_);
}

void visual_perception::PoseEstimation::drawMarkersPose(cv::Mat& image)
{
    for(int i=0; i < marker_ids_.size(); i++)
    {
        cv::aruco::drawAxis(image,camera_matrix_,dist_coeffs_,rot_vector_[i],trans_vector_[i],0.1);
    }
}

visual_perception::PoseEstimation::~PoseEstimation()
{
    delete frame_grabber_;
    delete marker_images_;
    delete eye_chain_;
    delete head_state_;
    delete head_state_input_port_;
}

