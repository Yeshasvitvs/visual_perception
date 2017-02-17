#include "frame_grabber.h"

visual_perception::FrameGrabber::FrameGrabber(std::string option)
{
    std::cout << "Selected option : " << option << std::endl;
    
    if(option=="opencv" || option=="-opencv" || option=="--opencv" )
    {
        cv_capture_ptr_ = new cv::VideoCapture(0); //Opening camera device
        if(!cv_capture_ptr_->isOpened())
        {
            std::cout << "OpenCV camera device failed to open!" << std::endl;
            
        }
    }
    else 
    {
        //Checking yarp network before creating ports 
        cam_port_name_ = "/" + option  + "/cam/left";  
        if(!yarp::os::Network::initialized())
        {
            yarp::os::Network::init();
        }
        
        input_frame_port_ptr_ = new yarp::os::BufferedPort<yarp::sig::ImageOf< yarp::sig::PixelBgr > >;
        if(input_frame_port_ptr_->open("/visual_perception/frame/in"))
        {
            std::cout << "Opened the port " << input_frame_port_ptr_->getName() << std::endl;
        }
        
        std::string robot; 
        if(option=="icub" || option=="-icub" || option=="--icub")
        {
            robot = "icub";
            cam_port_name_ = "/" + robot  + "/cam/left";
        }
        else 
        { 
            robot = "icubSim";
            cam_port_name_ = "/" + robot  + "/cam/left";
        }
    }
}



void visual_perception::FrameGrabber::cvMatCapture()
{
        bool capSuccess = cv_capture_ptr_->read(input_cv_mat_);
        if(!capSuccess)
        {
            std::cout << "Cannot read images from the camera device!" << std::endl;
        }
        else 
        {
            //std::cout << "Captured image from the camera device" << std::endl;
            //cv::namedWindow("Input Image",CV_WINDOW_AUTOSIZE);
            //cv::imshow("Input Image",inputMat);
            //cv::waitKey(1);
        }
}

void visual_perception::FrameGrabber::yarpFrameCapture()
{
    std::cout << "Running yarp frame capture thread" << std::endl;
    
    if(!yarp::os::Network::checkNetwork())
    {
        std::cout << "Yarp network is not detected!" << std::endl;
    }   
    else 
    {
        bool port_connection = yarp::os::Network::connect(cam_port_name_,input_frame_port_ptr_->getName());
        if(!port_connection)
        {
            std::cout << "Error in connecting port " << cam_port_name_ << \
            " to " << input_frame_port_ptr_->getName() << std::endl;
        }
        else
        {
            std::cout << "Successfully connected port " << cam_port_name_ << \
            " to " << input_frame_port_ptr_->getName() << std::endl;
            
            std::cout << "Capturing frames from icub eye" << std::endl;
            input_yarp_frame_ = input_frame_port_ptr_->read(); //Reading images from icub camera
            IplImage *dummy = (IplImage*)(*input_yarp_frame_).getIplImage();
            input_yarp_mat_ = cv::cvarrToMat(dummy);            
        }
    }
}

visual_perception::FrameGrabber::~FrameGrabber()
{
    cv_capture_ptr_->release();
    delete cv_capture_ptr_;
    
    if(yarp::os::Network::disconnect(cam_port_name_,input_frame_port_ptr_->getName()))
    {
        std::cout << "Successfully disconnected ports " << cam_port_name_ << \
            " and " << input_frame_port_ptr_->getName() << std::endl;
    }
    delete input_frame_port_ptr_;
    delete input_yarp_frame_;
}

