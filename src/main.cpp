#include "frame_grabber.h"
#include "pose_estimate.h"

using namespace cv;
using namespace std; 
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;



using namespace visual_perception;

static int show_usage_info()
{
    // Pass the argument for how to access the camera
    std::cerr << "Error in arguments passed to the porgram!" << std::endl;
    std::cerr << "Use any one of the following arguments :" <<  " 1)opencv 2) icub 3) icubGazeboSim " << std::endl;
    return 1;
}


int main(int argc,char** argv)
{

    std::string option = argv[1];
    std::cout << "option : " << option << std::endl;
    
    std::string eye = argv[2];
    std::cout << "eye : " << eye << std::endl;
    
    if (argc < 2) 
    {
        show_usage_info();
    }
    else
    {
        
        if(option=="opencv" || option=="-opencv" || option=="--opencv")
        {
            std::cout << "Visual perception run using stand alone camera through opencv" << std::endl;
        }
        else if(option=="icub" || option=="-icub" || option=="--icub")
        {
            std::cout << "Visual perception run using iCub eyes" << std::endl;
        }
        else if(option=="icubSim" || option=="-icubSim" || option=="--icubSim")
        {
            std::cout << "Visual perception run using iCubGazeboSim eyes" <<std::endl;
        }
        else show_usage_info();
    }
    
    visual_perception::PoseEstimation pose(option,eye);
    pose.saveMarkerImages();
    
    while(1)
    {
        pose.frame_grabber_->cvMatCapture();
        cv::Mat image =pose.frame_grabber_->getCVMat();
        pose.detectMarkersAndComputePose(image);
        
        if(!image.empty())
        {
            std::cout << "Captured image" << std::endl;
            cv::namedWindow("Input Image",CV_WINDOW_AUTOSIZE);
            cv::imshow("Input Image",image);
            char key = (char) cv::waitKey(1);
            if (key == 27)
                break;
        }
    }
    
    return 0;
}
