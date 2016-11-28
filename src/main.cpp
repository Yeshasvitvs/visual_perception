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
        
        if(option=="opencv" || option=="-opencv" || option=="--opencv" )
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
        //pose.eyePoseCompute();
        //pose.getEyePose();
        //std::cout << "Received eye pose : " << pose.getEyePose().toString() << std::endl; 
    }
    
    //visual_perception::FrameGrabber frame_grabber(option); //Seding the option to constructor
    /*while(1)
    {
        //std::cout << "This is another code " << std::endl;
        frame_grabber.cvMatCapture();
        cv::Mat image =frame_grabber.getCVMat();
        
        //frame_grabber.yarpFrameCapture();
        //cv::Mat image =frame_grabber.getYarpMat();
        
        if(!image.empty())
        {
            std::cout << "Captured image" << std::endl;
            cv::namedWindow("Input Image",CV_WINDOW_AUTOSIZE);
            cv::imshow("Input Image",image);
            cv::waitKey(1);
        }
    }*/
    
    return 0;
}





//yarp::os::BufferedPort<yarp::sig::ImageOf< PixelBgr > > inputFramePort;
    //inputFramePort.open("/visual_perception/frame/in");
    
    //yarp::os::BufferedPort<yarp::sig::ImageOf< PixelBgr > > outputFramePort;
    //outputFramePort.open("/visual_perception/frame/out");
    
    //Getting Eye Pose
    //iCub::iKin::iCubEye left_eye("left_v2"); //iCub Eye Kinematic Chains
    //iCub::iKin::iCubEye right_eye("right_v2");
    
    //left_eye.releaseLink(0);
    //left_eye.releaseLink(1);
    //left_eye.releaseLink(2);
    
    //right_eye.releaseLink(0);
    //right_eye.releaseLink(1);
    //right_eye.releaseLink(2);
    
    //yarp::sig::Vector head_state(6);
    //yarp::os::BufferedPort<yarp::sig::Vector> headStateInputPort;
    //headStateInputPort.open("/visual_perception/head/state:i");
    //yarpNetwork.connect("/icubGazeboSim/head/state:o","/visual_perception/head/state:i");
    
    /*while(1)
    {
        
        yarp::sig::Vector *head_state = headStateInputPort.read();
        //headStateInputPort.read(&head_state);
        std::cout << head_state->toString() << std::endl;
        if(head_state->length()!=0)
        {
            std::cout << "Received head encoder values : " << head_state->toString() << std::endl;
        }
        else std::cout << "Error while reading head encoders!" << std::endl;
    
   
        //3D Pose
        yarp::sig::Vector left_eye_pose = left_eye.EndEffPose(iCub::ctrl::CTRL_DEG2RAD**head_state); 
        yarp::sig::Vector right_eye_pose = right_eye.EndEffPose(CTRL_DEG2RAD**head_state);
    
        if(left_eye_pose.length()!=0)
        {
            std::cout<< "Computed Left eye pose : " << left_eye_pose.toString() << std::endl;
        }   
        if(right_eye_pose.length()!=0)
        {
            std::cout<< "Computed Right eye pose : " << right_eye_pose.toString() << std::endl;
        }
    
    
        
        yarp::sig::ImageOf< PixelBgr > *inFrame = inputFramePort.read();
        if(inFrame!=NULL){
            yarp::sig::ImageOf< PixelBgr > &outFrame = outputFramePort.prepare();
            outFrame = *inFrame;
            outputFramePort.write();
        }
    }*/


//Pure OpenCV 
/*int main(int argc, char** argv )
{
    cv::VideoCapture cap(0);
    cv::Mat inputFrame;
    
    if(!cap.isOpened()){
        
        std::cout << "Cannot open the cameras" << std::endl;
        return 1;
        
    }
    
    while(1){
    
        bool capSuccess = cap.read(inputFrame);
        
        if(!capSuccess){
            std::cout << "Cannot read input frames from the camera stream" << std::cout;
            break; 
        }
        
        cv::namedWindow("Input Frames",CV_WINDOW_AUTOSIZE);
        cv::imshow("Input Frames",inputFrame);
        cv::waitKey(1);
        
    }
    return 0;
}*/

