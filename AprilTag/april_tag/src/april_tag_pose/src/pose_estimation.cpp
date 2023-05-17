#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp_ros/vpROSGrabber.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#ifdef VISP_HAVE_XML2
#include  <visp3/core/vpXmlParserCamera.h>
#endif

int main(int argc, const char** argv){
    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    double tagSize = 0.077;
    float quad_decimate = 0.1;
    std::string intrinsic_file = "/home/yuxuan/Project/Big_Rover/AprilTag/april_tag/src/april_tag_pose/src/camera.xml";
    vpCameraParameters cam;
    cam.init();
    vpXmlParserCamera parser;
    parser.parse(cam,intrinsic_file,"",
    vpCameraParameters::perspectiveProjWithDistortion);


    try{
        vpImage<unsigned char> I;
        vpROSGrabber g;
        g.setImageTopic("/camera/image_raw");
        g.open(I);
        #ifdef VISP_HAVE_X11
            vpDisplayX d(I);    
        #else
            std::cout<<"No image viewer is available\n";
        #endif
        
        vpDetectorBase *detector = new vpDetectorAprilTag(tagFamily);
        dynamic_cast<vpDetectorAprilTag *>(detector)->setAprilTagQuadDecimate(quad_decimate);
        dynamic_cast<vpDetectorAprilTag *>(detector)->setAprilTagPoseEstimationMethod(poseEstimationMethod);
        dynamic_cast<vpDetectorAprilTag *>(detector)->setAprilTagNbThreads(1);
        dynamic_cast<vpDetectorAprilTag *>(detector)->setDisplayTag(true);
    
        std::vector<double> time_vec;
        double t0 = vpTime::measureTimeMs();
        for(;;){
            g.acquire(I);
            vpDisplay::display(I);
            double t = vpTime::measureTimeMs();
            std::vector<vpHomogeneousMatrix> cMo_vec;
            vpTranslationVector trans;
            vpRzyzVector eular_angle; 
            dynamic_cast<vpDetectorAprilTag *>(detector)->detect(I, tagSize, cam, cMo_vec);
            time_vec.push_back(t-t0);
            vpDisplay::displayText(I,20,20,"Click to quit...",vpColor::red);
            for(size_t i=0;i<cMo_vec.size();i++){
                vpDisplay::displayFrame(I,cMo_vec[i],cam,tagSize/2,vpColor::none,3);
                trans = cMo_vec[i].getTranslationVector();
                eular_angle.buildFrom(cMo_vec[i].getRotationMatrix());
            }
            std::cout<<"T"<<trans<<std::endl;
            std::cout<<"Angle"<<eular_angle<<std::endl;
            vpDisplay::flush(I);
            if(vpDisplay::getClick(I,false)){
                break;
            }
        }
        }catch(vpException e){
            std::cout << "Catch an exception:"<< e <<std::endl;
        }
}