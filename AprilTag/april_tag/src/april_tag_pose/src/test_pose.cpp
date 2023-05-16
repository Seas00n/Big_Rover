#include <iostream>

#include "depthai/depthai.hpp"

#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp_ros/vpROSGrabber.h>

int main(int argc,  const char **argv){
    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    vpCameraParameters cam;
    double tagSize = 0.053;
    float quad_decimate = 1.00;
    bool display_tag = true;
    cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, 312.1889954, 243.4373779);
    std::cout<<"cam:\n"<<cam<<std::endl;
    vpDetectorBase *detector = new vpDetectorAprilTag(tagFamily);
    dynamic_cast<vpDetectorAprilTag *>(detector)->setAprilTagQuadDecimate(quad_decimate);
    dynamic_cast<vpDetectorAprilTag *>(detector)->setAprilTagPoseEstimationMethod(poseEstimationMethod);
    dynamic_cast<vpDetectorAprilTag *>(detector)->setDisplayTag(display_tag);
    try{
        vpImage<unsigned char> I;
        for(;;){
            vpDisplay::display(I);
            std::vector<vpHomogeneousMatrix> cMo_vec;
            dynamic_cast<vpDetectorAprilTag *>(detector)->detect(I, tagSize, cam, cMo_vec);
            for (size_t i = 0; i < cMo_vec.size(); i++) {
                vpDisplay::displayFrame(I, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3);
                std::cout<<cMo_vec[i]<<std::endl;
            }
            vpDisplay::displayText(I, 20, 20, "Click to quit.", vpColor::red);
            vpDisplay::flush(I);
            if (vpDisplay::getClick(I, false))
                break;
        }
        delete detector;
    }catch(const vpException &e){
        std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
    }
    return EXIT_SUCCESS;
}