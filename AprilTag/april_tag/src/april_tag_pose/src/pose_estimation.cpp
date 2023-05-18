#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp_ros/vpROSGrabber.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#ifdef VISP_HAVE_XML2
#include  <visp3/core/vpXmlParserCamera.h>
#endif
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"


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

    static tf2_ros::TransformBroadcaster bc1;
    static tf2_ros::TransformBroadcaster bc2;
    static tf2_ros::TransformBroadcaster bc3;
    static tf2_ros::TransformBroadcaster bc4;
    
    geometry_msgs::TransformStamped tfs1;
    tfs1.header.frame_id = "world";
    tfs1.header.stamp = ros::Time::now();
    tfs1.child_frame_id = "son1";
    
    geometry_msgs::TransformStamped tfs2;
    tfs2.header.frame_id = "world";
    tfs2.header.stamp = ros::Time::now();
    tfs2.child_frame_id = "son2";

    geometry_msgs::TransformStamped tfs3;
    tfs3.header.frame_id = "world";
    tfs3.header.stamp = ros::Time::now();
    tfs3.child_frame_id = "son3";

    geometry_msgs::TransformStamped tfs4;
    tfs4.header.frame_id = "world";
    tfs4.header.stamp = ros::Time::now();
    tfs4.child_frame_id = "son4";



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
        std::stringstream ss;

        for(;;){
            g.acquire(I);
            vpDisplay::display(I);
            double t = vpTime::measureTimeMs();
            std::vector<vpHomogeneousMatrix> cMo_vec;
            vpTranslationVector trans;
            vpRxyzVector eular_angle; 
            dynamic_cast<vpDetectorAprilTag *>(detector)->detect(I, tagSize, cam, cMo_vec);
            time_vec.push_back(t-t0);
            vpDisplay::displayText(I,20,20,"Click to quit...",vpColor::red);
            for(size_t i=0;i<cMo_vec.size();i++){
                vpDisplay::displayFrame(I,cMo_vec[i],cam,tagSize/2,vpColor::none,3);
                trans = cMo_vec[i].getTranslationVector();
                eular_angle.buildFrom(cMo_vec[i].getRotationMatrix());
                tf2::Quaternion qtn;
                qtn.setRPY(eular_angle[0],eular_angle[1],eular_angle[2]);
                std::string tag_msg = dynamic_cast<vpDetectorAprilTag *>(detector)->getMessage(i);
                std::size_t tag_id_pos = tag_msg.find("id:");
                int tag_id = atoi(tag_msg.substr(tag_id_pos).c_str());
                ss.str("");
                ss<<"id:"<<tag_id;
                vpRect bbox = detector->getBBox(i);
                vpDisplay::displayText(I,
                                (int)(bbox.getTop()-10),
                                (int)(bbox.getLeft()-10),
                                ss.str(),vpColor::red);
                if(i == 0){
                    tfs1.transform.translation.x = trans[0];
                    tfs1.transform.translation.y = trans[1];
                    tfs1.transform.translation.z = trans[2];
                    tfs1.transform.rotation.x = qtn.getX();
                    tfs1.transform.rotation.y = qtn.getY();
                    tfs1.transform.rotation.z = qtn.getZ();
                    tfs1.transform.rotation.w = qtn.getW();
                    bc1.sendTransform(tfs1);
                }else if(i == 1){
                    tfs2.transform.translation.x = trans[0];
                    tfs2.transform.translation.y = trans[1];
                    tfs2.transform.translation.z = trans[2];
                    tfs2.transform.rotation.x = qtn.getX();
                    tfs2.transform.rotation.y = qtn.getY();
                    tfs2.transform.rotation.z = qtn.getZ();
                    tfs2.transform.rotation.w = qtn.getW();
                    bc2.sendTransform(tfs2);
                }else if(i == 2){
                    tfs3.transform.translation.x = trans[0];
                    tfs3.transform.translation.y = trans[1];
                    tfs3.transform.translation.z = trans[2];
                    tfs3.transform.rotation.x = qtn.getX();
                    tfs3.transform.rotation.y = qtn.getY();
                    tfs3.transform.rotation.z = qtn.getZ();
                    tfs3.transform.rotation.w = qtn.getW();
                    bc3.sendTransform(tfs3);
                }else{
                    tfs4.transform.translation.x = trans[0];
                    tfs4.transform.translation.y = trans[1];
                    tfs4.transform.translation.z = trans[2];
                    tfs4.transform.rotation.x = qtn.getX();
                    tfs4.transform.rotation.y = qtn.getY();
                    tfs4.transform.rotation.z = qtn.getZ();
                    tfs4.transform.rotation.w = qtn.getW();
                    bc4.sendTransform(tfs4);
                }

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