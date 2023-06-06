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
#include <ros/ros.h>
#include <Eigen/Eigen>

int main(int argc, char** argv){
    ros::init(argc,argv,"pose_estimation");
    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_ORTHOGONAL_ITERATION;
    double tagSize = 0.222;//0.077
    float quad_decimate = 0.2;
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
    static tf2_ros::TransformBroadcaster bc0;



    geometry_msgs::TransformStamped tfs0;
    tfs0.header.frame_id = "world";
    tfs0.header.stamp = ros::Time::now();
    tfs0.child_frame_id  = "camera_base";
    tf2::Quaternion qtn_base;
    qtn_base.setRPY(-M_PI/2,0,0);
    tfs0.transform.translation.x = 0;
    tfs0.transform.translation.y = 0;
    tfs0.transform.translation.z = 0;
    tfs0.transform.rotation.x = qtn_base.getX();
    tfs0.transform.rotation.y = qtn_base.getY();
    tfs0.transform.rotation.z = qtn_base.getZ();
    tfs0.transform.rotation.w = qtn_base.getW();




    geometry_msgs::TransformStamped tfs1;
    tfs1.header.frame_id = "camera_base";
    tfs1.header.stamp = ros::Time::now();
    tfs1.child_frame_id = "son1";
    
    geometry_msgs::TransformStamped tfs2;
    tfs2.header.frame_id = "camera_base";
    tfs2.header.stamp = ros::Time::now();
    tfs2.child_frame_id = "son2";

    geometry_msgs::TransformStamped tfs3;
    tfs3.header.frame_id = "camera_base";
    tfs3.header.stamp = ros::Time::now();
    tfs3.child_frame_id = "son3";

    geometry_msgs::TransformStamped tfs4;
    tfs4.header.frame_id = "camera_base";
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
        dynamic_cast<vpDetectorAprilTag *>(detector)->setAprilTagNbThreads(2);
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
                //在相机坐标系下的位置和姿态
                trans = cMo_vec[i].getTranslationVector();
                eular_angle.buildFrom(cMo_vec[i].getRotationMatrix());                
                tf2::Quaternion qtn;
                qtn.setRPY(eular_angle[0],eular_angle[1],eular_angle[2]);
                
                vpDisplay::displayFrame(I,cMo_vec[i],cam,tagSize/2,vpColor::none,3);
                std::string tag_msg = dynamic_cast<vpDetectorAprilTag *>(detector)->getMessage(i);
                const char* tag_msg_char = tag_msg.c_str();
                int tag_id = atoi((const char *)tag_msg_char+10);
                ss.str("");
                ss<<"id:"<<tag_id;
                vpRect bbox = detector->getBBox(i);
                vpDisplay::displayText(I,
                                (int)(bbox.getTop()-10),
                                (int)(bbox.getLeft()-10),
                                ss.str(),vpColor::red);
                if(tag_id == 0){
                    tfs1.transform.translation.x = trans[0];
                    tfs1.transform.translation.y = trans[1];
                    tfs1.transform.translation.z = trans[2];
                    tfs1.transform.rotation.x = qtn.getX();
                    tfs1.transform.rotation.y = qtn.getY();
                    tfs1.transform.rotation.z = qtn.getZ();
                    tfs1.transform.rotation.w = qtn.getW();
                    tfs1.header.stamp = ros::Time::now();
                    bc1.sendTransform(tfs1);
                }else if(tag_id == 1){
                    tfs2.transform.translation.x = trans[0];
                    tfs2.transform.translation.y = trans[1];
                    tfs2.transform.translation.z = trans[2];
                    tfs2.transform.rotation.x = qtn.getX();
                    tfs2.transform.rotation.y = qtn.getY();
                    tfs2.transform.rotation.z = qtn.getZ();
                    tfs2.transform.rotation.w = qtn.getW();
                    tfs2.header.stamp = ros::Time::now();
                    bc2.sendTransform(tfs2);
                }else if(tag_id == 2){
                    tfs3.transform.translation.x = trans[0];
                    tfs3.transform.translation.y = trans[1];
                    tfs3.transform.translation.z = trans[2];
                    tfs3.transform.rotation.x = qtn.getX();
                    tfs3.transform.rotation.y = qtn.getY();
                    tfs3.transform.rotation.z = qtn.getZ();
                    tfs3.transform.rotation.w = qtn.getW();
                    tfs3.header.stamp = ros::Time::now();
                    bc3.sendTransform(tfs3);
                }else{
                    tfs4.transform.translation.x = trans[0];
                    tfs4.transform.translation.y = trans[1];
                    tfs4.transform.translation.z = trans[2];
                    tfs4.transform.rotation.x = qtn.getX();
                    tfs4.transform.rotation.y = qtn.getY();
                    tfs4.transform.rotation.z = qtn.getZ();
                    tfs4.transform.rotation.w = qtn.getW();
                    tfs4.header.stamp = ros::Time::now();
                    bc4.sendTransform(tfs4);
                }
                tfs0.header.stamp = ros::Time::now();
                bc0.sendTransform(tfs0);
            }
            // std::cout<<"T"<<sqrt(trans[0]*trans[0]+trans[1]*trans[1]+trans[2]*trans[2])<<std::endl;
            // std::cout<<"Angle"<<eular_angle<<std::endl;
            vpDisplay::flush(I);
            if(vpDisplay::getClick(I,false)){
                break;
            }
        }
        }catch(vpException e){
            std::cout << "Catch an exception:"<< e <<std::endl;
        }
}