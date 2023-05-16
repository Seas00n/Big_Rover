#include <ros/ros.h>
#include <depthai/depthai.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

int main(int argc, char **argv){
    dai::Pipeline pipeline;
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    xoutRgb->setStreamName("rgb");
    camRgb->setPreviewSize(300, 300);
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
    camRgb->preview.link(xoutRgb->input);
    dai::Device device(pipeline, dai::UsbSpeed::SUPER);
    auto qRgb = device.getOutputQueue("rgb", 4, false);
    ros::init(argc,argv,"cam_node");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("image_raw", 10);
    int video_device = 1;
    int frame_rate = 25;
    nh.param<int>("video_device", video_device, 0);
    nh.param<int>("frame_rate", frame_rate, 30);
    ros::Rate loop_rate(frame_rate);
    while(nh.ok()){
        cv::Mat image;
        auto inRgb = qRgb->get<dai::ImgFrame>();
        cv::imshow("rgb",inRgb->getCvFrame());
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = ros::Time::now();
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = image;
        pub.publish(out_msg.toImageMsg());
        int key = cv::waitKey(1);
        if(key == 'q'){
            break;
        }
        loop_rate.sleep();
    }
}