#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

template<typename ... Args>
static std::string str_format(const std::string& format, Args ... args)
{
	auto size_buf = std::snprintf(nullptr, 0, format.c_str(), args ...) + 1;
	std::unique_ptr<char[]> buf(new(std::nothrow) char[size_buf]);

	if (!buf)
		return std::string("");

	std::snprintf(buf.get(), size_buf, format.c_str(), args ...);
	return std::string(buf.get(), buf.get() + size_buf - 1);
}

int main() {
    using namespace std;
    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("rgb");

    // Properties
    camRgb->setPreviewSize(300, 300);
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);

    // Linking
    camRgb->preview.link(xoutRgb->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline, dai::UsbSpeed::SUPER);

    cout << "Connected cameras: " << device.getConnectedCameraFeatures() << endl;

    // Print USB speed
    cout << "Usb speed: " << device.getUsbSpeed() << endl;

    // Bootloader version
    if(device.getBootloaderVersion()) {
        cout << "Bootloader version: " << device.getBootloaderVersion()->toString() << endl;
    }

    // Device name
    cout << "Device name: " << device.getDeviceName() << endl;

    // Output queue will be used to get the rgb frames from the output defined above
    auto qRgb = device.getOutputQueue("rgb", 4, false);


    int num_of_calibration = 30;
    int num_images = 0;
    while(true) {
        auto inRgb = qRgb->get<dai::ImgFrame>();

        // Retrieve 'bgr' (opencv format) frame
        cv::imshow("rgb", inRgb->getCvFrame());

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            break;
        }else if(key=='t'||key=='T'){
            if(num_images< num_of_calibration){
                std::string path = str_format("/home/yuxuan/Project/Big_Rover/AprilTag/april_tag/src/calibration/src/chessboard-%02d.jpg",num_images);
                cv::imwrite(path,inRgb->getCvFrame());
                std::cout<<"Store "<<num_images<<"Pictures"<<std::endl;
                num_images++;
            }
        }
    }
    device.close();
    return 0;
}
