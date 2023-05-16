#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp_ros/vpROSGrabber.h>

int main(int argc, const char** argv){
    try{
        vpImage<vpRGBa> I;
        vpROSGrabber g;
        g.setImageTopic("/cam_node/image_raw");
        g.open(I);
    #ifdef VISP_HAVE_X11
        vpDisplayX d(I);    
    #else
        std::cout<<"No image viewer is available\n";
    #endif
        while(1){
            g.acquire(I);
            vpDisplay::display(I);
            vpDisplay::displayText(I,20,20,
            "Click to quit...",vpColor::red);
            vpDisplay::flush(I);
            if(vpDisplay::getClick(I,false)){
                break;
            }
        }
    }catch(vpException e){
        std::cout << "Catch an exception:"<< e <<std::endl;
    }
}