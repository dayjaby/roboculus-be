#include <Base/Common/W32Compat.hh> // sprintf wrn
#include <VideoSource/VideoSource_OpenNI.hh>
#include <VideoSource/VideoSourceCapabilities.hh>
#include <VideoSource/VideoServer.hh>
#include <Base/Image/Image.hh>
#include <Base/Image/ImageIO.hh>
#include <Base/Image/ImageConvert.hh>
#include <Base/Debug/TimeMeasure.hh>
#include <Image/Camera.hh>
#include <vector>
#include <string>

namespace BIAS {

class VideoSource_OpenNI_Depth : public BIAS::VideoSource_OpenNI
{
public:
  VideoSource_OpenNI_Depth()
  {
    SetDepthCaptureMode(BIAS::VideoSource_Kinect_Base::DepthEuclidean);
    tmp.Init(width,height,depth_channels,ImageBase::ST_float);
    tmp.SetColorModel(ImageBase::CM_Depth);
    InitDepthImage(tmp);
  }
  virtual ~VideoSource_OpenNI_Depth() {
  }

  virtual int GrabSingle(BIAS::Camera<unsigned char>& image) {
    int result = BIAS::VideoSource_OpenNI::GrabSingleDepth(tmp);
    /*ImageConvert::ConvertST is inconvenient for our purpose,
      because it merely casts the float to unsigned char,
      which is a mistake for values>255
      BIAS::ImageConvert::ConvertST(tmp,image,BIAS::ImageBase::ST_unsignedchar);*/
    image.Init(width, height, convert_channels, BIAS::ImageBase::ST_unsignedchar);
    /* do some dirty float to 2* unsigned char converting
       the value of the depth_channel is the distance in millimeters,
       thus it's not a problem to omit the decimal place for our purpose
       bytes to send per second: 30*640*480*2 ~= 18MB!
    */
    /// TODO: consider using only 1 convert channel to use only half the bandwidth
    for(int x=0;x<width;x++) {
        for(int y=0;y<height;y++) {
            int v = tmp.GetValue<int>(tmp,x,y,0);
            if(v>65535) v = 65535;
            // two convert channels
            image.SetValue(image,x,y,0,(unsigned char)v&0xFF);
            image.SetValue(image,x,y,1,(unsigned char)(v>>8)&0xFF);
        }
    }
    return result;
  }
private:
  // constant values, don't change!
  enum { width = 640, height = 480, depth_channels = 1, convert_channels = 2};
  BIAS::Camera<float> tmp;
};

class FPSVideoServer : public BIAS::VideoServer
{
public:
  FPSVideoServer(int fps)
  : fps_(fps)
  , usecPerFrame_(1000000.0/fps)
  {
  }

  virtual ~FPSVideoServer() {}

  int Process() {
    timer_.Start();
    int res=ProcessOneImage();
    timer_.Stop();
    /** calculate the real fps
     * if the real fps is higher than the desired fps, wait the rest of the time
    **/
    double realTime = timer_.GetRealTime();
    double waitTime = usecPerFrame_ - realTime;
    if(waitTime  > 0) {
      biasusleep(waitTime);
      realTime+=waitTime;
    }

    realFps_ = 1000000.0f / float( realTime );
    timer_.Reset();
    return res;
  }

  double getFPS() { return fps_; }
  double getRealFPS() { return realFps_; }
  double getUsecPerFrame() { return usecPerFrame_; }
private:
  const double fps_;
  double realFps_;
  const double usecPerFrame_;
  BIAS::TimeMeasure timer_;
};

}


int main(int argc, char**argv) {
  BIAS::VideoSource_OpenNI_Depth cam;
  cam.OpenDevice();
  cam.PreGrab();
  std::cout << "Init server" << std::endl;
  // init a video server with 30 FPS, because the kinect camera provides 30 FPS
  BIAS::FPSVideoServer server(30);
  std::cout << server.InitFromExistingSource(&cam,44887) << std::endl;

  while(true) {
    if(server.Process() == 0) {
      std::cout << "Image sent with " << server.getRealFPS() << " FPS" << std::endl;
    } else {
      //std::cout << "No client connected; " << server.getRealFPS() << " FPS" << std::endl;
    }
  }
  cam.CloseDevice();

  return 0;
}
