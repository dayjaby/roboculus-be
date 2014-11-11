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
    std::cout << "initf" << std::endl;
    tmp.Init(640,480,1,ImageBase::ST_float);
    std::cout << "initf" << std::endl;
    tmp.SetColorModel(ImageBase::CM_Depth);
    std::cout << "initf" << std::endl;
    //InitDepthImage(tmp);
  }
  virtual ~VideoSource_OpenNI_Depth() {
  }
  
  virtual int GrabSingle(BIAS::Camera<unsigned char>& image) {
    std::cout << "Grab single" << std::endl;
    int result = BIAS::VideoSource_OpenNI::GrabSingleDepth(tmp);
    std::cout << "Grab single" << std::endl;
    BIAS::ImageConvert::ConvertST(tmp,image,BIAS::ImageBase::ST_unsignedchar);
    std::cout << "Grab single" << std::endl;
    return result;
  }
private:
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
  nitFromExistingSource(VideoSource *theSource, int port) {
133  int res;
134  if (port!=port_)
135  port_=port;
136  cout <<"Initializing VideoServer on port:"<<port_<<endl;
137  camera_ = theSource;
138  if (!camera_->IsActive()) {
139  res = camera_->OpenDevice();
140  if (res!=0) return res;
141  res = camera_->PreGrab();
142  if (res!=0) return res;
143  }
144 
145  imgWidth_=camera_->GetWidth();
146  imgHeight_=camera_->GetHeight();
147 
148 #ifdef BIAS_HAVE_LIBJPEG
149  if (jpeg_ > 0) {
150  if (Compressor_.Init() < 0) {
151  BIASERR("Could not initialize JPEG-compression!!!");
152  return -1;
153  }
154  }
155 #else //BIAS_HAVE_LIBJPEG
156  if (jpeg_ > 0) {
157  BIASERR("Compression mode not available without LIBJPEG!");
158  }
159 #endif //BIAS_HAVE_LIBJPEG
160  if (useUDP_){
161  // Transmitter_.Connect(TargetAddress_->c_str(),*SendPort_);
162  } else {
163  server_.WaitForConnections(port_);
164  }
165  camera_->InitImage(CamImage_);
166  cout<<"Grabbing image size: "<< imgWidth_<<"x"<< imgHeight_<<endl;
167  Initialized_ = true;
168  return 0;
169 }}
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
  return 0;
}