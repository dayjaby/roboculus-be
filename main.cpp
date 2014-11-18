#include <Base/Common/W32Compat.hh> // sprintf wrn
#include <VideoSource/VideoSource_OpenNI.hh>
#include <VideoSource/VideoSource_Kinect.hh>
#include <VideoSource/VideoSourceCapabilities.hh>
#include <VideoSource/VideoServer.hh>
#include <Base/Image/Image.hh>
#include <Base/Image/ImageIO.hh>
#include <Base/Image/ImageConvert.hh>
#include <Base/Debug/TimeMeasure.hh>
#include <Image/Camera.hh>
#include <vector>
#include <string>
#include <cstdlib>
namespace BIAS {

class FPSOpenNiVideoServer : public BIAS::VideoServer
{
public:
  FPSOpenNiVideoServer(int fps)
  : fps_(fps)
  , usecPerFrame_(1000000.0/fps)
  {
  }

  virtual ~FPSOpenNiVideoServer() {}
  
  virtual int InitFromExistingSource(BIAS::VideoSource_OpenNI *theSource, int port) {
    std::cout << "init" << std::endl;
    openNI = theSource;
    int res = BIAS::VideoServer::InitFromExistingSource(theSource,port);
    openNI->InitDepthImage(depth_);
    depth_send_.Init(640,480,2);
    return 0;
  }
  
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
  int ProcessOneImage()  {
    int ret = 0;
    if (! Initialized_) {
      BIASERR("VideoServer not initialized");
      return -1;
    }
    if (server_.GetConnections()<=0)
    {
      return 1;
    }
    openNI->GrabSingle(CamImage_);
    
    ret = sendImage("BIAS_IMAGE",CamImage_);
    if(ret==0) {
      openNI->GrabSingleDepth(depth_);
      for(int x=0;x<640;x++) {
	  for(int y=0;y<480;y++) {
	      int v = (int)(depth_.PixelValue(x,y,0));
	      if(v>65535) v = 65535;
	      // two convert channels
	      depth_send_.SetPixel((unsigned char)v&0xFF,x,y,0);
	      depth_send_.SetPixel((unsigned char)(v>>8)&0xFF,x,y,1);
	  }
      }
      ret = sendImage("BIAS_IMAGE_DEPTH",depth_send_);
    }
    return ret;
  }

  double getFPS() { return fps_; }
  double getRealFPS() { return realFps_; }
  double getUsecPerFrame() { return usecPerFrame_; }
protected:
  template <class T>
  int sendImage(const std::string& msgName, BIAS::Camera<T> img ) {
    int ret = 0;
    img.SetUID(BIAS::UUID::GenerateUUID());
    if (jpeg_ > 0) {
      BIASERR("LIBJPEG not configured --> no compression available !!!");
      return -1;
    } else {
      if (useUDP_){
      } else {
	std::stringstream dataToSend;
	dataToSend<<img;
	msgSize_= dataToSend.str().length();

	ret = server_.SendMsg(msgName,(char*)&(dataToSend.str()[0]), msgSize_);
	if (ret == -1) {
	  BIASERR("Sending of image data failed");
	  return ret;
	}
      }
    }
    return ret;
  }
private:
  BIAS::VideoSource_OpenNI *openNI;
  const double fps_;
  double realFps_;
  const double usecPerFrame_;
  BIAS::TimeMeasure timer_;
  BIAS::Camera<unsigned char> depth_send_;
  BIAS::Camera<float> depth_;
};

}


int main(int argc, char**argv) {
  BIAS::VideoSource_Kinect cam;
  BIAS::Camera<unsigned char> img;
  cam.OpenDevice();
  cam.SetVideoModeColor();
  cam.InitImage(img);
  cam.PreGrab();
  cam.GrabSingle(img);
  cam.PostGrab();
  cam.CloseDevice();
  BIAS::ImageIO::ExportLibJPEG("test.jpg",img);
  /*int port = atoi(argv[1]);
  BIAS::VideoSource_OpenNI cam;
  BIAS::FPSOpenNiVideoServer server(30);
  try {
  BIAS::Camera<unsigned char> img;
  cam.OpenDevice();
  cam.PreGrab();
  cam.InitImage(img);
  cam.GrabSingle(img);
  BIAS::ImageIO::ExportLibJPEG("test.jpg",img);
  return 0;
  std::cout << server.InitFromExistingSource(&cam,port) << std::endl;
    while(true) {
      if(server.Process() == 0) {
	std::cout << "Image sent with " << server.getRealFPS() << " FPS" << std::endl;
      } else {
	//std::cout << "No client connected; " << server.getRealFPS() << " FPS" << std::endl;
      }
    }
  } catch(std::exception& e) {
    std::cout << e.what() << std::endl;
  }
  cam.CloseDevice();

  return 0;*/
}
