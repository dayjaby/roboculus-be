#include <Base/Common/W32Compat.hh> // sprintf wrn
#include <VideoSource/VideoSource_OpenNI.hh>
#include <VideoSource/VideoSourceCapabilities.hh>
#include <Base/Image/Image.hh>
#include <Base/Image/ImageIO.hh>
#include <Image/Camera.hh>
#include <vector>
#include <string>


int main(int argc, char**argv) {
  BIAS::VideoSource_OpenNI* cam = new BIAS::VideoSource_OpenNI;
  cam->SetDepthCaptureMode(BIAS::VideoSource_Kinect_Base::DepthEuclidean);
  BIAS::Camera<float> img;
  cam->StartDepthStream();
  cam->OpenDevice();
  cam->CloseDevice();
  cam->InitImage(img);
  cam->PreGrab();
  cam->GrabSingleDepth(img);
  cam->PostGrab();
  cam->StopDepthStream();
  cam->CloseDevice();
  delete(cam);
  BIAS::ImageIO::ExportLibJPEG("test.jpg",img);
  return 0;
}