#include <cisstCommon/cmnPath.h>
#include <sawOpenSceneGraph/osaOSGBody.h>
#include <sawOpenSceneGraph/osaOSGPick.h>
#include <sawOpenSceneGraph/osaOSGMono.h>
#include <sawOpenSceneGraph/osaOSGWorld.h>

#include <cisstVector/vctMatrixRotation3.h>

class MyPick : public osaOSGPick{
  void PickHandler(osaOSGBody *body, Event, double, double) {
    if (body != NULL) {
      std::cout << body->getName() << std::endl;
    }
  }
};

int main(){

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  // Create the OSG world
  osg::ref_ptr<osaOSGWorld> world = new osaOSGWorld;

  // Create a camera
  int x = 0, y = 0;
  int width = 640, height = 480;
  double Znear = 0.1, Zfar = 10.0;
  osg::ref_ptr<osaOSGMono> camera;
  camera = new osaOSGMono( world,
			     x, y, width, height,
			     55.0, ((double)width)/((double)height),
			     Znear, Zfar );
  camera->Initialize();
  camera->addEventHandler( new MyPick() );

  // Create objects
  cmnPath path;
  path.AddRelativeToCisstShare("models");
  path.AddRelativeToCisstShare("models/hubble");
  vctFrame4x4<double> Rt( vctMatrixRotation3<double>(),
			  vctFixedSizeVector<double,3>(0.0, 0.0, 0.5) );

  std::cout << path.Find("background.3ds") << std::endl;
  vctFrame4x4<double> eye;
  osg::ref_ptr<osaOSGBody> background;
  background = new osaOSGBody( path.Find("background.3ds"), world, eye );
  background->setName( "background" );

  osg::ref_ptr<osaOSGBody> hubble;
  hubble = new osaOSGBody( path.Find("hst.3ds"), world, Rt );
  hubble->setName( "hubble" );

  while( !camera->done() ) {
    camera->frame();
  }

  return 0;

}
