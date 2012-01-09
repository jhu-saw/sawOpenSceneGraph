#include <cisstOSAbstraction/osaSleep.h>
#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/osaOSGMono.h>
#include <sawOpenSceneGraph/osaOSGImage.h>
#include <sawOpenSceneGraph/osaOSGBody.h>
#include <osgDB/ReadFile>

int main( ){

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  osg::ref_ptr< osaOSGWorld > world = new osaOSGWorld;
  
  // Create a camera
  int x = 0, y = 0;
  int width = 640, height = 480;
  double Znear = 0.1, Zfar = 10.0;
  osg::ref_ptr< osaOSGMono > camera;
  camera = new osaOSGMono( world,
			   x, y, width, height,
			   55.0, ((double)width)/((double)height),
			   Znear, Zfar );
  camera->Initialize();

  osg::ref_ptr< osaOSGImage > image;
  image = new osaOSGImage( -.5, -.5, 1, 1, world, vctFrame4x4<double>() );

  int i=0;
  while( i < 124 ){

    char buffer[128];
    sprintf( buffer, "walkcircle/frame%04d.tif", i++ );

    image->SetImage( std::string( buffer ) );
    camera->frame();

    osaSleep( 1.0/30.0 );

  }


  return 0;

}
