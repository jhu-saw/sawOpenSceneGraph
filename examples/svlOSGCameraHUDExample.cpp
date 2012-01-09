#include <cisstOSAbstraction/osaSleep.h>
#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/osaOSGMono.h>
#include <sawOpenSceneGraph/osaOSGHUD.h>
#include <sawOpenSceneGraph/osaOSGBody.h>
#include <sawOpenSceneGraph/svlOSGImage.h>
#include <osgText/Text>

#include <cisstStereoVision.h>


int main( int, char** argv ){

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

  // Create the objects
  std::string path( CISST_SOURCE_ROOT"/etc/cisstRobot/objects/" );

  vctFrame4x4<double> Rt(  vctMatrixRotation3<double>(),
			   vctFixedSizeVector<double,3>( 0.0, 0.0, 0.5 ) );

  osg::ref_ptr< osaOSGBody > background;
  background = new osaOSGBody( path+"background.3ds", world, 
 			       vctFrame4x4<double>() );
  osg::ref_ptr< osaOSGBody > hubble;
  hubble = new osaOSGBody( path+"hst.3ds", world, Rt );
  
  osg::ref_ptr< osaOSGHUD > hud;
  hud = new osaOSGHUD( world, width, height, camera );

  //osg::ref_ptr< osaOSGImage > image;
  //image = new osaOSGImage( 0, 0, width, height, hud );

  svlInitialize();

  // Creating SVL objects
  svlStreamManager stream;
  svlOSGImage imageseq( 0, 0, width, height, hud );

  svlFilterSourceVideoFile source(1);
  source.SetFilePath( "xray.avi" );


  stream.SetSourceFilter( &source );
  source.GetOutput()->Connect( imageseq.GetInput() );

  if (stream.Play() != SVL_OK)
    std::cout <<"error"<<std::endl;

  while( 1 )
    { camera->frame(); }

  pause();

  return 0;

}
