#include <cisstOSAbstraction/osaSleep.h>
#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/osaOSGMono.h>
#include <sawOpenSceneGraph/osaOSGHUD.h>
#include <sawOpenSceneGraph/osaOSGBody.h>
#include <sawOpenSceneGraph/osaOSGImage.h>
#include <osgText/Text>
#undef SAW_OPENSCENEGRAPH_SUPPORTS_OPENCV

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

  osg::ref_ptr< osaOSGImage > image;
  image = new osaOSGImage( 0, 0, width, height, hud );
  hud->addChild( hubble);

  std::cout << "ESC to quit" << std::endl;

  // animate and render
  int i=0;
  double theta=1.0;

  while( !camera->done() ){

    // rotate hubble
    vctFixedSizeVector<double,3> u( 0.0, 0.0, 1.0 );
    vctAxisAngleRotation3<double> Rwh( u, theta );
    vctFrame4x4<double> Rtwh(  Rwh,
			       vctFixedSizeVector<double,3>( 0.0, 0.0, 0.5 ) );
    hubble->SetTransform( Rtwh );

    // zoom out the camera
    vctFrame4x4<double> Rtwc( vctMatrixRotation3<double>(),
			      vctFixedSizeVector<double,3>(0.0, 0.0, theta ));
    camera->SetTransform( Rtwc );


    char buffer[128];
    sprintf( buffer, "walkstraight/frame%04d.tif", i++ );

    if( i < 124 )
      image->SetImage( std::string( buffer ) );
    if( i == 124 ) i = 0;
    camera->frame();
    
    theta += 0.001;
    
    osaSleep( 1.0/30.0 );

  }

  return 0;

}
