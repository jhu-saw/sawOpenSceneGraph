#include <cisstCommon/cmnPath.h>
#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/osaOSGStereo.h>
#include <sawOpenSceneGraph/osaOSGBody.h>

int main(){

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  osg::ref_ptr< osaOSGWorld > world = new osaOSGWorld;

  // Create a camera
  int x = 0, y = 0;
  int width = 320, height = 240;
  double Znear = 0.1, Zfar = 10.0;
  osg::ref_ptr< osaOSGCamera > camera;
  camera = new osaOSGStereo( world,
			     x, y, width, height,
			     55.0, ((double)width)/((double)height),
			     Znear, Zfar,
			     0.10,
			     false );
  camera->Initialize( std::string( "Stereo-" ) );

  // Create the objects
  cmnPath path;
  path.AddRelativeToCisstShare("/models");
  path.AddRelativeToCisstShare("/models/hubble");

  vctFrame4x4<double> Rt(  vctMatrixRotation3<double>(),
			   vctFixedSizeVector<double,3>( 0.0, 0.0, 0.5 ) );

  osg::ref_ptr< osaOSGBody > hubble;
  hubble = new osaOSGBody( path.Find("hst.3ds"), world, Rt );

  osg::ref_ptr< osaOSGBody > background;
  background = new osaOSGBody( path.Find("background.3ds"), world,
			       vctFrame4x4<double>() );

  std::cout << "ESC to quit" << std::endl;

  // animate and render
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
			      vctFixedSizeVector<double,3>(0.0, 0.0, theta ) );
    camera->SetTransform( Rtwc );

    camera->frame();

    theta += 0.001;

  }

  return 0;

}
