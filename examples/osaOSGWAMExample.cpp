#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/osaOSGMono.h>
#include <sawOpenSceneGraph/osaOSGManipulator.h>

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
  camera = new osaOSGMono( world,
			     x, y, width, height,
			     55.0, ((double)width)/((double)height),
			     Znear, Zfar );
  camera->Initialize();

  
  std::string path( CISST_SOURCE_ROOT"/cisst/etc/cisstRobot/WAM/" );
  vctFrame4x4<double> Rtw0;
  
  std::vector< std::string > models;
  models.push_back( path + "l1.obj" );
  models.push_back( path + "l2.obj" );
  models.push_back( path + "l3.obj" );
  models.push_back( path + "l4.obj" );
  models.push_back( path + "l5.obj" );
  models.push_back( path + "l6.obj" );
  models.push_back( path + "l7.obj" );

  osg::ref_ptr<osaOSGManipulator> wam;
  wam = new osaOSGManipulator( models,
				 world,
				 Rtw0,
				 path + "wam7.rob",
				 path + "l0.obj" );
  
  std::cout << "ESC to quit" << std::endl;

  vctDynamicVector<double> q( 7, 0.0 );
  while( !camera->done() ){
    
    for( size_t i=0; i<7; i++ ) q[i] += 0.001;
    wam->SetPositions( q );
    camera->frame();

  }

  return 0;

}
