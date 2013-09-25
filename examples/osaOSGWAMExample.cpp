#include <cisstCommon/cmnPath.h>
#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/osaOSGMono.h>
#include <sawOpenSceneGraph/osaOSGManipulator.h>

int main( int, char** ){

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
			   Znear, Zfar,
			   true,
			   vctFrame4x4<double>(),
			   true );
  camera->Initialize();


  cmnPath path;
  path.AddRelativeToCisstShare("/models/WAM");
  vctFrame4x4<double> Rtw0;

  std::vector< std::string > models;
  models.push_back( path.Find("l1.obj") );
  models.push_back( path.Find("l2.obj") );
  models.push_back( path.Find("l3.obj") );
  models.push_back( path.Find("l4.obj") );
  models.push_back( path.Find("l5.obj") );
  models.push_back( path.Find("l6.obj") );
  models.push_back( path.Find("l7.obj") );

  osg::ref_ptr<osaOSGManipulator> wam;

  wam = new osaOSGManipulator( models,
			       world,
			       Rtw0,
			       path.Find("wam7.rob"),
			       path.Find("l0.obj") );

  std::cout << "ESC to quit" << std::endl;

  vctDynamicVector<double> q( 7, 0.0 );
  while( !camera->done() ){
    for( size_t i=0; i<7; i++ ) q[i] += 0.001;
    wam->SetPositions( q );
    camera->frame();
  }

  return 0;

}
