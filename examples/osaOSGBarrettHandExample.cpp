#include <cisstCommon/cmnPath.h>
#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/osaOSGMono.h>
#include <sawOpenSceneGraph/osaOSGBH.h>

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

  cmnPath path;
  path.AddRelativeToCisstShare("/models/BH");
  vctFrame4x4<double> Rtw0;

  std::vector< std::string > models;
  models.push_back( path.Find("l0.obj") );
  models.push_back( path.Find("l1.obj") );
  models.push_back( path.Find("l2.obj") );
  models.push_back( path.Find("l3.obj") );

  osg::ref_ptr<osaOSGBH> bh;
  bh = new osaOSGBH( path.Find("l0.obj"),
		     path.Find("l1.obj"),
		     path.Find("l2.obj"),
		     path.Find("l3.obj"),
		     world,
		     Rtw0,
		     path.Find("f1f2.rob"),
		     path.Find("f3.rob") );

  std::cout << "ESC to quit" << std::endl;

  vctDynamicVector<double> q( 4, 0.0 );
  while( !camera->done() ){

    for( size_t i=0; i<4; i++ ) q[i] += 0.001;
    bh->SetPositions( q );
    camera->frame();

  }

  return 0;

}
