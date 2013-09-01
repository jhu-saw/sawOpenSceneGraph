#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnGetChar.h>
#include <saw3Dconnexion/mts3Dconnexion.h>

#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/mtsOSGMono.h>
#include <sawOpenSceneGraph/osaOSGDragger.h>

int main( int argc, char** argv ){

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

  mts3Dconnexion snleft("snleft", 0.01 );
  mts3Dconnexion snright("snright", 0.01 );
  if( argc == 4 ){
    snleft.Configure( std::string( argv[1] ) );
    snright.Configure( std::string( argv[1] ) );
  }
  else{
    std::cerr << "Usage: " << argv[0] << " joystick_device_file" << std::endl;
    return -1;
  }
  taskManager->AddComponent( &snleft );
  taskManager->AddComponent( &snright );

  osg::ref_ptr< osaOSGWorld > world = new osaOSGWorld;

  // Create a camera
  mtsOSGMono* camera;
  {
    int x = 0, y = 0;
    int width = 320, height = 240;
    double Znear = 0.1, Zfar = 10.0;
    camera = new mtsOSGMono( "camera",
			     world,
			     x, y,
			     width, height,
			     55.0, ((double)width)/((double)height),
			     Znear, Zfar, false );
    //camera->Initialize();
    taskManager->AddComponent( camera );
  }

  cmnPath path;
  path.AddRelativeToCisstShare("/models/hubble");
  path.AddRelativeToCisstShare("/models");
  path.Add( "/home/sleonard/src/wvu-jhu/trunk/models/objects/" );

  osg::ref_ptr< osaOSGDragger > hubble;
  hubble = new osaOSGDragger( path.Find( "hst.3ds" ), world );

  osg::ref_ptr< osaOSGDragger > landsat;
  landsat = new osaOSGDragger( path.Find( "landsat.fbx" ), world, 
			       vctFrame4x4<double>(), 0.01);
  
  double xl = 0.0;
  double yl = -0.2;
  double zl = -0.318577;
  vct3 tl( xl, yl, zl );
  osg::ref_ptr< osaOSGPointer > left;
  left = new osaOSGPointer(world, 
			   vctFrame4x4<double>(vctMatrixRotation3<double>(),tl),
			   "dvleft",
			   0.01,
			   1.0);

  osg::ref_ptr< osaOSGPointer > right;
  right = new osaOSGPointer(world, 
			   vctFrame4x4<double>(vctMatrixRotation3<double>(),tl),
			   "dvright",
			   0.01,
			   1.0);

  osg::ref_ptr< osaOSGMouse > mouse;
  camera->addEventHandler( new osaOSGMouse() );

  taskManager->CreateAll();
  taskManager->WaitForStateAll( mtsComponentState::READY );

  taskManager->StartAll();
  taskManager->WaitForStateAll( mtsComponentState::ACTIVE );

  std::cout << "ENTER to quit" << std::endl;
  cmnGetChar();

  taskManager->KillAll();
  taskManager->Cleanup();

  return 0;

}
