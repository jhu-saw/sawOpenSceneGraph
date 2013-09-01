#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnGetChar.h>
#include <saw3Dconnexion/osa3Dconnexion.h>

#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/mtsOSGMono.h>
#include <sawOpenSceneGraph/osaOSGBodyUI.h>

int main( int argc, char** argv ){

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

  osa3Dconnexion snleft;//("snleft", 0.01 );
  osa3Dconnexion snright;//("snright", 0.01 );
  if( argc == 3 ){
    //snleft.Configure( std::string( argv[1] ) );
    //snright.Configure( std::string( argv[2] ) );
    snleft.Open( std::string( argv[1] ) );
    snright.Open( std::string( argv[2] ) );
  }
  else{
    std::cerr << "Usage: " << argv[0] << " js1_dev_file js2_dev_file\n";
    return -1;
  }
  //taskManager->AddComponent( &snleft );
  //taskManager->AddComponent( &snright );

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

  osg::ref_ptr< osaOSGBodyUI > hubble;
  hubble = new osaOSGBodyUI( path.Find( "hst.3ds" ), world );

  osg::ref_ptr< osaOSGBodyUI > landsat;
  landsat = new osaOSGBodyUI( path.Find( "landsat.fbx" ), world, 
			      vctFrame4x4<double>(), 0.01);
  
  double xl = 0.0;
  double yl = -0.2;
  double zl = -0.318577;
  vct3 tl( xl, yl, zl );
  osg::ref_ptr< osaOSGPointer3D > left;
  left = new osaOSGPointer3D(world, 
			     vctFrame4x4<double>(vctMatrixRotation3<double>(),tl),
			     "dvleft",
			     0.01,
			     1.0);
  
  osg::ref_ptr< osaOSGPointer3D > right;
  right = new osaOSGPointer3D(world, 
			      vctFrame4x4<double>(vctMatrixRotation3<double>(),tl),
			      "dvright",
			      0.01,
			      1.0);

  osg::ref_ptr< osaOSGPointer2D > mouse;
  camera->addEventHandler( new osaOSGPointer2D() );

  taskManager->CreateAll();
  taskManager->WaitForStateAll( mtsComponentState::READY );

  taskManager->StartAll();
  taskManager->WaitForStateAll( mtsComponentState::ACTIVE );

  std::cout << "ENTER to quit" << std::endl;
  while(1){

    osa3Dconnexion::Event el = snleft.WaitForEvent();
    switch( el.type ){
    case osa3Dconnexion::Event::MOTION:
      {
	vctFixedSizeVector<double,3> v( el.data[0], el.data[1], el.data[2] );
	left->SetVelocity( v );
	std::cout << v << std::endl;
      }
      break;
    case osa3Dconnexion::Event::BUTTON_PRESSED:
      left->Select();
      break;
    case osa3Dconnexion::Event::BUTTON_RELEASED:
      left->Release();
      break;
    default:
      break;
    }    

    osa3Dconnexion::Event er = snright.WaitForEvent();
    switch( er.type ){
    case osa3Dconnexion::Event::MOTION:
      {
	vctFixedSizeVector<double,3> v( er.data[0], er.data[1], er.data[2] );
	//right->SetVelocity( v );
	std::cout << v << std::endl;
      }
      break;
    case osa3Dconnexion::Event::BUTTON_PRESSED:
      right->Select();
      break;
    case osa3Dconnexion::Event::BUTTON_RELEASED:
      right->Release();
      break;
    default:
      break;
    }    

  }

  taskManager->KillAll();
  taskManager->Cleanup();

  return 0;

}
