#include <cisstCommon/cmnGetChar.h>

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsMatrix.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/mtsOSGStereo.h>
#include <sawOpenSceneGraph/osaOSGHUD.h>
#include <sawOpenSceneGraph/mtsOSGBody.h>
#include <sawOpenSceneGraph/svlOSGImage.h>

#include <cisstStereoVision.h>

// Hubble motion 
class HubbleMotion : public mtsTaskPeriodic {

private:

  // The position that the camera will be fetching
  prmPositionCartesianGet Rt;
  double theta;

public:

  HubbleMotion() : mtsTaskPeriodic( "HubbleMotion", 0.01, true ){
    theta = 0;
    Rt.Position().Translation()[2] = 0.5;

    StateTable.AddData( Rt, "PositionOrientation" );

    // provide the camera position
    mtsInterfaceProvided* output = AddInterfaceProvided( "Output" );
    output->AddCommandReadState( StateTable, Rt, "GetPositionCartesian" );

  }

  void Configure( const std::string& ){}
  void Startup(){}
  void Run(){
    ProcessQueuedCommands();
    
    // rotate hubble
    vctFixedSizeVector<double,3> u( 0.0, 0.0, 1.0 );
    vctAxisAngleRotation3<double> Rwh( u, theta );
    vctFrm3 Rtwh( Rwh, vctFixedSizeVector<double,3>( 0.0, 0.0, 0.5 ) );

    Rt.Position() = Rtwh;
    theta += 0.001;

  }
  
  void Cleanup(){}

};

int main(){

  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  osg::ref_ptr< osaOSGWorld > world = new osaOSGWorld;

  // Create a camera
  int x = 0, y = 0;
  int width = 640, height = 480;
  double Znear = 0.1, Zfar = 10.0;

  osg::Node::NodeMask maskleft  = 0x01;
  osg::Node::NodeMask maskright = 0x02;

  mtsOSGStereo* camera;
  camera = new mtsOSGStereo( "camera",
			     world,
			     x, y, 
			     width, height,
			     55.0, ((double)width)/((double)height),
			     Znear, Zfar, 
			     0.1 );
  camera->setCullMask( maskleft, osaOSGStereo::LEFT );
  camera->setCullMask( maskright, osaOSGStereo::RIGHT );
  taskManager->AddComponent( camera );

  // head up displays
  osg::ref_ptr< osaOSGHUD > hudleft;
  hudleft = new osaOSGHUD( world, width, height, camera, 0 );

  osg::ref_ptr< osaOSGHUD > hudright;
  hudright = new osaOSGHUD( world, width, height, camera, 1 );

  // create the hubble motion
  HubbleMotion hmotion;
  taskManager->AddComponent( &hmotion );

  // Create the objects
  std::string path( CISST_SOURCE_ROOT"/etc/cisstRobot/objects/" );

  vctFrame4x4<double> Rt(  vctMatrixRotation3<double>(),
			   vctFixedSizeVector<double,3>( 0.0, 0.0, 0.5 ) );

  osg::ref_ptr< osaOSGBody > background;
  background = new osaOSGBody( path+"background.3ds", world, 
  			       vctFrame4x4<double>() );

  osg::ref_ptr< mtsOSGBody > hubble;
  hubble = new mtsOSGBody( "hubble", path+"hst.3ds", world, Rt );
  taskManager->AddComponent( hubble.get() );

  // connect the motion to hubble
  taskManager->Connect( hubble->GetName(), "Input",
			hmotion.GetName(), "Output" );

  // start the components
  taskManager->CreateAll();
  taskManager->WaitForStateAll( mtsComponentState::READY );

  // HUD
  hudleft->Initialize();
  hudright->Initialize();

  taskManager->StartAll();
  taskManager->WaitForStateAll( mtsComponentState::ACTIVE );

  // Start the svl stuff
  svlInitialize();

  // Creating SVL objects
  svlStreamManager streamleft;
  //svlFilterSourceVideoFile sourceleft(1);
  svlFilterSourceVideoCapture sourceleft(1);
  svlOSGImage imageleft( 0, 0, width, height, hudleft );
  svlFilterImageWindow windowleft;

  //sourceleft.SetFilePath("xray.avi");
  sourceleft.DialogSetup();
  imageleft.setNodeMask( maskleft );

  streamleft.SetSourceFilter( &sourceleft );
  //sourceleft.GetOutput()->Connect( windowleft.GetInput() );
  sourceleft.GetOutput()->Connect( imageleft.GetInput() );
  
  svlStreamManager streamright; 
  //svlFilterSourceVideoFile sourceright(1);
  svlFilterSourceVideoCapture sourceright(1); 
  svlOSGImage imageright( 0, 0, width, height, hudright );
  svlFilterImageWindow windowright;

  //sourceright.SetFilePath("traffic.avi");
  sourceright.DialogSetup();
  imageright.setNodeMask( maskright );

  streamright.SetSourceFilter( &sourceright );
  //sourceright.GetOutput()->Connect( windowright.GetInput() );
  sourceright.GetOutput()->Connect( imageright.GetInput() );
  
  if( streamleft.Play() != SVL_OK ) std::cout <<"error"<<std::endl;
  if( streamright.Play() != SVL_OK ) std::cout <<"error"<<std::endl;


  std::cout << "ENTER to exit." << std::endl;
  cmnGetChar();
  cmnGetChar();

  streamleft.Release();
  streamright.Release();

  taskManager->KillAll();
  taskManager->Cleanup();

  return 0;

}
