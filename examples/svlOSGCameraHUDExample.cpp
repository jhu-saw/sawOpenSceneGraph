#include <cisstCommon/cmnGetChar.h>

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsMatrix.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/mtsOSGMono.h>
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

int main( int, char** argv ){

  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  osg::ref_ptr< osaOSGWorld > world = new osaOSGWorld;
  
  // Create a camera
  int x = 0, y = 0;
  int width = 640, height = 480;
  double Znear = 0.1, Zfar = 10.0;

  osaOSGMono* cameraleft;
  osg::Node::NodeMask maskleft = 0x01;
  cameraleft = new osaOSGMono( world,
			       x, y, 
			       width, height,
			       55.0, ((double)width)/((double)height),
			       Znear, Zfar );
  cameraleft->Initialize();
  cameraleft->setCullMask( maskleft );

  
  osaOSGMono* cameraright;
  osg::Node::NodeMask maskright = 0x02;
  cameraright = new osaOSGMono( world,
				x, y, 
				width, height,
				55.0, ((double)width)/((double)height),
				Znear, Zfar );
  cameraright->Initialize();
  cameraright->setCullMask( maskright );
  

  // HUD
  osg::ref_ptr< osaOSGHUD > hudleft;
  hudleft = new osaOSGHUD( world, width, height, cameraleft );

  osg::ref_ptr< osaOSGHUD > hudright;
  hudright = new osaOSGHUD( world, width, height, cameraright );


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


  // Start the svl stuff
  svlInitialize();

  // Creating SVL objects
  svlStreamManager streamleft;
  svlFilterSourceVideoCapture sourceleft(1);
  svlOSGImage imageleft( 0, 0, width, height, hudleft );

  
  svlStreamManager streamright; 
  svlFilterSourceVideoCapture sourceright(1); 
  svlOSGImage imageright( 0, 0, width, height, hudright );
  

  sourceleft.DialogSetup();
  imageleft.setNodeMask( maskleft );

  streamleft.SetSourceFilter( &sourceleft );
  sourceleft.GetOutput()->Connect( imageleft.GetInput() );
  
  
  sourceright.DialogSetup();
  imageright.setNodeMask( maskright );

  streamright.SetSourceFilter( &sourceright );
  sourceright.GetOutput()->Connect( imageright.GetInput() );
  

  if (streamleft.Play() != SVL_OK)
    std::cout <<"error"<<std::endl;

  
  if (streamright.Play() != SVL_OK)
    std::cout <<"error"<<std::endl;
  

  taskManager->CreateAll();
  taskManager->WaitForStateAll( mtsComponentState::READY );
  
  taskManager->StartAll();
  taskManager->WaitForStateAll( mtsComponentState::ACTIVE );

  while(1){

    cameraleft->frame();
    cameraright->frame();

  }
  
  cmnGetChar();
  cmnGetChar();

  return 0;

}
