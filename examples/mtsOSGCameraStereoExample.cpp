#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnPath.h>

#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/mtsOSGStereo.h>
#include <sawOpenSceneGraph/mtsOSGBody.h>

// Class to move the camera
class CameraMotion : public mtsTaskPeriodic {

private:

  // The position that the camera will be fetching
  prmPositionCartesianGet prmRt;

public:

  CameraMotion() : mtsTaskPeriodic( "CameraMotion", 0.01, true ){
    prmRt.Position().Translation()[2] = 1;

    StateTable.AddData( prmRt, "PositionOrientation" );

    // provide the camera position
    mtsInterfaceProvided* output = AddInterfaceProvided( "Output" );
    output->AddCommandReadState( StateTable, prmRt, "GetPositionCartesian" );

  }

  void Configure( const std::string& ){}
  void Startup(){}
  void Run(){
    ProcessQueuedCommands();
    prmRt.Position().Translation()[2] += 0.001;
    prmRt.SetValid( true );
  }

  void Cleanup(){}

};

// Camera to move an object
class HubbleMotion : public mtsTaskPeriodic {

private:

  // The position that the camera will be fetching
  prmPositionCartesianGet prmRt;
  double theta;

public:

  HubbleMotion() : mtsTaskPeriodic( "HubbleMotion", 0.01, true ){
    theta = 0;
    prmRt.Position().Translation()[2] = 0.5;

    StateTable.AddData( prmRt, "PositionOrientation" );

    // provide the camera position
    mtsInterfaceProvided* output = AddInterfaceProvided( "Output" );
    output->AddCommandReadState( StateTable, prmRt, "GetPositionCartesian" );

  }

  void Configure( const std::string& ){}
  void Startup(){}
  void Run(){
    ProcessQueuedCommands();

    // rotate hubble
    vctFixedSizeVector<double,3> u( 0.0, 0.0, 1.0 );
    vctAxisAngleRotation3<double> Rwh( u, theta );
    vctFrm3 Rtwh;
    Rtwh.Rotation().FromRaw( Rwh );
    Rtwh.Translation().Assign( vctFixedSizeVector<double,3>( 0.0, 0.0, 0.5 ) );

    prmRt.Position() = Rtwh;
    prmRt.SetValid( true );
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
  int width = 320, height = 240;
  double Znear = 0.1, Zfar = 10.0;
  mtsOSGStereo* camera;
  camera = new mtsOSGStereo( "camera",
			     world,
			     x, y,
			     width, height,
			     55.0, ((double)width)/((double)height),
			     Znear, Zfar,
			     0.1,
			     false );
  taskManager->AddComponent( camera );

  // create the camera motion
  CameraMotion cmotion;
  taskManager->AddComponent( &cmotion );

  // create the hubble motion
  HubbleMotion hmotion;
  taskManager->AddComponent( &hmotion );

  cmnPath path;
  path.AddRelativeToCisstShare("/models");
  path.AddRelativeToCisstShare("/models/hubble");

  vctFrame4x4<double> Rt(  vctMatrixRotation3<double>(),
			   vctFixedSizeVector<double,3>( 0.0, 0.0, 0.5 ) );
  mtsOSGBody* hubble;
  hubble = new mtsOSGBody( "hubble", path.Find("hst.3ds"), world, Rt, 0.8 );
  taskManager->AddComponent( hubble );

  osg::ref_ptr< osaOSGBody > background;
  background = new osaOSGBody( path.Find("background.3ds"), world,
			       vctFrame4x4<double>() );

  taskManager->Connect( camera->GetName(), "Input",
  			cmotion.GetName(), "Output" );

  taskManager->Connect( hubble->GetName(), "Input",
			hmotion.GetName(), "Output" );

  taskManager->CreateAll();
  taskManager->WaitForStateAll( mtsComponentState::READY );

  taskManager->StartAll();
  taskManager->WaitForStateAll( mtsComponentState::ACTIVE );

  cmnGetChar();
  std::cout << "ENTER to quit" << std::endl;
  cmnGetChar();

  taskManager->KillAll();
  taskManager->Cleanup();

  return 0;

}
