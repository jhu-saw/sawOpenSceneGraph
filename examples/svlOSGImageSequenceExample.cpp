#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnPath.h>

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsMatrix.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/mtsOSGStereo.h>
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
    vctFrm3 Rtwh;
    Rtwh.Rotation().FromRaw( Rwh );
    Rtwh.Translation().Assign( vctFixedSizeVector<double,3>( 0.0, 0.0, 0.5 ) );

    Rt.Position() = Rtwh;
    theta += 0.001;

  }

  void Cleanup(){}

};

int main( ){

  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  // Create the OSG world
  osg::ref_ptr< osaOSGWorld > world = new osaOSGWorld;

  // Create OSG camera
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

  // create the hubble motion
  HubbleMotion hmotion;
  taskManager->AddComponent( &hmotion );


  // create hubble
  cmnPath path;
  path.AddRelativeToCisstShare("/models/hubble");
  path.AddRelativeToCisstShare("/movies");

  vctFrame4x4<double> Rt( vctMatrixRotation3<double>(),
			  vctFixedSizeVector<double,3>( 0.0, 0.0, 0.5 ) );
  mtsOSGBody* hubble;
  hubble = new mtsOSGBody( "hubble", path.Find("hst.3ds"), world, Rt, 1.0, .5 );
  taskManager->AddComponent( hubble );


  // connect the motion to hubble
  taskManager->Connect( hubble->GetName(), "Input",
  			hmotion.GetName(), "Output" );


  // start the components
  taskManager->CreateAll();
  taskManager->WaitForStateAll( mtsComponentState::READY );
  taskManager->StartAll();
  taskManager->WaitForStateAll( mtsComponentState::ACTIVE );


  // Start the svl stuff
  svlInitialize();

  // Creating SVL objects
  svlStreamManager streamleft;
  svlFilterSourceVideoFile sourceleft(1);
  svlOSGImage imageleft( -0.5, -0.5, 1, 1, world );

  // Configure the filters
  sourceleft.SetFilePath( path.Find( "left.mpg" ) );
  imageleft.setNodeMask( maskleft );

  streamleft.SetSourceFilter( &sourceleft );
  sourceleft.GetOutput()->Connect( imageleft.GetInput() );

  svlStreamManager streamright;
  svlFilterSourceVideoFile sourceright(1);
  svlOSGImage imageright( -0.5, -0.5, 1, 1, world );

  sourceright.SetFilePath( path.Find( "right.mpg" ) );
  imageright.setNodeMask( maskright );

  streamright.SetSourceFilter( &sourceright );
  sourceright.GetOutput()->Connect( imageright.GetInput() );

  // start the streams
  if (streamleft.Play() != SVL_OK)
    { std::cerr << "Cannot start left stream." <<std::endl; }

  if (streamright.Play() != SVL_OK)
    { std::cerr << "Cannot start right stream." <<std::endl; }

  std::cout << "ENTER to exit." << std::endl;
  cmnGetChar();
  cmnGetChar();

  streamleft.Release();
  streamright.Release();

  taskManager->KillAll();
  taskManager->Cleanup();

  return 0;

}
