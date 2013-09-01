#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnPath.h>

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
    vctFrm3 Rtwh;
    Rtwh.Rotation().FromRaw( Rwh );
   Rtwh.Translation().Assign( vctFixedSizeVector<double,3>( 0.0, 0.0, 0.5 ) );

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

  vctFixedSizeMatrix<double,3,3> Kl( 526.0554,      0.0, 313.2596,
				          0.0, 525.0756, 233.6962,
				          0.0,      0.0,   1.0 );
  vctFixedSizeMatrix<double,3,3> Kr( 534.6877,      0.0, 327.7359,
				          0.0, 533.5979, 249.4243,
				          0.0,      0.0,   1.0 );
  vctMatrixRotation3<double> Rlr( 0.9887, -0.0126, -0.1497,
				  0.0085,  0.9996, -0.0277,
				  0.1500,  0.0261,  0.9883,
				  VCT_NORMALIZE );
  vctFixedSizeVector<double,3> tlr( -0.0410, 0.0000, -0.0042 );
  vctFrame4x4<double> Rtlr( Rlr, tlr );
  mtsOSGStereo* camera;
  camera = new mtsOSGStereo( "camera",
			     world,
			     x, y,
			     width, height,
			     Kl, Kr, Rtlr,
			     Znear, Zfar,
			     true );
  camera->setCullMask( maskleft, osaOSGStereo::LEFT );
  camera->setCullMask( maskright, osaOSGStereo::RIGHT );
  taskManager->AddComponent( camera );

  // head up displays
  osg::ref_ptr< osaOSGHUD > hudleft = new osaOSGHUD( world, camera );
  osg::ref_ptr< osaOSGHUD > hudright = new osaOSGHUD( world, camera );

  // create the hubble motion
  HubbleMotion hmotion;
  taskManager->AddComponent( &hmotion );

  // Create the objects
  cmnPath path;
  path.AddRelativeToCisstShare("/models/hubble");
  path.AddRelativeToCisstShare("/movies");

  vctFrame4x4<double> Rt( vctMatrixRotation3<double>(),
			  vctFixedSizeVector<double,3>( 0.0, 0.0, 0.5 ) );

  mtsOSGBody* hubble;
  hubble = new mtsOSGBody( "hubble", path.Find("hst.3ds"), world, Rt, 0.1, 0.6);
  taskManager->AddComponent( hubble );

  // connect the motion to hubble
  taskManager->Connect( hubble->GetName(), "Input",
  			hmotion.GetName(), "Output" );

  // start the components
  taskManager->CreateAll();
  taskManager->WaitForStateAll( mtsComponentState::READY );

  taskManager->StartAll();
  taskManager->WaitForStateAll( mtsComponentState::ACTIVE );

  //cmnGetChar();

  // Start the svl stuff
  svlInitialize();

  // Creating SVL objects
  svlStreamManager streamleft;
  svlFilterSourceVideoFile sourceleft(1);
  svlFilterImageFlipRotate flipleft;
  svlFilterImageRectifier  rectifierleft;
  svlOSGImage imageleft( 0, 0, width, height, hudleft );

  sourceleft.SetFilePath( path.Find( "left.mpg" ) );
  flipleft.SetVerticalFlip( true );
  {
    vct3x3 R( 1.0, 0.0, 0.0,
	      0.0, 1.0, 0.0,
	      0.0, 0.0, 1.0 );
    vct2 f( 526.0554,  525.0756 );
    vct2 c( 313.2596,  233.6962 );
    vctFixedSizeVector<double,7> k(    -0.3438,
				       0.1027,
				       0.0013,
				       0.0002,
				       0.0,
				       0.0,
				       0.0 );
    double alpha = 0.0;
    rectifierleft.SetTableFromCameraCalibration( 480, 640,
						 R,
						 f,
						 c,
						 k,
						 alpha,
						 0 );
  }
  imageleft.setNodeMask( maskleft );

  streamleft.SetSourceFilter( &sourceleft );
  sourceleft.GetOutput()->Connect( flipleft.GetInput() );
  flipleft.GetOutput()->Connect( imageleft.GetInput() );
  
  svlStreamManager streamright; 
  svlFilterSourceVideoFile sourceright(1);

  svlFilterImageFlipRotate flipright;
  svlFilterImageRectifier  rectifierright;
  svlOSGImage imageright( 0, 0, width, height, hudright );

  sourceright.SetFilePath( path.Find( "right.mpg" ) );
  flipright.SetVerticalFlip( true );
  {
    vct3x3 R( 1.0, 0.0, 0.0,
	      0.0, 1.0, 0.0,
	      0.0, 0.0, 1.0 );
    vct2 f( 534.6877,  533.5979 );
    vct2 c( 327.7359,  249.4243 );
    vctFixedSizeVector<double,7> k(-0.3618,
				    0.1450,
				    0.0013,
				    0.0002,
				    0.0,
				    0.0,
				    0.0 );
    double alpha = 0.0;
    rectifierright.SetTableFromCameraCalibration( 480, 640,
						  R,
						  f,
						  c,
						  k,
						  alpha,
						  0 );
  }

  imageright.setNodeMask( maskright );

  streamright.SetSourceFilter( &sourceright );
  sourceright.GetOutput()->Connect( flipright.GetInput() );
  flipright.GetOutput()->Connect( imageright.GetInput() );

  if( streamleft.Play() != SVL_OK )
    { std::cerr << "Cannot start left stream." <<std::endl; }

  if( streamright.Play() != SVL_OK )
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
