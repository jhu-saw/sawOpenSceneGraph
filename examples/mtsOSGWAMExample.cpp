#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnPath.h>

#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/mtsOSGMono.h>
#include <sawOpenSceneGraph/mtsOSGManipulator.h>

class WAMMotion : public mtsTaskPeriodic {

private:

  mtsFunctionRead  GetPositions;
  mtsFunctionWrite SetPositions;

  vctDynamicVector<double> vctq;

public:

  WAMMotion() : mtsTaskPeriodic( "WAMMotion", 0.01, true ){

    vctq.SetSize(7);
    vctq.SetAll(0.0);

    mtsInterfaceRequired* input = AddInterfaceRequired( "Input" );
    mtsInterfaceRequired* output = AddInterfaceRequired( "Output" );

    input->AddFunction( "GetPositionJoint", GetPositions );
    output->AddFunction( "SetPositionJoint", SetPositions );

  }

  void Configure( const std::string& ){}
  void Startup(){}
  void Run(){
    ProcessQueuedCommands();

    prmPositionJointGet prmqin;
    GetPositions( prmqin );

    prmPositionJointSet prmqout;
    prmqout.SetSize( 7 );
    prmqout.Goal() = vctq;
    prmqout.SetValid( true );
    SetPositions( prmqout );

    for( size_t i=0; i<vctq.size(); i++ ) vctq[i] += 0.001;

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
  mtsOSGMono* worldcamera;
  worldcamera = new mtsOSGMono( "WorldCamera",
			        world,
				x, y, width, height,
				55.0, ((double)width)/((double)height),
				Znear, Zfar );
  worldcamera->Configure();
  taskManager->AddComponent( worldcamera );


  vctFrame4x4<double> Rt7cam( vctMatrixRotation3<double>( -1.0,  0.0,  0.0,
							  0.0,  1.0,  0.0,
							  0.0,  0.0, -1.0 ),
			      vctFixedSizeVector<double,3>(0.0, 0.0, 0.1) );
  mtsOSGMono* WAMcamera;
  WAMcamera = new mtsOSGMono( "WAMCamera",
			      world,
			      x, y, width, height,
			      55.0, ((double)width)/((double)height),
			      Znear, Zfar, false, Rt7cam );
  WAMcamera->Configure();
  taskManager->AddComponent( WAMcamera );



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

  mtsOSGManipulator* WAM;
  WAM = new mtsOSGManipulator( "WAM",
			       0.01,
			       OSA_CPU1,
			       20,
			       models,
			       world,
			       Rtw0,
			       path.Find("wam7.rob"),
			       path.Find("l0.obj") );
  taskManager->AddComponent( WAM );

  WAMMotion motion;
  taskManager->AddComponent( &motion );

  taskManager->Connect( motion.GetName(), "Input",  WAM->GetName(), "Output" );
  taskManager->Connect( motion.GetName(), "Output", WAM->GetName(), "Input" );
  taskManager->Connect( WAM->GetName(),    "Output",
			WAMcamera->GetName(), "Input" );

  taskManager->CreateAll();
  taskManager->WaitForStateAll( mtsComponentState::READY );

  taskManager->StartAll();
  taskManager->WaitForStateAll( mtsComponentState::ACTIVE );

  cmnGetChar();
  cmnGetChar();

  taskManager->KillAll();
  taskManager->Cleanup();

  return 0;

}
