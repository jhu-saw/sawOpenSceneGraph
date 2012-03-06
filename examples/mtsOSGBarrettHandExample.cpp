#include <cisstCommon/cmnPath.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/osaOSGMono.h>
#include <sawOpenSceneGraph/mtsOSGBH.h>

class BHMotion : public mtsTaskPeriodic {

private:

  mtsFunctionRead  GetPositions;
  mtsFunctionWrite SetPositions;

  vctDynamicVector<double> vctq;

public:

  BHMotion() : mtsTaskPeriodic( "BHMotion", 0.01, true ){

    vctq.SetSize(4);
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
    prmqout.SetSize( 4 );
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
  int width = 320, height = 240;
  double Znear = 0.1, Zfar = 10.0;
  osg::ref_ptr< osaOSGCamera > camera;
  camera = new osaOSGMono( world,
			     x, y, width, height,
			     55.0, ((double)width)/((double)height),
			     Znear, Zfar );
  camera->Initialize();

  cmnPath path;
  path.AddRelativeToCisstShare("cisstRobot/BH");
  vctFrame4x4<double> Rtw0;

  mtsOSGBH* BH;
  BH = new mtsOSGBH( "BH",
		     0.01,
		     OSA_CPU1,
		     20,
		     path.Find("l0.obj"),
		     path.Find("l1.obj"),
		     path.Find("l2.obj"),
		     path.Find("l3.obj"),
		     world,
		     Rtw0,
		     path.Find("f1f2.rob"),
		     path.Find("f3.rob") );
  taskManager->AddComponent( BH );

  BHMotion motion;
  taskManager->AddComponent( &motion );

  taskManager->Connect( motion.GetName(), "Input",  BH->GetName(), "Output" );
  taskManager->Connect( motion.GetName(), "Output", BH->GetName(), "Input" );

  taskManager->CreateAll();
  taskManager->WaitForStateAll( mtsComponentState::READY );

  taskManager->StartAll();
  taskManager->WaitForStateAll( mtsComponentState::ACTIVE );

  std::cout << "ESC to quit" << std::endl;
  while( !camera->done() )
    { camera->frame(); }

  taskManager->KillAll();
  taskManager->Cleanup();

  return 0;

}
