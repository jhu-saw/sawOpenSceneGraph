#include <cisstCommon/cmnGetChar.h>

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

  vctDynamicVector<double> q;

public:

  WAMMotion() : mtsTaskPeriodic( "WAMMotion", 0.01, true ){

    q.SetSize(7);
    q.SetAll(0.0);

    mtsInterfaceRequired* input = AddInterfaceRequired( "Input" );
    mtsInterfaceRequired* output = AddInterfaceRequired( "Output" );

    input->AddFunction( "GetPositionJoint", GetPositions );
    output->AddFunction( "SetPositionJoint", SetPositions );

  }

  void Configure( const std::string& ){}
  void Startup(){}
  void Run(){
    ProcessQueuedCommands();
    
    prmPositionJointGet qin;
    GetPositions( qin );
    
    prmPositionJointSet qout;
    qout.SetSize( 7 );
    qout.Goal() = q;
    SetPositions( qout );
    
    for( size_t i=0; i<q.size(); i++ ) q[i] += 0.001;

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
			      vctFixedSizeVector<double,3>(0.0) );
  mtsOSGMono* WAMcamera;
  WAMcamera = new mtsOSGMono( "WAMCamera",
			      world,
			      x, y, width, height,
			      55.0, ((double)width)/((double)height),
			      Znear, Zfar, false, Rt7cam );
  WAMcamera->Configure();
  taskManager->AddComponent( WAMcamera );
  


  std::string path( CISST_SOURCE_ROOT"/cisst/etc/cisstRobot/WAM/" );
  vctFrame4x4<double> Rtw0;
  
  std::vector< std::string > models;
  models.push_back( path + "l1.obj" );
  models.push_back( path + "l2.obj" );
  models.push_back( path + "l3.obj" );
  models.push_back( path + "l4.obj" );
  models.push_back( path + "l5.obj" );
  models.push_back( path + "l6.obj" );
  models.push_back( path + "l7.obj" );

  mtsOSGManipulator* WAM;
  WAM = new mtsOSGManipulator( "WAM",
			       0.01,
			       OSA_CPU1,
			       20,
			       models,
			       world,
			       Rtw0,
			       path + "wam7.rob",
			       path + "l0.obj" );
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

  taskManager->KillAll();
  taskManager->Cleanup();

  return 0;

}
