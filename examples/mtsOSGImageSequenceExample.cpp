#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnGetChar.h>

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsMatrix.h>

#include <cisstOSAbstraction/osaSleep.h>
#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/mtsOSGMono.h>
#include <sawOpenSceneGraph/mtsOSGImage.h>
#include <sawOpenSceneGraph/osaOSGBody.h>
#include <osgDB/ReadFile>

class ImageServer : public mtsTaskPeriodic {
private:

  mtsUCharMat mtsimg;  // Mx(Nx3)
  int i;               // image counter
  cmnPath path;

public:

  ImageServer( const std::string& name ) :
    mtsTaskPeriodic( name, 1.0/30.0, true ){

    path.AddRelativeToCisstShare("/images/left");

    mtsInterfaceProvided* output = AddInterfaceProvided( "Output" );
    StateTable.AddData( mtsimg, "Image" );
    output->AddCommandReadState( StateTable, mtsimg, "GetImage" );

    i = 0;

  }

  void Configure( const std::string& ){}
  void Startup(){}
  void Run(){

    ProcessQueuedCommands();

    // load the osg image
    char buffer[128];
    sprintf( buffer, "left%07d.jpg", i++ );
    osg::ref_ptr<osg::Image> osgimg;
    osgimg = osgDB::readImageFile( path.Find( buffer ) );

    // copy the osg image to mtsMatrix
    mtsimg.SetSize( osgimg->t(), osgimg->s()*3 );
    memcpy( mtsimg.Pointer(), osgimg->data(), osgimg->getTotalSizeInBytes() );

    // validate the data
    bool valid = true;
    mtsimg.SetValid( valid );

    if( i == 49 ) i=0;

  }
  
  void Cleanup(){}

};

int main( ){

  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  osg::ref_ptr< osaOSGWorld > world = new osaOSGWorld;
  
  // Create a camera
  int x = 0, y = 0;
  int width = 640, height = 480;
  double Znear = 0.1, Zfar = 10.0;
  mtsOSGMono* camera;
  camera = new mtsOSGMono( "camera", 
			   world,
			   x, y, width, height,
			   55.0, ((double)width)/((double)height),
			   Znear, Zfar );
  taskManager->AddComponent( camera );

  mtsOSGImage* client = new mtsOSGImage( "client", -.5, -.5, 1.0, 1.0, world );
  taskManager->AddComponent( client );

  ImageServer* server = new ImageServer( "server" );
  taskManager->AddComponent( server );
  
  taskManager->Connect( server->GetName(), "Output", 
			client->GetName(), "Input" );

  taskManager->CreateAll();
  taskManager->WaitForStateAll( mtsComponentState::READY );

  taskManager->StartAll();
  taskManager->WaitForStateAll( mtsComponentState::ACTIVE );

  std::cout << "ENTER to exit" << std::endl;
  cmnGetChar();
  cmnGetChar();

  taskManager->KillAll();
  taskManager->Cleanup();

  return 0;

}
