#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsMatrix.h>

#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/osaOSGMono.h>
#include <sawOpenSceneGraph/osaOSGBody.h>
#include <sawOpenSceneGraph/svlOSGImage.h>
#include <osgDB/ReadFile>

#include <cisstStereoVision.h>

class ImageServer : public mtsTaskPeriodic {
private:

  mtsUCharMat mtsimg;  // Mx(Nx3)
  int i;               // image counter

public:

  ImageServer( const std::string& name ) :
    mtsTaskPeriodic( name, 1.0/30.0, true ){

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
    char filename[128];
    sprintf( filename, "walkstraight/frame%04d.tif", i++ );
    osg::ref_ptr<osg::Image> osgimg;
    osgimg = osgDB::readImageFile( std::string( filename ) );

    // copy the osg image to mtsMatrix
    mtsimg.SetSize( osgimg->t(), osgimg->s()*3 );
    memcpy( mtsimg.Pointer(), osgimg->data(), osgimg->getTotalSizeInBytes() );

    // validate the data
    bool valid = true;
    mtsimg.SetValid( valid );

    if( i == 125 ) i=0;

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
  osg::ref_ptr< osaOSGMono > camera;
  camera = new osaOSGMono( world,
			   x, y, width, height,
			   55.0, ((double)width)/((double)height),
			   Znear, Zfar );
  camera->Initialize();

  // Create the objects
  std::string path( CISST_SOURCE_ROOT"/etc/cisstRobot/objects/" );

  vctFrame4x4<double> Rt(  vctMatrixRotation3<double>(),
			   vctFixedSizeVector<double,3>( 0.0, 0.0, 0.5 ) );
  osg::ref_ptr< osaOSGBody > hubble;
  hubble = new osaOSGBody( path+"hst.3ds", world, Rt );


  svlInitialize();

  // Creating SVL objects
  svlStreamManager stream;
  svlOSGImage imageseq( -0.5, -0.5, 1, 1, world );

  svlFilterSourceVideoFile source(1);
  source.SetFilePath( "xray.avi" );


  stream.SetSourceFilter( &source );
  source.GetOutput()->Connect( imageseq.GetInput() );

  if (stream.Play() != SVL_OK)
    std::cout <<"error"<<std::endl;

  taskManager->CreateAll();
  taskManager->StartAll();

  while( 1 )
    { camera->frame(); }

  pause();

  return 0;

}
