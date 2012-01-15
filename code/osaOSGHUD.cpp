#include <sawOpenSceneGraph/osaOSGHUD.h>
#include <cisstOSAbstraction/osaSleep.h>

osaOSGHUD::osaOSGHUD( osaOSGWorld* world,
		      int width, int height,
		      osaOSGCamera* camera ):
  world( world ),
  width( width ),
  height( height ),
  initialized( false ){
  
  if( world != NULL && camera != NULL ){
    Initialize( (osg::Node*)world, 
		width, height, 
		(osgViewer::Viewer*)camera );
    viewer = camera;
  }

}

osaOSGHUD::osaOSGHUD( osaOSGWorld* world,
		      int width, int height,
		      mtsOSGCameraTask* camera ):
  world( world ),
  width( width ),
  height( height ),
  initialized( false ){
  
  if( world != NULL && camera != NULL ){
    Initialize( (osg::Node*)world, 
		width, height, 
		(osgViewer::Viewer*)camera->camera);
    viewer = camera->camera;
  }
  
}

void osaOSGHUD::Initialize( osg::Node* world, 
			    int width, int height, 
			    osgViewer::Viewer* viewer ){

  // Set the intrinsic paramters
  setProjectionMatrixAsOrtho2D( 0, width, 0, height );
  setViewMatrix( osg::Matrix::identity() );
  setReferenceFrame( osg::Camera::ABSOLUTE_RF );
  setRenderOrder( osg::Camera::NESTED_RENDER );
  setClearMask( GL_DEPTH_BUFFER_BIT );
  
  getOrCreateStateSet()->setMode( GL_LIGHTING, GL_FALSE );
  getOrCreateStateSet()->setMode( GL_DEPTH_TEST, GL_FALSE );
  
  // don't capture events
  setAllowEventFocus(false);
  
  // render this camera in the same viewport as the other camera
  osgViewer::Viewer::Windows windows;
  viewer->getWindows(windows);
  
  if( !windows.empty() ){
    
    setGraphicsContext( windows[0] );
    setViewport( windows[0]->getTraits()->x,
		 windows[0]->getTraits()->y,
		 windows[0]->getTraits()->width, 
		 windows[0]->getTraits()->height );
    
    // now create a group and add the new camera as a child and the old world
    // as the other child
    osg::ref_ptr<osg::Group> group = new osg::Group;
    osg::Node* one = this;
    osg::Node* two = world;
    one->getOrCreateStateSet()->setRenderBinDetails(1,"RenderBin");
    two->getOrCreateStateSet()->setRenderBinDetails(2,"RenderBin");
    group->addChild(one);
    group->addChild(two);
    group->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
    two->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::ON);
    viewer->setSceneData( group.get() );
    initialized = true;
  }
  else{ 
    std::cout << "Invalid context"<<std::endl;
  }
}

osaOSGHUD::~osaOSGHUD(){}

