#include <sawOpenSceneGraph/osaOSGHUD.h>
#include <cisstOSAbstraction/osaSleep.h>

osaOSGHUD::osaOSGHUD( osaOSGWorld* world,
		      int width, int height,
		      osaOSGCamera* camera,
		      int window ) :
  root( world ),
  viewer( camera ),
  width( width ),
  height( height ),
  window( window ),
  initialized( false ){

  if( world != NULL && camera != NULL )
    { Initialize( root, width, height, viewer, window ); }

}

osaOSGHUD::osaOSGHUD( osg::Node* node,
		      int width, int height,
		      osaOSGCamera* camera,
		      int window ) :
  root( node ),
  viewer( camera ),
  width( width ),
  height( height ),
  window( window ),
  initialized( false ){
  
  if( root != NULL && camera != NULL )
    { Initialize( root, width, height, viewer, window ); }

}

osaOSGHUD::osaOSGHUD( osaOSGWorld* world,
		      int width, int height,
		      mtsOSGCameraTask* camera,
		      int window ) :
  root( world ),
  viewer( camera->camera ),
  width( width ),
  height( height ),
  window( window ),
  initialized( false ){

  if( world != NULL && camera != NULL )
    { Initialize( root, width, height, viewer, window ); }
  
}

osaOSGHUD::osaOSGHUD( osg::Node* node,
		      int width, int height,
		      mtsOSGCameraTask* camera,
		      int window ) :
  root( node ),
  viewer( camera->camera ),
  width( width ),
  height( height ),
  window( window ),
  initialized( false ){

  if( root != NULL && camera != NULL )
    { Initialize( root, width, height, viewer, window ); }

}



void osaOSGHUD::Initialize( osg::Node* world, 
			    int width, int height, 
			    osgViewer::Viewer* viewer,
			    size_t window ){

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

  if( window < windows.size() ){
    
    setGraphicsContext( windows[window] );
    setViewport( windows[window]->getTraits()->x,
		 windows[window]->getTraits()->y,
		 windows[window]->getTraits()->width, 
		 windows[window]->getTraits()->height );

    // search for the HUD branch
    bool found = false;
    for( unsigned int i=0; i<world->getNumParents(); i++ ){

      osg::ref_ptr<osg::Node> parent = world->getParent( i );
      osg::ref_ptr<osg::Group> group = parent->asGroup();
      if( group != 0 ){
	for( unsigned int j=0; j<group->getNumChildren(); j++ ){
	  osg::ref_ptr<osg::Node> child = group->getChild( j );
	  if( child->getName() == "HUD" ){ 
	    found = true;
	    osg::ref_ptr<osg::Group> hud = child->asGroup();
	    hud->addChild( this );
	  }
	}
      }
    }

    if( !found ){

      // create/configure the hud branch
      osg::ref_ptr<osg::Group> hud = new osg::Group;
      hud->setName( "HUD" );
      hud->getOrCreateStateSet()->setRenderBinDetails( 1, "RenderBin" );
      hud->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
      
      // add this camera to the hud branch
      hud->addChild( this );
      
      // configure the world branch
      world->getOrCreateStateSet()->setRenderBinDetails( 2, "RenderBin" );
      world->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::ON);

      
      osg::ref_ptr<osg::Group> group = new osg::Group;
      group->addChild( hud );
      group->addChild( world );
      
      viewer->setSceneData( group.get() );

    }

    initialized = true;

  }

}

osaOSGHUD::~osaOSGHUD(){}

