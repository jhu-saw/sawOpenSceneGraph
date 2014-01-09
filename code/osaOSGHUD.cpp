#include <sawOpenSceneGraph/osaOSGHUD.h>
#include <cisstOSAbstraction/osaSleep.h>

osaOSGHUD::osaOSGHUD( osaOSGWorld* world,
		      osaOSGCamera* camera ) :
  root( world ),
  viewer( camera ){

  if( world != NULL && camera != NULL ){ 
    if( camera->getCamera()->getGraphicsContext() )
      { Initialize( camera->getCamera()->getGraphicsContext() ); }
    else
      { camera->AddHUD( this ); }
  }

}

osaOSGHUD::osaOSGHUD( osaOSGWorld* world,
		      mtsOSGCameraTask* camera ):
  root( world ),
  viewer( camera->camera ){

  if( world != NULL && camera != NULL ){ 
    if( camera->camera->getCamera()->getGraphicsContext() )
      { Initialize( camera->camera->getCamera()->getGraphicsContext() ); }
    else
      { camera->camera->AddHUD( this ); }
  }

}

void osaOSGHUD::Initialize( osg::GraphicsContext* gc ){

  // Set the intrinsic paramters
  setProjectionMatrixAsOrtho2D( 0, gc->getTraits()->width, 
				0, gc->getTraits()->height );
  setViewMatrix( osg::Matrix::identity() );
  setReferenceFrame( osg::Camera::ABSOLUTE_RF );
  setRenderOrder( osg::Camera::NESTED_RENDER );
  setClearMask( GL_DEPTH_BUFFER_BIT );
  
  getOrCreateStateSet()->setMode( GL_LIGHTING, GL_FALSE );
  getOrCreateStateSet()->setMode( GL_DEPTH_TEST, GL_FALSE );

  // don't capture events
  setAllowEventFocus(false);

  setGraphicsContext( gc );
  setViewport( gc->getTraits()->x,
	       gc->getTraits()->y,
	       gc->getTraits()->width, 
	       gc->getTraits()->height );

  // search for the HUD branch
  bool found = false;
  for( unsigned int i=0; i<root->getNumParents(); i++ ){
    
    osg::ref_ptr<osg::Node> parent = root->getParent( i );
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
    root->getOrCreateStateSet()->setRenderBinDetails( 2, "RenderBin" );
    root->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::ON);
    
    
    osg::ref_ptr<osg::Group> group = new osg::Group;
    group->addChild( hud );
    group->addChild( root );
    
    viewer->setSceneData( group.get() );
    
  }
  
}

osaOSGHUD::~osaOSGHUD(){}

