#include <sawOpenSceneGraph/osaOSGPick.h>

#include <cisstCommon/cmnLogger.h>

osaOSGPick::osaOSGPick() : osgGA::GUIEventHandler(), mousex(0), mousey(0){}


bool osaOSGPick::handle( const osgGA::GUIEventAdapter& ea, 
			 osgGA::GUIActionAdapter& aa ){

  osg::ref_ptr< osgViewer::Viewer > viewer;
  viewer = dynamic_cast<osgViewer::Viewer*>( &aa );

  switch( ea.getEventType() ){

  case osgGA::GUIEventAdapter::PUSH:
    Pick( PUSH, ea.getXnormalized(), ea.getYnormalized(), viewer );
    break;
      
  case osgGA::GUIEventAdapter::DRAG:
    if( mousex != ea.getXnormalized() || mousey != ea.getYnormalized() )
      { PickHandler( NULL, DRAG, ea.getXnormalized(), ea.getYnormalized() ); }
    break;

  case osgGA::GUIEventAdapter::RELEASE:
    PickHandler( NULL, RELEASE, ea.getXnormalized(), ea.getYnormalized() );
    break;

  default:
    break;
  }

  mousex = ea.getXnormalized();
  mousey = ea.getYnormalized();

  return true;
}



// Perform a pick operation.
bool osaOSGPick::Pick( osaOSGPick::Event event, 
		       double x, 
		       double y, 
		       osgViewer::Viewer* viewer ){

  // Nothing to pick.
  if( !viewer->getSceneData() )
    { return false; }

  double w( .05 ), h( .05 ); // tolerance

  // create a polytope picker
  osg::ref_ptr< osgUtil::PolytopeIntersector> picker;
  picker = new osgUtil::PolytopeIntersector( osgUtil::Intersector::PROJECTION,
					     x-w, y-h, x+w, y+h );
  osgUtil::IntersectionVisitor iv( picker );
  viewer->getCamera()->accept( iv );

  // something was picked
  if( picker->containsIntersections() ){

    // get the first intersection
    osgUtil::PolytopeIntersector::Intersections& 
      intersections = picker->getIntersections();
    osgUtil::PolytopeIntersector::Intersections::const_iterator 
      intersection = intersections.begin();

    // get the node path
    const osg::NodePath& nodePath = intersection->nodePath;
    unsigned int idx = nodePath.size();
    
    // go up the path searching for a body
    while(idx--){
      osg::ref_ptr<osaOSGBody> body;
      body = dynamic_cast< osaOSGBody* >( nodePath[ idx ] );
      
      if( body.get() != NULL ){
        PickHandler( body.get(), event, x, y );
        return true;
      }
    }
    
  }

  return false;
}
