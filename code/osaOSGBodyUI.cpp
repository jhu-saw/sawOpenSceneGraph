#include <sawOpenSceneGraph/osaOSGBodyUI.h>
#include <cisstCommon/cmnPath.h>
#include <osg/ComputeBoundsVisitor>
#include <osgUtil/PolytopeIntersector>

// This is to set the color of a handle
void osaOSGHandle::ResetCallback::operator()( osg::Node* node, 
					      osg::NodeVisitor* nv ){

  osg::Referenced* data = node->getUserData();
  osaOSGBody::UserData* userdata;
  userdata = dynamic_cast<osaOSGBody::UserData*>( data );

  // change the transform 
  if( userdata != NULL ){
    osaOSGHandle* handle = dynamic_cast<osaOSGHandle*>(userdata->GetBody());
    if( handle != NULL ){ handle->Reset(); }
  }
  
  traverse( node, nv );
  
}   

osaOSGHandle::osaOSGHandle( osaOSGBody* body, 
			    const std::string& name,
			    const vctFixedSizeVector<double,3>& toh ) : 
  osaOSGBody( body,
	      vctFrame4x4<double>( vctMatrixRotation3<double>(), toh) ),
  state( FREE ),
  body( body ){

  setName( "handle" + name );

  geode = new osg::Geode();
  sphere = new osg::Sphere( osg::Vec3( 0.0, 0.0, 0.0 ), 0.01 );
  drawable = new osg::ShapeDrawable( sphere );
  geode->addDrawable( drawable );

  osgscale->addChild( geode );

  resetcallback = new osaOSGHandle::ResetCallback;
  osgtransform->addUpdateCallback( resetcallback );

}

void osaOSGHandle::Select(){ 
  state = SELECTED;
  SetColor();
}

void osaOSGHandle::Release(){ 
  state = FREE; 
  SetColor();
}

void osaOSGHandle::SetColor(){
  if( IsFree() ) { drawable->setColor( osg::Vec4( 1.0, 0.0, 0.0, 0.0 ) ); } 
  else           { drawable->setColor( osg::Vec4( 1.0, 1.0, 0.0, 0.0 ) ); } 
}

void osaOSGHandle::Reset(){
  Release();
  SetColor();
}

vctMatrixRotation3<double> Rodrigues( vctFixedSizeVector<double,3>& w ){

  double theta = w.Norm();

  vctMatrixRotation3<double> I( 1.0, 0.0, 0.0,
				0.0, 1.0, 0.0,
				0.0, 0.0, 1.0,
				VCT_NORMALIZE );
  
  if( 0.001 < theta ){

    w.NormalizedSelf();

    vctMatrixRotation3<double> I( 1.0, 0.0, 0.0,
				  0.0, 1.0, 0.0,
				  0.0, 0.0, 1.0,
				  VCT_NORMALIZE );
    
    vctFixedSizeMatrix<double,3,3> wh;
    wh[0][0] =   0.0; wh[0][1] = -w[2]; wh[0][2] =  w[1];
    wh[1][0] =  w[2]; wh[1][1] =   0.0; wh[1][2] = -w[0];
    wh[2][0] = -w[1]; wh[2][1] =  w[0]; wh[2][2] =   0.0;
    
    vctFixedSizeMatrix<double,3,3> wh2 = wh * wh.Transpose();
    
    vctFixedSizeMatrix<double,3,3> r = I + wh*sin(theta) + wh2*(1.0-cos(theta));
    vctMatrixRotation3<double> R( r[0][0], r[0][1], r[0][2],
				  r[1][0], r[1][1], r[1][2],
				  r[2][0], r[2][1], r[2][2],
				  VCT_NORMALIZE );
    return R;

  }

  return I;

}

vctFrame4x4<double> osaOSGHandle::Drag( const vctFixedSizeVector<double,3>& v ){

  Select();

  // pos/ori of handle wrt body
  vctFrm3 Rthb = GetTransform();
  Rthb.InverseSelf();
  vctFixedSizeVector<double,3> thb = Rthb.Translation();
  vctMatrixRotation3<double> Rhb = Rthb.Rotation();

  vctFixedSizeMatrix<double,6,6> Ad( 0.0 );
  Ad[0][0] = Rhb[0][0]; Ad[0][1] = Rhb[1][0]; Ad[0][2] = Rhb[2][0]; 
  Ad[1][0] = Rhb[0][1]; Ad[1][1] = Rhb[1][1]; Ad[1][2] = Rhb[2][1]; 
  Ad[2][0] = Rhb[0][2]; Ad[2][1] = Rhb[1][2]; Ad[2][2] = Rhb[2][2]; 

  Ad[3][3] = Rhb[0][0]; Ad[3][4] = Rhb[1][0]; Ad[3][5] = Rhb[2][0]; 
  Ad[4][3] = Rhb[0][1]; Ad[4][4] = Rhb[1][1]; Ad[4][5] = Rhb[2][1]; 
  Ad[5][3] = Rhb[0][2]; Ad[5][4] = Rhb[1][2]; Ad[5][5] = Rhb[2][2]; 


  vctFixedSizeMatrix<double,3,3> ph;
  ph[0][0] =     0.0; ph[0][1] = -thb[2]; ph[0][2] =  thb[1];
  ph[1][0] =  thb[2]; ph[1][1] =     0.0; ph[1][2] = -thb[0];
  ph[2][0] = -thb[1]; ph[2][1] =  thb[0]; ph[2][2] =     0.0;

  vctFixedSizeMatrix<double,3,3> phR( Rhb.Transpose() * ph );
  Ad[3][0] = -phR[0][0]; Ad[3][1] = -phR[0][1]; Ad[3][2] = -phR[0][2]; 
  Ad[4][0] = -phR[1][0]; Ad[4][1] = -phR[1][1]; Ad[4][2] = -phR[1][2]; 
  Ad[5][0] = -phR[2][0]; Ad[5][1] = -phR[2][1]; Ad[5][2] = -phR[2][2]; 

  vctFixedSizeVector<double,6> fth( v[0], v[1], v[2], 0.0, 0.0, 0.0 );
  vctFixedSizeVector<double,6> ftb = Ad * fth;

  vctFixedSizeVector<double,3> w( 3.0*ftb[3], 3.0*ftb[4], 3.0*ftb[5] );
  vctMatrixRotation3<double> Rw = Rodrigues( w );

  vctFixedSizeVector<double,3> tw( ftb[0], ftb[1], ftb[2] );

  vctMatrixRotation3<double> Rwb;
  vctFixedSizeVector<double,3> twb;
  Rwb = GetBody()->GetTransform().Rotation()*Rw;
  twb = GetBody()->GetTransform().Translation()+tw;

  vctFrame4x4<double> Rtwb( Rwb, twb );			    
  body->SetTransform( Rtwb );
  
  vctFixedSizeVector<double,3> tbh = GetTransform().Translation();
  
  return vctFrame4x4<double>( vctMatrixRotation3<double>(), twb + Rwb*tbh );
  
}



void osaOSGPointer3D::IntersectionCallback::operator()( osg::Node* node, 
							osg::NodeVisitor* nv ){
  osg::Referenced* data = node->getUserData();
  osaOSGBody::UserData* userdata;
  userdata = dynamic_cast<osaOSGBody::UserData*>( data );

  // change the transform 
  if( userdata != NULL ){
    osaOSGPointer3D* pointer;
    pointer = dynamic_cast<osaOSGPointer3D*>(userdata->GetBody());
    if( pointer != NULL ){ pointer->CheckIntersection(); }
  }
  
  traverse( node, nv );
  
}   

osaOSGPointer3D::osaOSGPointer3D( osaOSGWorld* world, 
				const vctFrame4x4<double>& Rt,
				const std::string& name,
				double scale,
				double alpha ) :
  osaOSGBody( world, Rt ),
  world( world ),
  velocity( 0.0, 0.0, 0.0 ),
  select( false ){
  
  osaOSGBody::setName( name );

  geode = new osg::Geode();
  sphere = new osg::Sphere( osg::Vec3( 0.0, 0.0, 0.0 ), 0.01 );
  drawable = new osg::ShapeDrawable( sphere );
  geode->addDrawable( drawable );

  osgscale->addChild( geode );
  BuildPolytope();
  
  // Add an update callback to the transform for the intersector
  intersectioncallback = new osaOSGPointer3D::IntersectionCallback;
  osgtransform->addUpdateCallback( intersectioncallback );

}

void osaOSGPointer2D::PickHandler( osaOSGBody* body, 
				   osaOSGPick::Event e, 
				   double x, 
				   double y ){

  osg::ref_ptr< osaOSGHandle > h;
  h = dynamic_cast< osaOSGHandle* >( body );

  vctFixedSizeVector<double,3> xyz( x, y, 0.0 );
  vctFixedSizeVector<double,3> v = xyz - xyzprev;
  xyzprev = xyz;

  if( select && handle != NULL && e == osaOSGPick::DRAG )
    { handle->Drag( v ); }
  
  if( e == osaOSGPick::RELEASE ){
    handle = NULL;
    select = false;
  }

  // intersect with an osgBody but no self intersection
  if( h.get() != NULL ){
    switch( e ){

    case osaOSGPick::PUSH:
      handle = h;
      select = true;
      break;

    default: 
      break;

    }
  }

}

void osaOSGPointer3D::BuildPolytope(){

  polytope.clear();

  // resize and find the original position
  // const osg::Vec3d& scale = osgscale->getScale();
  const osg::Vec3d& t = osgtransform->getMatrix().getTrans();
  
  // extract all the triangles
  osaOSGBody::GeodeVisitor gv;
  geode->accept( gv );
  
  polytope.clear();
  polytope.setToBoundingBox( osg::BoundingBox( t.x() - 0.01,//scale.x(),
					       t.y() - 0.01,//scale.y(),
					       t.z() - 0.01,//scale.z(),
					       t.x() + 0.01,//scale.x(), 
					       t.y() + 0.01,//scale.y(), 
					       t.z() + 0.01/*scale.z()*/) );
}

void osaOSGPointer3D::SetVelocity( const vctFixedSizeVector<double,3>& v ){ 
  velocity = v;
  if( !select ){
    vctFrm3 Rtwp = GetTransform();
    SetTransform( vctFrame4x4<double>( vctMatrixRotation3<double>(),
				       Rtwp.Translation() + velocity ) );
  }
}


void osaOSGPointer3D::CheckIntersection(){

  BuildPolytope();

  // create a polytope picker
  osg::ref_ptr< osgUtil::PolytopeIntersector> intersector;
  intersector = new osgUtil::PolytopeIntersector( polytope );

  // compute all intersections
  osgUtil::IntersectionVisitor iv( intersector );
  world->accept( iv );

  // something intersected
  if( intersector->containsIntersections() ){
  
    // get the first intersection
    osgUtil::PolytopeIntersector::Intersections& 
      intersections = intersector->getIntersections();

    // for each intersections
    osgUtil::PolytopeIntersector::Intersections::const_iterator inti;
    for( inti=intersections.begin(); inti!=intersections.end(); inti++ ){

      // get the node path
      const osg::NodePath& nodePath = inti->nodePath;
      unsigned int idx = nodePath.size();
      
      // go up the path searching for a body
      while(idx--){
	
	osg::ref_ptr<osaOSGHandle> handle;
	handle = dynamic_cast< osaOSGHandle* >( nodePath[ idx ] );

	// intersect with an osgBody but no self intersection
	if( handle.get() != NULL ){

	  if( !select && handle->IsFree() )
	    { handle->Select(); }
	  if( select && handle->IsFree() )
	    { SetTransform( handle->Drag(velocity) ); }

	  break;

	}
      }
    }
  }

}

osaOSGBodyUI::osaOSGBodyUI( const std::string& model,
			    osaOSGWorld* world,
			    const vctFrame4x4<double>& Rt,
			    double scale,
			    double alpha,
			    const vctFrame4x4<double>& Rtoffset,
			    const std::string& option ) :
  osaOSGBody( model, world, Rt, scale, alpha, Rtoffset, option ){
  
  osg::ComputeBoundsVisitor cbv;
  cbv.apply( *this );
  
  osg::BoundingBox bb = cbv.getBoundingBox();
  
  double dx = 1.2*(bb.xMax() - bb.xMin())/2.0;
  double dy = 1.2*(bb.yMax() - bb.yMin())/2.0;
  double dz = 1.2*(bb.zMax() - bb.zMin())/2.0;
  
  double cx = (bb.xMax() + bb.xMin())/2.0;
  double cy = (bb.yMax() + bb.yMin())/2.0;
  double cz = (bb.zMax() + bb.zMin())/2.0;
 
  {
    vctFixedSizeVector<double,3> t( cx, cy, cz+dz );
    top = new osaOSGHandle( this, "top", t );
  }
  {
    vctFixedSizeVector<double,3> t( cx, cy, cz-dz );
    bottom = new osaOSGHandle( this, "bottom", t );
  }
  {
    vctFixedSizeVector<double,3> t( cx-dx, cy, cz );
    left = new osaOSGHandle( this, "left", t );
  }
  {
    vctFixedSizeVector<double,3> t( cx+dx, cy, cz );
    right = new osaOSGHandle( this, "right", t );
  }
  {
    vctFixedSizeVector<double,3> t( cx, cy-dy, cz );
    front = new osaOSGHandle( this, "front", t );
  }
  {
    vctFixedSizeVector<double,3> t( cx, cy+dy, cz );
    rear = new osaOSGHandle( this, "rear", t );
  }

}

