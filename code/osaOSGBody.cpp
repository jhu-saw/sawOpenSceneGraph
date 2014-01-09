/*

  Author(s): Simon Leonard
  Created on: Nov 11 2009

  (C) Copyright 2008 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <osgDB/ReadFile> 
#include <osg/PolygonMode>
#include <osg/Point>
#include <osg/Material>
#include <osg/BlendFunc> 
#include <osg/AlphaFunc>
#include <algorithm>

#include <sawOpenSceneGraph/osaOSGBody.h>


const vctFixedSizeVector<unsigned char,3> osaOSGBody::RGBDEFAULT = vctFixedSizeVector<unsigned char,3>( 255, 0, 0 );

osaOSGBody::TransparencyVisitor::TransparencyVisitor() :
  osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ){}

//void osaOSGBody::GeodeVisitor::apply( osg::Geode& geode  ){
void osaOSGBody::TransparencyVisitor::apply( osg::Geode& geode  ){

  // for each drawable
  for( size_t i=0; i<geode.getNumDrawables(); i++ ){

    // set blending
    osg::StateSet* stateset = geode.getDrawable(i)->getOrCreateStateSet();
    stateset->setMode( GL_BLEND, 
		       osg::StateAttribute::ON | 
		       osg::StateAttribute::OVERRIDE );
    stateset->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
  

    // change the alpha of the color (no material)
    if( geode.getDrawable(i)->asGeometry() ){

      // get the geometry
      osg::ref_ptr<osg::Geometry> geometry;
      geometry = geode.getDrawable(i)->asGeometry();

      // get its color
      osg::ref_ptr<osg::Vec4Array> colorarray;
      colorarray = dynamic_cast<osg::Vec4Array*>( geometry->getColorArray() );

      if( colorarray ){
	for( unsigned int i=0; i<colorarray->size(); i++ ){
	  osg::Vec4* color= &colorarray->operator[](i);
	  color->set( color->x(), color->y(), color->z(), alpha );
	}
      }
    }

    // change the alpha of the material
    osg::StateAttribute* attribute;
    attribute = stateset->getAttribute( osg::StateAttribute::MATERIAL );

    osg::ref_ptr<osg::Material> material;
    material = dynamic_cast<osg::Material*>( attribute );
    
    if( material != NULL ){

      material->setAlpha( osg::Material::FRONT_AND_BACK, alpha );
      stateset->setAttributeAndModes( material.get(), 
				      osg::StateAttribute::ON | 
				      osg::StateAttribute::OVERRIDE );
      osg::BlendFunc* bf;
      bf = new osg::BlendFunc( osg::BlendFunc::SRC_ALPHA,
			       osg::BlendFunc::ONE_MINUS_SRC_ALPHA );
      stateset->setAttributeAndModes(bf);

    }
    
  }

  traverse( geode );

}

// Default constructor for the geode visitor
osaOSGBody::GeodeVisitor::GeodeVisitor() : 
  osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ){}

// Method called for each geode during the traversal
//void osaOSGBody::GeodeVisitor::apply( osg::Geode& geode  ){
void osaOSGBody::GeodeVisitor::apply( osg::Geode& geode  ){

  for( size_t i=0; i< geode.getNumDrawables(); ++i ){
    // create a triangle extractor
    osg::TriangleFunctor<TriangleExtractor> te;
    // apply the extractor to the drawable
    geode.getDrawable( i )->accept( te );

    // copy the drawable triangles to the geode triangles
    geodetriangles.insert( geodetriangles.end(),
			   te.drawabletriangles.begin(),
			   te.drawabletriangles.end() );
  }

  traverse( geode );
}

// For each triangle in a drawable this operator is called
void 
osaOSGBody::GeodeVisitor::TriangleExtractor::operator ()
  ( const osg::Vec3& p1, const osg::Vec3& p2, const osg::Vec3& p3, bool ){

  osaOSGBody::GeodeVisitor::Triangle triangle;
  triangle.p1 = p1;
  triangle.p2 = p2;
  triangle.p3 = p3;
  drawabletriangles.push_back( triangle );

}



// This is called at each update traversal
void osaOSGBody::TransformCallback::operator()( osg::Node* node, 
						osg::NodeVisitor* nv ){
  osg::Referenced* data = node->getUserData();
  osaOSGBody::UserData* userdata;
  userdata = dynamic_cast<osaOSGBody::UserData*>( data );

  // change the transform 
  if( userdata != NULL )
    { userdata->GetBody()->UpdateTransform(); }
  traverse( node, nv );

}   

// This is called at each update traversal
void osaOSGBody::SwitchCallback::operator()( osg::Node* node, 
					     osg::NodeVisitor* nv ){

  osg::Referenced* data = node->getUserData();
  osaOSGBody::UserData* userdata;
  userdata = dynamic_cast<osaOSGBody::UserData*>( data );

  // change the switch
  if( userdata != NULL )
    { userdata->GetBody()->UpdateSwitch(); }
  traverse( node, nv );

}   

osaOSGBody::osaOSGBody( osaOSGWorld* world, const vctFrame4x4<double>& Rt ) :
  transform(Rt),
  onoff( SWITCH_ON ){
  
  Initialize(1.0); // scale = 1.0
  if( world != NULL )
    { world->addChild( this ); }
  
  UpdateTransform();
}

osaOSGBody::osaOSGBody( osaOSGBody* body, const vctFrame4x4<double>& Rt ) :
  transform(Rt),
  onoff( SWITCH_ON ){
  
  Initialize(1.0); // scale = 1.0
  
  // Once this is done add the body to the world
  if( body != NULL )
    { body->osgtransform->addChild( this ); }

  UpdateTransform();

}


osaOSGBody::osaOSGBody( const std::string& model, 
			const vctFrame4x4<double>& Rt,
			double scale,
			double alpha,
			const vctFrame4x4<double>& Rtoffset,
			const std::string& options ) :
  transform( Rt ),
  Rtoffset( Rtoffset ),
  onoff( SWITCH_ON ){
  
  Initialize( scale );
  ReadModel( model, options, alpha );

  UpdateTransform();

}

osaOSGBody::osaOSGBody( const std::string& model, 
			osaOSGWorld* world,
			const vctFrame4x4<double>& Rt,
			double scale,
			double alpha,
			const vctFrame4x4<double>& Rtoffset,
			const std::string& options ) :
  
  transform( Rt ),
  Rtoffset( Rtoffset ),
  onoff( SWITCH_ON ){

  Initialize( scale );
  ReadModel( model, options, alpha );

  // Once this is done add the body to the world
  if( world != NULL )
    { world->addChild( this ); }

  UpdateTransform();
}

osaOSGBody::osaOSGBody( const std::string& model, 
			osaOSGBody* body,
			const vctFrame4x4<double>& Rt,
			double scale,
			double alpha,
			const vctFrame4x4<double>& Rtoffset,
			const std::string& options ) :

  transform( Rt ),
  Rtoffset( Rtoffset ),
  onoff( SWITCH_ON ){
  
  Initialize( scale );
  ReadModel( model, options, alpha );

  // Once this is done add the body to the world
  if( body != NULL )
    { body->osgtransform->addChild( this ); }

  UpdateTransform();
}

osaOSGBody::osaOSGBody( const std::string& model, 
			osaOSGWorld* world,
			const vctFrm3& Rt,
			double scale,
			double alpha,
			const std::string& options ) :

  onoff( SWITCH_ON ){

  // Hack to avoid non-normalized rotations!
  const vctMatrixRotation3<double>& R = Rt.Rotation();
  vctQuaternionRotation3<double> q( R, VCT_NORMALIZE );
  transform = vctFrame4x4<double>( q, Rt.Translation() );

  Initialize( scale );
  ReadModel( model, options, alpha );

  // Once this is done add the body to the world
  if( world != NULL )
    { world->addChild( this ); }

}

osaOSGBody::osaOSGBody( const vctDynamicMatrix<double>& pointcloud,
			osaOSGWorld* world,
			const vctFrm3& Rt,
			const vctFixedSizeVector<unsigned char,3>& rgb,
			float size ):

  onoff( SWITCH_ON ){

  // Hack to avoid non-normalized rotations!
  const vctMatrixRotation3<double>& R = Rt.Rotation();
  vctQuaternionRotation3<double> q( R, VCT_NORMALIZE );
  transform = vctFrame4x4<double>( q, Rt.Translation() );

  Initialize();
  Read3DData( pointcloud, rgb, size );

  if( world != NULL )
    { world->addChild( this ); }
 
}


osaOSGBody::~osaOSGBody(){}

void osaOSGBody::Initialize( double scale ){

  // always moving  
  setDataVariance( osg::Object::DYNAMIC );
  
  // Setup the user data for this body. This can be used to recover the body
  // from callbacks
  userdata = new osaOSGBody::UserData( this );
  this->setUserData( userdata );

  osgscale = new osg::PositionAttitudeTransform();
  osgscale->setScale( osg::Vec3( scale, scale, scale ) );
  if( scale != 1.0 ){
    osg::ref_ptr<osg::StateSet> stateset = osgscale->getOrCreateStateSet(); 
    stateset->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
  }

  
  // Create and configure the transform node
  osgtransform = new osg::MatrixTransform;
  osgtransform->setUserData( userdata );
  
  // Add an update callback to the transform
  transformcallback = new osaOSGBody::TransformCallback;
  osgtransform->setUpdateCallback( transformcallback );
  
  // Create and configure the switch node
  osgswitch = new osg::Switch();
  osgswitch->setUserData( userdata );
  
  // Add an update callback to the switch
  switchcallback = new osaOSGBody::SwitchCallback;
  osgswitch->setUpdateCallback( switchcallback );

  osgtransform->addChild( osgscale );
  osgswitch->addChild( osgtransform );
  this->addChild( osgswitch );
  
  SetTransform( transform );

}

void osaOSGBody::AddTransformCallback() {
    if( transformcallback == NULL) {
        transformcallback = new osaOSGBody::TransformCallback;
        osgtransform->setUpdateCallback( transformcallback );
    }
}

void osaOSGBody::ReadModel( const std::string& model,
			    const std::string& options,
			    double alpha ){

  osg::ref_ptr< osgDB::ReaderWriter::Options > osgoptions;
  osgoptions = new osgDB::ReaderWriter::Options( options );

  std::string path;
  size_t found;
#if (CISST_OS == CISST_WINDOWS)
  found = model.rfind( '/' );
#else
  found = model.rfind( '/' );
#endif    

  if( found != std::string::npos )
    { path.assign( model, 0, found ); }
  osgoptions->setDatabasePath( path );
  
  osg::ref_ptr<osg::Node> node = osgDB::readNodeFile( model, osgoptions );

  if( node != NULL ){
    osaOSGBody::TransparencyVisitor tv; 
    tv.alpha = alpha;
    node->accept( tv ); 
    // Add the node to the transformation node
    osgscale->addChild( node );
  }
  else{
    CMN_LOG_RUN_ERROR << "Failed to create node for: " << model << std::endl;
  }

}

void osaOSGBody::Read3DData( const vctDynamicMatrix<double>& pc,
			       const vctFixedSizeVector<unsigned char,3>& RGB,
			       float size ){
  
  size_t npoints= 0;
  if( pc.rows() == 3 )
    { npoints = pc.cols(); }
  else{
    // invalid data
  }
  
  // First, create a geode
  //osg::ref_ptr<osg::Geode> geode;
  try{ osggeode = new osg::Geode; }
  catch( std::bad_alloc& ){ 
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << "Failed to create a geode." 
		      << std::endl;
  }
  osgscale->addChild( osggeode );

  // then create a geometry
  osg::ref_ptr<osg::Geometry> pointsGeom;
  try{ pointsGeom = new osg::Geometry; }
  catch( std::bad_alloc& ){ 
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << "Failed to create a points geometry." 
		      << std::endl;
  }
  pointsGeom->getOrCreateStateSet()->setAttribute( new osg::Point( size ),
  						   osg::StateAttribute::ON );
  // add the geometry to the geod
  osggeode->addDrawable( pointsGeom );

  // Create an array primitive set 
  osg::ref_ptr<osg::DrawArrays> drawArrayPoints;
  try{ drawArrayPoints = new osg::DrawArrays( osg::PrimitiveSet::POINTS ); }
  catch( std::bad_alloc& ){ 
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << "Failed to create array of points."
		      << std::endl;
  }
  // add the set to the geometry
  pointsGeom->addPrimitiveSet( drawArrayPoints );

  // add a vector of vertices
  osg::ref_ptr<osg::Vec3Array> vertexData;
  try{ vertexData = new osg::Vec3Array; }
  catch( std::bad_alloc& ){ 
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << "Failed to create vertices array." 
		      << std::endl;
  }
  pointsGeom->setVertexArray( vertexData );
  
  for( size_t i=0; i<npoints; i++ )
    { vertexData->push_back( osg::Vec3( pc[0][i], pc[1][i], pc[2][i] ) ); }

  osg::ref_ptr<osg::Vec4Array> colorArray = new osg::Vec4Array;
  colorArray->push_back( osg::Vec4( RGB[0], RGB[1], RGB[2], 1.0f ) );
  
  osg::ref_ptr< osg::TemplateIndexArray< unsigned int, 
					 osg::Array::UIntArrayType,
					 4, 1 > > colorIndexArray;
  colorIndexArray = new osg::TemplateIndexArray< unsigned int, 
						 osg::Array::UIntArrayType,
						 4, 1>;
  colorIndexArray->push_back(0);

  pointsGeom->setColorArray( colorArray );
  //pointsGeom->setColorIndices(colorIndexArray);
  pointsGeom->setColorBinding(osg::Geometry::BIND_OVERALL);

  drawArrayPoints->setFirst( 0 );
  drawArrayPoints->setCount( vertexData->size() );

}

// This is called from the body's callback
// This reads a transformation if the body is connected to an interface
void osaOSGBody::UpdateTransform(){

  vctFrame4x4<double> Rt( transform );
  osgtransform->setMatrix( osg::Matrix ( Rt[0][0], Rt[1][0], Rt[2][0], 0.0,
					 Rt[0][1], Rt[1][1], Rt[2][1], 0.0,
					 Rt[0][2], Rt[1][2], Rt[2][2], 0.0,
					 Rt[0][3], Rt[1][3], Rt[2][3], 1.0 ) );
}

void osaOSGBody::UpdateSwitch()
{ osgswitch->setValue( 0, (bool)onoff ); }

void osaOSGBody::SetTransform( const vctFrame4x4<double>& Rt )
{ transform = Rt * Rtoffset; }

void osaOSGBody::SetTransform( const vctFrm3& Rt ){
  // Hack to avoid non-normalized rotations!
  const vctMatrixRotation3<double>& R = Rt.Rotation();
  vctQuaternionRotation3<double> q( R, VCT_NORMALIZE );
  SetTransform( vctFrame4x4<double>( q, Rt.Translation() ) );
}

vctFrm3 osaOSGBody::GetTransform() const{ 
  vctMatrixRotation3<double> R(transform[0][0],transform[0][1],transform[0][2],
			       transform[1][0],transform[1][1],transform[1][2],
			       transform[2][0],transform[2][1],transform[2][2]);
  return vctFrm3( R, transform.Translation() ); 
}


//! Set the switch of the body
void osaOSGBody::SwitchOn()
{ onoff = SWITCH_ON; }

void osaOSGBody::SwitchOff()
{ onoff = SWITCH_OFF; }
  
void osaOSGBody::SetModeFill(){
  osg::ref_ptr<osg::StateSet> state = getOrCreateStateSet();
  osg::ref_ptr<osg::PolygonMode> pm;
  pm = dynamic_cast<osg::PolygonMode*>
    ( state->getAttribute( osg::StateAttribute::POLYGONMODE ));
  if( !pm ){
    pm = new osg::PolygonMode;
    state->setAttribute( pm );
  }
  pm->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL );
}

void osaOSGBody::SetModeLine(){
  osg::ref_ptr<osg::StateSet> state = getOrCreateStateSet();
  osg::ref_ptr<osg::PolygonMode> pm;
  pm = dynamic_cast<osg::PolygonMode*>
    ( state->getAttribute( osg::StateAttribute::POLYGONMODE ));
  if( !pm ){
    pm = new osg::PolygonMode;
    state->setAttribute( pm );
  }
  pm->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
}

void osaOSGBody::SetModePoint(){
  osg::ref_ptr<osg::StateSet> state = getOrCreateStateSet();
  osg::ref_ptr<osg::PolygonMode> pm;
  pm = dynamic_cast<osg::PolygonMode*>
    ( state->getAttribute( osg::StateAttribute::POLYGONMODE ));
  if( !pm ){
    pm = new osg::PolygonMode;
    state->setAttribute( pm );
  }
  pm->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::POINT );
}


vctDynamicMatrix<double> osaOSGBody::GetVertices(){

  std::vector< osg::Vec3 > vertices;

  osaOSGBody::GeodeVisitor gv;
  this->accept( gv );

  //vctDynamicMatrix<double> vertices( 3, gv.geodetriangles.size(), 0.0 );

  for( size_t i=0; i<gv.geodetriangles.size(); i++ ){
  
    vertices.push_back(  gv.geodetriangles[i].p1 );
    vertices.push_back(  gv.geodetriangles[i].p2 );
    vertices.push_back(  gv.geodetriangles[i].p3 );

  }
  
  std::sort( vertices.begin(), vertices.end() );

  std::vector< osg::Vec3 >::iterator last;
  last = std::unique( vertices.begin(), vertices.end() );
  
  vertices.resize( last - vertices.begin() );

  vctDynamicMatrix<double> P( 3, vertices.size() );
  for( size_t i=0; i<vertices.size(); i++ ){
    P[0][i] = vertices[i][0];
    P[1][i] = vertices[i][1];
    P[2][i] = vertices[i][2];
  }

  return P;

}
