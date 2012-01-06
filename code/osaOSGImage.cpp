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

#include <algorithm>

#include <sawOpenSceneGraph/osaOSGImage.h>


// This is called at each update traversal
void osaOSGImage::TransformCallback::operator()( osg::Node* node, 
						 osg::NodeVisitor* nv ){
  osg::Referenced* data = node->getUserData();
  osaOSGImage::UserData* userdata;
  userdata = dynamic_cast<osaOSGImage::UserData*>( data );
  
  // change the transform 
  if( userdata != NULL )
    { userdata->GetImage()->UpdateTransform(); }
  traverse( node, nv );

}   

// This is called at each update traversal
void osaOSGImage::SwitchCallback::operator()( osg::Node* node, 
					      osg::NodeVisitor* nv ){

  osg::Referenced* data = node->getUserData();
  osaOSGImage::UserData* userdata;
  userdata = dynamic_cast<osaOSGImage::UserData*>( data );

  // change the switch
  if( userdata != NULL )
    { userdata->GetImage()->UpdateSwitch(); }
  traverse( node, nv );

}   


osaOSGImage::osaOSGImage( size_t width, 
			  size_t height,
			  const vctFrame4x4<double>& Rt ):

  transform( Rt ),
  onoff( SWITCH_ON ),
  width( width ),
  height( height ),
  data( NULL ){
  
  Initialize();

}

osaOSGImage::osaOSGImage( size_t width, 
			  size_t height,
			  osaOSGWorld* world,
			  const vctFrame4x4<double>& Rt ):

  transform( Rt ),
  onoff( SWITCH_ON ),
  width( width ),
  height( height ),
  data( NULL ){
  
  Initialize();

  // Once this is done add the image to the world
  if( world != NULL )
    { world->addChild( this ); }

}

osaOSGImage::osaOSGImage( size_t width, 
			  size_t height,
			  osaOSGWorld* world,
			  const vctFrm3& Rt ):

  onoff( SWITCH_ON ),
  width( width ),
  height( height ),
  data( NULL ){

  // Hack to avoid non-normalized rotations!
  const vctMatrixRotation3<double>& R = Rt.Rotation();
  vctQuaternionRotation3<double> q( R, VCT_NORMALIZE );
  transform = vctFrame4x4<double>( q, Rt.Translation() );

  Initialize();

  // Once this is done add the image to the world
  if( world != NULL )
    { world->addChild( this ); }

}

osaOSGImage::~osaOSGImage(){}

void osaOSGImage::Initialize(){

  // always moving  
  setDataVariance( osg::Object::DYNAMIC );
  
  // Setup the user data for this image. This can be used to recover the image
  // from callbacks
  userdata = new osaOSGImage::UserData( this );
  this->setUserData( userdata );

  // Create and configure the transform node
  osgtransform = new osg::MatrixTransform;
  osgtransform->setUserData( userdata );
  
  // Add an update callback to the transform
  transformcallback = new osaOSGImage::TransformCallback;
  osgtransform->setUpdateCallback( transformcallback );
  
  // Create and configure the switch node
  osgswitch = new osg::Switch();
  osgswitch->setUserData( userdata );
  
  // Add an update callback to the switch
  switchcallback = new osaOSGImage::SwitchCallback;
  osgswitch->setUpdateCallback( switchcallback );

  // create the image attached to the texture
  image = new osg::Image;
  image->setPixelBufferObject( new osg::PixelBufferObject( image.get() ) );

  // create the texture and attach the image
  texture = new osg::Texture2D;
  texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
  texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
  texture->setWrap( osg::Texture::WRAP_R, osg::Texture::REPEAT );
  texture->setResizeNonPowerOfTwoHint( false );
  texture->setImage( image.get() );

  // state set
  stateset = new osg::StateSet;
  stateset->setTextureAttributeAndModes( 0, texture, osg::StateAttribute::ON );

  // create a texture
  geode = new osg::Geode;
  osg::ref_ptr< osg::Geometry > geom;
  geom = osg::createTexturedQuadGeometry( osg::Vec3(width/-2.0,height/-2.0,0.0),
					  osg::Vec3( width, 0.0, 0.0),
					  osg::Vec3( 0.0, height, 0.0) );

  geode->addDrawable( geom );
  geode->setStateSet( stateset );

  osgtransform->addChild( geode ); 
  osgswitch->addChild( osgtransform );
  this->addChild( osgswitch );
  
  SetTransform( transform );


}

void osaOSGImage::SetImage( const osg::Image* img ){

  if( img->valid() ){

    // if mismatch
    if( image->s()                        != img->s()                        ||
	image->t()                        != img->t()                        ||
	image->r()                        != img->r()                        ||
	image->getInternalTextureFormat() != img->getInternalTextureFormat() ||
	image->getPixelFormat()           != img->getPixelFormat()           ||
	image->getDataType()              != img->getDataType() ){
    
      // clear the data and reset the image
      if( data != NULL ) delete[] data;
      data = new unsigned char[ img->getTotalSizeInBytes() ];

      image->setImage( img->s(),
		       img->t(),
		       img->r(), 
		       img->getInternalTextureFormat(),
		       img->getPixelFormat(), 
		       img->getDataType(),
		       data,
		       osg::Image::NO_DELETE );
    }

    memcpy( data, img->data(), img->getTotalSizeInBytes() );
    image->dirty();

  }

}


void osaOSGImage::SetImage( const std::string& filename ){

  osg::ref_ptr<osg::Image> img = osgDB::readImageFile( filename );
  SetImage( img );

}

// This is called from the image's callback
// This reads a transformation if the image is connected to an interface
void osaOSGImage::UpdateTransform(){

  vctFrame4x4<double> Rt( transform );
  osgtransform->setMatrix( osg::Matrix ( Rt[0][0], Rt[1][0], Rt[2][0], 0.0,
					 Rt[0][1], Rt[1][1], Rt[2][1], 0.0,
					 Rt[0][2], Rt[1][2], Rt[2][2], 0.0,
					 Rt[0][3], Rt[1][3], Rt[2][3], 1.0 ) );
}

void osaOSGImage::UpdateSwitch()
{ osgswitch->setValue( 0, onoff ); }

void osaOSGImage::SetTransform( const vctFrame4x4<double>& Rt )
{ transform = Rt; }

void osaOSGImage::SetTransform( const vctFrm3& Rt ){
  // Hack to avoid non-normalized rotations!
  const vctMatrixRotation3<double>& R = Rt.Rotation();
  vctQuaternionRotation3<double> q( R, VCT_NORMALIZE );
  SetTransform( vctFrame4x4<double>( q, Rt.Translation() ) );
}

vctFrm3 osaOSGImage::GetTransform() const{ 
  vctMatrixRotation3<double> R(transform[0][0],transform[0][1],transform[0][2],
			       transform[1][0],transform[1][1],transform[1][2],
			       transform[2][0],transform[2][1],transform[2][2]);
  return vctFrm3( R, transform.Translation() ); 
}


//! Set the switch of the image
void osaOSGImage::SwitchOn()
{ onoff = SWITCH_ON; }

void osaOSGImage::SwitchOff()
{ onoff = SWITCH_OFF; }
  
