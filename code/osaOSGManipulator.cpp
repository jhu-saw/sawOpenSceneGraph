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

#include <sawOpenSceneGraph/osaOSGManipulator.h>

osaOSGManipulator::osaOSGManipulator(const std::vector<std::string>& models,
					 osaOSGWorld* world,
					 const vctFrame4x4<double>& Rtw0,
					 const std::string& robotfile,
					 const std::string& basemodel ) :
  robManipulator( robotfile, Rtw0 ),
  base( NULL ) {

  if( !basemodel.empty() ){ 
    // create the base and add it to the manipulator
    base =  new osaOSGBody( basemodel, (osaOSGWorld*)this, Rtw0 ); 
  }

  // create a group for the links
  osglinks = new osg::Group();

  // create the links and add them to the link group
  for( size_t i=0; i<links.size(); i++ ){
    osg::ref_ptr<osaOSGBody> li;
    li = new osaOSGBody( models[i], 
			   (osaOSGWorld*)(osglinks.get()), 
			   vctFrame4x4<double>() );
  }

  // add the links to the manipulator
  addChild( osglinks );

  // add the manipulator to the world
  world->addChild( this );
  
}

osaOSGManipulator::osaOSGManipulator(const std::vector<std::string>& models,
					 osaOSGWorld* world,
					 const vctFrm3& Rtw0,
					 const std::string& robotfile,
					 const std::string& basemodel ) :
  robManipulator( robotfile, 
		  vctFrame4x4<double>( Rtw0.Rotation(), Rtw0.Translation() ) ),
  base( NULL ){
  
  if( !basemodel.empty() ){ 
    // create the base and add it to the manipulator
    base =  new osaOSGBody( basemodel, (osaOSGWorld*)this, Rtw0 ); 
  }

  // create a group for the links
  osglinks = new osg::Group();

  // create the links and add them to the link group
  for( size_t i=0; i<links.size(); i++ ){
    osg::ref_ptr<osaOSGBody> li;
    li = new osaOSGBody( models[i], 
			   (osaOSGWorld*)(osglinks.get()),
			   vctFrame4x4<double>() );
  }

  // add the links to the manipulator
  addChild( osglinks );

  // add the manipulator to the world
  world->addChild( this );

}

osaOSGManipulator::~osaOSGManipulator(){}

osaOSGManipulator::Errno
osaOSGManipulator::GetPositions( vctDynamicVector<double>& q ) const {
  q = this->q; 
  return osaOSGManipulator::ESUCCESS;
}

osaOSGManipulator::Errno 
osaOSGManipulator::SetPositions( const vctDynamicVector<double>& qs ){
  
  // Ensure one joint value per link
  if( qs.size() == links.size() ){
    
    q = qs;
    
    // For each children
    for( unsigned int i=0; i<GetNumLinks(); i++ ){
      osg::ref_ptr<osaOSGBody> li = GetLink( i );
      if( li.get() != NULL )
	{ li->SetTransform( ForwardKinematics( q , i+1 ) ); }
    }

    return osaOSGManipulator::ESUCCESS;
  }
  
  CMN_LOG_RUN_ERROR << "Expected " << links.size() << " values. "
		    << "size(qs)=" << qs.size()
		    << std::endl;
  
  return osaOSGManipulator::EFAILURE;
  
}

unsigned int osaOSGManipulator::GetNumLinks()
{ return osglinks->getNumChildren(); }

osaOSGBody* osaOSGManipulator::GetLink( size_t i ){
  if( i < GetNumLinks() )
    { return dynamic_cast<osaOSGBody*>( osglinks->getChild(i) ); }
  return NULL;
}

