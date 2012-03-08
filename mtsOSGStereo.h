/*

  Author(s): Simon Leonard
  Created on: Dec 02 2009

  (C) Copyright 2009 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsOSGStereo_h
#define _mtsOSGStereo_h


#include <sawOpenSceneGraph/osaOSGStereo.h>
#include <sawOpenSceneGraph/mtsOSGCameraTask.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

class mtsOSGStereo : public mtsOSGCameraTask {
  
 public:
  
  // Main constructor
  /**
  */
  mtsOSGStereo( const std::string& name,
		osaOSGWorld* world,
		int x, int y, int width, int height,
		double fovy, double aspectRatio,
		double zNear, double zFar,
		double baseline,
		bool trackball = true,
		const vctFrame4x4<double>& Rtoffset = vctFrame4x4<double>() ) :
  mtsOSGCameraTask( name, 
		    new osaOSGStereo( world,
				      x, y, 
				      width, height, 
				      fovy, aspectRatio, 
				      zNear, zFar,
				      baseline,
				      trackball,
				      Rtoffset ) ){}
  
 mtsOSGStereo( const std::string& name,
	       osaOSGWorld* world,
	       int x, int y, int width, int height,
	       const vctFixedSizeMatrix<double,3,3>& Kl,
	       const vctFixedSizeMatrix<double,3,3>& Kr,
	       const vctFrame4x4<double>& vctRt,
	       double zNear, double zFar,
	       bool trackball,
	       const vctFrame4x4<double>& Rtoffset = vctFrame4x4<double>() ) :
  mtsOSGCameraTask( name, 
		    new osaOSGStereo( world,
				      x, y, 
				      width, height, 
				      Kl, Kr, vctRt,
				      zNear, zFar,
				      trackball,
				      Rtoffset ) ){}
  
  ~mtsOSGStereo(){}

  void setCullMask( osg::Node::NodeMask mask, osaOSGStereo::Camera idx ){ 
    osaOSGStereo* stereo = dynamic_cast< osaOSGStereo* >( camera.get() );
    if( stereo != NULL )
      { stereo->setCullMask( mask, idx ); }
  }

};

#endif
