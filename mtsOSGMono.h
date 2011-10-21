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


#ifndef _mtsOSGMono_h
#define _mtsOSGMono_h


#include <sawOpenSceneGraph/osaOSGMono.h>
#include <sawOpenSceneGraph/mtsOSGCameraTask.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

class mtsOSGMono : public mtsOSGCameraTask {
  
 public:
  
  // Main constructor
  mtsOSGMono( const std::string& name,
	      osaOSGWorld* world,
	      int x, int y, int width, int height,
	      double fovy, double aspectRatio,
	      double zNear, double zFar,
	      bool trackball = true,
	      const vctFrame4x4<double>& Rtoffset = vctFrame4x4<double>(),
	      bool offscreenrendering = false ) : 
    mtsOSGCameraTask( name, 
		      new osaOSGMono( world, 
				      x, y, 
				      width, height, 
				      fovy, aspectRatio,
				      zNear, zFar,
				      trackball,
				      Rtoffset,
				      offscreenrendering ) ){}
  ~mtsOSGMono(){}

};

#endif
