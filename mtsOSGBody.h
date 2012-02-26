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


#ifndef _mtsOSGBody_h
#define _mtsOSGBody_h

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <sawOpenSceneGraph/osaOSGBody.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

class CISST_EXPORT mtsOSGBody : 
  public mtsComponent,
  public osaOSGBody {
  
 private:

  //! The input interface
  mtsInterfaceRequired* input;
  mtsFunctionRead GetPosition;

  //! This update method is called from the mtsOSGBody::UpdateCallback
  virtual void UpdateTransform();

 public:
  
  // Main constructor
  mtsOSGBody( const std::string& name,
	      const std::string& model,
	      osaOSGWorld* world,
	      const vctFrame4x4<double>& Rt,
	      double scale = 1.0,
	      double alpha = 1.0,
	      const std::string& option = std::string("") );

  ~mtsOSGBody(){}

};

#endif
