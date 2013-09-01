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
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <sawOpenSceneGraph/osaOSGBody.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

class CISST_EXPORT mtsOSGBody : public mtsComponent{
  
 protected:

  class Body : public osaOSGBody{

  protected:

    //! This update method is called from the mtsOSGBody::UpdateCallback
    virtual void UpdateTransform();
    
  public:

    //! The input interface
    mtsInterfaceRequired* input;
    mtsFunctionRead GetPosition;

    mtsInterfaceRequired* output;
    mtsFunctionWrite SetPosition;

    Body( const std::string& model,
	  osaOSGWorld* world,
	  const vctFrame4x4<double>& Rt,
	  double scale,
	  double alpha,
	  const vctFrame4x4<double>& Rtoffset,
	  const std::string& option ) :
      osaOSGBody( model, world, Rt, scale, alpha, Rtoffset, option ),
      input( NULL ){}
  };

  Body body;

 public:
  
  // Main constructor
  mtsOSGBody( const std::string& name,
	      const std::string& model,
	      osaOSGWorld* world,
	      const vctFrame4x4<double>& Rt,
	      double scale = 1.0,
	      double alpha = 1.0,
	      const vctFrame4x4<double>& Rtoffset = vctFrame4x4<double>(),
	      const std::string& option = std::string("") );

  ~mtsOSGBody(){}

};

#endif
