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


#include <sawOpenSceneGraph/mtsOSGBody.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstCommon/cmnLogger.h>

mtsOSGBody::mtsOSGBody( const std::string& name,
			const std::string& model,
			osaOSGWorld* world,
			const vctFrame4x4<double>& Rt,
			double scale,
			double alpha,
			const vctFrame4x4<double>& Rtoffset,
			const std::string& option ):
  mtsComponent( name ),
  body( model, world, Rt, scale, alpha, Rtoffset, option ){

  // Create the IO interface and add read/write commands
  body.input = AddInterfaceRequired( "Input", MTS_OPTIONAL );
  if( body.input )
    { body.input->AddFunction( "GetPositionCartesian", body.GetPosition ); }
  else{
    CMN_LOG_CLASS_RUN_ERROR << "Failed to create the interface Input" 
			    << std::endl;
  }

  body.output = AddInterfaceRequired( "Output", MTS_OPTIONAL );
  if( body.output )
    { body.output->AddFunction( "SetPositionCartesian", body.SetPosition ); }
  else{
    CMN_LOG_CLASS_RUN_ERROR << "Failed to create the interface Input" 
			    << std::endl;
  }

}

void mtsOSGBody::Body::UpdateTransform(){

  if( GetPosition.IsValid() ){

    // Get the position of the camera
    prmPositionCartesianGet prmRt;
    GetPosition( prmRt );

    bool valid = false;
    prmRt.GetValid( valid );

    // Set the transformation
    if( valid )
      { SetTransform( prmRt.Position() ); }
    
  }

  // Update the transformation
  osaOSGBody::UpdateTransform();

  {
    prmPositionCartesianSet prmRt;

    prmRt.SetGoal( GetTransform() );
    bool valid = true;
    prmRt.SetValid( valid );
    SetPosition( prmRt );
  }

}
