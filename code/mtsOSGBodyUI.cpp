#include <sawOpenSceneGraph/mtsOSGBodyUI.h>
#include <cisstMultiTask/mtsVector.h>
#include <cisstCommon/cmnPath.h>

mtsOSGPointer3D::mtsOSGPointer3D( const std::string& name,
				  osaOSGWorld* world, 
				  const vctFrame4x4<double>& Rt,
				  double scale, 
				  double alpha ) : 
  mtsComponent( name ),
  pointer( world, Rt, name, scale, alpha ){

  // Create the IO interface and add read/write commands
  pointer.input = AddInterfaceRequired( "Input", MTS_OPTIONAL );
  if( pointer.input ){ 
    pointer.input->AddFunction( "GetPositionCartesian", pointer.GetPosition );
    //pointer.input->AddFunction( "GetVelocityCartesian", pointer.GetVelocity );
    pointer.input->AddFunction( "GetButton",            pointer.GetButtons );
  }
  else{
    CMN_LOG_CLASS_RUN_ERROR << "Failed to create the interface Input" 
			    << std::endl;
  }

}


void mtsOSGPointer3D::Pointer::UpdateTransform(){

  if( GetPosition.IsValid() ){

    // Get the position of the pointer
    prmPositionCartesianGet prmRt;
    GetPosition( prmRt  );

    // Set the transformation
    bool valid = false;
    prmRt.GetValid( valid );

    vctFrm3 vctRt;
    prmRt.GetPosition( vctRt );
      
    vctFixedSizeVector<double,3> v = vctRt.Translation() - told;
    told = vctRt.Translation();
    if( valid ){
      SwitchOn();
      SetVelocity( v );
    }
    else{
      SwitchOff();
      SetTransform( vctRt );
    }

  }
  /*
  if( GetVelocity.IsValid() ){
    mtsDoubleVec vw;
    GetVelocity( vw  );

    bool valid = false;
    vw.GetValid( valid );
    if( valid ){
      SwitchOn();
      SetVelocity( vct3( vw[0], vw[1], vw[2] ) );
    }
    else{
      SwitchOff();
    }
  }
  */
  if( GetButtons.IsValid() ){

    mtsBool button;
    GetButtons( button );

    bool valid = false;
    button.GetValid( valid );
    if( valid ){
      if( button ) { Select(); }
      else         { Release(); }
    }

  }

  // Update the transformation
  osaOSGBody::UpdateTransform();

}


mtsOSGBodyUI::mtsOSGBodyUI( const std::string& name,
			    const std::string& model,
			    osaOSGWorld* world,
			    const vctFrame4x4<double>& Rt,
			    double scale,
			    double alpha,
			    const vctFrame4x4<double>& Rtoffset,
			    const std::string& option ) : 
  mtsComponent( name ),
  bodyui( model, world, vctFrame4x4<double>(),scale, alpha, Rtoffset, option ){

  bodyui.output = AddInterfaceRequired( "Output", MTS_OPTIONAL );
  if( bodyui.output ){ 
    bodyui.output->AddFunction( "SetPositionCartesian", 
				bodyui.SetPositionCartesian );
  }

  bodyui.SetTransform( Rt );

}


mtsOSGBodyUI::BodyUI::BodyUI( const std::string& model,
			      osaOSGWorld* world,
			      const vctFrame4x4<double>& Rt,
			      double scale,
			      double alpha,
			      const vctFrame4x4<double>& Rtoffset,
			      const std::string& option ) :
      osaOSGBodyUI( model, world, Rt, scale, alpha, Rtoffset, option ),
      output( NULL ){}


void mtsOSGBodyUI::BodyUI::UpdateTransform(){

  // Update the transformation
  osaOSGBody::UpdateTransform();

  prmPositionCartesianGet prmRtwb;
  bool valid = true;

  prmRtwb.SetPosition( GetTransform() );
  prmRtwb.SetValid( valid );
  SetPositionCartesian( prmRtwb );

}
