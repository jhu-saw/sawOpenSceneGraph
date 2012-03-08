/*

  Author(s): Simon Leonard
  Created on: Nov 11 2009

  (C) Copyright 2009-2011 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawOpenSceneGraph/mtsOSGManipulatorTask.h>
#include <cisstCommon/cmnLogger.h>

// main constructor
mtsOSGManipulatorTask::mtsOSGManipulatorTask( const std::string& name,
					      double period,
					      osaOSGManipulator* manipulator,
					      osaCPUMask cpumask,
					      int priority,
					      InputType inputtype ) :
  mtsTaskPeriodic( name, period, true ),
  manipulator( manipulator ),
  inputtype( inputtype ),
  inputp( NULL ),
  inputr( NULL ),
  output( NULL ),
  ctl( NULL ),
  cpumask( cpumask ),
  priority( priority ){

  if( inputtype == mtsOSGManipulatorTask::PROVIDE_INPUT ){
    inputp = AddInterfaceProvided( "Input" );
    if( inputp ){
      StateTable.AddData( prmqin, "PositionJointInput" );
      inputp->AddCommandWriteState( StateTable, prmqin, "SetPositionJoint" );
    }
    else{
      CMN_LOG_RUN_ERROR << "Failed to create interface Input for " << GetName()
			<< std::endl;
    }

  }
  else{ 
    inputr = AddInterfaceRequired( "Input" );
    if( inputr )
      { inputr->AddFunction( "GetPositionJoint", GetPositionJoint ); }
    else{
      CMN_LOG_RUN_ERROR << "Failed to create interface Input for " << GetName()
			<< std::endl;
    }
  }

  output = AddInterfaceProvided( "Output" );
  if( output ){
    
    StateTable.AddData( prmqout,  "PositionJointOutput" );
    StateTable.AddData( prmRtout, "PositionCartesianOutput" );
    output->AddCommandReadState( StateTable, prmRtout, "GetPositionCartesian" );
    output->AddCommandReadState( StateTable, prmqout,  "GetPositionJoint" );
    
  }
  else{
    CMN_LOG_RUN_ERROR << "Failed to create interface Output for " << GetName()
		      << std::endl;
  }

}

void mtsOSGManipulatorTask::Run(){

  ProcessQueuedCommands();

  if( manipulator.get() != NULL ){

    vctDynamicVector<double> vctq( prmqout.Position() );
    if( inputtype == PROVIDE_INPUT ){
      bool valid=false;
      prmqin.GetValid( valid );
      if( valid ) { vctq = prmqin.Goal(); }
    }
    else{ 
      prmPositionJointGet prmqinput;
      GetPositionJoint( prmqinput );
      bool valid=false;
      prmqinput.GetValid( valid );
      if( valid ) { prmqinput.GetPosition( vctq ); }
    }

    if( manipulator->SetPositions( vctq ) !=
	osaOSGManipulator::ESUCCESS ){
      CMN_LOG_RUN_ERROR << "Failed to get position for " << GetName()
			<< std::endl;
    }

    if( manipulator->GetPositions( prmqout.Position()) !=
	osaOSGManipulator::ESUCCESS ){
      CMN_LOG_RUN_ERROR << "Failed to get position for " << GetName()
			<< std::endl;
    }

    prmRtout.Position().FromRaw( manipulator->ForwardKinematics( vctq ) );
    prmRtout.SetValid( true );
  }

}
