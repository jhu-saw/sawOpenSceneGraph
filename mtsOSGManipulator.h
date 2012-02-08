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

#ifndef _mtsOSGManipulator_h
#define _mtsOSGManipulator_h

#include <sawOpenSceneGraph/mtsOSGManipulatorTask.h>
#include <sawOpenSceneGraph/osaOSGManipulator.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

class mtsOSGManipulator : public mtsOSGManipulatorTask {

 public:

  enum InputType{ PROVIDE_INPUT, REQUIRE_INPUT };

  mtsOSGManipulator( const std::string& name,
		     double period,
		     osaCPUMask cpumask,
		     int priority,
		     const std::vector<std::string>& models,
		     osaOSGWorld* world,
		     const vctFrame4x4<double>& Rtw0,
		     const std::string& robfilename,
		     const std::string& basemodel,
		     mtsOSGManipulatorTask::InputType inputtype =
		     mtsOSGManipulatorTask::PROVIDE_INPUT ):
    mtsOSGManipulatorTask( name,
			   period, 
			   new osaOSGManipulator( models,
						  world,
						  Rtw0,
						  robfilename,
						  basemodel ),
			   cpumask, 
			   priority,
			   inputtype ){}
  
  // main constructor
  mtsOSGManipulator( const std::string& name,
		     double period,
		     osaCPUMask cpumask,
		     int priority,
		     const std::vector<std::string>& models,
		     osaOSGWorld* world,
		     const vctFrm3& Rtw0,
		     const std::string& robfilename,
		     const std::string& basemodel,
		     mtsOSGManipulatorTask::InputType inputtype =
		     mtsOSGManipulatorTask::PROVIDE_INPUT ):

    mtsOSGManipulatorTask( name, 
			   period, 
			   new osaOSGManipulator( models,
						  world,
						  Rtw0,
						  robfilename,
						  basemodel ),
			   cpumask, 
			   priority,
			   inputtype ){}
};

#endif





