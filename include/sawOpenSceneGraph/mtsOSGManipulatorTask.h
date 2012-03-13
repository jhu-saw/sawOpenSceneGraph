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

#ifndef _mtsOSGManipulatorTask_h
#define _mtsOSGManipulatorTask_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>

#include <cisstOSAbstraction/osaCPUAffinity.h>

#include <sawOpenSceneGraph/osaOSGManipulator.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

class CISST_EXPORT mtsOSGManipulatorTask : public mtsTaskPeriodic {

 public:

  enum InputType{ PROVIDE_INPUT, REQUIRE_INPUT };

 private:

  osg::ref_ptr<osaOSGManipulator> manipulator;

  prmPositionCartesianGet prmRtout;
  prmPositionJointGet prmqout;
  prmPositionJointSet prmqin;

  mtsOSGManipulatorTask::InputType inputtype;
  mtsFunctionRead GetPositionJoint;
  mtsInterfaceProvided* inputp;
  mtsInterfaceRequired* inputr;
  mtsInterfaceProvided* output;
  mtsInterfaceProvided* ctl;

  osaCPUMask cpumask;
  int priority;

 public:

  mtsOSGManipulatorTask( const std::string& name,
			 double period,
			 osaOSGManipulator* manipulator,
			 osaCPUMask cpumask,
			 int priority,
			 mtsOSGManipulatorTask::InputType inputtype = 
			 mtsOSGManipulatorTask::PROVIDE_INPUT );
  
  ~mtsOSGManipulatorTask(){}

  void Configure( const std::string& CMN_UNUSED(argv) = "" ){}

  void Startup(){}
  void Run();
  void Cleanup(){}

};

#endif





