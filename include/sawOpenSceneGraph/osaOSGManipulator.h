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

#ifndef _osaOSGManipulator_h
#define _osaOSGManipulator_h

#include <cisstRobot/robManipulator.h>
#include <sawOpenSceneGraph/osaOSGBody.h>
#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

class CISST_EXPORT osaOSGManipulator : 

  public osg::Group,
  public robManipulator{

 protected:

  //! Store the current joints values
  vctDynamicVector<double> q;

  osg::ref_ptr<osaOSGBody> base;

  osg::ref_ptr<osg::Group> osglinks;


  //!
  osaOSGManipulator( const vctFrame4x4<double>& Rtw0,
		       const std::string& robotfile ) :
    robManipulator( robotfile, Rtw0 ),
    base( NULL ){}

  //!
  osaOSGManipulator( const vctFrm3& Rtw0,
		       const std::string& robotfile ) :
    robManipulator( robotfile, vctFrame4x4<double>( Rtw0.Rotation(),
						    Rtw0.Translation()  ) ),
    base( NULL ){}

 public: 

  //! OSG Manipulator generic constructor
  /**
     This constructor initializes an OSG manipulator with the kinematics and 
     dynamics contained in a file. Plus it initializes the OSG elements of the
     manipulators (bodies and joints) for the engine.
     \param models A vector of 3D model file names
     \param world[in]  The OSG world
     \param Rtw0 The offset transformation of the robot base
     \param robotfile The file with the kinematics and dynamics parameters
     \param basemodel The file name of the base 3D model
  */
  osaOSGManipulator( const std::vector<std::string>& models,
		       osaOSGWorld* world,
		       const vctFrame4x4<double>& Rtw0,
		       const std::string& robfilename,
		       const std::string& basemodel = "" );

  //! OSG Manipulator generic constructor
  /**
     This constructor initializes an OSG manipulator with the kinematics and 
     dynamics contained in a file. Plus it initializes the OSG elements of the
     manipulators (bodies and joints) for the engine.
     \param models A vector of 3D model file names
     \param world  The OSG world
     \param Rtw0 The offset transformation of the robot base
     \param robotfile The file with the kinematics and dynamics parameters
     \param basemodel The file name of the base 3D model
  */
  osaOSGManipulator( const std::vector<std::string>& models,
		       osaOSGWorld* world,
		       const vctFrm3& Rtw0,
		       const std::string& robfilename,
		       const std::string& basemodel = "" );

  ~osaOSGManipulator();

  void Initialize();

  //! Return the joints positions
  /**
     Query each joint and return the joint positions
     \param q[in] A vector of joints positions
     \return ESUCCESS if successfull. EFAILURE otherwise.
  */
  virtual 
    osaOSGManipulator::Errno 
    GetPositions( vctDynamicVector<double>& q ) const;

  //! Set the joint position
  /**
     This sets the position command and stores a local copy
     \param q A vector of joint positions
     \return ESUCCESS if successfull. EFAILURE otherwise.
  */
  virtual 
    osaOSGManipulator::Errno 
    SetPositions( const vctDynamicVector<double>& q );

  unsigned int GetNumLinks();

  osaOSGBody* GetLink( size_t i );
  osaOSGBody* GetBase() 
  { return base.get(); }

  virtual void Attach( osaOSGManipulator* CMN_UNUSED(tool) ){}

};

#endif
