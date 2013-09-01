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


#ifndef _mtsOSGCamera_h
#define _mtsOSGCamera_h

#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <sawOpenSceneGraph/osaOSGCamera.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

class CISST_EXPORT mtsOSGCameraTask : public mtsTaskContinuous{

  friend class osaOSGHUD;
  
 protected:

  osg::ref_ptr<osaOSGCamera> camera;

 private:

  //! The input interface
  mtsInterfaceRequired* input;
  mtsFunctionRead GetPosition;


  //! User data for the MTS camera
  /** 
      This class stores a pointer to a MTS camera object. This pointer is is
      used during traversals to capture/process images and update the
      orientation/position of the camera.
  */
  class Data : public osg::Referenced {
  private:
    //! Pointer to a camera object
    mtsOSGCameraTask* mtsCamera;
  public:
    //! Default constructor.
    Data( mtsOSGCameraTask* camera ) : mtsCamera( camera ){}
    //! Get the pointer to the camera
    mtsOSGCameraTask* GetCameraTask() { return mtsCamera; }
  };


  //! Update Callback
  /**
     This callback is used to update the position/orientation of the camera
     during the update traversal.
  */
  class UpdateCallback : public osg::NodeCallback {
    //! Callback operator
    /**
       This operator is called during the update traversal. It's purpose is to
       update the position/orientation of the camera by calling the Update
       method.
    */
    void operator()( osg::Node* node, osg::NodeVisitor* );
  };

  //! This update method is called from the mtsOSGCamera::UpdateCallback
  virtual void UpdateTransform();

 public:
  
  // Main constructor
  mtsOSGCameraTask( const std::string& name, osaOSGCamera* camera );


  void setCullMask( osg::Node::NodeMask mask );

  ~mtsOSGCameraTask(){}

  void Startup();
  void Run();
  void Cleanup();

  inline osg::ref_ptr<osaOSGCamera> GetCamera(void) 
  { return camera; }

  void addEventHandler( osgGA::GUIEventHandler *eventHandler )
  { camera->addEventHandler( eventHandler ); }

};

#endif
