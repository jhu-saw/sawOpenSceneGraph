#
#
# (C) Copyright 2012 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

set(
  REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstRobot
     cisstNumerical
     cisstMultiTask
     cisstParameterTypes
     cisstStereoVision )

find_package( cisst REQUIRED ${REQUIRED_CISST_LIBRARIES} )

if( cisst_FOUND_AS_REQUIRED )

  include( ${CISST_USE_FILE} )
  cisst_find_saw_component( sawOpenSceneGraph REQUIRED )

  include_directories( ${sawOpenSceneGraph_INCLUDE_DIR} )
  link_directories( ${sawOpenSceneGraph_LIBRARY_DIR} )

  set (sawOpenSceneGraph_EXAMPLES
       osaOSGCameraMonoExample
       osaOSGCameraStereoExample
       osaOSGCameraHUDExample
       osaOSGWAMExample
       osaOSGBarrettHandExample
       osaOSGPickExample
       osaOSGImageSequenceExample

       mtsOSGCameraMonoExample
       mtsOSGCameraStereoExample
       mtsOSGImageSequenceExample
       mtsOSGWAMExample
       mtsOSGBarrettHandExample

       svlOSGImageSequenceExample
       svlOSGCameraHUDExample)

  foreach (_example ${sawOpenSceneGraph_EXAMPLES})
    add_executable (${_example} ${_example}.cpp)
    target_link_libraries (${_example} ${sawOpenSceneGraph_LIBRARIES} ${saw3Dconnexion_LIBRARIES} )
    cisst_target_link_libraries (${_example} ${REQUIRED_CISST_LIBRARIES})
    set_property (TARGET ${_example} PROPERTY FOLDER "sawOpenSceneGraph")
  endforeach ()

endif( cisst_FOUND_AS_REQUIRED )

