#
# $Id$
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
  cisst_find_component( sawOpenSceneGraph REQUIRED )

  include_directories( ${sawOpenSceneGraph_INCLUDE_DIR} )
  link_directories( ${sawOpenSceneGraph_LIBRARY_DIR} )

  add_executable( osaOSGCameraMonoExample   osaOSGCameraMonoExample.cpp )
  add_executable( osaOSGCameraStereoExample osaOSGCameraStereoExample.cpp )
  add_executable( osaOSGCameraHUDExample    osaOSGCameraHUDExample.cpp )
  add_executable( osaOSGWAMExample          osaOSGWAMExample.cpp )
  add_executable( osaOSGBarrettHandExample  osaOSGBarrettHandExample.cpp )
  add_executable( osaOSGPickExample         osaOSGPickExample.cpp )

  add_executable( osaOSGImageSequenceExample osaOSGImageSequenceExample.cpp )

  add_executable( mtsOSGCameraMonoExample   mtsOSGCameraMonoExample.cpp )
  add_executable( mtsOSGCameraStereoExample mtsOSGCameraStereoExample.cpp )
  add_executable( mtsOSGImageSequenceExample mtsOSGImageSequenceExample.cpp )
  add_executable( mtsOSGWAMExample          mtsOSGWAMExample.cpp )
  add_executable( mtsOSGBarrettHandExample  mtsOSGBarrettHandExample.cpp )

  add_executable( svlOSGImageSequenceExample svlOSGImageSequenceExample.cpp )
  add_executable( svlOSGCameraHUDExample     svlOSGCameraHUDExample.cpp )

  target_link_libraries( osaOSGCameraMonoExample      ${sawOpenSceneGraph_LIBRARIES} ) 
  target_link_libraries( osaOSGCameraStereoExample    ${sawOpenSceneGraph_LIBRARIES} ) 
  target_link_libraries( osaOSGCameraHUDExample       ${sawOpenSceneGraph_LIBRARIES} ) 
  target_link_libraries( osaOSGWAMExample             ${sawOpenSceneGraph_LIBRARIES} ) 
  target_link_libraries( osaOSGBarrettHandExample     ${sawOpenSceneGraph_LIBRARIES} ) 
  target_link_libraries( osaOSGPickExample            ${sawOpenSceneGraph_LIBRARIES} ) 
  target_link_libraries( osaOSGImageSequenceExample            ${sawOpenSceneGraph_LIBRARIES} ) 

  target_link_libraries( mtsOSGCameraMonoExample      ${sawOpenSceneGraph_LIBRARIES} ) 
  target_link_libraries( mtsOSGCameraStereoExample    ${sawOpenSceneGraph_LIBRARIES} ) 
  target_link_libraries( mtsOSGImageSequenceExample   ${sawOpenSceneGraph_LIBRARIES} ) 
  target_link_libraries( mtsOSGWAMExample             ${sawOpenSceneGraph_LIBRARIES} ) 
  target_link_libraries( mtsOSGBarrettHandExample     ${sawOpenSceneGraph_LIBRARIES} ) 

  target_link_libraries( svlOSGImageSequenceExample   ${sawOpenSceneGraph_LIBRARIES} ) 
  target_link_libraries( svlOSGCameraHUDExample       ${sawOpenSceneGraph_LIBRARIES} ) 

  cisst_target_link_libraries( osaOSGCameraMonoExample   ${REQUIRED_CISST_LIBRARIES} ) 
  cisst_target_link_libraries( osaOSGCameraStereoExample ${REQUIRED_CISST_LIBRARIES} ) 
  cisst_target_link_libraries( osaOSGCameraHUDExample ${REQUIRED_CISST_LIBRARIES} ) 
  cisst_target_link_libraries( osaOSGWAMExample          ${REQUIRED_CISST_LIBRARIES} ) 
  cisst_target_link_libraries( osaOSGBarrettHandExample  ${REQUIRED_CISST_LIBRARIES} ) 
  cisst_target_link_libraries( osaOSGPickExample         ${REQUIRED_CISST_LIBRARIES} ) 

  cisst_target_link_libraries( osaOSGImageSequenceExample         ${REQUIRED_CISST_LIBRARIES} ) 

  cisst_target_link_libraries( mtsOSGCameraMonoExample   ${REQUIRED_CISST_LIBRARIES} ) 
  cisst_target_link_libraries( mtsOSGCameraStereoExample ${REQUIRED_CISST_LIBRARIES} ) 
  cisst_target_link_libraries( mtsOSGImageSequenceExample ${REQUIRED_CISST_LIBRARIES} ) 
  cisst_target_link_libraries( mtsOSGWAMExample          ${REQUIRED_CISST_LIBRARIES} ) 
  cisst_target_link_libraries( mtsOSGBarrettHandExample  ${REQUIRED_CISST_LIBRARIES} ) 

  cisst_target_link_libraries( svlOSGImageSequenceExample ${REQUIRED_CISST_LIBRARIES} ) 
  cisst_target_link_libraries( svlOSGCameraHUDExample     ${REQUIRED_CISST_LIBRARIES} ) 

endif( cisst_FOUND_AS_REQUIRED )

