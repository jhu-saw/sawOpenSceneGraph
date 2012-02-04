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

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstRobot
     cisstNumerical
     cisstMultiTask
     cisstParameterTypes
     cisstStereoVision )

set(
  REQUIRED_OPENSCENEGRAPH_LIBRARIES
  osgDB
  osgGA
  osgViewer
  osgText
  osgUtil )
#  osgVolume
#  osgText )

find_package( cisst REQUIRED ${REQUIRED_CISST_LIBRARIES} )
find_package( OpenSceneGraph REQUIRED ${REQUIRED_OPENSCENEGRAPH_LIBRARIES} )
find_package( OpenCV2 )

if( cisst_FOUND_AS_REQUIRED AND OPENSCENEGRAPH_FOUND )

  include (${CISST_USE_FILE})

  SET(HEADER_FILES
    ${HEADER_FILES}

    ../osaOSGBH.h
    ../osaOSGBody.h
    ../osaOSGCamera.h
    ../osaOSGHUD.h
    ../osaOSGImage.h
    ../osaOSGManipulator.h
    ../osaOSGMono.h
    ../osaOSGPick.h
    ../osaOSGStereo.h
    ../osaOSGWorld.h

    ../mtsOSGBody.h

    ../mtsOSGCameraTask.h
    ../mtsOSGImage.h
    ../mtsOSGMono.h
    ../mtsOSGStereo.h

    ../mtsOSGBH.h
    ../mtsOSGManipulator.h
    ../mtsOSGManipulatorTask.h

    ../svlOSGImage.h
    )

  SET(SOURCE_FILES
    ${SOURCE_FILES}

    osaOSGBH.cpp
    osaOSGBody.cpp
    osaOSGCamera.cpp
    osaOSGHUD.cpp
    osaOSGImage.cpp
    osaOSGManipulator.cpp
    osaOSGMono.cpp
    osaOSGPick.cpp
    osaOSGStereo.cpp
    osaOSGWorld.cpp

    mtsOSGBody.cpp

    mtsOSGCameraTask.cpp
    mtsOSGImage.cpp

    mtsOSGManipulatorTask.cpp

    svlOSGImage.cpp
    )

  # message( ${OPENSCENEGRAPH_INCLUDE_DIRS} )

  set(SAW_OPENSCENEGRAPH_INCLUDE_DIRS
    ${INCLUDE_OUTPUT_PATH}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
    )

  set(SAW_OPENSCENEGRAPH_LINK_LIBRARIES
    ${OPENSCENEGRAPH_LIBRARIES}
    )

  cisst_cmake_debug("SAW_OPENSCENEGRAPH_INCLUDE_DIRS: ${SAW_OPENSCENEGRAPH_INCLUDE_DIRS}")

  # Enable OpenCV integration
  if( OpenCV2_FOUND )
    mark_as_advanced( FORCE ${OpenCV2_OPTIONS})

    #set( SAW_OPENSCENEGRAPH_SUPPORTS_OPENCV ON )

    #list(APPEND SAW_OPENSCENEGRAPH_INCLUDE_DIRS ${OpenCV2_INCLUDE_DIRS} )
    #list(APPEND SAW_OPENSCENEGRAPH_LINK_LIBRARIES ${OpenCV2_LIBRARIES} )
  endif( OpenCV2_FOUND )

  # Configure build targets
  include_directories(
    ${CISST_INCLUDE_DIR}
    ${SAW_OPENSCENEGRAPH_INCLUDE_DIRS}
    )

  add_library( sawOpenSceneGraph ${HEADER_FILES} ${SOURCE_FILES} )

  target_link_libraries( sawOpenSceneGraph ${SAW_OPENSCENEGRAPH_LINK_LIBRARIES} )

  cisst_target_link_libraries( sawOpenSceneGraph ${REQUIRED_CISST_LIBRARIES} )

  # Set parameters for source config file generation
  set( sawOpenSceneGraph_INCLUDE_DIR "${sawComponents_SOURCE_DIR}" "${sawComponents_BINARY_DIR}" ${SAW_OPENSCENEGRAPH_INCLUDE_DIRS} )
  set( sawOpenSceneGraph_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set( sawOpenSceneGraph_LIBRARIES sawOpenSceneGraph ${SAW_OPENSCENEGRAPH_LINK_LIBRARIES} )

  # Generate config files
  configure_file (
    "${CMAKE_CURRENT_SOURCE_DIR}/sawOpenSceneGraphConfig.cmake.in"
    "${sawComponents_BINARY_DIR}/sawOpenSceneGraphConfig.cmake"
    @ONLY)

  # Generate header with package configuration
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/sawOpenSceneGraphConfig.h.in"
    "${sawComponents_BINARY_DIR}/include/sawOpenSceneGraph/sawOpenSceneGraphConfig.h"
    @ONLY)


  # Install targets for headers and library
  install( FILES ${HEADER_FILES}
    DESTINATION include/sawOpenSceneGraph)

  install( FILES "${sawComponents_BINARY_DIR}/include/sawOpenSceneGraph/sawOpenSceneGraphConfig.h"
    DESTINATION include/sawOpenSceneGraph)

  install( TARGETS sawOpenSceneGraph
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

  install( FILES "${sawComponents_BINARY_DIR}/sawOpenSceneGraphConfig.cmake"
    DESTINATION ${CISST_CMAKE_MODULES_INSTALL_SUFFIX}/components)

  # Hide advanced options
  mark_as_advanced(
    OPENTHREADS_INCLUDE_DIR
    OPENTHREADS_LIBRARY
    OPENTHREADS_LIBRARY_DEBUG
    OSGDB_INCLUDE_DIR
    OSGDB_LIBRARY
    OSGDB_LIBRARY_DEBUG
    OSGGA_INCLUDE_DIR
    OSGGA_LIBRARY
    OSGGA_LIBRARY_DEBUG
    OSGTEXT_INCLUDE_DIR
    OSGTEXT_LIBRARY
    OSGTEXT_LIBRARY_DEBUG
    OSGUTIL_INCLUDE_DIR
    OSGUTIL_LIBRARY
    OSGUTIL_LIBRARY_DEBUG
    OSGVIEWER_INCLUDE_DIR
    OSGVIEWER_LIBRARY
    OSGVIEWER_LIBRARY_DEBUG
    OSGVOLUME_INCLUDE_DIR
    OSGVOLUME_LIBRARY
    OSGVOLUME_LIBRARY_DEBUG
    OSG_INCLUDE_DIR
    OSG_LIBRARY
    OSG_LIBRARY_DEBUG
    )

endif( cisst_FOUND_AS_REQUIRED AND OPENSCENEGRAPH_FOUND )
