
set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstRobot
     cisstNumerical
     cisstMultiTask
     cisstParameterTypes)

set(
  REQUIRED_OPENSCENEGRAPH_LIBRARIES
  osgDB 
  osgGA 
  osgViewer
  osgUtil 
  osgVolume 
  osgText )

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
find_package(OpenSceneGraph COMPONENTS ${REQUIRED_OPENSCENEGRAPH_LIBRARIES} )
find_package( OpenCV2 )

if( cisst_FOUND_AS_REQUIRED AND OPENSCENEGRAPH_FOUND )

  if( OpenCV2_FOUND )
    set( SAW_OPENSCENEGRAPH_SUPPORTS_OPENCV ON )
    mark_as_advanced( FORCE ${OpenCV2_OPTIONS})
  endif( OpenCV2_FOUND )
  
  SET( 
    HEADER_FILES 
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
    ../mtsOSGMono.h
    ../mtsOSGStereo.h

    ../mtsOSGBH.h
    ../mtsOSGManipulator.h
    ../mtsOSGManipulatorTask.h
    )
    
    SET(
      SOURCE_FILES
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

      mtsOSGManipulatorTask.cpp
      )
    

    set( 
      SAW_OPENSCENEGRAPH_INCLUDE_DIRS 
      ${INCLUDE_OUTPUT_PATH} 
      ${OPENSCENEGRAPH_INCLUDE_DIRS} )
    set( SAW_OPENSCENEGRAPH_LINK_LIBRARIES ${OPENSCENEGRAPH_LIBRARIES} )

    if( OpenCV2_FOUND )
      set( 
	SAW_OPENSCENEGRAPH_INCLUDE_DIRS
	${SAW_OPENSCENEGRAPH_INCLUDE_DIRS}
	${OpenCV2_INCLUDE_DIRS} )
      set( 
	SAW_OPENSCENEGRAPH_LINK_LIBRARIES
	${SAW_OPENSCENEGRAPH_LINK_LIBRARIES}
	${OpenCV2_LIBRARIES} )
    endif( OpenCV2_FOUND )

    include_directories( ${CISST_INCLUDE_DIR} ${SAW_OPENSCENEGRAPH_INCLUDE_DIRS} )

    add_library( sawOpenSceneGraph ${HEADER_FILES} ${SOURCE_FILES} )

    target_link_libraries( 
      sawOpenSceneGraph 
      ${SAW_OPENSCENEGRAPH_LINK_LIBRARIES} )

    cisst_target_link_libraries( sawOpenSceneGraph ${REQUIRED_CISST_LIBRARIES} )

    set( 
      sawOpenSceneGraph_INCLUDE_DIR 
      "${sawComponents_SOURCE_DIR}" 
      "${sawComponents_BINARY_DIR}"
      ${SAW_OPENSCENEGRAPH_INCLUDE_DIRS} )

    set(sawOpenSceneGraph_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
    set( 
      sawOpenSceneGraph_LIBRARIES 
      sawOpenSceneGraph 
      ${SAW_OPENSCENEGRAPH_LINK_LIBRARIES} )

    configure_file (
      "${CMAKE_CURRENT_SOURCE_DIR}/sawOpenSceneGraphConfig.cmake.in"
      "${sawComponents_BINARY_DIR}/sawOpenSceneGraphConfig.cmake"
      @ONLY@)

    configure_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/sawOpenSceneGraphConfig.h.in"
      "${sawComponents_BINARY_DIR}/include/sawOpenSceneGraph/sawOpenSceneGraphConfig.h"
      @ONLY)

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
