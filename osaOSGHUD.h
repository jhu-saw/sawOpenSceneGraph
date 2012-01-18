

#ifndef _osaOSGHUD_h
#define _osaOSGHUD_h


#include <sawOpenSceneGraph/sawOpenSceneGraphConfig.h>

#include <cisstVector/vctDynamicNArray.h>
#include <sawOpenSceneGraph/osaOSGCamera.h>
#include <sawOpenSceneGraph/mtsOSGCameraTask.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>


class CISST_EXPORT osaOSGHUD : public osg::Camera {

  friend class osaOSGMono;
  friend class osaOSGStereo;
  
 private:
  
  osg::ref_ptr< osg::Node > root;
  osg::ref_ptr< osgViewer::Viewer > viewer;
  
  void Initialize( osg::GraphicsContext* gc );

 public : 

  //! Create an Head Up Display
  /**
  */
  osaOSGHUD( osaOSGWorld* world, 
	     osaOSGCamera* camera );

  osaOSGHUD( osaOSGWorld* world,
	     mtsOSGCameraTask* camera );

  ~osaOSGHUD();


#ifdef SAW_OPENSCENEGRAPH_SUPPORTS_OPENCV

 public:

  osaOSGCamera::Errno GetRangeData( vctDynamicMatrix<double>& rangedata );
  osaOSGCamera::Errno GetRGBPlanarImage(vctDynamicNArray<unsigned char,3>& rgb);

  osaOSGCamera::Errno GetRGBImage( cv::Mat& rgb );
  osaOSGCamera::Errno GetDepthImage( cv::Mat& depth );

#endif
  
};

#endif
