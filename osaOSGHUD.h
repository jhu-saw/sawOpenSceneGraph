

#ifndef _osaOSGHUD_h
#define _osaOSGHUD_h

#include <sawOpenSceneGraph/sawOpenSceneGraphConfig.h>

#include <cisstVector/vctDynamicNArray.h>
#include <sawOpenSceneGraph/osaOSGCamera.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>


class CISST_EXPORT osaOSGHUD : public osg::Camera {

 private:

  osg::ref_ptr<osg::Group> group;


 public : 

  //! Create an OSG mono camera (master camera)
  /**
     Create an OSG mono wrapped in a MTS continuous task. The camera also
     creates an MTS required interface called "Transformation" if a function
     name is provided. This function is used to update the position of the
     camera at each update traversal.
     \param world The world the camera belongs to
     \param width The width of the camera image
  */
  osaOSGHUD( osaOSGWorld* world, int width, int height, osaOSGCamera* camera );
  
  ~osaOSGHUD();

#ifdef SAW_OPENSCENEGRAPH_SUPPORTS_OPENCV

 public:

  osaOSGCamera::Errno GetRangeData( vctDynamicMatrix<double>& rangedata );
  osaOSGCamera::Errno GetRGBPlanarImage( vctDynamicNArray<unsigned char,3>& rgb );
  osaOSGCamera::Errno GetRGBImage( cv::Mat& rgb );
  osaOSGCamera::Errno GetDepthImage( cv::Mat& depth );

#endif
  
};

#endif
