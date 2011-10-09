

#ifndef _osaOSGMono_h
#define _osaOSGMono_h

#include <sawOpenSceneGraph/sawOpenSceneGraphConfig.h>

#if CISST_OSG_OPENCV
#include <opencv2/opencv.hpp>
#endif

#include <cisstVector/vctDynamicNArray.h>
#include <sawOpenSceneGraph/osaOSGCamera.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>


class CISST_EXPORT osaOSGMono : public osaOSGCamera {

 private:

  //! X pixel coordinate of the window's top left corner
  int x;
  //! Y pixel coordinate of the window's top left corner
  int y;
  //! Image width
  int width;
  //! Image height
  int height;

 public : 

  //! Create an OSG mono camera (master camera)
  /**
     Create an OSG mono wrapped in a MTS continuous task. The camera also
     creates an MTS required interface called "Transformation" if a function
     name is provided. This function is used to update the position of the
     camera at each update traversal.
     \param world The world the camera belongs to
     \param x The X offset of the camera window
     \param y The Y offset of the camera window
     \param width The width of the camera image
     \param height The height of the camera image
     \param fovy The field of view angle
     \param aspectRatio The aspect ratio of the camera
     \param zNear The near buffer distance
     \param zFar  The far buffer distance
     \param trackball Create the default trackball
     \param offscreenrendering Set to true if you want to render off screen
  */
  osaOSGMono( osaOSGWorld* world,
		int x, int y, int width, int height,
		double fovy, double aspectRatio,
		double zNear, double zFar,
		bool trackball = true,
		bool offscreenrendering = false );
  
  ~osaOSGMono();

  void Initialize();
    
#if CISST_OSG_OPENCV

 public:

  osaOSGCamera::Errno GetRangeData( vctDynamicMatrix<double>& rangedata );
  osaOSGCamera::Errno GetRGBPlanarImage( vctDynamicNArray<unsigned char,3>& rgb );
  osaOSGCamera::Errno GetRGBImage( cv::Mat& rgb );
  osaOSGCamera::Errno GetDepthImage( cv::Mat& depth );

#endif
  
};

#endif