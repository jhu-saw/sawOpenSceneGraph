

#ifndef _osaOSGCamera_h
#define _osaOSGCamera_h

#include <cisstCommon/cmnPortability.h>

#include <sawOpenSceneGraph/sawOpenSceneGraphConfig.h>

#ifdef SAW_OPENSCENEGRAPH_SUPPORTS_OPENCV
//#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#endif

#include <osgViewer/Viewer>

#include <cisstVector/vctDynamicMatrix.h>
#include <cisstVector/vctTransformationTypes.h>
#include <cisstVector/vctFrame4x4.h>
#include <sawOpenSceneGraph/osaOSGWorld.h>

#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

class mtsOSGCamera;
class osaOSGHUD;

class CISST_EXPORT osaOSGCamera : public osgViewer::Viewer{

  friend class osaOSGHUD;

 public:

  enum Errno { ESUCCESS, EFAILURE };

 private:

  //! 
  vctFrame4x4<double> Rtoffset;

  //! Flag for offscreen rendering
  bool offscreenrendering;

  //! Update the position and orientation of the camera
  /**
     This method can be called to update the position and orientation of
     the camera. It is called from the update callback during the update 
     traversal.
  */
  virtual void UpdateTransform();
  osg::Matrixd osgtransform;

  //! Allow mtsOSGCameraTask to update this transformation
  friend class mtsOSGCameraTask;

 protected:

  bool IsOffscreenRenderingEnabled() const {return offscreenrendering;}

  //! User data for the camera
  /** 
      This class stores a pointer to a camera object. This pointer is is used
      during traversals to capture/process images and update the
      orientation/position of the camera.
  */
  class Data : public osg::Referenced {
  private:
    //! Pointer to a camera object
    osg::ref_ptr<osaOSGCamera> camera;
  public:
    //! Default constructor.
    Data( osaOSGCamera* camera ) : camera( camera ){}
    //! Get the pointer to the camera
    osaOSGCamera* GetCamera() { return camera; }
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




  // Only enable this if OpenCV2 is enabled
#ifdef SAW_OPENSCENEGRAPH_SUPPORTS_OPENCV

  //! Final drawing callback
  /**
     This class is used to capture/process images during the final drawing
     callback traversal.
  */
  class FinalDrawCallback : public osg::Camera::DrawCallback {
    
  public:

    // Data for the final draw callback. This data is used to copy images
    // outside the drawing traversal.
    class Data : public osg::Referenced {

    private:
      
      bool rangerequest;
      bool depthrequest;
      bool colorrequest;

      //! Converted 3D range data.
      /**
        Create a 3xN range data matrix.
	[ x1 ... xN ]
	[ y1 ... yN ]
	[ z1 ... zN ]
      */
      vctDynamicMatrix<double> rangedata;

      //! Depth image
      /**
	 This is a depth image, where at each pixel, you have the depth of 
	 the projected point.
      */
      cv::Mat depthimage;
      
      //! RGB image
      cv::Mat rgbimage;
      
    public:
      
      Data( size_t width, size_t height );
      ~Data();
      
      void RequestRangeData()       { rangerequest = true; }
      void RequestDepthImage()      { depthrequest = true; }
      void RequestRGBImage()        { colorrequest = true; }

      bool RangeDataRequested()       const { return rangerequest; }
      bool DepthImageRequested()      const { return depthrequest; }
      bool RGBImageRequested()        const { return colorrequest; }

      vctDynamicMatrix<double> GetRangeData() const;
      cv::Mat GetDepthImage() const;
      cv::Mat GetRGBImage() const;

      void SetRangeData( const vctDynamicMatrix<double>& rangedata );
      void SetDepthImage( const cv::Mat& depthimage );
      void SetRGBImage( const cv::Mat& rgbimage );

    }; // Data

  private:

    //! OSG image containing the depth buffer
    osg::ref_ptr<osg::Image> depthbufferimg;

    //! OSG image containing the color buffer
    osg::ref_ptr<osg::Image> colorbufferimg;
  
    //! Callback operator
    /**
       This callback is called during the final drawing traversal. This 
       operator calls the Capture method to grab frames from the depth and 
       color buffers.
    */
    virtual void operator () ( osg::RenderInfo& ) const;
    
    //! Convert the depth buffer to range data
    void ComputeRangeData( osg::Camera* camera ) const;

    //! Convert the depth buffer to a depth image
    void ComputeDepthImage( osg::Camera* camera ) const;

    //! Convert the color buffer to a color image
    void ComputeRGBImage( osg::Camera* camera ) const;

  public:
    
    //! Default constructor
    /**
       Constructor for a final drawing callback object. This object will
       determine the operation performed at each frame rendering.
       \param camera A pointer to an OSG camera
       \param capturedepth Read and convert depth buffer
       \param capturecolor Read and convert color buffer
    */
    FinalDrawCallback( osg::Camera* camera );
    
    ~FinalDrawCallback();
    
  }; // FinalDrawCallback

#endif // SAW_OPENSCENEGRAPH_SUPPORTS_OPENCV

  std::vector< osg::ref_ptr< osaOSGHUD > > huds;
  void AddHUD( osaOSGHUD* hud );

 public : 

  //! Create an OSG viewer (actually it's a OSG viewer)
  /**
     Create an OSG viewer wrapped in a MTS continuous task. The camera also
     creates an MTS required interface called "Transformation" if a function
     name is provided. This function is used to update the position of the
     camera at each update traversal.
     \param world The world the camera belongs to
     \param trackball Create the default trackball
     \param offscreenrendering Render in p-buffer
  */
  osaOSGCamera( osaOSGWorld* world,
		bool trackball = true,
		const vctFrame4x4<double>& Rtoffset = vctFrame4x4<double>(),
		bool offscreenrendering = false );

  ~osaOSGCamera();

  virtual void Initialize( const std::string& CMN_UNUSED(name)=std::string() ){}

  void setCullMask( osg::Node::NodeMask mask )
  { getCamera()->setCullMask( mask ); }

  void SetTransform( const vctFrm3& Rt );
  void SetTransform( const vctFrame4x4<double>& Rt );
  vctFrm3 GetTransform() const;  

};

#endif
