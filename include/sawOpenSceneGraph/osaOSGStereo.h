/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#ifndef _osaOSGStereo_h
#define _osaOSGStereo_h

#include <cisstVector/vctDynamicNArray.h>
#include <sawOpenSceneGraph/osaOSGCamera.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

class CISST_EXPORT osaOSGStereo : public osaOSGCamera {

 public:
    
    enum Camera{ LEFT, RIGHT };
    
 private:
    
    //! X pixel coordinate of the window's top left corner
    int x;
    //! Y pixel coordinate of the window's top left corner
    int y;
    //! Image width
    int width;
    //! Image height
    int height;
    
    //! The baseline between the cameras
    double baseline;
    
 public : 
    
    //! Create a stereo OSG viewer (actually it's a OSG viewer)
    /**
       Create an stereo OSG viewer.
       \param world The world the camera belongs to
       \param x The X offset of the camera window
       \param y The Y offset of the camera window
       \param width The width of the camera image
       \param height The height of the camera image
       \param fovy The field of view angle
       \param aspectRatio The aspect ratio of the camera
       \param zNear The near buffer distance
       \param zFar  The far buffer distance
       \param baseline The base line between the two cameras
       \param trackball Use the default trackball
       \param Rtoffset Offset transformation of the left camera
    */
    osaOSGStereo( osaOSGWorld* world,
                  int x, int y, int width, int height,
                  double fovy, double aspectRatio,
                  double zNear, double zFar,
                  double baseline,
                  bool trackball = true,
                  const vctFrame4x4<double>& Rtoffset = vctFrame4x4<double>() );
    
    //! Create a stereo OSG viewer (actually it's a OSG viewer)
    /**
       Create an stereo OSG viewer.
       \param world The world the camera belongs to
       \param x The X offset of the camera window
       \param y The Y offset of the camera window
       \param width The width of the camera image
       \param height The height of the camera image
       \param Kl Intrinsic parameters of the left camera
       \param Kr Intrinsic parameters of the right camera
       \param Rtlr Position/orientation of the right camera wrt the left camera
       \param zNear The near buffer distance
       \param zFar  The far buffer distance
       \param baseline The base line between the two cameras
       \param trackball Use the default trackball
       \param Rtoffset Offset transformation of the left camera
    */
    osaOSGStereo( osaOSGWorld* world,
                  int x, int y, int width, int height,
                  const vctFixedSizeMatrix<double,3,3>& Kl,
                  const vctFixedSizeMatrix<double,3,3>& Kr,
                  const vctFrame4x4<double>& Rtlr,
                  double zNear, double zFar,
                  bool trackball = true,
                  const vctFrame4x4<double>& Rtoffset = vctFrame4x4<double>());

    ~osaOSGStereo( );
    
    void setCullMask( osg::Node::NodeMask mask, osaOSGStereo::Camera idx ){ 
        osg::View::Slave& slave = getSlave( idx );
        osg::ref_ptr<osg::Camera> camera = slave._camera.get();
        camera->setCullMask( mask ); 
    }
    
    virtual void Initialize( const std::string& name = std::string() );
    
#if 0
#ifdef SAW_OPENSCENEGRAPH_SUPPORTS_OPENCV
 public:
    //std::list< std::list< osaOSGBody* > > GetVisibilityList( size_t idx );
    vctDynamicMatrix<double> GetRangeData( size_t idx );
    vctDynamicNArray<unsigned char,3> GetRGBPlanarImage( size_t idx );
    cv::Mat GetRGBImage( size_t idx );
#endif
#endif
};

#endif
