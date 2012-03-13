
#ifndef _svlOSGImage_h
#define _svlOSGImage_h

#include <cisstStereoVision/svlFilterBase.h>
#include <sawOpenSceneGraph/osaOSGImage.h>
#include <sawOpenSceneGraph/osaOSGHUD.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

class CISST_EXPORT svlOSGImage : public svlFilterBase {

  CMN_DECLARE_SERVICES( CMN_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT );
  
 public:

  svlOSGImage();
  svlOSGImage( float x, float y, 
	       float width, float height,
	       osaOSGWorld* world );
  svlOSGImage( float x, float y, 
	       float width, float height,
	       osaOSGHUD* hud );
  virtual ~svlOSGImage();

  void setNodeMask( osg::Node::NodeMask mask );

 protected:

  virtual int Initialize( svlSample* syncInput, svlSample*& syncOutput );
  virtual int Process( svlProcInfo* procInfo, 
		       svlSample* syncInput, 
		       svlSample* &syncOutput );
  virtual int Release();

 private:

  // Derive osaOSGImage and overload UpdateImate
  class Image : public osaOSGImage{
  protected:
    virtual void UpdateImage();
  public:
    svlOSGImage* svlImage;
    Image( float x, float y,  float width, float height, osaOSGWorld* world );
    Image( float x, float y,  float width, float height, osaOSGHUD* hud );
    ~Image();
  };

  osg::ref_ptr<svlOSGImage::Image> image;
  void UpdateImage();

};

CMN_DECLARE_SERVICES_INSTANTIATION_EXPORT( svlOSGImage )

#endif

