/*

  Author(s): Simon Leonard
  Created on: Dec 02 2009

  (C) Copyright 2009 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsOSGImage_h
#define _mtsOSGImage_h

#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <sawOpenSceneGraph/osaOSGImage.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

class CISST_EXPORT mtsOSGImage : public mtsTaskContinuous{
  
 private:

  //! The input interface
  mtsInterfaceRequired* input;
  mtsFunctionRead GetImage;

  // Derive osaOSGImage and overload UpdateImate
  class Image : public osaOSGImage{
  protected:
    virtual void UpdateImage();
  public:
    mtsOSGImage* mtsImage;
    Image( float x, float y,  float width, float height, osaOSGWorld* world );
    Image( float x, float y,  float width, float height, osaOSGHUD* hud );
    ~Image(){}
  };

  osg::ref_ptr<mtsOSGImage::Image> image;
  void UpdateImage();

 public:
  
  // Main constructor
  mtsOSGImage( const std::string& name, 
	       float x, float y,
	       float width, float height,
	       osaOSGWorld* world );
  ~mtsOSGImage(){}

  void Startup();
  void Run();
  void Cleanup();
  
};

#endif
