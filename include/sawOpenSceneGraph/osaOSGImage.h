/*

  Author(s): Simon Leonard
  Created on: Nov 11 2009

  (C) Copyright 2008 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _osaOSGImage_h
#define _osaOSGImage_h

#include <osg/Image>
#include <osg/Texture2D>
#include <osg/Group>
#include <osg/Switch>
#include <osg/MatrixTransform>
#include <osg/TriangleFunctor>
#include <osg/PositionAttitudeTransform>
#include <osgUtil/TransformAttributeFunctor>

#include <cisstVector/vctTransformationTypes.h>

#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/osaOSGHUD.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

class CISST_EXPORT osaOSGImage : public osg::Group {

 public:
  
  enum Switch{ SWITCH_OFF, SWITCH_ON };
  
 protected:

  // Callback stuff

  // This is to store a pointer to the image
  class UserData : public osg::Referenced {
  private:
    osg::ref_ptr<osaOSGImage> imagesequence;
  public:
    UserData( osaOSGImage* imgseq ) : imagesequence( imgseq ){}
    osaOSGImage* GetImage() { return imagesequence; }
  };
  osg::ref_ptr<osg::Referenced> userdata;

  // This is used to update the position of the image
  class TransformCallback : public osg::NodeCallback {    
  public:
    TransformCallback(){}
    void operator()( osg::Node* node, osg::NodeVisitor* nv );
  };
  // The transform callback
  osg::ref_ptr<TransformCallback> transformcallback;

  //! This method is called from the transform callback
  virtual void UpdateTransform();

  // The vct transform
  vctFrame4x4<double> transform;

  // The osg transform
  osg::ref_ptr<osg::MatrixTransform> osgtransform;



  // This is used to update the position of the image
  class SwitchCallback : public osg::NodeCallback {    
  public:
    SwitchCallback(){}
    void operator()( osg::Node* node, osg::NodeVisitor* nv );
  };
  // The switch callback
  osg::ref_ptr<SwitchCallback> switchcallback;

  //! This method is called from the switch callback
  virtual void UpdateSwitch();

  // The switch
  Switch onoff;

  // The switch
  osg::ref_ptr< osg::Switch> osgswitch;

 protected:

  // in order for the image to update itself at each traversal we need to
  // a new class and overloads the following methods
  class Image : public osg::Image {

    bool requiresUpdateCall() const { return true; }

    void update( osg::NodeVisitor* ){ 
      osg::Referenced* data = getUserData();
      osaOSGImage::UserData* userdata;
      userdata = dynamic_cast<osaOSGImage::UserData*>( data );

      // update the image
      if( userdata != NULL )
      { userdata->GetImage()->UpdateImage(); }
    }

  };
  

  float x, y;
  float width, height;
  unsigned char*                     data;
  osg::ref_ptr<osg::Image>       osgimage;
  osg::ref_ptr<osg::Texture2D> osgtexture;
  osg::ref_ptr<osg::StateSet> osgstateset;
  osg::ref_ptr<osg::Geode>       osggeode;
  osg::ref_ptr<osg::Geometry>     osggeom;

  void Initialize();

  virtual void UpdateImage(){}

 public: 

  //! OSG Image constructor
  /**
     Create a OSG image.
     \param x The horizontal position of the image
     \param y The vertical position of the image
     \param width The width of the rendered geode (not of the image in pixels)
     \param height The height of the rendered geode (not of the image in pixels)
     \param world The OSG world the image belongs to
     \param Rt The initial transformation of the image
  */
  osaOSGImage( float x,
	       float y, 
	       float width, 
	       float height,
	       osaOSGWorld* world, 
	       const vctFrame4x4<double>& Rt = vctFrame4x4<double>() );

  //! OSG Image constructor
  /**
     Create a OSG image.
     \param x The horizontal position of the image
     \param y The vertical position of the image
     \param width The width of the rendered geode (not of the image in pixels)
     \param height The height of the rendered geode (not of the image in pixels)
     \param hud The OSG HUD the image belongs to
     \param Rt The initial transformation of the image
  */
  osaOSGImage( float x,
	       float y, 
	       float width, 
	       float height,
	       osaOSGHUD* hud, 
	       const vctFrame4x4<double>& Rt = vctFrame4x4<double>() );

  //! OSG Image constructor
  /**
     Create a OSG image.
     \param x The horizontal position of the image
     \param y The vertical position of the image
     \param width The width of the rendered geode (not of the image in pixels)
     \param height The height of the rendered geode (not of the image in pixels)
     \param world The OSG world the image belongs to
     \param Rt The initial transformation of the image
  */
  osaOSGImage( float x,
	       float y, 
	       float width, 
	       float height,
	       osaOSGWorld* world, 
	       const vctFrm3& Rt );

  ~osaOSGImage();

  //! Set the transform of the image
  virtual void SetTransform( const vctFrame4x4<double>& Rt );
  virtual void SetTransform( const vctFrm3& Rt );
  virtual vctFrm3 GetTransform() const;

  void SetImage( const osg::Image* image );
  void SetImage( const std::string& filename );

  //! Set the switch of the image
  void SwitchOn();
  void SwitchOff();
  
};

#endif
