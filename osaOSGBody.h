
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

#ifndef _osaOSGBody_h
#define _osaOSGBody_h

#include <osg/Group>
#include <osg/Switch>
#include <osg/MatrixTransform>
#include <osg/TriangleFunctor>
#include <osg/PositionAttitudeTransform>
#include <osgUtil/TransformAttributeFunctor>

#include <cisstVector/vctTransformationTypes.h>

#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

class CISST_EXPORT osaOSGBody : public osg::Group {

 public:

  enum Switch{ SWITCH_OFF, SWITCH_ON };

 protected:

  // This class is used to extract the triangle mesh out of the OSG
  // classes/structures. It is a geode visitor that traverse the drawable 
  // objects and extract all the triangles from all the drawables
  class GeodeVisitor : public osg::NodeVisitor {
    
  private:

    // Create a structure to hold a triangle
    struct Triangle
    { osg::Vec3 p1, p2, p3; };

    // For each drawable, a TriangleExtractor object is created. The operator() 
    // is called for each triangle of the drawable
    struct TriangleExtractor {

      // The list of triangles for a drawable
      std::vector< osaOSGBody::GeodeVisitor::Triangle > drawabletriangles;

      // This method is called for each triangle of the drawable. All it does
      // is to copy the vertices to the vector
      inline void operator ()( const osg::Vec3& p1, 
			       const osg::Vec3& p2, 
			       const osg::Vec3& p3, 
			       bool treatVertexDataAsTemporary );
    };
    
  public:
    
    // the list of triangles for the geode (composed of several drawables)
    std::vector< osaOSGBody::GeodeVisitor::Triangle > geodetriangles;
    
    // Default constructor
    GeodeVisitor();

    // This method is called for each geode. It scans all the drawable of the 
    // geode and extract/copy the triangles to the triangle vector
    virtual void apply( osg::Geode& geode );
    
  };  // GeodeVisitor


  // Callback stuff

  // This is to store a pointer to the body
  class UserData : public osg::Referenced {
  private:
    osg::ref_ptr<osaOSGBody> body;
  public:
    UserData( osaOSGBody* body ) : body( body ){}
    osaOSGBody* GetBody() { return body; }
  };
  osg::ref_ptr<UserData> userdata;

  // A scaling factor
  osg::ref_ptr<osg::PositionAttitudeTransform> osgscale;

  // This is used to update the position of the body
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


  
  // This is used to update the position of the body
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



  // I/O
  void ReadModel( const std::string& fname,
		  const std::string& options );

  void Read3DData( const vctDynamicMatrix<double>& pc,
		   const vctFixedSizeVector<unsigned char,3>& RGB = RGBDEFAULT,
		   float size = 3.0 );

 public: 

  static const vctFixedSizeVector<unsigned char,3> RGBDEFAULT;

  //! OSG Body constructor
  /**
     Create a OSG body component. The body will add a required interface *if*
     a function name is passed.
     \param model The file name of a 3D model
     \param Rt The initial transformation of the body
  */
  osaOSGBody( const std::string& model,
		const vctFrame4x4<double>& Rt,
		double scale = 1.0,
		const std::string& option = std::string("") );

  //! OSG Body constructor
  /**
     Create a OSG body component. The body will add a required interface *if*
     a function name is passed.
     \param model The file name of a 3D model
     \param world The OSG world the body belongs to
     \param Rt The initial transformation of the body
  */
  osaOSGBody( const std::string& model,
		osaOSGWorld* world,
		const vctFrame4x4<double>& Rt,
		double scale = 1.0,
		const std::string& option = std::string("") );

  //! OSG Body constructor
  /**
     Create a OSG body component. The body will add a required interface *if*
     a function name is passed.
     \param model The file name of a 3D model
     \param world The OSG world the body belongs to
     \param Rt The initial transformation of the body
  */
  osaOSGBody( const std::string& model,
		osaOSGWorld* world,
		const vctFrm3& Rt,
		double scale = 1.0,
		const std::string& option = std::string("") );


  //! Construcor for 3D point cloud
  osaOSGBody( const vctDynamicMatrix<double>& pointcloud,
		osaOSGWorld* world,
	        const vctFrm3& Rt,
		const vctFixedSizeVector<unsigned char,3>& rgb=RGBDEFAULT,
		float size = 3.0 );

  ~osaOSGBody();

  void Initialize( double scale = 1.0 );

  //! Set the transform of the body
  virtual void SetTransform( const vctFrame4x4<double>& Rt );
  virtual void SetTransform( const vctFrm3& Rt );
  virtual vctFrm3 GetTransform() const;


  //! Set the switch of the body
  void SwitchOn();
  void SwitchOff();
  
  void SetModeLine();
  void SetModePoint();
  void SetModeFill();

  virtual vctDynamicMatrix<double> GetVertices();

};

#endif
