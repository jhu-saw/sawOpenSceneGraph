#ifndef _osaOSGBodyUI_h
#define _osaOSGBodyUI_h

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Shape>

#include <sawOpenSceneGraph/osaOSGBody.h>
#include <sawOpenSceneGraph/osaOSGPick.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

// Handle for body dragger
class CISST_EXPORT osaOSGHandle : public osaOSGBody {

  enum State { FREE, SELECTED, DRAG };
  State state;

  osg::ref_ptr< osaOSGBody > body;

  osg::ref_ptr< osg::Geode > geode;
  osg::ref_ptr< osg::ShapeDrawable > drawable;
  osg::ref_ptr< osg::Sphere > sphere;

  // This is used to update the position of the body
  class ResetCallback : public osg::NodeCallback {    
  public:
    ResetCallback(){}
    void operator()( osg::Node* node, osg::NodeVisitor* nv );
  };
  osg::ref_ptr<ResetCallback> resetcallback;

 public:

  osaOSGHandle( osaOSGBody* body, 
		const std::string& name,
		const vctFixedSizeVector<double,3>& toh );
  
  osaOSGBody* GetBody(){ return body; }
  void SetColor();

  bool IsSelected() const { return state == SELECTED; }
  bool IsFree() const { return state == FREE; }

  void Reset();
  void Select();
  void Release();

  vctFrame4x4<double> Drag( const vctFixedSizeVector<double,3>& v );

};

// Handle for body dragger
class CISST_EXPORT osaOSGPointer2D : public osaOSGPick {

 protected:

  bool select;
  vctFixedSizeVector<double,3> xyzprev;
  osg::ref_ptr< osaOSGHandle > handle;

 public:

  osaOSGPointer2D() : select( false ), xyzprev( 0.0 ){}

  void PickHandler( osaOSGBody* body, osaOSGPick::Event e, double x, double y );

};

// Handle for body dragger
class CISST_EXPORT osaOSGPointer3D : public osaOSGBody {

 protected:

  osg::ref_ptr< osg::Geode > geode;
  osg::ref_ptr< osg::ShapeDrawable > drawable;
  osg::ref_ptr< osg::Sphere > sphere;

  // This is used to update the position of the body
  class IntersectionCallback : public osg::NodeCallback {    
  public:
    IntersectionCallback(){}
    void operator()( osg::Node* node, osg::NodeVisitor* nv );
  };
  osg::ref_ptr<IntersectionCallback> intersectioncallback;

  osg::ref_ptr< osaOSGWorld > world;

  vctFixedSizeVector<double,3> velocity;

  osg::Polytope polytope;
  void BuildPolytope();
  
  bool select;

 public:

  osaOSGPointer3D( osaOSGWorld* world, 
		   const vctFrame4x4<double>& Rt,
		   const std::string& name,
		   double scale=1.0, 
		   double alpha=1.0 );

  void CheckIntersection();

  void Select() { select = true; }
  void Release() { select = false; }
  void SetVelocity( const vctFixedSizeVector<double,3>& v );

};

class CISST_EXPORT osaOSGBodyUI : public osaOSGBody {

 protected:

  osg::ref_ptr< osaOSGHandle > top, bottom, left, right, front, rear;

 public:

  osaOSGBodyUI( const std::string& model,
		osaOSGWorld* world,
		const vctFrame4x4<double>& Rt = vctFrame4x4<double>(),
		double scale = 1.0,
		double alpha = 1.0,
		const vctFrame4x4<double>& Rtoffset = vctFrame4x4<double>(),
		const std::string& option = std::string("") );

};

#endif
