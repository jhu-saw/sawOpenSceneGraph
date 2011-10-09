#ifndef _osaOSGPick_h
#define _osaOSGPick_h

#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>

#include <sawOpenSceneGraph/osaOSGBody.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

class CISST_EXPORT osaOSGPick : public osgGA::GUIEventHandler{

 protected:

  double mousex;
  double mousey;

  bool Pick( double x, double y, osgViewer::Viewer* viewer );

  virtual void PickHandler( osaOSGBody* body ){}

 public:

  osaOSGPick();

  // OSG virtual method
  bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa );

};

#endif
