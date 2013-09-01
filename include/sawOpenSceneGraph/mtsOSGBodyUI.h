#ifndef _mtsOSGBodyUI_h
#define _mtsOSGBodyUI_h

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Shape>

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <sawOpenSceneGraph/osaOSGBodyUI.h>
#include <sawOpenSceneGraph/sawOpenSceneGraphExport.h>

class CISST_EXPORT mtsOSGPointer3D : public mtsComponent {

 protected:

  //! Derive the base pointer class and add mts stuff
  class Pointer : public osaOSGPointer3D {

  protected:

    //! This update method is called from the OSG callback
    virtual void UpdateTransform();
    
  public:

    vctFixedSizeVector<double,3> told;

    //! The input interface
    mtsInterfaceRequired* input;
    mtsFunctionRead GetPosition;
    mtsFunctionRead GetVelocity;
    mtsFunctionRead GetButtons;
    
    Pointer( osaOSGWorld* world, 
	     const vctFrame4x4<double>& Rt,
	     const std::string& name,
	     double scale=1.0, 
	     double alpha=1.0 ) :
      osaOSGPointer3D( world, Rt, name, scale, alpha ),
      input( NULL ){}
    
  };


  Pointer pointer;

 public:
  
  mtsOSGPointer3D( const std::string& name,
		   osaOSGWorld* world, 
		   const vctFrame4x4<double>& Rt,
		   double scale=1.0, 
		   double alpha=1.0 );
  
};

class CISST_EXPORT mtsOSGBodyUI : public mtsComponent {

 protected:

  class BodyUI : public osaOSGBodyUI {

  protected:

    //! This update method is called from the OSG callback
    virtual void UpdateTransform();

  public:

    mtsInterfaceRequired* output;
    mtsFunctionWrite SetPositionCartesian;

    BodyUI( const std::string& model,
	    osaOSGWorld* world,
	    const vctFrame4x4<double>& Rt = vctFrame4x4<double>(),
	    double scale = 1.0,
	    double alpha = 1.0,
	    const vctFrame4x4<double>& Rtoffset = vctFrame4x4<double>(),
	    const std::string& option = std::string("") );
      
  };

  BodyUI bodyui;

 public:
  
  mtsOSGBodyUI( const std::string& name,
		const std::string& model,
		osaOSGWorld* world,
		const vctFrame4x4<double>& Rt = vctFrame4x4<double>(),
		double scale = 1.0,
		double alpha = 1.0,
		const vctFrame4x4<double>& Rtoffset = vctFrame4x4<double>(),
		const std::string& option = std::string("") );
  
};

#endif
