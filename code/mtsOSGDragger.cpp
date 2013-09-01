#include <sawOpenSceneGraph/mtsOSGDragger.h>

mtsOSGPointer::mtsOSGPointer( const std::string& name,
			      osaOSGWorld* world, 
			      const vctFrame4x4<double>& Rt,
			      double scale, 
			      double alpha ) :

  mtsTaskPeriodic( name, 0.01, true ){
}


void mtsOSGPointer::Configure( const std::string& ) {}

void mtsOSGPointer::Startup(){}
void mtsOSGPointer::Run(){}
void mtsOSGPointer::Cleanup(){}

