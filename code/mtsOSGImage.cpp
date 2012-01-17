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

#include <sawOpenSceneGraph/mtsOSGImage.h>
#include <cisstMultiTask/mtsMatrix.h>

mtsOSGImage::Image::Image( float x, float y,  
			   float width, float height, 
			   osaOSGWorld* world ):
  osaOSGImage( x, y, width, height, world, vctFrame4x4<double>() ){}

mtsOSGImage::Image::Image( float x, float y,  
			   float width, float height, 
			   osaOSGHUD* hud ):
  osaOSGImage( x, y, width, height, hud, vctFrame4x4<double>() ){}


void mtsOSGImage::Image::UpdateImage(){
  if( mtsImage != NULL )
    { mtsImage->UpdateImage(); }
}

mtsOSGImage::mtsOSGImage( const std::string& name, 
			  float x, float y,
			  float width, float height,
			  osaOSGWorld* world ) : 
  mtsTaskContinuous( name ){

  image = new mtsOSGImage::Image( x, y, width, height, world );
  image->mtsImage = this;

  // Create the IO interface and add read/write commands
  input = AddInterfaceRequired( "Input" );//, MTS_OPTIONAL );
  if( input )
    { input->AddFunction( "GetImage", GetImage ); }
  else{
    CMN_LOG_CLASS_RUN_ERROR << "Failed to create the interface Input"
                            << std::endl;
  }

}

void mtsOSGImage::UpdateImage(){
  
  if( GetImage.IsValid() ){

    // Get the position of the camera
    mtsUCharMat mtsimg;
    GetImage( mtsimg );
    
    bool valid = false;
    mtsimg.GetValid( valid );
    if( valid ){

      // Set the transformation
      osg::ref_ptr<osg::Image> osgimg = new osg::Image();
      osgimg->setImage( mtsimg.width()/3,
			mtsimg.height(),
			1,
			3,
			GL_RGB,
			GL_UNSIGNED_BYTE,
			mtsimg.Pointer(),
			osg::Image::NO_DELETE );

      image->SetImage( osgimg );
      
    }

    valid = false;
    mtsimg.SetValid( valid );

  }

}

#include <cisstOSAbstraction/osaSleep.h>
void mtsOSGImage::Startup(){}
void mtsOSGImage::Run(){ ProcessQueuedCommands(); osaSleep(1.0/30.0); }
void mtsOSGImage::Cleanup(){}


