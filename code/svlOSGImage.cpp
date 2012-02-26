#include <sawOpenSceneGraph/svlOSGImage.h>

CMN_IMPLEMENT_SERVICES_DERIVED( svlOSGImage, svlFilterBase )

svlOSGImage::Image::Image( float x, float y,  
			   float width, float height, 
			   osaOSGWorld* world ):
osaOSGImage( x, y, width, height, world, vctFrame4x4<double>() ){}

svlOSGImage::Image::Image( float x, float y,  
			   float width, float height, 
			   osaOSGHUD* hud ):
osaOSGImage( x, y, width, height, hud, vctFrame4x4<double>() ){}

svlOSGImage::Image::~Image(){}

void svlOSGImage::Image::UpdateImage(){
  if( svlImage != NULL )
    { svlImage->UpdateImage(); }
}



svlOSGImage::svlOSGImage() :
  svlFilterBase(),
  image( NULL ){
  
  AddInput("input", true);
  AddInputType("input", svlTypeImageMono8);
  AddInputType("input", svlTypeImageMono8Stereo);

}

svlOSGImage::svlOSGImage( float x, float y,  
			  float width, float height, 
			  osaOSGWorld* world ):
  svlFilterBase(){
  
  AddInput("input", true);
  AddInputType("input", svlTypeImageMono8);
  AddInputType("input", svlTypeImageMono8Stereo);

  image = new svlOSGImage::Image( x, y, width, height, world );
  image->svlImage = this;


}

svlOSGImage::svlOSGImage( float x, float y,  
			  float width, float height, 
			  osaOSGHUD* hud ):
  svlFilterBase(){
  
  AddInput("input", true);
  AddInputType("input", svlTypeImageMono8);
  AddInputType("input", svlTypeImageMono8Stereo);

  image = new svlOSGImage::Image( x, y, width, height, hud );
  image->svlImage = this;

}

svlOSGImage::~svlOSGImage()
{ Release(); }

int svlOSGImage::Initialize( svlSample* syncInput, svlSample*& syncOutput ){
  Release();
  return SVL_OK;
}

int svlOSGImage::Process( svlProcInfo* procInfo, 
			  svlSample* syncInput, 
			  svlSample* &syncOutput ){
  syncOutput = syncInput;

  _SkipIfAlreadyProcessed( syncInput, syncOutput );
  _SkipIfDisabled();



  svlSampleImage* img = dynamic_cast<svlSampleImage*>(syncInput);
  unsigned int videochannels = img->GetVideoChannels();
    
  // Set the transformation
  osg::ref_ptr<osg::Image> osgimg = new osg::Image;
  osgimg->setImage( img->GetWidth( 0 ),
		    img->GetHeight( 0 ),
		    1,
		    3,
		    GL_BGR,
		    GL_UNSIGNED_BYTE,
		    img->GetUCharPointer( 0 ),
		    osg::Image::NO_DELETE );
  
  image->SetImage( osgimg );
  
  return SVL_OK;
}

int svlOSGImage::Release()
{ return SVL_OK; }


void svlOSGImage::UpdateImage(){}

void svlOSGImage::setNodeMask( osg::Node::NodeMask mask ){

  if( image != NULL ){
    image->setNodeMask( mask );
  }

}
