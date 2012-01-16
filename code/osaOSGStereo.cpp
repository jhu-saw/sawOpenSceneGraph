#include <sawOpenSceneGraph/osaOSGStereo.h>

osaOSGStereo::osaOSGStereo( osaOSGWorld* world,
			    int x, int y, int width, int height,
			    double fovy, double aspectRatio,
			    double zNear, double zFar,
			    double baseline, 
			    bool trackball,
			    const vctFrame4x4<double>& Rtoffset ) :
  osaOSGCamera( world, trackball, Rtoffset ),
  x( x ),                              // x position
  y( y ),                              // y position
  width( width ),                      // width of images
  height( height ),
  baseline( baseline ){

  // Set the intrinsic paramters
  getCamera()->setProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);

  // Setup the left camera
  {
    // Create a new (slave) camera
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;

    // Create the view port. Again, the reason why the viewport is created here
    // is because the SVL stuff in the final draw callback needs to be created
    // during the constructor
    camera->setViewport( new osg::Viewport( 0, 0, width, height) );

    // add this slave camera to the viewer, with a shift left of the
    // projection matrix
    addSlave( camera.get(), 
	      osg::Matrixd(),
	      osg::Matrixd::translate( baseline/2.0, 0.0, 0.0 ) );

    // Only do drawing callback if OpenCV is enabled
#ifdef SAW_OPENSCENEGRAPH_SUPPORTS_OPENCV
    // Create a drawing callback. This callback is set to capture depth+color 
    // buffer (true, true)
    osg::ref_ptr<osaOSGCamera::FinalDrawCallback> finaldrawcallback;
    try{ finaldrawcallback =  new FinalDrawCallback( camera ); }
    catch( std::bad_alloc& ){
      CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
			<< "Failed to allocate FinalDrawCallback."
			<< std::endl;
    }
    CMN_ASSERT( finaldrawcallback );
    camera->setFinalDrawCallback( finaldrawcallback );
#endif //SAW_OPENSCENEGRAPH_SUPPORTS_OPENCV

  }


  // setup the right camera
  {
    // Create a new (slave) camera
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;

    // Create the view port. Again, the reason why the viewport is created here
    // is because the SVL stuff in the final draw callback needs to be created
    // during the constructor
    camera->setViewport( new osg::Viewport( 0, 0, width, height) );
    
    // add this slave camera to the viewer, with a shift right of the 
    // projection matrix                                  
    addSlave( camera.get(), 
	      osg::Matrixd(),
	      osg::Matrixd::translate( -baseline/2.0, 0.0, 0.0 ) );

    // Only do drawing callback if OpenCV is enabled
#ifdef SAW_OPENSCENEGRAPH_SUPPORTS_OPENCV
    // Create a drawing callback. This callback is set to capture depth+color 
    // buffer (true, true)
    osg::ref_ptr<osaOSGCamera::FinalDrawCallback> finaldrawcallback;
    try{ finaldrawcallback =  new FinalDrawCallback( camera ); }
    catch( std::bad_alloc& ){
      CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
			<< "Failed to allocate FinalDrawCallback."
			<< std::endl;
    }
    CMN_ASSERT( finaldrawcallback );
    camera->setFinalDrawCallback( finaldrawcallback );
#endif // SAW_OPENSCENEGRAPH_SUPPORTS_OPENCV

  }

}

osaOSGStereo::~osaOSGStereo(){}

void osaOSGStereo::Initialize(){

  // Create the graphic context traits. The reason why this is here is because
  // Windows somehow requires the context to be allocated within the same thread
  // as the rendering thread. And this method is called within the camera thread
  {

    osg::ref_ptr<osg::GraphicsContext::Traits> traits;
    traits = new osg::GraphicsContext::Traits;
    traits->x = x + 0;
    traits->y = y + 0;
    traits->width = width;
    traits->height = height;
    traits->windowDecoration = true;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;

    // slave "0" is the first slave. In this case the "left" camera
    osg::View::Slave& slave = getSlave( osaOSGStereo::LEFT );
    osg::Camera* camera = slave._camera.get();

    osg::ref_ptr<osg::GraphicsContext> gc;
    gc = osg::GraphicsContext::createGraphicsContext( traits.get() );
    camera->setGraphicsContext( gc.get() );

    GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
    camera->setDrawBuffer( buffer );
    camera->setReadBuffer( buffer );
    camera->setClearColor( osg::Vec4d( 0.0, 0.0, 0.0, 0.0 ) );

  }

  {

    osg::ref_ptr<osg::GraphicsContext::Traits> traits;
    traits = new osg::GraphicsContext::Traits;
    traits->x = x + width;
    traits->y = y + 0;
    traits->width = width;
    traits->height = height;
    traits->windowDecoration = true;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;

    // slave "1" is the second slave. In this case the "right" camera
    osg::View::Slave& slave = getSlave( osaOSGStereo::RIGHT );
    osg::Camera* camera = slave._camera.get();

    osg::ref_ptr<osg::GraphicsContext> gc;
    gc = osg::GraphicsContext::createGraphicsContext( traits.get() );
    camera->setGraphicsContext( gc.get() );

    GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
    camera->setDrawBuffer(buffer);
    camera->setReadBuffer(buffer);
    camera->setClearColor( osg::Vec4d( 0.0, 0.0, 0.0, 0.0 ) );

  }

}

#if 0
#ifdef SAW_OPENSCENEGRAPH_SUPPORTS_OPENCV
#include <cisstOSAbstraction/osaSleep.h>
/*
std::list< std::list< osaOSGBody* > > osaOSGStereo::GetVisibilityList( size_t idx ){

  // Get the left/right slave
  const osg::View::Slave& slave = getSlave( idx );
  osg::Camera* camera = slave._camera.get();

  // get the camera final draw callback
  osg::Camera::DrawCallback* dcb = NULL;
  dcb = camera->getFinalDrawCallback();

  // 
  osg::ref_ptr<osg::Referenced> ref = dcb->getUserData();
  osg::ref_ptr<osaOSGCamera::FinalDrawCallback::Data> data = NULL;
  data = dynamic_cast<osaOSGCamera::FinalDrawCallback::Data*>( ref.get() );
  if( data != NULL ){
    data->RequestVisibilityList();
    while( data->VisibilityListRequested() ) {osaSleep( 1.0 );}
    return data->GetVisibilityList();
  }
  return std::list< std::list< osaOSGBody* > >();
}
*/

vctDynamicNArray<unsigned char,3> osaOSGStereo::GetRGBPlanarImage(size_t idx){

  // Get the left/right slave
  const osg::View::Slave& slave = getSlave( idx );
  osg::Camera* camera = slave._camera.get();

  // get the camera final draw callback
  osg::Camera::DrawCallback* dcb = NULL;
  dcb = camera->getFinalDrawCallback();

  osg::ref_ptr<osg::Referenced> ref = dcb->getUserData();
  osg::ref_ptr<osaOSGCamera::FinalDrawCallback::Data> data = NULL;
  data = dynamic_cast<osaOSGCamera::FinalDrawCallback::Data*>( ref.get() );
  if( data != NULL ){
    data->RequestRGBImage();
    osaSleep( 1.0 );
    cv::Mat rgbimage = data->GetRGBImage();
    cv::Size size = rgbimage.size();

    vctDynamicNArray<unsigned char, 3> x;
    x.SetSize( vctDynamicNArray<unsigned char, 3>::nsize_type( size.height, 
							       size.width, 3 ) );
    memcpy( x.Pointer(), 
	    rgbimage.ptr<unsigned char>(), 
	    size.height*size.width*3 );

    return x;
  }
  return vctDynamicNArray<unsigned char, 3>();
}

vctDynamicMatrix<double> osaOSGStereo::GetRangeData( size_t idx ) {

  // Get the left slave
  const osg::View::Slave& slave = getSlave( idx );
  osg::ref_ptr< osg::Camera > camera = slave._camera.get();

  osg::ref_ptr< osg::Camera::DrawCallback > dcb;
  dcb = camera->getFinalDrawCallback();
 
  // 
  osg::ref_ptr<osg::Referenced> ref = dcb->getUserData();
  osg::ref_ptr<osaOSGCamera::FinalDrawCallback::Data> data = NULL;
  data = dynamic_cast<osaOSGCamera::FinalDrawCallback::Data*>( ref.get() );
  if( data != NULL ){
    data->RequestRangeData();
    osaSleep( 1.0 );
    return data->GetRangeData();
  }
  return vctDynamicMatrix<double>();

}

cv::Mat osaOSGStereo::GetRGBImage( size_t idx ) {

  // Get the left/right slave
  const osg::View::Slave& slave = getSlave( idx );
  osg::ref_ptr< osg::Camera > camera = slave._camera.get();

  osg::ref_ptr< osg::Camera::DrawCallback > dcb;
  dcb = getCamera()->getFinalDrawCallback();

  // 
  osg::ref_ptr<osg::Referenced> ref = dcb->getUserData();
  osg::ref_ptr<osaOSGCamera::FinalDrawCallback::Data> data = NULL;
  data = dynamic_cast<osaOSGCamera::FinalDrawCallback::Data*>( ref.get() );
  if( data != NULL ){
    data->RequestRGBImage();
    osaSleep( 1.0 );
    return data->GetRGBImage();
  }
  return cv::Mat();

}


/*
vctDynamicMatrix<unsigned char> osaOSGStereo::GetRGBPixelImage(size_t idx) const{

  // Get the left/right slave
  const osg::View::Slave& slave = getSlave( idx );
  osg::Camera* camera = slave._camera.get();

  // get the camera final draw callback
  const osg::Camera::DrawCallback* dcb = NULL;
  dcb = camera->getFinalDrawCallback();

  // cast
  const osaOSGCamera::FinalDrawCallback* finaldrawcallback = NULL;
  finaldrawcallback=dynamic_cast<const osaOSGCamera::FinalDrawCallback*>(dcb);

  CMN_ASSERT( finaldrawcallback != NULL );
  const cv::Mat& rgbimage = finaldrawcallback->GetRGBImage();

  cv::Size size = rgbimage.size();
  vctDynamicMatrix<unsigned char> x( size.height, size.width*3 );
  memcpy( x.Pointer(), rgbimage.ptr<unsigned char>(), size.height*size.width*3 );

  return x;
}
*/
#endif // SAW_OPENSCENEGRAPH_SUPPORTS_OPENCV
#endif
