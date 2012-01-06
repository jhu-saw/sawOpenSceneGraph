#include <cisstOSAbstraction/osaSleep.h>
#include <sawOpenSceneGraph/osaOSGWorld.h>
#include <sawOpenSceneGraph/osaOSGMono.h>
#include <sawOpenSceneGraph/osaOSGHUD.h>
#include <sawOpenSceneGraph/osaOSGBody.h>
#include <sawOpenSceneGraph/osaOSGImage.h>
#include <osgText/Text>
#undef SAW_OPENSCENEGRAPH_SUPPORTS_OPENCV

int main( int, char** argv ){

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  osg::ref_ptr< osaOSGWorld > world = new osaOSGWorld;
  
  // Create a camera
  int x = 0, y = 0;
  int width = 320, height = 240;
  double Znear = 0.1, Zfar = 10.0;
  osg::ref_ptr< osaOSGMono > camera;
  camera = new osaOSGMono( world,
			   x, y, width, height,
			   55.0, ((double)width)/((double)height),
			   Znear, Zfar,
			   true );
  camera->Initialize();

  // Create the objects
  std::string path( CISST_SOURCE_ROOT"/etc/cisstRobot/objects/" );

  vctFrame4x4<double> Rt(  vctMatrixRotation3<double>(),
			   vctFixedSizeVector<double,3>( 0.0, 0.0, 0.5 ) );

  osg::ref_ptr< osaOSGBody > hubble;
  hubble = new osaOSGBody( path+"hst.3ds", world, Rt );
  
  osg::ref_ptr< osaOSGBody > background;
  //background = new osaOSGBody( path+"background.3ds", world, 
  //			       vctFrame4x4<double>() );

  osg::ref_ptr< osaOSGHUD > hud;
  hud = new osaOSGHUD(world, width, height, camera );

  osg::ref_ptr< osaOSGImage > image;
  image = new osaOSGImage( 320, 240, NULL, vctFrame4x4<double>() );
  hud->addChild( image );


  osg::Geode* geode = new osg::Geode();

  std::string timesFont("fonts/arial.ttf");

  //osg::StateSet* stateset = geode->getOrCreateStateSet();
  //stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

  osg::Vec3 position(0.0, .0f,0.0f);
  osgText::Text* text = new  osgText::Text;
  geode->addDrawable( text );
  
  text->setFont(timesFont);
  text->setPosition(position);
  text->setText("Head Up Displays are simple :-)");
  /*
  osg::Geometry* geom = new osg::Geometry;
  geom = osg::createTexturedQuadGeometry( osg::Vec3(-, 1/-2.0,0.0),
					  osg::Vec3( 1,  0.0, 0.0 ),
					  osg::Vec3( 0.0, 1, 0.0 ) );
  /*
  osg::Vec3Array* vertices = new osg::Vec3Array;
  float depth = 0.0;//bb.zMin()-0.1;
  vertices->push_back(osg::Vec3(bb.xMin(),bb.yMax(),depth));
  vertices->push_back(osg::Vec3(bb.xMin(),bb.yMin(),depth));
  vertices->push_back(osg::Vec3(bb.xMax(),bb.yMin(),depth));
  vertices->push_back(osg::Vec3(bb.xMax(),bb.yMax(),depth));
  geom->setVertexArray(vertices);

  osg::Vec3Array* normals = new osg::Vec3Array;
  normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
  geom->setNormalArray(normals);
  geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

  osg::Vec4Array* colors = new osg::Vec4Array;
  colors->push_back(osg::Vec4(1.0f,1.0,0.8f,0.2f));
  geom->setColorArray(colors);
  geom->setColorBinding(osg::Geometry::BIND_OVERALL);


  geode->addDrawable(geom);
  */

  /*
  osg::BoundingBox bb;
  for(unsigned int i=0;i<geode->getNumDrawables();++i)
    {bb.expandBy(geode->getDrawable(i)->getBound());}

  /*
  */
  hud->addChild( geode );
  //hud->addChild( image->geode );
  


  std::cout << "ESC to quit" << std::endl;

  // animate and render
  int i=0;
  double theta=1.0;

  while( !camera->done() ){

    // rotate hubble
    vctFixedSizeVector<double,3> u( 0.0, 0.0, 1.0 );
    vctAxisAngleRotation3<double> Rwh( u, theta );
    vctFrame4x4<double> Rtwh(  Rwh,
			       vctFixedSizeVector<double,3>( 0.0, 0.0, 0.5 ) );
    //hubble->SetTransform( Rtwh );

    // zoom out the camera
    vctFrame4x4<double> Rtwc( vctMatrixRotation3<double>(),
			      vctFixedSizeVector<double,3>(0.0, 0.0, theta ));
    camera->SetTransform( Rtwc );


    char buffer[128];
    sprintf( buffer, "walkcircle/frame%04d.tif", i++ );

    if( i < 124 )
      image->SetImage( std::string( buffer ) );
    if( i == 124 ) i = 0;
    camera->frame();
    
    theta += 0.001;
    
    osaSleep( 1.0/30.0 );

  }

  return 0;

}
