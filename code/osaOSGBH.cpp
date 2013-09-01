#include <sawOpenSceneGraph/osaOSGBH.h>

osaOSGBH::osaOSGBH(	const std::string& palmmodel,
			const std::string& metacarpalmodel,
			const std::string& proximalmodel,
			const std::string& intermediatemodel,
			osaOSGWorld* world,
			const vctFrame4x4<double>& Rtwb,
			const std::string& f1f2filename,
			const std::string& f3filename ):
  osaOSGManipulator( std::vector< std::string >(), 
		       world, 
		       Rtwb, 
		       std::string(),
		       palmmodel ),
  f1( NULL ),
  f2( NULL ),
  f3( NULL ){

  std::vector<std::string> modelsf1f2;
  modelsf1f2.push_back( metacarpalmodel );
  modelsf1f2.push_back( proximalmodel );
  modelsf1f2.push_back( intermediatemodel );

  vctFrame4x4<double> Rtbf1( vctMatrixRotation3<double>(),
			     vctFixedSizeVector<double,3>( 0.0,  0.026,-0.01) );
  vctFrame4x4<double> Rtbf2( vctMatrixRotation3<double>(),
			     vctFixedSizeVector<double,3>( 0.0, -0.026,-0.01) );

  f1 = new osaOSGManipulator( modelsf1f2, 
				(osaOSGWorld*)this, 
				Rtwb*Rtbf1, 
				f1f2filename );
  f2 = new osaOSGManipulator( modelsf1f2, 
				(osaOSGWorld*)this, 
				Rtwb*Rtbf2, 
				f1f2filename );

  std::vector<std::string> modelsf3;
  modelsf3.push_back( proximalmodel );
  modelsf3.push_back( intermediatemodel );
  vctFrame4x4<double> Rtbf3( vctMatrixRotation3<double>( 1.0, 0.0, 0.0,
							 0.0, 0.0,-1.0,
							 0.0, 1.0, 0.0 ),
			     vctFixedSizeVector<double,3>( 0.05, 0.0,0.0 ) );

  f3 = new osaOSGManipulator( modelsf3, 
				(osaOSGWorld*)this,
				Rtwb*Rtbf3, 
				f3filename );
}


osaOSGBH::osaOSGBH(	const std::string& palmmodel,
			const std::string& metacarpalmodel,
			const std::string& proximalmodel,
			const std::string& intermediatemodel,
			osaOSGWorld* world,
			const vctFrm3& Rtwb,
			const std::string& f1f2filename,
			const std::string& f3filename ):
  osaOSGManipulator( std::vector< std::string >(), 
		       world, 
		       Rtwb, 
		       std::string(),
		       palmmodel ),
  f1( NULL ),
  f2( NULL ),
  f3( NULL ){

  std::vector<std::string> modelsf1f2;
  modelsf1f2.push_back( metacarpalmodel );
  modelsf1f2.push_back( proximalmodel );
  modelsf1f2.push_back( intermediatemodel );

  vctFrm3 Rtbf1( vctMatrixRotation3<double>(),
		 vctFixedSizeVector<double,3>( 0.0,  0.026,-0.01) );
  vctFrm3 Rtbf2( vctMatrixRotation3<double>(),
		 vctFixedSizeVector<double,3>( 0.0, -0.026,-0.01) );

  f1 = new osaOSGManipulator( modelsf1f2, 
				(osaOSGWorld*)this, 
				Rtwb*Rtbf1, 
				f1f2filename );
  f2 = new osaOSGManipulator( modelsf1f2, 
				(osaOSGWorld*)this, 
				Rtwb*Rtbf2, 
				f1f2filename );

  std::vector<std::string> modelsf3;
  modelsf3.push_back( proximalmodel );
  modelsf3.push_back( intermediatemodel );
  vctFrm3 Rtbf3( vctMatrixRotation3<double>( 1.0, 0.0, 0.0,
					     0.0, 0.0,-1.0,
					     0.0, 1.0, 0.0 ),
		 vctFixedSizeVector<double,3>( 0.05, 0.0,0.0 ) );
  
  f3 = new osaOSGManipulator( modelsf3, 
				(osaOSGWorld*)this,
				Rtwb*Rtbf3, 
				f3filename );
}

osaOSGBH::~osaOSGBH(){}


osaOSGBH::Errno osaOSGBH::GetPositions( vctDynamicVector<double>& qs )const{
  qs = q;
  return osaOSGBH::ESUCCESS;
}

osaOSGBH::Errno osaOSGBH::SetPositions(const vctDynamicVector<double>& qs){

  if( qs.size() == 4 ){

    q = qs;

    if( f1.get() != NULL ){ 
      vctDynamicVector<double> q( 3, qs[3], qs[0], qs[0]*45.0/140.0 );
      if( f1->SetPositions( q ) != osaOSGManipulator::ESUCCESS ){
	CMN_LOG_RUN_ERROR << "Failed to set position of finger 1" << std::endl;
	return osaOSGBH::EFAILURE;
      }
    }
    else{
      CMN_LOG_RUN_ERROR << "Finger 1 is missing" << std::endl;
      return osaOSGBH::EFAILURE;
    }

    if( f2.get() != NULL ){
      vctDynamicVector<double> q( 3, -qs[3], qs[1], qs[1]*45.0/140.0 );
      if( f2->SetPositions( q ) != osaOSGManipulator::ESUCCESS ){
	CMN_LOG_RUN_ERROR << "Failed to set position of finger 2" << std::endl;
	return osaOSGBH::EFAILURE;
      }
    }
    else{
      CMN_LOG_RUN_ERROR << "Finger 2 is missing" << std::endl;
      return osaOSGBH::EFAILURE;
    }

    if( f3.get() != NULL ){
      vctDynamicVector<double> q( 2,         qs[2], qs[2]*45.0/140.0 );
      if( f3->SetPositions( q ) != osaOSGManipulator::ESUCCESS ){
	CMN_LOG_RUN_ERROR << "Failed to set position of finger 3" << std::endl;
	return osaOSGBH::EFAILURE;
      }
    }
    else{
      CMN_LOG_RUN_ERROR << "Finger 3 is missing" << std::endl;
      return osaOSGBH::EFAILURE;
    }

    return osaOSGBH::ESUCCESS;

  }
  
  else{
    CMN_LOG_RUN_ERROR << "Expected 4 joints values. Got " << qs.size()
		      << std::endl;
    return osaOSGBH::EFAILURE;  
  }

}
