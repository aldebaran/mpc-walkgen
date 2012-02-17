namespace MPCWalkgen{

	double pow2(double v){
		return v*v;
	}

	double pow3(double v){
		return pow2(v)*v;
	}

	double pow4(double v){
		return pow2(pow2(v));
	}

	double pow5(double v){
		return pow4(v)*v;
	}

}
