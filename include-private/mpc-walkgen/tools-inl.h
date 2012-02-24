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

	// Methods relative to the computation of polynomials
	double p(const Eigen::Matrix<double,6,1> & factor, double x)  {
		return factor(0)*pow5(x) + factor(1)*pow4(x) + factor(2)*pow3(x) + factor(3)*pow2(x) + factor(4)*x + factor(5);
	}

	double dp(const Eigen::Matrix<double,6,1> & factor, double x) {
		return 5*factor(0)*pow4(x) + 4*factor(1)*pow3(x) + 3*factor(2)*pow2(x) + 2*factor(3)*x + factor(4);
	}

	double ddp(const Eigen::Matrix<double,6,1> & factor, double x) {
		return 20*factor(0)*pow3(x) + 12*factor(1)*pow2(x) + 6*factor(2)*x + 2*factor(3);
	}
}
