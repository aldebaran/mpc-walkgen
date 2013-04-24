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
  double p(const Eigen::Vector4d & factor, double x)  {
    return factor(0)*pow3(x) + factor(1)*pow2(x) + factor(2)*x + factor(3);
  }

  double dp(const Eigen::Vector4d & factor, double x) {
    return 3*factor(0)*pow2(x) + 2*factor(1)*x + factor(2);
  }

  double ddp(const Eigen::Vector4d & factor, double x) {
    return 6*factor(0)*x + 2*factor(1);
  }

}
