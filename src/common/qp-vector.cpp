#include "qp-vector.h"
#include <Eigen/Cholesky>
#include <cmath>

using namespace MPCWalkgen;
using namespace Eigen;




QPVector::QPVector(const int nbRowsMin, const int nbRowsMax)
	:sizeRows_(nbRowsMax-nbRowsMin+1)
	,constantPart_(sizeRows_)
	,matrix_(sizeRows_)
	,nbRowsMin_(nbRowsMin)
	,nbRows_(nbRowsMin)
	,vectorElem_(0)
	,rowOrder_(nbRowsMax)
{
  for(int i=0;i<sizeRows_;++i){
    constantPart_[i].setZero(i+nbRowsMin_);
    matrix_[i].setZero(i+nbRowsMin_);
  }

  for(int i=0;i<nbRowsMax;++i){
    rowOrder_(i)=i;
  }
}

QPVector::~QPVector(){}

void QPVector::addTerm(const VectorXd & vec, const int row)
{
  int nbRows = vec.rows();
  for (int i=0;i<nbRows;++i){
    matrix_[vectorElem_](rowOrder_(row+i))+=vec(i);
  }
}

void QPVector::setConstantPart(const VectorXd & mat){
  int nbRows = mat.rows();
  for (int i=0;i<=nbRows;++i){
    constantPart_[vectorElem_](rowOrder_(i))=mat(i);
  }
}

void QPVector::reset(){
  matrix_[vectorElem_].fill(0);
}

void QPVector::resize(const int nbRows){
  nbRows_ = nbRows;
  vectorElem_ = nbRows_-nbRowsMin_;
}

void QPVector::rowOrder(const Eigen::VectorXi & order){
  rowOrder_=order;
}
