#include "qp-matrix.h"
#include <Eigen/Cholesky>
#include <cmath>

using namespace MPCWalkgen;
using namespace Eigen;




QPMatrix::QPMatrix(const int nbRowsMin, const int nbColsMin,
					 const int nbRowsMax, const int nbColsMax)
	:sizeRows_(nbRowsMax-nbRowsMin+1)
	,sizeCols_(nbColsMax-nbColsMin+1)
	,constantPart_(sizeRows_*sizeCols_)
	,matrix_(sizeRows_*sizeCols_)
	,choleskyMatrix_(sizeRows_*sizeCols_)
	,nbRowsMin_(nbRowsMin)
	,nbColsMin_(nbColsMin)
	,nbRows_(nbRowsMin)
	,nbCols_(nbColsMin)
	,vectorElem_(0)
	,choleskyMatrixOutdated_(true)
	,rowOrder_(nbRowsMax)
	,colOrder_(nbColsMax)
{
  for(int i=0;i<sizeRows_;++i){
    for (int j=0;j<sizeCols_;++j){
      constantPart_[i+j*sizeRows_].setZero(i+nbRowsMin_, j+nbColsMin_);
      matrix_[i+j*sizeRows_].setZero(i+nbRowsMin_, j+nbColsMin_);
      choleskyMatrix_[i+j*sizeRows_].setZero(i+nbRowsMin_, j+nbColsMin_);
    }
  }

  for(int i=0;i<nbRowsMax;++i){
    rowOrder_(i)=i;
  }
  for(int i=0;i<nbColsMax;++i){
    colOrder_(i)=i;
  }
}

QPMatrix::~QPMatrix(){}

void QPMatrix::addTerm(const MatrixXd & mat,
		const int row , const int col )
{
	int nbRows = mat.rows();
	int nbCols = mat.cols();
	for (int i=0;i<nbRows;++i){
		for (int j=0;j<nbCols;++j){
			matrix_[vectorElem_](rowOrder_(row+i),colOrder_(col+j))+=mat(i,j);
		}
	}
	choleskyMatrixOutdated_=true;

}

void QPMatrix::setConstantPart(const MatrixXd & mat){
	int nbRows = mat.rows();
	int nbCols = mat.cols();
	for (int i=0;i<=nbRows;++i){
		for (int j=0;j<nbCols;++j){
			constantPart_[vectorElem_](rowOrder_(i),colOrder_(j))=mat(i,j);
		}
	}
}

void QPMatrix::reset(){
  matrix_[vectorElem_].fill(0);
  choleskyMatrixOutdated_=true;
}

void QPMatrix::resize(const int nbRows, const int nbCols){
  nbRows_ = nbRows;
  nbCols_ = nbCols;
  vectorElem_ = nbRows_-nbRowsMin_ + (nbCols_-nbColsMin_) *sizeRows_;
}

MatrixXd & QPMatrix::cholesky(){
	computeCholesky();
	return choleskyMatrix_[vectorElem_];
}

MatrixXd & QPMatrix::cholesky(MatrixXd & partialCholesky){
	computeCholesky(partialCholesky);
	return choleskyMatrix_[vectorElem_];
}

void QPMatrix::colOrder(const Eigen::VectorXi & order){
	colOrder_=order;
}

void QPMatrix::rowOrder(const Eigen::VectorXi & order){
	rowOrder_=order;
}

void QPMatrix::computeCholesky(const MatrixXd & partialCholesky){
	if (choleskyMatrixOutdated_){
		Eigen::MatrixXd & chol = choleskyMatrix_[vectorElem_];
		int imin=partialCholesky.rows();
		if (imin>0){
			chol.fill(0);
			chol.block(0,0,imin,imin)=partialCholesky;
			double tmp;
			for(int j=0;j<nbCols_;++j){
				if (j>=imin){
					for(int i=0;i<j;++i){
						chol(j,i)=0;
					}
					tmp = matrix_[vectorElem_](j,j);
					for(int k=0;k<j;++k){
						tmp -= chol(k,j)*chol(k,j);
					}
					if (fabs(tmp)>EPSILON){
						chol(j,j) = sqrt(tmp);
					}else{
						chol(j,j) = 0;
					}

				}
				for(int i=std::max(j+1,imin);i<nbRows_;++i){
					tmp=matrix_[vectorElem_](j,i);
					for(int k=0;k<j;++k){
						tmp -= chol(k,j)*chol(k,i);
					}
					if (fabs(tmp)>EPSILON && fabs(chol(j,j))>EPSILON){
						chol(j,i) = tmp / chol(j,j);
					}else{
						chol(j,i) = 0;
					}
				}
			}
		}else{
			chol=matrix_[vectorElem_].llt().matrixL().transpose();
		}
		choleskyMatrixOutdated_=false;
	}
}

void QPMatrix::computeCholesky(){
	if (choleskyMatrixOutdated_){
		Eigen::MatrixXd & chol = choleskyMatrix_[vectorElem_];
		chol=matrix_[vectorElem_].llt().matrixL().transpose();
		choleskyMatrixOutdated_=false;
	}
}
