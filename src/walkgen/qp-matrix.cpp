#include <mpc-walkgen/qp-matrix.h>
#include <mpc-walkgen/types.h>
#include <Eigen/Cholesky>
#include <cmath>

using namespace MPCWalkgen;
using namespace Eigen;




QPMatrix::QPMatrix(const int nbRows, const int nbCols,
					 const int nbRowsMax, const int nbColsMax)
	:constantPart_(nbRowsMax,nbColsMax)
	,matrix_(nbRowsMax,nbColsMax)
	,denseMatrix_(nbRowsMax,nbColsMax)
	,choleskyMatrix_(nbRowsMax,nbColsMax)
	,nbRows_(nbRows)
	,nbCols_(nbCols)
	,nbRowsMax_(nbRowsMax)
	,nbColsMax_(nbColsMax)
	,denseMatrixOutdated_(true)
	,choleskyMatrixOutdated_(true)
	,rowOrder_(nbRowsMax)
	,colOrder_(nbColsMax)
{
	bool undefinedSizeMax=false;
	if (nbRowsMax<nbRows){
		nbRowsMax_=nbRows_;
		undefinedSizeMax=true;
	}
	if (nbColsMax<nbCols){
		nbColsMax_=nbCols_;
		undefinedSizeMax=true;
	}
	if (undefinedSizeMax){
		constantPart_.resize(nbRowsMax,nbColsMax);
		matrix_.resize(nbRowsMax,nbColsMax);
	}

	constantPart_.fill(0);
	matrix_.fill(0);
	denseMatrix_.fill(0);
	choleskyMatrix_.fill(0);
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
			matrix_(rowOrder_(row+i),colOrder_(col+j))+=mat(i,j);
		}
	}
	denseMatrixOutdated_=true;
	choleskyMatrixOutdated_=true;

}

void QPMatrix::setConstantPart(const MatrixXd & mat){
	int nbRows = mat.rows();
	int nbCols = mat.cols();
	for (int i=0;i<=nbRows;++i){
		for (int j=0;j<nbCols;++j){
			constantPart_(rowOrder_(i),colOrder_(j))=mat(i,j);
		}
	}
}

void QPMatrix::reset(const bool withConstantPart){
	if (withConstantPart){
		matrix_.block(0,0,nbRows_,nbCols_) = constantPart_.block(0,0,nbRows_,nbCols_);
	}else{
		matrix_.block(0,0,nbRows_,nbCols_).fill(0);
	}
	denseMatrixOutdated_=true;
	choleskyMatrixOutdated_=true;
}

void QPMatrix::resize(const int nbRows, const int nbCols,
		const bool preserve, const bool withConstantPart){

	bool maxSizeChanged=false;

	if (nbRows>nbRowsMax_){
		nbRowsMax_=nbRows;
		maxSizeChanged=true;
	}

	if (nbCols>nbColsMax_){
		nbColsMax_=nbCols;
		maxSizeChanged=true;
	}

	if (maxSizeChanged){
		if (preserve){
			MatrixXd tmp1 = constantPart_;
			MatrixXd tmp2 = matrix_;
			constantPart_.setZero(nbRowsMax_, nbColsMax_);
			matrix_.setZero(nbRowsMax_, nbColsMax_);
			int previousRowMax = tmp1.rows();
			int previouscolMax = tmp1.cols();

			for(int i=0;i<previousRowMax;++i){
				for(int j=0;j<previouscolMax;++j){
					constantPart_(i,j)=tmp1(i,j);
					matrix_(i,j)=tmp1(i,j);
				}
			}
		}else{
			constantPart_.resize(nbRowsMax_, nbColsMax_);
			matrix_.resize(nbRowsMax_, nbColsMax_);
			reset(withConstantPart);
		}
	}

	nbRows_=nbRows;
	nbCols_=nbCols;
}

MatrixXd & QPMatrix::dense(){
	if (denseMatrix_.cols()!=nbCols_ || denseMatrix_.rows()!=nbRows_){
		denseMatrix_.resize(nbRows_,nbCols_);
		denseMatrixOutdated_=true;
	}
	if (denseMatrixOutdated_){
		denseMatrix_.block(0,0,nbRows_, nbCols_) = matrix_.block(0,0,nbRows_, nbCols_);
		denseMatrixOutdated_=false;
	}
	return denseMatrix_;
}

MatrixXd & QPMatrix::cholesky(){
	MatrixXd partialCholesky(0,0);
	computeCholesky(partialCholesky);
	return choleskyMatrix_;
}

MatrixXd & QPMatrix::cholesky(MatrixXd & partialCholesky){
	computeCholesky(partialCholesky);
	return choleskyMatrix_;
}

void QPMatrix::colOrder(const Eigen::VectorXi & order){
	colOrder_=order;
}

void QPMatrix::rowOrder(const Eigen::VectorXi & order){
	rowOrder_=order;
}

void QPMatrix::computeCholesky(MatrixXd & partialCholesky){
	if (choleskyMatrix_.cols()!=nbCols_ || choleskyMatrix_.rows()!=nbRows_){
		choleskyMatrix_.resize(nbRows_,nbCols_);
		choleskyMatrixOutdated_=true;
	}
	if (choleskyMatrixOutdated_){

		int imin=partialCholesky.rows();
		if (imin>0){
			choleskyMatrix_.fill(0);
			choleskyMatrix_.block(0,0,imin,imin)=partialCholesky;
			double tmp;
			for(int j=0;j<nbCols_;++j){
				if (j>=imin){
					for(int i=0;i<j;++i){
						choleskyMatrix_(j,i)=0;
					}
					tmp = matrix_(j,j);
					for(int k=0;k<j;++k){
						tmp -= choleskyMatrix_(k,j)*choleskyMatrix_(k,j);
					}
					if (fabs(tmp)>EPSILON){
						choleskyMatrix_(j,j) = sqrt(tmp);
					}else{
						choleskyMatrix_(j,j) = 0;
					}

				}
				for(int i=std::max(j+1,imin);i<nbRows_;++i){
					tmp=matrix_(j,i);
					for(int k=0;k<j;++k){
						tmp -= choleskyMatrix_(k,j)*choleskyMatrix_(k,i);
					}
					if (fabs(tmp)>EPSILON && fabs(choleskyMatrix_(j,j))>EPSILON){
						choleskyMatrix_(j,i) = tmp / choleskyMatrix_(j,j);
					}else{
						choleskyMatrix_(j,i) = 0;
					}
				}
			}
		}else{
			choleskyMatrix_=matrix_.block(0,0,nbRows_, nbCols_).llt().matrixL().transpose();
		}
		choleskyMatrixOutdated_=false;
	}
}
