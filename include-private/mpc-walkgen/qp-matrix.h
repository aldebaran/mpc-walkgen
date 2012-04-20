#ifndef QP_MATRIX
#define QP_MATRIX

////////////////////////////////////////////////////////////////////////////////
///
///\file	qp-matrix.h
///\brief	A class to store QP matrices
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author	Keith François
///\version	1.0
///\date	05/01/12
///
////////////////////////////////////////////////////////////////////////////////

#include <vector>
#include <Eigen/Dense>

namespace MPCWalkgen{

	class QPMatrix{
		public:
			QPMatrix(const int nbRowsMin=1, const int nbColsMin=1,
					 const int nbRowsMax=1, const int nbColsMax=1);
			~QPMatrix();

			void addTerm(const Eigen::MatrixXd & mat,
					const int row = 0, const int col = 0);

			void setConstantPart(const Eigen::MatrixXd & mat);

			void reset();

			void resize(const int nbRows, const int nbCols=1);

			Eigen::MatrixXd & cholesky();
			Eigen::MatrixXd & cholesky(Eigen::MatrixXd & partialCholesky);

			void colOrder(const Eigen::VectorXi & order);
			void rowOrder(const Eigen::VectorXi & order);


		// accessors
			inline Eigen::MatrixXd & operator()(void) {
				choleskyMatrixOutdated_=true;
				return matrix_[vectorElem_];
			}
			inline double & operator()(int row, int col=0){return matrix_[vectorElem_](row,col);}

			inline int nbRows() const	 {return nbRows_;}
			inline int nbCols() const    {return nbCols_;}

			inline int nbRowsMax() const {return nbRowsMin_+sizeRows_-1;}
			inline int nbColsMax() const {return nbColsMin_+sizeCols_-1;}

		private:
			void computeCholesky(Eigen::MatrixXd & partialCholesky);

		private:

			int sizeRows_;
			int sizeCols_;

			std::vector<Eigen::MatrixXd> constantPart_;
			std::vector<Eigen::MatrixXd> matrix_;
			std::vector<Eigen::MatrixXd> choleskyMatrix_;

			int nbRowsMin_;
			int nbColsMin_;

			int nbRows_;
			int nbCols_;
			int vectorElem_;

			bool choleskyMatrixOutdated_;

			Eigen::VectorXi rowOrder_;
			Eigen::VectorXi colOrder_;
	};

}

/*! \fn MPCWalkgen::QPMatrix::QPMatrix(const int nbRows, const int nbCols,
					 const int nbRowsMax=1, const int nbColsMax=1)
* \brief Constructor. nbRowsMax/nbColsMax must be greater or equal than nbRows/nbCols
* \param nbRows Declared row number of dense matrix
* \param nbCols Declared col number of dense matrix
* \param nbRowsMax number of rows of storage matrix
* \param nbColsMax number of cols of storage matrix
*/

/*! \fn void MPCWalkgen::QPMatrix::addTerm(const Eigen::MatrixXd & mat,
					const int row = 0, const int col = 0)
* \brief Add the content of matrix mat into the QPMatrix, starting at position (row/col)
*/

/*! \fn void MPCWalkgen::QPMatrix::setConstantPart(const Eigen::MatrixXd & mat)
* \brief Define the constant part of the QPMatrix
*/

/*! \fn void MPCWalkgen::QPMatrix::reset(const bool withConstantPart = false)
* \brief Erase the QPMatrix
* \param withConstantPart If true, Set QPMatrix to it constant part
*/

/*! \fn void MPCWalkgen::QPMatrix::resize(const int nbRows, const int nbCols=1,
					const bool preserve=false, const bool withConstantPart = false)
* \brief define the new declared dimension of dense matrix
* \param preserve if true, the content will be conserved. Else, reset method will be called
* \param withConstantPart for reset method, only if preserve=true
*/

/*! \fn Eigen::MatrixXd & MPCWalkgen::QPMatrix::operator()(void)
* \brief Return the storage matrix
*/

/*! \fn double & MPCWalkgen::QPMatrix::operator()(int row, int col=0)
* \brief return the value of the matrix at (row,col)
*/

/*! \fn Eigen::MatrixXd & MPCWalkgen::QPMatrix::dense()
* \brief return the dense matrix
*/

/*! \fn int MPCWalkgen::QPMatrix::nbRows() const
* \brief return the dense matrix number of rows
*/

/*! \fn int MPCWalkgen::QPMatrix::nbCols() const
* \brief return the dense matrix number of cols
*/

/*! \fn int MPCWalkgen::QPMatrix::nbRowsMax() const
* \brief return the storage matrix number of rows
*/

/*! \fn int MPCWalkgen::QPMatrix::nbColsMax() const
* \brief return the storage matrix number of cols
*/

#endif //QP_MATRIX
