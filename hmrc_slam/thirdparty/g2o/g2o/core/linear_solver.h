// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_LINEAR_SOLVER_H
#define G2O_LINEAR_SOLVER_H
#include "sparse_block_matrix.h"
#include "sparse_block_matrix_ccs.h"

namespace g2o {

template <typename MatrixType>
class LinearSolver
{
  public:
    LinearSolver() {};
    virtual ~LinearSolver() {}

        virtual bool init() = 0;

        virtual bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b) = 0;

        virtual bool solveBlocks(double**&blocks, const SparseBlockMatrix<MatrixType>& A) { (void)blocks; (void) A; return false; }


        virtual bool solvePattern(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<std::pair<int, int> >& blockIndices, const SparseBlockMatrix<MatrixType>& A){
      (void) spinv;
      (void) blockIndices;
      (void) A;
      return false;
    }

    //! write a debug dump of the system matrix if it is not PSD in solve
    virtual bool writeDebug() const { return false;}
    virtual void setWriteDebug(bool) {}
};

template <typename MatrixType>
class LinearSolverCCS : public LinearSolver<MatrixType>
{
  public:
    LinearSolverCCS() : LinearSolver<MatrixType>(), _ccsMatrix(0) {}
    ~LinearSolverCCS()
    {
      delete _ccsMatrix;
    }

  protected:
    SparseBlockMatrixCCS<MatrixType>* _ccsMatrix;

    void initMatrixStructure(const SparseBlockMatrix<MatrixType>& A)
    {
      delete _ccsMatrix;
      _ccsMatrix = new SparseBlockMatrixCCS<MatrixType>(A.rowBlockIndices(), A.colBlockIndices());
      A.fillSparseBlockMatrixCCS(*_ccsMatrix);
    }
};

} // end namespace

#endif
