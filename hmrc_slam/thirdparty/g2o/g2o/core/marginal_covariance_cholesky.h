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

#ifndef G2O_MARGINAL_COVARIANCE_CHOLESKY_H
#define G2O_MARGINAL_COVARIANCE_CHOLESKY_H

#include "optimizable_graph.h"
#include "sparse_block_matrix.h"

#include <cassert>
#include <vector>

#ifdef _MSC_VER
#include <unordered_map>
#else
#include <tr1/unordered_map>
#endif


namespace g2o {

    class  MarginalCovarianceCholesky {
    protected:
            typedef std::tr1::unordered_map<int, double>     LookupMap;
    
    public:
      MarginalCovarianceCholesky();
      ~MarginalCovarianceCholesky();

            void computeCovariance(double** covBlocks, const std::vector<int>& blockIndices);


            void computeCovariance(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<int>& rowBlockIndices, const std::vector< std::pair<int, int> >& blockIndices);


            void setCholeskyFactor(int n, int* Lp, int* Li, double* Lx, int* permInv);

    protected:
      // information about the cholesky factor (lower triangle)
      int _n;           ///< L is an n X n matrix
      int* _Ap;         ///< column pointer of the CCS storage
      int* _Ai;         ///< row indices of the CCS storage
      double* _Ax;      ///< values of the cholesky factor
      int* _perm;       ///< permutation of the cholesky factor. Variable re-ordering for better fill-in

      LookupMap _map;             ///< hash look up table for the already computed entries
      std::vector<double> _diag;  ///< cache 1 / H_ii to avoid recalculations

      //! compute the index used for hashing
      int computeIndex(int r, int c) const { /*assert(r <= c);*/ return r*_n + c;}
            double computeEntry(int r, int c);
  };

}

#endif
