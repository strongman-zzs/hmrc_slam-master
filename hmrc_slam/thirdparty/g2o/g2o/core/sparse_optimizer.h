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

#ifndef G2O_GRAPH_OPTIMIZER_CHOL_H_
#define G2O_GRAPH_OPTIMIZER_CHOL_H_

#include "../stuff/macros.h"

#include "optimizable_graph.h"
#include "sparse_block_matrix.h"
#include "batch_stats.h"

#include <map>

namespace g2o {

  // forward declaration
  class ActivePathCostFunction;
  class OptimizationAlgorithm;
  class EstimatePropagatorCost;

  class  SparseOptimizer : public OptimizableGraph {

    public:
    enum {
      AT_COMPUTEACTIVERROR = OptimizableGraph::AT_NUM_ELEMENTS,
      AT_NUM_ELEMENTS, // keep as last element
    };

    friend class ActivePathCostFunction;

    // Attention: _solver & _statistics is own by SparseOptimizer and will be
    // deleted in its destructor.
    SparseOptimizer();
    virtual ~SparseOptimizer();

    // new interface for the optimizer
    // the old functions will be dropped
        virtual bool initializeOptimization(HyperGraph::EdgeSet& eset);

        virtual bool initializeOptimization(HyperGraph::VertexSet& vset, int level=0);

        virtual bool initializeOptimization(int level=0);

        virtual bool updateInitialization(HyperGraph::VertexSet& vset, HyperGraph::EdgeSet& eset);
  
        virtual void computeInitialGuess();

        virtual void computeInitialGuess(EstimatePropagatorCost& propagator);

        virtual void setToOrigin();


        int optimize(int iterations, bool online = false);

        bool computeMarginals(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<std::pair<int, int> >& blockIndices);

        bool computeMarginals(SparseBlockMatrix<MatrixXd>& spinv, const Vertex* vertex) {
      if (vertex->hessianIndex() < 0) {
          return false;
      }
      std::vector<std::pair<int, int> > index;
      index.push_back(std::pair<int, int>(vertex->hessianIndex(), vertex->hessianIndex()));
      return computeMarginals(spinv, index);
    }

        bool computeMarginals(SparseBlockMatrix<MatrixXd>& spinv, const VertexContainer& vertices) {
      std::vector<std::pair<int, int> > indices;
      for (VertexContainer::const_iterator it = vertices.begin(); it != vertices.end(); ++it) {
        indices.push_back(std::pair<int, int>((*it)->hessianIndex(),(*it)->hessianIndex()));
      }
      return computeMarginals(spinv, indices);
    }

    //! finds a gauge in the graph to remove the undefined dof.
    // The gauge should be fixed() and then the optimization can work (if no additional dof are in
    // the system. The default implementation returns a node with maximum dimension.
    virtual Vertex* findGauge();

    bool gaugeFreedom();

        double activeChi2() const;
        double activeRobustChi2() const;

    //! verbose information during optimization
    bool verbose()  const {return _verbose;}
    void setVerbose(bool verbose);

        void setForceStopFlag(bool* flag);
    bool* forceStopFlag() const { return _forceStopFlag;};

    //! if external stop flag is given, return its state. False otherwise
    bool terminate() {return _forceStopFlag ? (*_forceStopFlag) : false; }

    //! the index mapping of the vertices
    const VertexContainer& indexMapping() const {return _ivMap;}
    //! the vertices active in the current optimization
    const VertexContainer& activeVertices() const { return _activeVertices;}
    //! the edges active in the current optimization
    const EdgeContainer& activeEdges() const { return _activeEdges;}

        virtual bool removeVertex(HyperGraph::Vertex* v);

        VertexContainer::const_iterator findActiveVertex(const OptimizableGraph::Vertex* v) const;
        EdgeContainer::const_iterator findActiveEdge(const OptimizableGraph::Edge* e) const;

    //! the solver used by the optimizer
    const OptimizationAlgorithm* algorithm() const { return _algorithm;}
    OptimizationAlgorithm* solver() { return _algorithm;}
    void setAlgorithm(OptimizationAlgorithm* algorithm);

    //! push the estimate of a subset of the variables onto a stack
    void push(SparseOptimizer::VertexContainer& vlist);
    //! push the estimate of a subset of the variables onto a stack
    void push(HyperGraph::VertexSet& vlist);
    //! push all the active vertices onto a stack
    void push();
    //! pop (restore) the estimate a subset of the variables from the stack
    void pop(SparseOptimizer::VertexContainer& vlist);
    //! pop (restore) the estimate a subset of the variables from the stack
    void pop(HyperGraph::VertexSet& vlist);
    //! pop (restore) the estimate of the active vertices from the stack
    void pop();

    //! ignore the latest stored element on the stack, remove it from the stack but do not restore the estimate
    void discardTop(SparseOptimizer::VertexContainer& vlist);
    //! same as above, but for the active vertices
    void discardTop();
    using OptimizableGraph::discardTop;

        virtual void clear();

        void computeActiveErrors();

        G2O_ATTRIBUTE_DEPRECATED(void linearizeSystem())
    {
      // nothing needed, linearization is now done inside the solver
    }

        void update(const double* update);

        const BatchStatisticsContainer& batchStatistics() const { return _batchStatistics;}
        BatchStatisticsContainer& batchStatistics() { return _batchStatistics;}
    
    void setComputeBatchStatistics(bool computeBatchStatistics);
    
    bool computeBatchStatistics() const { return _computeBatchStatistics;}

        //! add an action to be executed before the error vectors are computed
    bool addComputeErrorAction(HyperGraphAction* action);
    //! remove an action that should no longer be execured before computing the error vectors
    bool removeComputeErrorAction(HyperGraphAction* action);

    

    protected:
    bool* _forceStopFlag;
    bool _verbose;

    VertexContainer _ivMap;
    VertexContainer _activeVertices;   ///< sorted according to VertexIDCompare
    EdgeContainer _activeEdges;        ///< sorted according to EdgeIDCompare

    void sortVectorContainers();
 
    OptimizationAlgorithm* _algorithm;

        bool buildIndexMapping(SparseOptimizer::VertexContainer& vlist);
    void clearIndexMapping();

    BatchStatisticsContainer _batchStatistics;   ///< global statistics of the optimizer, e.g., timing, num-non-zeros
    bool _computeBatchStatistics;
  };
} // end namespace

#endif
