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

#ifndef G2O_CACHE_HH_
#define G2O_CACHE_HH_

#include <map>

#include "optimizable_graph.h"

namespace g2o {

  class CacheContainer;
  
  class  Cache: public HyperGraph::HyperGraphElement
  {
    public:
      friend class CacheContainer;
      class  CacheKey
      {
        public:
          friend class CacheContainer;
          CacheKey();
          CacheKey(const std::string& type_, const ParameterVector& parameters_);

          bool operator<(const CacheKey& c) const;

          const std::string& type() const { return _type;}
          const ParameterVector& parameters() const { return _parameters;}

        protected:
          std::string _type;
          ParameterVector _parameters;
      };

      Cache(CacheContainer* container_ = 0, const ParameterVector& parameters_ = ParameterVector());

      CacheKey key() const;

      OptimizableGraph::Vertex* vertex();
      OptimizableGraph* graph();
      CacheContainer* container();
      ParameterVector& parameters();

      void update();

      virtual HyperGraph::HyperGraphElementType elementType() const { return HyperGraph::HGET_CACHE;}

    protected:
      //! redefine this to do the update
      virtual void updateImpl() = 0;

            Cache* installDependency(const std::string& type_, const std::vector<int>& parameterIndices);

            virtual bool resolveDependancies();

      bool _updateNeeded;
      ParameterVector _parameters;
      std::vector<Cache*> _parentCaches;
      CacheContainer* _container;
  };

  class  CacheContainer: public std::map<Cache::CacheKey, Cache*>
  {
    public:
      CacheContainer(OptimizableGraph::Vertex* vertex_);
      virtual ~CacheContainer();
      OptimizableGraph::Vertex* vertex();
      OptimizableGraph* graph();
      Cache* findCache(const Cache::CacheKey& key);
      Cache* createCache(const Cache::CacheKey& key);
      void setUpdateNeeded(bool needUpdate=true);
      void update();
    protected:
      OptimizableGraph::Vertex* _vertex;
      bool _updateNeeded;
  };


  template <typename CacheType>
  void OptimizableGraph::Edge::resolveCache(CacheType*& cache, 
      OptimizableGraph::Vertex* v, 
      const std::string& type_, 
      const ParameterVector& parameters_)
  {
    cache = 0;
    CacheContainer* container= v->cacheContainer();
    Cache::CacheKey key(type_, parameters_);
    Cache* c = container->findCache(key);
    if (!c) {
      c = container->createCache(key);
    }
    if (c) {
      cache = dynamic_cast<CacheType*>(c); 
    }
  }

} // end namespace

#endif
