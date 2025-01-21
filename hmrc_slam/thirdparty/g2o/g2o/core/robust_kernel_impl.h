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

#ifndef G2O_ROBUST_KERNEL_IMPL_H
#define G2O_ROBUST_KERNEL_IMPL_H

#include "robust_kernel.h"

namespace g2o {

    class  RobustKernelScaleDelta : public RobustKernel
  {
    public:
            explicit RobustKernelScaleDelta(const RobustKernelPtr& kernel, double delta = 1.);
      explicit RobustKernelScaleDelta(double delta = 1.);

      //! return the underlying kernel
      const RobustKernelPtr kernel() const { return _kernel;}
      //! use another kernel for the underlying operation
      void setKernel(const RobustKernelPtr& ptr);

      void robustify(double error, Eigen::Vector3d& rho) const;

    protected:
      RobustKernelPtr _kernel;
  };

    class  RobustKernelHuber : public RobustKernel
  {
    public:
      virtual void setDelta(double delta);
      virtual void setDeltaSqr(const double &delta, const double &deltaSqr);
      virtual void robustify(double e2, Eigen::Vector3d& rho) const;

    private:
      float dsqr;
  };

     class  RobustKernelTukey : public RobustKernel
  {
    public:

      virtual void setDeltaSqr(const double &deltaSqr, const double &inv);
      virtual void robustify(double e2, Eigen::Vector3d& rho) const;
    private:
      float _deltaSqr;
      float _invDeltaSqr;
  };


    class  RobustKernelPseudoHuber : public RobustKernel
  {
    public:
      virtual void robustify(double e2, Eigen::Vector3d& rho) const;
  };

    class  RobustKernelCauchy : public RobustKernel
  {
    public:
      virtual void robustify(double e2, Eigen::Vector3d& rho) const;
  };

    class  RobustKernelSaturated : public RobustKernel
  {
    public:
      virtual void robustify(double e2, Eigen::Vector3d& rho) const;
  };

    class  RobustKernelDCS : public RobustKernel
  {
    public:
      virtual void robustify(double e2, Eigen::Vector3d& rho) const;
  };

} // end namespace g2o

#endif
