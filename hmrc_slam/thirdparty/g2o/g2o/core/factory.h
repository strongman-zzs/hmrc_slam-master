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

#ifndef G2O_FACTORY_H
#define G2O_FACTORY_H

#include "../../config.h"
#include "../stuff/misc.h"
#include "hyper_graph.h"
#include "creators.h"

#include <string>
#include <map>
#include <iostream>

// define to get some verbose output
//#define G2O_DEBUG_FACTORY

namespace g2o {

  class AbstractHyperGraphElementCreator;
  
    class  Factory
  {
    public:

      //! return the instance
      static Factory* instance();

      //! free the instance
      static void destroy();

            void registerType(const std::string& tag, AbstractHyperGraphElementCreator* c);

            void unregisterType(const std::string& tag);

            HyperGraph::HyperGraphElement* construct(const std::string& tag) const;

            HyperGraph::HyperGraphElement* construct(const std::string& tag, const HyperGraph::GraphElemBitset& elemsToConstruct) const;

            bool knowsTag(const std::string& tag, int* elementType = 0) const;

      //! return the TAG given a vertex
      const std::string& tag(const HyperGraph::HyperGraphElement* v) const;

            void fillKnownTypes(std::vector<std::string>& types) const;

            void printRegisteredTypes(std::ostream& os, bool comment = false) const;

    protected:
      class CreatorInformation
      {
        public:
          AbstractHyperGraphElementCreator* creator;
          int elementTypeBit;
          CreatorInformation()
          {
            creator = 0;
            elementTypeBit = -1;
          }
        
          ~CreatorInformation()
          {
            std::cout << "Deleting " << (void*) creator << std::endl;
            
            delete creator;
          }
      };

      typedef std::map<std::string, CreatorInformation*>               CreatorMap;
      typedef std::map<std::string, std::string>                      TagLookup;
      Factory();
      ~Factory();

      CreatorMap _creator;     ///< look-up map for the existing creators
      TagLookup _tagLookup;    ///< reverse look-up, class name to tag

    private:
      static Factory* factoryInstance;
  };

  template<typename T>
  class RegisterTypeProxy
  {
    public:
      RegisterTypeProxy(const std::string& name) : _name(name)
      {
#ifdef G2O_DEBUG_FACTORY
        std::cout << __FUNCTION__ << ": Registering " << _name << " of type " << typeid(T).name() << std::endl;
#endif
        Factory::instance()->registerType(_name, new HyperGraphElementCreator<T>());
      }

      ~RegisterTypeProxy()
      {
#ifdef G2O_DEBUG_FACTORY
        std::cout << __FUNCTION__ << ": Unregistering " << _name << " of type " << typeid(T).name() << std::endl;
#endif
        Factory::instance()->unregisterType(_name);
      }

    private:
      std::string _name;
  };

#if defined _MSC_VER && defined G2O_SHARED_LIBS
#  define G2O_FACTORY_EXPORT __declspec(dllexport)
#  define G2O_FACTORY_IMPORT __declspec(dllimport)
#else
#  define G2O_FACTORY_EXPORT
#  define G2O_FACTORY_IMPORT
#endif

  // These macros are used to automate registering types and forcing linkage
#define G2O_REGISTER_TYPE(name, classname) \
    extern "C" void G2O_FACTORY_EXPORT g2o_type_##classname(void) {} \
    static g2o::RegisterTypeProxy<classname> g_type_proxy_##classname(#name);

#define G2O_USE_TYPE_BY_CLASS_NAME(classname) \
    extern "C" void G2O_FACTORY_IMPORT g2o_type_##classname(void); \
    static g2o::ForceLinker proxy_##classname(g2o_type_##classname);

#define G2O_REGISTER_TYPE_GROUP(typeGroupName) \
    extern "C" void G2O_FACTORY_EXPORT g2o_type_group_##typeGroupName(void) {}

#define G2O_USE_TYPE_GROUP(typeGroupName) \
    extern "C" void G2O_FACTORY_IMPORT g2o_type_group_##typeGroupName(void); \
    static g2o::ForceLinker g2o_force_type_link_##typeGroupName(g2o_type_group_##typeGroupName);
}

#endif
