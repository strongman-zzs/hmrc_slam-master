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

#ifndef G2O_STRING_TOOLS_H
#define G2O_STRING_TOOLS_H

#include <string>
#include <sstream>
#include <cstdlib>
#include <vector>

#include "macros.h"

namespace g2o {

// @{


 std::string trim(const std::string& s);

 std::string trimLeft(const std::string& s);

 std::string trimRight(const std::string& s);

 std::string strToLower(const std::string& s);

 std::string strToUpper(const std::string& s);

template <typename OutputIterator>
OutputIterator readInts(const char* str, OutputIterator out)
{
  char* cl  = (char*)str;
  char* cle = cl;
  while (1) {
    int id = strtol(cl, &cle, 10);
    if (cl == cle)
      break;
    *out++ = id;
    cl = cle;
  }
  return out;
}

template <typename OutputIterator>
OutputIterator readFloats(const char* str, OutputIterator out)
{
  char* cl  = (char*)str;
  char* cle = cl;
  while (1) {
    double val = strtod(cl, &cle);
    if (cl == cle)
      break;
    *out++ = val;
    cl = cle;
  }
  return out;
}

 std::string formatString(const char* fmt, ...) G2O_ATTRIBUTE_FORMAT12;

 int strPrintf(std::string& str, const char* fmt, ...) G2O_ATTRIBUTE_FORMAT23;

template<typename T>
bool convertString(const std::string& s, T& x, bool failIfLeftoverChars = true)
{
  std::istringstream i(s);
  char c;
  if (!(i >> x) || (failIfLeftoverChars && i.get(c)))
    return false;
  return true;
}

template<typename T>
T stringToType(const std::string& s, bool failIfLeftoverChars = true)
{
  T x;
  convertString(s, x, failIfLeftoverChars);
  return x;
}

 bool strStartsWith(const std::string & str, const std::string& substr);

 bool strEndsWith(const std::string & str, const std::string& substr);

 std::string strExpandFilename(const std::string& filename);

 std::vector<std::string> strSplit(const std::string& s, const std::string& delim);

 int readLine(std::istream& is, std::stringstream& currentLine);

// @}

} // end namespace

#endif
