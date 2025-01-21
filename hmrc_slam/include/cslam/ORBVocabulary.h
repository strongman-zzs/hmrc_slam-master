
#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

//C++
#include <boost/shared_ptr.hpp>

//Thirdparty
#include"thirdparty/DBoW2/DBoW2/FORB.h"
#include"thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace hmrc_slam
{

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
  ORBVocabulary;

typedef boost::shared_ptr<ORBVocabulary> vocptr;

} //end ns

#endif // ORBVOCABULARY_H
