
#ifndef __D_T_FEATURE_VECTOR__
#define __D_T_FEATURE_VECTOR__

#include "BowVector.h"
#include <map>
#include <vector>
#include <iostream>

namespace DBoW2 {

/// Vector of nodes with indexes of local features
class FeatureVector: 
  public std::map<NodeId, std::vector<unsigned int> >
{
public:

    FeatureVector(void);
  
    ~FeatureVector(void);
  
    void addFeature(NodeId id, unsigned int i_feature);

    friend std::ostream& operator<<(std::ostream &out, const FeatureVector &v);
    
};

} // namespace DBoW2

#endif

