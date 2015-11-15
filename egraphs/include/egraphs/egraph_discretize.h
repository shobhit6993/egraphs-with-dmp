#ifndef EGRAPH_DISCRETIZE
#define EGRAPH_DISCRETIZE

#include<vector>

class EGraphDiscretize{
  public:
    //This function takes a continuous state vector and discretizes it. 
    //The planner assumes that states that have the same discretized coordinate
    //are equivalent.
    //The implementation usually involves dividing the continuous elements by
    //the resolution for that dimension. 
    //This function MUST be an inverse of discToCont
    //Use TestEGraphable if you want to confirm this.
    virtual void contToDisc(const std::vector<double>& c, std::vector<int>& d) = 0;

    //This function takes a discrete state vector and makes it continuous.
    //The implementation usually involves multiplying the discrete elements by
    //the resolution for that dimension (the choice of adding a "half cell" on 
    //is up to you....this is the difference between have continuous states in
    //the center of cells or on the border).
    //This function MUST be an inverse of contToDisc
    //Use TestEGraphable if you want to confirm this.
    virtual void discToCont(const std::vector<int>& d, std::vector<double>& c) = 0;
};

#endif
