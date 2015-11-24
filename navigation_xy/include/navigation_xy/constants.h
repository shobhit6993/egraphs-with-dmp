#include <cmath>
#include <string>

const int kNumMarkers = 1;
const int kOffset = 30;
const int kDOF = 2;

const std::string kParametersFile = "/home/shobhit/egraphs-with-dmp/sandbox/navigation_xy/config/params.in";

const double kParamK = 10.0;
const double kParamD = 2 * sqrt(kParamD);
const double kParamTau = 1.0;
const double kParamEta = 10.0;
const double kParamP_0 = 2.0;
const double kParamDT = 0.1;

const double kSpeed = 0.1;

const double kRobotSpeed = 1;
const double kObsSpeed = 0.7;

const int kSleep = 200 * 1000;