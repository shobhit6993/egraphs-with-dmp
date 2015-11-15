#ifndef DMP_H_
#define DMP_H_

#include "iostream"
#include "potential_field_dmp/Trajectory_1D.h"
#include "potential_field_dmp/Parameters.h"
#include "potential_field_dmp/Plan.h"
#include "potential_field_dmp/WayPoint.h"

using namespace std;
using namespace potential_field_dmp;

const double kProximityThreshold = 0.01;
const double dt = 0.001;

class DMPequation {
public:

    DMPequation(double k, double d, double alpha, double eta, double p_0) :
        k(k), d(d), alpha(alpha), eta(eta), p_0(p_0) { }

    double k;
    double d;
    double alpha;
    double eta;
    double p_0;

};

void SetParametersDMP_1D(const Parameters &param,
                         const double alpha,
                         std::vector<DMPequation*> &dmp_equation);

void SetParametersDMP_nD(const std::vector<Parameters> &param,
                         std::vector<DMPequation*> &dmp_equation);

void GenerateTrajectory_1D(const double start,
                           const double goal,
                           const double initial_velocity,
                           const double obs_pos,
                           const double dt,
                           const double tau,
                           const DMPequation* dmp_equation,
                           Trajectory_1D &trajectory);

void GenerateTrajectory_nD(const vector<double> start,
                           const vector<double> goal,
                           const vector<double> initial_velocity,
                           const vector<double> obs_pos,
                           const double dt,
                           const double tau,
                           const vector<DMPequation*> &dmp_equation,
                           Plan &generated_plan);

double CalculatePhase(double t, double tau, double alpha);
double CalculateAlpha();
bool IsNear(const double curr, const double goal);
double CalculateDistanceFromObstacle(double x, double obs_pos);
double CalculateDerivativeOfDistance(double x);
double CalculatePotentialGradient(double x, double eta, double p_0, double obs_pos);



#endif //DMP_H_