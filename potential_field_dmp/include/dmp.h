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

  DMPequation(double k, double d, double tau,
              double alpha, double eta, double p_0,
              double lambda, double beta) :
    k(k), d(d), tau(tau), alpha(alpha), eta(eta), p_0(p_0), lambda(lambda), beta(beta) { }

  double k;
  double d;
  double tau;
  double alpha;
  double eta;
  double p_0;
  double lambda;
  double beta;

};

void SetParametersDMP_1D(const Parameters &param,
                         const double alpha,
                         std::vector<DMPequation*> &dmp_equation);

void SetParametersDMP_nD(const std::vector<Parameters> &param,
                         std::vector<DMPequation*> &dmp_equation);

// [ generic functions
double CalculatePhase(double t, double tau, double alpha);
double CalculateAlpha();
bool IsNear(const std::vector<double>& robot_pos,
            const std::vector<double>& goal);
double CalculateDistanceFromObstacle(const std::vector<double>& robot_pos,
                                     const std::vector<double>& obs_pos);
// ]

// [ for static obstacles
WayPoint IntegrateForOneTimestep( const int dim_index,
                                  const DMPequation* dmp_equation,
                                  const double start,
                                  const double goal,
                                  const double time_resolution,
                                  const double curr_time,
                                  const vector<double>& obs_pos,
                                  vector<double>& curr_pos,
                                  vector<double>& curr_vel
                                );

void GenerateTrajectory_nD(const vector<double> start,
                           const vector<double> goal,
                           const vector<double> initial_velocity,
                           const vector<double> obs_pos,
                           const double dt,
                           const double tau,
                           const vector<DMPequation*> &dmp_equation,
                           Plan &generated_plan);

double CalculatePotentialGradient(const std::vector<double>& robot_pos,
                                  const std::vector<double>& obs_pos,
                                  const int dim_index,
                                  const double p_0,
                                  const double eta);
double CalculateDerivativeOfDistance(const double x,
                                     const double x_0,
                                     const double dist);
// ]

// [ for moving obstacles
double CalculateRelativeSpeed(const std::vector<double>& robot_vel,
                              const std::vector<double>& obs_pos);
double CalculateCosTheta(const std::vector<double>& robot_pos,
                         const std::vector<double>& robot_vel,
                         const std::vector<double>& obs_pos,
                         const std::vector<double>& obs_vel,
                         double dist,
                         double rel_speed);
double CalculateGradientCosine(const std::vector<double>& robot_pos,
                               const std::vector<double>& robot_vel,
                               const std::vector<double>& obs_pos,
                               const std::vector<double>& obs_vel,
                               double dist,
                               double rel_speed,
                               int dim_index);
double CalculatePotentialGradient(const std::vector<double>& robot_pos,
                                  const std::vector<double>& robot_vel,
                                  const std::vector<double>& obs_pos,
                                  const std::vector<double>& obs_vel,
                                  const int dim_index,
                                  const double lambda,
                                  const double beta);
void UpdateCurrObsPos(double dt,
                      const std::vector<double>& obs_vel,
                      vector<double>& curr_obs_pos);
void GenerateTrajectoryMoving_nD(const vector<double>& start,
                                 const vector<double>& goal,
                                 const vector<double>& initial_velocity,
                                 const vector<double>& init_obs_pos,
                                 const vector<double>& obs_vel,
                                 const double dt,
                                 const double tau,
                                 const vector<DMPequation*> &dmp_equation,
                                 Plan & generated_plan);
WayPoint IntegrateForOneTimestepMoving( const int dim_index,
                                        const DMPequation * dmp_equation,
                                        const double start,
                                        const double goal,
                                        const double time_resolution,
                                        double curr_time,
                                        vector<double>& curr_obs_pos,
                                        const vector<double>& obs_vel,
                                        vector<double>& curr_pos,
                                        vector<double>& curr_vel);
// ]

#endif //DMP_H_