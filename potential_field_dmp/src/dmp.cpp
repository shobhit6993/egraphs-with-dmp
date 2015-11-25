#include "dmp.h"
#include "math.h"
#include "vector"
#include "iostream"
#include "potential_field_dmp/Trajectory_1D.h"
#include "potential_field_dmp/Parameters.h"
#include "potential_field_dmp/Plan.h"
#include "potential_field_dmp/WayPoint.h"

using namespace std;
using namespace potential_field_dmp;


double CalculatePhase(double t, double tau, double alpha) {
    return exp(-(alpha / tau) * t);
}

double CalculateAlpha() {
    return log(100);
}

bool IsNear(const std::vector<double>& robot_pos,
            const std::vector<double>& goal) {
    int dim = robot_pos.size();
    double sum = 0;

    for (int d = 0; d < dim; ++d) {
        sum = sum + (robot_pos[d] - goal[d]) * (robot_pos[d] - goal[d]);
    }
    return (sqrt(sum) < kProximityThreshold);
}

double CalculateDistanceFromObstacle(const std::vector<double>& robot_pos,
                                     const std::vector<double>& obs_pos) {
    int dim = robot_pos.size();
    double sum = 0;

    for (int d = 0; d < dim; ++d) {
        sum = sum + (robot_pos[d] - obs_pos[d]) * (robot_pos[d] - obs_pos[d]);
    }
    return sqrt(sum);
}

double CalculateDerivativeOfDistance(const double x,
                                     const double x_0,
                                     const double dist) {
    return (x - x_0) / dist;
}

double CalculatePotentialGradient(const std::vector<double>& robot_pos,
                                  const std::vector<double>& obs_pos,
                                  const int dim_index,
                                  const double p_0,
                                  const double eta) {
    double p_x = CalculateDistanceFromObstacle(robot_pos, obs_pos);
    if (p_x > p_0)
        return 0.0;

    double p_x_dash = CalculateDerivativeOfDistance(robot_pos[dim_index], obs_pos[dim_index], p_x);
    double temp = ((p_0 - p_x) * p_x_dash) / (p_0 * p_x * p_x * p_x);

    return -(eta * temp);
}

// set parameters for a 1D DMP.
// param[in] param: DMP parameters - k, d, tau
// param[in] alpha: rate of decay of phase variable
// param[out] dmp_equation vector of dmp equations
// Parameters is the tuple (k, d, tau, eta, p_0)
void SetParametersDMP_1D(const Parameters &param,
                         const double alpha,
                         std::vector<DMPequation*> &dmp_equation) {

    double k = param.k;
    double d = param.d;
    double tau = param.tau;
    double eta = param.eta;
    double p_0 = param.p_0;
    double lambda = param.lambda;
    double beta = param.beta;

    DMPequation *new_dmp_equation = new DMPequation(k, d, tau, alpha, eta, p_0, lambda, beta);
    dmp_equation.push_back(new_dmp_equation);

}

// param[in] param: vector of parameters, one for each dimension
// param[out] dmp_equation vector of dmp equations
void SetParametersDMP_nD(const std::vector<Parameters> &param,
                         std::vector<DMPequation*> &dmp_equation) {

    int num_dim = param.size();  // num of dimensions
    double alpha = CalculateAlpha();

    for (int i = 0; i < num_dim; i++) {
        SetParametersDMP_1D(param[i], alpha, dmp_equation);
    }
}

void GenerateTrajectory_nD(const vector<double> start,
                           const vector<double> goal,
                           const vector<double> initial_velocity,
                           const vector<double> obs_pos,
                           const double dt,
                           const double tau,
                           const vector<DMPequation*> &dmp_equation,
                           Plan &generated_plan) {

    int dim = dmp_equation.size();

    generated_plan.traj.resize(dim);
    double curr_time = 0.0;

    WayPoint w;
    w.acceleration = 0.0;
    w.timestep = curr_time;

    // [ push start point of each dimension in corresponding trajectory
    for (int d = 0; d < dim; ++d) {
        w.position = start[d];
        w.velocity = initial_velocity[d];
        generated_plan.traj[d].waypoint.push_back(w);
    }
    // ]

    vector<double> curr_pos(start);
    vector<double> curr_vel(initial_velocity);

    bool check = true;
    while (check) {
        check = false;
        for (int d = 0; d < dim; ++d) {
            // if (!IsNear(curr_pos[d], goal[d]) && curr_time <= dmp_equation[d]->tau) {
            if (!IsNear(curr_pos, goal) && curr_time <= dmp_equation[d]->tau) {
                w = IntegrateForOneTimestep(d, dmp_equation[d],
                                            start[d], goal[d],
                                            dt, curr_time, obs_pos,
                                            curr_pos, curr_vel);

                generated_plan.traj[d].waypoint.push_back(w);
                check = true;
            }
        }
        curr_time += dt;
    }

    size_t max_timesteps = 0;
    int m;

    for (int d = 0; d < dim; ++d) {
        if (max_timesteps < generated_plan.traj[d].waypoint.size()) {
            m = d;
            max_timesteps = generated_plan.traj[d].waypoint.size();
        }
    }

    // need to make sure that trajectories in each dimension are of equal length
    // those which are smaller will have their last x value repeated for future timesteps
    for (int i = 0; i < dim; ++i) {
        size_t num_timesteps = generated_plan.traj[i].waypoint.size();
        if (num_timesteps < max_timesteps) {

            WayPoint last_waypoint = generated_plan.traj[i].waypoint[num_timesteps - 1];
            for (size_t j = num_timesteps; j < max_timesteps; ++j) {
                last_waypoint.timestep = generated_plan.traj[m].waypoint[j].timestep;
                generated_plan.traj[i].waypoint.push_back(last_waypoint);
            }
        }
    }
}

WayPoint IntegrateForOneTimestep( const int dim_index,
                                  const DMPequation* dmp_equation,
                                  const double start,
                                  const double goal,
                                  const double time_resolution,
                                  double curr_time,
                                  const vector<double>& obs_pos,
                                  vector<double>& curr_pos,
                                  vector<double>& curr_vel) {
    double k = dmp_equation->k;
    double d = dmp_equation->d;
    double tau = dmp_equation->tau;
    double alpha = CalculateAlpha();
    double eta =  dmp_equation->eta;
    double p_0 = dmp_equation->p_0;

    double x = curr_pos[dim_index];
    double v = curr_vel[dim_index];

    double s, x_dot, v_dot, pot_gr;

    WayPoint w;

    std::cout << "dim= " << dim_index << std::endl;
    for (double t = 0; t <= time_resolution; t = t + dt)
    {
        s = CalculatePhase(curr_time + t, tau, alpha);
        pot_gr = CalculatePotentialGradient(curr_pos, obs_pos, dim_index, p_0, eta);
        // f_s = dmp_equation->interpolator->InterpolatedValue(s);
        // v_dot = (k * (goal - x) - d * v - k * (goal - start) * s + k * f_s) / tau;
        v_dot = (k * (goal - x) - d * v - k * (goal - start) * s - pot_gr) / tau;
        x_dot = v / tau;

        x = x + x_dot * dt;
        v = v + v_dot * dt;

        curr_pos[dim_index] = x;
        curr_vel[dim_index] = v;

        // std::cout << "t=" << t << " s=" << s << " pot_gr=" << pot_gr << " v_dot=" << v_dot << " x_dot=" << x_dot << " x=" << x << " v=" << v << " g=" << goal << std::endl;
    }

    w.position = x;
    w.velocity = v;
    w.acceleration = 0.0;
    w.timestep = curr_time;
    std::cout << curr_time << "\t" << x << "\t" << goal << std::endl;

    return w;
}

double CalculateRelativeSpeed(const std::vector<double>& robot_vel,
                              const std::vector<double>& obs_vel) {
    double norm_v = 0;
    double rel_vel;
    for (int i = 0; i < robot_vel.size(); ++i) {
        rel_vel = robot_vel[i] - obs_vel[i];
        norm_v = norm_v + rel_vel * rel_vel;
    }
    return sqrt(norm_v);
}

double CalculateCosTheta(const std::vector<double>& robot_pos,
                         const std::vector<double>& robot_vel,
                         const std::vector<double>& obs_pos,
                         const std::vector<double>& obs_vel,
                         double dist,
                         double rel_speed) {
    std::vector<double> rel_vel;
    std::vector<double> rel_pos;

    for (int i = 0; i < robot_pos.size(); ++i) {
        rel_pos.push_back(robot_pos[i] - obs_pos[i]);
    }

    for (int i = 0; i < robot_vel.size(); ++i) {
        rel_vel.push_back(robot_vel[i] - obs_vel[i]);
    }

    double numerator = 0;
    for (int i = 0; i < rel_vel.size(); ++i) {
        numerator += rel_vel[i] * rel_pos[i];
    }

    return numerator / (rel_speed * dist);
}

// Written in a hurry. works only for 2D. Sorry
double CalculateGradientCosine(const std::vector<double>& robot_pos,
                               const std::vector<double>& robot_vel,
                               const std::vector<double>& obs_pos,
                               const std::vector<double>& obs_vel,
                               double dist,
                               double rel_speed,
                               int dim_index) {
    std::vector<double> rel_vel;
    std::vector<double> rel_pos;

    for (int i = 0; i < robot_pos.size(); ++i) {
        rel_pos.push_back(robot_pos[i] - obs_pos[i]);
    }

    for (int i = 0; i < robot_vel.size(); ++i) {
        rel_vel.push_back(robot_vel[i] - obs_vel[i]);
    }

    int zero = dim_index;
    int one = (dim_index + 1) % 2;
    double numerator = rel_pos[one] * (rel_vel[zero] * rel_pos[one] - rel_vel[one] * rel_pos[zero]);

    return numerator / (rel_speed * pow(dist, 3));
}

double CalculatePotentialGradient(const std::vector<double>& robot_pos,
                                  const std::vector<double>& robot_vel,
                                  const std::vector<double>& obs_pos,
                                  const std::vector<double>& obs_vel,
                                  const int dim_index,
                                  const double lambda,
                                  const double beta) {
    double p = CalculateDistanceFromObstacle(robot_pos, obs_pos);
    double rel_speed = CalculateRelativeSpeed(robot_vel, obs_vel);
    double cosine =
        CalculateCosTheta(robot_pos, robot_vel,
                          obs_pos, obs_vel,
                          p, rel_speed);
    double grad_cosine =
        CalculateGradientCosine(robot_pos, robot_vel,
                                obs_pos, obs_vel,
                                p, rel_speed, dim_index);
    double grad_p =
        CalculateDerivativeOfDistance(robot_pos[dim_index],
                                      obs_pos[dim_index], p);

    double term1 = lambda * pow(cosine, beta - 1);
    double term2 = rel_speed / p;
    double term3 = beta * grad_cosine - cosine * grad_p;

    return term1 * term2 * term3;
}

void UpdateCurrObsPos(double dt,
                      const std::vector<double>& obs_vel,
                      vector<double>& curr_obs_pos) {
    for (int i = 0; i < curr_obs_pos.size(); ++i) {
        curr_obs_pos[i] += obs_vel[i] * dt;
    }
}

void GenerateTrajectoryMoving_nD(const vector<double>& start,
                                 const vector<double>& goal,
                                 const vector<double>& initial_velocity,
                                 const vector<double>& init_obs_pos,
                                 const vector<double>& obs_vel,
                                 const double dt,
                                 const double tau,
                                 const vector<DMPequation*> &dmp_equation,
                                 Plan & generated_plan) {

    int dim = dmp_equation.size();

    generated_plan.traj.resize(dim);
    double curr_time = 0.0;

    WayPoint w;
    w.acceleration = 0.0;
    w.timestep = curr_time;

    // [ push start point of each dimension in corresponding trajectory
    for (int d = 0; d < dim; ++d) {
        w.position = start[d];
        w.velocity = initial_velocity[d];
        generated_plan.traj[d].waypoint.push_back(w);
    }
    // ]

    vector<double> curr_pos(start);
    vector<double> curr_vel(initial_velocity);
    vector<double> curr_obs_pos(init_obs_pos);

    bool check = true;
    while (check) {
        check = false;
        for (int d = 0; d < dim; ++d) {
            // if (!IsNear(curr_pos[d], goal[d]) && curr_time <= dmp_equation[d]->tau) {
            if (!IsNear(curr_pos, goal) && curr_time <= dmp_equation[d]->tau) {
                w = IntegrateForOneTimestepMoving(d, dmp_equation[d],
                                                  start[d], goal[d],
                                                  dt, curr_time,
                                                  curr_obs_pos, obs_vel,
                                                  curr_pos, curr_vel);

                generated_plan.traj[d].waypoint.push_back(w);
                check = true;
            }
        }
        curr_time += dt;
    }

    size_t max_timesteps = 0;
    int m;

    for (int d = 0; d < dim; ++d) {
        if (max_timesteps < generated_plan.traj[d].waypoint.size()) {
            m = d;
            max_timesteps = generated_plan.traj[d].waypoint.size();
        }
    }

    // need to make sure that trajectories in each dimension are of equal length
    // those which are smaller will have their last x value repeated for future timesteps
    for (int i = 0; i < dim; ++i) {
        size_t num_timesteps = generated_plan.traj[i].waypoint.size();
        if (num_timesteps < max_timesteps) {

            WayPoint last_waypoint = generated_plan.traj[i].waypoint[num_timesteps - 1];
            for (size_t j = num_timesteps; j < max_timesteps; ++j) {
                last_waypoint.timestep = generated_plan.traj[m].waypoint[j].timestep;
                generated_plan.traj[i].waypoint.push_back(last_waypoint);
            }
        }
    }
}

WayPoint IntegrateForOneTimestepMoving( const int dim_index,
                                        const DMPequation * dmp_equation,
                                        const double start,
                                        const double goal,
                                        const double time_resolution,
                                        double curr_time,
                                        vector<double>& curr_obs_pos,
                                        const vector<double>& obs_vel,
                                        vector<double>& curr_pos,
                                        vector<double>& curr_vel) {
    double k = dmp_equation->k;
    double d = dmp_equation->d;
    double tau = dmp_equation->tau;
    double alpha = CalculateAlpha();
    double lambda =  dmp_equation->lambda;
    double beta = dmp_equation->beta;

    // if we make this relative, we will have to change distanc computation etc in calcPotGrad fn.
    double x = curr_pos[dim_index];
    double v = curr_vel[dim_index];

    double s, x_dot, v_dot, pot_gr;

    WayPoint w;

    std::cout << "dim= " << dim_index << std::endl;
    for (double t = 0; t <= time_resolution; t = t + dt) {
        s = CalculatePhase(curr_time + t, tau, alpha);
        pot_gr = CalculatePotentialGradient(curr_pos, curr_vel,
                                            curr_obs_pos, obs_vel,
                                            dim_index, lambda, beta);

        v_dot = (k * (goal - x) - d * v - k * (goal - start) * s - pot_gr) / tau;
        x_dot = v / tau;

        x = x + x_dot * dt;
        v = v + v_dot * dt;

        curr_pos[dim_index] = x;
        curr_vel[dim_index] = v;
        UpdateCurrObsPos(dt, obs_vel, curr_obs_pos);

        // std::cout << "t=" << t << " s=" << s << " pot_gr=" << pot_gr << " v_dot=" << v_dot << " x_dot=" << x_dot << " x=" << x << " v=" << v << " g=" << goal << std::endl;
    }

    w.position = x;
    w.velocity = v;
    w.acceleration = 0.0;
    w.timestep = curr_time;
    std::cout << curr_time << "\t" << x << "\t" << goal << std::endl;

    return w;
}