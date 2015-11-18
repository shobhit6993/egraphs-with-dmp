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
                                     const std::vector<double>& obs_pos,
                                     const std::string& mode) {
    int dim = robot_pos.size();
    double sum = 0;

    if (mode == "static,L2") {
        for (int d = 0; d < dim; ++d) {
            sum = sum + (robot_pos[d] - obs_pos[d]) * (robot_pos[d] - obs_pos[d]);
        }
        return sqrt(sum);
    // }
    // else if (mode == "static,L1") {
    //     for (int d = 0; d < dim; ++d) {
    //         sum = sum + abs(robot_pos[d] - obs_pos[d]);
    //     }
    //     return sum;
    } else {
        std::cout << "ERROR: invalid mode. Exiting..." << std::endl;
        exit(1);
    }
}

double CalculateDerivativeOfDistance(const double x,
                                     const double x_0,
                                     const double dist,
                                     const std::string& mode) {
    if (mode == "static,L2")
        return (x - x_0) / dist;
    // else if (mode == "static,L1")
    //     return 1;
    else {
        std::cout << "ERROR: invalid mode. Exiting..." << std::endl;
        exit(1);
    }
}

double CalculatePotentialGradient(const std::vector<double>& robot_pos,
                                  const std::vector<double>& obs_pos,
                                  const int dim_index,
                                  const double p_0,
                                  const double eta,
                                  const std::string& mode) {
    double p_x = CalculateDistanceFromObstacle(robot_pos, obs_pos, mode);
    if (p_x > p_0)
        return 0.0;

    double p_x_dash = CalculateDerivativeOfDistance(robot_pos[dim_index], obs_pos[dim_index], p_x, mode);
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
    // tau=1;
    // double start = trajectory.waypoint[0].position;
    // double goal = trajectory.waypoint[n - 1].position;

    DMPequation *new_dmp_equation = new DMPequation(k, d, tau, alpha, eta, p_0);
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
                           const string& mode,
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
                                            mode, curr_pos, curr_vel);

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
                                  const string& mode,
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

    if (dim_index == 0) std::cout << "dim= " << dim_index << std::endl;
    for (double t = 0; t <= time_resolution; t = t + dt)
    {
        s = CalculatePhase(curr_time + t, tau, alpha);
        pot_gr = CalculatePotentialGradient(curr_pos, obs_pos, dim_index, p_0, eta, mode);
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
    if (dim_index == 0) std::cout << curr_time << "\t" << x << "\t" << goal << std::endl;

    return w;
}