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

bool IsNear(const double curr, const double goal) {
    return (abs(curr - goal) <= kProximityThreshold);
}

double CalculateDistanceFromObstacle(double x, double obs_pos) {
    return abs(x - obs_pos);
}

double CalculateDerivativeOfDistance(double x) {
    return 1;
}

double CalculatePotentialGradient(double x, double eta, double p_0, double obs_pos) {
    double p_x = CalculateDistanceFromObstacle(x, obs_pos);
    double p_x_dash = CalculateDerivativeOfDistance(x);
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

    DMPequation *new_dmp_equation = new DMPequation(k, d, alpha, eta, p_0);
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

//
void GenerateTrajectory_1D(const double start,
                           const double goal,
                           const double initial_velocity,
                           const double obs_pos,
                           const double time_resolution,
                           const double tau,
                           const DMPequation* dmp_equation,
                           Trajectory_1D &trajectory) {

    trajectory.waypoint.clear();

    double k = dmp_equation->k;
    double d = dmp_equation->d;
    // double tau = dmp_equation->tau;
    double alpha = CalculateAlpha();
    double eta =  dmp_equation->eta;
    double p_0 = dmp_equation->p_0;

    double x = start;
    double v = initial_velocity;
    double curr_time = 0.0;

    double s, x_dot, v_dot, pot_gr;

    WayPoint w;
    w.position = x;
    w.velocity = v;
    w.acceleration = 0.0;
    w.timestep = curr_time;

    trajectory.waypoint.push_back(w);
    // std::cout << x << " 1d " << goal << std::endl;
    // while (curr_time < tau) {
    while (!IsNear(x, goal) && curr_time < 10) {
        // calculates x and v at t=timestep by integrating
        for (double t = 0; t <= time_resolution; t = t + dt)
        {
            s = CalculatePhase(curr_time + t, tau, alpha);
            pot_gr = CalculatePotentialGradient(x, eta, p_0, obs_pos);
            // f_s = dmp_equation->interpolator->InterpolatedValue(s);
            // v_dot = (k * (goal - x) - d * v - k * (goal - start) * s + k * f_s) / tau;
            v_dot = (k * (goal - x) - d * v - k * (goal - start) * s - pot_gr) / tau;
            x_dot = v / tau;

            x = x + x_dot * dt;
            v = v + v_dot * dt;

            std::cout << "t=" << t << " s=" << s << " pot_gr=" << pot_gr << " v_dot=" << v_dot << " x_dot=" << x_dot << " x=" << x << " v=" << v << " g=" << goal << std::endl;
        }
        std::cout << "---------" << std::endl;

        // does not compute and return acceleration values in WayPoint
        w.position = x;
        w.velocity = v;
        w.acceleration = 0.0;
        w.timestep = curr_time;
        std::cout << curr_time << "\t" << x << "\t" << goal << std::endl;

        trajectory.waypoint.push_back(w);
        curr_time += time_resolution;
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
    Trajectory_1D trajectory_1d;
    size_t max_timesteps = 0;
    int m;

    for (int i = 0; i < dim; ++i) {
        trajectory_1d.waypoint.clear();
        GenerateTrajectory_1D(start[i], goal[i], initial_velocity[i], obs_pos[i], dt, tau, dmp_equation[i], trajectory_1d);
        generated_plan.traj.push_back(trajectory_1d);

        if (max_timesteps < trajectory_1d.waypoint.size()) {
            m = i;
            max_timesteps = trajectory_1d.waypoint.size();
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