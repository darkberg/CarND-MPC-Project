#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

class MPC
{
public:
  MPC();
  virtual ~MPC();
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  vector<double> GetMpcX();
  vector<double> GetMpcY();

private:
  vector<double> mpc_x;
  vector<double> mpc_y;

};

#endif /* MPC_H */
