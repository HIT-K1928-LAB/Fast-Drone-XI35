#include "yopo_minco_planner/minco_solver.h"
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace yopo_minco_planner {
static Eigen::Matrix<double,1,6> basis(double t, int d) {
  Eigen::Matrix<double,1,6> b = Eigen::Matrix<double,1,6>::Zero();
  for (int k=d;k<6;++k) {
    double f=1; for(int j=0;j<d;++j) f*=k-j;
    b[k]=f*std::pow(t,k-d);
  }
  return b;
}
bool MincoSolver::solve(const Eigen::Matrix3d& head,const Eigen::Matrix3d& tail,
                        const Eigen::Vector3d& inner,const Eigen::Vector2d& durations) {
  valid_=false;
  if(!head.allFinite() || !tail.allFinite() || !inner.allFinite() ||
     !durations.allFinite() || durations.minCoeff()<0.1 || durations.maxCoeff()>120.) return false;
  Eigen::Matrix<double,12,12> A=Eigen::Matrix<double,12,12>::Zero();
  Eigen::Matrix<double,12,3> B=Eigen::Matrix<double,12,3>::Zero();
  for(int d=0;d<3;++d) { A.block<1,6>(d,0)=basis(0,d); B.row(d)=head.row(d); }
  A.block<1,6>(3,0)=basis(durations[0],0); B.row(3)=inner.transpose();
  for(int d=0;d<=4;++d) {
    A.block<1,6>(4+d,0)=basis(durations[0],d);
    A.block<1,6>(4+d,6)=-basis(0,d);
  }
  for(int d=0;d<3;++d) { A.block<1,6>(9+d,6)=basis(durations[1],d); B.row(9+d)=tail.row(d); }
  // Equilibrate rows and columns before QR; do not explicitly invert A.
  Eigen::Matrix<double,12,12> scaled=A;
  Eigen::Matrix<double,12,3> rhs=B;
  for(int r=0;r<12;++r) { double f=scaled.row(r).cwiseAbs().maxCoeff(); scaled.row(r)/=f; rhs.row(r)/=f; }
  Eigen::Matrix<double,12,1> col;
  for(int c=0;c<12;++c) { col[c]=scaled.col(c).norm(); scaled.col(c)/=col[c]; }
  auto qr=scaled.colPivHouseholderQr();
  if(qr.rank()!=12) return false;
  coefficients_=qr.solve(rhs);
  for(int c=0;c<12;++c) coefficients_.row(c)/=col[c];
  const double error=(A*coefficients_-B).cwiseAbs().maxCoeff();
  valid_=coefficients_.allFinite() && error<1e-6*(1+B.cwiseAbs().maxCoeff());
  durations_=durations;
  return valid_;
}
Eigen::Vector3d MincoSolver::evaluate(double t,int derivative) const {
  if(!valid_ || !std::isfinite(t) || derivative<0 || derivative>4) throw std::runtime_error("Invalid MINCO evaluation");
  t=std::clamp(t,0.,duration());
  int piece=t>=durations_[0] ? 1 : 0;
  if(piece) t-=durations_[0];
  return (basis(t,derivative)*coefficients_.block<6,3>(piece*6,0)).transpose();
}
}
