#include "kdl_ros_control/kdl_planner.h"

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

/////////////////////////////////////////////////////////

void KDLPlanner::trapezoidal_vel(double t,double tc,double &s,double &s_d,double &s_dd){
  
  double s_ddot_c = 6.0/(std::pow(trajDuration_,2));

  if(t <= tc)
  {
    s=0.5*s_ddot_c*std::pow(t,2);
    s_d = s_ddot_c*t;
    s_dd = s_ddot_c;
  }
  else if(t <= trajDuration_-tc)
  {
    s=s_ddot_c*tc*(t-tc/2);
    s_d = s_ddot_c*tc;
    s_dd = 0;
  }
  else
  {
    s=1-0.5*s_ddot_c*std::pow(trajDuration_-tc,2);
    s_d = s_ddot_c*(trajDuration_-t);
    s_dd = -s_ddot_c;
  }
}

void KDLPlanner::cubic_polinomial(double t,double &s,double &s_d,double &s_dd){
  double a_2=3/(std::pow(trajDuration_,2));
  double a_3=-2/(std::pow(trajDuration_,3));

  s=a_3*std::pow(t,3)+a_2*std::pow(t,2);
  s_d =3*a_3*std::pow(t,2)+2*a_2*t;
  s_dd = 6*a_3*t+2*a_2;
}

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius){
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
}

KDLPlanner::KDLPlanner(double _trajDuration,double _trajRadius,double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    trajRadius_ = _trajRadius;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

trajectory_point KDLPlanner::compute_trajectory(double time,int choice)
{
  double s,s_d,s_dd;
  cubic_polinomial(time,s,s_d,s_dd);
  //trapezoidal_vel(time,0.7,s,s_d,s_dd);
  
  trajectory_point traj;

  if(choice < 2)
  {
    // Create circular trajectory in the y-z plane
    traj.pos.x() = trajInit_.x();
    traj.pos.y() = trajInit_.y() - trajRadius_ * cos(2 * 3.14 * s)+trajRadius_;
    traj.pos.z() = trajInit_.z() - trajRadius_ * sin(2 * 3.14 * s);

    // Set velocity and acceleration based on derivatives
    traj.vel.y() = trajRadius_ * 2 * 3.14 * s_d * sin(2 * 3.14 * s);
    traj.vel.z() = -trajRadius_ * 2 * 3.14 * s_d * cos(2 * 3.14 * s);

    traj.acc.y() = trajRadius_ * (2 * 3.14) * s_dd * sin(2 * 3.14 * s)+trajRadius_ * (2 * 3.14) *(2 * 3.14) * std::pow(s_d,2)* cos(2 * 3.14 * s);
    traj.acc.z() = trajRadius_ * (2 * 3.14) *(2 * 3.14) * std::pow(s_d,2) * sin(2 * 3.14 * s)-trajRadius_ * (2 * 3.14) * s_dd * cos(2 * 3.14 * s);
  }
  else
  {
    // Create linear trajectory 
    traj.pos = trajInit_ + s*(trajEnd_-trajInit_);
    traj.vel = s_d*(trajEnd_-trajInit_);
    traj.acc = s_dd*(trajEnd_-trajInit_);
   
  }
   return traj;
}

