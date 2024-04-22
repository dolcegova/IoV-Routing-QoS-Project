/*
 * Auteur : Oussama SENOUCI 
   Nom du projet  Enhancing Routing and Quality of Service Using AI-driven Technique
for Internet of Vehicles Contexts
 */

#include "Model.h"
#include <math.h>

namespace ns3
{
  TypeId Model::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::Model")
      .SetParent<Object> ()
      ;
    return tid;
  }
	
  double Model::CalculateAcceleration(Ptr<Vehicle> bwd, Ptr<Vehicle> vwd)
  {
    if (vwd != 0/*null*/)
      {
        double delta_v = bwd->GetVelocity() - vwd->GetVelocity();
        double s = (bwd->GetDirection())*(vwd->GetPosition().x - bwd->GetPosition().x) - bwd->GetLength(); //pos: in the back of vehicles
        double vel = bwd->GetVelocity();
        double s_star_raw = m_minimumGap + vel * m_timeHeadway + (vel * delta_v) / (2 * m_sqrtAccDec);
        double s_star = (s_star_raw > m_minimumGap) ? s_star_raw : m_minimumGap;
        double acc = m_acceleration * (1 - pow(vel / m_desiredVelocity, m_deltaV) - (s_star * s_star) / (s * s));
        return acc;
      }
    else
      {
        double delta_v = bwd->GetVelocity() - 25.0;
        double s = 500;
        double vel = bwd->GetVelocity();
        double s_star_raw = m_minimumGap + vel * m_timeHeadway + (vel * delta_v) / (2 * m_sqrtAccDec);
        double s_star = (s_star_raw > m_minimumGap) ? s_star_raw : m_minimumGap;
        double acc = m_acceleration * (1 - pow(vel / m_desiredVelocity, m_deltaV) - (s_star * s_star) / (s * s));
	    return acc;
      }
  }

  void Model::SetDesiredVelocity(double desiredVelocity)
  {
    m_desiredVelocity=desiredVelocity;
  }

  double Model::GetDesiredVelocity()
  {
    return m_desiredVelocity;
  }

  void Model::SetDeltaV(double deltaV)
  {
    m_deltaV=deltaV;
  }

  double Model::GetDeltaV()
  {
    return m_deltaV;
  }

  void Model::SetAcceleration(double acceleration)
  {
    m_acceleration=acceleration;
  }

  double Model::GetAcceleration()
  {
    return m_acceleration;
  }

  void Model::SetDeceleration(double deceleration)
  {
    m_deceleration=deceleration;
  }

  double Model::GetDeceleration()
  {
    return m_deceleration;
  }

  void Model::SetMinimumGap(double minimumGap)
  {
    m_minimumGap=minimumGap;
  }

  double Model::GetMinimumGap()
  {
    return m_minimumGap;
  }

  void Model::SetTimeHeadway(double timeHeadway)
  {
    m_timeHeadway=timeHeadway;
  }

  double Model::GetTimeHeadway()
  {
    return m_timeHeadway;
  }

  void Model::SetSqrtAccelerationDeceleration(double sqrtAccDec)
  {
    m_sqrtAccDec=sqrtAccDec;
  }

  double Model::GetSqrtAccelerationDeceleration()
  {
    return m_sqrtAccDec; 
  }
}
