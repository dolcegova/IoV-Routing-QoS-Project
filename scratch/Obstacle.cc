/*
 * Auteur : Oussama SENOUCI 
   Nom du projet  Enhancing Routing and Quality of Service Using AI-driven Technique
for Internet of Vehicles Contexts
 */


#include "Obstacle.h"

namespace ns3
{
  TypeId Obstacle::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::Obstacle")
      .SetParent<Vehicle> ()
      .AddConstructor<Obstacle> ()
      ;
    return tid;
  }

  Obstacle::Obstacle()
  {
    SetAcceleration(0.0);
    SetVelocity(0.0);
    SetModel(0);
    SetLaneChange(0);
  }
	
  void Obstacle::Accelerate(Ptr<Vehicle> vwd)
  {
    SetAcceleration(0.0);
  }

  bool Obstacle::CheckLaneChange(Ptr<Vehicle> frontOld, Ptr<Vehicle> frontNew, Ptr<Vehicle> backNew, bool toLeft)
  {
    return false;
  }

  void Obstacle::TranslatePosition(double dt)
  {
    ;
  }

  void Obstacle::TranslateVelocity(double dt)
  {
    SetVelocity(0.0);
  }

  double Obstacle::Acceleration(Ptr<Vehicle> vwd)
  {
    return 0.0;
  }
}
