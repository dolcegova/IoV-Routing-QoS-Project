/*
 * Auteur : Oussama SENOUCI 
   Nom du projet  Enhancing Routing and Quality of Service Using AI-driven Technique
for Internet of Vehicles Contexts
 */


#ifndef CLASS_OBSTACLE_
#define CLASS_OBSTACLE_

#include "ns3/ptr.h"
#include "Vehicle.h"

namespace ns3
{
  /**
  * \brief Obstacle is a static Vehicle with no mobility (its Velocity = Acceleration = Model = LaneChange = 0).  
  *
  * An Obstacle can be used as an barrier, a road obstacle, a VANET RSU, or a station along/side the roadway [Highway].
  * An Obstacle has all the capabilities of a Vehicle except it can not be mobile.
  */
  class Obstacle: public ns3::Vehicle
  {
    public:

      /// Override TypeId.
      static TypeId GetTypeId (void);
      /// Constructor to initialized velocity, acceleration, model, lanechange to zero.
      Obstacle();
      /// Never accelerates. Set its acceleration always to 0.
      virtual void Accelerate(Ptr<Vehicle> vwd);
      /// Never changelanes. Always returns false. 
      virtual bool CheckLaneChange(Ptr<Vehicle> frontOld, Ptr<Vehicle> frontNew, Ptr<Vehicle> backNew, bool toLeft);
      /// Never moves. This function must do nothing.
      virtual void TranslatePosition(double dt);
      /// Never speeds. Set its velocity always to 0.
      virtual void TranslateVelocity(double dt);
      /// Never accelerates and returns 0. Since the returned computed acceleration must be always 0 for a static object.
      virtual double Acceleration(Ptr<Vehicle> vwd);
  };
};
#endif
