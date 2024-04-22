/*
 * Auteur : Oussama SENOUCI 
   Nom du projet  Enhancing Routing and Quality of Service Using AI-driven Technique
for Internet of Vehicles Contexts
 */


#ifndef CLASS_CONTROLLER_
#define CLASS_CONTROLLER_

#include "Highway.h"
#include "Obstacle.h"

using namespace ns3;
using namespace std;

namespace ns3
{

  /**
  * \brief Controller is the main class to manage the events (callbacks), traces, rules, and etc.
  *
  * Controller can be assumed as an application which is tied with the highway and vehicles.
  * we implement the VANETs simulations here, design and form the basic of each experiments. 
  */
  class Controller : public Object
  {
    private:
      double T;
      Ptr<Highway> highway;
    public:
      /// Constructor.
      Controller();
      /// Constructor.
      Controller(Ptr<Highway> highway);
      /// to broadcast a warning by vehicle veh. 
      void BroadcastWarning(Ptr<Vehicle> veh);
      /// event handler for ReceiveData callbacks.
      void ReceiveData (Ptr<Vehicle> veh, Ptr<const Packet> packet, Address address);
      /// event handler for ControlVehicle callbacks.
      bool ControlVehicle(Ptr<Highway> highway, Ptr<Vehicle> vehicle, double dt);
      /// event handler for InitVehicle callbacks.
      bool InitVehicle(Ptr<Highway> highway,  int& VID);
      /// sets the highway bound to this controller.
      void SetHighway(Ptr<Highway> highway);
      /// returns the highway bound to this controller.
      Ptr<Highway> GetHighway();
      /// a flag for plotting vehicles.
      bool Plot;
  };
}
#endif
