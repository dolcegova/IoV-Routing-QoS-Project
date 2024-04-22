package com.move.commonData;

/**
   Thesis implementation
   Class : routeData.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/


import java.util.*;

public class routeData{

      ArrayList allRoutes;
      ArrayList allTypes;
      ArrayList allVehicles;
      boolean empty;

      public routeData(){
          allRoutes = new ArrayList();
          allTypes  = new ArrayList();
          allVehicles = new ArrayList();
      }

      public ArrayList returnRoute(){
          return allRoutes;
      }

      public ArrayList returnType(){
          return allTypes;
      }

      public ArrayList returnVehicles(){
          return allVehicles;
      }

      public void addRoutes(routeParameter route){
          allRoutes.add(route);
      }

      public void addTypes(vtypeParameter vtype){
          allTypes.add(vtype);
      }

      public void addVehicles(vehicleParameter vehicle){
          allVehicles.add(vehicle);
      }

      public void clearArrayLists(){
          allRoutes.clear();
          allTypes.clear();
          allVehicles.clear();
      }

      public void deleteRoute(int index){
           allRoutes.remove(index);
      }

      public void deleteType(int index){
           allTypes.remove(index);
      }

      public void deleteVehicle(int index){
           allVehicles.remove(index);
      }

}








