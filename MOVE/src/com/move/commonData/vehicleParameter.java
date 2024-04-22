package com.move.commonData;
/**
   Thesis implementation
   Class : vehicleParameter.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/


public class vehicleParameter{

       String id;
       String route;
       String type;
       String depart;
       String period;
       String repno;


       public vehicleParameter(String identification,String routeID,
                          String typeID, String departTime, String periodTime, String repnumber){

              id           = new String(identification);
              route        = new String(routeID);
              type         = new String(typeID);
              depart       = new String(departTime);
              period       = new String(periodTime);
              repno        = new String(repnumber);
       }

       public String getID(){
           return id;
       }

       public String getRoute(){
           return route;
       }

       public String getType(){
           return type;
       }

       public String getDepart(){
           return depart;
       }

       public String getPeriod(){
           return period;
       }

       public String getRepno(){
           return repno;
       }
}

