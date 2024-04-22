package com.move.commonData;
/**
   Thesis implementation
   Class : typeParameter.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/


public class typeParameter{

       String id;
       String name;
       String nolanes;
       String speed;
       String priority;
       String capacity;
       String parkingPossible; 

       public typeParameter(String identification,String typeName,String typePriority,
                          String typeLanes,String typeSpeed,String typeCapacity,
			  String typeParking){

          id       = new String(identification);
	      name     = new String(typeName);
          nolanes  = new String(typeLanes);
          speed    = new String(typeSpeed);
          priority = new String(typePriority);
	      capacity = new String(typeCapacity);
	      parkingPossible = new String(typeParking);
       }
       
       public String getID(){
             return id;
       }
       
       public String getName(){
             return name;
       }
       
       public String getNolanes(){
             return nolanes;
       }
       
       public String getSpeed(){
             return speed;
       }
       
       public String getPriority(){
             return priority;
       }
       
       public String getCapacity(){
             return capacity;
       }
       
       public String getParkingPossible(){
             return parkingPossible;
       }
       
       
}
