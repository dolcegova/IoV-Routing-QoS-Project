package com.move.commonData;
/**
   Thesis implementation
   Class : vehInfo.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/


public class vehInfo{

       String id;
       String pos;
       String speed;

       public vehInfo(String identification,String position,
                          String speedPara){

              id       = new String(identification);
              pos      = new String(position);
              speed    = new String(speedPara);

       }

       public String getID(){
           return id;
       }

       public String getPosition(){
           return pos;
       }

       public String getSpeed(){
           return speed;
       }
}

