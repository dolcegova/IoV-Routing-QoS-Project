package com.move.commonData;
/**
   Thesis implementation
   Class : vtypeParameter.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/


public class vtypeParameter{

       String id;
       String accel;
       String decel;
       String sigma;
       String length;
       String maxspeed;


       public vtypeParameter(String identification,String acceleration,
                          String deceleration,String vsigma, String vlength, String vmaxspeed){

              id       = new String(identification);
              accel    = new String(acceleration);
              decel    = new String(deceleration);
              sigma    = new String(vsigma);
              length   = new String(vlength);
              maxspeed = new String(vmaxspeed);
       }

       public String getID(){
           return id;
       }

       public String getAccel(){
           return accel;
       }

       public String getDecel(){
           return decel;
       }

       public String getSigma(){
           return sigma;
       }

       public String getLength(){
           return length;
       }

       public String getMaxspeed(){
           return maxspeed;
       }

}

