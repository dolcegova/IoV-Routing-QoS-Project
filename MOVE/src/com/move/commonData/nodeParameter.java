package com.move.commonData;
/**
   Thesis implementation
   Class : nodeParameter.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/

public class nodeParameter{

       String id;
       String X;
       String Y;
       String type;


       public nodeParameter(String identification,String Xposition,
                          String Yposition,String junctionType){

              id       = new String(identification);
              X        = new String(Xposition);
              Y        = new String(Yposition);
              type     = new String(junctionType);
       }

       public String getID(){
           return id;
       }

       public String getX(){
           return X;
       }

       public String getY(){
           return Y;
       }

       public String getType(){
           return type;
       }

}

