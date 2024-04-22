package com.move.commonData;
public class NodePosition{

       String xCoordinate;
       String yCoordinate;

       public NodePosition(String x,String y){
             xCoordinate = x;
             yCoordinate = y;
       }

       public String getX(){
         return  xCoordinate;
       }

        public String getY(){
         return  yCoordinate;
       }
}


