package com.move.commonData;

/**
   Thesis implementation
   Class : nodeData.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/

import java.util.*;

public class nodeData{

      ArrayList allNodes;
      boolean empty;

      public nodeData(){
          allNodes = new ArrayList();
      }

      public ArrayList returnData(){
          return allNodes;
      }

      public void addNodes(nodeParameter node){
          allNodes.add(node);
      }

      public void clearNodes(){
          allNodes.clear();
      }

      public boolean isEmpty(){
          empty=allNodes.isEmpty();
          return empty;
      }

      public void deleteNode(int index){
           allNodes.remove(index);
      }

}








