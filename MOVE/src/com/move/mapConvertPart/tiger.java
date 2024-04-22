package com.move.mapConvertPart;
import java.io.*;
import java.util.*;

import com.move.construct;
import com.move.commonData.NodePosition;
import com.move.commonData.edgeData;
import com.move.commonData.edgeParameter;
import com.move.commonData.nodeData;
import com.move.commonData.nodeParameter;

public class tiger {
	String typename; // For type definitions file
	String fullname;
	String outputname;

	public tiger(String full, String output) {
		fullname = full;
		outputname = output;
	}

	public void convertData() {
		String[] Tokens;
		String edgeID;
		String typeID;
		String xFrom;
		String yFrom;
		String xTo;
		String yTo;

		int node_counter = 0;
		List<String> node_ = new ArrayList<String>();
		List<String> nodeX_ = new ArrayList<String>();
		List<String> nodeY_ = new ArrayList<String>();

		String keyFrom;
		String keyTo;

		edgeData edges = new edgeData();
		nodeData nodes = new nodeData();
		int index = 0;

		try {
			File filterfile = new File(fullname);
			BufferedReader getContent = new BufferedReader(new FileReader(
					filterfile));
			StringBuffer tmpcontent = new StringBuffer();
			String line = getContent.readLine();

			HashMap map = new HashMap();

			while (line != null) {
				boolean findNode = false;
				Tokens = line.split(" ");
				edgeID = Tokens[0];
				typeID = Tokens[1];
				xFrom = Tokens[2];
				yFrom = Tokens[3];
				xTo = Tokens[4];
				yTo = Tokens[5];

				keyFrom = xFrom + yFrom;
				NodePosition tempNode = (NodePosition) map
						.get((Object) keyFrom);
				NodePosition nodePos;

				findNode = false;
				if (tempNode == null) {
					Integer nodeID = new Integer(index);
					nodePos = new NodePosition(xFrom, yFrom);
					nodeParameter node = new nodeParameter(("node" + nodeID
							.toString()), xFrom, yFrom, "priority");

					String temp_X = xFrom;
					String temp_Y = yFrom;
					
					if (node_counter == 0) {
						node_.add(("node" + nodeID.toString()));
						nodeX_.add(temp_X);
						nodeY_.add(temp_Y);						
						findNode = true;
						node_counter++;
					} else {
						for (int check = 0; check < node_counter; check++) {
							if (nodeX_.get(check).compareTo(temp_X) == 0 && 
									nodeY_.get(check).compareTo(temp_Y) == 0) {
								findNode = true;
								break;
							}
						}
					}
					if (findNode == false) {
						node_.add(("node" + nodeID.toString()));
						nodeX_.add(temp_X);
						nodeY_.add(temp_Y);
						node_counter++;
					}

					index++;
					nodes.addNodes(node);
					map.put((Object) keyFrom, (Object) nodePos);
				}

				/*
				 * else if((xFrom.compareTo(tempNode.getX())!=0)
				 * &&(yFrom.compareTo(tempNode.getY())!=0)){
				 * //System.out.println("testing begin!!!!!"); Integer nodeID =
				 * new Integer(index); nodePos = new NodePosition(xFrom,yFrom);
				 * nodeParameter node = new
				 * nodeParameter(("node"+nodeID.toString()),xFrom,
				 * yFrom,"priority"); index++; nodes.addNodes(node);
				 * map.put((Object)keyFrom,(Object)nodePos);
				 * //System.out.println("testing!!!!!"); }
				 */

				keyTo = xTo + yTo;
				tempNode = (NodePosition) map.get((Object) keyTo);

				// System.out.println("xFrom: "+xFrom+" yFrom: "+yFrom+
				// "xTo: "+xTo+" yTo: "+yTo);
				// System.out.println("xt: "+ */
				
				findNode = false;
				if (tempNode == null) {
					Integer nodeID = new Integer(index);
					nodePos = new NodePosition(xTo, yTo);
					nodeParameter node = new nodeParameter(("node" + nodeID
							.toString()), xTo, yTo, "priority");

					String temp_X = xTo;
					String temp_Y = yTo;
					
					if (node_counter == 0) {
						node_.add(("node" + nodeID.toString()));
						nodeX_.add(temp_X);
						nodeY_.add(temp_Y);						
						findNode = true;
						node_counter++;
					} else {
						for (int check = 0; check < node_counter; check++) {
							if (nodeX_.get(check).compareTo(temp_X) == 0 && 
									nodeY_.get(check).compareTo(temp_Y) == 0) {
								findNode = true;
								break;
							}
						}
					}
					if (findNode == false) {
						node_.add(("node" + nodeID.toString()));
						nodeX_.add(temp_X);
						nodeY_.add(temp_Y);
						node_counter++;
					}
					
					index++;
					nodes.addNodes(node);
					map.put((Object) keyTo, (Object) nodePos);
				}

				/*
				 * else if((xTo.compareTo(tempNode.getX())!=0)
				 * &&(yTo.compareTo(tempNode.getY())!=0)){ Integer nodeID = new
				 * Integer(index); nodePos = new NodePosition(xTo,yTo);
				 * nodeParameter node = new
				 * nodeParameter(("node"+nodeID.toString()),xTo,
				 * yTo,"priority"); index++; nodes.addNodes(node);
				 * map.put((Object)keyTo,(Object)nodePos); }
				 */

				/* Modify this to add support for more types of roads */
				
				String temp_NodeFrom = "";
				String temp_NodeTo = "";
				
				for (int check = 0; check < node_counter; check++) {
					if (nodeX_.get(check).compareTo(xFrom) == 0 && 
							nodeY_.get(check).compareTo(yFrom) == 0) {
						temp_NodeFrom = node_.get(check);						
					} else if (nodeX_.get(check).compareTo(xTo) == 0 && 
							nodeY_.get(check).compareTo(yTo) == 0) {
						temp_NodeTo = node_.get(check);						
					}  
				}
				
				if (typeID.compareTo("A15") == 0) {
					edgeParameter newEdge = new edgeParameter(edgeID, temp_NodeFrom, temp_NodeTo);

					newEdge.setNolanes("6");
					newEdge.setSpeed("70");
					newEdge.setPriority("75");

					edges.addEdges(newEdge);
				}
				if (typeID.compareTo("A25") == 0) {
					edgeParameter newEdge = new edgeParameter(edgeID, temp_NodeFrom, temp_NodeTo);

					newEdge.setNolanes("4");
					newEdge.setSpeed("90");
					newEdge.setPriority("80");

					edges.addEdges(newEdge);
				}
				if (typeID.compareTo("A41") == 0) {
					edgeParameter newEdge = new edgeParameter(edgeID, temp_NodeFrom, temp_NodeTo);

					newEdge.setNolanes("2");
					newEdge.setSpeed("15");
					newEdge.setPriority("70");

					edges.addEdges(newEdge);
				}
				if (typeID.compareTo("A45") == 0) {
					edgeParameter newEdge = new edgeParameter(edgeID, temp_NodeFrom, temp_NodeTo);

					newEdge.setNolanes("2");
					newEdge.setSpeed("20");
					newEdge.setPriority("70");

					edges.addEdges(newEdge);
				}
				if (typeID.compareTo("A63") == 0) {
					edgeParameter newEdge = new edgeParameter(edgeID, temp_NodeFrom, temp_NodeTo);

					newEdge.setNolanes("2");
					newEdge.setSpeed("30");
					newEdge.setPriority("75");

					edges.addEdges(newEdge);
				}
				if (typeID.compareTo("A64") == 0) {
					edgeParameter newEdge = new edgeParameter(edgeID, temp_NodeFrom, temp_NodeTo);

					newEdge.setNolanes("2");
					newEdge.setSpeed("20");
					newEdge.setPriority("70");

					edges.addEdges(newEdge);
				}
				if (typeID.compareTo("A74") == 0) {
					edgeParameter newEdge = new edgeParameter(edgeID, temp_NodeFrom, temp_NodeTo);

					newEdge.setNolanes("1");
					newEdge.setSpeed("10");
					newEdge.setPriority("50");

					edges.addEdges(newEdge);
				}
				line = getContent.readLine();
			}
		} catch (Exception e) {
			System.out.println("Error when read line!");
		}

		construct buildNode = new construct(nodes, outputname + ".nod.xml");
		construct buildEdge = new construct(edges, outputname + ".edg.xml");
	}

}
