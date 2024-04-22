package com.move.mapConvertPart;
/*
 * ----------------------------------------------------------------------------
 * Copyright 2006-2008,
 * Ada Lezama Lugo, Michal Piorkowski, Maxim Raya, EPFL-I&C-LCA
 * All rights reserved
 * ----------------------------------------------------------------------------
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * ----------------------------------------------------------------------------
 */

/**
 * MapUtilities.java
 * Created on November 23, 2007, 7:05 AM
 * 
 * Class used to manipulate and retrieve information from the road network file
 * 
 * @version 1.0
 * @author Ada Lezama<ada.lezamalugo@epfl.ch>
 */

import java.util.*;
import java.io.*;
import org.jdom.*;

public class MapUtilities {
	public final static String LANESDB_FILENAME = "./files/lanes_db.txt";

	/** Creates a new instance of MapCrop */
	public MapUtilities() {
	}
	/**
	 * Generate a plain text file containing the lanes information in the network file
	 * Used as network information for NS2 agents
	 *
	 * @param networkFile
	 *            the network file containing the lanes information
	 */
	public static void generateLanesDB(String networkFile) {
		try {

			File file = new File(LANESDB_FILENAME);
			File mkdir = new File("./files");

			if (file.exists()) {
				try {
					file.createNewFile();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
			else {
				try {
					mkdir.mkdir();
					file.createNewFile();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}

			BufferedWriter writer = new BufferedWriter(new FileWriter(file));

			/** This method constructs the JDom tree from the XML input file */
			org.jdom.input.SAXBuilder builder = new org.jdom.input.SAXBuilder();
			org.jdom.Document inputDocument = builder.build(new File(networkFile));

			org.jdom.xpath.XPath path = org.jdom.xpath.XPath.newInstance("//lane");
			List list = path.selectNodes(inputDocument);
			Iterator i = list.iterator();

			while (i.hasNext()) {
				Element edge = (Element) i.next();

				String laneId = edge.getAttributeValue("id");
				String maxSpeed = edge.getAttributeValue("maxspeed");
				String laneLength = edge.getAttributeValue("length");
				String neighbors = "";

				// Find neighbors
				org.jdom.xpath.XPath succPath = org.jdom.xpath.XPath.newInstance("//succ");
				List succList = succPath.selectNodes(inputDocument);
				Iterator succIt = succList.iterator();
				while (succIt.hasNext()) {
					Element succ = (Element) succIt.next();
					if (succ.getAttributeValue("lane").equals(laneId)) {
						List neighborsList = succ.getChildren("succlane");
						Iterator j = neighborsList.iterator();
						while (j.hasNext()) {
							Element n = (Element) j.next();
							neighbors += n.getAttributeValue("lane") + "/";
						}
					}

				}

				//if( Double.parseDouble(maxSpeed) > 0.1){ // Do not consider pedestrian roads in the lanes DB
				writer.write(laneId + "," + maxSpeed + "," + laneLength + "," + neighbors);
				writer.newLine();
				//}

			}

			writer.flush();
			writer.close();

		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
