package com.move.mapConvertPart;
import javax.swing.*;

import com.move.JaveFileFilter;
import com.move.construct;

import java.awt.event.*;
import java.awt.*;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

/**
 * @author Chien-Ming Chou
 */
public class mapConvert extends JFrame {

	private String filename = "untitled";
//	private String filename = "C:/Users/Jensen/Desktop/Japan.kml";
//	private String filename = "C:/Users/Jensen/Desktop/Tainan.kml";
//	private String filename = "C:/Users/Jensen/Desktop/map3.osm";

	private int osenv;
	static JFileChooser chooser;
	private JTextField textFieldTigerInput;
	private JTextField textFieldTigerOutput;
	private JTextArea textArea;

	private static int modeOption = 1;
	private final ButtonGroup buttonGroup = new ButtonGroup();
	private JTextField txtKmlinput1;
	private JTextField txtkmlinput2;
	private JTextField textFieldKMLoutput;
	private JTextField textFieldOSMinput;
	private JTextField textFieldOSMN;
	private JTextField textFieldOSMS;
	private JTextField textFieldOSMW;
	private JTextField textFieldOSME;
	private JTextField textFieldOSMoutput;

	List<List<Integer>> output_list_ID = new ArrayList<List<Integer>>();
	List<List<String>> output_list_node = new ArrayList<List<String>>(); // 0 = fromnode, 1 = tonode, 2 = cross node..

	public mapConvert(String sumoPath, int os) {
		setMinimumSize(new Dimension(600, 670));
		setTitle("Map converter");
		getContentPane().setLayout(null);

		JRadioButton rdbtnTigerMapdat = new JRadioButton("TIGER map");
		buttonGroup.add(rdbtnTigerMapdat);
		rdbtnTigerMapdat.setFont(new Font("PMingLiU", Font.BOLD, 12));
		rdbtnTigerMapdat.setBounds(36, 28, 91, 23);
		rdbtnTigerMapdat.setSelected(true);
		rdbtnTigerMapdat.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				TigerOptionActionPerformed(evt);
			}
		});
		getContentPane().add(rdbtnTigerMapdat);

		JRadioButton rdbtnKmlMap = new JRadioButton("KML map");
		buttonGroup.add(rdbtnKmlMap);
		rdbtnKmlMap.setFont(new Font("PMingLiU", Font.BOLD, 12));
		rdbtnKmlMap.setBounds(36, 113, 81, 23);
		rdbtnKmlMap.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				KmlOptionActionPerformed(evt);
			}
		});
		getContentPane().add(rdbtnKmlMap);

		JRadioButton rdbtnOsmMap = new JRadioButton("OSM map");
		buttonGroup.add(rdbtnOsmMap);
		rdbtnOsmMap.setFont(new Font("PMingLiU", Font.BOLD, 12));
		rdbtnOsmMap.setBounds(36, 227, 79, 23);
		rdbtnOsmMap.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				OsmOptionActionPerformed(evt);
			}
		});
		getContentPane().add(rdbtnOsmMap);

		JLabel lblPleaseSelectYour = new JLabel("Please select your source to generate map");
		lblPleaseSelectYour.setFont(new Font("PMingLiU", Font.BOLD, 13));
		lblPleaseSelectYour.setBounds(10, 10, 270, 18);
		lblPleaseSelectYour.setForeground(new Color(0, 102, 0));
		getContentPane().add(lblPleaseSelectYour);

		JLabel lblTigerFiledat = new JLabel("TIGER file (*.dat only)");
		lblTigerFiledat.setFont(new Font("PMingLiU", Font.PLAIN, 13));
		lblTigerFiledat.setBounds(84, 57, 114, 16);
		getContentPane().add(lblTigerFiledat);

		JLabel lblOutputFileName = new JLabel("Output file name");
		lblOutputFileName.setFont(new Font("PMingLiU", Font.PLAIN, 13));
		lblOutputFileName.setBounds(84, 86, 82, 16);
		getContentPane().add(lblOutputFileName);

		textFieldTigerInput = new JTextField();
		textFieldTigerInput.setBounds(223, 54, 200, 23);
		getContentPane().add(textFieldTigerInput);
		textFieldTigerInput.setColumns(10);

		textFieldTigerOutput = new JTextField();
		textFieldTigerOutput.setBounds(223, 83, 200, 23);
		getContentPane().add(textFieldTigerOutput);
		textFieldTigerOutput.setColumns(10);

		JButton buttonTigerInput = new JButton("...");
		buttonTigerInput.setBounds(430, 54, 41, 23);
		getContentPane().add(buttonTigerInput);
		buttonTigerInput.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				TigerInputActionPerformed(evt);
			}
		});

		JButton buttonTigerOutput = new JButton("...");
		buttonTigerOutput.setBounds(430, 83, 41, 23);
		buttonTigerOutput.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				TigerOutputActionPerformed(evt);
			}
		});

		getContentPane().add(buttonTigerOutput);

		JButton btnOk = new JButton("OK");
		btnOk.setBounds(28, 600, 87, 23);
		btnOk.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				OKButtonActionPerformed(evt, sumoPath, osenv);
			}
		});
		getContentPane().add(btnOk);

		JButton btnCancel = new JButton("Cancel");
		btnCancel.setBounds(125, 600, 87, 23);
		btnCancel.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				cancelButtonActionPerformed(evt);
			}
		});
		getContentPane().add(btnCancel);

		JScrollPane scrollPane = new JScrollPane();
		scrollPane.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_ALWAYS);
		scrollPane.setVerticalScrollBarPolicy(ScrollPaneConstants.VERTICAL_SCROLLBAR_ALWAYS);
		scrollPane.setBounds(28, 398, 522, 189);
		getContentPane().add(scrollPane);

		textArea = new JTextArea();
		scrollPane.setViewportView(textArea);
		textArea.setAlignmentY(Component.TOP_ALIGNMENT);
		textArea.setAlignmentX(Component.LEFT_ALIGNMENT);
		textArea.setDoubleBuffered(true);
		textArea.setLineWrap(true);

		JLabel lblKmlFile = new JLabel("KML file");
		lblKmlFile.setFont(new Font("PMingLiU", Font.PLAIN, 13));
		lblKmlFile.setBounds(84, 142, 46, 16);
		getContentPane().add(lblKmlFile);

		txtKmlinput1 = new JTextField();
		txtKmlinput1.setBounds(223, 140, 200, 21);
		getContentPane().add(txtKmlinput1);
		txtKmlinput1.setColumns(10);

		JButton buttonKMLinput1 = new JButton("...");
		buttonKMLinput1.setBounds(430, 139, 41, 23);
		getContentPane().add(buttonKMLinput1);
		buttonKMLinput1.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				KmlInputActionPerformed(evt);
			}
		});

		JLabel lblKMLtolerance = new JLabel("Tolerance");
		lblKMLtolerance.setFont(new Font("PMingLiU", Font.PLAIN, 13));
		lblKMLtolerance.setBounds(84, 171, 48, 16);
		getContentPane().add(lblKMLtolerance);

		txtkmlinput2 = new JTextField();
		txtkmlinput2.setText("10");
		txtkmlinput2.setBounds(223, 169, 200, 21);
		getContentPane().add(txtkmlinput2);
		txtkmlinput2.setColumns(10);

		JLabel lblOutputFileName_1 = new JLabel("Output file name");
		lblOutputFileName_1.setFont(new Font("PMingLiU", Font.PLAIN, 13));
		lblOutputFileName_1.setBounds(84, 200, 82, 16);
		getContentPane().add(lblOutputFileName_1);

		textFieldKMLoutput = new JTextField();
		textFieldKMLoutput.setBounds(223, 198, 200, 21);
		getContentPane().add(textFieldKMLoutput);
		textFieldKMLoutput.setColumns(10);

		JButton buttonKMLoutput = new JButton("...");
		buttonKMLoutput.setBounds(430, 197, 41, 23);
		getContentPane().add(buttonKMLoutput);
		buttonKMLoutput.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				KmlOutputActionPerformed(evt);
			}
		});

		JLabel lblOsmFile = new JLabel("OSM file");
		lblOsmFile.setFont(new Font("PMingLiU", Font.PLAIN, 13));
		lblOsmFile.setBounds(84, 256, 46, 16);
		getContentPane().add(lblOsmFile);

		textFieldOSMinput = new JTextField();
		textFieldOSMinput.setBounds(223, 254, 200, 21);
		getContentPane().add(textFieldOSMinput);
		textFieldOSMinput.setColumns(10);

		JButton buttonOSMinput = new JButton("...");
		buttonOSMinput.setBounds(430, 253, 41, 23);
		getContentPane().add(buttonOSMinput);
		buttonOSMinput.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				OsmInputActionPerformed(evt);
			}
		});

		JLabel lblCaptureArea = new JLabel("Capture area (latitude and longitude)");
		lblCaptureArea.setFont(new Font("PMingLiU", Font.PLAIN, 13));
		lblCaptureArea.setBounds(84, 282, 176, 16);
		getContentPane().add(lblCaptureArea);

		JLabel lblNorth = new JLabel("North:");
		lblNorth.setFont(new Font("PMingLiU", Font.PLAIN, 13));
		lblNorth.setBounds(163, 308, 31, 16);
		getContentPane().add(lblNorth);

		JLabel lblSouth = new JLabel("South:");
		lblSouth.setFont(new Font("PMingLiU", Font.PLAIN, 13));
		lblSouth.setBounds(163, 333, 31, 16);
		getContentPane().add(lblSouth);

		textFieldOSMN = new JTextField();
		textFieldOSMN.setBounds(204, 306, 96, 21);
		getContentPane().add(textFieldOSMN);
		textFieldOSMN.setColumns(10);
		textFieldOSMN.addKeyListener(new KeyAdapter() {
			public void keyTyped(KeyEvent e) {
				int keyChar = e.getKeyChar();
				if (keyChar >= KeyEvent.VK_0 && keyChar <= KeyEvent.VK_9 || keyChar <= KeyEvent.VK_PERIOD) {

				} else {
					e.consume();
				}
			}
		});

		textFieldOSMS = new JTextField();
		textFieldOSMS.setBounds(204, 331, 96, 21);
		getContentPane().add(textFieldOSMS);
		textFieldOSMS.setColumns(10);
		textFieldOSMS.addKeyListener(new KeyAdapter() {
			public void keyTyped(KeyEvent e) {
				int keyChar = e.getKeyChar();
				if (keyChar >= KeyEvent.VK_0 && keyChar <= KeyEvent.VK_9 || keyChar <= KeyEvent.VK_PERIOD) {

				} else {
					e.consume();
				}
			}
		});

		JLabel lblEast = new JLabel("East:");
		lblEast.setFont(new Font("PMingLiU", Font.PLAIN, 13));
		lblEast.setBounds(322, 334, 46, 15);
		getContentPane().add(lblEast);

		JLabel lblWest = new JLabel("West:");
		lblWest.setFont(new Font("PMingLiU", Font.PLAIN, 13));
		lblWest.setBounds(322, 309, 46, 15);
		getContentPane().add(lblWest);

		textFieldOSMW = new JTextField();
		textFieldOSMW.setBounds(363, 306, 96, 21);
		getContentPane().add(textFieldOSMW);
		textFieldOSMW.setColumns(10);
		textFieldOSMW.addKeyListener(new KeyAdapter() {
			public void keyTyped(KeyEvent e) {
				int keyChar = e.getKeyChar();
				if (keyChar >= KeyEvent.VK_0 && keyChar <= KeyEvent.VK_9 || keyChar <= KeyEvent.VK_PERIOD) {

				} else {
					e.consume();
				}
			}
		});

		textFieldOSME = new JTextField();
		textFieldOSME.setBounds(363, 331, 96, 21);
		getContentPane().add(textFieldOSME);
		textFieldOSME.setColumns(10);
		textFieldOSME.addKeyListener(new KeyAdapter() {
			public void keyTyped(KeyEvent e) {
				int keyChar = e.getKeyChar();
				if (keyChar >= KeyEvent.VK_0 && keyChar <= KeyEvent.VK_9 || keyChar <= KeyEvent.VK_PERIOD) {

				} else {
					e.consume();
				}
			}
		});

		JLabel lblOutputFileName_2 = new JLabel("Output file name");
		lblOutputFileName_2.setFont(new Font("PMingLiU", Font.PLAIN, 13));
		lblOutputFileName_2.setBounds(84, 362, 82, 16);
		getContentPane().add(lblOutputFileName_2);

		textFieldOSMoutput = new JTextField();
		textFieldOSMoutput.setBounds(223, 360, 200, 21);
		getContentPane().add(textFieldOSMoutput);
		textFieldOSMoutput.setColumns(10);

		JButton buttonOSMoutput = new JButton("...");
		buttonOSMoutput.setBounds(430, 359, 41, 23);
		getContentPane().add(buttonOSMoutput);
		buttonOSMoutput.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				OsmOutputActionPerformed(evt);
			}
		});

		osenv = os;
		initComponents(sumoPath, os);

		setVisible(true);
	}

	private void initComponents(String sumoPath, int os) {
		if (chooser == null) {
			chooser = new JFileChooser("Open File");
		}
//		filename = sumoPath;
	}

	private void TigerInputActionPerformed(ActionEvent evt) {
		String extension = ".dat";
		chooser.setFileFilter(new JaveFileFilter(extension));
		int returnVal = chooser.showDialog(this, "Create");
		if (returnVal == JFileChooser.APPROVE_OPTION)
			textFieldTigerInput.setText(chooser.getSelectedFile().getPath());
	}

	private void TigerOutputActionPerformed(ActionEvent evt) {
		String extension = "";
		chooser.setFileFilter(new JaveFileFilter(extension));
		int returnVal = chooser.showOpenDialog(this);
		if (returnVal == JFileChooser.APPROVE_OPTION)
			textFieldTigerOutput.setText(chooser.getSelectedFile().getPath());
	}

	private void KmlInputActionPerformed(ActionEvent evt) {
		String extension = ".kml";
		chooser.setFileFilter(new JaveFileFilter(extension));
		int returnVal = chooser.showDialog(this, "Create");
		if (returnVal == JFileChooser.APPROVE_OPTION)
			txtKmlinput1.setText(chooser.getSelectedFile().getPath());
	}

	private void KmlOutputActionPerformed(ActionEvent evt) {
		String extension = "";
		chooser.setFileFilter(new JaveFileFilter(extension));
		int returnVal = chooser.showOpenDialog(this);
		if (returnVal == JFileChooser.APPROVE_OPTION)
			textFieldKMLoutput.setText(chooser.getSelectedFile().getPath());
	}
	
	private void OsmInputActionPerformed(ActionEvent evt) {
		String extension = ".osm";
		chooser.setFileFilter(new JaveFileFilter(extension));
		int returnVal = chooser.showDialog(this, "Create");
		if (returnVal == JFileChooser.APPROVE_OPTION)
			textFieldOSMinput.setText(chooser.getSelectedFile().getPath());
	}

	private void OsmOutputActionPerformed(ActionEvent evt) {
		String extension = "";
		chooser.setFileFilter(new JaveFileFilter(extension));
		int returnVal = chooser.showOpenDialog(this);
		if (returnVal == JFileChooser.APPROVE_OPTION)
			textFieldOSMoutput.setText(chooser.getSelectedFile().getPath());
	}

	private double deg2rad(double deg) {
		return (deg * Math.PI / 180.0);
	}

	private double rad2deg(double rad) {
		return (rad * 180.0 / Math.PI);
	}

	private double distance(double lat1, double lng1, double lat2, double lng2) {
		double theta = lng1 - lng2;

		double dist = Math.sin(deg2rad(lat1)) * Math.sin(deg2rad(lat2)) + Math.cos(deg2rad(lat1))
				* Math.cos(deg2rad(lat2)) * Math.cos(deg2rad(theta));

		dist = Math.acos(dist);
		dist = rad2deg(dist);

		double miles = dist * 60 * 1.1515;
		return miles * 1.609344 * 1000;
	}

	private void OKButtonActionPerformed(ActionEvent evt, String sumoPath, int envos) {
		if (modeOption == 1) {
			tiger t = new tiger(textFieldTigerInput.getText(), textFieldTigerOutput.getText());
			t.convertData();
			String cmd;
			
//			String inputFile = filename;
			String inputFile = textFieldTigerInput.getText();
			File input = new File(inputFile);
			
//			System.out.println("11  " + textFieldTigerInput.getText());
//			System.out.println("11 sumoPath  " + sumoPath);
			if (input.exists() == true) {
				construct netcconstruction = new construct(textFieldTigerOutput.getText() + ".netc.cfg",
						textFieldTigerOutput.getText() + ".nod.xml",
						textFieldTigerOutput.getText() + ".edg.xml", "",
						textFieldTigerOutput.getText() + ".net.xml", 1, "Unknown", "2", "25", "75");
				if (envos == 0) {
					//cmd = "cmd /c " + sumoPath + "netconvert -v -c " + outputPath.getText() + ".netc.cfg";
					cmd = "\"" + sumoPath + System.getProperty("file.separator") + "netconvert.exe" + "\"";
					cmd += " -v -c ";
					cmd += "\"" + textFieldTigerOutput.getText() + ".netc.cfg" + "\"";
				}
				else {
					cmd = "netconvert -v -c " + textFieldTigerOutput.getText() + ".netc.cfg";
				}

//				System.out.println("cmd   " + cmd );
				
				try {
					Process process = Runtime.getRuntime().exec(cmd);

					new Thread(new SubTask(process.getInputStream())).start();
					new Thread(new SubTask(process.getErrorStream())).start();
					process.waitFor();

					java.io.InputStream in = process.getInputStream();
				} catch (Exception err) {
					err.printStackTrace();
				}
			} else {
				textArea.append("not found the input file " + input + "\n");
				textArea.setCaretPosition(textArea.getDocument().getLength());
			}			
		}
		else if (modeOption == 2) {
			kmlToSumo();
		}
		else if (modeOption == 3) {
			osmToSumo();
		} else {
			textArea.append("impossible case, a bug\n");
			textArea.setCaretPosition(textArea.getDocument().getLength());
		}
//		this.dispose();
	}

	public void kmlToSumo() {
		List<String> node_list_ID = new ArrayList<String>();
		List<Double> node_list_X = new ArrayList<Double>();
		List<Double> node_list_Y = new ArrayList<Double>();

		int relCounter = 0;
		List<Integer> rel_ID = new ArrayList<Integer>();
		List<String> rel_fromnode = new ArrayList<String>();
		List<String> rel_tonode = new ArrayList<String>();

		List<String> ori = new ArrayList<String>();
		List<String> del = new ArrayList<String>();

		List<Double> offset_x = new ArrayList<Double>();
		List<Double> offset_y = new ArrayList<Double>();

		List<String> edge_content = new ArrayList<String>();
		List<String> node_content = new ArrayList<String>();

		String inputFile = txtKmlinput1.getText();
//		String inputFile = filename;

		File input = new File(inputFile);

		if (input.exists() == true) {
			String lineStr = null;
			BufferedReader getContent;
			try {
				textArea.append("KML file reading: " + input + "\n");
				textArea.setCaretPosition(textArea.getDocument().getLength());

				int lineCounter = 0;
				getContent = new BufferedReader(new FileReader(input), 65535);
				lineStr = getContent.readLine();
				while (lineStr != null) {
//					System.out.println(" > " + lineStr);

					String[] strArray, strArray2;
					strArray = lineStr.trim().split(" ");

					if (strArray.length > 0) {
						String target = "<coordinates>";
						String targetLine = strArray[0];
						if (targetLine.compareTo(target) == 0) {
							lineStr = getContent.readLine().trim();
							strArray = lineStr.trim().split(" ");
							int relLen = 0;
							for (int index = 0; index < strArray.length; index++) {

								strArray2 = strArray[index].trim().split(",");
//								System.out.println(" = " + strArray2[0] + " " + strArray2[1]);
								String ID = String.valueOf(lineCounter) + "n" + String.valueOf(index);
								node_list_ID.add(ID);
								node_list_X.add(Double.parseDouble(strArray2[0]));
								node_list_Y.add(Double.parseDouble(strArray2[1]));

//								$node_list["$id"] = array($x, $y);

								relLen = index;
							}

							for (int index = 0; index < relLen; index++) {
								String fromnode_ = String.valueOf(lineCounter) + "n" + String.valueOf(index);
								String tonode_ = String.valueOf(lineCounter) + "n" + String.valueOf(index + 1);
								rel_ID.add(relCounter);
								rel_fromnode.add(fromnode_);
								rel_tonode.add(tonode_);

								// add row:
								output_list_ID.add(new ArrayList<Integer>());
								output_list_node.add(new ArrayList<String>());
								// add a column:
								output_list_ID.get(output_list_ID.size() - 1).add(relCounter);
								output_list_node.get(output_list_node.size() - 1).add(fromnode_);
								output_list_node.get(output_list_node.size() - 1).add(tonode_);
								relCounter++;
							}
							lineCounter++;
						}
					}

					lineStr = getContent.readLine();
				}

				textArea.append("Finish" + "\n");
				textArea.setCaretPosition(textArea.getDocument().getLength());

//				dumpKMLOutputList();

//				for (int index = 0; index < node_list_ID.size(); index ++ ) {
//					System.out.println(node_list_ID.get(index) + " " + node_list_X.get(index) 
//							+ " " + node_list_Y.get(index));
//					// php Array ( [0n0] => Array ( [0] => 139.4154835783964 [1] => 36.26605491214913 )
//					// java 1n0 139.4154835783964 36.26605491214913
//				}

//				System.out.println(" lineCounter = " + lineCounter);

				for (int index = 0; index < rel_ID.size(); index++) {
//					System.out.println(rel_ID.get(index) + " fromEdge = " + rel_fromnode.get(index)
//							+ " toEdge = " + rel_tonode.get(index));
					// php Array ( [0] => Array ( [0] => 0n0 [1] => 0n1 ) 
					//             [1] => Array ( [0] => 0n1 [1] => 0n2 ) )
					//             [16] => Array ( [0] => 3n3 [1] => 3n4 ) 
					//             [17] => Array ( [0] => 3n4 [1] => 3n5 ) )

					// $rel_list[$rel_count] = array($fromnode, $tonode);
					// $rel_list[0] == rel_ID, $rel_list[0][0] = rel_ID.from, $rel_list[0][1] = rel_ID.to 
					// java			0 0n0 0n1   1 0n1 0n2    
					//              16 3n3 3n4     		17 3n4 3n5
				}

				int crossCount = 0;

				textArea.append("Parsing..." + "\n");
				textArea.setCaretPosition(textArea.getDocument().getLength());

				// r
				for (int index = 0; index < rel_ID.size(); index++) {
					//$rel_list[0] == rel_ID, $rel_list[0] = rel_ID.from, $rel_list[1] = rel_ID.to 
					String toID = rel_tonode.get(index);
					double toX = 0;
					double toY = 0;
					String _fromID = rel_fromnode.get(index);
					double _fromX = 0;
					double _fromY = 0;

					// $node_list[$rel_list[$r][0]][0] = _fromX  -> x1
					// $node_list[$rel_list[$r][0]][1] = _fromY  -> y1

					// $node_list[$rel_list[$r][1]][0] = toX     -> x2
					// $node_list[$rel_list[$r][1]][1] = toY     -> y2

					for (int found = 0; found < node_list_ID.size(); found++) {
//						System.out.println("   ^^^ toID > " + toID + "  node_list_ID.get(found) " + node_list_ID.get(found));
						if (node_list_ID.get(found).compareTo(toID) == 0) {
//							System.out.println("! dest > " + node_list_ID.get(found) + " node_x = "
//									+ node_list_X.get(found) + " node_y = " + node_list_Y.get(found));
							toX = node_list_X.get(found);
							toY = node_list_Y.get(found);
						}

						if (node_list_ID.get(found).compareTo(_fromID) == 0) {
//							System.out.println("! dest > " + node_list_ID.get(found) + " node_x = "
//									+ node_list_X.get(found) + " node_y = " + node_list_Y.get(found));
							_fromX = node_list_X.get(found);
							_fromY = node_list_Y.get(found);
						}

					}

//					System.out.println("______ toID > " + toID + " toX = "
//							+ toX + " toY = " + toY);

					// i
					for (int index2 = 0; index2 < rel_ID.size(); index2++) {
						String fromID = rel_fromnode.get(index2);
						double fromX = 0;
						double fromY = 0;
						String _toID = rel_tonode.get(index2);
						double _toX = 0;
						double _toY = 0;

						for (int found = 0; found < node_list_ID.size(); found++) {
//							System.out.println(" fromID = " + fromID + "   sourID > " + node_list_ID.get(found) + " node_x = " 
//									+ node_list_X.get(found) + " node_y = " + node_list_Y.get(found));
							if (node_list_ID.get(found).compareTo(fromID) == 0) {
//								System.out.println("!! sourID > " + node_list_ID.get(found) + " node_x = "
//										+ node_list_X.get(found) + " node_y = " + node_list_Y.get(found));
								fromX = node_list_X.get(found);
								fromY = node_list_Y.get(found);
							}

							if (node_list_ID.get(found).compareTo(_toID) == 0) {
//								System.out.println("!! sourID > " + node_list_ID.get(found) + " node_x = "
//										+ node_list_X.get(found) + " node_y = " + node_list_Y.get(found));
								_toX = node_list_X.get(found);
								_toY = node_list_Y.get(found);
							}
						}

						if (toX == fromX) {
//							System.out.println(toID + " toX = " + toX + " fromID = " + fromID + " fromX = " + fromX);
							continue;
						}

						// php Array ( [0n0] => Array ( [0] => 139.4154835783964 [1] => 36.26605491214913 )
						// $rel_list[$rel_count] = array($fromnode, $tonode);
						// $rel_list[0] == rel_ID, $rel_list[0][0] = rel_ID.from, $rel_list[0][1] = rel_ID.to 

						// $node_list[$rel_list[$r][0]][0] = _fromX  -> x1
						// $node_list[$rel_list[$r][0]][1] = _fromY  -> y1

						// $node_list[$rel_list[$r][1]][0] = toX     -> x2
						// $node_list[$rel_list[$r][1]][1] = toY     -> y2

						// $node_list[$rel_list[$i][0]][0] = fromX   -> x3
						// $node_list[$rel_list[$i][0]][1] = fromY   -> y3

						// $node_list[$rel_list[$i][1]][0] = _toX    -> x4
						// $node_list[$rel_list[$i][1]][1] = _toY    -> y4

//						$a = @check(
//							$node_list[$rel_list[$r][0]][0], $node_list[$rel_list[$r][0]][1], 
//							$node_list[$rel_list[$r][1]][0], $node_list[$rel_list[$r][1]][1],
//							$node_list[$rel_list[$i][0]][0], $node_list[$rel_list[$i][0]][1],
//							$node_list[$rel_list[$i][1]][0], $node_list[$rel_list[$i][1]][1]
//							);

//						System.out.println(" x1 = " + _fromX);
//						System.out.println(" x2 = " + toX);
//						System.out.println(" x3 = " + fromX);
//						System.out.println(" x4 = " + _toX);

//						System.out.println(" y1 = " + _fromY);
//						System.out.println(" y2 = " + toY);
//						System.out.println(" y3 = " + fromY);
//						System.out.println(" y4 = " + _toY);

						double a1 = _fromY - toY;
						double a2 = fromY - _toY;
						double b1 = toX - _fromX;
						double b2 = _toX - fromX;
						double c1 = toX * _fromY - _fromX * toY;
						double c2 = _toX * fromY - fromX * _toY;
						double d = a1 * b2 - a2 * b1;

//						System.out.println(" d = " + d);

						double itsX = 0;
						double itsY = 0;
						if (d != 0) {
							double d1 = (c1 * b2) - (c2 * b1);
							double d2 = (a1 * c2) - (a2 * c1);

//							System.out.println(" d1 = " + d1 + " d2 = " + d2);

							itsX = d1 / d;
							itsY = d2 / d;

//							System.out.println(" x1 = " + _fromX);
//							System.out.println(" x2 = " + toX);
//							System.out.println(" x3 = " + fromX);
//							System.out.println(" x4 = " + _toX);

//							System.out.println(" y1 = " + _fromY);
//							System.out.println(" y2 = " + toY);
//							System.out.println(" y3 = " + fromY);
//							System.out.println(" y4 = " + _toY);

							if (Math.min(_fromX, toX) < itsX && itsX < Math.max(_fromX, toX)
									&& Math.min(_fromY, toY) < itsY && itsY < Math.max(_fromY, toY)
									&& Math.min(fromX, _toX) < itsX && itsX < Math.max(fromX, _toX)
									&& Math.min(fromY, _toY) < itsY && itsY < Math.max(fromY, _toY))
								;
							else {
								itsX = 0;
								itsY = 0;
							}
						} else {
							itsX = 0;
							itsY = 0;
						}

//						System.out.println(" itsX = " + itsX + " itsY = " + itsY);

						if (itsX != 0) {
//							System.out.println(" itsX = " + itsX + " itsY = " + itsY);
							String crossID = "c" + crossCount;

							boolean findSame = false;
							for (int found = 0; found < node_list_ID.size(); found++) {
//								System.out.println("!! node_list_ID > " + node_list_ID.get(found) + " node_x = "
//										+ node_list_X.get(found) + " node_y = " + node_list_Y.get(found));

								// already existing, just skip
								if (itsX == node_list_X.get(found)) {
									findSame = true;
//									System.out.println("!!!! ");
									crossID = node_list_ID.get(found);
//									fromX = node_list_X.get(found);
//									fromY = node_list_Y.get(found);
								}
							}

							if (findSame == false) {
								node_list_ID.add(crossID);
								node_list_X.add(itsX);
								node_list_Y.add(itsY);
							}

							// calculate distance 
							double distBegin = distance(_fromX, _fromY, itsX, itsY);
							double distEnd = distance(toX, toY, itsX, itsY);

							if (distBegin == distEnd)
								continue;

							List<String> temp_cross_ID = new ArrayList<String>();
							List<Double> temp_cross_X = new ArrayList<Double>();
							List<Double> temp_cross_Y = new ArrayList<Double>();

							double tolerance = Double.parseDouble(txtkmlinput2.getText());
							if (distBegin > tolerance && distEnd > tolerance) {

								// index = r, index2 = i in php

//								output_list_ID.get( output_list_ID.size() -1 ).add( relCounter );
								output_list_node.get(index).add(crossID);
							} else if (distBegin > tolerance && distEnd < tolerance) {
								int locIndex = 0;
								String targetStr = rel_tonode.get(index);
//								System.out.println(" targetStr -> " + targetStr );

								for (int findIndex = 0; findIndex < output_list_node.get(index).size(); findIndex++) {
									if (output_list_node.get(index).get(findIndex).compareTo(targetStr) == 0) {
										locIndex = findIndex;
									}
								}

//								System.out.println(" locIndex -> " + locIndex );
								ori.add(output_list_node.get(index).get(locIndex));
								del.add(crossID);

//								System.out.println(" ori add -> " + output_list_node.get(index).get(locIndex) );
//								System.out.println(" del add -> " + crossID );
							} else if (distBegin < tolerance && distEnd > tolerance) {
								int locIndex = 0;
								String targetStr = rel_fromnode.get(index);
//								System.out.println(" targetStr -> " + targetStr );

								for (int findIndex = 0; findIndex < output_list_node.get(index).size(); findIndex++) {
									if (output_list_node.get(index).get(findIndex).compareTo(targetStr) == 0) {
										locIndex = findIndex;
									}
								}

//								System.out.println(" locIndex -> " + locIndex );
								ori.add(output_list_node.get(index).get(locIndex));
								del.add(crossID);

//								System.out.println(" ori add -> " + output_list_node.get(index).get(locIndex) );
//								System.out.println(" del add -> " + crossID );
							} else {
								int locIndex = 0;
								String targetStr = rel_fromnode.get(index);
//								System.out.println("1 targetStr -> " + targetStr );

								for (int findIndex = 0; findIndex < output_list_node.get(index).size(); findIndex++) {
									if (output_list_node.get(index).get(findIndex).compareTo(targetStr) == 0) {
										locIndex = findIndex;
									}
								}

//								System.out.println(" locIndex -> " + locIndex );
								ori.add(output_list_node.get(index).get(locIndex));
								del.add(crossID);

//								System.out.println("1 ori add -> " + output_list_node.get(index).get(locIndex) );
//								System.out.println("1 del add -> " + crossID );

								targetStr = rel_fromnode.get(index);
//								System.out.println("2 targetStr -> " + targetStr );

								for (int findIndex = 0; findIndex < output_list_node.get(index).size(); findIndex++) {
									if (output_list_node.get(index).get(findIndex).compareTo(targetStr) == 0) {
										locIndex = findIndex;
									}
								}

//								System.out.println(" locIndex -> " + locIndex );
								ori.add(output_list_node.get(index).get(locIndex));
								del.add(crossID);

//								System.out.println("2 ori add -> " + output_list_node.get(index).get(locIndex) );
//								System.out.println("2 del add -> " + crossID );
							}

							crossCount++;
						}
					}
				} // end of r loop

//				dumpKMLOutputList();

				textArea.append("Done, KML file generating" + "\n");
				textArea.setCaretPosition(textArea.getDocument().getLength());

				//輸出 XML 格式，整理數據後即進行字串輸出
				// format 
				// output_list_ID = 17
				// output_list_node = 3n4
				for (int r = 0; r < output_list_ID.size(); r++) {
					int locIndex1 = -1;
					int locIndex2 = -1;
					String targetStr1 = output_list_node.get(r).get(0);
					String targetStr2 = output_list_node.get(r).get(1);

					for (int findIndex = 0; findIndex < ori.size(); findIndex++) {
						if (targetStr1.compareTo(ori.get(findIndex)) == 0) {
							locIndex1 = findIndex;
							String replaceStr = del.get(locIndex1);
//							System.out.println("replaceStr = " + replaceStr);
							output_list_node.get(r).set(0, replaceStr);
						}
						if (targetStr2.compareTo(ori.get(findIndex)) == 0) {
							locIndex2 = findIndex;
							String replaceStr = del.get(locIndex2);
//							System.out.println("replaceStr = " + replaceStr);
							output_list_node.get(r).set(1, replaceStr);
						}
					}

//					if ( locIndex1 != -1) {
//						System.out.println("targetStr1 = " + targetStr1);
//						System.out.println("locIndex1 = " + locIndex1);
//					}
//					if ( locIndex2 != -1) {
//						System.out.println("targetStr2 = " + targetStr2);
//						System.out.println("locIndex2 = " + locIndex2);
//					}						

				}

				//把重複的資料移除					
				for (int r = 0; r < output_list_ID.size(); r++) {
					int locIndex1 = -1;
					int locIndex2 = -1;
					String targetStr1 = output_list_node.get(r).get(0);
					String targetStr2 = output_list_node.get(r).get(1);

					if (targetStr1.compareTo(targetStr2) == 0) {
						output_list_node.get(r).remove(1);
//						System.out.println("remove case  targetStr1  = " + targetStr1 + "  r = " + r);
					}

				}

//				dumpKMLOutputList();

				for (int r = 0; r < output_list_ID.size(); r++) {

					int cont = output_list_node.get(r).size();

					// 原本這邊有sort 想說對功能不影響 就拿掉了
//					for (int findIndex = 0; findIndex < cont; findIndex ++) {
//						String targetStr = output_list_node.get(r).get(findIndex);
////						System.out.println("targetStr = " + targetStr);
//						
//						for (int nodeIndex = 0; nodeIndex < node_list_ID.size(); nodeIndex++) {
//							if( targetStr.compareTo( node_list_ID.get(nodeIndex))  == 0 ) {
//								System.out.println("x = " + node_list_X.get(nodeIndex));
//								node_list_X.get(nodeIndex);
//							}
//						}
//						
//					}

					int idNum = 0;
					for (int y = 0; y < cont; y++) {
						int Yloc = 0;
						if (cont == 1)
							break;
						if (y == cont - 1)
							continue;
						else
							Yloc = y + 1;

						String inputStr = "<edge id=\"" + r + "_" + idNum + "-1\" fromnode=\"" + output_list_node.get(r).get(y)
								+ "\" tonode=\"" + output_list_node.get(r).get(Yloc)
								+ "\" priority=\"75\" nolanes=\"2\" speed=\"17\" />";

						String inputStr2 = "<edge id=\"" + r + "_" + idNum + "-2\" fromnode=\"" + output_list_node.get(r).get(Yloc)
								+ "\" tonode=\"" + output_list_node.get(r).get(y)
								+ "\" priority=\"75\" nolanes=\"2\" speed=\"17\" />";

						edge_content.add(inputStr);
						edge_content.add(inputStr2);
//						System.out.println(inputStr);
//						System.out.println(inputStr2);

						idNum++;
					}

				}

				/* 
				 * 將經緯度轉換為直角坐標系 
				 * return  轉換後坐標
				 */

				//convertLLtoTM  ,Universal Transverse Mercator (UTM)
				// http://www.ibm.com/developerworks/library/j-coordconvert/
				CoordinateConversion convert = new CoordinateConversion();
//				Latitudes range from -90 to 90 (2D Y axis). Longitudes range from -180 to 180. (2D X axis)

				double minX = 999999999;
				double minY = 999999999;

				for (int found = 0; found < node_list_ID.size(); found++) {
					double longitude = node_list_X.get(found);
					double latitude = node_list_Y.get(found);
//					System.out.println(" longitude = " + longitude);
//					System.out.println(" latitude = " + latitude);					
					String UTM = convert.latLon2UTM(latitude, longitude);
//					System.out.println(" UTM = " + UTM);
					String[] strArray;

					strArray = UTM.split(" ");

//					System.out.println(" strArray[2] " + strArray[2]);
//					System.out.println(" strArray[3] " + strArray[3]);						

					if (minX > Double.parseDouble(strArray[2]))
						minX = Double.parseDouble(strArray[2]);
					if (minY > Double.parseDouble(strArray[3]))
						minY = Double.parseDouble(strArray[3]);

					offset_x.add(Double.parseDouble(strArray[2]));
					offset_y.add(Double.parseDouble(strArray[3]));
				}
//				System.out.println("offset_x: " + minX + "  offset_y: " + minY);

				textArea.append("offset_x: " + minX + "  offset_y: " + minY + "\n");
				textArea.setCaretPosition(textArea.getDocument().getLength());

				for (int found = 0; found < node_list_ID.size(); found++) {

					String nodeID = node_list_ID.get(found);
					double locX = offset_x.get(found) - minX;
					double locY = offset_y.get(found) - minY;
					String inputStr;
					if (nodeID.substring(0, 1).compareToIgnoreCase("c") == 0) {
						inputStr = "<node id=\"" + nodeID + "\" x=\"" + locX + "\" y=\"" + locY + "\" type=\"traffic_light\" />";
					} else {
						inputStr = "<node id=\"" + nodeID + "\" x=\"" + locX + "\" y=\"" + locY + "\" type=\"priority\" />";
					}

//					System.out.println("inputStr = " + inputStr);
					node_content.add(inputStr);
				}

				PrintWriter printer;
//				System.out.println("Files writing to path: " + textFieldKMLoutput.getText());

				textArea.append("Files writing to path: " + textFieldKMLoutput.getText() + "\n");
				textArea.setCaretPosition(textArea.getDocument().getLength());

//				textFieldKMLoutput.setText("C:\\Users\\Jensen\\Desktop\\testKML");		
				try {
					if (textFieldKMLoutput.getText().length() == 0) {
//						System.out.println("Please check your output path" + "\n");

						textArea.append("Please check your output path" + "\n");
						textArea.setCaretPosition(textArea.getDocument().getLength());
					} else {
						printer = new PrintWriter(new BufferedWriter(
								new FileWriter(textFieldKMLoutput.getText() + ".nod.xml"), 32768));

						printer.println("<nodes>");
						for (int i = 0; i < node_content.size(); i++) {
							printer.println(node_content.get(i));
						}
						printer.println("</nodes>\n<!-- offset_x: " + minX + ", offset_y:" + minY + " -->");
						printer.flush();
						printer.close();

						printer = new PrintWriter(new BufferedWriter(
								new FileWriter(textFieldKMLoutput.getText() + ".edg.xml"), 32768));

						printer.println("<edges>");
						for (int i = 0; i < edge_content.size(); i++) {
							printer.println(edge_content.get(i));
						}
						printer.println("</edges>");
						printer.flush();
						printer.close();

						textArea.append("Map convert success" + "\n");
						textArea.setCaretPosition(textArea.getDocument().getLength());
					}

				} catch (IOException e) {
					System.out.println("print error!");
				}

			} catch (FileNotFoundException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

		} else {
			textArea.append("not found the input file " + input + "\n");
			textArea.setCaretPosition(textArea.getDocument().getLength());
		}
	}

	public void osmToSumo() {
		double maxlat = 0;
		double maxlon = 0;
		double minlat = 0;
		double minlon = 0;
		boolean loadDefault = false;

//		if ( textFieldOSMN.getText() ) {
//			
//		}
		
		String input_maxlat = textFieldOSMN.getText();
		String input_minlat = textFieldOSMS.getText();
		String input_minlon = textFieldOSMW.getText();
		String input_maxlon = textFieldOSME.getText();

		if (input_maxlat.length() == 0 || input_minlat.length() == 0 ||
				input_minlon.length() == 0 || input_maxlon.length() == 0) {
			System.out.println("lat or lon no value, directly capture whole osm map");
			loadDefault = true;
		} else {
			System.out.println("directly capture osm map by ");
			maxlat = Double.parseDouble(input_maxlat);
			maxlon = Double.parseDouble(input_maxlon);
			minlat = Double.parseDouble(input_minlat);
			minlon = Double.parseDouble(input_minlon);
		}

		List<String> node_list_ID = new ArrayList<String>();
		List<Double> node_list_X = new ArrayList<Double>();
		List<Double> node_list_Y = new ArrayList<Double>();
		List<Boolean> node_list_inBound = new ArrayList<Boolean>();

		List<String> way_list_ID = new ArrayList<String>();
		List<String> way_ref = new ArrayList<String>();
		List<List<String>> way_list_ref = new ArrayList<List<String>>();
		List<String> way_list_type = new ArrayList<String>();

		List<String> numWay_node_ID = new ArrayList<String>();
		List<String> numWay_node_wayID = new ArrayList<String>();
		List<Double> numWay_node_X = new ArrayList<Double>();
		List<Double> numWay_node_Y = new ArrayList<Double>();

		List<String> edge_content = new ArrayList<String>();
		List<String> node_content = new ArrayList<String>();

		List<String> intersectNodeList = new ArrayList<String>();
		List<Integer> intersectNodeValue = new ArrayList<Integer>();
		
		String inputFile = textFieldOSMinput.getText();
//		String inputFile = filename;

		File input = new File(inputFile);

		if (input.exists() == true) {
			String lineStr = null;
			BufferedReader getContent;
			try {
				textArea.append("OSM file reading: " + input + "\n");
				textArea.setCaretPosition(textArea.getDocument().getLength());

				int lineCounter = 0;
				getContent = new BufferedReader(new FileReader(input), 65535);
				lineStr = getContent.readLine();
				while (lineStr != null) {
					boolean skipReadlineFlag = false;
//					System.out.println(" > " + lineStr);

					String[] strArray, strArray2, strArray3, strArray4;
					strArray = lineStr.trim().split("\"");

					if (strArray.length > 0) {
//						System.out.println(strArray[0]);

						String targetLine = strArray[0];
//						System.out.println(targetLine);

						if (loadDefault == true && targetLine.compareTo("<bounds minlat=") == 0) {
							strArray = lineStr.trim().split("\"");
							System.out.println(strArray[0]);	// <bounds minlat=
							System.out.println(strArray[1]);	// 23.0908000
							System.out.println(strArray[2]);	// minlon=
							System.out.println(strArray[3]);	// 120.2543000
							System.out.println(strArray[4]);	// maxlat=	
							System.out.println(strArray[5]);	// 23.1027000
							System.out.println(strArray[6]);	// maxlon=
							System.out.println(strArray[7]);	// 120.2692000

							maxlat = Double.parseDouble(strArray[5]);	// maxlat  north  23.1027000
							maxlon = Double.parseDouble(strArray[7]);   // maxlon  east   120.2692000
							minlat = Double.parseDouble(strArray[1]); 	// minlat  south  23.0908000 
							minlon = Double.parseDouble(strArray[3]);   // minlon  west   120.2543000
//							System.in.read();
						}

						if (targetLine.compareTo("<node id=") == 0) {
							strArray = lineStr.trim().split(" ");
							int relLen = 0;

//							System.out.println(strArray[0]);	// node ID
//							System.out.println(strArray[1]);
//							System.out.println(strArray[2]);
//							System.out.println(strArray[3]);
//							System.out.println(strArray[8]);	// lat
//							System.out.println(strArray[9]);	// lon

							String ID = null;
							String lat = null;
							String lon = null;
							if (strArray[0].compareTo("<node") == 0) {
//								System.out.println("hit node id");
								strArray2 = strArray[1].trim().split("\"");
//								System.out.println("strArray2[1] = " + strArray2[1]);
								ID = strArray2[1];
							}

							strArray3 = strArray[8].trim().split("\"");
							if (strArray3[0].compareTo("lat=") == 0) {
//								System.out.println("hit lat=");								
//								System.out.println("strArray3[1] = " + strArray3[1]);
								lat = strArray3[1];
							}

							strArray4 = strArray[9].trim().split("\"");
							if (strArray4[0].compareTo("lon=") == 0) {
//								System.out.println("hit lon=");								
//								System.out.println("strArray4[1] = " + strArray4[1]);
								lon = strArray4[1];
							}
							node_list_ID.add(ID);
							node_list_X.add(Double.parseDouble(lon));
							node_list_Y.add(Double.parseDouble(lat));
							node_list_inBound.add(false);

//							System.in.read();
						}

						else if (targetLine.compareTo("<way id=") == 0) {
							strArray = lineStr.trim().split(" ");

//							System.out.println(strArray[0]);	// <way
//							System.out.println(strArray[1]);	// id="218556460"

							String wayID = null;
							if (strArray[0].compareTo("<way") == 0) {
//								System.out.println("hit way id");
								strArray2 = strArray[1].trim().split("\"");
//								System.out.println("strArray2[1] = " + strArray2[1]);
								wayID = strArray2[1];

								lineStr = getContent.readLine();
								strArray2 = lineStr.trim().split(" ");
//								System.out.println("strArray2[0] = " + strArray2[0]);
//								System.out.println("strArray2[1] = " + strArray2[1]);
								if (strArray2[0].compareTo("<nd") == 0) {

									strArray3 = strArray2[1].trim().split("\"");
//									System.out.println("strArray3[0] = " + strArray3[0]);
//									System.out.println("strArray3[1] = " + strArray3[1]);
									way_ref = new ArrayList<String>();
									way_ref.add(strArray3[1]);
									way_list_ref.add(way_ref);
//									System.out.println("way_list_ref.size = " + way_list_ref.size());

									lineStr = getContent.readLine();
									boolean findTypeFlag = false;
									while (lineStr.compareTo("</way>") != 0 && skipReadlineFlag == false) {
//										System.out.println("lineStr = " + lineStr);

										if (lineStr.compareTo(" </way>") == 0
												|| lineStr.compareTo("</osm>") == 0) {
											skipReadlineFlag = true;
										} else {
											strArray2 = lineStr.trim().split(" ");
//											System.out.println("strArray2[0] = " + strArray2[0]);
//											System.out.println("strArray2[1] = " + strArray2[1]);
											if (strArray2[0].compareTo("<nd") == 0) {
												strArray3 = strArray2[1].trim().split("\"");
//												System.out.println("strArray3[1] = " + strArray3[1]);
												way_list_ref.get(way_list_ref.size() - 1).add(strArray3[1]);
											}

											if (strArray2[0].compareTo("<tag") == 0 &&
													strArray2[1].compareTo("k=\"highway\"") == 0) {
//												System.out.println(" get highway " );
												way_list_type.add("highway");
												findTypeFlag = true;
											}

											lineStr = getContent.readLine();
										}
									}

									way_list_ID.add(wayID);
									if (findTypeFlag == false) {
										way_list_type.add("normal");
									}
								}
							}

//							System.in.read();

						}

					}

					if (skipReadlineFlag == true)
						;
					else
						lineStr = getContent.readLine();
				} // end of while 

//				System.out.println("node_list_ID.size() = " + node_list_ID.size());
//				System.out.println("way_list_ID.size() = " + way_list_ID.size());
//				System.out.println("way_list_type.size() = " + way_list_type.size());
//				System.out.println("way_list_ref.size() = " + way_list_ref.size());

				for (int dump = 0; dump < way_list_ref.size(); dump++) {
//					System.out.println(dump + " way_list_ID.get(dump) = " + way_list_ID.get(dump));
//					System.out.println(dump + " way_list_ref.size() = " + way_list_ref.get(dump).size());
//					System.out.println(dump + " way_list_type.get(dump) = " + way_list_type.get(dump));

					if (way_list_type.get(dump).compareTo("highway") == 0) {
						for (int dumpIndex = 0; dumpIndex < (way_list_ref.get(dump).size() - 1); dumpIndex++) {
//							System.out.println(dumpIndex + "  = " + way_list_ref.get(dump).get(dumpIndex));

							// 一次抓兩點 這樣才有辦法建置一道路，如果只有一個點沒辦法
							String targetStr1 = way_list_ref.get(dump).get(dumpIndex);
							String targetStr2 = way_list_ref.get(dump).get(dumpIndex + 1);

//							System.out.println(dumpIndex + " targetStr1 = " + targetStr1);
//							System.out.println(dumpIndex + " targetStr2 = " + targetStr2);

							boolean inBound = false;
							double lat = 0;
							double lon = 0;
							for (int find = 0; find < node_list_ID.size(); find++) {

								if (targetStr1.compareTo(node_list_ID.get(find)) == 0) {
									lat = node_list_Y.get(find);
									lon = node_list_X.get(find);
//									System.out.println(find + " targetStr = " + targetStr1 + " lat = " + lat + " lon = " + lon);
//									targetStr = 1241909271 lat = 23.1013716 lon = 120.2602301
//									System.in.read();

//									maxlat = Double.parseDouble(strArray[5]);	// maxlat  north  23.1027000
//									maxlon = Double.parseDouble(strArray[7]);   // maxlon  east   120.2692000
//									minlat = Double.parseDouble(strArray[1]); 	// minlat  south  23.0908000 
//									minlon = Double.parseDouble(strArray[3]);   // minlon  west   120.2543000

//									((float)$temp_compare[sprintf("%d",$id1)][1] > (float)$south) && 
//									((float)$temp_compare[sprintf("%d",$id1)][1] < (float)$north) &&
//									((float)$temp_compare[sprintf("%d",$id1)][0] > (float)$west) &&
//									((float)$temp_compare[sprintf("%d",$id1)][0] < (float)$east)

//									$temp_compare[sprintf("%d",$id1)][1] = 23.1013921, = lat south and north 
//									$temp_compare[sprintf("%d",$id1)][0] = 120.2605222, = lon  west and east 

									// if targetStr1 inbound, targetStr2 also put into  
									if ((lon > minlon && lon < maxlon) &&
											(lat > minlat && lat < maxlat)) {
										inBound = true;
										node_list_inBound.set(find, true);
//										System.out.println(" inBound ");
										boolean findDuplicated = false;

										for (int findIndex = 0; findIndex < numWay_node_ID.size(); findIndex++) {
											if (targetStr1.compareTo(numWay_node_ID.get(findIndex)) == 0 &&
													way_list_ID.get(dump).compareTo(numWay_node_wayID.get(findIndex)) == 0) {
												findDuplicated = true;
											}
										}

										if (findDuplicated == false) {
											numWay_node_ID.add(targetStr1);
											numWay_node_wayID.add(way_list_ID.get(dump));
											numWay_node_X.add(lon);
											numWay_node_Y.add(lat);

//											System.out.println(dump + " wayID = " + way_list_ID.get(dump) + " targetStr1 = "
//													+ targetStr1 + " lat = " + lat + " lon = " + lon);
										}

									}
								}
							}

							for (int find = 0; find < node_list_ID.size(); find++) {
								//不檢查是否在bound中
								if (inBound == true && targetStr2.compareTo(node_list_ID.get(find)) == 0) {
									lat = node_list_Y.get(find);
									lon = node_list_X.get(find);
									node_list_inBound.set(find, true);
//									System.out.println(find + " targetStr = " + targetStr + " lat = " + lat + " lon = " + lon);

//									if ((lon > minlon && lon < maxlon) &&
//											(lat > minlat && lat < maxlat)) {

									boolean findDuplicated = false;

									for (int findIndex = 0; findIndex < numWay_node_ID.size(); findIndex++) {
										if (targetStr2.compareTo(numWay_node_ID.get(findIndex)) == 0 &&
												way_list_ID.get(dump).compareTo(numWay_node_wayID.get(findIndex)) == 0) {
											findDuplicated = true;
//											System.out.println(find + " findDuplicated targetStr2 = " + targetStr2 + " lat = " + lat
//													+ " lon = " + lon);
										}
									}

									if (findDuplicated == false) {
										numWay_node_ID.add(targetStr2);
										numWay_node_wayID.add(way_list_ID.get(dump));
										numWay_node_X.add(lon);
										numWay_node_Y.add(lat);

//										System.out.println(dump + " wayID = " + way_list_ID.get(dump) + " targetStr2 = " + targetStr2
//												+ " lat = " + lat + " lon = " + lon);
									}

//									}
								}
							}
						}
					}
//					System.in.read();
				}

//				for (int find = 0; find < numWay_node_ID.size(); find++) {
//					System.out.println(find + " numWay_node_wayID = " + numWay_node_wayID.get(find) + " numWay_node_ID = "
//							+ numWay_node_ID.get(find));
//				}
//				System.in.read();

				// merge road by tolerance	
				/* 
				 * 以單一線段上單一點來，與其他線段上各點進行比對
				 * 若兩點之距離小於預設容忍值，則進行合併，
				 * 單一線段上各點則忽略不進行計算，藉以維持線段形狀
				 */
				String currentWayID = null;
				for (int find = 0; find < numWay_node_ID.size(); find++) {

					currentWayID = numWay_node_wayID.get(find);
//					System.out.println(" numWay_node_wayID = " + numWay_node_wayID.get(find) + " numWay_node_ID = "
//							+ numWay_node_ID.get(find));
//					System.out.println(" numWay_node_X = " + numWay_node_X.get(find) + " numWay_node_Y = " + numWay_node_Y.get(find));

					for (int dump = 0; dump < numWay_node_ID.size(); dump++) {

//						System.out.println(" -> numWay_node_wayID = " + numWay_node_wayID.get(dump) + " numWay_node_ID = "
//								+ numWay_node_ID.get(dump));
//						System.out.println(" -> numWay_node_X = " + numWay_node_X.get(dump) + " numWay_node_Y = "
//								+ numWay_node_Y.get(dump));

						if (currentWayID.compareTo(numWay_node_wayID.get(dump)) != 0) {
//							System.out.println(" -> numWay_node_wayID = " + numWay_node_wayID.get(dump) + " numWay_node_ID = "
//									+ numWay_node_ID.get(dump));
//							System.out.println(" -> numWay_node_X = " + numWay_node_X.get(dump) + " numWay_node_Y = "
//									+ numWay_node_Y.get(dump));

//							numWay_node_X.add(lon);
//							numWay_node_Y.add(lat);

							double nodeDist = distance(numWay_node_Y.get(find), numWay_node_X.get(find),
									numWay_node_Y.get(dump), numWay_node_X.get(dump));

							if (nodeDist < 75) {
//								System.out.println(" !!! ");
//								numWay_node_wayID.set(dump, numWay_node_wayID.get(find));	// 這欄更新後 資料會亂掉
//								System.out.println("before numWay_node_ID.get(dump) " + numWay_node_ID.get(dump));

								numWay_node_ID.set(dump, numWay_node_ID.get(find));
								numWay_node_Y.set(dump, numWay_node_Y.get(find));
								numWay_node_X.set(dump, numWay_node_X.get(find));

//								System.out.println("numWay_node_wayID.get(dump) " + numWay_node_wayID.get(dump));

//								System.out.println("after numWay_node_ID.get(dump) " + numWay_node_ID.get(dump));

//								System.out.println("numWay_node_X.get(dump) " + numWay_node_X.get(dump));
//								System.out.println("numWay_node_Y.get(dump) " + numWay_node_Y.get(dump));
							}

						}
					}
//					System.in.read();
				}

				// 將暫存陣列中各點、線段寫入預定寫入檔案中內容
				int tempCounter = 0;				
				List<String> checkDupData = new ArrayList<String>();
				for (int find = 0; find < numWay_node_ID.size() - 1; find++) {

					if (numWay_node_wayID.get(find).compareTo(numWay_node_wayID.get(find + 1)) == 0) {
//						System.out.println(" numWay_node_wayID = " + numWay_node_wayID.get(find) + " numWay_node_ID = "
//								+ numWay_node_ID.get(find));
//						System.out.println("     numWay_node_wayID + 1  = " + numWay_node_wayID.get(find + 1) + " numWay_node_ID = "
//								+ numWay_node_ID.get(find + 1));

						if (numWay_node_ID.get(find).compareTo(numWay_node_ID.get(find + 1)) == 0) {
							continue;
						}

						String tempStr = numWay_node_ID.get(find) + "-" + numWay_node_ID.get(find + 1);
						String tempStr2 = numWay_node_ID.get(find + 1) + "-" + numWay_node_ID.get(find);

						boolean foundFlag = false;
						for (int index = 0; index < checkDupData.size(); index++) {
							if (checkDupData.get(index).compareTo(tempStr) == 0 ||
									checkDupData.get(index).compareTo(tempStr2) == 0) {
								foundFlag = true;
								break;
							}
						}

						if (foundFlag == false) {
							checkDupData.add(tempStr);
//							checkDupData.add(tempStr2);		
						}

//						System.out.println("     		tempStr  = " + tempStr);
//						System.out.println("    		tempStr2  = " + tempStr2);
						String tempEdge = "<edge id=\"" + tempCounter + "w" + find + "-1\" fromnode=\"" + numWay_node_ID.get(find)
								+ "\" tonode=\"" + numWay_node_ID.get(find + 1) + "\" priority=\"75\" nolanes=\"2\" speed=\"40\" />";
						String tempEdge2 = "<edge id=\"" + tempCounter + "w" + find + "-2\" fromnode=\""
								+ numWay_node_ID.get(find + 1) + "\" tonode=\"" + numWay_node_ID.get(find)
								+ "\" priority=\"75\" nolanes=\"2\" speed=\"40\" />";
//						System.out.println(tempEdge);
//						System.out.println(tempEdge2);
						edge_content.add(tempEdge);
						edge_content.add(tempEdge2);

						// a counter judge intersection or no for a node
						foundFlag = false;
						String targetStr = numWay_node_ID.get(find);
						int foundLoc = 0;
						for (int index = 0; index < intersectNodeList.size(); index++) {
							if (intersectNodeList.get(index).compareTo(targetStr) == 0) {
								foundFlag = true;
								foundLoc = index;
								break;
							}
						}

						if (foundFlag == false) {
							intersectNodeList.add(targetStr);
							intersectNodeValue.add(2);
						} else {
							intersectNodeValue.set(foundLoc, intersectNodeValue.get(foundLoc) + 2);
						}

						foundFlag = false;
						targetStr = numWay_node_ID.get(find + 1);
						foundLoc = 0;
						for (int index = 0; index < intersectNodeList.size(); index++) {
							if (intersectNodeList.get(index).compareTo(targetStr) == 0) {
								foundFlag = true;
								foundLoc = index;
								break;
							}
						}

						if (foundFlag == false) {
							intersectNodeList.add(targetStr);
							intersectNodeValue.add(2);
						} else {
							intersectNodeValue.set(foundLoc, intersectNodeValue.get(foundLoc) + 2);
						}

					} else {
						tempCounter++;
//						System.out.println(" ");
					}

//					System.in.read();
				}

				CoordinateConversion convert = new CoordinateConversion();
				double minX = 999999999;
				double minY = 999999999;
				
				// 統一進行坐標轉換
				for (int find = 0; find < node_list_ID.size(); find++) {
//					System.out.println("     		node_list_ID  = " + node_list_ID.get(find) + "   " + node_list_inBound.get(find));

					for (int index = 0; index < intersectNodeList.size(); index++) {
						if (node_list_ID.get(find).compareTo(intersectNodeList.get(index)) == 0 && node_list_inBound.get(find) == true) {
//							System.out.println("     		intersectNodeList  = " + intersectNodeList.get(index) + "  value = " + intersectNodeValue.get(index));
							
							double longitude = node_list_X.get(find);
							double latitude = node_list_Y.get(find);
//							System.out.println(" longitude = " + longitude);
//							System.out.println(" latitude = " + latitude);					
							String UTM = convert.latLon2UTM(latitude, longitude);
//							System.out.println(" UTM = " + UTM);
							String[] strArray;

							strArray = UTM.split(" ");

//							System.out.println(" strArray[2] " + strArray[2]);
//							System.out.println(" strArray[3] " + strArray[3]);						

							if (minX > Double.parseDouble(strArray[2]))
								minX = Double.parseDouble(strArray[2]);
							if (minY > Double.parseDouble(strArray[3]))
								minY = Double.parseDouble(strArray[3]);
							
							String inputStr = null;
							
							if ( intersectNodeValue.get(index) > 4 ) {
								inputStr = "<node id=\"" + node_list_ID.get(find) + "\" x=\"" + strArray[2] + "\" y=\"" + strArray[3] + "\" type=\"traffic_light\" />";
							} else {
								inputStr = "<node id=\"" + node_list_ID.get(find) + "\" x=\"" + strArray[2] + "\" y=\"" + strArray[3] + "\" type=\"priority\" />";
							}

//							System.out.println(inputStr);
							node_content.add(inputStr);
						}
					}

				}

//				textArea.append("minX: " + minX + "  minY: " + minY + "\n");
//				textArea.setCaretPosition(textArea.getDocument().getLength());
				
				textArea.append("Finish" + "\n");
				textArea.setCaretPosition(textArea.getDocument().getLength());

				PrintWriter printer;
//				System.out.println("Files writing to path: " + textFieldKMLoutput.getText());

				textArea.append("Files writing to path: " + textFieldOSMoutput.getText() + "\n");
				textArea.setCaretPosition(textArea.getDocument().getLength());

//				textFieldOSMinput.setText("C:\\Users\\Jensen\\Desktop\\testOSM");		
				
				try {
					if (textFieldOSMoutput.getText().length() == 0) {
//						System.out.println("Please check your output path" + "\n");

						textArea.append("Please check your output path" + "\n");
						textArea.setCaretPosition(textArea.getDocument().getLength());
					} else {
						printer = new PrintWriter(new BufferedWriter(
								new FileWriter(textFieldOSMoutput.getText() + ".nod.xml"), 32768));

						printer.println("<nodes>");
						for (int i = 0; i < node_content.size(); i++) {
							printer.println(node_content.get(i));
						}
						printer.println("</nodes>\n<!-- offset_x: " + minX + ", offset_y:" + minY + " -->");
						printer.flush();
						printer.close();

						printer = new PrintWriter(new BufferedWriter(
								new FileWriter(textFieldOSMoutput.getText() + ".edg.xml"), 32768));

						printer.println("<edges>");
						for (int i = 0; i < edge_content.size(); i++) {
							printer.println(edge_content.get(i));
						}
						printer.println("</edges>");
						printer.flush();
						printer.close();

						textArea.append("Map convert success" + "\n");
						textArea.setCaretPosition(textArea.getDocument().getLength());
					}

				} catch (IOException e) {
					System.out.println("print error!");
				}
				
				// 額外加入圖徵主體選取，由不同點作為起始點，記錄與其相連節點最多者視為 Mainbody
				// 找一起點, 開始放資料進去, 然後開始比對有無conneciton的資料, 如果有, 繼續丟進去
				
				List<String> tempData = new ArrayList<String>();
				List<List<String>> storeAllTempData = new ArrayList<List<String>>();
				int largeValue = -1;
				int largeLoc = -1;
				System.out.println("edge_content.size() = " + edge_content.size());
				for (int find = 0; find < edge_content.size(); find++) {
					
					tempData = new ArrayList<String>();
					
//					System.out.println(edge_content.get(find));		
					String[] strArray, strArray2, strArray3;
					strArray = edge_content.get(find).trim().split("\"");
					tempData.add(edge_content.get(find));
					
//					System.out.println("1 edge_content = " + edge_content.get(find));
					
//					<edge id="28w215-1" fromnode="1848255446" tonode="1848255447" priority="75" nolanes="2" speed="40" />
//					System.out.println(strArray[0]);	// <edge id=
//					System.out.println(strArray[1]);	// 28w215-1
//					System.out.println(strArray[2]);	// fromnode=
//					System.out.println(strArray[3]);	// 1848255446
//					System.out.println(strArray[4]);	// tonode=
//					System.out.println(strArray[5]);	// 1848255447
//					System.out.println(strArray[6]);	// priority=				
					
					for (int find2 = 0; find2 < edge_content.size(); find2++) {
//						System.out.println("2 edge_content = " + edge_content.get(find2));
						strArray2 = edge_content.get(find2).trim().split("\"");
						
						if (find == find2)
							continue;
						
						boolean hit = false;
						for (int find3 = 0; find3 < tempData.size(); find3++) {
							strArray3 = tempData.get(find3).trim().split("\"");
//							System.out.println(tempData.size() + "   3 edge_content = " + tempData.get(find3));
							if (strArray2[3].compareTo(strArray3[3]) == 0 || 
									strArray2[3].compareTo(strArray3[5]) == 0 ||
									strArray2[5].compareTo(strArray3[3]) == 0 ||
									strArray2[5].compareTo(strArray3[5]) == 0 ) {
								hit = true;
								break;
							}								
						}					
						
						if ( hit == true ) {
							boolean sameData = false;
							String targetStr = edge_content.get(find2);
							for (int find3 = 0; find3 < tempData.size(); find3++) {								
								if (targetStr.compareTo(tempData.get(find3)) == 0 ) {
									sameData = true;
									break;
								}								
							}
							
							if ( sameData == false) {
								tempData.add(targetStr);
//								System.out.println("   add " + targetStr);
							}
						}
						
//						System.in.read();	
					}
					
//					for (int find2 = 0; find2 < tempData.size(); find2++) {
//						System.out.println(find2 + "  " + tempData.get(find2));	
//					}
					
//					System.out.println(tempData.size());	
					
					storeAllTempData.add(tempData);
					
					if ( tempData.size() > largeValue) {
						largeValue = tempData.size();
						largeLoc = storeAllTempData.size()-1;
					}
					
//					for (int find2 = 0; find2 < storeAllTempData.size(); find2++) {
//						System.out.println(find2 + "  " + storeAllTempData.get(find2).size());
//						for (int find3 = 0; find3 < storeAllTempData.get(find2).size(); find3++) {
//							System.out.println(find3 + "       " + storeAllTempData.get(find2).get(find3));
//						}
//					}
					
//					System.in.read();	
				}
				
				System.out.println(" largeValue = " + largeValue + ", largeLoc = " + largeLoc);	
				
				// mainbody print
//				for (int find2 = 0; find2 < storeAllTempData.get(largeLoc).size(); find2++) {
//					System.out.println(find2 + "  " + storeAllTempData.get(largeLoc).get(find2));
//				}
				
				try {
					if (textFieldOSMoutput.getText().length() == 0) {
//						System.out.println("Please check your output path" + "\n");

						textArea.append("Please check your output path" + "\n");
						textArea.setCaretPosition(textArea.getDocument().getLength());
					} else {
						printer = new PrintWriter(new BufferedWriter(
								new FileWriter(textFieldOSMoutput.getText() + "_mainBody_" + ".nod.xml"), 32768));

						printer.println("<nodes>");
						for (int i = 0; i < node_content.size(); i++) {
							printer.println(node_content.get(i));
						}
						printer.println("</nodes>\n<!-- offset_x: " + minX + ", offset_y:" + minY + " -->");
						printer.flush();
						printer.close();

						printer = new PrintWriter(new BufferedWriter(
								new FileWriter(textFieldOSMoutput.getText() + "_mainBody_" + ".edg.xml"), 32768));

						printer.println("<edges>");
						for (int i = 0; i < storeAllTempData.get(largeLoc).size(); i++) {
							printer.println(storeAllTempData.get(largeLoc).get(i));
						}
						printer.println("</edges>");
						printer.flush();
						printer.close();

						textArea.append("Map convert success (only pick out mainbody part)" + "\n");
						textArea.setCaretPosition(textArea.getDocument().getLength());
					}

				} catch (IOException e) {
					System.out.println("print error!");
				}
				
			} catch (Exception err) {
				err.printStackTrace();
			}
		}
	}
	void dumpKMLOutputList() {
		for (int check = 0; check < output_list_ID.size(); check++) {
			for (int check2 = 0; check2 < output_list_ID.get(check).size(); check2++) {
				System.out.println(" output_list_ID = " + output_list_ID.get(check).get(check2));
			}

			for (int check2 = 0; check2 < output_list_node.get(check).size(); check2++) {
				System.out.println("   output_list_node = " + output_list_node.get(check).get(check2));
			}
		}
	}

	class SubTask implements Runnable {
		public SubTask(InputStream istream)
		{
			if (istream == null)
				iReader = new BufferedReader(new InputStreamReader(System.in));
			else
				iReader = new BufferedReader(new InputStreamReader(istream));
		}

		public void run()
		{
			try {
				String input = iReader.readLine();
				while (input != null)
				{
//		        System.out.println(input);
					textArea.append(input + "\n");
					textArea.setCaretPosition(textArea.getDocument().getLength());
					input = iReader.readLine();
				}
			} catch (IOException ioe) {
			}
		}

		private BufferedReader iReader;
	}

	private void cancelButtonActionPerformed(ActionEvent evt) {
		this.dispose();
	}

	private void TigerOptionActionPerformed(ActionEvent evt) {
		modeOption = 1;
	}

	private void KmlOptionActionPerformed(ActionEvent evt) {
		modeOption = 2;
	}

	private void OsmOptionActionPerformed(ActionEvent evt) {
		modeOption = 3;
	}

	public static void main(String args[]) {
		new mapConvert("_", 1);		// test only
	}
}
