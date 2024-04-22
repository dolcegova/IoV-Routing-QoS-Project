package com.move;
import javax.swing.*;
import java.awt.event.*;
import java.awt.*;
import java.io.*;

public class randomNetGen extends JFrame {
	private String filename = "untitled";
	private int osenv;
	static JFileChooser chooser;

	private int randoption = 0;
	private int gridjunctionsoption = 0;
	private int gridjunctionXoption = 0;
	private int gridjunctionYoption = 0;

	private int gridlengthoption = 0;
	private int gridlengthXoption = 0;
	private int gridlengthYoption = 0;

	private int spideraxesoption = 0;
	private int spidercirclesoption = 0;
	private int spiderdistanceoption = 0;

	private JButton OKButton;
	private ButtonGroup buttonGroup1;
	private JButton cancelButton;
	private JMenu helpMenu;
	private JLabel jLabel1;
	private JLabel jLabel9;
	private JMenuBar jMenuBar;
	private JPanel jPanel1;
	private JButton openOutput;
	private JMenuItem quickHelpMenu;

	private JRadioButton gridOption;
	private JRadioButton randomOption;
	private JRadioButton spiderOption;

	private JTextField outputPath;

	private JTextField gridLength;
	private JTextField gridLengthX;
	private JTextField gridLengthY;
	private JTextField gridJunctionX;
	private JTextField gridJunctionY;
	private JTextField gridJunctions;
	private JTextField spiderAxes;
	private JTextField spiderCircles;
	private JTextField spiderDistances;

	private JCheckBox gridJunctionsCheck;
	private JCheckBox gridRoadsCheck;
	private JCheckBox xJunctionCheck;
	private JCheckBox xLengthCheck;
	private JCheckBox yJunctionCheck;
	private JCheckBox yLengthCheck;

	private JCheckBox spiderAxesCheck;
	private JCheckBox spiderCirclesCheck;
	private JCheckBox spiderDistanceCheck;

	private JTextArea statusText;


	public randomNetGen(String sumoPath, int os) {
		osenv = os;
		initComponents(sumoPath, os);
	}

	private void initComponents(String sumoPath, int os) {
		GridBagConstraints gridBagConstraints;

		buttonGroup1 = new ButtonGroup();
		jPanel1 = new JPanel();
		jLabel1 = new JLabel();
		gridOption = new JRadioButton();
		spiderOption = new JRadioButton();
		randomOption = new JRadioButton();
		gridJunctions = new JTextField();
		gridLength = new JTextField();
		xJunctionCheck = new JCheckBox();
		gridJunctionX = new JTextField();
		gridJunctionY = new JTextField();
		gridLengthY = new JTextField();
		gridLengthX = new JTextField();
		jLabel9 = new JLabel();
		outputPath = new JTextField();
		spiderAxes = new JTextField();
		spiderCircles = new JTextField();
		spiderDistances = new JTextField();
		OKButton = new JButton();
		cancelButton = new JButton();
		openOutput = new JButton();
		yJunctionCheck = new JCheckBox();
		xLengthCheck = new JCheckBox();
		yLengthCheck = new JCheckBox();
		gridJunctionsCheck = new JCheckBox();
		gridRoadsCheck = new JCheckBox();
		spiderAxesCheck = new JCheckBox();
		spiderCirclesCheck = new JCheckBox();
		spiderDistanceCheck = new JCheckBox();
		statusText = new JTextArea();
		jMenuBar = new JMenuBar();
		helpMenu = new JMenu();
		quickHelpMenu = new JMenuItem();

		if(chooser == null) {
			chooser = new JFileChooser("Open File");
		}

		jPanel1.setLayout(new GridBagLayout());
		setTitle("Random Map Generator");

		jLabel1.setText("Random Map Generator");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.insets = new Insets(10, 10, 0, 236);
		jPanel1.add(jLabel1, gridBagConstraints);

		gridOption.setText("Grid Layout");
		buttonGroup1.add(gridOption);
		gridOption.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				gridOptionActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.insets = new Insets(6, 10, 0, 281);
		jPanel1.add(gridOption, gridBagConstraints);

		spiderOption.setText("Spider Layout");
		buttonGroup1.add(spiderOption);
		spiderOption.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				spiderOptionActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.insets = new Insets(6, 10, 0, 267);
		jPanel1.add(spiderOption, gridBagConstraints);

		randomOption.setText("Totally Random Layout");
		buttonGroup1.add(randomOption);
		randomOption.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				randomOptionActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.insets = new Insets(14, 10, 0, 217);
		jPanel1.add(randomOption, gridBagConstraints);

		gridJunctions.setText("9");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = 29;
		gridBagConstraints.insets = new Insets(6, -130, 4, 0);
		jPanel1.add(gridJunctions, gridBagConstraints);

		gridLength.setText("100");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.ipadx = 19;
		gridBagConstraints.insets = new Insets(6, 50, 4, 0);
		jPanel1.add(gridLength, gridBagConstraints);

		xJunctionCheck.setText("Overrides number of junctions in x direction");
		xJunctionCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				xJunctionCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.insets = new Insets(6, 60, 0, 47);
		jPanel1.add(xJunctionCheck, gridBagConstraints);

		gridJunctionX.setText("1");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 29;
		gridBagConstraints.insets = new Insets(6, -130, 4, 0);
		jPanel1.add(gridJunctionX, gridBagConstraints);

		gridJunctionY.setText("1");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.ipadx = 29;
		gridBagConstraints.insets = new Insets(6, -130, 4, 0);
		jPanel1.add(gridJunctionY, gridBagConstraints);

		gridLengthY.setText("100");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 8;
		gridBagConstraints.ipadx = 19;
		gridBagConstraints.insets = new Insets(6, 50, 4, 0);
		jPanel1.add(gridLengthY, gridBagConstraints);

		gridLengthX.setText("100");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.ipadx = 19;
		gridBagConstraints.insets = new Insets(6, 50, 4, 0);
		jPanel1.add(gridLengthX, gridBagConstraints);

		jLabel9.setText("Set Output File");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 13;
		gridBagConstraints.insets = new Insets(36, 20, 4, 278);
		jPanel1.add(jLabel9, gridBagConstraints);

		outputPath.setText(".net.xml");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 13;
		gridBagConstraints.ipadx = 114;
		gridBagConstraints.insets = new Insets(36, 120, 0, 100);
		jPanel1.add(outputPath, gridBagConstraints);

		spiderAxes.setText("1");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.ipadx = 39;
		gridBagConstraints.insets = new Insets(6, 300, 4, 50);
		jPanel1.add(spiderAxes, gridBagConstraints);

		spiderCircles.setText("1");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.ipadx = 39;
		gridBagConstraints.insets = new Insets(6, 300, 4, 50);
		jPanel1.add(spiderCircles, gridBagConstraints);

		spiderDistances.setText("100");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 12;
		gridBagConstraints.ipadx = 29;
		gridBagConstraints.insets = new Insets(6, -190, 4, 50);
		jPanel1.add(spiderDistances, gridBagConstraints);

		filename = sumoPath;
		osenv = os;

		OKButton.setText("OK");
		//OKButton.setPreferredSize(new Dimension(50,23));
		OKButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				OKButtonActionPerformed(evt,filename,osenv);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 15;
		gridBagConstraints.ipadx = 19;
		gridBagConstraints.insets = new Insets(10, 20, 12, 290);
		jPanel1.add(OKButton, gridBagConstraints);

		cancelButton.setText("Cancel");
		//cancelButton.setPreferredSize(new Dimension(50,23));
		cancelButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				cancelButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 15;
		gridBagConstraints.insets = new Insets(10, 100, 12, 207);
		jPanel1.add(cancelButton, gridBagConstraints);

		openOutput.setText("...");
		//openOutput.setPreferredSize(new Dimension(45,23));
		openOutput.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openOutputActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 13;
		gridBagConstraints.ipadx = -13;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(36, 280, 0, 70);
		jPanel1.add(openOutput, gridBagConstraints);

		yJunctionCheck.setText("Overrides number of junctions in y direction");
		yJunctionCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				yJunctionCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.insets = new Insets(6, 60, 0, 48);
		jPanel1.add(yJunctionCheck, gridBagConstraints);

		xLengthCheck.setText("Overrides length of horizontal roads");
		xLengthCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				xLengthCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.insets = new Insets(6, 60, 0, 92);
		jPanel1.add(xLengthCheck, gridBagConstraints);

		yLengthCheck.setText("Overrides length of vertical roads");
		yLengthCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				yLengthCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 8;
		gridBagConstraints.insets = new Insets(6, 60, 0, 106);
		jPanel1.add(yLengthCheck, gridBagConstraints);

		gridJunctionsCheck.setText("Set number of junctions in both x-y directions");
		gridJunctionsCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				gridJunctionsCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.insets = new Insets(6, 30, 0, 69);
		jPanel1.add(gridJunctionsCheck, gridBagConstraints);

		gridRoadsCheck.setText("Set length of roads in both horizontal and vertical directions");
		gridRoadsCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				gridRoadsCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(6, 30, 0, 50);
		jPanel1.add(gridRoadsCheck, gridBagConstraints);

		spiderAxesCheck.setText("Set number of axes within the net");
		spiderAxesCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				spiderAxesCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.insets = new Insets(6, 30, 0, 135);
		jPanel1.add(spiderAxesCheck, gridBagConstraints);

		spiderCirclesCheck.setText("Set number of circles within the net");
		spiderCirclesCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				spiderCirclesCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.insets = new Insets(6, 30, 0, 124);
		jPanel1.add(spiderCirclesCheck, gridBagConstraints);

		spiderDistanceCheck.setText("Set the distances between the circles");
		spiderDistanceCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				spiderDistanceCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 12;
		gridBagConstraints.insets = new Insets(6, 30, 0, 111);
		jPanel1.add(spiderDistanceCheck, gridBagConstraints);

		statusText.setEditable(false);
		JScrollPane scrollPane = new JScrollPane(statusText,
		JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
		JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 14;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 440;
		gridBagConstraints.ipady = 84;
		gridBagConstraints.insets = new Insets(10, 20, 0, 20);
		jPanel1.add(scrollPane, gridBagConstraints);

		getContentPane().add(jPanel1, BorderLayout.CENTER);

		helpMenu.setText("Help");
		quickHelpMenu.setText("Quick Help");
		quickHelpMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				quickHelpMenuActionPerformed(evt);
			}
		});

		helpMenu.add(quickHelpMenu);
/*Quick help is disabled for now*/
		//jMenuBar.add(helpMenu);

		setJMenuBar(jMenuBar);

		pack();
		setVisible(true);
	}

	private void OKButtonActionPerformed(ActionEvent evt, String sumoPath, int envos) {

		String line;
		String cmd;
		String options = "";

		if (randoption == 0){
			options = " --grid-net ";
			if (gridjunctionsoption == 1){
				options+= " --grid-number " + gridJunctions.getText() + " ";
				if (gridjunctionXoption == 1){
					options+= " --grid-x-number " + gridJunctionX.getText() + " ";
				}
				else if (gridjunctionYoption == 1){
					options+= " --grid-y-number " + gridJunctionY.getText() + " ";
				}
			}
			if (gridlengthoption == 1){
				options+= " --grid-length " + gridLength.getText() + " ";
				if (gridlengthXoption == 1){
					options+= " --grid-x-length " + gridLengthX.getText() + " ";
				}
				else if (gridlengthYoption == 1){
					options+= " --grid-y-length " + gridLengthY.getText() + " ";
				}
			}
		}
		else if (randoption == 1){
			options = " --spider-net ";
			if (spideraxesoption == 1){
				options+= " --spider-arm-number " + spiderAxes.getText() + " ";
			}
			if (spidercirclesoption == 1){
				options+= " --spider-circle-number " + spiderCircles.getText() + " ";
			}
			if (spiderdistanceoption == 1){
				options+= " --spider-space-rad " + spiderDistances.getText() + " ";
			}
		}
		else if (randoption == 2){
			options = " --random-net";
		}

		/*Need to do OS environment check*/
		if (envos == 0){
			//cmd = "cmd /c " + sumoPath + "netgen" + options + " -v -o " + "\"" +outputPath.getText()+ "\"";
			cmd = "\""  + sumoPath + System.getProperty("file.separator") + "netgen" + "\"";
			cmd += options + " -v -o ";
			cmd += "\"" + outputPath.getText() + "\"";
			//System.out.println(cmd);
		}
		else {
			cmd = "netgen" + options + " -v -o " + outputPath.getText();
		}

		try {
			Process process = Runtime.getRuntime().exec(cmd);			
			
			new Thread(new SubTask(process.getInputStream())).start();
		    new Thread(new SubTask(process.getErrorStream())).start();
			process.waitFor();
						
//			BufferedReader buf = new BufferedReader(new InputStreamReader(process.getInputStream()));
//			BufferedReader buf2 = new BufferedReader(new InputStreamReader(process.getErrorStream()));
//			while ((line = buf.readLine()) != null){
//				statusText.append(line + "\n");
//				statusText.setCaretPosition(statusText.getDocument().getLength());
//			}
//			while ((line = buf2.readLine()) != null){
//				statusText.append(line + "\n");
//				statusText.setCaretPosition(statusText.getDocument().getLength());
//			}
			process.destroy();
//			buf.close();
//			buf2.close();
		}
		catch (Exception err) {
			err.printStackTrace();
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
				statusText.append(input + "\n");
				statusText.setCaretPosition(statusText.getDocument().getLength());
		        input = iReader.readLine();
		      }
		    }
		    catch (IOException ioe) {
		    }
		  }
		 
		  private BufferedReader iReader;
	}

	private void cancelButtonActionPerformed(ActionEvent evt) {
		this.dispose();
	}

	private void quickHelpMenuActionPerformed(ActionEvent evt) {
		JOptionPane.showMessageDialog(this, "Not yet available");
	}

	private void gridOptionActionPerformed(ActionEvent evt) {
		randoption = 0;
	}

	private void spiderOptionActionPerformed(ActionEvent evt) {
		randoption = 1;
	}

	private void randomOptionActionPerformed(ActionEvent evt) {
		randoption = 2;
	}

	private void openOutputActionPerformed(ActionEvent evt) {
		String extension = ".net.xml";
		String openPath = "untitled";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showDialog(this,"Create");
		if(returnVal == JFileChooser.APPROVE_OPTION) {
			openPath=chooser.getSelectedFile().getPath();
		if ( openPath.indexOf(extension) >= 0 )
			;
		else 
			openPath=chooser.getSelectedFile().getPath() + extension;		
		outputPath.setText( openPath );
		}
	}

	private void spiderDistanceCheckItemStateChanged(ItemEvent evt) {
		if (evt.getStateChange() == ItemEvent.DESELECTED) {
			spiderdistanceoption = 0;
		}
		else {
			spiderdistanceoption = 1;
		}
	}

	private void spiderCirclesCheckItemStateChanged(ItemEvent evt) {
		if (evt.getStateChange() == ItemEvent.DESELECTED) {
			spidercirclesoption = 0;
		}
		else {
			spidercirclesoption = 1;
		}
	}

	private void spiderAxesCheckItemStateChanged(ItemEvent evt) {
		if (evt.getStateChange() == ItemEvent.DESELECTED) {
			spideraxesoption = 0;
		}
		else {
			spideraxesoption = 1;
		}
	}

	private void yLengthCheckItemStateChanged(ItemEvent evt) {
		if (evt.getStateChange() == ItemEvent.DESELECTED) {
			gridlengthYoption = 0;
		}
		else {
			gridlengthYoption = 1;
		}
	}

	private void xLengthCheckItemStateChanged(ItemEvent evt) {
		if (evt.getStateChange() == ItemEvent.DESELECTED) {
			gridlengthXoption = 0;
		}
		else {
			gridlengthXoption = 1;
		}
	}

	private void gridRoadsCheckItemStateChanged(ItemEvent evt) {
		if (evt.getStateChange() == ItemEvent.DESELECTED) {
			gridlengthoption = 0;
		}
		else {
			gridlengthoption = 1;
		}
	}

	private void yJunctionCheckItemStateChanged(ItemEvent evt) {
		if (evt.getStateChange() == ItemEvent.DESELECTED) {
			gridjunctionYoption = 0;
		}
		else {
			gridjunctionYoption = 1;
		}
	}

	private void xJunctionCheckItemStateChanged(ItemEvent evt) {
		if (evt.getStateChange() == ItemEvent.DESELECTED) {
			gridjunctionXoption = 0;
		}
		else {
			gridjunctionXoption = 1;
		}
	}

	private void gridJunctionsCheckItemStateChanged(ItemEvent evt) {
		if (evt.getStateChange() == ItemEvent.DESELECTED) {
			gridjunctionsoption = 0;
		}
		else {
			gridjunctionsoption = 1;
		}
	}
}
