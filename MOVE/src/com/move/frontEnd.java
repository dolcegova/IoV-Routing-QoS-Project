package com.move;
import java.awt.*;
import java.awt.event.*;
import java.io.*;
import java.awt.*;
import java.awt.event.*;

import javax.swing.*;
import javax.swing.event.*;

import com.move.mapConvertPart.mapConvert;

/*
Creator: Feliz Kristianto Karnadi and Zhi Hai Mo
UNSW - Sydney
*/

public class frontEnd extends JFrame {

	private static String sumoPath ="";

	private int osenv;

	private JMenuItem aboutmenu;
	private JButton automaticRouteButton;
	private JButton busTimetableButton;
	private JButton cfgButton;
	private JButton edgesButton;
	private JMenu envMenu;
	private JMenu file;	
	private JButton flowButton;
	private JButton guisimButton;
	private JMenu help;
	private JLabel jLabel1;
	private JLabel jLabel10;
	private JLabel jLabel11;
	private JLabel jLabel12;
	private JLabel jLabel13;
	private JLabel jLabel14;
	private JLabel jLabel15;
	private JLabel jLabel16;
	private JLabel jLabel17;
	private JLabel jLabel19;
	private JLabel jLabel2;
	private JLabel jLabel20;
	private JLabel jLabel21;
	private JLabel jLabel22;
	private JLabel jLabel23;
	private JLabel jLabel24;
	private JLabel jLabel25;
	private JLabel jLabel26;
	private JLabel jLabel28;
	private JLabel jLabel3;
	private JLabel jLabel4;
	private JLabel jLabel5;
	private JLabel jLabel6;
	private JLabel jLabel7;
	private JLabel jLabel8;
	private JLabel jLabel9;
	private JMenuBar jMenuBar;
	private JPanel jPanel1;
	private JSeparator jSeparator1;	
	private JButton manualRouteButton;
	private JButton netConvertButton;
	private JButton netGenButton;
	private JButton netcButton;
	private JButton nodesButton;
	private JMenuItem quickHelp;
	private JMenuItem quitMenu;
	private JMenuItem setPathMenu;
	private JButton sumoButton;
	private JButton tigerButton;
	private JButton tripButton;
	private JButton turnButton;
	private JButton typesButton;	
	private JScrollPane jScrollPane1;
	
	private int sumoBinariesFlag = 0;
	
	public frontEnd() {
		String osname = System.getProperty("os.name","").toLowerCase();
		if ( osname.startsWith("windows") ) {
		   // windows
		   osenv = 0;
		}	    
		else {
		   osenv = 1;
		}
		initComponents();
	}


	private void initComponents() {
		GridBagConstraints gridBagConstraints;		
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		
		jPanel1 = new JPanel();
		jLabel1 = new JLabel();
		jLabel2 = new JLabel();
		cfgButton = new JButton();
		netcButton = new JButton();
		nodesButton = new JButton();
		edgesButton = new JButton();
		netConvertButton = new JButton();
		netGenButton = new JButton();
		jLabel5 = new JLabel();
		jLabel6 = new JLabel();
		manualRouteButton = new JButton();
		automaticRouteButton = new JButton();
		sumoButton = new JButton();
		guisimButton = new JButton();
		jLabel8 = new JLabel();
		jLabel9 = new JLabel();
		jLabel10 = new JLabel();
		typesButton = new JButton();
		jLabel11 = new JLabel();
		jLabel12 = new JLabel();
		jLabel13 = new JLabel();
		jLabel4 = new JLabel();
		tigerButton = new JButton();
		jLabel7 = new JLabel();
		jLabel15 = new JLabel();
		jLabel16 = new JLabel();
		jLabel17 = new JLabel();
		jLabel19 = new JLabel();
		jLabel21 = new JLabel();
		flowButton = new JButton();
		tripButton = new JButton();
		turnButton = new JButton();
		jLabel22 = new JLabel();
		jLabel23 = new JLabel();
		jLabel24 = new JLabel();
		jLabel3 = new JLabel();
		jLabel20 = new JLabel();
		jLabel25 = new JLabel();
		jLabel14 = new JLabel();
		jLabel26 = new JLabel();
		busTimetableButton = new JButton();
		jLabel28 = new JLabel();
		jMenuBar = new JMenuBar();
		file = new JMenu();		
		setPathMenu = new JMenuItem();
		envMenu = new JMenu();
		
		jSeparator1 = new JSeparator();
		quitMenu = new JMenuItem();
		help = new JMenu();
		quickHelp = new JMenuItem();
		aboutmenu = new JMenuItem();
		
		jScrollPane1 = new JScrollPane();

		getContentPane().setLayout(new GridBagLayout());

		setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
		setTitle("MOVE");
		
		jPanel1.setLayout(new GridBagLayout());
		jLabel1.setFont(new Font("Arial", 1, 14));
		jLabel1.setForeground(new Color(0, 51, 153));
		jLabel1.setText("Simulation");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 36;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(13, 10, 0, 0);
		jPanel1.add(jLabel1, gridBagConstraints);

		jLabel2.setFont(new Font("Arial", 1, 14));
		jLabel2.setForeground(new Color(0, 51, 153));
		jLabel2.setText("Map Editor");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(8, 10, 0, 0);
		jPanel1.add(jLabel2, gridBagConstraints);

		cfgButton.setText("Configuration");
		cfgButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				cfgButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 37;
		gridBagConstraints.gridwidth = 9;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 33;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 30, 0, 0);
		jPanel1.add(cfgButton, gridBagConstraints);

		netcButton.setText("Configuration");
		netcButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				netcButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.gridwidth = 9;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 33;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 30, 0, 0);
		jPanel1.add(netcButton, gridBagConstraints);

		nodesButton.setText("Node");
		nodesButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				nodesButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 9;
		gridBagConstraints.ipadx = 73;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(6, 30, 0, 0);
		jPanel1.add(nodesButton, gridBagConstraints);

		edgesButton.setText("Edge");
		edgesButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				edgesButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.gridwidth = 9;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 73;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(2, 30, 0, 0);
		jPanel1.add(edgesButton, gridBagConstraints);

		netConvertButton.setText("Create Map");
		netConvertButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				netConvertButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.gridwidth = 9;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 41;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 30, 0, 0);
		jPanel1.add(netConvertButton, gridBagConstraints);

		netGenButton.setText("Random Map");
		netGenButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				netGenButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 14;
		gridBagConstraints.gridwidth = 8;
		gridBagConstraints.ipadx = 32;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(6, 30, 0, 0);
		jPanel1.add(netGenButton, gridBagConstraints);

		jLabel5.setText("Create random map");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 14;
		gridBagConstraints.gridwidth = 6;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipady = 4;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(6, 23, 0, 0);
		jPanel1.add(jLabel5, gridBagConstraints);

		jLabel6.setFont(new Font("Arial", 1, 14));
		jLabel6.setForeground(new Color(0, 51, 153));
		jLabel6.setText("Vehicle Movement Editor");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 20;
		gridBagConstraints.gridwidth = 12;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(13, 10, 0, 0);
		jPanel1.add(jLabel6, gridBagConstraints);

		manualRouteButton.setText("Manual Vehicle");
		manualRouteButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				manualRouteButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 31;
		gridBagConstraints.gridwidth = 9;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 27;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(6, 30, 0, 0);
		jPanel1.add(manualRouteButton, gridBagConstraints);

		automaticRouteButton.setText("Create Vehicle");
		automaticRouteButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				automaticRouteButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 28;
		gridBagConstraints.gridwidth = 9;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 29;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(13, 30, 0, 0);
		jPanel1.add(automaticRouteButton, gridBagConstraints);

		sumoButton.setText("Run Simulation");
		sumoButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				sumoButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 41;
		gridBagConstraints.gridwidth = 9;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 27;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 30, 23, 0);
		jPanel1.add(sumoButton, gridBagConstraints);

		guisimButton.setText("Visualization");
		guisimButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				guisimButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 39;
		gridBagConstraints.gridwidth = 9;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 39;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 30, 0, 0);
		jPanel1.add(guisimButton, gridBagConstraints);

		jLabel8.setText("Junction and dead end");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 9;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipady = 4;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(6, 23, 0, 0);
		jPanel1.add(jLabel8, gridBagConstraints);

		jLabel9.setText("Road");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(2, 23, 0, 0);
		jPanel1.add(jLabel9, gridBagConstraints);

		jLabel10.setFont(new Font("Dialog", 1, 24));
		jLabel10.setForeground(new Color(255, 0, 51));
		jLabel10.setText("Mobility Model Generator for VANET");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 29;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(10, 10, 0, 21);
		jPanel1.add(jLabel10, gridBagConstraints);

		typesButton.setText("Edge Type");
		typesButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				typesButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.gridwidth = 10;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 48;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 30, 0, 0);
		jPanel1.add(typesButton, gridBagConstraints);

		jLabel11.setForeground(new Color(153, 51, 0));
		jLabel11.setText("(optional) road type");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.gridwidth = 7;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 23, 0, 0);
		jPanel1.add(jLabel11, gridBagConstraints);

		jLabel12.setText("Map configuration");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 23, 0, 0);
		jPanel1.add(jLabel12, gridBagConstraints);

		jLabel13.setText("Generate map");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 23, 0, 0);
		jPanel1.add(jLabel13, gridBagConstraints);

		jLabel4.setText("Generate vehicle movement");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 28;
		gridBagConstraints.gridwidth = 12;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(13, 23, 0, 0);
		jPanel1.add(jLabel4, gridBagConstraints);

		tigerButton.setText("Map Converter");
		tigerButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				tigerButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 17;
		gridBagConstraints.gridwidth = 9;
		gridBagConstraints.gridheight = 3;
		gridBagConstraints.ipadx = 25;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(6, 30, 0, 0);
		jPanel1.add(tigerButton, gridBagConstraints);

		jLabel7.setText("Generate map's node and edge files");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 17;
		gridBagConstraints.gridwidth = 11;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(6, 23, 0, 0);
		jPanel1.add(jLabel7, gridBagConstraints);

		jLabel15.setText("Manually set the movement for each vehicle");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 31;
		gridBagConstraints.gridwidth = 16;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(12, 23, 0, 0);
		jPanel1.add(jLabel15, gridBagConstraints);

		jLabel16.setText("Visualize simulation");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 39;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 23, 0, 0);
		jPanel1.add(jLabel16, gridBagConstraints);

		jLabel17.setText("Run simulation on background");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 41;
		gridBagConstraints.gridwidth = 13;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 23, 0, 0);
		jPanel1.add(jLabel17, gridBagConstraints);

		jLabel19.setText("Simulation configuration");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 37;
		gridBagConstraints.gridwidth = 10;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 23, 0, 0);
		jPanel1.add(jLabel19, gridBagConstraints);

		jLabel21.setForeground(new Color(51, 102, 0));
		jLabel21.setText("Manual Vehicle Movement");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 30;
		gridBagConstraints.gridwidth = 7;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 30, 0, 0);
		jPanel1.add(jLabel21, gridBagConstraints);

		flowButton.setText("Flow");
		flowButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				flowButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 22;
		gridBagConstraints.gridwidth = 9;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 75;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(6, 30, 0, 0);
		jPanel1.add(flowButton, gridBagConstraints);

		tripButton.setText("Trip");
		tripButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				tripButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 26;
		gridBagConstraints.gridwidth = 9;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 79;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 30, 0, 0);
		jPanel1.add(tripButton, gridBagConstraints);

		turnButton.setText("Turn");
		turnButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				turnButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 24;
		gridBagConstraints.gridwidth = 9;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 75;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 30, 0, 0);
		jPanel1.add(turnButton, gridBagConstraints);

		jLabel22.setText("Vehicle trip definition");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 22;
		gridBagConstraints.gridwidth = 8;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(6, 23, 0, 0);
		jPanel1.add(jLabel22, gridBagConstraints);

		jLabel23.setForeground(new Color(153, 51, 0));
		jLabel23.setText("(optional) trip for each vehicle type");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 26;
		gridBagConstraints.gridwidth = 14;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 23, 0, 0);
		jPanel1.add(jLabel23, gridBagConstraints);

		jLabel24.setText("Probability of directions on each junction");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 24;
		gridBagConstraints.gridwidth = 15;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 23, 0, 0);
		jPanel1.add(jLabel24, gridBagConstraints);

		jLabel3.setForeground(new Color(0, 102, 0));
		jLabel3.setText("Manual Map");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 30, 0, 0);
		jPanel1.add(jLabel3, gridBagConstraints);

		jLabel20.setForeground(new Color(0, 102, 0));
		jLabel20.setText("Random Map");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 13;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 30, 0, 0);
		jPanel1.add(jLabel20, gridBagConstraints);

		jLabel25.setForeground(new Color(0, 102, 0));
		jLabel25.setText("Import Map (TIGER, KML, OSM)");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 16;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(2, 30, 0, 0);
		jPanel1.add(jLabel25, gridBagConstraints);

		jLabel14.setForeground(new Color(0, 102, 0));
		jLabel14.setText("Automatic Vehicle Movement");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 21;
		gridBagConstraints.gridwidth = 11;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 30, 0, 0);
		jPanel1.add(jLabel14, gridBagConstraints);

		jLabel26.setForeground(new Color(51, 102, 0));
		jLabel26.setText("Bus Timetable Generator");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 33;
		gridBagConstraints.gridwidth = 6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(13, 30, 0, 0);
		jPanel1.add(jLabel26, gridBagConstraints);

		busTimetableButton.setText("Timetable");
		busTimetableButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				busTimetableButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 34;
		gridBagConstraints.gridwidth = 9;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 51;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(6, 30, 0, 0);
		jPanel1.add(busTimetableButton, gridBagConstraints);

		jLabel28.setText("Bus timetable");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 34;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(6, 23, 0, 0);
		jPanel1.add(jLabel28, gridBagConstraints);

//		jScrollPane1.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);	
//		jScrollPane1.setViewportView(jPanel1);				
//		getContentPane().add(jScrollPane1, gridBagConstraints);
//        JScrollPane scrollPane = new JScrollPane(jPanel1);        
        //Add the scroll pane to this window.
//        getContentPane().add(scrollPane, gridBagConstraints);

		
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.ipady = -2;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 0, 6, 0);
		getContentPane().add(jPanel1, gridBagConstraints);

		
		
		
		setPathMenu.setText("Set SUMO Binaries Path");
		setPathMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				setPathMenuActionPerformed(evt);
			}
		});

		file.setText("File");
		file.add(setPathMenu);		
		file.add(jSeparator1);

		quitMenu.setText("Quit");
		quitMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				quitMenuActionPerformed(evt);
			}
		});

		file.add(quitMenu);
		
		jMenuBar.add(file);		
		setJMenuBar(jMenuBar);
		
		pack();
		setVisible(true);
	}
	
	private void quitMenuActionPerformed(ActionEvent evt) {
		this.dispose();
	}

	private void sumoButtonActionPerformed(ActionEvent evt) {
		sumoBinariesFlag = 0;
		try {
			File filterfile	=	new	File("path.txt");
			BufferedReader getContent	=	new	BufferedReader(new FileReader(filterfile));
			sumoPath = getContent.readLine();
			sumoBinariesFlag = 1;
		}
		catch (Exception e) {}
		if ( sumoBinariesFlag == 0 && osenv == 0){
			JOptionPane.showMessageDialog(this, "Please first set SUMO Binaries \n");
		}
		else {
			runSumo runsum = new runSumo(sumoPath,osenv);
		}			
	}
	
	private void guisimButtonActionPerformed(ActionEvent evt) {
		String cmd = "";
		sumoBinariesFlag = 0;
		if (osenv == 0){			
			try {
				File filterfile	=	new	File("path.txt");
				BufferedReader getContent	=	new	BufferedReader(new FileReader(filterfile));
				sumoPath = getContent.readLine();
				sumoBinariesFlag = 1;
			}
			catch (Exception e) {}
			if ( sumoBinariesFlag == 0 && osenv == 0){
				JOptionPane.showMessageDialog(this, "Please first set SUMO Binaries \n");
			}
			else {				
				String binaryPath = sumoPath + "gui.exe" ;
				File check	=	new	File(binaryPath);
		
				if ( check.exists() == true ) {					
					cmd = "\"" + sumoPath + System.getProperty("file.separator") + "gui.exe" + "\"";	
				} else {
					binaryPath = sumoPath + "gui.exe" ;
					check	=	new	File(binaryPath);
					cmd = "\"" + sumoPath + System.getProperty("file.separator") + "sumo-gui.exe" + "\"";	
				}	
				
				try {
					Process process = Runtime.getRuntime().exec(cmd);
				}
				catch (Exception err) {
					err.printStackTrace();
				}
			}						
		}
		else {
			cmd = "sumo-gui";
			try {
				Process process = Runtime.getRuntime().exec(cmd);
			}
			catch (Exception err) {
				err.printStackTrace();
			}
		}		
	}

	private void manualRouteButtonActionPerformed(ActionEvent evt) {
		manualRoute newmanual = new manualRoute();
	}

	private void turnButtonActionPerformed(ActionEvent evt) {
		turnEditor newturns = new turnEditor();
	}

	private void tripButtonActionPerformed(ActionEvent evt) {
		tripEditor newtrip = new tripEditor();
	}

	private void flowButtonActionPerformed(ActionEvent evt) {
		flowEditor newflows = new flowEditor();
	}

	private void typesButtonActionPerformed(ActionEvent evt) {
		typeEditor roadType = new typeEditor();
	}

	private void tigerButtonActionPerformed(ActionEvent evt) {
		sumoBinariesFlag = 0;
		try {
			File filterfile	=	new	File("path.txt");
			BufferedReader getContent	=	new	BufferedReader(new FileReader(filterfile));
			sumoPath = getContent.readLine();
			sumoBinariesFlag = 1;
		}
		catch (Exception e) {}
		if ( sumoBinariesFlag == 0 && osenv == 0){
			JOptionPane.showMessageDialog(this, "Please first set SUMO Binaries \n");
		}
		else {
//			System.out.println(" sumoPath = " + sumoPath);
//			System.out.println(" osenv = " + osenv);
			mapConvert importTiger = new mapConvert(sumoPath,osenv);			
		}		
	}

	private void netConvertButtonActionPerformed(ActionEvent evt) {
		sumoBinariesFlag = 0;
		try {
			File filterfile	=	new	File("path.txt");
			BufferedReader getContent	=	new	BufferedReader(new FileReader(filterfile));
			sumoPath = getContent.readLine();
			sumoBinariesFlag = 1;
		}
		catch (Exception e) {}
		
		if ( sumoBinariesFlag == 0 && osenv == 0){
			JOptionPane.showMessageDialog(this, "Please first set SUMO Binaries \n");
		}
		else {
			netConvert newnetconv = new netConvert(sumoPath,osenv);
		}		
	}

	private void setPathMenuActionPerformed(ActionEvent evt) {
	    String osname = System.getProperty("os.name","").toLowerCase();
	    
	    if ( osenv == 0 ) {
		    // windows		    
		    setPath newsetpath = new setPath();
		}	    
	    else {			
			JOptionPane.showMessageDialog(this, "The current OS environment is set as Linux.\n" +
			"You do not need to set the SUMO binaries path.\n" +
			"It's already in your exported Path after installation.\n");
		}		
	}

	private void automaticRouteButtonActionPerformed(ActionEvent evt) {
		sumoBinariesFlag = 0;
		try {
			File filterfile	=	new	File("path.txt");
			BufferedReader getContent	=	new	BufferedReader(new FileReader(filterfile));
			sumoPath = getContent.readLine();
			sumoBinariesFlag = 1;
		}
		catch (Exception e) {}
		if ( sumoBinariesFlag == 0 && osenv == 0){
			JOptionPane.showMessageDialog(this, "Please first set SUMO Binaries \n");
		}
		else {
			autoRoutes openAuto = new autoRoutes(sumoPath,osenv);
		}
	}

	private void netGenButtonActionPerformed(ActionEvent evt) {
		sumoBinariesFlag = 0;
		try {
			File filterfile	=	new	File("path.txt");
			BufferedReader getContent	=	new	BufferedReader(new FileReader(filterfile));
			sumoPath = getContent.readLine();
			sumoBinariesFlag = 1;
		}
		catch (Exception e) {}
		if ( sumoBinariesFlag == 0 && osenv == 0){
			JOptionPane.showMessageDialog(this, "Please first set SUMO Binaries \n");
		}
		else {
			randomNetGen openRandom = new randomNetGen(sumoPath,osenv);
		}		
	}

	private void edgesButtonActionPerformed(ActionEvent evt) {
		edgeEditor openEdge = new edgeEditor();
	}

	private void netcButtonActionPerformed(ActionEvent evt) {
		netcEditor openNetc = new netcEditor();
	}

	private void cfgButtonActionPerformed(ActionEvent evt) {
		cfgEditor openCfg = new cfgEditor();
	}

	private void nodesButtonActionPerformed(ActionEvent evt) {
		nodeEditor openNode = new nodeEditor();
	}

	private void busTimetableButtonActionPerformed(ActionEvent evt) {
		busTimetableEditor openBus = new busTimetableEditor();
	}
	
	public void main(String args[]) {		
		new frontEnd();
	}
}

