package com.move.ns2Part;
/*
 * This file builds by Chine-Ming Chou, NCKU
 * 2009/02/18
 * 
 *  */


import javax.swing.*;

import java.awt.*;
import java.awt.event.*;
import java.io.*;


public class setTraCI extends JFrame {
	PrintWriter  printer;
	private JButton OKButton;
	private JTextField traciHostname;
	private JTextField traciPort;
	private JTextField traciPenetration;
	private JTextField traciSimulationTime;
	private JButton cancelButton;
	private JLabel jLabel1;
	private JLabel jLabel2;
	private JLabel jLabel3;
	private JLabel jLabel4;
	private JLabel jLabel5;
	
	public setTraCI(){
		initComponents();
	}

	public void initComponents() {
		GridBagConstraints gridBagConstraints;		
	
		jLabel1 = new JLabel();
		jLabel2 = new JLabel();				
		jLabel3 = new JLabel();
		jLabel4 = new JLabel();
		jLabel5 = new JLabel();
		traciHostname = new JTextField();
		traciPort = new JTextField();
		traciPenetration = new JTextField();
		traciSimulationTime = new JTextField();
		OKButton = new JButton();
		cancelButton = new JButton();		
		getContentPane().setLayout(new GridBagLayout());
		setTitle("TraCI simulation configuration");
		
		
		int openFlag = 0;

		try {
			File openfile	=	new	File("traciConfigure.txt");
			BufferedReader getContent	=	new	BufferedReader(new FileReader(openfile));
			
			openFlag = 1;
			traciHostname.setText( getContent.readLine() );
			traciPort.setText( getContent.readLine() );
			traciPenetration.setText( getContent.readLine() );
			traciSimulationTime.setText( getContent.readLine() );
		}
		catch (Exception e) {}
		
		
		jLabel1.setFont(new Font("Dialog", 1, 14));
		jLabel1.setText("Set TraCI Simulation Parameters");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.insets = new Insets(20, 0, 0, 49);
		getContentPane().add(jLabel1, gridBagConstraints);

		jLabel2.setText("Server IP:");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipady = 5;
		gridBagConstraints.anchor = java.awt.GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(21, 20, 4, 14);
		getContentPane().add(jLabel2, gridBagConstraints);

		if ( openFlag == 0 )
			traciHostname.setText("localhost");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.ipadx = 78;
		gridBagConstraints.anchor = java.awt.GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(21, 110, 4, 14);
		getContentPane().add(traciHostname, gridBagConstraints);

		jLabel3.setText("Port:");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipady = 5;
		gridBagConstraints.anchor = java.awt.GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(61, 20, 4, 14);
		getContentPane().add(jLabel3, gridBagConstraints);
		
		if ( openFlag == 0 )
			traciPort.setText("8888");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.anchor = java.awt.GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(61, 110, 4, 14);
		getContentPane().add(traciPort, gridBagConstraints);
		
		jLabel4.setText("Penetration Ratio (0~1):");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipady = 5;
		gridBagConstraints.anchor = java.awt.GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(13, 20, 4, 14);
		getContentPane().add(jLabel4, gridBagConstraints);
		
		if ( openFlag == 0 )
			traciPenetration.setText("1");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.ipadx = 20;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.anchor = java.awt.GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(13, 200, 4, 14);
		getContentPane().add(traciPenetration, gridBagConstraints);
		
		jLabel5.setText("Total Simulation Time of SUMO:");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipady = 5;
		gridBagConstraints.anchor = java.awt.GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(13, 20, 4, 14);
		getContentPane().add(jLabel5, gridBagConstraints);
		
		if ( openFlag == 0 )
			traciSimulationTime.setText("1000");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.anchor = java.awt.GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(13, 260, 4, 14);
		getContentPane().add(traciSimulationTime, gridBagConstraints);
				
		OKButton.setText("OK");
		OKButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				OKButtonActionPerformed(evt);
			}
		});
		
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.gridwidth = 2;
		//gridBagConstraints.ipadx = 3;
		//gridBagConstraints.ipady = 3;
		gridBagConstraints.insets = new Insets(14, 10, 16, 277);
		getContentPane().add(OKButton, gridBagConstraints);

		cancelButton.setText("Cancel");
		cancelButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				cancelButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 5;
		//gridBagConstraints.ipadx = 3;
		gridBagConstraints.insets = new Insets(14, 70, 16, 187);
		getContentPane().add(cancelButton, gridBagConstraints);

		//pack();		
		java.awt.Dimension screenSize = java.awt.Toolkit.getDefaultToolkit().getScreenSize();
		//setBounds((screenSize.width-1000)/2, screenSize.height-1000, 950, 750);
		setBounds(0, 0, 400, 350);		
		setVisible(true);
	}
	private void cancelButtonActionPerformed(ActionEvent evt) {
		this.dispose();
	}
	private void OKButtonActionPerformed(ActionEvent evt) {
		try{
			printer = new PrintWriter(new	BufferedWriter(new FileWriter("traciConfigure.txt")));
		}
		catch(IOException	e){};		
		printer.print(traciHostname.getText() + "\n");
		printer.print(traciPort.getText() + "\n");
		printer.print(traciPenetration.getText() + "\n");
		printer.print(traciSimulationTime.getText() + "\n");
		printer.close();
		this.dispose();
	}
}

