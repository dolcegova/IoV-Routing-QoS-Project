package com.move;
/*
 * This file builds by Chine-Ming Chou, NCKU
 * 2009/02/04
 * 
 *  */


import javax.swing.*;

import java.awt.*;
import java.awt.event.*;
import java.io.*;


public class setPath extends JFrame {
	PrintWriter  printer;
	private JButton OKButton;
	private JTextField binaryLocation;
	private JButton cancelButton;
	private JLabel jLabel1;
	private JLabel jLabel2;
	private JLabel jLabel3;	
	private JButton binariesPathButton;
	private String filename = "untitled";
	static JFileChooser chooser;
	

	public setPath(){
		initComponents();
	}

	public void initComponents() {
		GridBagConstraints gridBagConstraints;

		if(chooser == null) {
			chooser = new JFileChooser("Open File");
		}

		jLabel1 = new JLabel();
		jLabel2 = new JLabel();
		binaryLocation = new JTextField();		
		jLabel3 = new JLabel();
		OKButton = new JButton();
		cancelButton = new JButton();
		binariesPathButton = new javax.swing.JButton();
		getContentPane().setLayout(new GridBagLayout());
		setTitle("Set SUMO Binaries Path");

		try {
			File filterfile	=	new	File("path.txt");
			BufferedReader getContent	=	new	BufferedReader(new FileReader(filterfile));
			filename = getContent.readLine();
		}
		catch (Exception e) {}
		//System.out.println(filename);
		binaryLocation.setText( filename );
		
		jLabel1.setFont(new Font("Dialog", 1, 14));
		jLabel1.setText("Set your current SUMO Binaries path");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.insets = new Insets(20, 0, 0, 49);
		getContentPane().add(jLabel1, gridBagConstraints);

		jLabel2.setText("Path:");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.insets = new Insets(21, 20, 4, 14);
		getContentPane().add(jLabel2, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.ipadx = 300;
		gridBagConstraints.insets = new Insets(21, 0, 0, 10);
		getContentPane().add(binaryLocation, gridBagConstraints);

		binariesPathButton.setText("...");		
		binariesPathButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				selectBinariesPath(evt);
			}
		});
		
        binariesPathButton.setPreferredSize(new java.awt.Dimension(30, 20));
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 2;
        gridBagConstraints.gridy = 1;        
        gridBagConstraints.insets = new Insets(22, 0, 4, 20);
        add(binariesPathButton, gridBagConstraints);
				
		jLabel3.setFont(new Font("Dialog", 0, 12));
		jLabel3.setText("Example: C:\\sumo-0.12.3\\bin\\");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(10, 20, 0, 194);
		getContentPane().add(jLabel3, gridBagConstraints);

		OKButton.setText("OK");
		OKButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				OKButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 29;
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
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = 7;
		gridBagConstraints.insets = new Insets(14, 40, 16, 187);
		getContentPane().add(cancelButton, gridBagConstraints);

		pack();
		setVisible(true);
	}
	private void selectBinariesPath(ActionEvent evt) {
		chooser.setFileSelectionMode( JFileChooser.DIRECTORIES_ONLY );
		 int returnVal = chooser.showOpenDialog(this);
		 if(returnVal == JFileChooser.APPROVE_OPTION) 
			 filename=chooser.getSelectedFile().getPath();
		 binaryLocation.setText( filename + System.getProperty("file.separator") );			
	}	
	private void cancelButtonActionPerformed(ActionEvent evt) {
		this.dispose();
	}
	private void OKButtonActionPerformed(ActionEvent evt) {
		try{
			printer = new PrintWriter(new	BufferedWriter(new FileWriter("path.txt")));
		}
		catch(IOException	e){};
		//System.out.println(binaryLocation.getText());
		printer.print(binaryLocation.getText() + "\n");
		printer.close();
		this.dispose();
	}
}
