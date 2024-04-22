package com.move;
import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.EventQueue;
import java.awt.ItemSelectable;

/*
 * Creator: Feliz Kristianto Karnadi and Zhi Hai Mo
 * UNSW - Sydney
 * 
 * Maintain: Jensen Chou
 */

public class vanetsim extends JFrame {

	public vanetsim() {
		initComponents();
	}
	private JButton MOVEButton;
	private JButton TCLButton;
	private JLabel jLabel1;
	private JLabel jLabel2;
	private JLabel jLabel3;

	private void initComponents() {
		GridBagConstraints gridBagConstraints;

		jLabel1 = new JLabel();
		MOVEButton = new JButton();
		TCLButton = new JButton();
		jLabel2 = new JLabel();
		jLabel3 = new JLabel();

		getContentPane().setLayout(new GridBagLayout());
		setTitle("Rapid Generator for VANET Simulation (v 2.94)");
		setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
		jLabel1.setFont(new Font("Arial", 1, 20));
		jLabel1.setForeground(new Color(0, 51, 255));
		jLabel1.setText("Rapid Generation of Realistic Simulation for VANET");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.ipadx = 14;
		gridBagConstraints.ipady = 6;
		gridBagConstraints.insets = new Insets(10, 20, 0, 13);
		getContentPane().add(jLabel1, gridBagConstraints);

		MOVEButton.setText("Mobility Model");
		//MOVEButton.setPreferredSize(new Dimension(100,23));
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.insets = new Insets(20, 30, 0, 0);
		getContentPane().add(MOVEButton, gridBagConstraints);

		MOVEButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				MOVEButtonActionPerformed(evt);
			}
		});

		TCLButton.setText("Traffic Model");
		//TCLButton.setPreferredSize(new Dimension(100,23));
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 5;
		gridBagConstraints.insets = new Insets(7, 30, 28, 0);
		getContentPane().add(TCLButton, gridBagConstraints);

		TCLButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				TCLButtonActionPerformed(evt);
			}
		});

		jLabel2.setText("Generation of road map topology and vehicle movement");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipady = 6;
		gridBagConstraints.insets = new Insets(20, 10, 0, 0);
		getContentPane().add(jLabel2, gridBagConstraints);

		jLabel3.setText("Generation of network traffic");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipady = 6;
		gridBagConstraints.insets = new Insets(7, 10, 0, 0);
		getContentPane().add(jLabel3, gridBagConstraints);

		pack();
		setVisible(true);
	}

	public static void main(String args[]) {
		new vanetsim();		
	}

	private void MOVEButtonActionPerformed(ActionEvent evt) {
		frontEnd fr = new frontEnd();
	}

	private void TCLButtonActionPerformed(ActionEvent evt) {
		trafficModel tr = new trafficModel();
	}
}
