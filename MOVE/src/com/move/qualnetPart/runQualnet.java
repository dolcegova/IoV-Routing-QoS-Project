package com.move.qualnetPart;
import javax.swing.*;

import com.move.JaveFileFilter;

import java.awt.event.*;
import java.awt.*;
import java.io.*;

/**
*
* @author  Feliz Kristianto
*/

public class runQualnet extends javax.swing.JFrame {
	private String filename = "untitled";
	static JFileChooser chooser;

	private JMenu HelpMenu;
	private JButton OKButton;
	private JButton cancelButton;
	private JTextField qualnetPath;
	private JLabel jLabel1;
	private JLabel jLabel3;
	private JMenuBar jMenuBar1;
	private JButton openFile;
	private JMenuItem quickHelpMenu;
	private JTextArea statusText;
	public runQualnet() {
		initComponents();
	}


	private void initComponents() {
		GridBagConstraints gridBagConstraints;

		jLabel1 = new JLabel();
		qualnetPath = new JTextField();
		jLabel3 = new JLabel();
		openFile = new JButton();
		statusText = new JTextArea();
		cancelButton = new JButton();
		OKButton = new JButton();
		jMenuBar1 = new JMenuBar();
		HelpMenu = new JMenu();
		quickHelpMenu = new JMenuItem();
		if(chooser == null) {
			chooser = new JFileChooser("Open File");
		}

		getContentPane().setLayout(new GridBagLayout());
		
		setTitle("Run Qualnet in console");
		jLabel1.setFont(new Font("Tahoma", 1, 12));
		jLabel1.setText("Run Qualnet in background Console");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(20, 73, 0, 0);
		getContentPane().add(jLabel1, gridBagConstraints);

		qualnetPath.setText("filename.config");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.ipadx = 159;
		gridBagConstraints.ipady = 1;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(15, 0, 0, 0);
		getContentPane().add(qualnetPath, gridBagConstraints);

		jLabel3.setText("Qualnet configuration file");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 29;
		gridBagConstraints.ipady = 6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(15, 30, 0, 0);
		getContentPane().add(jLabel3, gridBagConstraints);

		openFile.setText("...");
		openFile.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openFileActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(15, 0, 0, 0);
		getContentPane().add(openFile, gridBagConstraints);

		statusText.setEditable(false);
		JScrollPane scrollPane = new JScrollPane(statusText,
		JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
		JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 10;
		gridBagConstraints.ipadx = 450;
		gridBagConstraints.ipady = 150;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(7, 30, 0, 32);
		getContentPane().add(scrollPane, gridBagConstraints);

		cancelButton.setText("Cancel");
		cancelButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				cancelButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(20, 13, 26, 0);
		getContentPane().add(cancelButton, gridBagConstraints);

		OKButton.setText("OK");
		OKButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				OKButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(20, 40, 26, 0);
		getContentPane().add(OKButton, gridBagConstraints);

		HelpMenu.setText("Help");
		quickHelpMenu.setText("Quick Help");
		quickHelpMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				quickHelpMenuActionPerformed(evt);
			}
		});

		HelpMenu.add(quickHelpMenu);
/*Quick help is disabled for now*/
		//jMenuBar1.add(HelpMenu);

		setJMenuBar(jMenuBar1);

		pack();
		setVisible(true);
	}

	private void quickHelpMenuActionPerformed(ActionEvent evt) {
		JOptionPane.showMessageDialog(this, "Not yet available");
	}

	private void OKButtonActionPerformed(ActionEvent evt) {
		String line;
		String cmd;
		cmd = "qualnet " + qualnetPath.getText();
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

	private void openFileActionPerformed(ActionEvent evt) {
		String extension = ".config";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showOpenDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION)
		qualnetPath.setText(chooser.getSelectedFile().getPath());
	}

	public static void main(String args[]) {
		new runQualnet();
	}





}
