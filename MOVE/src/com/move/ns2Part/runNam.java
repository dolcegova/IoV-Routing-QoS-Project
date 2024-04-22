package com.move.ns2Part;
import javax.swing.*;

import com.move.JaveFileFilter;

import java.awt.event.*;
import java.awt.*;
import java.io.*;

/**
*
* @author  Feliz Kristianto
*/

public class runNam extends JFrame {
	private String filename = "untitled";
	static JFileChooser chooser;

	private JMenu HelpMenu;
	private JButton OKButton;
	private JButton cancelButton;
	private JTextField namPath;
	private JLabel jLabel1;
	private JLabel jLabel3;
	private JMenuBar jMenuBar1;
	private JButton openFile;
	private JMenuItem quickHelpMenu;
	private JTextArea statusText;

	public runNam() {
		initComponents();
	}


	private void initComponents() {
		GridBagConstraints gridBagConstraints;

		jLabel1 = new JLabel();
		namPath = new JTextField();
		openFile = new JButton();
		jLabel3 = new JLabel();
		statusText = new JTextArea();
		OKButton = new JButton();
		cancelButton = new JButton();
		jMenuBar1 = new JMenuBar();
		HelpMenu = new JMenu();
		quickHelpMenu = new JMenuItem();

		getContentPane().setLayout(new GridBagLayout());
		if(chooser == null) {
			chooser = new JFileChooser("Open File");
		}

		setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
		setTitle("visualize NS-2");
		jLabel1.setFont(new Font("Tahoma", 1, 12));
		jLabel1.setText("Run Nam in background Console");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.insets = new Insets(20, 20, 10, 20);
		getContentPane().add(jLabel1, gridBagConstraints);

		namPath.setText("filename.nam");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.ipadx = 150;
		gridBagConstraints.ipady = 1;
		gridBagConstraints.anchor = GridBagConstraints.NORTHEAST;
		gridBagConstraints.insets = new Insets(10, 0, 0, 0);
		getContentPane().add(namPath, gridBagConstraints);

		openFile.setText("...");
		openFile.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openFileActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(10, 0, 0, 0);
		getContentPane().add(openFile, gridBagConstraints);

		jLabel3.setText("Nam trace file");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(10, 20, 0, 0);
		getContentPane().add(jLabel3, gridBagConstraints);

		statusText.setEditable(false);
		JScrollPane scrollPane = new JScrollPane(statusText,
		JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
		JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.ipadx = 450;
		gridBagConstraints.ipady = 150;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(10, 20, 10, 20);
		getContentPane().add(scrollPane, gridBagConstraints);

		OKButton.setText("OK");
		OKButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				OKButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHEAST;
		gridBagConstraints.insets = new Insets(20, 20, 20, 0);
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
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(20, 10, 20, 20);
		getContentPane().add(cancelButton, gridBagConstraints);

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
		cmd = "nam " + namPath.getText();
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
		String extension = ".nam";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showOpenDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION)
		namPath.setText(chooser.getSelectedFile().getPath());
	}

	public static void main(String args[]) {

		new runNam();

	}




}
