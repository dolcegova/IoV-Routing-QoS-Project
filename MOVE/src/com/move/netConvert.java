package com.move;
import javax.swing.*;
import java.awt.event.*;
import java.awt.*;
import java.io.*;

/**
*
* @author  Feliz Kristianto
*/
public class netConvert extends javax.swing.JFrame {
	private String filename = "untitled";
	private int osenv;
	static JFileChooser chooser;
	private JButton OKButton;
	private JButton cancelButton;
	private JTextField configurationPath;
	private JMenu helpMenu;
	private JLabel jLabel1;
	private JLabel jLabel6;
	private JMenuBar jMenuBar;
	private JTextArea statusText;
	private JButton openPath;
	private JMenuItem quickHelpButton;

	public netConvert(String sumoPath, int os) {
		osenv = os;		
		initComponents(sumoPath, os);
	}

	private void initComponents(String sumoPath, int os) {
		GridBagConstraints gridBagConstraints;
		
		setTitle("Generate map");

		jLabel1 = new JLabel();
		OKButton = new JButton();
		cancelButton = new JButton();
		jLabel6 = new JLabel();
		configurationPath = new JTextField();
		openPath = new JButton();
		statusText = new JTextArea();
		jMenuBar = new JMenuBar();
		helpMenu = new JMenu();
//		quickHelpButton = new JMenuItem();

		getContentPane().setLayout(new GridBagLayout());
		if(chooser == null) {
			chooser = new JFileChooser("Open File");
		}
		setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);

		jLabel1.setFont(new Font("Tahoma", 1, 12));
		jLabel1.setHorizontalAlignment(SwingConstants.CENTER);
		jLabel1.setText("Generate the Map");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.insets = new Insets(20, 20, 0, 0);
		getContentPane().add(jLabel1, gridBagConstraints);

		filename = sumoPath;
		OKButton.setText("OK");
		OKButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				OKButtonActionPerformed(evt,filename,osenv);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHEAST;
		gridBagConstraints.insets = new Insets(10, 20, 20, 0);
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
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(10, 10, 20, 0);
		getContentPane().add(cancelButton, gridBagConstraints);

		jLabel6.setText("Open map configuration file");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(20, 20, 0, 10);
		getContentPane().add(jLabel6, gridBagConstraints);

		configurationPath.setText(".netc.cfg");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.ipadx = 150;
		gridBagConstraints.anchor = GridBagConstraints.NORTHEAST;
		gridBagConstraints.insets = new Insets(20, 0, 0, 0);
		getContentPane().add(configurationPath, gridBagConstraints);

		openPath.setText("...");
		openPath.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openPathActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(20, 0, 0, 20);
		getContentPane().add(openPath, gridBagConstraints);

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
		gridBagConstraints.insets = new Insets(10, 20, 0, 20);
		getContentPane().add(scrollPane, gridBagConstraints);

		helpMenu.setText("Help");
//		quickHelpButton.setText("Quick Help");
//		quickHelpButton.addActionListener(new ActionListener() {
//			public void actionPerformed(ActionEvent evt) {
//				quickHelpActionPerformed(evt);
//			}
//		});
//		helpMenu.add(quickHelpButton);
/*Quick help is disabled for now*/
		//jMenuBar.add(helpMenu);

		setJMenuBar(jMenuBar);

		pack();
		setVisible(true);
	}
	private void quickHelpActionPerformed(ActionEvent evt) {
		JOptionPane.showMessageDialog(this, "Not yet available");
	}

	private void OKButtonActionPerformed(ActionEvent evt, String sumoPath, int envos) {
		String line;
		String cmd;
		/*Need to do OS environment check*/
		if (envos == 0){
			//cmd = "cmd /c " + sumoPath + "netconvert -v -c " + configurationPath.getText();
			cmd = "\"" + sumoPath + "netconvert.exe" + "\""; 
			cmd += " -v -c ";
			cmd += "\"" + configurationPath.getText() + "\""; 
			//System.out.println(cmd);
		}
		else {
			cmd = "netconvert -v -c " + configurationPath.getText();
		}
		try {
//			System.out.println(cmd);
			Process process = Runtime.getRuntime().exec(cmd);			
			
			new Thread(new SubTask(process.getInputStream())).start();
		    new Thread(new SubTask(process.getErrorStream())).start();
			process.waitFor();
			
//			BufferedReader buf = new BufferedReader(new InputStreamReader(process.getInputStream()), 65535);
//			BufferedReader buf2 = new BufferedReader(new InputStreamReader(process.getErrorStream()), 65535);
//			
//			while ((line = buf.readLine()) != null){
//				statusText.append(line + "\n");
//				statusText.setCaretPosition(statusText.getDocument().getLength());
//			}
//			
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

	private void openPathActionPerformed(ActionEvent evt) {
		String extension = ".netc.cfg";
		chooser.addChoosableFileFilter( new JaveFileFilter(extension) );
		int returnVal =  chooser.showDialog(this,"Create");
		if(returnVal == JFileChooser.APPROVE_OPTION)
		configurationPath.setText(chooser.getSelectedFile().getPath());
	}        
}
