package com.move;
import javax.swing.*;
import java.awt.event.*;
import java.awt.*;
import java.io.*;

public class runSumo extends JFrame {
	
	private String filename = "untitled";
	private int osenv;
	static JFileChooser chooser;	
	
	private JMenu HelpMenu;
	private JButton OKButton;
	private JButton cancelButton;
	private JTextField cfgPath;
	private JLabel jLabel1;
	private JLabel jLabel3;
	private JMenuBar jMenuBar1;
	private JButton openFile;
	private JMenuItem quickHelpMenu;
	private JTextArea statusText;

	public runSumo(String sumoPath, int os) {
		osenv = os;
		initComponents(sumoPath, os);
	}

	private void initComponents(String sumoPath, int os) {
		GridBagConstraints gridBagConstraints;

		jLabel1 = new JLabel();
		cfgPath = new JTextField();
		jLabel3 = new JLabel();
		openFile = new JButton();
		statusText = new JTextArea();
		cancelButton = new JButton();
		OKButton = new JButton();
		jMenuBar1 = new JMenuBar();
		HelpMenu = new JMenu();
		quickHelpMenu = new JMenuItem();
		
		setTitle("Run SUMO simulation");
		
		if(chooser == null) {
			chooser = new JFileChooser("Open File");
		}
		filename = sumoPath;
		getContentPane().setLayout(new GridBagLayout());

		setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
		jLabel1.setFont(new Font("Tahoma", 1, 12));
		jLabel1.setText("Run Sumo in background Console");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.insets = new Insets(20, 20, 10, 20);
		getContentPane().add(jLabel1, gridBagConstraints);

		cfgPath.setText("filename.sumo.cfg");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.ipadx = 150;
		gridBagConstraints.ipady = 1;
		gridBagConstraints.anchor = GridBagConstraints.NORTHEAST;
		gridBagConstraints.insets = new Insets(10, 10, 0, 0);
		getContentPane().add(cfgPath, gridBagConstraints);

		jLabel3.setText("Sumo Configuration File");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(10, 20, 0, 0);
		getContentPane().add(jLabel3, gridBagConstraints);

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
		gridBagConstraints.insets = new Insets(10, 0, 0, 20);
		getContentPane().add(openFile, gridBagConstraints);

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
		gridBagConstraints.insets = new Insets(20, 10, 20, 0);
		getContentPane().add(cancelButton, gridBagConstraints);

		OKButton.setText("OK");
		OKButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				OKButtonActionPerformed(evt,filename,osenv);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(20, 20, 20, 0);
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

	private void OKButtonActionPerformed(ActionEvent evt, String sumoPath, int envos) {
		String line;		
		String cmd;
		
		/*Need to do OS environment check*/
		if (envos == 0){
			//cmd = "cmd /c " + sumoPath + "sumo -v -c " + cfgPath.getText();
			cmd = "\"" + sumoPath + System.getProperty("file.separator") + "sumo.exe" + "\"";
			cmd += " -v -c ";
			cmd += "\"" + cfgPath.getText() + "\"";			
		}
		else {
			cmd = "sumo -v -c " + cfgPath.getText();
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

	private void openFileActionPerformed(ActionEvent evt) {
		String extension = ".sumo.cfg";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showOpenDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION)
		cfgPath.setText(chooser.getSelectedFile().getPath());
	}

}
