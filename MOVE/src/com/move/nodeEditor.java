package com.move;
import javax.swing.table.AbstractTableModel;
import javax.swing.*;

import java.util.*;
import java.awt.*;
import java.awt.event.*;

import javax.swing.JFileChooser;

import com.move.commonData.nodeData;
import com.move.commonData.nodeParameter;

/*
Creator: Feliz Kristianto Karnadi and Zhi Hai Mo
UNSW - Sydney
*/

public class nodeEditor extends JFrame {

	private String filename = "untitled";
	private NodeDynamicTableModel mytable = new NodeDynamicTableModel();
	private JTable table;
	static JFileChooser chooser;

	private JButton addNodesButton;
	private JButton autoIDButton;
	private JMenu fileMenu;
	private JMenu helpMenu;
	private JLabel jLabel1;
	private JLabel jLabel2;
	private JMenuBar jMenuBar;
	private JScrollPane jScrollPane1;
	private JSeparator jSeparator1;
	private JMenuItem newMenu;

	private JMenuItem openMenu;
	private JMenuItem quickhelpmenu;
	private JMenuItem quitMenu;
	private JButton removeNodesButton;
	private JMenuItem saveAsMenu;
	private JMenuItem saveMenu;

	public nodeEditor() {
		initComponents();
	}

	private void initComponents() {
		GridBagConstraints gridBagConstraints;

		jLabel1 = new JLabel();
		removeNodesButton = new JButton();
		autoIDButton = new JButton();
		addNodesButton = new JButton();
		jScrollPane1 = new JScrollPane();

		jLabel2 = new JLabel();
		jMenuBar = new JMenuBar();
		fileMenu = new JMenu();
		newMenu = new JMenuItem();
		openMenu = new JMenuItem();
		saveMenu = new JMenuItem();
		saveAsMenu = new JMenuItem();
		jSeparator1 = new JSeparator();
		quitMenu = new JMenuItem();
		helpMenu = new JMenu();
		quickhelpmenu = new JMenuItem();

		if(chooser == null) {
			chooser = new JFileChooser("Node XML File");
		}

		getContentPane().setLayout(new GridBagLayout());
		setTitle("Map Node Editor");
		jLabel1.setFont(new Font("Dialog", 1, 18));
		jLabel1.setText("Map Nodes Editor");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.ipadx = 6;
		gridBagConstraints.insets = new Insets(10, 38, 0, 0);
		getContentPane().add(jLabel1, gridBagConstraints);

		removeNodesButton.setText("Remove Node");
		//removeNodesButton.setPreferredSize(new Dimension(79,23));
		removeNodesButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				removeNodesButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 11;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(3, 80, 27, 24);
		getContentPane().add(removeNodesButton, gridBagConstraints);

		autoIDButton.setText("Automatic ID");
		//autoIDButton.setPreferredSize(new Dimension(95,23));
		autoIDButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				autoIDButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 14;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(3, 20, 27, 0);
		getContentPane().add(autoIDButton, gridBagConstraints);

		addNodesButton.setText("Add Node");
		//addNodesButton.setPreferredSize(new Dimension(79,23));
		addNodesButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				addNodesButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 31;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(20, 80, 0, 24);
		getContentPane().add(addNodesButton, gridBagConstraints);

		jScrollPane1.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		table = new JTable(mytable);
		jScrollPane1.setViewportView(table);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.fill = GridBagConstraints.BOTH;
		gridBagConstraints.ipadx = 100;
		gridBagConstraints.ipady = -150;
		gridBagConstraints.insets = new Insets(16, 20, 0, 24);
		getContentPane().add(jScrollPane1, gridBagConstraints);

		jLabel2.setText("Set the node ID automatically");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(20, 20, 0, 0);
		getContentPane().add(jLabel2, gridBagConstraints);

		fileMenu.setText("File");
		newMenu.setText("New");
		newMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				newMenuActionPerformed(evt);
			}
		});

		fileMenu.add(newMenu);

		openMenu.setText("Open");
		openMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				try{
					openMenuActionPerformed(evt);
				}
				catch (Exception e){
				}
			}
		});

		fileMenu.add(openMenu);

		saveMenu.setText("Save");
		saveMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				saveMenuActionPerformed(evt);
			}
		});

		fileMenu.add(saveMenu);

		saveAsMenu.setText("Save As");
		saveAsMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				saveAsMenuActionPerformed(evt);
			}
		});

		fileMenu.add(saveAsMenu);

		fileMenu.add(jSeparator1);

		quitMenu.setText("Quit");
		quitMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				quitMenuActionPerformed(evt);
			}
		});

		fileMenu.add(quitMenu);

		jMenuBar.add(fileMenu);

		helpMenu.setText("Help");
		quickhelpmenu.setText("Quick Help");
		quickhelpmenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				quickhelpmenuActionPerformed(evt);
			}
		});

		helpMenu.add(quickhelpmenu);
		/*Quick help is disabled for now*/
		//jMenuBar.add(helpMenu);
		setJMenuBar(jMenuBar);
		pack();
		setVisible(true);
	}
	class NodeDynamicTableModel extends AbstractTableModel {
		private String[] columnNames = {"ID","X","Y","Traffic Light"};
		Vector vectorTableData = new Vector(10,2);

		public int getColumnCount() {
			return 4;
		}

		public int getRowCount() {
			return vectorTableData.size();
		}

		public String getColumnName(int col) {
			return columnNames[col];
		}

		public Object getValueAt(int row, int col) {
			try {				
				//System.out.println(vectorTableData.elementAt(row));
				Vector rowData = (Vector)vectorTableData.elementAt(row);
				if (rowData != null) {
					return rowData.elementAt(col);
				}
				return null;
			} catch (Exception e) {
				System.out.println("Catch miss!!");
				return null;}
			}

			public Class getColumnClass(int c) {
				return getValueAt(0, c).getClass();
			}

			public boolean isCellEditable(int row, int col) {
				return true;
			}

			public void setValueAt(Object value, int row, int col) {
				((Vector)vectorTableData.elementAt(row)).setElementAt(value, col);
				fireTableCellUpdated(row, col);
			}

			public void addRow(Vector someVector) {
				vectorTableData.addElement(someVector);
				fireTableRowsInserted(vectorTableData.size() - 1, vectorTableData.size() -1);
			}

			public void removeRow(int row) {
				vectorTableData.removeElementAt(row);
				fireTableRowsDeleted(vectorTableData.size(), vectorTableData.size());
			}
		}

		private void addNodesButtonActionPerformed(ActionEvent evt) {
			Vector newRow = new Vector();
			newRow.addElement(new String(""));
			newRow.addElement(new String("0.0"));
			newRow.addElement(new String("0.0"));
			newRow.addElement(new Boolean(false));
			mytable.addRow(newRow);
		}

		private void removeNodesButtonActionPerformed(ActionEvent evt) {
			if (mytable.getRowCount() != 0)
			mytable.removeRow(mytable.getRowCount()-1);
		}

		private void autoIDButtonActionPerformed(ActionEvent evt) {
			for (int i= 0; i<mytable.getRowCount(); i++)
			mytable.setValueAt(new String("node"+i),i,0);

		}

		private void newMenuActionPerformed(ActionEvent evt) {
			while (mytable.getRowCount() != 0)
			mytable.removeRow(mytable.getRowCount()-1);
			filename = "untitled";
		}

		private void openMenuActionPerformed(ActionEvent evt) throws Exception{
			while (mytable.getRowCount() != 0)
			mytable.removeRow(mytable.getRowCount()-1);

			String extension = ".nod.xml";
			chooser.setFileFilter( new JaveFileFilter(extension) );
			int returnVal = chooser.showOpenDialog(this);
			if(returnVal == JFileChooser.APPROVE_OPTION)
			filename=chooser.getSelectedFile().getPath();

			Load l = new Load(filename);
			l.initNodes();
			l.readfile();
			nodeData test = l.getNodeData();
			ArrayList list = test.returnData();

			for(int i =0;i<list.size();i++){
				nodeParameter e = (nodeParameter) list.get(i);
				Vector newRow = new Vector();
				newRow.addElement(e.getID());
				newRow.addElement(e.getX());
				newRow.addElement(e.getY());
				String temp = e.getType();
				if (temp.equals("priority"))
				newRow.addElement(new Boolean(false));
				else if (temp.equals("traffic_light"))
				newRow.addElement(new Boolean(true));
				mytable.addRow(newRow);
			}
		}

		private void saveFile() {
			nodeData m = new nodeData();
			nodeParameter n;
			String[] temp = {"","","",""};
			for(int i=0; i<mytable.getRowCount(); i++){
				for (int j=0; j<mytable.getColumnCount(); j++) {
					if (j == 3){
						if ((Boolean)mytable.getValueAt(i,j) == Boolean.TRUE)
							temp[j]="traffic_light";
						else
							temp[j]="priority";
					}
					else{
						temp[j]=(String)mytable.getValueAt(i,j);
					}
				}
				n = new nodeParameter(temp[0],temp[1],temp[2],temp[3]);
			
				m.addNodes(n);
			}
			construct nodeconstruction = new construct(m,filename);					
		}
		
		private void saveMenuActionPerformed(ActionEvent evt) {
			if (filename == "untitled"){
				String extension = ".nod.xml";
				chooser.setFileFilter( new JaveFileFilter(extension) );
				int returnVal = chooser.showSaveDialog(this);			
				if(returnVal == JFileChooser.APPROVE_OPTION) {					
					filename=chooser.getSelectedFile().getPath();
					if ( filename.indexOf(extension) >= 0 )
						;
					else 
						filename=chooser.getSelectedFile().getPath() + extension;				
					saveFile();
				}
			}
			else {
				saveFile();
			}			
		}

		private void saveAsMenuActionPerformed(ActionEvent evt) {
			String extension = ".nod.xml";
			chooser.setFileFilter( new JaveFileFilter(extension) );
			int returnVal = chooser.showSaveDialog(null);			
			if(returnVal == JFileChooser.APPROVE_OPTION) {
				filename=chooser.getSelectedFile().getPath();
				if ( filename.indexOf(extension) >= 0 )
					;
				else 
					filename=chooser.getSelectedFile().getPath() + extension;				
				saveFile();
			}
		}

		private void quickhelpmenuActionPerformed(ActionEvent evt) {
			JOptionPane.showMessageDialog(this, "Not yet available");
		}

		private void quitMenuActionPerformed(ActionEvent evt) {
			this.dispose();
		}

		public static void main(String args[]) {
			new nodeEditor();
		}
	}
