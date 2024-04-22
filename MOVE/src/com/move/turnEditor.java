package com.move;
import javax.swing.*;

import java.util.*;

import javax.swing.table.*;

import com.move.commonData.TurnData;
import com.move.commonData.TurnEdge;
import com.move.commonData.TurnInterval;
import com.move.commonData.TurnParameter;

import java.awt.*;
import java.awt.event.*;

/*
Creator: Feliz Kristianto Karnadi and Zhi Hai Mo
UNSW - Sydney
*/

public class turnEditor extends JFrame {
	private String filename = "untitled";
	private TurnDynamicTableModel mytable = new TurnDynamicTableModel();
	private JTable table;
	static JFileChooser chooser;
	private static int saveflag = 0;

	private JButton addRowButton;
	private JTextField beginDef;
	private JButton beginDefButton;
	private JTextField endDef;
	private JButton endDefButton;
	private JMenu fileMenu;
	private JMenu helpMenu;
	private JLabel jLabel1;
	private JLabel jLabel2;
	private JMenuBar jMenuBar;
	private JPanel jPanel1;
	private JScrollPane jScrollPane1;
	private JSeparator jSeparator1;

	private JMenuItem newMenu;
	private JMenuItem openMenu;
	private JMenuItem quickHelpMenu;
	private JMenuItem quitMenu;
	private JButton removeRowButton;
	private JMenuItem saveAsMenu;
	private JMenuItem saveMenu;
	
	private ButtonGroup buttonGroup1;
	
	public turnEditor() {
		initComponents();
	}

	private void initComponents() {
		GridBagConstraints gridBagConstraints;

		jPanel1 = new JPanel();
		jScrollPane1 = new JScrollPane();

		jLabel1 = new JLabel();
		beginDefButton = new JButton();
		beginDef = new JTextField();
		endDef = new JTextField();
		endDefButton = new JButton();
		removeRowButton = new JButton();
		addRowButton = new JButton();
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
		quickHelpMenu = new JMenuItem();
		
		buttonGroup1 = new ButtonGroup();

		if(chooser == null) {
			chooser = new JFileChooser("Turn Definitions XML File");
		}

		jPanel1.setLayout(new GridBagLayout());
		setTitle ("Junction Turning Probability Editor");
		table = new JTable(mytable);

		jScrollPane1.setViewportView(table);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.ipadx = 37;
		gridBagConstraints.ipady = -209;
		gridBagConstraints.insets = new Insets(14, 10, 0, 19);
		jPanel1.add(jScrollPane1, gridBagConstraints);

		jLabel1.setText("Set Defaults");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.insets = new Insets(20, 20, 4, 21);
		jPanel1.add(jLabel1, gridBagConstraints);

		beginDefButton.setText("begin");
		//beginDefButton.setPreferredSize(new Dimension(69,23));
		beginDefButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				beginDefButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = 25;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		jPanel1.add(beginDefButton, gridBagConstraints);

		beginDef.setText("0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = 59;
		jPanel1.add(beginDef, gridBagConstraints);

		endDef.setText("1000");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 59;
		gridBagConstraints.insets = new Insets(0, 0, 50, 0);
		jPanel1.add(endDef, gridBagConstraints);

		endDefButton.setText("end");
		//endDefButton.setPreferredSize(new Dimension(69,23));
		endDefButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				endDefButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 35;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(0, 20, 50, 0);
		jPanel1.add(endDefButton, gridBagConstraints);

		removeRowButton.setText("Remove Turn");
		//removeRowButton.setPreferredSize(new Dimension(79,23));
		removeRowButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				removeRowButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 12;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(0, 180, 50, 0);
		jPanel1.add(removeRowButton, gridBagConstraints);

		addRowButton.setText("Add Turn");
		//addRowButton.setPreferredSize(new Dimension(79,23));
		addRowButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				addRowButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 36;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(30, 180, 10, 0);
		jPanel1.add(addRowButton, gridBagConstraints);

		jLabel2.setText("Junctions Turning Ratios Editor");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.insets = new Insets(40, 10, 0, 293);
		jPanel1.add(jLabel2, gridBagConstraints);

		getContentPane().add(jPanel1, BorderLayout.CENTER);
		
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

	class TurnDynamicTableModel extends AbstractTableModel {
		private String[] columnNames = {"Begin","End","From Edge","To Edge","Percentage"};
		Vector vectorTableData = new Vector(10,2);

		public int getColumnCount() {
			return 5;
		}

		public int getRowCount() {
			return vectorTableData.size();
		}

		public String getColumnName(int col) {
			return columnNames[col];
		}

		public Object getValueAt(int row, int col) {
			try {
				Vector rowData = (Vector)vectorTableData.elementAt(row);
				if (rowData != null) {
					return rowData.elementAt(col);
				}
				return null;
			} catch (Exception e) {return null;}
			}

			public String getStringAt(int row, int col) {
				try {
					Vector rowData = (Vector)vectorTableData.elementAt(row);
					if (rowData != null) {
						return rowData.elementAt(col).toString();
					}
					return "";
				} catch (Exception e) {return "";}
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

			private void newMenuActionPerformed(ActionEvent evt) {
				while (mytable.getRowCount() != 0)
				mytable.removeRow(mytable.getRowCount()-1);
				filename = "untitled";
			}

			private void openMenuActionPerformed(ActionEvent evt) throws Exception{
				while (mytable.getRowCount() != 0)
				mytable.removeRow(mytable.getRowCount()-1);

				String extension = ".turn.xml";
				chooser.setFileFilter( new JaveFileFilter(extension) );
				int returnVal = chooser.showOpenDialog(this);
				if(returnVal == JFileChooser.APPROVE_OPTION)
				filename=chooser.getSelectedFile().getPath();

				Load l = new Load(filename);
				l.initTurns();
				l.readfile();
				TurnData test = l.getTurnData();
				ArrayList list = test.returnData();

				for(int i =0;i<list.size();i++){
					TurnInterval interval = (TurnInterval) list.get(i);

					ArrayList list1 = interval.returnData();
					for (int j = 0; j < list1.size(); j++){
						TurnEdge edge = (TurnEdge) list1.get(j);

						ArrayList list2 = edge.returnData();
						for (int k = 0; k < list2.size(); k++){
							TurnParameter p = (TurnParameter) list2.get(k);
							Vector newRow = new Vector();
							newRow.addElement(interval.getBegin());
							newRow.addElement(interval.getEnd());
							newRow.addElement(edge.getID());
							newRow.addElement(p.getID());
							newRow.addElement(p.getPerc());
							mytable.addRow(newRow);
						}
					}
				}
			}
			
			private void saveFile() {
				TurnData m = new TurnData();
				TurnParameter n = new TurnParameter("","");
				TurnEdge o = new TurnEdge("");;
				TurnInterval p = new TurnInterval("","");

				String[] temp = {"","","","",""};
				for(int i = 0; i<mytable.getRowCount(); i++){
					for (int j=0; j<mytable.getColumnCount(); j++) {
						temp[j]=(String)mytable.getValueAt(i, j);
					}

					if (mytable.getRowCount() == 1){
						n = new TurnParameter(temp[3],temp[4]);
						o = new TurnEdge(temp[2]);
						p = new TurnInterval(temp[0],temp[1]);
						o.addParameters(n);
						p.addEdge(o);
						m.addIntervals(p);
						construct turnconstruction = new construct(m,filename);
						return;
					}

					if (i == 0){
						n = new TurnParameter(temp[3],temp[4]);
						o = new TurnEdge(temp[2]);
						p = new TurnInterval(temp[0],temp[1]);
						o.addParameters(n);
						p.addEdge(o);
						m.addIntervals(p);
					}

					else {
						if (temp[0].equals(mytable.getValueAt(i-1,0)) &&
						temp[1].equals(mytable.getValueAt(i-1,1)) &&
						temp[2].equals(mytable.getValueAt(i-1,2))){
							n = new TurnParameter(temp[3],temp[4]);
							o.addParameters(n);
							if (i == mytable.getRowCount() - 1){
								construct turnconstruction = new construct(m,filename);
								return;
							}
							continue;
						}

						if (temp[0].equals(mytable.getValueAt(i-1,0))  &&
						temp[1].equals(mytable.getValueAt(i-1,1))){
							n = new TurnParameter(temp[3],temp[4]);
							o = new TurnEdge(temp[2]);
							o.addParameters(n);
							p.addEdge(o);
							if (i == mytable.getRowCount() - 1){
								construct turnconstruction = new construct(m,filename);
								return;
							}
							continue;
						}
						else {
							n = new TurnParameter(temp[3],temp[4]);
							o = new TurnEdge(temp[2]);
							p = new TurnInterval(temp[0],temp[1]);
							o.addParameters(n);
							p.addEdge(o);
							m.addIntervals(p);
							if (i == mytable.getRowCount() - 1){
								construct turnconstruction = new construct(m,filename);
								return;
							}
							continue;
						}
					}
				}
			}

			private void saveMenuActionPerformed(ActionEvent evt) {
				if (filename == "untitled"){
					String extension = ".turn.xml";
					chooser.setFileFilter( new JaveFileFilter(extension) );
					int returnVal = chooser.showSaveDialog(this);
					if(returnVal == JFileChooser.APPROVE_OPTION){
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
				String extension = ".turn.xml";
				chooser.setFileFilter( new JaveFileFilter(extension) );
				int returnVal = chooser.showSaveDialog(this);
				if(returnVal == JFileChooser.APPROVE_OPTION){
					filename=chooser.getSelectedFile().getPath();
					if ( filename.indexOf(extension) >= 0 )
						;
					else 
						filename=chooser.getSelectedFile().getPath() + extension;				
					saveFile();
				}				
			}

			private void quitMenuActionPerformed(ActionEvent evt) {
				this.dispose();
			}

			private void addRowButtonActionPerformed(ActionEvent evt) {
				Vector newRow = new Vector();
				newRow.addElement(new String("")); //Begin
				newRow.addElement(new String("")); //End
				newRow.addElement(new String("")); //From
				newRow.addElement(new String("")); //To
				newRow.addElement(new String("0")); //Percentage
				mytable.addRow(newRow);
			}

			private void removeRowButtonActionPerformed(ActionEvent evt) {
				if (mytable.getRowCount() != 0)
				mytable.removeRow(mytable.getRowCount()-1);
			}

			private void quickHelpMenuActionPerformed(ActionEvent evt) {
				JOptionPane.showMessageDialog(this, "Not yet available");
			}

			private void endDefButtonActionPerformed(ActionEvent evt) {
				for (int i = 0; i < mytable.getRowCount(); i++)
				mytable.setValueAt(endDef.getText(),i,1);
			}

			private void beginDefButtonActionPerformed(ActionEvent evt) {
				for (int i = 0; i < mytable.getRowCount(); i++)
				mytable.setValueAt(beginDef.getText(),i,0);
			}

			public static void main(String args[]) {
				new turnEditor();
			}			
		}
