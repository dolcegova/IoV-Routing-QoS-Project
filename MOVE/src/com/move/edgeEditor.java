package com.move;
import javax.swing.*;
import javax.swing.table.*;

import java.util.*;
import java.awt.*;
import java.awt.event.*;
import com.move.commonData.*;

public class edgeEditor extends JFrame {

	private String filename = "untitled";
	private EdgeDynamicTableModel mytable = new EdgeDynamicTableModel();
	private JTable table;
	static JFileChooser chooser;

	private static int sdformat = 0;
	private static int propformat = 0;

	private JButton addEdgeButton;
	private JButton removeEdgeButton;

	private JButton automaticID;
	private JButton priorityDefButton;
	private JButton speedDefButton;
	private JButton nolanesDefButton;

	private JTextField nolanesDef;
	private JTextField priorityDef;
	private JTextField speedDef;

	private ButtonGroup buttonGroup1;
	private ButtonGroup buttonGroup2;

	private JMenuBar jMenuBar;
	private JMenu fileMenu;
	private JMenu helpMenu;
	private JMenuItem quickHelpMenu;
	private JMenuItem quitMenu;
	private JMenuItem newMenu;
	private JMenuItem saveAsMenu;
	private JMenuItem saveMenu;
	private JMenuItem openMenu;

	private JLabel jLabel1;
	private JLabel jLabel3;
	private JLabel jLabel5;
	private JLabel jLabel6;
	private JLabel jLabel7;

	private JScrollPane jScrollPane1;
	private JSeparator jSeparator1;
	private JTextField lengthDef;

	private JRadioButton ownDefinition;
	private JRadioButton typeDefOption;

	public edgeEditor() {
		initComponents();
	}

	private void initComponents() {
		GridBagConstraints gridBagConstraints;

		buttonGroup1 = new ButtonGroup();
		buttonGroup2 = new ButtonGroup();
		jLabel1 = new JLabel();
		automaticID = new JButton();
		addEdgeButton = new JButton();
		ownDefinition = new JRadioButton();
		typeDefOption = new JRadioButton();
		jLabel3 = new JLabel();
		jLabel5 = new JLabel();
		jLabel6 = new JLabel();
		removeEdgeButton = new JButton();
		nolanesDef = new JTextField();
		speedDef = new JTextField();
		priorityDef = new JTextField();
		lengthDef = new JTextField();
		nolanesDefButton = new JButton();
		jLabel7 = new JLabel();
		speedDefButton = new JButton();
		priorityDefButton = new JButton();
		jScrollPane1 = new JScrollPane();
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

		if (chooser == null) {
			chooser = new JFileChooser("Open File");
		}

		getContentPane().setLayout(new GridBagLayout());
		setTitle("Roads Editor");

		jLabel1.setFont(new Font("Dialog", 1, 18));
		jLabel1.setText("Roads Editor");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 7;
		gridBagConstraints.ipadx = 8;
		gridBagConstraints.insets = new Insets(10, 10, 0, 0);
		getContentPane().add(jLabel1, gridBagConstraints);

		automaticID.setText("Automatic ID");
		//automaticID.setPreferredSize(new Dimension(95,23));
		automaticID.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				automaticIDActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.ipadx = 4;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(3, 20, 0, 0);
		getContentPane().add(automaticID, gridBagConstraints);

		addEdgeButton.setText("Add Edge");
		//addEdgeButton.setPreferredSize(new Dimension(79,23));
		addEdgeButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				addEdgeButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 21;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 22;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(10, 143, 0, 0);
		getContentPane().add(addEdgeButton, gridBagConstraints);

		buttonGroup2.add(ownDefinition);
		ownDefinition.setText("own definitions");
		ownDefinition.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				ownDefinitionActionPerformed(evt);
			}
		});
		ownDefinition.setSelected(true);
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.insets = new Insets(3, 40, 0, 0);
		getContentPane().add(ownDefinition, gridBagConstraints);

		buttonGroup2.add(typeDefOption);
		typeDefOption.setText("definitions from types file");
		typeDefOption.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				typeDefOptionActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.gridwidth = 8;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.insets = new Insets(3, 3, 0, 0);
		getContentPane().add(typeDefOption, gridBagConstraints);

		jLabel3.setText("Descriptions of roads and attributes");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.insets = new Insets(10, 40, 0, 0);
		getContentPane().add(jLabel3, gridBagConstraints);

		jLabel5.setText("Assign automatic edge IDs");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 6;
		gridBagConstraints.insets = new Insets(10, 20, 0, 0);
		getContentPane().add(jLabel5, gridBagConstraints);

		jLabel6.setText("*Using types will ignore nolanes, speed, priority, and length");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.gridwidth = 12;
		gridBagConstraints.gridheight = 3;
		gridBagConstraints.insets = new Insets(3, 40, 0, 0);
		getContentPane().add(jLabel6, gridBagConstraints);

		removeEdgeButton.setText("Remove Edge");
		//removeEdgeButton.setPreferredSize(new Dimension(79,23));
		removeEdgeButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				removeEdgeButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 21;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 1;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(3, 143, 0, 0);
		getContentPane().add(removeEdgeButton, gridBagConstraints);

		nolanesDef.setText("2");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.gridwidth = 7;
		gridBagConstraints.ipadx = 69;
		gridBagConstraints.insets = new Insets(6, 0, 0, 0);
		getContentPane().add(nolanesDef, gridBagConstraints);

		speedDef.setText("40");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 13;
		gridBagConstraints.gridwidth = 7;
		gridBagConstraints.ipadx = 69;
		getContentPane().add(speedDef, gridBagConstraints);

		priorityDef.setText("75");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 69;
		gridBagConstraints.insets = new Insets(6, 0, 0, 0);
		getContentPane().add(priorityDef, gridBagConstraints);

//		lengthDef.setText("100");
//		gridBagConstraints = new GridBagConstraints();
//		gridBagConstraints.gridx = 10;
//		gridBagConstraints.gridy = 13;
//		gridBagConstraints.gridwidth = 3;
//		gridBagConstraints.ipadx = 69;
//		getContentPane().add(lengthDef, gridBagConstraints);

		nolanesDefButton.setText("nolanes");
		//nolanesDefButton.setPreferredSize(new Dimension(69,23));
		nolanesDefButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				nolanesDefButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 21;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.insets = new Insets(6, 20, 0, 0);
		getContentPane().add(nolanesDefButton, gridBagConstraints);

		jLabel7.setText("Set Defaults");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.gridheight = 4;
		gridBagConstraints.insets = new Insets(7, 20, 0, 0);
		getContentPane().add(jLabel7, gridBagConstraints);

		speedDefButton.setText("speed");
		//speedDefButton.setPreferredSize(new Dimension(69,23));
		speedDefButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				speedDefButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 13;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 31;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.insets = new Insets(0, 20, 14, 0);
		getContentPane().add(speedDefButton, gridBagConstraints);

		priorityDefButton.setText("priority");
		//priorityDefButton.setPreferredSize(new Dimension(69,23));
		priorityDefButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				priorityDefButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 23;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.insets = new Insets(6, 30, 0, 0);
		getContentPane().add(priorityDefButton, gridBagConstraints);

		jScrollPane1.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		table = new JTable(mytable);
		jScrollPane1.setViewportView(table);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 24;
		gridBagConstraints.fill = GridBagConstraints.BOTH;
		gridBagConstraints.ipadx = 400; //727;
		gridBagConstraints.ipady = -150; //233;
		gridBagConstraints.insets = new Insets(16, 20, 0, 23);
		getContentPane().add(jScrollPane1, gridBagConstraints);

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
				try {
					openMenuActionPerformed(evt);
				}
				catch (Exception e) {
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
	class EdgeDynamicTableModel extends AbstractTableModel {
		//private String[] columnNames = {"ID","From Node","To Node","Type", "No Lanes", "Speed", "Priority", "Length"};
		private String[] columnNames = {"ID", "From Node", "To Node", "Type", "No Lanes", "Speed", "Priority"};
		Vector vectorTableData = new Vector(10, 2);

		public int getColumnCount() {
			return 7;
		}

		public int getRowCount() {
			return vectorTableData.size();
		}

		public String getColumnName(int col) {
			return columnNames[col];
		}

		public Object getValueAt(int row, int col) {
			try {
				Vector rowData = (Vector) vectorTableData.elementAt(row);
				if (rowData != null) {
					return rowData.elementAt(col);
				}
				return null;
			} catch (Exception e) {
				return null;
			}
		}

		public Class getColumnClass(int c) {
			return getValueAt(0, c).getClass();
		}

		public boolean isCellEditable(int row, int col) {
			return true;
		}

		public void setValueAt(Object value, int row, int col) {
			((Vector) vectorTableData.elementAt(row)).setElementAt(value, col);
			fireTableCellUpdated(row, col);
		}

		public void addRow(Vector someVector) {
			vectorTableData.addElement(someVector);
			fireTableRowsInserted(vectorTableData.size() - 1, vectorTableData.size() - 1);
		}

		public void removeRow(int row) {
			vectorTableData.removeElementAt(row);
			fireTableRowsDeleted(vectorTableData.size(), vectorTableData.size());
		}
	}

	private void ownDefinitionActionPerformed(ActionEvent evt) {
		propformat = 0;
	}

	private void typeDefOptionActionPerformed(ActionEvent evt) {
		propformat = 1;
	}

	private void automaticIDActionPerformed(ActionEvent evt) {
		for (int i = 0; i < mytable.getRowCount(); i++)
			mytable.setValueAt(new String("edge" + i), i, 0);
	}

	private void lengthDefButtonActionPerformed(ActionEvent evt) {
		for (int i = 0; i < mytable.getRowCount(); i++)
			mytable.setValueAt(lengthDef.getText(), i, 7);
	}

	private void speedDefButtonActionPerformed(ActionEvent evt) {
		for (int i = 0; i < mytable.getRowCount(); i++)
			mytable.setValueAt(speedDef.getText(), i, 5);
	}

	private void nolanesDefButtonActionPerformed(ActionEvent evt) {
		for (int i = 0; i < mytable.getRowCount(); i++)
			mytable.setValueAt(nolanesDef.getText(), i, 4);
	}

	private void priorityDefButtonActionPerformed(ActionEvent evt) {
		for (int i = 0; i < mytable.getRowCount(); i++)
			mytable.setValueAt(priorityDef.getText(), i, 6);
	}

	private void newMenuActionPerformed(ActionEvent evt) {
		while (mytable.getRowCount() != 0)
			mytable.removeRow(mytable.getRowCount() - 1);
		filename = "untitled";
	}

	private void openMenuActionPerformed(ActionEvent evt) throws Exception {
		while (mytable.getRowCount() != 0)
			mytable.removeRow(mytable.getRowCount() - 1);

		String extension = ".edg.xml";
		chooser.setFileFilter(new JaveFileFilter(extension));
		int returnVal = chooser.showOpenDialog(this);
		if (returnVal == JFileChooser.APPROVE_OPTION)
			filename = chooser.getSelectedFile().getPath();

		Load l = new Load(filename);
		l.initEdges();
		l.readfile();
		edgeData test = l.getEdgeData();
		ArrayList list = test.returnData();

		for (int i = 0; i < list.size(); i++) {
			edgeParameter e = (edgeParameter) list.get(i);
			Vector newRow = new Vector();
			int type = e.getTypeFlag();

			if (type == 1) {
				newRow.addElement(e.getEdgeID()); //ID
				newRow.addElement(e.getFromnode()); //FromNode
				newRow.addElement(e.getTonode()); //ToNode
			}

			if (e.typeIDflag == 1) {
				newRow.addElement(e.getTypeID()); //Types
				newRow.addElement(""); //Nolanes
				newRow.addElement(""); //Speed
				newRow.addElement(""); //Priority
				newRow.addElement(""); //Length
			}
			else {
				newRow.addElement("");//Types
				newRow.addElement(e.getNolanes()); //Nolanes
				newRow.addElement(e.getSpeed()); //Speed
				newRow.addElement(e.getPriority()); //Priority
				newRow.addElement(e.getLength()); //Length
			}
			mytable.addRow(newRow);
		}
	}

	private void saveFile() {
		edgeData m = new edgeData();
		edgeParameter n;
		//String[] temp = {"","","","","","","",""};
		String[] temp = {"", "", "", "", "", "", ""};

		for (int i = 0; i < mytable.getRowCount(); i++) {
			for (int j = 0; j < mytable.getColumnCount(); j++) {
				temp[j] = (String) mytable.getValueAt(i, j);
			}
			n = new edgeParameter(temp[0], temp[1], temp[2]);
			if (propformat == 0) {
				n.setNolanes(temp[4]);
				n.setSpeed(temp[5]);
				n.setPriority(temp[6]);
				//n.setLength(temp[7]);
			}
			else if (propformat == 1) {
				n.setType(temp[3]);
			}
			m.addEdges(n);
		}
		construct edgeconstruction = new construct(m, filename);
	}

	private void saveMenuActionPerformed(ActionEvent evt) {
		if (filename == "untitled") {
			String extension = ".edg.xml";
			chooser.setFileFilter(new JaveFileFilter(extension));
			int returnVal = chooser.showSaveDialog(this);
			if (returnVal == JFileChooser.APPROVE_OPTION) {
				filename = chooser.getSelectedFile().getPath();
				if (filename.indexOf(extension) >= 0)
					;
				else
					filename = chooser.getSelectedFile().getPath() + extension;
				saveFile();
			}
		}
		else {
			saveFile();
		}
	}

	private void saveAsMenuActionPerformed(ActionEvent evt) {
		String extension = ".edg.xml";
		chooser.setFileFilter(new JaveFileFilter(extension));
		int returnVal = chooser.showSaveDialog(this);
		if (returnVal == JFileChooser.APPROVE_OPTION) {
			filename = chooser.getSelectedFile().getPath();
			if (filename.indexOf(extension) >= 0)
				;
			else
				filename = chooser.getSelectedFile().getPath() + extension;
			saveFile();
		}
	}

	private void quitMenuActionPerformed(ActionEvent evt) {
		this.dispose();
	}

	private void quickHelpMenuActionPerformed(ActionEvent evt) {
		JOptionPane.showMessageDialog(this, "Not yet available");
	}

	private void addEdgeButtonActionPerformed(ActionEvent evt) {
		Vector newRow = new Vector();
		newRow.addElement(new String("")); //ID
		newRow.addElement(new String("")); //FromNode
		newRow.addElement(new String("")); //ToNode
		newRow.addElement(new String("")); //Types
		newRow.addElement(new String("")); //Nolanes
		newRow.addElement(new String("")); //Speed
		newRow.addElement(new String("")); //Priority
		//newRow.addElement(new String("")); //Length
		mytable.addRow(newRow);
	}

	private void removeEdgeButtonActionPerformed(ActionEvent evt) {
		if (mytable.getRowCount() > 0)
			mytable.removeRow(mytable.getRowCount() - 1);
	}

	public static void main(String args[]) {
		new edgeEditor();
	}
}
