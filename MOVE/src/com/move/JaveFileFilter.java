package com.move;
/*
 *  Author: Chien-Ming Chou
 *  2009-02-10 
 *  
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *  
 */


import javax.swing.filechooser.FileFilter;
import javax.swing.filechooser.FileView;

import java.io.File;

public class JaveFileFilter extends FileFilter {
    private String extension = "";
    
    /** Creates a new instance of TransFileFilter */
    public JaveFileFilter( String extension ){ 
        this.extension = extension;
    }
    public boolean accept(File file) {
        boolean accepted = false;
        String filename = file.getName();
        if( filename.endsWith( extension ) || file.isDirectory() || extension.trim().length() == 0 )
            accepted = true;
        return accepted;
    }
    
    public String getDescription() {
        return "*" + extension;
    }
}