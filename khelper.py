#!/usr/bin/env python
# encoding: utf-8
from __future__ import division
from __future__ import print_function
"""
# @file khelper.py
# @brief Extract file access data after build, modify CDT project configuration
# (.cproject) accordingly
# @copyright Copyright (C) 2016, Elphel.inc.
# @param <b>License</b>
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http:#www.gnu.org/licenses/>.

@author:     Andrey Filippov
@license:    GPLv3.0+
@contact:    andrey@elphel.coml
@deffield    updated: Updated
"""
__author__ = "Andrey Filippov"
__copyright__ = "Copyright 2016, Elphel, Inc."
__license__ = "GPL"
__version__ = "3.0+"
__maintainer__ = "Andrey Filippov"
__email__ = "andrey@elphel.com"
__status__ = "Development"

import sys
import os
import time
import xml.etree.ElementTree as ET
'''
TODO:Automate, find out why separate touch commands are needed
Run this program twice:
1-st run ./khelper.py linux -1
and save shown timestamp
Then run (some mystery here)
touch src/drivers/ata/ahci_elphel.c
touch src/drivers/elphel/sensor_common.c
Wait 5 seconds and run (in a different console with appropriate sourcing)
bitbake linux-xlnx -c compile -f 
Then again
./khelper.py linux <timestamp_from_the_first_run>
If somethong went wrong you will need to resore .cproject from eclipse_project_setup directory
'''
def file_tree(flist): # Each file in list is a file, no directories
    ftree={}
    for p in flist:
        node = ftree
        seg_list=p.split(os.sep)
        last_i=len(seg_list)-1
        for i,segm in enumerate(seg_list):
            if not segm in node:
                if i == last_i:
                    node[segm] = None
                else:
                    node[segm] = {}
            node=node[segm]        
                        
    return ftree                   

def exclude_list(ftree, flist):
    mark = "*" # no file/dir name can be "*" 
    def list_tree_recursive(root):
        rslt = []
        if not mark in root:
            return [[""]] # convert to trailing "/" for directories
        for n in root:
            if not n == mark:
                if root[n] is None:
                    rslt.append([n])
                else:
                    
                    for l in list_tree_recursive(root[n]):
                        rslt.append([n]+l)
        return rslt
                        
    ftree[mark]=None # mark top level dir                                         
    for p in flist:
        node = ftree
        for segm in p.split(os.sep)[:-1]:
            node=node[segm]
            node[mark]=None # [mark] means used in flist
        del node[p.split(os.sep)[-1]]
    #print (ftree)
#    for k in ftree:
#       print(k) 
    #Now prune unused directories
    #prune_recursive(ftree) # (assuming root is used)
    # now create list
    files_list_list = list_tree_recursive(ftree)
#    print (files_list_list)
    #converrt to file paths
    pl = []
    for l in files_list_list:
        pl.append(os.path.join(*(l[1:])))
    pl = sorted (pl)
    return pl     
    
            
def proc_tree():
    DEBUG = True
    extensions =    [".h",".c",".cpp"]
    exclude_start = ["linux"+os.sep+"scripts"+os.sep,"linux"+os.sep+"source"+os.sep+"scripts"+os.sep]
    delta_t = 3 # seconds
    try:
        root_path = sys.argv[1]
    except:
        print ("Calling %s <root directory path> [timestamp]"%(os.path.basename(sys.argv[0])))
    try:
        start_time = float(sys.argv[2])
    except:
        start_time = 0.0

    touch_files= start_time < 0.0   
    print ("root_path = %s"%(root_path))
#    root_path = "/home/eyesis/git/poky/linux-elphel/linux/"    
    lstFiles = []
    # Append  files to a list
    for path, _, files in os.walk(root_path, followlinks = True):
        for f in files:
            for ext in extensions:
                if f.endswith(ext):
                    lstFiles.append(os.path.join(path, f))
                    break
     
    all_tree= file_tree(sorted(lstFiles))
    include_lst=[]        
    lst_a = []
    latest_at=0
    for p in lstFiles:
        if touch_files:
            if  os.path.islink(p):
                os.utime(os.path.realpath(p), None)
            else:    
                os.utime(p, None)
        else:        
#           at = time.ctime(os.stat(p).st_atime)
            at = os.stat(p).st_atime
            l = None
            if  os.path.islink(p):
                l = os.path.realpath(p)
                at = os.stat(l).st_atime
            latest_at = max((latest_at,at))    
            if at > (start_time + delta_t):
                #Scripts/lexers result in problems
                exclude=False
                for exStr in exclude_start:
                    if p.startswith(exStr):
                        exclude=True
                        break
                if exclude:
                    break        
                #exclude_start
                lst_a.append([p,at,l])
                include_lst.append(p)
    if touch_files:
        print (len(lstFiles), "last time = ", time.time())
        return    
            
    excluding = exclude_list(all_tree, include_lst)        
#    print (all_tree)
#    print (sorted(include_lst))
#    print ("|".join(excluding))
    if DEBUG:        
        with open("all_sources.lst","w" ) as f:
            for p in sorted(lstFiles):
                at = os.stat(p).st_atime
                lnk=""
                if  os.path.islink(p):
                    at = os.stat(os.path.realpath(p)).st_atime
                    lnk = os.path.realpath(p)
                print (p,at,lnk, file=f)
        with open("excluding.lst","w" ) as f:
            for p in excluding:
                print (p, file=f)
#    include_tree= file_tree(sorted(include_lst))
#    print(include_tree)
    root_dir=include_lst[0].split(os.sep)[0]
    print ("root_dir=",root_dir)
    
    xml= ET.parse(".cproject")
    root=xml.getroot()
#    for child in root:
#        print(child.tag, child.attrib)

    for child in root.iter('sourceEntries'):
        for gchild in child:
            print(gchild.tag)
    
    for child in root.iter('sourceEntries'):
        for gchild in child:
            if gchild.tag == 'entry':
                attr = gchild.attrib
                try:
                    if (attr['kind'] ==  'sourcePath') and (attr['name'] ==  root_dir):
                        child.remove (gchild)
                        print ("Removed existing entry ",gchild.tag)
                        break
                except:
                    print ("error matching attributes for ",gchild.tag)
                    pass
        break #after first   'sourceEntries' - should be just one?      
    ET.SubElement(child, 'entry', {"flags":"VALUE_WORKSPACE_PATH", "kind":"sourcePath", "name":root_dir, "excluding":"|".join(excluding)})
                             
    for child in root.iter('sourceEntries'):
        for gchild in child:
            print(gchild.tag)
            
    oneliner= ET.tostring(root)
    #overwrites original .cproject, may change to somethong different
    with open(".cproject", "wr") as f:
        f.write("""<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<?fileVersion 4.0.0?>""")
        f.write(oneliner)
            
    print (len(lstFiles), len(lst_a), "last access time = ",latest_at)
                
if __name__ == '__main__':
    proc_tree()