#!/usr/bin/env python
# encoding: utf-8
from __future__ import division
from __future__ import print_function
import sys
import os
import time
import xml.etree.ElementTree as ET
'''
TODO: make 2 runs, first before
bitbake linux-xlnx
khelper.py linux -1
#get timestamp
#next 2 commands - I do not understand why their access timestamp is set during previous command and does not change during bitbake 
touch /home/eyesis/git/elphel393/linux-elphel/src/drivers/ata/ahci_elphel.c
touch /home/eyesis/git/elphel393/linux-elphel/src/drivers/elphel/sensor_common.c
bitbake linux-xlnx -c compile -f
khelper.py linux <timestamp_from_the_first_run>
             
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