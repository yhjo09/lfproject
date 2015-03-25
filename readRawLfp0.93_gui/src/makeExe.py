'''
Created on 2015. 2. 27.

@author: young
'''

from distutils.core import setup
import py2exe
import glob
# import numpy
 
excludes = ["pywin", "pywin.debugger", "pywin.debugger.dbgcon", "pywin.dialogs", "pywin.dialogs.list", "win32com.server"]
 
options = {
#            "bundle_files":1,
           "compressed":1,
           "excludes":excludes,
           "dll_excludes":["w9xpopen.exe", "MSVCP90.dll"]
}
 
setup(
      options={"py2exe":options},
      zipfile=None,
      windows=["readRawLfp.py"],
      data_files=None
)


# setup(console=["readRawLfp.py"])