# linux-elphel
This project handles kernel drivers and driver modification for Elphel NC393 seriees cameras.
The best way to install it is by running installation script in the top project -
https://github.com/Elphel/elphel393 as different parts of the camera software are inter-dependent,
and some of the required header files are generated from the Verilog code in
[x393 FPGA project](https://github.com/Elphel/x393).

This is Eclipse CDT project, and the [setup sctipt](https://github.com/Elphel/elphel393) copies
project configuration (.project, .cproject, .settings) from eclipse_project_setup subdirectory.
Eclipse should be started after Linux kernel is built with `bitbake linux-xlnx` at least once
so all the required files are already present. Then you may import *linux-elphel* as
'existing Git project' in Eclipse (you need either Eclipse for C/C++ developmentg or intall [CDT
plugin](https://eclipse.org/cdt/) separately.

Do not run `bitbake linux-xlnx -c clean` while the project is open in Eclipse - this command
will delete all he staged files (kernel sources) and Eclipse will reset all file filters, so
*.cproject* file will have to be restored from *eclipse_project_setup*. 

Source files in Yocto project are staged (downloaded and optionally patched), so `-c clean` re-downloads
files and erases any of your files or changes if they were made in the source directory tree.
*linux-elphel* project keeps all the modified or new files in a sparse copy of the Linux kernel tree (*src*
sub-directory) and the bitbake recipe (`bitbake linux-xlnx -c link`) adds symlinks to the actual
project files. When the full kernel is opened in Eclipse, the base kernel files are from the staging
area (so will be refreshed after `-c clean`) and the project files edits will stay safe in the *src*
subdirectory.

The khelper.py script is used to generate filters for the kernel by collecting information - which
files where accessed during `bitbake linux-xlnx -c compile -f`, results are injected into *.cproject*
configuratgion file, so any of the Linux kernel files that were not accessed during build will be marked
as excluded from build (crossed out) and not resolved by the indexer.

