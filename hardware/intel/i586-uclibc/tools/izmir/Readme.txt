
1) Copy the approprate cl_upload_win.sh, cl_upload_osx.sh from this directory into .../arduino-1.5.2/hardware/tools/x86/bin/

2) Copy the appropraite 
 binary-redistro/<os>/* into the /arduino-1.5.2/hardware/tools/x86/bin/

----------------------------------------------------------

The Host download tools are currently reusing the lsz command from lrzsz.

To build the host programs:
tar -xvf lrzsz-0.12.20.tar.gz
cd lrzsz-0.12.20
./configure
make
cp src/lsz* .../arduino-1.5.2/hardware/tools/x86/bin/

Note: The for non Linux distributions, you must include bash the cygwin1.dll and other dependant .dlls used in the same directory as lsz. On windows cygwin, the cygwin1.dll used can be found in cycwin64/bin (example).


-----------------------------------------------------------------------------------


