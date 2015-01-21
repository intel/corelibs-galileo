Quick setup Izmir internal developers guide

############################### Linux ###############################
IDE:
	get http://downloads.arduino.cc/arduino-1.5.2-linux64.tgz
	tar xfz

	
	get sources
	git clone bodonog1@magnum.ir.intel.com:/p/clanton/users/software/izmir_repos/izmir-rt.git

	create symlink
	ln -s /nfs/iir/home/bodonog1/Development/izmir/izmir-rt/x86/ /nfs/iir/home/bodonog1/Development/arduino-1.5.2/hardware/arduino/x86

	Finally - symlink
	/nfs/iir/home/bodonog1/Development/arduino-1.5.2/hardware/arduino/x86/platform.linux.txt to /nfs/iir/home/bodonog1/Development/arduino-1.5.2/hardware/arduino/x86/platform.txt

Cross compiler:
	On Magnum server:
	ln -s /p/clanton/users/software/linux_xcompiler/1.4.1/sysroots /nfs/iir/home/bodonog1/Development/arduino-1.5.2/hardware/tools/x86
	#ln -s /opt/clanton-tiny/1.4.1/sysroots /nfs/iir/home/bodonog1/Development/arduino-1.5.2/hardware/tools/x86

Startup:
	arduino-1.5.2/arduino
	File => Preferences => Enable verbose output on compile and upload tool



############################## Windows ##############################

Prerequisites:
	Note assuming C:\Development directory - amend as necessary.
	Assuming installtion of "symlink" ability on Windows : http://www.howtogeek.com/howto/16226/complete-guide-to-symbolic-links-symlinks-on-windows-or-linux/
	Get : 	http://download.microsoft.com/download/8/B/4/8B42259F-5D70-43F4-AC2E-4B208FD8D66A/vcredist_x64.EXE
		http://schinagl.priv.at/nt/hardlinkshellext/HardLinkShellExt_X64.exe

IDE:

	
	get http://downloads.arduino.cc/arduino-1.5.2-windows.zip
	unzip to C:\arduino-1.5.2
	
	mkdir Development\izmir
	cd izmir
	get sources using tortoise git
	git clone bodonog1@magnum.ir.intel.com:/p/clanton/users/software/izmir_repos/izmir-rt.git

	create symbol link in windows explorer from C:\Development\izmir\izmir-rt\x86 to C:\arduino-1.5.2\hardware\arduino\x86
	Right click on C:\Development\izmir\izmir-rt\x86 - select plink source
	Right click in C:\arduino-1.5.2\hardware\arduino select "Drop as" and name x86

	Finally - symlink
	C:\arduino-1.5.2\hardware\arduino\x86\platform.win.txt to C:\arduino-1.5.2\hardware\arduino\x86\platform.txt

Cross compiler - Windows
	get /p/clanton/users/software/xcompiler/clanton-tiny-uclibc-i686-i586-toolchain-1.4.1.tar.bz2
	uncompress to C:\arduino-1.5.2\hardware\tools Development\izmir\tools
		(note @ current version unresolved headers to-do with netfilter should just be overwritten)
	create symlink from Development\sysroots to C:\arduino-1.5.2\hardware\tools\x86

Startup:
	arduino-1.5.2/arduino
	File => Preferences => Enable verbose output on compile and upload tool

############################## clloader ############################

How to run the clloader on the target

Load drivers
	modprobe pch_udc
	modprobe g_serial

Make /sketch directory
	mkdir /sketch

Run bootloader
	rm -f /sketch/* && rm -f /tmp/* && /bin/clloader -vvv --escape --binary < /dev/ttyGS0 > /dev/ttyGS0
