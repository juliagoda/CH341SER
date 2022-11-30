// ChangeLog 
// 1.0 - 1.1   modified to solve transmition between ch341 and ch341
// 1.1 - 1.2   Support high Linux kernel
Instructions

Note: 1.Please run followed executable programs as root privilege
      2.Current Driver support versions of linux kernel range from 2.6.25 to 3.13.x
      3.Current Driver support 32bits and 64bits linux systems

Usage:
	(load or unload linux driver of CH34x)
	//compile 
	#make
	//load ch34x chips driver
	#make load
	//unload ch34x chips driver
	#make unload
	// Depending on the system in wich you are compiling you may need to do some of the procedures bellow to avoid hickups:
	// First uninstall brltty (if you are blind **DO NOT DO THIS**) brltty helps peoples who can´t see to do things on system. 
	// After compiling **sign the module** (needed in systems with secure boot enabled):
	#kmodsign sha512 /var/lib/shim-signed/mok/MOK.priv /var/lib/shim-signed/mok/MOK.der ./ch34x.ko
	// Then load the module 
// 1.2 - 1.3 Fix some bugs			

