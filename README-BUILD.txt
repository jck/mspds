Building MSPDebugStack
======================

1. Windows
----------

1.1 Development environment
---------------------------

* You will need to have Microsoft Visual Studio 2010 SP1 installed.
* You will need to have a IAR EW430 5.51 or later installed if you want to
  build the firmware. IAR EW430 5.52 is required to build the BSL.


1.2. Dependencies
-----------------

In order to compile the DLL with MSVC 2010 you will need:

* boost
    -Download and build (http://www.boost.org)
	-Visual Studio users can get pre-built boost binaries from
	 http://sourceforge.net/projects/boost/files/boost-binaries/
    -Version used in official build is 1.53
    -Visual Studio must be configured to find boost headers and libs

* hidapi
    -Download and build (https://github.com/signal11/hidapi/downloads)
    -Version used in official build is 0.7.0
    -hidapi.h must be copied to ThirdParty\include
    -hidapi.lib must be copied to ThirdParty\lib


To compile the firmware and/or BSL with IAR EW430, the following tools are required:

* srecord
    -Download from http://sourceforge.net/projects/srecord/files/srecord-win32
    -Version used in official build is 1.59
    -srec_cat.exe must be copied to Bios\tools

* sed
    -Any Windows port, for example from http://sourceforge.net/projects/unxutils
    -sed.exe must be copied to Bios\tools


1.3.1 Building the firmware (optional)
--------------------------------------

* Build all projects in Bios\bios_core.eww

* Build all projects in Bios\eZ_FET_Bios.eww
	-Make sure eZ_FET configuration is selected for all projects

* Build all projects in Bios\MSP-FET_Bios.eww
	-Make sure MSP_FET configuration is selected for all projects

NOTE: Due to the license of DirectC, the required DirectC source is not part
      of the OS package and the depending project MSP-FET_FpgaUpdate.ewp is not
	  built by default.

	  To build the MSP-FET_FpgaUpdate project, please download DirectC 2.7 from
	  http://www.actel.com/download/program_debug/directc/dc27.aspx
	  and copy the sources to MSPDebugStack/Bios/src/fpga_update/DirectC. See
	  the README.txt in this directory for details.


1.3.2 Building the BSL (optional)
---------------------------------

* Build all project configurations in ThirdParty\BSL430_Firmware\BSL.eww


1.3.3 Building the DLL
----------------------

* Build BSL430_DLL.sln in ThirdParty\BSL430_DLL

* Build DLL430_v3_2010.sln

NOTE: Due to the license of DirectC, there is no image for the FPGA update
      included in the OS package. To build this image, please refer to 1.3.1.

	  To enable the FPGA update, comment in the line "#define FPGA_UPDATE" in
	  MSPDebugStack/DLL430_v3/src/TI/DLL430/UpdateManagerFet.cpp.


2. Linux
--------

2.1 Development environment
---------------------------

* You will need the GNU tool chain to use the existing Makefile.


2.2. Dependencies
-----------------

In order to compile the DLL with MSVC 2010 you will need:

* boost
    -Download from http://www.boost.org
    -Version used in official build is 1.53
    -Boost should be built with BOOST_THREAD_PATCH
     (add "define=BOOST_THREAD_PATCH" to the call to bjam)

* hidapi
    -Download from https://github.com/signal11/hidapi/downloads
    -Version used in official build is 0.7.0
    -The MSPDebugStack project assumes hidapi being built against libusb-1.0
     (the default used in the Makefile coming with hidapi 0.7.0)
    -hidapi.h must be copied to ThirdParty\include
    -hid-libusb.o must be copied to ThirdParty\lib


2.3 Building the shared object
------------------------------

* run "make STATIC=1 BIT32=1" in MSPDebugStack
    -If boost is not globally installed, use "make BOOST_DIR=<path to boost>".
     The makefile assumes headers in boost/ and libraries in stage/lib/ of the
     specified directory.

    -Linking dependencies statically (STATIC=1) is advised if the library will
     be copied between machines. A 32bit build (BIT32=1) is required for use
     with existing IDEs.
