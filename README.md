Goal
----

Here you will find the original code from Dmitry Denisenko which is used 
in many Altera OpenCL PCIe accelerators, like the Terasic DE5Net and Terasic OpenVino starter kit
that I'm currently using.

The original code supports [conservative] systems based on Centos6/7 OS with
"old" kernels like 3.8.

I try to adapt it to my systems running Ubuntu with various versions. Check testing section below.

This needs some tricky minor changes.

I use conditional compilation to make it kernel version independent.

I'm currently using Ubuntu 20.04.4 LTS with a 5.4.0-113-generic kernel.

To check your system details run:
$lsb_release -d
$uname -r



Altera OpenCL PCI Express Driver for Linux
------------------------------------------

This directory contains full source code for the Altera OpenCL 
PCIE Express Driver for Linux [1][2].  

The driver achieves approximately 3100 MB/sec on gen2 x 8 PCIe 
core with SG DMA on Stratix IV GX FPGA (77.5% efficiency).


LIMITATIONS
-----------
Only one single-threaded user process should be accessing the driver 
at a time.

DMA controller supports operations on 32-byte aligned data (both source and
destination must be aligned). Furthermore, the size of the data must also
be 32-byte aligned. If any of these alignments are not met, very slow non-
DMA transfer will be used.


PREREQUISITES
-------------
- GCC version in /usr/bin (the same version of gcc that was used to compile
  the kernel).

- Kernel include files, or the complete source. The make script
  (make_all.sh) assumes that the source is installed in 
  /usr/src/kernels/<version>. If the /usr/src/kernels/<version>
  is missing or does not contain Makefile, install kernel-devel
  package by running (as root) "yum install kernel-devel".


COMPIILE and INSTALL
--------------------

To compile and install the driver, run as root:
  aocl install
(The installation loads the driver, and sets up the necessary files 
to allow automatic load of driver upon reboot)


To manually load/unload the driver:
  sudo /sbin/modprobe aclpci_drv     (load)
  sudo /sbin/modprobe -r aclpci_drv  (unload)


TESTING
-------
The driver was developed and tested on CentOS 5.6, 64-bit with 
2.6.18-238.el5 kernel compiled for x86_64 architecture.


Also tested on 

CentOS 6.4, 64-bit with kernel 2.6.32-358.el6.x86_64

Ubuntu 16.04.6 with kernel 4.4.0-173

Ubuntu 16.04.6 with kernel 4.15.0-76

Ubuntu 17.10 with kernel 4.13


FEEDBACK
--------
Issues, comments, enhancements? 
Contact David Castells-Rufas david.castells@uab.cat or...

(the orginal author) Dmitry Denisenko at ddenisen (at) altera.com.


REFERENCES
----------
[1] Altera, Quartus, and Stratix are tradermarks of Altera Corporation.

[2] OpenCL and the OpenCL logo are trademarks of Apple Inc.
