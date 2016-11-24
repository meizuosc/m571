[M571](http://www.meizu.com)
=================

Linux kernel source code for the Meizu M2 Note.

HOW TO COMPILE
-----------

###1. Download source code###

  <code>git clone https://github.com/Moyster/android_kernel_m2note.git</code>

###2. Compiling###

Make sure your device tree uses :  
```
TARGET_KERNEL_CONFIG := cmXX_m2note_defconfig
```
(where "XX" = cm version, currently supported : 12 / 13 / 14)  

From within the Android Sources you want to build, do :  

```
. build/envsetup.sh && breakfast m2note  
make -jX bootimage  
(where -jX is your number of cores + 1, -j5 for a quadcore)  
```

This will give you a flashable boot.img located in out/target/product/device-xyz/  

