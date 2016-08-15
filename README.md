[M571](http://www.meizu.com)
=================

M571 repo is Linux kernel source code for Meizu M2 Note smartphones. With this repo, you can customize the source code and compile a Linux kernel image yourself. Enjoy it!

HOW TO COMPILE
-----------

###1. Download source code###

  <code>git clone https://github.com/meizuosc/m571.git</code>

###2. Compiling###

```
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- m2note_defconfig
mkdir out && make -j8 ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- -C `pwd` O=`pwd`/out
```

  Note:
  + Make sure you have arm cross tool chain, maybe you can download [here](http://www.linaro.org/downloads)
  + If you get a poor cpu in your compiling host, you should use "-j4" or lower instead of "-j8"

Get Help
--------

Checkout our community http://bbs.meizu.cn (in Chinese)
