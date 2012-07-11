There seems to be two sets of C++ libraries included with the BlackBerry NDK (2.0.1 as of this writing). One version is a slimmed down lib that's probably good for basic stuff ([Dinkum](https://bdsc.webapps.blackberry.com/native/reference/com.qnx.doc.dinkum/topic/cpp/index.html)), and the other version seems to be the GNU STL implementation.

The Dinkum lib resides in:  

     $BBNDK/target/qnx6/usr/include/cpp
     $BBNDK/target/$$TARGET/lib/gcc/4.4.2/libcpp.so
     $BBNDK/target/$$TARGET/lib/gcc/4.4.2/libcpp.a

The GNU lib resides in:  

    $BBNDK/target/qnx6/usr/include/c++/4.4.2/  
    $BBNDK/target/$$TARGET/lib/gcc/4.4.2/libstdc++.so  
    $BBNDK/target/$$TARGET/lib/gcc/4.4.2/libstdc++.a  

Since C++ is ABI dependent, there are additional target-specific headers in:  

    $BBNDK/target/qnx6/usr/include/c++/4.4.2/arm-unknown-nto-qnx6.5.0eabi/  
    $BBNDK/target/qnx6/usr/include/c++/4.4.2/arm-unknown-nto-qnx6.5.0eabi/pic  
    $BBNDK/target/qnx6/usr/include/c++/4.4.2/i486-pc-nto-qnx6.5.0/  
    $BBNDK/target/qnx6/usr/include/c++/4.4.2/i486-pc-nto-qnx6.5.0/pic  

I think that the `pic` folders are for 'position-independent-code' builds (so basically shared libs), but I'm not sure... would be nice if someone could confirm. It's also interesting that we're given static builds, as both sets of libs exist on both the simulator and device.

The Momentics IDE includes the Dinkum lib include dirs in its set of default include paths when a C/C++ project is created.

If the GCC tool chain is explicitly selected however, ([highlight/select a project] File -> Projects -> C/C++ Build -> Tool Chain Editor -> Current toolchain), these paths seem to be implicitly included with ntox86-gcc/g++:

    $BBNDK/host/linux/x86/usr/lib/gcc/i486-pc-nto-qnx6.5.0/4.4.2/include
    $BBNDK/target/qnx6/usr/include/c++/4.4.2
    $BBNDK/target/qnx6/usr/include/c++/4.4.2/i486-pc-nto-qnx6.5.0
    $BBNDK/target/qnx6/usr/include/c++/4.4.2/backward
    $BBNDK/target/qnx6/usr/include/

It looks like if you want to compile with the Dinkum libs, you use the qcc tool chain and if you want to use the GNU libs, use the gnu tool chain. None of these cases seem to require you to ever manually include additional paths. It might be possible that you need to specify the 'pic' paths for compiling shared libs.
