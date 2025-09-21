# Build Note

- Should install Pangolin first, git clone the repository to install.
- Should install Epoxy before install Pangolin.
- Should Add -DCMAKE_CXX_FLAGS="-Wno-error=missing-braces -Wno-error=type-limits" in cmake command to avoid the warning.
- Should modify the CMakeLists.txt of orb-slam-elastic to change the OpenCV version check or it will cause an error.
- Docker can't recognize -march=native, should use -mtune=generic instead.
- C++11/C++0x support check should be replaced with C++14 standard.
- stdint-gcc.h should be replaced with cstdint.
- ImuTypes.h should include <iostream>
- Should use clang instead of gcc.
- Should build ThirdParty libraries first.

## Warning
Add -DCMAKE_CXX_FLAGS="-Wno-error=missing-braces -Wno-error=type-limits" in cmake command to avoid the warning.
```bash
12.49 /home/Pangolin/components/pango_windowing/src/display_x11.cpp:434:16: error: suggest braces around initialization of subobject [-Werror,-Wmissing-braces]
12.49                (float)ev.xbutton.x, (float)ev.xbutton.y,
12.49                ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
12.49                {
12.49 /home/Pangolin/components/pango_windowing/src/display_x11.cpp:502:21: error: suggest braces around initialization of subobject [-Werror,-Wmissing-braces]
12.49                     (float)ev.xkey.x, (float)ev.xkey.y,
12.49                     ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
12.49                     {
12.64 2 errors generated.
12.64 make[2]: *** [CMakeFiles/pango_windowing.dir/build.make:88: CMakeFiles/pango_windowing.dir/components/pango_windowing/src/display_x11.cpp.o] Error 1
12.64 make[1]: *** [CMakeFiles/Makefile2:814: CMakeFiles/pango_windowing.dir/all] Error 2
12.64 make[1]: *** Waiting for unfinished jobs....
```
## Error
### 1
Soulution: Replace -march=native with -mtune=generic
```dockerfile   
RUN sed -i 's/-march=native/-mtune=generic/g' CMakeLists.txt
```

```bash
1.855 clangclangclangclang: : : : error: error: error: error: the clang compiler does not support '-march=native'the clang compiler does not support '-march=native'the clang compiler does not support '-march=native'the clang compiler does not support '-march=native'
1.855 
1.855 
1.855 
```
### 2
Soulution: Replace stdint-gcc.h with cstdint
```dockerfile
RUN sed -i 's|<stdint-gcc.h>|<cstdint>|' src/ORBmatcher.cc
RUN sed -i 's|<stdint-gcc.h>|<cstdint>|' Thirdparty/DBoW2/DBoW2/FORB.h
```

```bash
20.20 /home/orb-slam-elastic/src/ORBmatcher.cc:28:9: fatal error: 'stdint-gcc.h' file not found
20.20 #include<stdint-gcc.h>
20.20 ^~~~~~~~~~~~~~
20.21 3 warnings and 1 error generated.
20.21 make[2]: *** [CMakeFiles/ORB_SLAM3.dir/build.make:146: CMakeFiles/ORB_SLAM3.dir/src/ORBmatcher.cc.o] Error 1
20.21 make[2]: *** Waiting for unfinished jobs....
```

### 3
Soulution: Replace C++11/C++0x support check with C++14 standard
```dockerfile
RUN sed -i '/# Check C++11 or C++0x support/,/endif()/c\# Set C++14 standard\
set(CMAKE_CXX_STANDARD 14)\
set(CMAKE_CXX_STANDARD_REQUIRED ON)\
set(CMAKE_CXX_EXTENSIONS OFF)' CMakeLists.txt
```

```bash
4.853 /usr/local/include/sigslot/signal.hpp:291:32: error: no member named 'enable_if_t' in namespace 'std'
4.853 struct object_pointer<T*, std::enable_if_t<trait::is_pointer_v<T*>>> {
4.853                           ~~~~~^
4.853 /usr/local/include/sigslot/signal.hpp:291:68: error: expected unqualified-id
4.853 struct object_pointer<T*, std::enable_if_t<trait::is_pointer_v<T*>>> {
4.853                                                                    ^
4.884 /usr/local/include/sigslot/signal.hpp:291:32: error: no member named 'enable_if_t' in namespace 'std'
4.884 struct object_pointer<T*, std::enable_if_t<trait::is_pointer_v<T*>>> {
4.884                           ~~~~~^
4.884 /usr/local/include/sigslot/signal.hpp:291:68: error: expected unqualified-id
4.884 struct object_pointer<T*, std::enable_if_t<trait::is_pointer_v<T*>>> {
4.884                                                                    ^
4.948 /usr/local/include/sigslot/signal.hpp:298:31: error: no member named 'enable_if_t' in namespace 'std'
4.948 struct object_pointer<T, std::enable_if_t<trait::is_weak_ptr_v<T>>> {
4.948                          ~~~~~^
4.948 fatal error: too many errors emitted, stopping now [-ferror-limit=]
4.979 /usr/local/include/sigslot/signal.hpp:298:31: error: no member named 'enable_if_t' in namespace 'std'
4.979 struct object_pointer<T, std::enable_if_t<trait::is_weak_ptr_v<T>>> {
4.979                          ~~~~~^
4.979 fatal error: too many errors emitted, stopping now [-ferror-limit=]
5.400 3 warnings and 20 errors generated.
5.408 make[2]: *** [CMakeFiles/ORB_SLAM3.dir/build.make:76: CMakeFiles/ORB_SLAM3.dir/src/System.cc.o] Error 1
5.408 make[2]: *** Waiting for unfinished jobs....
5.413 [ 10%] Building CXX object Thirdparty/g2o/CMakeFiles/g2o.dir/g2o/core/hyper_graph_action.cpp.o
5.465 3 warnings and 20 errors generated.
5.473 make[2]: *** [CMakeFiles/ORB_SLAM3.dir/build.make:90: CMakeFiles/ORB_SLAM3.dir/src/Tracking.cc.o] Error 1
5.473 make[1]: *** [CMakeFiles/Makefile2:100: CMakeFiles/ORB_SLAM3.dir/all] Error 2
5.473 make[1]: *** Waiting for unfinished jobs....
```

### 4
Solution: Include <iostream> in ImuTypes.h

```dockerfile
RUN sed -i '/#include <utility>/a #include <iostream>' include/ImuTypes.h
```

```bash
36.77 In file included from /home/orb-slam-elastic/src/ImuTypes.cc:19:
36.77 /home/orb-slam-elastic/include/ImuTypes.h:203:14: error: no member named 'cout' in namespace 'std'
36.77         std::cout << "pint meas:\n";
36.77         ~~~~~^
36.78 /home/orb-slam-elastic/include/ImuTypes.h:205:18: error: no member named 'cout' in namespace 'std'
36.78             std::cout << "meas " << mvMeasurements[i].t << std::endl;
36.78             ~~~~~^
36.78 /home/orb-slam-elastic/include/ImuTypes.h:207:14: error: no member named 'cout' in namespace 'std'
36.78         std::cout << "end pint meas:\n";
36.78         ~~~~~^
```
