# Build Note

- Should install Pangolin first, git clone the repository to install.
- Should install Epoxy before install Pangolin.
- Should modify the CMakeLists.txt of orb-slam-elastic to change the OpenCV version check or it will cause an error.
- Should use clang instead of gcc. 

## Warning
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