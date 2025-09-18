# Build Note

- Should install Pangolin first, git clone the repository to install.
- Should install Epoxy before install Pangolin.
- Should Add -DCMAKE_CXX_FLAGS="-Wno-error=missing-braces -Wno-error=type-limits" in cmake command to avoid the warning.
- Should modify the CMakeLists.txt of orb-slam-elastic to change the OpenCV version check or it will cause an error.
- Should use clang instead of gcc. 

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

## Unresolved Error
```bash
/usr/local/include/sigslot/signal.hpp:109:87: error: 'P' does not refer to a value
constexpr bool is_weak_ptr_compatible_v = detail::is_weak_ptr_compatible<std::decay_t<P>>::value;
                                                                                      ^
/usr/local/include/sigslot/signal.hpp:108:20: note: declared here
template <typename P>
                   ^
/usr/local/include/sigslot/signal.hpp:109:79: error: no member named 'decay_t' in namespace 'std'
constexpr bool is_weak_ptr_compatible_v = detail::is_weak_ptr_compatible<std::decay_t<P>>::value;
                                                                         ~~~~~^
/usr/local/include/sigslot/signal.hpp:109:89: error: expected '(' for function-style cast or type construction
constexpr bool is_weak_ptr_compatible_v = detail::is_weak_ptr_compatible<std::decay_t<P>>::value;
```