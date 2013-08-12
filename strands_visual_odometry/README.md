## Troubleshooting
If you get an error message that states: 
```
/usr/lib/gcc/i686-linux-gnu/4.6/include/emmintrin.h:32:3: error: #error "SSE2 instruction set not enabled"
```
or similar, you have to add `set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3")` to the CMakeLists.txt file after the
```
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-O3")        ## Optimize
endif()
```
statement so that it looks like:
```
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-O3")        ## Optimize
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3")
```
