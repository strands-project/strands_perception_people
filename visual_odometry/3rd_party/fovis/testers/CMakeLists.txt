add_executable(absolute-orientation-tester 
    absolute_orientation_horn_tester.cpp)
pods_use_pkg_config_packages(absolute-orientation-tester 
    eigen3
    libfovis)

add_executable(refine-motion-estimate-tester 
    refine_motion_estimate_tester.cpp)
pods_use_pkg_config_packages(refine-motion-estimate-tester
    eigen3
    libfovis)

if(BOT2_LCMGL_FOUND)
add_executable(init-homography-estimate-tester 
    initial_homography_estimation_tester.cpp)
pods_use_pkg_config_packages(init-homography-estimate-tester 
    libfovis
    eigen3 
    bot2-core 
    bot2-lcmgl-client)
endif(BOT2_LCMGL_FOUND)
