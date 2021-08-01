add_library(fg_filtering
            FactorGraphFiltering.cpp GraphManager.cpp StaticTransforms.cpp)
target_link_libraries(fg_filtering ${catkin_LIBRARIES} gtsam gtsam_unstable)

install(TARGETS fg_filtering
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)