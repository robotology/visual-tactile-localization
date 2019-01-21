add_subdirectory(sfm-library)
add_subdirectory(gaze-ctrl-library)
add_subdirectory(object-tracking)

option(BUILD_OBJECT_TRACKING_VIEWER "Build object-tracking-viewer" OFF)
if (BUILD_OBJECT_TRACKING_VIEWER)
  add_subdirectory(object-tracking-viewer)
endif()
