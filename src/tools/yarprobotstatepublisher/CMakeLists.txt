# Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>

if(ISAAC_USES_IDYNTREE)
  project(yarprobotstatepublisher)

  add_executable(${PROJECT_NAME} include/robotstatepublisher.h src/main.cpp src/robotstatepublisher.cpp)
  target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include
                                                     ${YARP_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})
  install(TARGETS ${PROJECT_NAME} DESTINATION bin)
endif()
