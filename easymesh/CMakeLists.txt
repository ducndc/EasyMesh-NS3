include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# Create a custom library
add_library(
  easymesh-lib
  lib/easymesh-agent.cc
  lib/easymesh-assoc-manager.cc
  lib/easymesh-controller.cc
  lib/easymesh-link.cc
  lib/easymesh-radio.cc
  lib/channel-planner.cc
)
target_link_libraries(easymesh-lib ${libcore} ${ns3-libs})

# Link the custom library to the scratch executable
build_exec(
  EXECNAME easymesh_sim
  SOURCE_FILES easymesh.cc
  LIBRARIES_TO_LINK easymesh-lib
                    ${libcore}
                    ${ns3-libs}
                    ${ns3-contrib-libs}
  # use "${ns3-libs}" "${ns3-contrib-libs}" in case you want to link to all
  # enabled modules
  EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/easymesh
)
