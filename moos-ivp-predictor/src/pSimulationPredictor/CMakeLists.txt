#--------------------------------------------------------
# The CMakeLists.txt for:                       pSimulationPredictor
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  SimulationPredictor.cpp
  SimulationPredictor_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pSimulationPredictor ${SRC})

TARGET_LINK_LIBRARIES(pSimulationPredictor
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

