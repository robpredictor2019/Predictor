#--------------------------------------------------------
# The CMakeLists.txt for:                       pSimulationStateMachine
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  SimulationStateMachine.cpp
  SimulationStateMachine_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pSimulationStateMachine ${SRC})

TARGET_LINK_LIBRARIES(pSimulationStateMachine
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

