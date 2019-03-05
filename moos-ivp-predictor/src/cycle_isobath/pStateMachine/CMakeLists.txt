#--------------------------------------------------------
# The CMakeLists.txt for:                       pStateMachine
# Author(s):                              Louis VALERY
#--------------------------------------------------------

SET(SRC
  StateMachine.cpp
  StateMachine_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pStateMachine ${SRC})

TARGET_LINK_LIBRARIES(pStateMachine
   ${MOOS_LIBRARIES}
   mbutil
   m
   opencv_core
   opencv_highgui
   pthread)

