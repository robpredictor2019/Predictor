#--------------------------------------------------------
# The CMakeLists.txt for:                       pState2_turn_left
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  State2_turn_left.cpp
  State2_turn_left_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pState2_turn_left ${SRC})

TARGET_LINK_LIBRARIES(pState2_turn_left
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

