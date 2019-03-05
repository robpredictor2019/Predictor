#--------------------------------------------------------
# The CMakeLists.txt for:                       pState3_cross_river
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  State3_cross_river.cpp
  State3_cross_river_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pState3_cross_river ${SRC})

TARGET_LINK_LIBRARIES(pState3_cross_river
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)

