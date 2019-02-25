#--------------------------------------------------------
# The CMakeLists.txt for:                       pCommand
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  Command.cpp
  Command_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pCommand ${SRC})

TARGET_LINK_LIBRARIES(pCommand
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)

