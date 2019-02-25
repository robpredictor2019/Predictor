#--------------------------------------------------------
# The CMakeLists.txt for:                       pSuiviCap
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  SuiviCap.cpp
  SuiviCap_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pSuiviCap ${SRC})

TARGET_LINK_LIBRARIES(pSuiviCap
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)

