#--------------------------------------------------------
# The CMakeLists.txt for:                       pState1_isobath
# Author(s):                              Louis VALERY
#--------------------------------------------------------

SET(SRC
  State1_isobath.cpp
  State1_isobath_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pState1_isobath ${SRC})

TARGET_LINK_LIBRARIES(pState1_isobath
   ${MOOS_LIBRARIES}
   mbutil
   m
   opencv_core
   opencv_highgui
   pthread)
