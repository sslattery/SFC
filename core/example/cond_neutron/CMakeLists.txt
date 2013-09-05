INCLUDE(TribitsAddExecutableAndTest)
INCLUDE(TribitsCopyFilesToBinaryDir)

TRIBITS_ADD_EXECUTABLE_AND_TEST(
  ConductionExample
  SOURCES ConductionEvaluator conduction_main.cpp
  )

TRIBITS_COPY_FILES_TO_BINARY_DIR(
  CondNeutronINP
  SOURCE_FILES conduction_input.xml conduction_example.plot
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
  DEST_DIR ${CMAKE_CURRENT_BINARY_DIR}
  EXEDEPS SimpleExample
)
