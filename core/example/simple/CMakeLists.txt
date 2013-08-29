INCLUDE(TribitsAddExecutableAndTest)
INCLUDE(TribitsCopyFilesToBinaryDir)

TRIBITS_ADD_EXECUTABLE_AND_TEST(
  SimpleExample
  SOURCES SimpleEvaluator cxx_main.cpp
  )

TRIBITS_COPY_FILES_TO_BINARY_DIR(
  SimpleINP
  SOURCE_FILES input.xml simple_example.plot
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
  DEST_DIR ${CMAKE_CURRENT_BINARY_DIR}
  EXEDEPS SimpleExample
)
