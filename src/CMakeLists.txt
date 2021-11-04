set( SOURCE_FILES
     main.cpp
     tinyxml2.cpp
     xmlLogger.cpp
     mission.cpp
     map.cpp
     task.cpp
     config.cpp
     dynamicobstacles.cpp
     aa_sipp.cpp
     situatedSIPP/realtime_sipp.cpp
     situatedSIPP/situated_sipp.cpp
     situatedSIPP/learningAlgorithms/DijkstraLearning.cpp
     situatedSIPP/learningAlgorithms/learningAlgorithmBase.cpp
     constraints.cpp )

set( HEADER_FILES
     tinyxml2.h
     heuristic.h
     searchresult.h
     gl_const.h
     xmlLogger.h
     mission.h
     map.h
     task.h
     dynamicobstacles.h
     config.h
     lineofsight.h
     aa_sipp.h
     situatedSIPP/realtime_sipp.h
     situatedSIPP/situated_sipp.h
     situatedSIPP/learningAlgorithms/DijkstraLearning.hpp
     situatedSIPP/learningAlgorithms/learningAlgorithmBase.hpp
     structs.h
     constraints.h )

add_executable(ssipp ${SOURCE_FILES} ${HEADER_FILES} )
target_link_libraries(ssipp CONAN_PKG::boost)
