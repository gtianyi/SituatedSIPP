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
     situatedSIPP/learningAlgorithms/DijkstraLearning.cpp
     situatedSIPP/learningAlgorithms/PlrtaLearning.cpp
     situatedSIPP/learningAlgorithms/learningAlgorithmBase.cpp
     constraints.cpp
     situatedSIPP/subintervals.cpp
     tracy/TracyClient.cpp
     safeIntervals.cpp )

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
     situatedSIPP/structs.h
     situatedSIPP/realtime_sipp.h
     situatedSIPP/learningAlgorithms/DijkstraLearning.hpp
     situatedSIPP/learningAlgorithms/learningAlgorithmBase.hpp
     situatedSIPP/learningAlgorithms/PlrtaLearning.hpp
     situatedSIPP/subintervals.hpp
     structs.h
     json.hpp
     constraints.h
     safeIntervals.hpp )

add_executable(ssipp ${SOURCE_FILES} ${HEADER_FILES} )
target_link_libraries(ssipp CONAN_PKG::boost)
