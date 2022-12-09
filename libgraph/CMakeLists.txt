#将源文件添加到库
add_library(libgraph STATIC "tgaimage.h" "tgaimage.cpp" "model.h" "geometry.h" "model.cpp" "geometry.cpp"  )

target_include_directories(libgraph
							INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
							)