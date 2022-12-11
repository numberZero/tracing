cmake_minimum_required(VERSION 3.12)

project(space_refraction)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(Threads REQUIRED)

add_subdirectory(asyncpp)

add_compile_definitions(
	GL_GLEXT_PROTOTYPES
	GLM_FORCE_ALIGNED_GENTYPES
	GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
	GLM_FORCE_INTRINSICS
)

include_directories(.)


add_executable(space_refraction
	main.cxx
	#shaders.cxx
	shader.cxx
	io.cxx
)

target_link_libraries(space_refraction
	glfw
	GL
	fmt
	png
)


add_executable(prog2
	prog2.cxx
)


add_executable(prog3
	prog3.cxx
	render.cxx
	scene.cxx
	trace.cxx
)

option(MULTITHREAD "Use multithreading" ON)
option(DRAFT "Draft rendering" ON)

target_link_libraries(prog3
	Threads::Threads
)

target_include_directories(prog3 PRIVATE
	pcg-cpp/include/
)

target_compile_definitions(prog3 PRIVATE
	THREADS=$<BOOL:${MULTITHREAD}>
	BEST=$<NOT:$<BOOL:${DRAFT}>>
)


add_executable(prog4
	prog4.cxx
)

target_link_libraries(prog4
	Threads::Threads
	glfw
	GL
)


add_executable(prog5
	prog5.cxx
)

target_link_libraries(prog5
	Threads::Threads
	glfw
	GL
	asyncpp
)

target_compile_definitions(prog5 PUBLIC
	DIM=2
)


add_executable(prog6
	prog6.cxx
	shader.cxx
	io.cxx
)

target_link_libraries(prog6
	Threads::Threads
	glfw
	GL
	asyncpp
	png
	fmt
)

target_compile_definitions(prog6 PUBLIC
	DIM=3
)
