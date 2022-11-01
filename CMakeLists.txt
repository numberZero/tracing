cmake_minimum_required(VERSION 3.12)

project(space_refraction)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(Threads REQUIRED)

add_compile_definitions(
	GL_GLEXT_PROTOTYPES
	)

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

add_executable(prog4
	prog4.cxx
	)

target_link_libraries(prog3
	Threads::Threads
)

target_link_libraries(prog4
	Threads::Threads
	glfw
	GL
)

option(MULTITHREAD "Use multithreading" ON)
option(DRAFT "Draft rendering" ON)

target_compile_definitions(prog3 PRIVATE
	THREADS=$<BOOL:${MULTITHREAD}>
	BEST=$<NOT:$<BOOL:${DRAFT}>>
)

target_include_directories(prog3 PRIVATE
	.
	pcg-cpp/include/
)
