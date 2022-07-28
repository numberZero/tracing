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
	)

target_link_libraries(prog3
	Threads::Threads
)

target_compile_definitions(prog3 PRIVATE
	THREADS=1
)
