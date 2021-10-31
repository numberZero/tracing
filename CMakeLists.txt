cmake_minimum_required(VERSION 3.12)

project(space_refraction)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_compile_definitions(
	GL_GLEXT_PROTOTYPES
	)

link_libraries(
	glfw
	GL
	fmt
	png
	)

add_executable(space_refraction
	main.cxx
	#shaders.cxx
	shader.cxx
	io.cxx
	)

add_executable(refraction2
	refr2.cxx
	shader.cxx
	io.cxx
	)
