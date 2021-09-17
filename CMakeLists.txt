cmake_minimum_required(VERSION 3.12)

project(space_refraction)

add_executable(space_refraction
	main.cxx
	#shaders.cxx
	shader.cxx
	io.cxx
	)

target_compile_definitions(space_refraction PUBLIC
	GL_GLEXT_PROTOTYPES
	)

target_link_libraries(space_refraction PUBLIC
	glfw
	GL
	fmt
	)

target_compile_options(space_refraction PUBLIC
	#-fno-exceptions
	#-fno-rtti
	)
