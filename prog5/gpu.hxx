#pragma once
#include <vector>
#include <GL/gl.h>
#include "subspace.hxx"

using ProgramID = GLuint;
using BufferID = GLuint;

class GpuRiemannSubspace: public RiemannSubspace {
public:
	using RiemannSubspace::trace;
	static constexpr int workgroup_size = 64;

	ProgramID prog = 0;
	std::vector<std::byte> params;

	struct alignas(16) OutRay {
		vec3 pos;
		float dist;
		vec3 dir;
		// float pad;
	};

	std::vector<TraceResult> trace(std::vector<Ray> from) const override {
		GLuint idx = glGetProgramResourceIndex(prog, GL_UNIFORM_BLOCK, "ParamsBlock");
		if (idx == -1)
			throw "Bad program";
		GLint size;
		glGetProgramResourceiv(prog, GL_UNIFORM_BLOCK, idx, 1, (GLenum const[]){GL_BUFFER_DATA_SIZE}, 1, nullptr, &size);
		if (params.empty())
			throw "No parameters";
		if (params.size() > size)
			throw "Too many parameters";
		std::vector<std::byte> dParams(size);
		std::memcpy(dParams.data(), params.data(), params.size());

		int nrays = from.size();
		int ngroups = (nrays + workgroup_size - 1) / workgroup_size;
		from.resize(workgroup_size * ngroups);
		for (int k = nrays; k < from.size(); k++)
			from[k] = from[0]; // pad the array
		BufferID bufs[3];
		glCreateBuffers(3, bufs);
		BufferID bIn = bufs[0];
		BufferID bOut = bufs[1];
		BufferID bParams = bufs[2];
		glNamedBufferStorage(bIn, sizeof(from[0]) * from.size(), from.data(), 0);
		glNamedBufferStorage(bOut, sizeof(OutRay) * from.size(), nullptr, 0);
		glNamedBufferStorage(bParams, dParams.size(), dParams.data(), 0);

		glUseProgram(prog);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, bIn);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, bOut);
		glBindBufferBase(GL_UNIFORM_BUFFER, 2, bParams);
		glDispatchCompute(ngroups, 1, 1);
// 		glDispatchComputeGroupSizeARB(ngroups, 1, 1, workgroup_size, 1, 1);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, 0);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
		glBindBufferBase(GL_UNIFORM_BUFFER, 2, 0);
		glUseProgram(0);

		std::vector<OutRay> to;
		to.resize(nrays);
		glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
		glGetNamedBufferSubData(bOut, 0, sizeof(to[0]) * to.size(), to.data());

		glDeleteBuffers(3, bufs);

		std::vector<TraceResult> result;
		result.resize(nrays);
		for (int k = 0; k < nrays; k++) {
			OutRay ray = to[k];
			assert(!map->contains(ray.pos));
			Ray end;
			end.pos = ray.pos;
			end.dir = ray.dir;
			result[k] = {end, leave(end), ray.dist};
		}
		return result;
	}
};
