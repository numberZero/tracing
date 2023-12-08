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

	struct OutRay {
		vec4 pos_dist;
		vec4 dir_pad;
	};

	std::vector<TraceResult> trace(std::span<Ray> from) const override {
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
		int ngroups = nrays / workgroup_size;
		if (ngroups < 2)
			return RiemannSubspace::trace(from);

		auto batch = from.subspan(workgroup_size * ngroups);
		BufferID bufs[3];
		glCreateBuffers(3, bufs);
		BufferID bIn = bufs[0];
		BufferID bOut = bufs[1];
		BufferID bParams = bufs[2];
		glNamedBufferStorage(bIn, sizeof(batch[0]) * batch.size(), batch.data(), 0);
		glNamedBufferStorage(bOut, sizeof(OutRay) * batch.size(), nullptr, GL_MAP_READ_BIT);
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

		glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
		const auto *to = reinterpret_cast<OutRay *>(glMapNamedBuffer(bOut, GL_READ_ONLY));

		std::vector<TraceResult> result;
		result.resize(nrays);
		for (int k = 0; k < batch.size(); k++) {
			OutRay ray = to[k];
			Ray end;
			end.pos = vec3(ray.pos_dist);
			end.dir = vec3(ray.dir_pad);
			assert(!map->contains(end.pos));
			result[k] = {end, leave(end), ray.pos_dist.w};
		}

		glUnmapNamedBuffer(bOut);
		glDeleteBuffers(3, bufs);

		// handle the tail on the CPU
		for (int k = batch.size(); k < from.size(); k++) {
			result[k] = trace(from[k]);
		}

		return result;
	}
};
