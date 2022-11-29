#version 430

struct decomp3 {
	mat3 ortho;
	vec3 diag;
};

decomp3 halfmetric(vec3 pos);

mat3 metric(vec3 pos) {
	decomp3 m = halfmetric(pos);
	vec3 v = m.diag * m.diag;
	mat3 d = mat3(v.x, 0, 0, 0, v.y, 0, 0, 0, v.z);
	return transpose(m.ortho) * d * m.ortho;
}

mat3[3] dmetric(vec3 pos) {
	const float eps = 1.0e-3;
	mat3[3] result;
	for (int k = 0; k < 3; k++) {
		vec3 delta = vec3(0);
		delta[k] = eps;
		result[k] = (metric(pos + delta) - metric(pos - delta)) / (2.0 * eps);
	}
	return result;
}

mat3[3] krist(vec3 pos) {
	// Γ^i_k_l = .5 * g^i^m * (g_m_k,l + g_m_l,k - g_k_l,m)
	mat3 g = inverse(metric(pos)); // с верхними индексами
	mat3[3] d = dmetric(pos);
	mat3[3] ret;
	// ret[i][l][k] = sum((m) => .5f * g[m][i] * (d[k][l][m] + d[l][k][m] - d[m][k][l]))
	for (int i = 0; i < 3; i++)
	for (int l = 0; l < 3; l++)
	for (int k = 0; k < 3; k++) {
		float v = 0.0;
		for (int m = 0; m < 3; m++)
			v += g[m][i] * (d[l][k][m] + d[k][m][l] - d[m][k][l]);
		ret[i][l][k] = 0.5 * v;
	}
	return ret;
}
