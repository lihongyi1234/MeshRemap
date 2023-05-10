#include <iostream>
#include <vector>
#include <DirectXMath.h>
#include "OBJ_Loader.h"
#include <igl/per_vertex_normals.h>

#pragma comment(lib, "igl.lib")

struct MatrixMesh
{
	Eigen::MatrixXd V, N, TC;
	Eigen::MatrixXi F, FN, FTC;
};

void remapMesh(MatrixMesh& mesh_cpu)
{
	auto& f = mesh_cpu.F;
	auto& ftc = mesh_cpu.FTC;

	std::vector<int> v_index(mesh_cpu.V.rows());
	for (int i = 0; i < mesh_cpu.V.rows(); i++) { v_index[i] = i; }
	std::vector<std::vector<int>> v_tc_index(mesh_cpu.V.rows());

	for (int i = 0; i < f.rows(); i++) {
		for (int j = 0; j < 3; j++) {
			int f_Index = f(i, j);
			int ftc_Index = ftc(i, j);
			auto iter1 = find(v_tc_index[f_Index].begin(), v_tc_index[f_Index].end(), ftc_Index);
			if (iter1 == v_tc_index[f_Index].end()) {
				v_tc_index[f_Index].emplace_back(ftc_Index);
			}
		}
	}

	std::vector<int> v_tc_count(mesh_cpu.V.rows());
	std::vector<int> v_tc_start(mesh_cpu.V.rows());
	int count = 0;
	for (int i = 0; i < v_tc_index.size(); i++) {
		v_tc_count[i] = v_tc_index[i].size();
		v_tc_start[i] = count;
		count += v_tc_index[i].size();
	}

	Eigen::MatrixXd v_new, tc_new;
	Eigen::MatrixXi f_new;
	v_new.resize(mesh_cpu.TC.rows(), 3);
	tc_new.resize(mesh_cpu.TC.rows(), 2);
	f_new.resize(mesh_cpu.F.rows(), 3);
	count = 0;
	std::vector<int> vNew2vOld(mesh_cpu.TC.rows());
	std::vector<int> vNew2TcOld(mesh_cpu.TC.rows());
	for (int i = 0; i < v_tc_index.size(); i++) {
		for (int j = 0; j < v_tc_index[i].size(); j++) {
			int tc_index = v_tc_index[i][j];
			tc_new.row(count) = mesh_cpu.TC.row(tc_index);

			vNew2vOld[count] = i;
			vNew2TcOld[count] = tc_index;
			count++;
		}
	}
	//count = 0;
	for (int i = 0; i < mesh_cpu.V.rows(); i++) {
		int count1 = 0;
		for (int j = 0; j < v_tc_count[i]; j++) {
			v_new.row(v_tc_start[i] + count1) = mesh_cpu.V.row(i);
			count1++;
		}
	}

	for (int i = 0; i < mesh_cpu.F.rows(); i++) {
		for (int j = 0; j < 3; j++) {
			int v_index = mesh_cpu.F(i, j);
			int tc_index = mesh_cpu.FTC(i, j);
			int start_idx = v_tc_start[v_index];
			int count = v_tc_count[v_index];
			int exist = false;
			for (int k = 0; k < count; k++) {
				int idx = start_idx + k;
				if (vNew2vOld[idx] == v_index && vNew2TcOld[idx] == tc_index) {
					f_new(i, j) = idx;
					exist = true;
				}
			}
			if (!exist) {
				printf("F(%d,%d) Wrong!\n", i, j);
			}
		}
	}
	mesh_cpu.F = f_new;
	mesh_cpu.FTC = f_new;
	mesh_cpu.V = v_new;
	mesh_cpu.TC = tc_new;

	//recalculate normal
	igl::per_vertex_normals(mesh_cpu.V, mesh_cpu.F, mesh_cpu.N);
	mesh_cpu.FN = mesh_cpu.F;
}

int main(int argc, char**argv)
{
	std::string obj_fn = argc > 1 ? argv[1] : "";
	if (obj_fn.empty()) { return -1; }
	
	objl::Loader obj_loader;
	bool ret = obj_loader.LoadFile(obj_fn);
	if (!ret) {
		printf("load obj: %s failed\n", obj_fn.c_str());
		return -1;
	}
	MatrixMesh mesh;
	obj_loader.GetLoadedVerts(mesh.V, mesh.N, mesh.TC);
	obj_loader.LoadedMeshes[0].GetTriangleIndices(mesh.F, mesh.FTC, mesh.FN);
	
	remapMesh(mesh);
	printf("done!!!\n");
	return 0;
}