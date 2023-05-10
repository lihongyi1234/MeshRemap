/* OBJ loader. Based on https://github.com/Bly7/OBJ-Loader
*/
#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <math.h>
#include <Eigen/Eigen>
#include <boost/filesystem.hpp>

// Print progress to console while loading (large models)
//#define OBJL_CONSOLE_OUTPUT

// Description: The namespace that holds eveyrthing that
//	is needed and used for the OBJ Model Loader
namespace objl
{
	struct Material
	{
		Material()
		{
			name;
			Ns = 0.0f;
			Ni = 0.0f;
			d = 0.0f;
			illum = 0;
		}

		// Material Name
		std::string name;
		// Ambient Color
		Eigen::Vector3f Ka;
		// Diffuse Color
		Eigen::Vector3f Kd;
		// Specular Color
		Eigen::Vector3f Ks;
		// Specular Exponent
		float Ns;
		// Optical Density
		float Ni;
		// Dissolve
		float d;
		// Illumination
		int illum;
		// Ambient Texture Map
		std::string map_Ka;
		// Diffuse Texture Map
		std::string map_Kd;
		// Specular Texture Map
		std::string map_Ks;
		// Specular Hightlight Map
		std::string map_Ns;
		// Alpha Texture Map
		std::string map_d;
		// Bump Map
		std::string map_bump;
	};

	// Structure: Mesh
	//
	// Description: A Simple Mesh Object that holds
	//	a name, a vertex list, and an index list
	struct Mesh
	{
		// Default Constructor
		Mesh()
		{}

		Mesh(std::vector<Eigen::Vector3i> &_PositionIndices,
			std::vector<Eigen::Vector3i> &_TextureIndices,
			std::vector<Eigen::Vector3i> &_NormalIndices)
			: PositionIndices(_PositionIndices), TextureIndices(_TextureIndices), NormalIndices(_NormalIndices)
		{
			for (int i = 0; i < TextureIndices.size(); ++i) {
				if (TextureIndices[i][0] < 0 || TextureIndices[i][1] < 0 || TextureIndices[i][2] < 0) {
					TextureIndices.clear();
					break;
				}
			}

			for (int i = 0; i < NormalIndices.size(); ++i) {
				if (NormalIndices[i][0] < 0 || NormalIndices[i][1] < 0 || NormalIndices[i][2] < 0) {
					NormalIndices.clear();
					break;
				}
			}
		}

		void GetTriangleIndices(Eigen::MatrixXi &F, Eigen::MatrixXi &FTC, Eigen::MatrixXi &FN)
		{
			F.resize(PositionIndices.size(), 3);
			for (int i = 0; i < F.rows(); ++i)
				F.row(i) = PositionIndices[i];

			FTC.resize(TextureIndices.size(), 3);
			for (int i = 0; i < FTC.rows(); ++i)
				FTC.row(i) = TextureIndices[i];

			FN.resize(NormalIndices.size(), 3);
			for (int i = 0; i < FN.rows(); ++i)
				FN.row(i) = NormalIndices[i];
		}

		// Mesh Name
		std::string MeshName;
		// Index List
		std::vector<Eigen::Vector3i> PositionIndices;
		std::vector<Eigen::Vector3i> TextureIndices;
		std::vector<Eigen::Vector3i> NormalIndices;
		// Material
		Material MeshMaterial;
	};

	// Namespace: Math
	//
	// Description: The namespace that holds all of the math
	//	functions need for OBJL
	namespace math
	{
		// Eigen::Vector3f Cross Product
		Eigen::Vector3f CrossV3(const Eigen::Vector3f a, const Eigen::Vector3f b)
		{
			return Eigen::Vector3f(a[1] * b[2] - a[2] * b[1],
				a[2] * b[0] - a[0] * b[2],
				a[0] * b[1] - a[1] * b[0]);
		}

		// Eigen::Vector3f Magnitude Calculation
		float MagnitudeV3(const Eigen::Vector3f in)
		{
			return (sqrtf(powf(in[0], 2) + powf(in[1], 2) + powf(in[2], 2)));
		}

		// Eigen::Vector3f DotProduct
		float DotV3(const Eigen::Vector3f a, const Eigen::Vector3f b)
		{
			return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
		}

		// Angle between 2 Eigen::Vector3f Objects
		float AngleBetweenV3(const Eigen::Vector3f a, const Eigen::Vector3f b)
		{
			float angle = DotV3(a, b);
			angle /= (MagnitudeV3(a) * MagnitudeV3(b));
			return angle = acosf(angle);
		}

		// Projection Calculation of a onto b
		Eigen::Vector3f ProjV3(const Eigen::Vector3f a, const Eigen::Vector3f b)
		{
			Eigen::Vector3f bn = b / MagnitudeV3(b);
			return bn * DotV3(a, bn);
		}
	}

	// Namespace: Algorithm
	//
	// Description: The namespace that holds all of the
	// Algorithms needed for OBJL
	namespace algorithm
	{
		// Eigen::Vector3f Multiplication Opertor Overload
		Eigen::Vector3f operator*(const float& left, const Eigen::Vector3f& right)
		{
			return Eigen::Vector3f(right[0] * left, right[1] * left, right[2] * left);
		}

		// A test to see if P1 is on the same side as P2 of a line segment ab
		bool SameSide(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f a, Eigen::Vector3f b)
		{
			Eigen::Vector3f cp1 = math::CrossV3(b - a, p1 - a);
			Eigen::Vector3f cp2 = math::CrossV3(b - a, p2 - a);

			if (math::DotV3(cp1, cp2) >= 0)
				return true;
			else
				return false;
		}

		// Generate a cross produect normal for a triangle
		Eigen::Vector3f GenTriNormal(Eigen::Vector3f t1, Eigen::Vector3f t2, Eigen::Vector3f t3)
		{
			Eigen::Vector3f u = t2 - t1;
			Eigen::Vector3f v = t3 - t1;

			Eigen::Vector3f normal = math::CrossV3(u,v);

			return normal;
		}

		// Check to see if a Eigen::Vector3f Point is within a 3 Eigen::Vector3f Triangle
		bool inTriangle(Eigen::Vector3f point, Eigen::Vector3f tri1, Eigen::Vector3f tri2, Eigen::Vector3f tri3)
		{
			// Test to see if it is within an infinite prism that the triangle outlines.
			bool within_tri_prisim = SameSide(point, tri1, tri2, tri3) && SameSide(point, tri2, tri1, tri3)
				&& SameSide(point, tri3, tri1, tri2);

			// If it isn't it will never be on the triangle
			if (!within_tri_prisim)
				return false;

			// Calulate Triangle's Normal
			Eigen::Vector3f n = GenTriNormal(tri1, tri2, tri3);

			// Project the point onto this normal
			Eigen::Vector3f proj = math::ProjV3(point, n);

			// If the distance from the triangle to the point is 0
			//	it lies on the triangle
			if (math::MagnitudeV3(proj) == 0)
				return true;
			else
				return false;
		}

		// Split a String into a string array at a given token
		inline void split(const std::string &in,
			std::vector<std::string> &out,
			std::string token)
		{
			out.clear();

			std::string temp;

			for (int i = 0; i < int(in.size()); i++)
			{
				std::string test = in.substr(i, token.size());

				if (test == token)
				{
					if (!temp.empty())
					{
						out.push_back(temp);
						temp.clear();
						i += (int)token.size() - 1;
					}
					else
					{
						out.push_back("");
					}
				}
				else if (i + token.size() >= in.size())
				{
					temp += in.substr(i, token.size());
					out.push_back(temp);
					break;
				}
				else
				{
					temp += in[i];
				}
			}
		}

		// Get tail of string after first token and possibly following spaces
		inline std::string tail(const std::string &in)
		{
			size_t token_start = in.find_first_not_of(" \t");
			size_t space_start = in.find_first_of(" \t", token_start);
			size_t tail_start = in.find_first_not_of(" \t", space_start);
			size_t tail_end = in.find_last_not_of(" \t");
			if (tail_start != std::string::npos && tail_end != std::string::npos)
			{
				return in.substr(tail_start, tail_end - tail_start + 1);
			}
			else if (tail_start != std::string::npos)
			{
				return in.substr(tail_start);
			}
			return "";
		}

		// Get first token of string
		inline std::string firstToken(const std::string &in)
		{
			if (!in.empty())
			{
				size_t token_start = in.find_first_not_of(" \t");
				size_t token_end = in.find_first_of(" \t", token_start);
				if (token_start != std::string::npos && token_end != std::string::npos)
				{
					return in.substr(token_start, token_end - token_start);
				}
				else if (token_start != std::string::npos)
				{
					return in.substr(token_start);
				}
			}
			return "";
		}

		// Get element at given index position
		template <class T>
		inline const T & getElement(const std::vector<T> &elements, std::string &index)
		{
			int idx = std::stoi(index);
			if (idx < 0)
				idx = int(elements.size()) + idx;
			else
				idx--;
			return elements[idx];
		}

		inline const int getIndex(std::string &index)
		{
			int idx = std::stoi(index) - 1;
			return idx;
		}
	}

	// Class: Loader
	//
	// Description: The OBJ Model Loader
	class Loader
	{
	public:
		// Default Constructor
		Loader()
		{

		}
		~Loader()
		{
			LoadedMeshes.clear();
		}

		// Load a file into the loader
		//
		// If file is loaded return true
		//
		// If the file is unable to be found
		// or unable to be loaded return false
		bool LoadFile(std::string Path)
		{
			// If the file is not an .obj file return false
			if (Path.substr(Path.size() - 4, 4) != ".obj")
				return false;


			std::ifstream file(Path);

			if (!file.is_open())
				return false;

			LoadedPath = Path;
			LoadedMeshes.clear();
			LoadedPositions.clear();
			LoadedNormals.clear();
			LoadedTCoords.clear();

			std::vector<Eigen::Vector3i> PositionIndices;
			std::vector<Eigen::Vector3i> NormalIndices;
			std::vector<Eigen::Vector3i> TextureIndices;

			std::vector<std::string> MeshMatNames;

			bool listening = false;
			std::string meshname;

			Mesh tempMesh;

			#ifdef OBJL_CONSOLE_OUTPUT
			const unsigned int outputEveryNth = 1000;
			unsigned int outputIndicator = outputEveryNth;
			#endif

			std::string curline;
			while (std::getline(file, curline))
			{
				#ifdef OBJL_CONSOLE_OUTPUT
				if ((outputIndicator = ((outputIndicator + 1) % outputEveryNth)) == 1)
				{
					if (!meshname.empty())
					{
						std::cout
							<< "\r- " << meshname
							<< "\t| vertices > " << LoadedPositions.size()
							<< "\t| texcoords > " << LoadedTCoords.size()
							<< "\t| normals > " << LoadedNormals.size()
							<< "\t| triangles > " << (PositionIndices.size() / 3)
							<< (!MeshMatNames.empty() ? "\t| material: " + MeshMatNames.back() : "");
					}
				}
				#endif

				// Generate a Mesh Object or Prepare for an object to be created
				if (algorithm::firstToken(curline) == "o" || algorithm::firstToken(curline) == "g" || curline[0] == 'g')
				{
					if (!listening)
					{
						listening = true;

						if (algorithm::firstToken(curline) == "o" || algorithm::firstToken(curline) == "g")
						{
							meshname = algorithm::tail(curline);
						}
						else
						{
							meshname = "unnamed";
						}
					}
					else
					{
						// Generate the mesh to put into the array

						if (!PositionIndices.empty() && !LoadedPositions.empty())
						{
							// Create Mesh
							tempMesh = Mesh(PositionIndices, TextureIndices, NormalIndices);
							tempMesh.MeshName = meshname;

							// Insert Mesh
							LoadedMeshes.push_back(tempMesh);

							// Cleanup
							PositionIndices.clear();
							NormalIndices.clear();
							TextureIndices.clear();
							meshname.clear();

							meshname = algorithm::tail(curline);
						}
						else
						{
							if (algorithm::firstToken(curline) == "o" || algorithm::firstToken(curline) == "g")
							{
								meshname = algorithm::tail(curline);
							}
							else
							{
								meshname = "unnamed";
							}
						}
					}
					#ifdef OBJL_CONSOLE_OUTPUT
					std::cout << std::endl;
					outputIndicator = 0;
					#endif
				}
				// Generate a Vertex Position
				if (algorithm::firstToken(curline) == "v")
				{
					std::vector<std::string> spos;
					Eigen::Vector3f vpos;
					algorithm::split(algorithm::tail(curline), spos, " ");

					vpos[0] = std::stod(spos[0]);
					vpos[1] = std::stod(spos[1]);
					vpos[2] = std::stod(spos[2]);

					LoadedPositions.push_back(vpos);
				}
				// Generate a Vertex Texture Coordinate
				if (algorithm::firstToken(curline) == "vt")
				{
					std::vector<std::string> stex;
					Eigen::Vector2f vtex;
					algorithm::split(algorithm::tail(curline), stex, " ");

					vtex[0] = std::stod(stex[0]);
					vtex[1] = std::stod(stex[1]);

					LoadedTCoords.push_back(vtex);
				}
				// Generate a Vertex Normal;
				if (algorithm::firstToken(curline) == "vn")
				{
					std::vector<std::string> snor;
					Eigen::Vector3f vnor;
					algorithm::split(algorithm::tail(curline), snor, " ");

					vnor[0] = std::stod(snor[0]);
					vnor[1] = std::stod(snor[1]);
					vnor[2] = std::stod(snor[2]);

					LoadedNormals.push_back(vnor);
				}
				// Generate a Face (vertices & indices)
				if (algorithm::firstToken(curline) == "f")
				{
					// Generate the vertices
					Eigen::Vector3i PositionIdx, TextureIdx, NormalIdx;
					ReadTriangleIndicesRawOBJ(PositionIdx, TextureIdx, NormalIdx, curline);
					PositionIndices.emplace_back(PositionIdx);
					TextureIndices.emplace_back(TextureIdx);
					NormalIndices.emplace_back(NormalIdx);
				}
				// Get Mesh Material Name
				if (algorithm::firstToken(curline) == "usemtl")
				{
					MeshMatNames.push_back(algorithm::tail(curline));

					// Create new Mesh, if Material changes within a group
					if (!PositionIndices.empty() && !LoadedPositions .empty())
					{
						// Create Mesh
						tempMesh = Mesh(PositionIndices, TextureIndices, NormalIndices);
						tempMesh.MeshName = meshname;
						int i = 2;
						while(1) {
							tempMesh.MeshName = meshname + "_" + std::to_string(i);

							for (auto &m : LoadedMeshes)
								if (m.MeshName == tempMesh.MeshName)
									continue;
							break;
						}

						// Insert Mesh
						LoadedMeshes.push_back(tempMesh);

						// Cleanup
						PositionIndices.clear();
						TextureIndices.clear();
						NormalIndices.clear();
					}

					#ifdef OBJL_CONSOLE_OUTPUT
					outputIndicator = 0;
					#endif
				}
				// Load Materials
				if (algorithm::firstToken(curline) == "mtllib")
				{
					// Generate LoadedMaterial

#if 0
					// Generate a path to the material file
					std::vector<std::string> temp;
					algorithm::split(Path, temp, "/");

					std::string pathtomat = "";

					if (temp.size() != 1)
					{
						for (int i = 0; i < temp.size() - 1; i++)
						{
							pathtomat += temp[i] + "/";
						}
					}
#else
					std::string pathtomat = boost::filesystem::path(Path).parent_path().string() + "/";
#endif

					pathtomat += algorithm::tail(curline);

					#ifdef OBJL_CONSOLE_OUTPUT
					std::cout << std::endl << "- find materials in: " << pathtomat << std::endl;
					#endif

					// Load Materials
					LoadMaterials(pathtomat);
				}
			}

			#ifdef OBJL_CONSOLE_OUTPUT
			std::cout << std::endl;
			#endif

			// Deal with last mesh

			if (!PositionIndices.empty() && !LoadedPositions.empty())
			{
				// Create Mesh
				tempMesh = Mesh(PositionIndices, TextureIndices, NormalIndices);
				tempMesh.MeshName = meshname;

				// Insert Mesh
				LoadedMeshes.push_back(tempMesh);
			}

			file.close();

			// Set Materials for each Mesh
			for (int i = 0; i < MeshMatNames.size(); i++)
			{
				std::string matname = MeshMatNames[i];

				// Find corresponding material name in loaded materials
				// when found copy material variables into mesh material
				for (int j = 0; j < LoadedMaterials.size(); j++)
				{
					if (LoadedMaterials[j].name == matname)
					{
						LoadedMeshes[i].MeshMaterial = LoadedMaterials[j];
						break;
					}
				}
			}

			if (LoadedMeshes.empty() && LoadedPositions.empty())
			{
				return false;
			}
			else
			{
				return true;
			}
		}

		void GetLoadedVerts(Eigen::MatrixXd &V, Eigen::MatrixXd &N, Eigen::MatrixXd &TC)
		{
			V.resize(LoadedPositions.size(), 3);
			for (int i = 0; i < V.rows(); ++i)
				for (int j = 0; j < 3; ++j)
					V(i, j) = LoadedPositions[i][j];

			N.resize(LoadedNormals.size(), 3);
			for (int i = 0; i < N.rows(); ++i)
				for (int j = 0; j < 3; ++j)
					N(i, j) = LoadedNormals[i][j];

			TC.resize(LoadedTCoords.size(), 2);
			for (int i = 0; i < TC.rows(); ++i)
				for (int j = 0; j < 2; ++j)
					TC(i, j) = LoadedTCoords[i][j];
		}

		std::string LoadedPath;

		// Loaded Mesh Objects
		std::vector<Mesh> LoadedMeshes;

		// Position Vector
		std::vector<Eigen::Vector3f> LoadedPositions;
		// Normal Vector
		std::vector<Eigen::Vector3f> LoadedNormals;
		// Texture Coordinate Vector
		std::vector<Eigen::Vector2f> LoadedTCoords;

		// Loaded Material Objects
		std::vector<Material> LoadedMaterials;

	private:
		// Read triangle indices from raw obj
		void ReadTriangleIndicesRawOBJ(Eigen::Vector3i &PositionIdx,
			Eigen::Vector3i &TextureIdx, Eigen::Vector3i &NormalIdx,
			std::string icurline)
		{
			std::vector<std::string> sface, svert;
			algorithm::split(algorithm::tail(icurline), sface, " ");

			if (sface.size() != 3) {
				printf("[OBJ Loader][ERROR] only triangle is supported!\n");
				return;
			}

			// For every given vertex do this
			for (int i = 0; i < int(sface.size()); i++)
			{
				// See What type the vertex is.
				int vtype;

				algorithm::split(sface[i], svert, "/");

				// Check for just position - v1
				if (svert.size() == 1)
				{
					// Only position
					vtype = 1;
				}

				// Check for position & texture - v1/vt1
				if (svert.size() == 2)
				{
					// Position & Texture
					vtype = 2;
				}

				// Check for Position, Texture and Normal - v1/vt1/vn1
				// or if Position and Normal - v1//vn1
				if (svert.size() == 3)
				{
					if (svert[1] != "")
					{
						// Position, Texture, and Normal
						vtype = 4;
					}
					else
					{
						// Position & Normal
						vtype = 3;
					}
				}

				// Calculate and store the vertex
				switch (vtype)
				{
				case 1: // P
					PositionIdx[i] = algorithm::getIndex(svert[0]);
					TextureIdx[i] = -1;
					NormalIdx[i] = -1;
					break;
				
				case 2: // P/T
					PositionIdx[i] = algorithm::getIndex(svert[0]);
					TextureIdx[i] = algorithm::getIndex(svert[1]);
					NormalIdx[i] = -1;
					break;
				
				case 3: // P//N
					PositionIdx[i] = algorithm::getIndex(svert[0]);
					TextureIdx[i] = -1;
					NormalIdx[i] = algorithm::getIndex(svert[2]);
					break;
				
				case 4: // P/T/N
					PositionIdx[i] = algorithm::getIndex(svert[0]);
					TextureIdx[i] = algorithm::getIndex(svert[1]);
					NormalIdx[i] = algorithm::getIndex(svert[2]);
					break;
				
				default:
					break;

				}
			}
		}

		// Load Materials from .mtl file
		bool LoadMaterials(std::string path)
		{
			// If the file is not a material file return false
			if (path.substr(path.size() - 4, path.size()) != ".mtl")
				return false;

			std::ifstream file(path);

			// If the file is not found return false
			if (!file.is_open())
				return false;

			Material tempMaterial;

			bool listening = false;

			// Go through each line looking for material variables
			std::string curline;
			while (std::getline(file, curline))
			{
				// new material and material name
				if (algorithm::firstToken(curline) == "newmtl")
				{
					if (!listening)
					{
						listening = true;

						if (curline.size() > 7)
						{
							tempMaterial.name = algorithm::tail(curline);
						}
						else
						{
							tempMaterial.name = "none";
						}
					}
					else
					{
						// Generate the material

						// Push Back loaded Material
						LoadedMaterials.push_back(tempMaterial);

						// Clear Loaded Material
						tempMaterial = Material();

						if (curline.size() > 7)
						{
							tempMaterial.name = algorithm::tail(curline);
						}
						else
						{
							tempMaterial.name = "none";
						}
					}
				}
				// Ambient Color
				if (algorithm::firstToken(curline) == "Ka")
				{
					std::vector<std::string> temp;
					algorithm::split(algorithm::tail(curline), temp, " ");

					if (temp.size() != 3)
						continue;

					tempMaterial.Ka[0] = std::stof(temp[0]);
					tempMaterial.Ka[1] = std::stof(temp[1]);
					tempMaterial.Ka[2] = std::stof(temp[2]);
				}
				// Diffuse Color
				if (algorithm::firstToken(curline) == "Kd")
				{
					std::vector<std::string> temp;
					algorithm::split(algorithm::tail(curline), temp, " ");

					if (temp.size() != 3)
						continue;

					tempMaterial.Kd[0] = std::stof(temp[0]);
					tempMaterial.Kd[1] = std::stof(temp[1]);
					tempMaterial.Kd[2] = std::stof(temp[2]);
				}
				// Specular Color
				if (algorithm::firstToken(curline) == "Ks")
				{
					std::vector<std::string> temp;
					algorithm::split(algorithm::tail(curline), temp, " ");

					if (temp.size() != 3)
						continue;

					tempMaterial.Ks[0] = std::stof(temp[0]);
					tempMaterial.Ks[1] = std::stof(temp[1]);
					tempMaterial.Ks[2] = std::stof(temp[2]);
				}
				// Specular Exponent
				if (algorithm::firstToken(curline) == "Ns")
				{
					tempMaterial.Ns = std::stof(algorithm::tail(curline));
				}
				// Optical Density
				if (algorithm::firstToken(curline) == "Ni")
				{
					tempMaterial.Ni = std::stof(algorithm::tail(curline));
				}
				// Dissolve
				if (algorithm::firstToken(curline) == "d")
				{
					tempMaterial.d = std::stof(algorithm::tail(curline));
				}
				// Illumination
				if (algorithm::firstToken(curline) == "illum")
				{
					tempMaterial.illum = std::stoi(algorithm::tail(curline));
				}
				// Ambient Texture Map
				if (algorithm::firstToken(curline) == "map_Ka")
				{
					tempMaterial.map_Ka = algorithm::tail(curline);
				}
				// Diffuse Texture Map
				if (algorithm::firstToken(curline) == "map_Kd")
				{
					tempMaterial.map_Kd = algorithm::tail(curline);
				}
				// Specular Texture Map
				if (algorithm::firstToken(curline) == "map_Ks")
				{
					tempMaterial.map_Ks = algorithm::tail(curline);
				}
				// Specular Hightlight Map
				if (algorithm::firstToken(curline) == "map_Ns")
				{
					tempMaterial.map_Ns = algorithm::tail(curline);
				}
				// Alpha Texture Map
				if (algorithm::firstToken(curline) == "map_d")
				{
					tempMaterial.map_d = algorithm::tail(curline);
				}
				// Bump Map
				if (algorithm::firstToken(curline) == "map_Bump" || algorithm::firstToken(curline) == "map_bump" || algorithm::firstToken(curline) == "bump")
				{
					tempMaterial.map_bump = algorithm::tail(curline);
				}
			}

			// Deal with last material

			// Push Back loaded Material
			LoadedMaterials.push_back(tempMaterial);

			// Test to see if anything was loaded
			// If not return false
			if (LoadedMaterials.empty())
				return false;
			// If so return true
			else
				return true;
		}
	};
}
