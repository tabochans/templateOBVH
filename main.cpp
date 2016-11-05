#include"Intersect.h"
#include"OBVH.h"
#include"tiny_obj_loader.h"
#include"stb_image_write.h"
#include<iostream>

using namespace Primitive;
using namespace mSec;
using namespace PTUtility;

std::vector<Polygon> Facets;


bool Load(const char* fileName){

	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	std::string err;
	bool ret = tinyobj::LoadObj(shapes, materials, err, fileName, NULL, 1);

	if (!err.empty()) {
		std::cerr << err << std::endl;
	}
	if (!ret) {
		std::cerr << "Failed to load/parse .obj.\n" << std::endl;
		return false;
	}

	for (int sps = 0; sps < shapes.size(); sps++){

		for (int idx = 0, face = 0; idx + 2 < shapes[0].mesh.indices.size(); idx += 3){

			Polygon facet(
				Vertex(
				Vec3(
				shapes[sps].mesh.positions[0 + 3 * shapes[0].mesh.indices[idx + 0]],
				shapes[sps].mesh.positions[1 + 3 * shapes[0].mesh.indices[idx + 0]],
				shapes[sps].mesh.positions[2 + 3 * shapes[0].mesh.indices[idx + 0]]),

				Vec3(
				shapes[sps].mesh.normals[0 + 3 * shapes[0].mesh.indices[idx + 0]],
				shapes[sps].mesh.normals[1 + 3 * shapes[0].mesh.indices[idx + 0]],
				shapes[sps].mesh.normals[2 + 3 * shapes[0].mesh.indices[idx + 0]]),

				Vec2(0, 0)
				),

				Vertex(
				Vec3(
				shapes[sps].mesh.positions[0 + 3 * shapes[0].mesh.indices[idx + 1]],
				shapes[sps].mesh.positions[1 + 3 * shapes[0].mesh.indices[idx + 1]],
				shapes[sps].mesh.positions[2 + 3 * shapes[0].mesh.indices[idx + 1]]),

				Vec3(
				shapes[sps].mesh.normals[0 + 3 * shapes[0].mesh.indices[idx + 1]],
				shapes[sps].mesh.normals[1 + 3 * shapes[0].mesh.indices[idx + 1]],
				shapes[sps].mesh.normals[2 + 3 * shapes[0].mesh.indices[idx + 1]]),

				Vec2(0, 0)
				),

				Vertex(
				Vec3(
				shapes[sps].mesh.positions[0 + 3 * shapes[0].mesh.indices[idx + 2]],
				shapes[sps].mesh.positions[1 + 3 * shapes[0].mesh.indices[idx + 2]],
				shapes[sps].mesh.positions[2 + 3 * shapes[0].mesh.indices[idx + 2]]),

				Vec3(
				shapes[sps].mesh.normals[0 + 3 * shapes[0].mesh.indices[idx + 2]],
				shapes[sps].mesh.normals[1 + 3 * shapes[0].mesh.indices[idx + 2]],
				shapes[sps].mesh.normals[2 + 3 * shapes[0].mesh.indices[idx + 2]]),

				Vec2(0, 0))
				);

			Facets.push_back(facet);
		}
	}

	return true;
}



int main(){
	
	char image[256 * 256 * 4];

	std::cout << "Load Object Start" << std::endl;
	Load("dragon.obj");
	std::cout << "Load Object End" << std::endl;


	std::cout << "Create OBVH Start" << std::endl;
	OBVH<Polygon> m_graph;
	m_graph.CreateGraph(Facets);
	std::cout << "Create OBVH End" << std::endl;


	std::cout << "Ray trace Start" << std::endl;
	const float SCALE = 2.0f;
	for (int i = 0; i < 256 * 256; i++){
		int x = i % 256;
		int y = 255 - i / 256;

		float rx = SCALE * (x / static_cast<float>(256) - 0.5f);
		float ry = SCALE * (y / static_cast<float>(256) - 0.5f);

		Ray ray(Vec3(rx, ry, -10), Vec3(0, 0, 1));
		std::vector<OBVH<Polygon>::IscData> result;

		int ss = m_graph.GetElementFromRay(ray, result);
		
		Vec3 Normal(0, 0, 0);
		if (ss > 0){
			Normal =
				result[0]._u * (result[0]._pElement->m_V[2].m_Normal - result[0]._pElement->m_V[0].m_Normal)
				+ result[0]._v * (result[0]._pElement->m_V[2].m_Normal - result[0]._pElement->m_V[0].m_Normal)
				+ result[0]._pElement->m_V[0].m_Normal;
			Normal.normalize();
		}


		int cl = std::max(0, std::min(255, (int)(255 * Normal.y())));
		
		
		image[i * 4 + 0] = 0;
		image[i * 4 + 1] = cl;
		image[i * 4 + 2] = cl;
		image[i * 4 + 3] = 255;



	}
	std::cout << "Ray trace End" << std::endl;


	stbi_write_png("result.png", 256, 256, 4, image, 256 * 4);


	return 0;
}