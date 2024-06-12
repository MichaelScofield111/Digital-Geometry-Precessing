#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "PolyMesh/IOManager.h"
#include "Algorithm.h"
#include <unistd.h>
#include <stdio.h>

using namespace acamcad;
using namespace polymesh;

int main(int argc, char** argv)
{
    if (argc != 4) {
        std::cout << "===error argument===\n";
        std::cout << "ShortestPath:	ACAM_mesh_HW1.exe	alien.obj	vertices.txt	path.txt\n";
		std::cout << "MST:		ACAM_mesh_HW1.exe	alien.obj	vertices.txt	path.txt\n";
		std::cout << "=================================================\n";
        return 0;
    }
    
    // sucessful argument
    // store information
    std::string mesh_path = argv[1];
    PolyMesh* mesh = new PolyMesh();
    loadMesh(mesh_path, mesh);
    std::cout << mesh->numVertices();
    std::ifstream _in(argv[2]);
    std::string line;
    std::vector<int> landmarks;

    while(std::getline(_in, line)) {
        std::stringstream ss;
        ss << line;
        int vid; // id
        ss >> vid;
        landmarks.push_back(vid);
    }

    std::vector<int> vertice, edges;
    if(landmarks.size() == 2){
        // only start vid to end vid
        // record the path
        std::vector<int> path;
        int sid = landmarks[0], eid = landmarks[1];
        Dijkstra(*mesh, sid, eid, path);
        vertice.push_back(path[0]);
		for (int i = 0; i < path.size() - 1; i++)
		{
			vertice.push_back(path[i + 1]);
			edges.push_back(mesh->edgeBetween(mesh->vert(path[i]), mesh->vert(path[i+1]))->index());
		}
    }
    else if(landmarks.size() > 2){
        std::vector<std::vector<int>> path;
        Dijkstra_group(*mesh, landmarks, path);
        for (auto a : path)
		{
			edges.push_back(mesh->edgeBetween(mesh->vert(a[0]), mesh->vert(a[1]))->index());
			for (int i = 1; i < a.size() - 1; i++)
			{
				edges.push_back(mesh->edgeBetween(mesh->vert(a[i]), mesh->vert(a[i+1]))->index());
				vertice.push_back(a[i]);
			}
		}
		for (auto a : landmarks)
			vertice.push_back(a);
    }else{
        std::cout << "error fail file input \n" << std::endl;
    }
    //out put 
    std::ofstream _out(argv[3]);
	_out << "VERTICES\n";
	for (auto a : vertice)
		_out << a << std::endl;
	_out << "EDGES\n";
	for (auto a : edges)
		_out << a << std::endl;
	_out.close();

    
}