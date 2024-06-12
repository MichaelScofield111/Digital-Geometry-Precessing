#pragma once
#include <PolyMesh/PolyMesh.h>
#include <PolyMesh/PolyMesh_Base.h>
#include <queue>
#include <vector>

using namespace acamcad;
using namespace polymesh;

struct node
{
    int id;
    double dis;

    node(int id, double d): id(id), dis(dis) {};
    // compare
    bool operator < (const node& w) const{
        return dis > w.dis;
    } 
};

struct PathInfo
{
    int sid, eid;
    double length;
    std::vector<int> path;

    PathInfo(int sid, int eid, double length){
        this->sid = sid;
        this->eid = eid;
        this->length = length;
    }
    
    bool operator<(const PathInfo& rhs) const
	{
		return length > rhs.length;
	}

};


void Dijkstra(PolyMesh& mesh, int sid, int eid, std::vector<int>& path);

void Dijkstra_group(PolyMesh& mesh, std::vector<int>& input, std::vector<std::vector<int>>& path);