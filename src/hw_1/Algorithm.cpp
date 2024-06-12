#include "Algorithm.h"
#include <algorithm>

#define DBL_MAX 2147483647

// compute sid to eid min distance in mesh
void Dijkstra(PolyMesh& Mesh, int s_p, int e_p, std::vector<int>& path)
{   
    // store dis and id in que
    std::priority_queue<node> que;

    std::vector<double> distance(Mesh.numVertices(), DBL_MAX);
    std::vector<int> st(Mesh.numVertices(), 0);
    std::vector<int> v_p(Mesh.numVertices(), -1);

    distance[s_p] = 0.0;
    que.push(node(s_p, 0.0));
    v_p[s_p] = s_p;
    
    while(que.size()){
        auto t = que.top();
        que.pop();

        
        if(st[t.id]) continue;
        if(t.id == e_p) break;

        st[t.id] = 1;
        // interator every next edge
        // get point of struct
        MVert* v1 = Mesh.vert(t.id);
        for(VertexVertexIter it = Mesh.vv_iter(v1); it.isValid(); it ++){
            MVert* v2 = *it;
            MEdge* e12 = Mesh.edgeBetween(v1, v2);
            if(distance[v2->index()] > distance[v1->index()] + e12->length())
            {
                //  update dis v2
                distance[v2->index()] = distance[v1->index()] + e12->length();
                // add v2
                que.push(node(v2->index(), distance[v2->index()]));
                // record v1 -> v2
                v_p[v2->index()] = v1->index();
            }
        }

    }

    // path is record all path from s_p to e_p
    path.clear();
    path.push_back(e_p);
    do{
        e_p = v_p[e_p];
        path.push_back(e_p);
    }while (e_p != path[e_p]);

}

void Dijkstra_group(PolyMesh& mesh, std::vector<int>& input, std::vector<std::vector<int>>& path)
{   
    std::sort(input.begin(), input.end());
    std::priority_queue<PathInfo> complete_graph;

    // get complete graph
    int n = input.size() - 1;
    for(int i = 0; i < n; i ++) {
        std::vector<int> is_lmk(mesh.numVertices(), 0);
        
        // point need to compute 
        for (int j = i; j < input.size(); j ++) is_lmk[input[j]] = 1;
        int count = input.size() - i;
        
        // start point
        int s_p = input[i];
        std::vector<int> st(mesh.numVertices(), 0);
        std::vector<double> distance(mesh.numVertices(), DBL_MAX);
        std::priority_queue<node> que;
        // record path
        std::vector<int> v_p(mesh.numVertices(), -1);
        v_p[s_p] = s_p;
        distance[s_p] = 0.0;
        que.push(node(s_p, 0));

        while(count != 0) {
            auto t = que.top();
            que.pop();

            if(st[t.id] == 1) continue;
            if(is_lmk[t.id] == 1) {
                // point t.id get min distance 
                count --;
                is_lmk[t.id] = 0;
            }

            st[t.id] = 1;
            MVert* v1 = mesh.vert(t.id);
            for (VertexVertexIter vv_it = mesh.vv_iter(v1); vv_it.isValid(); ++vv_it)
			{
				MVert* v2 = *vv_it;
				MEdge* e12 = mesh.edgeBetween(v1, v2);
				if (distance[v1->index()] + e12->length() < distance[v2->index()])
				{
					distance[v2->index()] = distance[v1->index()] + e12->length();
					que.push(node(v2->index(), distance[v2->index()]));
					v_p[v2->index()] = t.id;
				}
			}
        }
        // compute all min distance origin point is s_p
        for (int j = i + 1; j < input.size(); j ++){
            std::vector<int> p;
            // end point
            int e_p = input[j];
            double l = distance[e_p];
            p.push_back(e_p);
            do{
                e_p = v_p[e_p];
                p.push_back(e_p);
            }while (e_p != v_p[e_p]);
            PathInfo path_info(s_p, input[j], l);
            path_info.path = p;
            // build complete_graph
            complete_graph.push(path_info);
        }
    }
    int nv = input.size();
    std::vector<int> s_t_c(nv);
    // init gather
    for (int i = 0; i < nv; i ++) s_t_c[i] = i;

    int all_edges = 0;
    while(all_edges < nv - 1){
        // add edges in gather
        auto tmp = complete_graph.top();
        complete_graph.pop();
        // find s_p index and e_p index in input
        int m_id = -1, n_id = -1;
        for (int u = 0; u < nv; u ++){
            if (input[u] == tmp.sid) m_id = u;
            if (input[u] == tmp.eid) n_id = u; 
        }
        
        // add m_id --- n_id edges into gather
        m_id = s_t_c[m_id];
        n_id = s_t_c[n_id];
        // pre do 
        if (m_id < n_id) std::swap(m_id, n_id);

        if (m_id != n_id) {
            path.push_back(tmp.path);
            all_edges ++;
            for (int i = 0; i < nv; i ++) {
                if(s_t_c[i] == n_id) s_t_c[i] = m_id;
            }
        }
    }
}