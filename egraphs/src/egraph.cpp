#include<egraphs/egraph.h>

using namespace std;

//TODO: This is an arbitrary size...
#define ARBITRARY_HASH_TABLE_SIZE 32*1024

EGraph::EGraph(EGraphDiscretize* eg_disc, int dimensions, int num_constants){
  eg_disc_ = eg_disc;
  num_dims_ = dimensions;
  num_constants_ = num_constants;
  num_edges_ = 0;
  num_components_ = 0;
  cluster_radius_ = 1.0;
  search_iteration_ = 0;

  hashtable.resize(ARBITRARY_HASH_TABLE_SIZE);
}

EGraph::EGraph(EGraphDiscretize* eg_disc, string filename){
  eg_disc_ = eg_disc;
  hashtable.resize(ARBITRARY_HASH_TABLE_SIZE);
  load(filename);
}

EGraph::EGraph(string filename, string stats_filename){
  hashtable.resize(ARBITRARY_HASH_TABLE_SIZE);
  load(filename);
  loadStats(stats_filename);
}

EGraph::~EGraph(){
  clearEGraph();
}

void EGraph::clearEGraph(){
  boost::recursive_mutex::scoped_lock lock(egraph_mutex_);
  hashtable.clear();
  hashtable.resize(ARBITRARY_HASH_TABLE_SIZE);

  for(unsigned int i=0; i<id2vertex.size(); i++)
    delete id2vertex[i];

  id2vertex.clear();
  num_edges_ = 0;
}

//recordStats
//takes the vector of coords in the path
//increments counts on egraph edges that were used
void EGraph::recordStats(vector<vector<double> >& coords){
  boost::recursive_mutex::scoped_lock lock(egraph_mutex_);
  if(coords.size() < 2){
    ROS_WARN("[EGraph] Less than 2 points in the path so there are no edges. No stats to record.");
    return;
  }

  //convert continuous coordinates to discrete ones
  vector<vector<int> > disc_coords;
  for(unsigned int i=0; i<coords.size(); i++){
    if(num_dims_+num_constants_ != int(coords[i].size())){
      ROS_ERROR("[EGraph] There is a coordinate in the path that doesn't have enough fields!");
      return;
    }
    vector<int> dc;
    contToDisc(coords[i], dc);
    disc_coords.push_back(dc);
  }

  //for each edge that already exists in the egraph, increment the use count
  vector<EGraphVertex*> path_vertices;
  for(unsigned int i=1; i<disc_coords.size(); i++){
    EGraphVertex* v1 = getVertex(disc_coords[i-1]);
    EGraphVertex* v2 = getVertex(disc_coords[i]);
    if(v1 && v2){
      for(unsigned int i=0; i<v1->neighbors.size(); i++){
        if(v1->neighbors[i]->id == v2->id){
          v1->use_frequency[i]++;
          break;
        }
      }
      for(unsigned int i=0; i<v2->neighbors.size(); i++){
        if(v2->neighbors[i]->id == v1->id){
          v2->use_frequency[i]++;
          break;
        }
      }
    }
  }
}

//resetStats
//zeros the counts on egraph edge usage
void EGraph::resetStats(){
  for(unsigned int i=0; i<id2vertex.size(); i++){
    EGraphVertex* v = id2vertex[i];
    for(unsigned int j=0; j<v->use_frequency.size(); j++){
      v->use_frequency[i] = 0;
    }
  }
}

//prune
//takes the desired max number of edges, and a pruning method
//method 0 - drop random edges
//method 1 - drop least used edges
//method 2 - drop least used edges that don't disconnect the graph
//method 3 - drop edges that are not c-space diverse enough
void EGraph::prune(int max_size, int method){
  if(num_edges_ <= max_size){
    ROS_INFO("EGraph is already within the max size. No pruning required.");
    return;
  }

  ROS_INFO("Pruning EGraph of %d edges down to %d edges using method %d",num_edges_,max_size,method);

  if(method==0){
    ROS_INFO("Prune random edges...");

    //build a list of all the edges
    ROS_INFO("build list of edges...");
    vector<pair<int,int> > edge_list;
    for(unsigned int i=0; i<id2vertex.size(); i++){
      EGraphVertex* v = id2vertex[i];
      for(unsigned int j=0; j<v->neighbors.size(); j++){
        if(v->id < v->neighbors[j]->id){
          pair<int,int> p;
          p.first = v->id;
          p.second = v->neighbors[j]->id;
          edge_list.push_back(p);
        }
      }
    }
    //randomly permute the list
    ROS_INFO("randomly permute the list...");
    for(unsigned int i=0; i<edge_list.size()-1; i++){
      int idx = int(rand()%(edge_list.size()-1-i))+i;
      //ROS_INFO("idx %d, size %d",idx,edge_list.size());
      pair<int,int> temp = edge_list[i];
      edge_list[i] = edge_list[idx];
      edge_list[idx] = temp;
    }
    //remove the first k edges in the list
    ROS_INFO("remove the first k edges...");
    int k = num_edges_ - max_size;
    for(int i=0; i<k; i++){
      int id1 = edge_list[i].first;
      int id2 = edge_list[i].second;

      EGraphVertex* v1 = id2vertex[id1];
      int idx = -1;
      for(unsigned int j=0; j<v1->neighbors.size(); j++){
        if(v1->neighbors[j]->id == id2){
          idx = j;
          break;
        }
      }
      v1->neighbors.erase(v1->neighbors.begin()+idx);
      v1->costs.erase(v1->costs.begin()+idx);
      v1->use_frequency.erase(v1->use_frequency.begin()+idx);

      EGraphVertex* v2 = id2vertex[id2];
      if(v1->id!=v2->id){
        idx = -1;
        for(unsigned int j=0; j<v2->neighbors.size(); j++){
          if(v2->neighbors[j]->id == id1){
            idx = j;
            break;
          }
        }
        v2->neighbors.erase(v2->neighbors.begin()+idx);
        v2->costs.erase(v2->costs.begin()+idx);
        v2->use_frequency.erase(v2->use_frequency.begin()+idx);
      }
      num_edges_--;
    }
  }
  else if(method==1){
    ROS_INFO("Prune least used edges...");

    //build a list of all the edges (with use frequency)
    vector<pair<int,pair<int,int> > > edge_list;
    for(unsigned int i=0; i<id2vertex.size(); i++){
      EGraphVertex* v = id2vertex[i];
      for(unsigned int j=0; j<v->use_frequency.size(); j++){
        if(v->id < v->neighbors[j]->id){
          pair<int,pair<int,int> > p;
          p.first = v->use_frequency[j];
          p.second.first = v->id;
          p.second.second = v->neighbors[j]->id;
          edge_list.push_back(p);
        }
      }
    }
    //sort edge list from lowest use to highest
    sort(edge_list.begin(),edge_list.end());

    int k = num_edges_ - max_size;
    for(int i=0; i<k; i++){
      int id1 = edge_list[i].second.first;
      int id2 = edge_list[i].second.second;

      EGraphVertex* v1 = id2vertex[id1];
      int idx = -1;
      for(unsigned int j=0; j<v1->neighbors.size(); j++){
        if(v1->neighbors[j]->id == id2){
          idx = j;
          break;
        }
      }
      v1->neighbors.erase(v1->neighbors.begin()+idx);
      v1->costs.erase(v1->costs.begin()+idx);
      v1->use_frequency.erase(v1->use_frequency.begin()+idx);

      EGraphVertex* v2 = id2vertex[id2];
      if(v1->id!=v2->id){
        idx = -1;
        for(unsigned int j=0; j<v2->neighbors.size(); j++){
          if(v2->neighbors[j]->id == id1){
            idx = j;
            break;
          }
        }
        v2->neighbors.erase(v2->neighbors.begin()+idx);
        v2->costs.erase(v2->costs.begin()+idx);
        v2->use_frequency.erase(v2->use_frequency.begin()+idx);
      }
      num_edges_--;
    }
  }
  else if(method==2){
    ROS_INFO("Prune least used edges that don't disconnect the graph...");
    
    //initialize the heap with vertices of degree 1
    ROS_INFO("initialize heap with vertices of degree 1");
    CHeap heap;
    vector<EGraphVertexHeapElement*> element_list;
    heap.makeemptyheap();
    for(unsigned int i=0; i<id2vertex.size(); i++){
      EGraphVertex* v = id2vertex[i];
      if(v->neighbors.size()==1){
        //add vertex id and  edge cost to the CHeap
        EGraphVertexHeapElement* e = new EGraphVertexHeapElement();
        e->id = v->id;
        e->heapindex = 0;
        element_list.push_back(e);
        CKey key;
        key.key[0] = v->use_frequency.front();
        //ROS_INFO("insert %d with key %d",e->id,key.key[0]);
        heap.insertheap(e,key);
      }
    }
    //repeat until we are under the max number of edges
    ROS_INFO("repeat until we are under the max edge count");
    while(num_edges_ > max_size){
      //ROS_ERROR("\n the size %d\n",id2vertex[4330]->use_frequency.size());
      if(!heap.emptyheap()){
        ROS_INFO("using heap");
        EGraphVertexHeapElement* e = (EGraphVertexHeapElement*)heap.deleteminheap();
        EGraphVertex* v1 = id2vertex[e->id];
        if(v1->neighbors.size() != 1)
          continue;
        EGraphVertex* v2 = v1->neighbors.front();
        v1->neighbors.clear();
        v1->costs.clear();
        v1->use_frequency.clear();

        if(v2->id!=v1->id){
          int idx = -1;
          for(unsigned int i=0; i<v2->neighbors.size(); i++){
            if(v2->neighbors[i]->id == v1->id){
              idx = i;
              break;
            }
          }
          v2->neighbors.erase(v2->neighbors.begin()+idx);
          v2->costs.erase(v2->costs.begin()+idx);
          v2->use_frequency.erase(v2->use_frequency.begin()+idx);
          if(v2->neighbors.size() == 1){
            EGraphVertexHeapElement* e2 = new EGraphVertexHeapElement();
            e2->id = v2->id;
            e2->heapindex = 0;
            element_list.push_back(e2);
            CKey key;
            key.key[0] = v2->use_frequency.front();
            heap.insertheap(e2,key);
          }
        }
        num_edges_--;
      }
      else{
        ROS_INFO("remove global min edge (no leaves)");
        int min_freq = INFINITECOST;
        int id1 = -1;
        int id2 = -1;
        //ROS_INFO("0");
        for(unsigned int i=0; i<id2vertex.size(); i++){
          EGraphVertex* v = id2vertex[i];
          //ROS_INFO("(%d=%d)->use_freq %d",i,v->id,v->use_frequency.size());
          for(unsigned int j=0; j<v->use_frequency.size(); j++){
            if(v->use_frequency[j] < min_freq){
              //ROS_INFO("%d = %d (%d)",v->use_frequency.size(),v->neighbors.size(),j);
              min_freq = v->use_frequency[j];
              id1 = v->id;
              id2 = v->neighbors[j]->id;
            }
          }
        }
        //ROS_INFO("1");
        EGraphVertex* v1 = id2vertex[id1];
        int idx = -1;
        for(unsigned int j=0; j<v1->neighbors.size(); j++){
          if(v1->neighbors[j]->id == id2){
            idx = j;
            break;
          }
        }
        //ROS_INFO("2");
        v1->neighbors.erase(v1->neighbors.begin()+idx);
        v1->costs.erase(v1->costs.begin()+idx);
        v1->use_frequency.erase(v1->use_frequency.begin()+idx);
        if(v1->neighbors.size() == 1){
          EGraphVertexHeapElement* e = new EGraphVertexHeapElement();
          e->id = v1->id;
          e->heapindex = 0;
          element_list.push_back(e);
          CKey key;
          key.key[0] = v1->use_frequency.front();
          heap.insertheap(e,key);
        }
        //ROS_INFO("3");

        EGraphVertex* v2 = id2vertex[id2];
        if(v1->id!=v2->id){
          idx = -1;
          for(unsigned int j=0; j<v2->neighbors.size(); j++){
            if(v2->neighbors[j]->id == id1){
              idx = j;
              break;
            }
          }
          //ROS_INFO("4");
          v2->neighbors.erase(v2->neighbors.begin()+idx);
          v2->costs.erase(v2->costs.begin()+idx);
          v2->use_frequency.erase(v2->use_frequency.begin()+idx);
          if(v2->neighbors.size() == 1){
            EGraphVertexHeapElement* e = new EGraphVertexHeapElement();
            e->id = v2->id;
            e->heapindex = 0;
            element_list.push_back(e);
            CKey key;
            key.key[0] = v2->use_frequency.front();
            heap.insertheap(e,key);
          }
        }
        num_edges_--;
        //ROS_INFO("5");
      }
    }
  }
  else if(method==3){
    ROS_INFO("Prune edges that are not c-space diverse...");
    double dist = cluster_radius_;
    vector<pair<int,int> > d;
    for(unsigned int i=0; i<id2vertex.size(); i++){
      int count = 0;
      EGraphVertex* v1 = id2vertex[i];
      for(unsigned int j=0; j<id2vertex.size(); j++){
        double temp = 0;
        EGraphVertex* v2 = id2vertex[j];
        for(unsigned int k=0; k<v1->coord.size(); k++)
          temp += (v1->coord[k] - v2->coord[k])*(v1->coord[k] - v2->coord[k]);
        if(temp <= dist)
          count++;
      }
      d.push_back(pair<int,int>(count,v1->id));
    }
    sort(d.begin(),d.end());

    //remove edges incident to vertices with the highest counts
    for(int i=d.size()-1; num_edges_>max_size; i--){
      EGraphVertex* v = id2vertex[i];
      for(unsigned int j=0; j<v->neighbors.size(); j++){
        
        EGraphVertex* v2 = v->neighbors[j];
        if(v2->id!=v->id){
          int idx = -1;
          for(unsigned int k=0; k<v2->neighbors.size(); k++){
            if(v2->neighbors[k]->id == v->id){
              idx = k;
              break;
            }
          }
          v2->neighbors.erase(v2->neighbors.begin()+idx);
          v2->costs.erase(v2->costs.begin()+idx);
          v2->use_frequency.erase(v2->use_frequency.begin()+idx);
        }
        num_edges_--;
      }
      v->neighbors.clear();
      v->costs.clear();
      v->use_frequency.clear();
    }
  }
  else
    ROS_ERROR("Invalid pruning method provided...");

  computeComponents();
}

bool EGraph::addPaths(const vector<vector<vector<double> > >& paths, const vector<vector<int> >& path_costs){
  if(paths.size() != path_costs.size()){
    ROS_ERROR("The number of paths must be the same as the number of path cost vectors");
    return false;
  }

  for(unsigned int i=0; i<paths.size(); i++)
    addPathHelper(paths[i],path_costs[i]);

  computeComponents();

  ROS_INFO("[EGraph] addPaths complete. EGraph now contains %d vertices",int(id2vertex.size()));
  return true;
}

bool EGraph::addPath(const vector<vector<double> >& coords, const vector<int>& costs){
  if(!addPathHelper(coords, costs))
    return false;

  computeComponents();

  ROS_INFO("[EGraph] addPath complete. EGraph now contains %d vertices",int(id2vertex.size()));
  return true;
}

bool EGraph::addPathHelper(const vector<vector<double> >& coords, const vector<int>& costs){
  boost::recursive_mutex::scoped_lock lock(egraph_mutex_);
  //error checking
  if(coords.size() < 2){
    ROS_WARN("[EGraph] Less than 2 points in the added path...doing nothing.");
    return true;
  }
  if(costs.size() != coords.size()-1){
    ROS_ERROR("[EGraph] When giving a path to the E-Graph, it should have one less cost than it has coordinates");
    return false;
  }
  
  //convert continuous coordinates to discrete ones
  vector<vector<int> > disc_coords;
  vector<vector<double> > constants;
  for(unsigned int i=0; i<coords.size(); i++){
    if(num_dims_+num_constants_ != int(coords[i].size())){
      ROS_ERROR("[EGraph] There is a coordinate in the path that doesn't have enough fields!");
      ROS_ERROR("[EGraph] Expecting %d, got coordinate of size %lu", num_dims_+num_constants_, coords[i].size());
      return false;
    }
    vector<int> dc;
    contToDisc(coords[i], dc);
    disc_coords.push_back(dc);
    vector<double> temp;
    for(unsigned int j=num_dims_; j<coords[i].size(); j++)
      temp.push_back(coords[i][j]);
    constants.push_back(temp);
  }

  //if a vertex is not in the graph add it
  vector<EGraphVertex*> path_vertices;
  for(unsigned int i=0; i<disc_coords.size(); i++){
    EGraphVertex* v = getVertex(disc_coords[i]);
    if(!v)
      v = createVertex(disc_coords[i],constants[i]);
    path_vertices.push_back(v);
  }

  //add each edge
  for(unsigned int i=1; i<path_vertices.size(); i++){
    //ROS_INFO("adding %d->%d at a cost of %d",path_vertices[i-1]->id,path_vertices[i]->id,costs[i-1]);
    addEdge(path_vertices[i-1],path_vertices[i],costs[i-1]);
  }

  return true;
}

void EGraph::computeComponents(){
  //compute connected components
  for(unsigned int i=0; i<id2vertex.size(); i++)
    id2vertex[i]->component = -1;
  num_components_ = 0;
  vector<EGraphVertex*> q;
  for(unsigned int i=0; i<id2vertex.size(); i++){
    EGraphVertex* v = id2vertex[i];
    if(v->component<0){
      q.push_back(v);
      v->component = num_components_;
      while(!q.empty()){
        v = q.back();
        q.pop_back();
        for(unsigned int j=0; j<v->neighbors.size(); j++){
          EGraphVertex* u = v->neighbors[j];
          if(v->valid[j] && u->component<0){
            u->component = num_components_;
            q.push_back(u);
          }
        }
      }
      num_components_++;
    }
  }
  ROS_INFO("[EGraph] there are %d components",num_components_);
}

//print E-Graph
void EGraph::print(){
  boost::recursive_mutex::scoped_lock lock(egraph_mutex_);
  for(unsigned int i=0; i<id2vertex.size(); i++){
    EGraphVertex* v = id2vertex[i];
    printf("id:%d coord:(",v->id);
    for(unsigned int j=0; j<v->coord.size(); j++)
      printf("%d,",v->coord[j]);
    printf(") constants:(");
    for(unsigned int j=0; j<v->constants.size(); j++)
      printf("%f,",v->constants[j]);
    printf(") neighbors:{");
    for(unsigned int j=0; j<v->neighbors.size(); j++)
      printf("(%d,%d),",v->neighbors[j]->id,v->costs[j]);
    printf("}\n");
  }
}

bool EGraph::saveStats(string filename){
  boost::recursive_mutex::scoped_lock lock(egraph_mutex_);
  FILE* fout = fopen(filename.c_str(),"w");
  if(!fout){
    ROS_ERROR("Could not open file \"%s\" to save E-Graph stats",filename.c_str());
    fclose(fout);
    return false;
  }

  fprintf(fout,"%d\n",int(id2vertex.size()));
  for(unsigned int i=0; i<id2vertex.size(); i++){
    EGraphVertex* v = id2vertex[i];
    fprintf(fout,"%d ",int(v->use_frequency.size()));
    for(unsigned int j=0; j<v->use_frequency.size(); j++)
      fprintf(fout,"%d ",v->use_frequency[j]);
    fprintf(fout,"\n");
  }
  fclose(fout);
  return true;
}

bool EGraph::loadStats(string filename){
  boost::recursive_mutex::scoped_lock lock(egraph_mutex_);
  FILE* fin = fopen(filename.c_str(),"r");
  if(!fin){
    ROS_ERROR("Could not open file \"%s\" to load E-Graph stats",filename.c_str());
    fclose(fin);
    return false;
  }

  //read in the number of vertices
  int num_vertices;
  if(fscanf(fin,"%d", &num_vertices) != 1){
    ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...",filename.c_str());
    fclose(fin);
    return false;
  }
  if(num_vertices!=int(id2vertex.size())){
    ROS_ERROR("The EGraph stats file \"%s\" does not correspond to the currently loaded EGraph (different number of vertices)",filename.c_str());
    fclose(fin);
    return false;
  }
  for(int i=0; i<num_vertices; i++){
    EGraphVertex* v = id2vertex[i];
    int num_neighbors;
    if(fscanf(fin,"%d", &num_neighbors) != 1){
      ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...",filename.c_str());
      fclose(fin);
      return false;
    }
    if(num_neighbors!=int(v->neighbors.size())){
      ROS_ERROR("The EGraph stats file \"%s\" does not correspond to the currently loaded EGraph (vertex %d has a different number of neighbors)",filename.c_str(),i);
      fclose(fin);
      return false;
    }

    //read in the neighbors
    int freq;
    for(int j=0; j<num_neighbors; j++){
      if(fscanf(fin,"%d",&freq) != 1){
        ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...",filename.c_str());
        fclose(fin);
        return false;
      }
      v->use_frequency[j] = freq;
    }
  }
  fclose(fin);
  return true;
}

bool EGraph::save(string filename){
  boost::recursive_mutex::scoped_lock lock(egraph_mutex_);
  FILE* fout = fopen(filename.c_str(),"w");
  if(!fout){
    ROS_ERROR("Could not open file \"%s\" to save E-Graph",filename.c_str());
    fclose(fout);
    return false;
  }

  //write the number of dimensions and constants in the state
  fprintf(fout,"%d %d\n",num_dims_,num_constants_);

  //save the id2vertex table
  //one line for each vertex
  //"coord_values num_neighbors neighbor_ids costs_to_neighbors"
  fprintf(fout,"%d\n",int(id2vertex.size()));
  for(unsigned int i=0; i<id2vertex.size(); i++){
    EGraphVertex* v = id2vertex[i];
    vector<double> coord;
    discToCont(v,coord);
    for(unsigned int j=0; j<coord.size(); j++)
      fprintf(fout,"%f ",coord[j]);
    fprintf(fout,"%d ",int(v->neighbors.size()));
    for(unsigned int j=0; j<v->neighbors.size(); j++)
      fprintf(fout,"%d ",v->neighbors[j]->id);
    for(unsigned int j=0; j<v->costs.size(); j++)
      fprintf(fout,"%d ",v->costs[j]);
    for(unsigned int j=0; j<v->valid.size(); j++){

      if (v->valid[j]){ assert(v->costs[j] > 0); }

      fprintf(fout,"%d ",(v->valid[j]==true));
    }
    fprintf(fout,"\n");
  }

  fclose(fout);
  return true;
}

bool EGraph::load(string filename, bool clearCurrentEGraph){
  boost::recursive_mutex::scoped_lock lock(egraph_mutex_);
  if(clearCurrentEGraph)
    clearEGraph();
  else{
    ROS_ERROR("Loading an E-Graph without clearing the old one is not supported yet...");
    return false;
  }

  FILE* fin = fopen(filename.c_str(),"r");
  if(!fin){
    ROS_ERROR("Could not open file \"%s\" to load E-Graph",filename.c_str());
    fclose(fin);
    return false;
  }

  //read the number of dimensions
  if(fscanf(fin,"%d", &num_dims_) != 1){
    ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...1",filename.c_str());
    fclose(fin);
    return false;
  }

  //read the number of constants
  if(fscanf(fin,"%d", &num_constants_) != 1){
    ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...2",filename.c_str());
    fclose(fin);
    return false;
  }

  //read in the number of vertices
  int num_vertices;
  if(fscanf(fin,"%d", &num_vertices) != 1){
    ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...4",filename.c_str());
    fclose(fin);
    return false;
  }
  for(int i=0; i<num_vertices; i++){
    EGraphVertex* v = new EGraphVertex();
    v->id = i;
    id2vertex.push_back(v);
  }

  //read in each vertex
  for(int i=0; i<num_vertices; i++){
    //printf("vertex %d\n",i);
    //read in the vertex coordinate
    EGraphVertex* v = id2vertex[i];
    //printf("1\n");
    double val;
    vector<double> coord;
    for(int j=0; j<num_dims_; j++){
      if(fscanf(fin,"%lf",&val) != 1){
        ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...5",filename.c_str());
        fclose(fin);
        return false;
      }
      coord.push_back(val);
    }
    contToDisc(coord,v->coord);
    for(int j=0; j<num_constants_; j++){
      if(fscanf(fin,"%lf",&val) != 1){
        ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...6",filename.c_str());
        fclose(fin);
        return false;
      }
      v->constants.push_back(val);
    }
    //printf("2\n");
    int idx = getHashBin(v->coord);
    //printf("3\n");
    hashtable[idx].push_back(v);
    //printf("4\n");

    //printf("  read num neighbors\n");
    //read in the number of neighbors
    int num_neighbors;
    if(fscanf(fin,"%d", &num_neighbors) != 1){
      ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...7",filename.c_str());
      fclose(fin);
      return false;
    }

    //printf("  read in neighbors\n");
    //read in the neighbors
    int id;
    for(int j=0; j<num_neighbors; j++){
      if(fscanf(fin,"%d",&id) != 1){
        ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...8",filename.c_str());
        fclose(fin);
        return false;
      }
      v->neighbors.push_back(id2vertex[id]);
      //v->valid.push_back(true);
      v->use_frequency.push_back(0);
      if(v->id < id)
        num_edges_++;
    }

    //printf("  read in costs\n");
    //read in the costs to the neighbors
    int cost;
    for(int j=0; j<num_neighbors; j++){
      if(fscanf(fin,"%d",&cost) != 1){
        ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...9",filename.c_str());
        fclose(fin);
        return false;
      }
      v->costs.push_back(cost);
    }
    
    //printf("  read in valid\n");
    //read in the valid to the neighbors
    int valid;
    for(int j=0; j<num_neighbors; j++){
      if(fscanf(fin,"%d",&valid) != 1){
        ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...10",filename.c_str());
        fclose(fin);
        return false;
      }
      v->valid.push_back(valid==true);
    }
  }
  fclose(fin);

  ROS_INFO("finished reading in egraph of size %lu", id2vertex.size());
  computeComponents();

  return true;
}

void EGraph::updateEdge(EGraphVertex* v1, EGraphVertex* v2, bool valid, int cost){
  for(unsigned int i=0; i<v1->neighbors.size(); i++){
    if(v1->neighbors[i]==v2){
      v1->valid[i] = valid;
      v1->costs[i] = cost;
    }
  }
  for(unsigned int i=0; i<v2->neighbors.size(); i++){
    if(v2->neighbors[i]==v1){
      v2->valid[i] = valid;
      v2->costs[i] = cost;
    }
  }
}

void EGraph::updateEdge(EGraphVertex* v1, EGraphVertex* v2, bool valid){
  for(unsigned int i=0; i<v1->neighbors.size(); i++)
    if(v1->neighbors[i]==v2)
      v1->valid[i] = valid;
  for(unsigned int i=0; i<v2->neighbors.size(); i++)
    if(v2->neighbors[i]==v1)
      v2->valid[i] = valid;
}

void EGraph::invalidateVertex(EGraphVertex* v1){
  for (size_t i=0; i < v1->neighbors.size(); i++){
      EGraphVertex* v2 = v1->neighbors[i];
      for (size_t j=0; j < v2->neighbors.size(); j++){
          if (v2->neighbors[j] == v1){
              v2->valid[j] = false;
          }
      }
      v1->valid[i] = false;
  }
}

unsigned int EGraph::inthash(unsigned int key){
  key += (key << 12);
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}

int EGraph::getHashBin(vector<int>& coord){
  int hash = 0;
  for(int i=0; i<num_dims_; i++){
    hash += inthash(coord[i])<<i;
  }
  return inthash(hash) & (hashtable.size()-1);
}

EGraph::EGraphVertex* EGraph::getVertex(vector<int>& coord){
  int idx = getHashBin(coord);
  for(unsigned int i=0; i<hashtable[idx].size(); i++){
    bool isEqual = true;
    for(unsigned int j=0; j<hashtable[idx][i]->coord.size(); j++){
      if(coord[j]!=hashtable[idx][i]->coord[j]){
        isEqual = false;
        break;
      }
    }
    if(isEqual)
      return hashtable[idx][i];
  }
  return NULL;
}

EGraph::EGraphVertex* EGraph::createVertex(vector<int>& coord, vector<double>& constants){
  EGraphVertex* v = new EGraphVertex();
  v->coord = coord;
  v->constants = constants;
  v->id = id2vertex.size();
  id2vertex.push_back(v);
  int idx = getHashBin(coord);
  hashtable[idx].push_back(v);
  return v;
}

void EGraph::addEdge(EGraphVertex* v1, EGraphVertex* v2, int cost){
  bool done = false;
  for(unsigned int i=0; i<v1->neighbors.size(); i++){
    if(v1->neighbors[i]==v2){
      if(cost < v1->costs[i]){
        ROS_WARN("[EGraph] This edge already exists, but the new one is cheaper. Overwriting...%d->%d",v1->costs[i],cost);
        v1->costs[i] = cost;
        v1->valid[i] = true;
      }
      done = true;
      break;
    }
  }
  if(!done){
    v1->neighbors.push_back(v2);
    v1->costs.push_back(cost);
    v1->valid.push_back(true);
    v1->use_frequency.push_back(0);
    num_edges_++;
  }

  done = false;
  for(unsigned int i=0; i<v2->neighbors.size(); i++){
    if(v2->neighbors[i]==v1){
      if(cost < v2->costs[i]){
        ROS_WARN("[EGraph] This edge already exists, but the new one is cheaper. Overwriting...%d->%d",v2->costs[i],cost);
        v2->costs[i] = cost;
        v2->valid[i] = true;
      }
      done = true;
      break;
    }
  }
  if(!done){
    v2->neighbors.push_back(v1);
    v2->costs.push_back(cost);
    v2->valid.push_back(true);
    v2->use_frequency.push_back(0);
  }
}

void EGraph::discToCont(EGraphVertex* v, vector<double>& c){
  c.clear();
  vector<int> coord = v->coord;
  coord.resize(num_dims_);
  eg_disc_->discToCont(coord,c);
  //for(unsigned int i=0; i<v->coord.size(); i++)
    //c.push_back(v->coord[i]*res_[i]+min_[i]);
  for(unsigned int i=0; i<v->constants.size(); i++)
    c.push_back(v->constants[i]);
}

//double round(double r) {
  //return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
//}

void EGraph::contToDisc(vector<double> c, vector<int>& d){
  d.clear();
  vector<double> coord = c;
  coord.resize(num_dims_);
  eg_disc_->contToDisc(coord,d);
  //for(unsigned int i=0; i<res_.size(); i++)
    //d.push_back(round((c[i]-min_[i])/(res_[i])));
}

void EGraph::clearShortestPathCache(){
  search_iteration_++;
  heaps_.clear();
  heaps_.resize(num_components_);
}

//searches from v1 to v2
//uses the search_iteration counter to cache results between calls (and to allow on-the-fly initialization)
//this is supposed to be used to help find direct shortcuts.
//the caching assumes that v1 is the place you are shortcutting to (a search tree will be grown out of this root state)
//v2 should be the place you are getting on the the egraph to start the shortcut
//the planner should call clearShortestPathCache whenever the direct shortcut locations have changed (such as at the start of replan)
int EGraph::getShortestPath(EGraphVertex* v1, EGraphVertex* v2, vector<EGraphVertex*>* path, vector<int>* costs){
  //ROS_INFO("getShortestPath from %d to %d (components %d,%d)",v1->id,v2->id,v1->component,v2->component);
  assert(v1->component == v2->component);
  if(v1->search_iteration != search_iteration_){
    assert(v1->component < int(heaps_.size()));
    //ROS_INFO("init sc heap %d of %d with state %d as root",v1->component,heaps_.size(),v1->id);
    CHeap* heap = &(heaps_[v1->component]);
    heap->makeemptyheap();
    CKey key;
    key.key[0] = 0;
    v1->search_iteration = search_iteration_;
    v1->search_cost = 0;
    v1->heapindex = 0;
    heap->insertheap(v1,key);
  }
  if(v2->search_iteration != search_iteration_){
    assert(v1->search_cost == 0);
    CHeap* heap = &(heaps_[v1->component]);
    CKey key;
    CKey min_key = heap->getminkeyheap();
    while(!heap->emptyheap() && 
        (v2->search_iteration != search_iteration_ || v2->search_cost > min_key.key[0]) ){
      EGraphVertex* v = (EGraphVertex*)heap->deleteminheap();
      for(unsigned int i=0; i<v->neighbors.size(); i++){
        if(!v->valid[i])
          continue;
        EGraphVertex* u = v->neighbors[i];
        assert(u->component == v1->component);
        if(u->search_iteration != search_iteration_){
          //this vertex has not been discovered before
          //initialize it
          u->search_iteration = search_iteration_;
          u->search_cost = INFINITECOST;
          u->heapindex = 0;
        }
        int newCost = v->search_cost + v->costs[i];
        if (newCost <= 0){
          ROS_INFO("%d %d %d", v->search_cost, v->costs[i], (v->valid[i]==true));
        }
        assert(newCost > 0);
        if(u->search_cost > newCost){ //if we found a cheaper path to it
          key.key[0] = newCost;
          if(u->heapindex != 0)
            heap->updateheap(u,key);
          else
            heap->insertheap(u,key);
          u->search_cost = newCost;
        }
      }
      min_key = heap->getminkeyheap();
    }
    if(v2->search_iteration != search_iteration_){
      //v2 was not found...bad
      ROS_ERROR("[EGraph] getShortestPath was called on two vertices that are not connected. There is probably a bug in the heuristic's getDirectShortcut function (or the connected components are incorrect).");
      return INFINITECOST;
    }
  }


  if(path && costs){
    //reconstruct the search path
    vector<EGraphVertex*> p;
    vector<int > c;
    EGraphVertex* v = v2;
    p.push_back(v);
    while(v!=v1){
      int min_idx = -1;
      int min_cost = INFINITECOST;
      //ROS_INFO("descend from %d",v->id);
      for(unsigned int i=0; i<v->neighbors.size(); i++){
        if(v->neighbors[i]->search_iteration==search_iteration_ &&
            v->valid[i] &&
            v->neighbors[i]->search_cost + v->costs[i] < min_cost){
          min_cost = v->neighbors[i]->search_cost + v->costs[i];
          min_idx = i;
        }
      }
      if(min_idx<0){
        ROS_ERROR("[EGraph] getShortestPath found a local minima while reconstructing the path...");
        return INFINITECOST;
      }
      assert(v->costs[min_idx]>0);
      c.push_back(v->costs[min_idx]);
      p.push_back(v->neighbors[min_idx]);
      v = v->neighbors[min_idx];
    }
    //ROS_INFO("reverse reverse!");

    //reverse the path
    path->clear();
    costs->clear();
    path->reserve(p.size());
    costs->reserve(c.size());
    for(int i=p.size()-1; i>=0; i--)
      path->push_back(p[i]);
    int checksum = 0;
    for(int i=c.size()-1; i>=0; i--){
      costs->push_back(c[i]);
      checksum += c[i];
    }
    if(checksum!=v2->search_cost){
      ROS_ERROR("[EGraph] getShortestPath: the sum of the edges on the shortest path does not equal the g-value of the final state");
      return INFINITECOST;
    }
  }

  return v2->search_cost;
}

