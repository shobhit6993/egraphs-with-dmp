#ifndef EGRAPH_H
#define EGRAPH_H

#include<ros/ros.h>
#include<vector>
#include<string>
#include<boost/thread.hpp>
#include<sbpl/headers.h>
#include<egraphs/egraph_discretize.h>

class EGraph{
  public:

    class EGraphVertex : public AbstractSearchState{
      public:
        EGraphVertex(){
          id = -1;
          shortcutIteration = 0;
          component = -1;
          search_iteration = 0;
        };

        bool isNeighbor(EGraphVertex* v){
            for (size_t i=0; i < neighbors.size(); i++){
                EGraphVertex* neighbor = neighbors[i];
                if (neighbor == v){ return true;}
            }
            return false;
        };

        int id;
        std::vector<int> coord;
        std::vector<double> constants;
        //an adjacency list representing the graph (using the egraph ids)
        std::vector<EGraphVertex*> neighbors;
        std::vector<int> costs;
        std::vector<bool> valid;
        std::vector<int> use_frequency;
        int component;

        std::vector<EGraphVertex*> shortcuts;
        std::vector<int> shortcut_costs;
        int shortcutIteration;

        int search_iteration;
        int search_cost;
    };

    class EGraphVertexHeapElement: public AbstractSearchState{
      public:
        EGraphVertexHeapElement(){};
        ~EGraphVertexHeapElement(){};
          
        int id;
    };

    //constructor is told how many dimensions (dimensions are used for determin state equality) and number of constants (values
    //carried with the state but not used for identity and are never discretized)
    //load can be called after this in order bring up a stored E-Graph with different parameters than those stored in the file
    EGraph(EGraphDiscretize* eg_disc, int dimensions, int num_constants);

    //another constructor takes an egraph file to load
    //this will load the E-Graph using the parameters (min,max,resolution,names) stored in the file
    EGraph(EGraphDiscretize* eg_disc, std::string filename);

    EGraph(std::string filename, std::string stats_filename);

    ~EGraph();

    //void addDimension(double min, double max, double res, string name, double initial_val);

    void clearEGraph();

    //add path takes a vector of vectors of doubles (the waypoints on the path), a vector of costs
    //this add the edges to the e-graph. 
    //connected components are computed at the end
    bool addPath(const std::vector<std::vector<double> >& coords, const std::vector<int>& costs);
    //add paths runs add path on each path, and only computes the connected components once
    bool addPaths(const std::vector<std::vector<std::vector<double> > >& coords, const std::vector<std::vector<int> >& costs);
    bool addPathHelper(const std::vector<std::vector<double> >& coords, const std::vector<int>& costs);

    int getNumComponents(){return num_components_;};
    void computeComponents();

    void print();

    //save egraph
    bool save(std::string filename);

    //load egraph
    bool load(std::string filename, bool clearCurrentEGraph=true);

    bool saveStats(std::string filename);
    bool loadStats(std::string filename);

    void recordStats(std::vector<std::vector<double> >& coords);
    void resetStats();
    void prune(int max_size, int method);
    void setClusterRadius(double r){cluster_radius_ = r;};

    void updateEdge(EGraphVertex* v1, EGraphVertex* v2, bool valid);
    void updateEdge(EGraphVertex* v1, EGraphVertex* v2, bool valid, int cost);
    void invalidateVertex(EGraphVertex* v1);

    void discToCont(EGraphVertex* v, std::vector<double>& c);
    void contToDisc(std::vector<double> c, std::vector<int>& d);

    void clearShortestPathCache();
    int getShortestPath(EGraphVertex* v1, EGraphVertex* v2, std::vector<EGraphVertex*>* path=NULL, std::vector<int>* costs=NULL);

    //an id to coordinate mapping
    std::vector<EGraphVertex*> id2vertex;
    EGraphVertex* getVertex(std::vector<int>& coord);
    int search_iteration_;
  protected:

    unsigned int inthash(unsigned int key);
    int getHashBin(std::vector<int>& coord);

    EGraphVertex* createVertex(std::vector<int>& coord, std::vector<double>& constants);
    void addEdge(EGraphVertex* v1, EGraphVertex* v2, int cost);

    //a coordinate to id mapping
    //map<vector<double>,int> vertex2id;
    std::vector<std::vector<EGraphVertex*> > hashtable;

    int num_dims_;
    int num_constants_;
    int num_edges_;
    int num_components_;
    double cluster_radius_;

    std::vector<CHeap> heaps_;

    EGraphDiscretize* eg_disc_;

    boost::recursive_mutex egraph_mutex_;
};

#endif
