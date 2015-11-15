#include<egraphs/egraphable.h>
#include<egraphs/egraph_discretize.h>
#include<vector>
#include<string>
#include<ros/ros.h>

template <typename HeuristicType>
class TestEGraphable{
  public:

    TestEGraphable(EGraphable<HeuristicType>* egraphable, EGraphDiscretize* eg_disc, std::vector<double> mins, std::vector<double> maxs){
      egraphable_ = egraphable;
      egraph_discretize_ = eg_disc;
      min_ = mins;
      max_ = maxs;
    }

    bool discVectorsEqual(const std::vector<int>& disc_coord, const std::vector<int>& disc_coord2){
      for(unsigned int i=0; i<disc_coord.size(); i++)
        if(disc_coord[i]!=disc_coord2[i])
          return false;
      return true;
    }

    void printDiscVector(const std::vector<int>& coord, string str){
      printf("%s",str.c_str());
      for(unsigned int i=0; i<coord.size(); i++)
        printf("%d ", coord[i]);
      printf("\n");
    }

    void printContVector(const std::vector<double>& coord, string str){
      printf("%s",str.c_str());
      for(unsigned int i=0; i<coord.size(); i++)
        printf("%f ", coord[i]);
      printf("\n");
    }

    bool testDisc(const std::vector<double>& input_coord, std::vector<double>& cont_disc_input){
      std::vector<int> disc_coord;
      egraph_discretize_->contToDisc(input_coord, disc_coord);
      if(input_coord.size() != disc_coord.size()){
        ROS_ERROR("contToDisc doesn't return a discrete coordinate the same size as the input continuous one");
        return false;
      }
      std::vector<double> cont_coord;
      egraph_discretize_->discToCont(disc_coord, cont_coord);
      if(disc_coord.size() != cont_coord.size()){
        ROS_ERROR("contToDisc doesn't return a discrete coordinate the same size as the input continuous one");
        return false;
      }
      std::vector<int> disc_coord2;
      egraph_discretize_->contToDisc(cont_coord, disc_coord2);

      if(!discVectorsEqual(disc_coord,disc_coord2)){
        ROS_ERROR("contToDisc and discToCont are not inverses!\n");
        printContVector(input_coord,"input continuous coordinate:   ");
        printDiscVector(disc_coord, "discrete coordinate:           ");
        printContVector(cont_coord, "back to continuous coordinate: ");
        printDiscVector(disc_coord2,"to discrete coordinate again:  ");
        return false;
      }

      cont_disc_input = cont_coord;
      return true;
    }

    bool testConversions(const std::vector<double>& input_coord){
      int id = egraphable_->getStateID(input_coord);
      std::vector<double> cont_coord;
      egraphable_->getCoord(id, cont_coord);

      std::vector<int> disc_coord;
      egraph_discretize_->contToDisc(input_coord, disc_coord);
      std::vector<int> disc_coord2;
      egraph_discretize_->contToDisc(cont_coord, disc_coord2);

      if(!discVectorsEqual(disc_coord,disc_coord2)){
        printContVector(input_coord,"input:         ");
        printDiscVector(disc_coord,  "disc input:    ");
        printf("getStateID on input: %d\n", id);
        printContVector(cont_coord,  "getCoord:      ");
        printDiscVector(disc_coord2, "disc getCoord: ");
        ROS_ERROR("The discretized getCoord should be the same as the discretized input!");
        return false;
      }
      return true;
    }

    bool testCoordinate(const std::vector<double>& input_coord){
      std::vector<double> cont_disc_coord;
      if(!testDisc(input_coord, cont_disc_coord))
        return false;
      if(!testConversions(input_coord))
        return false;
      if(!testConversions(cont_disc_coord))
        return false;
      return true;
    }

    std::vector<double> getRandomCoordinate(){
      std::vector<double> coord(max_.size(),0);
      for(unsigned int i=0; i<max_.size(); i++)
        coord[i] = min_[i] + (max_[i] - min_[i])*((double)rand()/(double)RAND_MAX);
      return coord;
    }

    void runTests(int num_tests){
      for(int i=0; i<num_tests; i++){
        std::vector<double> coord = getRandomCoordinate();
        if(!testCoordinate(coord))
          return;
      }
      ROS_INFO("all %d tests passed!",num_tests);
    }


  private:
    EGraphable<HeuristicType>* egraphable_;
    EGraphDiscretize* egraph_discretize_;
    std::vector<double> min_;
    std::vector<double> max_;
};


