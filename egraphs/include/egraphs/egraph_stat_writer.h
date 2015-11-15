#ifndef EGRAPH_STAT_WRITER_H
#define EGRAPH_STAT_WRITER_H

#include <vector>
#include <stdio.h>

class EGraphStatWriter{
  public:
    static void writeStatsToFile(std::string filename, bool start_new_file, std::vector<std::string> names, std::vector<double> values){
      FILE* fout;
      if(start_new_file){
        fout = fopen(filename.c_str(),"w");
        fprintf(fout, "%% ");
        for(unsigned int i=0; i<names.size(); i++)
          fprintf(fout, "%s ", names[i].c_str());
        fprintf(fout, "\n");
      }
      else
        fout = fopen(filename.c_str(),"a");

      for(unsigned int i=0; i<values.size(); i++)
        fprintf(fout, "%.2f ", values[i]);
      fprintf(fout, "\n");

      fclose(fout);
    };
};

#endif
