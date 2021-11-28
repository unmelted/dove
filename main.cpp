#include "stab.hpp"

using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {

    char infile[40];
    char outfile[40];
    sprintf(infile,"%s.mp4", argv[1]);
    sprintf(outfile, "%s_out1.mp4", argv[1]);

    cout<<infile<<endl;
    cout<<outfile<<endl;
    int mode_dof = 2;
    int result = 0;

    if(mode_dof == 2) {
        //result = stab_2dof(infile, outfile);
        cout <<"TBD" << endl;
    } else if (mode_dof== 6) {
        result = stab_6dof(infile, outfile);
    }

}