rm dove 
rm -rf generated/CMakeFiles
rm generated/cmake_install.cmake
#rm generated/CMakeCache.txt

make -j4

FILE=dove
if [ -f "$FILE" ];then
    mv CMakeFiles generated/
    mv cmake_install.cmake generated/
    #mv CMakeCache.txt generated/

    echo "runfile created."
fi

if [ ! -d "generated" ];then
    mkdir generated
fi

if [ ! -d "analysis" ];then
    mkdir generated
fi

#./dove 4dmaker_598 166 596 3300 1520
./dove 4dmaker_600 5 1500 1 1
mv movie/4dmaker_600_out2.mp4 movie/4dmaker_600_5_1500.mp4

./dove 4dmaker_600 5 1000 1 1
mv movie/4dmaker_600_out2.mp4 movie/4dmaker_600_5_1000.mp4

./dove 4dmaker_600 5 500 1 1
mv movie/4dmaker_600_out2.mp4 movie/4dmaker_600_5_500.mp4

./dove 4dmaker_600 5 100 1 1
mv movie/4dmaker_600_out2.mp4 movie/4dmaker_600_5_100.mp4

./dove 4dmaker_600 10 1500 1 1
mv movie/4dmaker_600_out2.mp4 movie/4dmaker_600_10_1500.mp4

./dove 4dmaker_600 10 1000 1 1
mv movie/4dmaker_600_out2.mp4 movie/4dmaker_600_10_1000.mp4

./dove 4dmaker_600 10 500 1 1
mv movie/4dmaker_600_out2.mp4 movie/4dmaker_600_10_500.mp4

./dove 4dmaker_600 10 100 1 1
mv movie/4dmaker_600_out2.mp4 movie/4dmaker_600_10_100.mp4

./dove 4dmaker_600 10 50 1 1
mv movie/4dmaker_600_out2.mp4 movie/4dmaker_600_10_50.mp4

./dove 4dmaker_600 1 1500 1 1
mv movie/4dmaker_600_out2.mp4 movie/4dmaker_600_1_1500.mp4

./dove 4dmaker_600 1 1000 1 1
mv movie/4dmaker_600_out2.mp4 movie/4dmaker_600_1_1000.mp4

./dove 4dmaker_600 1 500 1 1
mv movie/4dmaker_600_out2.mp4 movie/4dmaker_600_1_500.mp4

./dove 4dmaker_600 1 100 1 1
mv movie/4dmaker_600_out2.mp4 movie/4dmaker_600_1_100.mp4



# ./dove 4dmaker_603 166 596 3300 1520
# ./dove 4dmaker_607 166 596 3300 1520
# ./dove 4dmaker_622 166 596 3300 1520
# ./dove 4dmaker_626 166 596 3300 1520
# ./dove 4dmaker_639 166 596 3300 1520
#python3 py/visualize_traj.py
