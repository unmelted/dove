rm dove 
rm -rf generated/CMakeFiles
rm generated/cmake_install.cmake
#rm ㅇgenerated/CMakeCache.txt

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
./dove figure_600.json
#./dove 4dmaker_603 166 596 3300 1520
#./dove 4dmaker_607 166 596 3300 1520
#./dove 4dmaker_622 166 596 3300 1520
#./dove 4dmaker_626 166 596 3300 1520
#./dove 4dmaker_639 166 596 3300 1520
#./dove 4NylanderGoal 166 596 3300 1520
#./dove BUT_TATAR_4-0
#./dove 2018_02_09_17_58_50
#./dove 2018_02_25_09_55_28
#./dove 2018_02_13_19_37_53_0
#python3 py/visualize_traj.py
