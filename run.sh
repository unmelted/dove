rm dove 
rm -rf generated/CMakeFiles
rm generated/cmake_install.cmake
rm generated/CMakeCache.txt

make -j4

FILE=dove
if [ -f "$FILE" ];then
    mv CMakeFiles generated/
    mv cmake_install.cmake generated/
    mv CMakeCache.txt generated/

    echo "runfile created."
fi

if [ ! -d "generated" ];then
    mkdir generated
fi

if [ ! -d "analysis" ];then
    mkdir generated
fi

./dove 4dmaker_600 166 596 3300 1520
python3 py/visualize_traj.py
