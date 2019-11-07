#
# step 1
#
rm *jpeg

cd alice_step1_part2_populationDensityGenerator_version3_aStarPathPlanning/build/
rm data*
rm log*
cmake ..
make
./alice

#
cd ../../
#python alice_step1_part2_tool_costFunction.py
python alice_step1_part2_tool_obstacleMap.py
