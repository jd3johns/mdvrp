# mdvrp

Solving the Multiple Depot Vehicle Routing Problem with Intelligent Waterdrops metaheuristic using the Cordeau benchmarks.

## Run Instructions

Code originally run with Matlab 2014a, so we do not guarantee it will run on any other version.

Steps:

1. Open Matlab 2014a
2. Change your directory to be the 'code' directory
3. Open the IWD.m file
4. Change the following variables in the IWD.m file as you desire:
  * pind -- problem index (1-5; which problem set to run)
  * soilMat = (weighting) * globalSoilMat -- edge length weighting of soil
    * (change "weighting" to any of the following three by uncommenting)
    * log(distMat + 1)
    * distMat
    * exp(distMat)
5. Run the IWD.m code
