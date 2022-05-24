# Anytime BOA*
we 
C++ implementation of AnytimeBOA [1].


You can compile the project using the following commands:

``` shell
cmake .
make
```
After typing the above commands. Cmake will generate `solver` in the `bin` folder.
You can type `solver --help` to see the expected input arguments.

Example usage:

``` shell
# find the Pareto-optimal frontier from node 20002 to node 164983 in the BAY map.
./bin/solver -m resources/dataset/USA-road-d.BAY.gr resources/dataset/USA-road-t.BAY.gr -s 20002 -g 164983 -a anytime -o output.txt
```

You can download the road networks we used in the paper from [here]( http://users.diag.uniroma1.it/challenge9/download.shtml).

[1] Han Zhang, Oren Salzman, T. K. Satish Kumar, Ariel Felner,Carlos Hernandez Ulloa, Sven Koenig. 2022. Anytime Approximate Bi-Objective Search. In SoCS.
