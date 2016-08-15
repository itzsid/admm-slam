ADMM-SLAM
===================================================
This library is an implementation of the algorithm described in Exactly Sparse Memory Efficient SLAM using the 
Multi-Block Alternating Direction Method of Multipliers (IROS 2015). The core library is developed in C++
language.

ADMM-SLAM is developed by [Siddharth Choudhary](mailto:siddharth.choudhary@gatech.edu) and 
[Luca Carlone](mailto:lcarlone@mit.edu) as part of their work at Georgia Tech. 

Prerequisites
------

- CMake (Ubuntu: `sudo apt-get install cmake`), compilation configuration tool.
- [Boost](http://www.boost.org/)  (Ubuntu: `sudo apt-get install libboost-all-dev`), portable C++ source libraries.
- [GTSAM](https://bitbucket.org/gtborg/gtsam) >= 3.0, a C++ library that implement smoothing and mapping (SAM) framework in robotics and vision.
Here we use factor graph implementations and inference/optimization tools provided by GTSAM.

Compilation & Installation
------

In the cpp folder excute:

```
$ mkdir build
$ cd build
$ cmake ..
$ make -j3
$ make check  # optonal, run unit tests
```

Run Experiments
------

```
$ cd scripts
$ bash runADMMOnSyntheticData.sh 
$ bash runADMMOnBenchmarkData.sh 
```

Questions & Bug reporting
-----

Please use Github issue tracker to report bugs. For other questions please contact [Siddharth Choudhary](mailto:siddharth.choudhary@gatech.edu) and
[Luca Carlone](mailto:lcarlone@mit.edu).

Acknowledgements
----
We wish to thank Gian Diego Tipaldi and Benjamin Suger
for sharing the datasets ETHCampus and AIS2Klinik, and for
authorizing the use of the data in Table I. We
gratefully acknowledge reviewers for the helpful comments.
This work was partially funded by the ARL MAST CTA
Project 1436607 “Autonomous Multifunctional Mobile Microsystems”
and by the National Science Foundation Award
11115678 “RI: Small: Ultra-Sparsifiers for Fast and Scalable
Mapping and 3D Reconstruction on Mobile Robots”.

Citing
-----

If you use this work, please cite following publication:

```
@inproceedings{Choudhary15iros,
  author    = {Siddharth Choudhary and
               Luca Carlone and
               Henrik I. Christensen and
               Frank Dellaert},
  title     = {Exactly Sparse Memory Efficient SLAM using the Multi-Block Alternating Direction Method of Multipliers},
  booktitle = {2015 {IEEE/RSJ} International Conference on Intelligent Robots and Systems, Hamburg, Germany},
  year      = {2015}
}
```


License
-----

ADMM-SLAM is released under the BSD license, reproduced in the file LICENSE in this directory.
