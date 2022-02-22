## AreaCoverage-library
Library for Area Coverage with multiple capacity-constrained robots:

The algorithms are described in the following papers. Please cite our publications when using the library.

> Area Coverage with Multiple  Capacity-Constrained Robots  
> Agarwal S and Akella S (2020)  
> In: IEEE Robotics Automation and Letters (RA-L), vol. 7, no. 2, pp. 3734-3741, April 2022, doi: 10.1109/LRA.2022.3146952.

> @ARTICLE{9697431,  
> author={Agarwal, Saurav and Akella, Srinivas},  
> journal={IEEE Robotics and Automation Letters},   
> title={Area Coverage With Multiple Capacity-Constrained Robots},  
> year={2022},  
> volume={7},  
> number={2},  
> pages={3734--3741},  
> doi={10.1109/LRA.2022.3146952}}  


The library uses the LineCoverage-library given in the repository:
https://github.com/UNCCharlotte-CS-Robotics/LineCoverage-library

> Line coverage with multiple robots.  
> Agarwal S and Akella S (2020)  
> In: IEEE International Conference on Robotics and Automation (ICRA). Paris, France, pp. 3248–3254.


## Installation
The library has been tested on Ubuntu 20.04 and should work with any GNU/Linux system provided the following dependencies have been installed.
These packages are usually already installed.

```bash
sudo apt-get install git build-essential cmake unzip
sudo apt-get install libyaml-cpp-dev gnuplot libglpk-dev
```

#### Installation of AreaCoverage-library (aclibrary):

Create a new directory for our workspace where we will be cloning the repository and creating builds:

**Build LineCoverage-library**

```bash
mkdir ~/coverage_ws && cd ~/coverage_ws   
git clone https://github.com/UNCCharlotte-CS-Robotics/LineCoverage-library.git    
cmake -S LineCoverage-library/ -B ./build/lclibrary -DCMAKE_INSTALL_PREFIX=install/  
cmake --build build/lclibrary  
cmake --install build/lclibrary  
```
**Build AreaCoverage-library**
```bash
cd ~/coverage_ws   
git clone https://github.com/UNCCharlotte-CS-Robotics/AreaCoverage-library.git    
cmake -S AreaCoverage-library/ -B ./build/aclibrary -DCMAKE_INSTALL_PREFIX=install/  
cmake --build build/aclibrary  
cmake --install build/aclibrary  
```


You should be able to see the binary files `mac_point_robot` and `mac_custom_robot` in the install directory after this process.

**Get the AreaCoverage-dataset**
```bash
cd  ~/coverage_ws    
git clone https://github.com/UNCCharlotte-CS-Robotics/AreaCoverage-dataset.git
```

**Run programs**

```bash
cd ~/coverage_ws
./install/bin/mac_point_robot AreaCoverage-library/config/default_config.yaml
```

Check the folder `AreaCoverage-dataset/RAL-main/`. You should find the input data and the results.  
Make a copy of the file `AreaCoverage-library/config/default_config.yaml` and change according to your preference.  

## Contact
Users are requested to raise bugs to help improve the library. We are working on improving the documentation. Please use the GitHub issues, pull requests, and discussions.  
The program is authored by Saurav Agarwal during his PhD (advised by Srinivas Akella) at the University of North Carolina at Charlotte.  
Saurav Agarwal: https://webpages.uncc.edu/sagarw10/  
Srinivas Akella: https://webpages.uncc.edu/sakella/

## License and Acknowledgments
Copyright (C) 2020--2022 University of North Carolina at Charlotte.  
The AreaCoverage-library is owned by the University of North Carolina at Charlotte and is protected by United States copyright laws and applicable international treaties and/or conventions.

The source code is licensed under GPLv3. Please see the LICENSE file.

DISCLAIMER OF WARRANTIES: THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT WARRANTY OF ANY KIND INCLUDING ANY WARRANTIES OF PERFORMANCE OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR USE OR PURPOSE OR OF NON-INFRINGEMENT. YOU BEAR ALL RISK RELATING TO QUALITY AND PERFORMANCE OF THE SOFTWARE OR HARDWARE.

The following repository is used for parsing JSON:  
https://github.com/nlohmann/json

YAML-CPP is used to parse yaml configuration files:  
https://github.com/jbeder/yaml-cpp
