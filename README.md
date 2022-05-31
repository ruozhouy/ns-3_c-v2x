# Cellular Vehicle-to-Everything (C-V2X) Mode 4 Communication Model for ns-3, with Collaborative Perception Use Case

Simulating collaborative perception use cases in C-V2X Mode 4. The simulator does not simulate actual perception traffic, but instead have each vehicle send multiple large-sized packets per scheduling round. Size of the collaborative perception packet can be configured in the script.

## Installation

1. Clone or download the source code from this repository
2. Navigate to the root directory of the cloned/downloaded repository
3. Configure the project using the command
```
   ./waf configure
```
4. Build the project using the command
```
   ./waf build
```

For more details regarding the configuration and building of ns-3 projects see [the ns-3 documentation](https://www.nsnam.org/documentation/).

## Usage

A example script for the usage of the C-V2X Mode 4 is located in the *scratch* directory of this repository (*v2x_communication_example.cc*).
To run the example script run:
```
   ./waf --run v2x_communication_example
```

To reproduce results in the paper, please run one of the python scripts:
```
   python scratch/run-1.py
```
or 
```
   python scratch/run-2.py
```

The results should be stored in
```
   scratch/v2x_data/
```



# Cite As

If you use our model in your research, please cite the following paper:

R. Yu, D. Yang, and H. Zhang, ["Edge-Assisted Collaborative Perception in Autonomous Driving: A Reflection on Communication Design"](https://people.engr.ncsu.edu/ryu5/docs/sec-wkshps-21-paper.pdf), in Proc. ACM SEC Workshop on Edge Computing and Communications (EdgeComm), 2021, pp. 1–6.

### Bibtex:
    @inproceedings{Yu2021a,
        author = {Yu, Ruozhou and Yang, Dejun and Zhang, Hao},
        booktitle = {Proc. ACM SEC Workshop on Edge Computing and Communications (EdgeComm)},
        pages = {1--6},
        title = {{Edge-Assisted Collaborative Perception in Autonomous Driving: A Reflection on Communication Design}},
        year = {2021}
    }



# Acknowledgement

The simulator is based on the following paper's open-source code:

F. Eckermann, M. Kahlert, C. Wietfeld, ["Performance Analysis of C-V2X Mode 4 Communication Introducing an Open-Source C-V2X Simulator"](https://www.kn.e-technik.tu-dortmund.de/.cni-bibliography/publications/cni-publications/Eckermann2019performance.pdf), In 2019 IEEE 90th Vehicular Technology Conference (VTC-Fall), Honolulu, Hawaii, USA, September 2019.

