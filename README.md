# Hierarchical AND/OR Graph

This is the repository for Hierarchical AND/OR grapph, which can be employed as a \textit{Task Representation} module.
It semantically formalizes a process between operators (humans and/or robots).
Using that, you can a easily represent a custom task to follow.
This module is developed mainly for represeting the human or robot tasks, but it is not limited to that. The user can easily embed the AND/OR graph in any middleware; in our case we develop it as a [ROS](http://www.ros.org/) package with a service-client paradigm.

# Contents
 - [:orange_book: The general idea](#orange_book-the-general-idea)
 - [:hammer: Dependencies and build](#hammer-dependencies-and-build)
 - [:running: Using the software](#running-using-the-software)
 - [:nerd_face: Mantainers](#nerd_face-mantainers)
 - [:page_facing_up: References](#page_facing_up-references)
 - [:information_source: Further information](information-source-further-information)
 - [:unlock: License](#unlock-license)
 
# :orange_book: The general idea

An AND/OR graph <img alt="$G(N,H)$" src="https://rawgit.com/leegao/readme2tex (fetch/master/svgs/e76c1e530872d588e8dd67bc062f9200.svg" align="middle" width="61.88919pt" height="24.56553pt"/> consists of a set of nodes <img alt="$N$" src="https://rawgit.com/leegao/readme2tex (fetch/master/svgs/f9c4988898e7f532b9f826a75014ed3c.svg" align="middle" width="14.94405pt" height="22.38192pt"/> and a set of hyper-arcs <img alt="$H$" src="https://rawgit.com/leegao/readme2tex (fetch/master/svgs/7b9a0316a2fcd7f01cfd556eedf72e96.svg" align="middle" width="14.94405pt" height="22.38192pt"/>. A node <img alt="${n \in N}$" src="https://rawgit.com/leegao/readme2tex (fetch/master/svgs/956a786478c04fbe5c78c42170382fd6.svg" align="middle" width="44.82423pt" height="22.38192pt"/> represents a \textit{state} of the cooperation, whereas a hyper-arc <img alt="$h \in H$" src="https://rawgit.com/leegao/readme2tex (fetch/master/svgs/af16a807b757d39218d284b0f1c337ca.svg" align="middle" width="44.42988pt" height="22.74591pt"/> represents a specific \textit{state transition} among states. In particular, a hyper-arc <img alt="$h$" src="https://rawgit.com/leegao/readme2tex (fetch/master/svgs/2ad9d098b937e46f9f58968551adac57.svg" align="middle" width="9.435855pt" height="22.74591pt"/> connects a set of \textit{child} nodes <img alt="$N_C \subseteq N$" src="https://rawgit.com/leegao/readme2tex (fetch/master/svgs/beaa182037d97b007e26c76344697062.svg" align="middle" width="61.028715pt" height="22.38192pt"/> to a \textit{parent} node <img alt="$n_P \in N$" src="https://rawgit.com/leegao/readme2tex (fetch/master/svgs/effe6cae71f78ba883170eeaa8024c0c.svg" align="middle" width="55.78485pt" height="22.38192pt"/>. 
The relation between child nodes in a hyper-arc is the logic AND, while the relation between different hyper-arcs inducing on the same parent node is the logic OR.
Each hyper-arc <img alt="${h \in H}$" src="https://rawgit.com/leegao/readme2tex (fetch/master/svgs/24a030ca58abb6722b0144d6897f93d2.svg" align="middle" width="44.42988pt" height="22.74591pt"/> corresponds to a sequence of ordered actions, <img alt="$A(h)$" src="https://rawgit.com/leegao/readme2tex (fetch/master/svgs/9b8327f3af89421e4e4821f1961d0b2f.svg" align="middle" width="34.456125pt" height="24.56553pt"/>, to be executed by the human operator or the robot to reach the cooperation status described by the parent node.
<p align="center"><img alt="$$&#10;\frac{n!}{k!(n-k)!} = {n \choose k}&#10;$$" src="https://rawgit.com/leegao/readme2tex (fetch/None/svgs/32737e0a8d5a4cf32ba3ab1b74902ab7.svg" align="middle" width="127.89183pt" height="39.30498pt"/></p>

# :hammer: Dependencies and build
The only dependency is [ROS](http://www.ros.org/) and you can download it from [here](http://www.ros.org/install/).

After installing ros, using git, clone the repo in your `src` directory of your ros workspace, or download the repo and extract it in `src` directory of your ros workspace.

```
cd <path to your ros workspace>
catkin_make
```

# :running: Using the software

To write an AND/OR graph description, please follow the instructions provided [here](./docs/Instructions.md).

You can generate [doxygen](http://www.doxygen.nl/) documentation using follwoing command:
```
cd <path to your ros workspace>/andor

doxygen doxygen_config
```

# :nerd_face: Mantainers
If you have any question regarding the implementation, you can open an issue in the repo and CC the maintainer or contact directly the maintainer.

* [Kourosh Darvish](https://github.com/kouroshD)

:email: kourosh.darvish@gmail.com

# :page_facing_up: References

[K. Darvish, F. Wanderlingh, B. Bruno, E. Simetti, F. Mastrogiovanni, and G. Casalino, ''Flexible human-robot cooperation models for assisted shop-floor tasks,'' Mechatronics, vol. 51, pp. 97–114, 2018.](https://www.sciencedirect.com/science/article/pii/S0957415818300485)
```
@article{DARVISH201897,
title = "Flexible human–robot cooperation models for assisted shop-floor tasks",
journal = "Mechatronics",
volume = "51",
pages = "97 - 114",
year = "2018",
issn = "0957-4158",
doi = "https://doi.org/10.1016/j.mechatronics.2018.03.006",
url = "http://www.sciencedirect.com/science/article/pii/S0957415818300485",
author = "Kourosh Darvish and Francesco Wanderlingh and Barbara Bruno and Enrico Simetti and Fulvio Mastrogiovanni and Giuseppe Casalino",
}
```
# :information_source: Further information

For more information please contact the follwoing authors:

* [Kourosh Darvish](https://github.com/kouroshD):

:email: kourosh.darvish@edu.unige.it

* [Fulvio Mastrogiovanni](https://www.dibris.unige.it/mastrogiovanni-fulvio):

:email: fulvio.mastrogiovanni@unige.it 


# :unlock: License
The license is porovided [here](./LICENSE).
