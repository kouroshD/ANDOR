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
 - [:unlock: License](#unlock-license)
 
# :orange_book: The general idea

<p align="center"><img alt="$$&#10;\frac{n!}{k!(n-k)!} = {n \choose k}&#10;$$" src="svgs/32737e0a8d5a4cf32ba3ab1b74902ab7.png?invert_in_darkmode" align=middle width="127.89183pt" height="39.30498pt"/></p>

An AND/OR graph $G(N,H)$ consists of a set of nodes $N$ and a set of hyper-arcs $H$. A node ${n \in N}$ represents a \textit{state} of the cooperation, whereas a hyper-arc $h \in H$ represents a specific \textit{state transition} among states. In particular, a hyper-arc $h$ connects a set of \textit{child} nodes $N_C \subseteq N$ to a \textit{parent} node $n_P \in N$. 
The relation between child nodes in a hyper-arc is the logic AND, while the relation between different hyper-arcs inducing on the same parent node is the logic OR.
Each hyper-arc ${h \in H}$ corresponds to a sequence of ordered actions, $A(h)$, to be executed by the human operator or the robot to reach the cooperation status described by the parent node.

# :hammer: Dependencies and build


# :running: Using the software


# :nerd_face: Mantainers
Kourosh Darvish:
:email: kourosh.darvish@edu.unige.it, kourosh.darvish@gmail.com
# :page_facing_up: References

[Darvish, Kourosh; Wanderlingh, Francesco; Bruno, Barbara; Simetti, Enrico; Mastrogiovanni, Fulvio; Casalino, Giuseppe "Flexible human–robot cooperation models for assisted shop-floor tasks", Journal of Mechatronics.](https://www.sciencedirect.com/science/article/pii/S0957415818300485)
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


# :unlock: License

