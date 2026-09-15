[![INFORMS Journal on Computing Logo](https://INFORMSJoC.github.io/logos/INFORMS_Journal_on_Computing_Header.jpg)](https://pubsonline.informs.org/journal/ijoc)

# Asynchronous Cooperative Optimization of a Capacitated Vehicle Routing Problem Solution

This archive is distributed in association with the [INFORMS Journal on
Computing](https://pubsonline.informs.org/journal/ijoc) under the [GPL-3.0 License](LICENSE).

The software and data in this repository are a snapshot of the software and data
that were used in the research reported on the paper
[Asynchronous Cooperative Optimization of a Capacitated Vehicle Routing Problem Solution](https://doi.org/10.1287/ijoc.2025.1772) by Luca Accorsi, Demetrio Laganà, Federico Michelotto, Roberto Musmanno, Daniele Vigo.

## Cite

To cite the contents of this repository, please cite both the paper and this repo, using their respective DOIs.

https://doi.org/10.1287/ijoc.2025.1772

https://doi.org/10.1287/ijoc.2025.1772.cd

Below is the BibTex for citing this snapshot of the repository.

```
@misc{filo2x,
  author =        {Luca Accorsi and Demetrio Lagan{\`a} and Federico Michelotto and Roberto Musmanno and Daniele Vigo},
  publisher =     {INFORMS Journal on Computing},
  title =         {{Asynchronous Cooperative Optimization of a Capacitated Vehicle Routing Problem Solution}},
  year =          {2026},
  doi =           {10.1287/ijoc.2025.1772.cd},
  url =           {https://github.com/INFORMSJoC/2025.1772},
  note =          {Available for download at https://github.com/INFORMSJoC/2025.1772},
}
```

## Description

This repository contains a C++ implementation of the algorithm proposed in the paper **Asynchronous Cooperative Optimization of a Capacitated Vehicle Routing Problem Solution** by Luca Accorsi, Demetrio Laganà, Federico Michelotto, Roberto Musmanno, Daniele Vigo.

The repository includes the source code of algorithm, the benchmark instances, and aggregate computational results.  

### Running the code
```
git clone git@github.com:INFORMSJoC/2025.1772.git
cd filo2x
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_VERBOSE=1
make -j

./filo2x ../instances/X/X-n1001-k43.vrp --solvers-num 8
```

## Repository Structure

This repository includes the following materials:

* `instances`: literature instances used during the experiments.
* `results`: aggregate computational results part of the online supplement of the paper and referenced in the paper appendix.
* `src`: algorithm source code.

## Ongoing Development
This code is being developed on an on-going basis at the author's [Github site](https://github.com/acco93/filo2x).
