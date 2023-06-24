
# R-NOCS

Numerical Optimal Control Solver for Robot Trajectory Optimization


## Dependencies
All the dependencies are installed at PREFIX=usr/local/

- [Eigen library V3.3.4](https://gitlab.com/libeigen/eigen/-/releases/3.3.4)

* [Ipopt library](https://coin-or.github.io/Ipopt/INSTALL.html)

* [Pinocchio library](https://github.com/stack-of-tasks/pinocchio)

* [rbdl library](https://rbdl.github.io/) (Optional)
## Installation

Clone this repository into your desired folder

```bash
  cd /path/to/RNOCS
  mkdir build
  cmake -DDYNLIBRARY=pinocchio .. (or rbdl)
  make (You can also try to speed up the compilation by using the -jN flag (e.g., make -j3), where N is the number of parallel compilation jobs)
```
    
## Support

For support, email daniel.cardona@cinvestav.mx


## Contributing

Contributions are always welcome!

See `contributing.md` for ways to get started.

Please adhere to this project's `code of conduct`.

