

// Pybind
#include <pybind11/embed.h>  // everything needed for embedding

#include "elevation_mapping_wrapper.hpp"

int main(int argc, char** argv) {

  py::scoped_interpreter guard{};
  elevation_mapping_cupy::ElevationMappingWrapper wrapper{};
  wrapper.initialize("/home/brian/workspace/elevation_mapping_cupy/elevation_mapping_cupy/config/parameters.yaml");

  py::gil_scoped_release release;
  return 0;
}
