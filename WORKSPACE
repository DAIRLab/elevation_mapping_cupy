workspace(name = "elevation_mapping_cupy")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

#gtest
http_archive(
    name = "gtest",
    sha256 = "8ad598c73ad796e0d8280b082cebd82a630d73e73cd3c70057938a6501bba5d7",
    strip_prefix = "googletest-1.14.0",
    url = "https://github.com/google/googletest/archive/v1.14.0.tar.gz",
)

# pcl
http_archive(
    name = "rules_pcl",
    sha256 = "067b536378e28d1846101981609022f13c1d2dee515e1134a1f9c167e6fc5e67",
    strip_prefix = "rules_pcl-main",
    url = "https://github.com/Brian-Acosta/rules_pcl/archive/main.tar.gz",
)

load("@rules_pcl//bzl:repositories.bzl", "pcl_repositories")

pcl_repositories()

load("@rules_pcl//bzl:init_deps.bzl", "pcl_init_deps")

pcl_init_deps()

# grid map
http_archive(
    name = "grid_map",
    sha256 = "3550f9d039ae40421fa17017790aff015053ea6080dd6cd57870465dc2f1d85f",
    strip_prefix = "grid_map_bazel-main",
    url = "https://github.com/Brian-Acosta/grid_map_bazel/archive/main.tar.gz",
)

# pybind
http_archive(
    name = "pybind11_bazel",
    sha256 = "21e2ea701d0f904cb4c81f402bfd8da7ac44a3ad8a0a300593ebb7c33f041edc",
    strip_prefix = "pybind11_bazel-master",
    urls = ["https://github.com/pybind/pybind11_bazel/archive/master.zip"],
)

http_archive(
    name = "pybind11",
    sha256 = "d475978da0cdc2d43b73f30910786759d593a9d8ee05b1b6846d1eb16c6d2e0c",
    build_file = "@pybind11_bazel//:pybind11.BUILD",
    strip_prefix = "pybind11-2.11.1",
    urls = ["https://github.com/pybind/pybind11/archive/v2.11.1.tar.gz"],
)

load("@pybind11_bazel//:python_configure.bzl", "python_configure")
python_configure(name = "local_config_python", python_version = "3")