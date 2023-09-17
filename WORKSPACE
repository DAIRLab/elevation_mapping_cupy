workspace(name = "elevation_mapping_cupy")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "gtest",
    sha256 = "8ad598c73ad796e0d8280b082cebd82a630d73e73cd3c70057938a6501bba5d7",
    strip_prefix = "googletest-1.14.0",
    url = "https://github.com/google/googletest/archive/v1.14.0.tar.gz",
)

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

http_archive(
    name = "grid_map",
    sha256 = "a8e4f8678680a5a88b4786903f84ecc5f86cb54e20ca675d7aed22493ce6dc86",
    strip_prefix = "grid_map_bazel-main",
    url = "https://github.com/Brian-Acosta/grid_map_bazel/archive/main.tar.gz",
)