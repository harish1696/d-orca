cmake_minimum_required(VERSION 2.8.3)
project(d-orca)

add_compile_options(-std=c++11)

add_library(PQP lib/libPQP.a BV.cpp Build.cpp PQP.cpp TriDist.cpp)
