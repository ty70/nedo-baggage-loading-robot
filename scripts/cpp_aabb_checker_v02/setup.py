from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension, build_ext

ext_modules = [
    Pybind11Extension(
        "cpp_aabb_checker",
        ["checker.cpp"],
        cxx_std=17,
        extra_compile_args=["-O3"],
    )
]

setup(
    name="cpp-aabb-checker",
    version="0.1.0",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
)
