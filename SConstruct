#!/usr/bin/env python
import os

api_version = ARGUMENTS.get("api_version", "4.7")

env = SConscript("godot-cpp/SConstruct", {"api_version": api_version})

if env["platform"] == "android":
    env.Append(LINKFLAGS=["-static-libstdc++"])

if ARGUMENTS.get("motion_matching_monitor", "1") == "1":
    env.Append(CPPDEFINES=["GMM_MONITOR_ENABLED"])

env.Append(CPPPATH=["src/", "src/compat"])

sources = []
for subdir in ["math", "features", "database", "trajectory", "runtime", "debug", "nodes", "system", "compat"]:
    sources += Glob("src/%s/*.cpp" % subdir)

platform_dir = env["platform"]
libpath = "addons/gmm/bin/{}/libmotion_matching{}{}".format(platform_dir, env["suffix"], env["SHLIBSUFFIX"])
library = env.SharedLibrary(libpath, source=sources)

Default(library)
