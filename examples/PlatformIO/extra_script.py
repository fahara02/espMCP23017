Import("env")

# Append flag only for C++ compiler
env.Append(CXXFLAGS=["-fno-rtti"])

env.Append(CCFLAGS=["-Wno-unused-parameter"])  
