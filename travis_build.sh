#! /bin/bash
# Bash script to build and test the project on Travis CI

# Build in debug mode with double precision and run the tests
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=${BUILD_TYPE} —DDOUBLE_PRECISION_ENABLED=${DOUBLE_PRECISION} -DCOMPILE_TESTS=True ../
make && make test ARGS="-V"

# Build in debug mode with single precision and run the tests
#cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug —DDOUBLE_PRECISION_ENABLED=False -DCOMPILE_TESTS=True ../
#make && make test ARGS="-V"

# Build in release mode with double precision and run the tests
#cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release —DDOUBLE_PRECISION_ENABLED=True -DCOMPILE_TESTS=True ../
#make && make test ARGS="-V"

# Build in release mode with single precision and run the tests
#cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release —DDOUBLE_PRECISION_ENABLED=False -DCOMPILE_TESTS=True ../
#make && make test ARGS="-V"

# Build in release mode with logs and profiler enabled
#cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release —DPROFILING_ENABLED=True -DLOGS_ENABLED=True ../
#make

# Build in debug mode with logs, profiler and code coverage enabled
#cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug —DPROFILING_ENABLED=True -DLOGS_ENABLED=True ../
#make

# Run the code coverage with (lcov) only on a single build (when compiler is GCC and OS is Linux)
if [[ "$RUN_CODE_COVERAGE" == "True" ]]; then

  # Build in debug mode with code coverage enabled
  cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DCODE_COVERAGE_ENABLED=True ../
  make && make test ARGS="-V"
  
fi
