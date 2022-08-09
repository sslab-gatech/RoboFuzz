pass:
	clang -O3 -funroll-loops -g -fPIC -c cov-trace.o.c -o cov-trace.o
	clang -O3 -funroll-loops -g -fPIC -c cov-fs.o.c -o cov-fs.o

turtle:
	colcon build --packages-select turtlesim --cmake-clean-cache --cmake-force-configure --cmake-arg -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="/home/seulbae/workspace/ros-clang/cov-trace.o -fsanitize-coverage=trace-pc-guard" --symlink-install
