all:
	g++ -O3 -mtune=native -march=native -DNDEBUG -Wall -Wextra -Wno-attributes -shared -std=c++17 -fPIC \
	`python3 -m pybind11 --includes` \
	-isystem./dependencies/install/include \
	xsens_MTi28A.cpp -o xsens_MTi28A`python3-config --extension-suffix` \
	-L./dependencies/install/lib \
	-lcmt

clean:
	rm xsens_MTi28A`python3-config --extension-suffix`
