all:
	rm -rf build install
	mkdir build install install/lib install/include


	CPLUS_INCLUDE_PATH=`pwd`/install/include make -C libcmt -j8
	mkdir install/include/libcmt
	cp libcmt/*.h install/include/libcmt
	cp libcmt/*.a install/lib

	# PyBind
	cd pybind11
	python3 -m pip install pybind11
	
	#rm -rf build
