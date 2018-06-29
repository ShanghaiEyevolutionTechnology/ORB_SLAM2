bsfile="buildok.log"

echo "Configuring and building Thirdparty/Pangolin ..."
if [ ! -x "Thirdparty/Pangolin" ]; then
	cd Thirdparty
	git clone https://github.com/stevenlovegrove/Pangolin.git
	if [ $? -ne 0 ];then
		echo "clone pangolin failed..."
		exit 1
	fi
	cd ..
fi

cd Thirdparty/Pangolin
mkdir build
cd build
if [ ! -f "$bsfile" ]; then
	sudo apt-get -y install libeigen3-dev
	sudo apt-get -y install libblas-dev
	sudo apt-get -y install liblapack-dev
	cmake .. -DCMAKE_BUILD_TYPE=Release
	make -j4

	if [ $? -ne 0 ];then
		echo "failed..."
		exit 1
	fi
	touch "$bsfile"
else
	echo "already build ok. skip.."
fi




cd ../../..
echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
if [ ! -f "$bsfile" ]; then
	cmake .. -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/opt/OpenCV320/share/OpenCV
	make -j4

	if [ $? -ne 0 ];then
		echo "failed..."
		exit 1
	fi
	touch "$bsfile"
else
	echo "already build ok. skip.."
fi




cd ../../..
echo "Configuring and building Thirdparty/g2o ..."

cd Thirdparty/g2o
mkdir build
cd build
if [ ! -f "$bsfile" ]; then
	cmake .. -DCMAKE_BUILD_TYPE=Release
	make -j4

	if [ $? -ne 0 ];then
		echo "failed..."
		exit 1
	fi
	touch "$bsfile"
else
	echo "already build ok. skip.."
fi



cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
if [ ! -f "ORBvoc.txt" ]; then
	tar -xf ORBvoc.txt.tar.gz
else
	echo "already Uncompress vocabulary. skip.."
fi




cd ..
echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/opt/OpenCV320/share/OpenCV
make VERBOSE=1
