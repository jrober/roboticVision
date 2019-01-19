#!/bin/bash

COLOR_NC='\e[0m' # No Color
COLOR_WHITE='\e[1;37m'
COLOR_BLACK='\e[0;30m'
COLOR_BLUE='\e[0;34m'
COLOR_LIGHT_BLUE='\e[1;34m'
COLOR_GREEN='\e[0;32m'
COLOR_LIGHT_GREEN='\e[1;32m'
COLOR_CYAN='\e[0;36m'
COLOR_LIGHT_CYAN='\e[1;36m'
COLOR_RED='\e[0;31m'
COLOR_LIGHT_RED='\e[1;31m'
COLOR_PURPLE='\e[0;35m'
COLOR_LIGHT_PURPLE='\e[1;35m'
COLOR_BROWN='\e[0;33m'
COLOR_YELLOW='\e[1;33m'
COLOR_GRAY='\e[0;30m'
COLOR_LIGHT_GRAY='\e[0;37m'


# This function downloads the OpenCV files and extracts them
function get_files {
    # See if files exist already. If they don't, download them.
    if [ ! -f opencv${1}-${OPENCV_VERSION}.zip ] && [ ! -d opencv${1}-${OPENCV_VERSION} ]; then
        echo -e "${COLOR_LIGHT_GREEN}Downloading opencv${1}-${OPENCV_VERSION}${COLOR_NC}"
        wget -O opencv${1}-${OPENCV_VERSION}.zip https://github.com/opencv/opencv${1}/archive/${OPENCV_VERSION}.zip
    else
        echo -e "${COLOR_LIGHT_GREEN}opencv${1}-${OPENCV_VERSION} already downloaded${COLOR_NC}"

    fi



    # If not extracted, extract them
    if [ ! -d opencv${1}-${OPENCV_VERSION} ]; then
        echo -e "${COLOR_LIGHT_GREEN}Extracting opencv${1}-${OPENCV_VERSION}.zip${COLOR_NC}"
        unzip opencv${1}-${OPENCV_VERSION}
    fi
}

#--------------------------------------------------------------------------------------------------------

echo -e "${COLOR_LIGHT_BLUE}OpenCV Setup${COLOR_NC}"

#--------------------------------------------------------------------------------------------------------
#                        CUDA
#--------------------------------------------------------------------------------------------------------

# See if the user wants to build it with CUDA
echo -e "${COLOR_CYAN}Do you want to build OpenCV with CUDA?${COLOR_NC}"
select yn in "Yes" "No"; do
    case $yn in
        Yes ) CUDA=true; break;;
        No ) CUDA=false; break;;
        * ) echo -e "${COLOR_CYAN}Please enter 1 or 2?${COLOR_NC}"
    esac
done


# Check if CUDA is installed
if $CUDA ; then
    CUDA_RELEASE=$(nvcc --version | grep -oh release....)

    if [[ $CUDA_RELEASE ]]; then
        CUDA_RELEASE=${CUDA_RELEASE:(-3)}
        echo -e "${COLOR_LIGHT_GREEN}Found CUDA ${CUDA_RELEASE}${COLOR_NC}"

    else
        echo -e "${COLOR_RED}CUDA not found! Exiting... ${COLOR_NC}"
        exit

    fi
fi

#--------------------------------------------------------------------------------------------------------
#                        INSTALL
#--------------------------------------------------------------------------------------------------------


# See if the user wants it installed
echo -e "${COLOR_CYAN}Do you wish to install OpenCV in addition to building it?${COLOR_NC}"
select yn in "Yes" "No"; do
    case $yn in
        Yes ) INSTALL_OPENCV=true; break;;
        No ) INSTALL_OPENCV=false; break;;
        * ) echo -e "${COLOR_CYAN}Please enter 1 or 2?${COLOR_NC}"
    esac
done

# If the User wants it installed, get install path

if $INSTALL_OPENCV ; then
    echo -e "${COLOR_CYAN}Please enter the install path for OpenCV. ex: /usr/local ${COLOR_NC}"
    read OPENCV_INSTALL_PREFIX

fi

#--------------------------------------------------------------------------------------------------------
#                        BUILD VERSION
#--------------------------------------------------------------------------------------------------------

# Select build version and download it
echo -e "${COLOR_CYAN}What version of OPENCV do you want to build? ex: 3.4.0 ${COLOR_NC}"
read OPENCV_VERSION

FOUND=false
URL=https://github.com/opencv/opencv/archive/

while ! $FOUND; do


    if [[ `wget -S --spider "$URL$OPENCV_VERSION.zip"  2>&1 | grep 'HTTP/1.1 200 OK'` ]]; then

        # Change directory
        cd ~/Downloads

        # Get opencv
        get_files

        # Get opencv_contribe
        get_files _contrib

        # Get opencv_extra
        get_files _extra

        FOUND=true

    else
        echo -e "${COLOR_CYAN}OpenCV-${OPENCV_VERSION} not found!. Please select another. ${COLOR_NC}"
        echo -e "${COLOR_CYAN}Visit https://opencv.org/releases.html to see possible versions.${COLOR_NC}"
        read OPENCV_VERSION
    fi



done


#--------------------------------------------------------------------------------------------------------
#                        INSTALL DEPENDENCIES
#--------------------------------------------------------------------------------------------------------


# Install dependencies
echo -e "${COLOR_LIGHT_GREEN}Installing Dependencies${COLOR_NC}"
sudo apt-get install -y build-essential checkinstall cmake pkg-config
sudo apt-get install -y libjpeg8-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get install -y libgtk2.0-dev
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install -y libatlas-base-dev gfortran
sudo apt-get install -y libhdf5-serial-dev
sudo apt-get install -y python2.7-dev


#--------------------------------------------------------------------------------------------------------
#                        BUILD OPENCV
#--------------------------------------------------------------------------------------------------------


echo -e "${COLOR_LIGHT_GREEN}Building OpenCV${COLOR_NC}"

cd opencv-${OPENCV_VERSION}

# Create build directory if it doesn't exist
if [ ! -d build ] ; then
    mkdir build
fi

cd build

# delete log file if it exists
if [ -f cmake_log.txt ] ; then
    rm cmake_log.txt
fi

# Build opencv
if $CUDA ; then

    cmake -DCMAKE_INSTALL_PREFIX=${OPENCV_INSTALL_PREFIX} \
                 -DCMAKE_BUILD_TYPE=RELEASE \
                 -DWITH_CUDA=ON \
                 -DENABLE_FAST_MATH=ON \
                 -DCUDA_FAST_MATH=ON \
                 -DWITH_CUBLAS=ON \
                 -DINSTALL_PYTHON_EXAMPLES=ON \
                 -DENABLE_PRECOMPILED_HEADERS=OFF \
                 -DWITH_OPENMP=ON \
                 -DWITH_NVCUVID=ON \
                 -DWITH_QT=ON \
                 -DWITH_GTK=OFF \
                 -DOPENCV_EXTRA_MODULES_PATH=~/Downloads/opencv_contrib-${OPENCV_VERSION}/modules .. >> cmake_log.txt 2>&1
else

    cmake -DCMAKE_INSTALL_PREFIX=${OPENCV_INSTALL_PREFIX} \
                 -DCMAKE_BUILD_TYPE=RELEASE \
                 -DWITH_CUDA=OFF \
                 -DENABLE_FAST_MATH=1 \
                 -DCUDA_FAST_MATH=0 \
                 -DWITH_CUBLAS=0 \
                 -DINSTALL_PYTHON_EXAMPLES=ON \
                 -DENABLE_PRECOMPILED_HEADERS=OFF \
                 -DWITH_OPENMP=ON \
                 -DWITH_NVCUVID=OFF \
                 -DOPENCV_EXTRA_MODULES_PATH=~/Downloads/opencv_contrib-${OPENCV_VERSION}/modules .. >> cmake_log.txt 2>&1


fi

COMPLETE=$(grep 'Configuring done' cmake_log.txt)

if [[ $COMPLETE ]] ; then

    echo -e "${COLOR_LIGHT_GREEN}Build Complete${COLOR_NC}"

else

    echo -e "${COLOR_RED}Error occured!!!${COLOR_NC}"

    cat cmake_log.txt

    echo -e "${COLOR_RED}Error occured!!!${COLOR_NC}"

    exit

fi

make -j8

echo -e "${COLOR_LIGHT_GREEN}Finished Building OpenCV${COLOR_NC}"


#--------------------------------------------------------------------------------------------------------
#                        Install OPENCV
#--------------------------------------------------------------------------------------------------------

if [[ $INSTALL_OPENCV ]] ; then

    echo -e "${COLOR_LIGHT_GREEN}Installing OpenCV${COLOR_NC}"
    sudo make install
    sudo ldconfig

fi

echo -e "${COLOR_LIGHT_GREEN}Finished Installing OpenCV${COLOR_NC}"

# delete log file if it exists
if [ -f cmake_log.txt] ; then
    rm cmake_log.txt
fi
