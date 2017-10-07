#! /bin/bash

# Create folder for Keras models
mkdir -p keras_models

# Get YAD2K
dir = "YAD2K"
if [ -d "$dir"]; then
  cd YAD2K
  git reset --hard
  cd ..
fi
git clone https://github.com/allanzelener/YAD2K.git

# Download weights and config files for YOLO models
wget -P data/ -nc https://pjreddie.com/media/files/tiny-yolo.weights
wget -P data/ -nc https://pjreddie.com/media/files/yolo.weights
wget -P data/ -nc https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/tiny-yolo.cfg
wget -P data/ -nc https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolo.cfg

cd YAD2K
# Make it work on Python 2.7
git fetch origin pull/33/head:python27
git checkout python27
pip install configparser

# Convert full YOLO model
./yad2k.py ../data/yolo.cfg ../data/yolo.weights ../keras_models/yolo.h5
# Fix bug in YAD2K
sed -i 's/read(16)/read(20)/g' yad2k.py
# Convert Tiny YOLO
./yad2k.py ../data/tiny-yolo.cfg ../data/tiny-yolo.weights ../keras_models/tiny-yolo.h5

# Convert Keras models to Tensorflow
cd ..
python load_yolo.py keras_models/yolo.h5 tf_models/
python load_yolo.py keras_models/tiny-yolo.h5 tf_models/

# Split the full YOLO model into 50MB chunks to fit on Github
cd tf_models
split -b 52428800 -d yolo.pb yolo.pb.
