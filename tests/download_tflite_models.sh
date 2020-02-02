#!/bin/bash

if [ $# -eq 0 ]; then
  DATA_DIR="models/mobilenet_v1"
else
  DATA_DIR="$1"
fi

# Install required packages
#python3 -m pip install -r requirements.txt

# Get TF Lite model and labels
#curl -O http://storage.googleapis.com/download.tensorflow.org/models/tflite/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip
#unzip coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip -d ${DATA_DIR}
#rm coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip
curl -O https://storage.googleapis.com/download.tensorflow.org/models/tflite_11_05_08/mobilenet_v2_1.0_224.tgz
unzip mobilenet_v2_1.0_224.tgz -d ${DATA_DIR}
rm mobilenet_v2_1.0_224.tgz

# Get a labels file with corrected indices, delete the other one
#(cd ${DATA_DIR} && curl -O  https://dl.google.com/coral/canned_models/coco_labels.txt)
#rm ${DATA_DIR}/labelmap.txt

echo -e "Files downloaded to ${DATA_DIR}"