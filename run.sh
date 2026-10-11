#!/bin/bash

FILE_NAME=$1

echo "mkdir -p results/$FILE_NAME"
mkdir -p results/$FILE_NAME

echo "EP_CPP_MODE=cpp \
EP_NEED_3D_BPP=./results/$FILE_NAME/bpp_layout_$FILE_NAME.json \
SC_SEARCH_TRIALS=16 \
SC_SEARCH_MAX_SECONDS=60 \
SC_FEASIBILITY_BEAM=12 \
SC_FEASIBILITY_BRANCH=8 \
python -m scripts.run_test \
  --module-path agents/$FILE_NAME/ \
  --config-path configs/sample_config.json \
  --verbose True \
  --render-mode None \
  --result-dir results/$FILE_NAME/ \
  --result-fname evaluation_results_$FILE_NAME.json \
  2>&1 | tee ./results/$FILE_NAME/$FILE_NAME.log"

EP_CPP_MODE=cpp \
EP_NEED_3D_BPP=./results/$FILE_NAME/bpp_layout_$FILE_NAME.json \
SC_SEARCH_TRIALS=16 \
SC_SEARCH_MAX_SECONDS=60 \
SC_FEASIBILITY_BEAM=12 \
SC_FEASIBILITY_BRANCH=8 \
python -m scripts.run_test \
  --module-path agents/$FILE_NAME/ \
  --config-path configs/sample_config.json \
  --verbose True \
  --render-mode None \
  --result-dir results/$FILE_NAME/ \
  --result-fname evaluation_results_$FILE_NAME.json \
  2>&1 | tee ./results/$FILE_NAME/$FILE_NAME.log

# echo "EP_CPP_MODE=cpp EP_NEED_3D_BPP=./results/$FILE_NAME/bpp_layout_$FILE_NAME.json SC_SEARCH_TRIALS=16 SC_SEARCH_MAX_SECONDS=60 python -m scripts.run_test   --module-path agents/$FILE_NAME/   --config-path configs/sample_config.json   --verbose True   --render-mode human"
# EP_CPP_MODE=cpp EP_NEED_3D_BPP=./results/$FILE_NAME/bpp_layout_$FILE_NAME.json SC_SEARCH_TRIALS=16 SC_SEARCH_MAX_SECONDS=60 python -m scripts.run_test   --module-path agents/$FILE_NAME/   --config-path configs/sample_config.json   --verbose True   --render-mode human