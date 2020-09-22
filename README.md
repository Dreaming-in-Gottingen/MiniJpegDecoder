# MiniJpegDecoder
jpeg decoder practice
decode jpeg to yuv file, you can dump MCU data in every step.

--------------------------------------------------------------
how to build?

step1. utils build
  cd utils; make

step2. decoder build
  cd decoder; make

--------------------------------------------------------------
how to run?

step1. copy lib and source env
  copy libcodec_utils.so to decoder dir
  export LD_LIBRARY_PATH=.

step2. run the bin
  ./JpegDecoder xx.jpg
