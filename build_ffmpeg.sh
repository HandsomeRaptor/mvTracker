#!/bin/sh

ENV_ROOT=`pwd`
FFMPEG_BUILD_FOLDER=build_system

set -e
set -u

jflag=
jval=4

while getopts 'j:' OPTION
do
  case $OPTION in
  j)	jflag=1
        	jval="$OPTARG"
	        ;;
  ?)	printf "Usage: %s: [-j concurrency_level] (hint: your cores + 20%%)\n" $(basename $0) >&2
		exit 2
		;;
  esac
done
shift $(($OPTIND - 1))

if [ "$jflag" ]
then
  if [ "$jval" ]
  then
    printf "Option -j specified (%d)\n" $jval
  fi
fi

cd `dirname $0`



# Source this shell script to get the same environment as the build script
#. ./env.source
if [ -z "$ENV_ROOT" ]; then
  if [ -f "./env.source" ]; then
    ENV_ROOT=`pwd`
    export ENV_ROOT
  fi
fi

if [ -z "$ENV_ROOT" ]; then
  echo "Missing ENV_ROOT variable" >&2
elif [ "${ENV_ROOT#/}" = "$ENV_ROOT" ]; then
  echo "ENV_ROOT must be an absolute path" >&2
else

  BUILD_DIR="${BUILD_DIR:-$ENV_ROOT/$FFMPEG_BUILD_FOLDER/build}"
  TARGET_DIR="${TARGET_DIR:-$ENV_ROOT/$FFMPEG_BUILD_FOLDER/target}"
  echo $BUILD_DIR
  echo $TARGET_DIR
#  DOWNLOAD_DIR="${DOWNLOAD_DIR:-$ENV_ROOT/dl}"
#  BIN_DIR="${BIN_DIR:-$ENV_ROOT/bin}"

  export LDFLAGS="-L${TARGET_DIR}/lib"
  export DYLD_LIBRARY_PATH="${TARGET_DIR}/lib"
  export PKG_CONFIG_PATH="$TARGET_DIR/lib/pkgconfig"
  #export CFLAGS="-I${TARGET_DIR}/include $LDFLAGS -static-libgcc -Wl,-Bstatic -lc"
  export CFLAGS="-I${TARGET_DIR}/include $LDFLAGS"
  export PATH="${TARGET_DIR}/bin:${PATH}"
  # Force PATH cache clearing
  hash -r
fi


# rm -rf "$BUILD_DIR" "$TARGET_DIR"
mkdir -p "$BUILD_DIR" "$TARGET_DIR"

# FFMpeg
echo "*** Building FFmpeg ***"
cd $BUILD_DIR/ffmpeg*

# add "--prefix=${OUTPUT_DIR:-$TARGET_DIR}" to /make custom install to "target folder"
CFLAGS="-I$TARGET_DIR/include" LDFLAGS="-L$TARGET_DIR/lib -lm" ./configure --prefix=${OUTPUT_DIR:-$TARGET_DIR} --extra-version=shared --disable-debug --enable-shared --disable-static --extra-cflags=--shared --disable-ffplay --disable-ffserver --disable-doc --enable-gpl --enable-pthreads --enable-pic --enable-postproc --enable-gray --enable-runtime-cpudetect --enable-nonfree --enable-version3 --disable-devices --disable-swresample --disable-swscale --disable-encoders --disable-filters --disable-hwaccels --disable-decoders --enable-decoder=h264 --disable-demuxers --enable-demuxer=h264 --enable-demuxer=rtp --enable-demuxer=rtsp --enable-demuxer=avi --enable-demuxer=mpegts --enable-demuxer=aac --enable-demuxer=mp3 --disable-muxers --enable-muxer=matroska --enable-muxer=mp4 --enable-muxer=avi --enable-muxer=mpeg2video --enable-muxer=mpegts --enable-muxer=rtp --enable-muxer=rtsp --disable-bzlib --disable-zlib

make -j2 && make install
