#ifndef MOTION_WATCH_H_
#define MOTION_WATCH_H_

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <vector>

extern "C" {
 #include <libavcodec/avcodec.h>
 #include <libavformat/avformat.h>
 #include <libavfilter/avcodec.h>
 #include <libavfilter/buffersink.h>
 #include <libavfilter/buffersrc.h>
}

#define IS_INTERLACED(a) 	((a)&MB_TYPE_INTERLACED)
#define IS_16X16(a)      	((a)&MB_TYPE_16x16)
#define IS_16X8(a)       	((a)&MB_TYPE_16x8)
#define IS_8X16(a)       	((a)&MB_TYPE_8x16)
#define IS_8X8(a)        	((a)&MB_TYPE_8x8)
#define USES_LIST(a, list) 	((a) & ((MB_TYPE_P0L0|MB_TYPE_P1L0)<<(2*(list))))

// FFMpeg interface change
#define FF_I_TYPE  AV_PICTURE_TYPE_I  			// Intra
#define FF_P_TYPE  AV_PICTURE_TYPE_P  			// Predicted
#define FF_B_TYPE  AV_PICTURE_TYPE_B  			// Bi-dir predicted
#define FF_S_TYPE  AV_PICTURE_TYPE_S  			// S(GMC)-VOP MPEG4
#define FF_SI_TYPE AV_PICTURE_TYPE_SI 			// Switching Intra
#define FF_SP_TYPE AV_PICTURE_TYPE_SP 			// Switching Predicted
#define FF_BI_TYPE AV_PICTURE_TYPE_BI
#define CODEC_TYPE_VIDEO AVMEDIA_TYPE_VIDEO

// program defines
#define MAX_MAP_SIDE 120
#define MAX_FILENAME 600
#define MAX_CONNAREAS 1000
#define AREABUFFER_SIZE 3

//default values
#define BIN_THRESHOLD 10
#define CONSIST_THRESHOLD 10
#define PACKET_SKIP 1
#define USE_SQUARE 0

//why even use enums?
#define MORPH_OP_ERODE 0
#define MORPH_OP_DILATE 1
#define MORPH_EL_CROSS 0
#define MORPH_EL_SQUARE 1

#define MV_PROJECT_FORWARDS -1
#define MV_PROJECT_BACKWARDS 1

#define BUFFER_NEXT(a) a
#define BUFFER_CURR(a) (((a - 1) % AREABUFFER_SIZE) + AREABUFFER_SIZE) % AREABUFFER_SIZE
#define BUFFER_PREV(a) (((a - 2) % AREABUFFER_SIZE) + AREABUFFER_SIZE) % AREABUFFER_SIZE

#define ROLLINGAVG(oldv, newv, lastsize) (newv + lastsize * oldv) / (lastsize + 1)

using namespace std;

class MoveDetector
{

  public:
	MoveDetector();
	virtual ~MoveDetector();

	struct coordinate
    {
        int x, y;
    };

	struct coordinateF
	{
        float x, y;
    };

    struct connectedArea
    {
		int id;
		int size;
		float directionX;
		float directionY;
		float directionXVar;
        float directionYVar;
        float directionMag;
        float directionAng;
        float centroidX;
        float centroidY;
        float uniformity;
        coordinate boundBoxU, boundBoxB;
        coordinateF delta, delta2, M2, normV;
        bool isTracked;
    };

    // debug file
	FILE *fvideo_desc;
	FILE *fvideomask_desc;
	char mask_filename[MAX_FILENAME];
	int movemask_file_flag;
	int movemask_std_flag;

	// memory
	//int mvGridSum[MAX_MAP_SIDE][MAX_MAP_SIDE];
	float mvGridArg[MAX_MAP_SIDE][MAX_MAP_SIDE];
	float mvGridMag[MAX_MAP_SIDE][MAX_MAP_SIDE];
	//coordinate mvGridCoords[MAX_MAP_SIDE][MAX_MAP_SIDE];

	int areaGridMarked[MAX_MAP_SIDE][MAX_MAP_SIDE];
    connectedArea areaBuffer[AREABUFFER_SIZE][MAX_CONNAREAS];
    coordinate mvGridCoords[AREABUFFER_SIZE][MAX_MAP_SIDE][MAX_MAP_SIDE];

    coordinateF bwProjected[MAX_MAP_SIDE][MAX_MAP_SIDE];
    coordinateF fwProjected[MAX_MAP_SIDE][MAX_MAP_SIDE];
    float similarityBW[MAX_MAP_SIDE][MAX_MAP_SIDE];
    float similarityFW[MAX_MAP_SIDE][MAX_MAP_SIDE];
    float similarityBWFW[MAX_MAP_SIDE][MAX_MAP_SIDE];
    int areaFgMarked[MAX_MAP_SIDE][MAX_MAP_SIDE];

    int currFrameBuffer;
    int delayedFrameNumber;
    int currFrameNumber;

    // tracking
	int nSectors;
	int nSectorsX;
	int nSectorsY;
	int nBlocksX;
	int nBlocksY;

	int output_width;
	int output_height;

	int mbPerSectorX;
	int mbPerSectorY;
	int sensivity;
	int amplify_yuv;

	int mb_stride;
	int mv_sample_log2;
	int mv_stride;
	int quarter_sample;
	int shift;

	// ffmpeg data
	AVFormatContext *fmt_ctx;
	AVCodecContext *dec_ctx;
	AVFrame *frame;
	AVPacket packet;
	int video_stream_index;
	int64_t last_pts;

	// misc and timing
	int count;
	double sum;
	int packet_skip;
	int useSquareElement;
	int binThreshold;

	// funcs
	void SetFileParams(char *gfilename, int gsector_size, char *gout_filename, int gsensivity, int gamplify);
	void WriteMaskFile(FILE *file);
	void WriteMapConsole();
	void Help(void);
	void AllocBuffers(void);
	void AllocAnalyzeBuffers(void);
	int OpenVideoFile(const char *filename);

	void MainDec();
	void MvScanFrame(int index, AVFrame *pict, AVCodecContext *ctx);

	void Close(void);

  private:
    void MotionFieldProcessing();

    void CalculateMagAng();
    void MorphologyProcess();
    void ErodeDilate(int kernelSize, int operation, int (*inputArray)[MAX_MAP_SIDE], int (*outputArray)[MAX_MAP_SIDE]);
	void DetectConnectedAreas(int (*inputArray)[MAX_MAP_SIDE], int (*outputArray)[MAX_MAP_SIDE]);
	void ProcessConnectedAreas(int (*markedAreas)[MAX_MAP_SIDE], connectedArea (&processedAreas)[MAX_CONNAREAS]);

    void TrackAreas();
    void SpatialConsistProcess();

    void TemporalConsistProcess();
    void ProjectMVectors(coordinate mVectors[][MAX_MAP_SIDE], coordinateF projected[][MAX_MAP_SIDE], int projectionDir = 1);
    void CalculateSimilarity(coordinate currentMV[][MAX_MAP_SIDE], coordinateF projectedMV[][MAX_MAP_SIDE], float metricOut[][MAX_MAP_SIDE]);
    void CalculateSimilarity(coordinateF currentMV[][MAX_MAP_SIDE], coordinateF projectedMV[][MAX_MAP_SIDE], float metricOut[][MAX_MAP_SIDE]);
    void DetectForeground();
    void SpatialFilter(int marked[][MAX_MAP_SIDE]);

    void PrepareFrameBuffers();
    void SkipDummyFrame();
};

#endif /* MOTION_WATCH_H_ */
