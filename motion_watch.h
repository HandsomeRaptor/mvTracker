#ifndef MOTION_WATCH_H_
#define MOTION_WATCH_H_

#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <vector>
#include <list>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavfilter/avcodec.h>
// #include <libavfilter/buffersink.h>
// #include <libavfilter/buffersrc.h>
#include <libavformat/avformat.h>
#include <libavutil/motion_vector.h>
#include <libavutil/avutil.h>
}

#define IS_INTERLACED(a) 	((a)&MB_TYPE_INTERLACED)
#define IS_16X16(a)      	((a)&MB_TYPE_16x16)
#define IS_16X8(a)       	((a)&MB_TYPE_16x8)
#define IS_8X16(a)       	((a)&MB_TYPE_8x16)
#define IS_8X8(a)        	((a)&MB_TYPE_8x8)
#define USES_LIST(a, list) 	((a) & ((MB_TYPE_P0L0|MB_TYPE_P1L0)<<(2*(list))))
#define IS_INTRA(a) ((a)&7)

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
#define MAX_MAP_SIDE 500
#define MAX_FILENAME 600
#define MAX_CONNAREAS 1000
#define AREABUFFER_SIZE 3
#define USE_YUV2MPEG2 1

//default values
#define BIN_THRESHOLD 10
//#define CONSIST_THRESHOLD 10
#define PACKET_SKIP 1
#define USE_SQUARE 0

//why even use enums?
#define MORPH_OP_ERODE 0
#define MORPH_OP_DILATE 1
#define MORPH_EL_CROSS 0
#define MORPH_EL_SQUARE 1

#define MV_PROJECT_FORWARDS -1
#define MV_PROJECT_BACKWARDS 1

#define SUBMB_TYPE_INTRA 1
#define SUBMB_TYPE_16x16 2
#define SUBMB_TYPE_16x8 3
#define SUBMB_TYPE_8x16 4
#define SUBMB_TYPE_8x8 5
#define SUBMB_TYPE_8x4 6
#define SUBMB_TYPE_4x8 7
#define SUBMB_TYPE_4x4 8

#define TRACKERSTATUS_NONE 0
#define TRACKERSTATUS_INTOFRAME 1
#define TRACKERSTATUS_OUTOFFRAME 2
#define TRACKERSTATUS_OCCLUSION 4
#define TRACKERSTATUS_TRACKING 8
#define TRACKERSTATUS_LOST 16

#define BUFFER_NEXT(a) a
#define BUFFER_CURR(a) (((a - 1) % AREABUFFER_SIZE) + AREABUFFER_SIZE) % AREABUFFER_SIZE
#define BUFFER_PREV(a) (((a - 2) % AREABUFFER_SIZE) + AREABUFFER_SIZE) % AREABUFFER_SIZE
#define BUFFER_OFFSET(a, b) (((a - (1 + b)) % AREABUFFER_SIZE) + AREABUFFER_SIZE) % AREABUFFER_SIZE
#define BUFFER_OLDEST(a) (((a + 1) % AREABUFFER_SIZE) + AREABUFFER_SIZE) % AREABUFFER_SIZE

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
        int areaID;
        int size;
        float directionX;
		float directionY;
		// float directionXVar;
        // float directionYVar;
        float directionMag;
        float directionAng;
        float centroidX;
        float centroidY;
        // float uniformity;
        coordinate boundBoxU, boundBoxB;
        // coordinateF delta, delta2, M2, normV;
        bool isTracked;
        bool isUsed;
        unsigned char areaStatus;
        int appearances;
    };

    struct trackedObject
    {
        int trackerID;
        int id;
        int areaID;
        int size;
        coordinate boundBoxU, boundBoxB;
        coordinate center;
        coordinate direction;
        coordinate predictedPos;
        unsigned char objStatus;
        float iou;
        coordinate candidatePos;
        int candidateAreaID;
        int candidateId;
        int lifeTime;
        connectedArea *candidateArea;

        trackedObject()
        {}
        trackedObject(connectedArea a)
        {
            trackerID = a.id;
            id = a.id;
            areaID = a.areaID;
            size = a.size;
            boundBoxU = a.boundBoxU;
            boundBoxB = a.boundBoxB;
            center.x = a.centroidX;
            center.y = a.centroidY;
            direction.x = -a.directionX;
            direction.y = -a.directionY;
            // predictedPos.x = center.x - direction.x;
            // predictedPos.y = center.y - direction.y;
            objStatus = TRACKERSTATUS_NONE;
            lifeTime = 3;
            iou = 0;
            candidatePos = {};
            candidateAreaID = 0;
            candidateId = 0;
        }

        void UpdateFromArea(connectedArea a)
        {
            size = a.size;
            boundBoxU = a.boundBoxU;
            boundBoxB = a.boundBoxB;
            center.x = a.centroidX;
            center.y = a.centroidY;
            direction.x = a.directionX;
            direction.y = a.directionY;
            objStatus = a.areaStatus;
        }
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

    int areaGridMarked[AREABUFFER_SIZE][MAX_MAP_SIDE][MAX_MAP_SIDE];
    connectedArea areaBuffer[AREABUFFER_SIZE][MAX_CONNAREAS];
    coordinate mvGridCoords[AREABUFFER_SIZE][MAX_MAP_SIDE][MAX_MAP_SIDE];
    int subMbTypes[AREABUFFER_SIZE][MAX_MAP_SIDE][MAX_MAP_SIDE];

    coordinateF bwProjected[MAX_MAP_SIDE][MAX_MAP_SIDE];
    coordinateF fwProjected[MAX_MAP_SIDE][MAX_MAP_SIDE];
    float similarityBW[MAX_MAP_SIDE][MAX_MAP_SIDE];
    float similarityFW[MAX_MAP_SIDE][MAX_MAP_SIDE];
    float similarityBWFW[MAX_MAP_SIDE][MAX_MAP_SIDE];
    int areaFgMarked[MAX_MAP_SIDE][MAX_MAP_SIDE];

    list<trackedObject> trackedObjects;

    int currFrameBuffer;
    int delayedFrameNumber;
    int currFrameNumber;

    // tracking
	int nSectors;
	int nSectorsX;
	int nSectorsY;
	int nBlocksX;
	int nBlocksY;

    int input_width;
    int input_height;
    int output_width;
    int output_height;
    int output_block_size;

    int mbPerSectorX;
	int mbPerSectorY;
	int sensivity;
	int amplify_yuv;

    float alpha;
    float beta;
    int sizeThreshold;

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
    bool perfTest;

    // funcs
    void SetFileParams(char *gfilename, int gsector_size, char *gout_filename, int gsensivity, int gamplify);
    void WriteMaskFile(FILE *file);
    void WriteFrameToFile(FILE *file, uint8_t Y[][MAX_MAP_SIDE], uint8_t U[][MAX_MAP_SIDE], uint8_t V[][MAX_MAP_SIDE]);
    void WriteMPEG2Header(FILE *file);
    void WriteMapConsole();
    void Help(void);
	void AllocBuffers(void);
	void AllocAnalyzeBuffers(void);
	int OpenVideoFile(const char *filename);
    int decode(AVCodecContext *avctx, AVFrame *frame, int *got_frame, AVPacket *pkt);

    void MainDec();
    void MvScanFrame(int index, AVFrame *pict, AVCodecContext *ctx);
    void MvScanFrameH(int index, AVFrame *pict, AVCodecContext *ctx);

    void Close(void);

  private:
    void MotionFieldProcessing();

    void CalculateMagAng();
    void MorphologyProcess();
    void ErodeDilate(int kernelSize, int operation, int (*inputArray)[MAX_MAP_SIDE], int (*outputArray)[MAX_MAP_SIDE]);
	void DetectConnectedAreas(int (*inputArray)[MAX_MAP_SIDE], int (*outputArray)[MAX_MAP_SIDE]);
    void DetectConnectedAreas2(int (*inputArray)[MAX_MAP_SIDE], int (*outputArray)[MAX_MAP_SIDE]);
    void ProcessConnectedAreas(int (*markedAreas)[MAX_MAP_SIDE], connectedArea (&processedAreas)[MAX_CONNAREAS]);

    void TrackAreas();
    void TrackedAreasFiltering();
    //void SpatialConsistProcess();

    void TemporalConsistProcess();
    void ProjectMVectors(coordinate mVectors[][MAX_MAP_SIDE], coordinateF projected[][MAX_MAP_SIDE], int projectionDir = 1);
    void CalculateSimilarity(coordinate currentMV[][MAX_MAP_SIDE], coordinateF projectedMV[][MAX_MAP_SIDE], float metricOut[][MAX_MAP_SIDE]);
    void CalculateSimilarity(coordinateF currentMV[][MAX_MAP_SIDE], coordinateF projectedMV[][MAX_MAP_SIDE], float metricOut[][MAX_MAP_SIDE]);
    void DetectForeground();
    void SpatialFilter(int marked[][MAX_MAP_SIDE]);

    void PrepareFrameBuffers();
    void SkipDummyFrame();

    void inline ValidateCoordinate(coordinate c);
};

#endif /* MOTION_WATCH_H_ */
