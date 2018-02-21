#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include <errno.h>
#include <sys/types.h>
#include <sys/inotify.h>
#include <limits.h>
#include <string.h>
#include <stack>
#include <algorithm>
#include <iterator>

#include "motion_watch.h"

MoveDetector::MoveDetector()
{
    int i, j;

    nSectors = 0;
    nSectorsX = 0;
    nSectorsY = 0;
    nBlocksX = 0;
    nBlocksY = 0;
    mb_stride = 0;
    mbPerSectorX = 0;
    mbPerSectorY = 0;
    sensivity = 0;
    quarter_sample = 0;
    mv_sample_log2 = 0;
    mv_stride = 0;
    shift = 0;

    count = 0;
    sum = 0;

    packet_skip = PACKET_SKIP;
    useSquareElement = USE_SQUARE;
    binThreshold = BIN_THRESHOLD;

    fvideo_desc = NULL;
    fvideomask_desc = NULL;
    movemask_file_flag = 0;
    amplify_yuv = 255;
    movemask_std_flag = 0;

    output_width = 0;
    output_height = 0;

    // ffmpeg
    fmt_ctx = NULL;
    dec_ctx = NULL;
    frame = avcodec_alloc_frame();
    if (!frame)
    {
        fprintf(stdout, "Could not allocate frame\n");
        exit(0);
    }
    video_stream_index = -1;
    last_pts = AV_NOPTS_VALUE;

    // init logic arrays
    fvideomask_desc = NULL;
}

MoveDetector::~MoveDetector()
{  }

void MoveDetector::AllocBuffers(void)
{
    avcodec_register_all();
    av_register_all();
}

void MoveDetector::AllocAnalyzeBuffers() 
{
    nBlocksX = (dec_ctx->width + 15) / 16;
    nBlocksY = (dec_ctx->height + 15) / 16;
    mb_stride = nBlocksX + 1;
    mv_sample_log2 = 4 - frame->motion_subsample_log2;
    mv_stride = (nBlocksX << mv_sample_log2) + (dec_ctx->codec_id == CODEC_ID_H264 ? 0 : 1);
    quarter_sample = (dec_ctx->flags & CODEC_FLAG_QPEL) != 0;
    shift = 1 + quarter_sample;

    if (nSectors != 0)
    {
        mbPerSectorX = nBlocksX / nSectors;
        mbPerSectorY = nBlocksY / nSectors;
        nSectorsX = nSectors;
        nSectorsY = nSectors;
    } else {
        mbPerSectorX = 1;
        mbPerSectorY = 1;
        nSectorsX = nBlocksX;
        nSectorsY = nBlocksY;
    }
    output_width = nSectorsX*mbPerSectorX*16;
    output_height = nSectorsY*mbPerSectorY*16;
}

void MoveDetector::MvScanFrame(int index, AVFrame *pict, AVCodecContext *ctx)
{
    int i, j, type, processed_frame;
    int mb_index, xy, dx, dy, mb_x, mb_y;
    uint8_t sector_x, sector_y, mb_sect_y, mb_sect_x;
    int sumx, sumy, nVectors;

    // prepare new array before
    for (i = 0; i < MAX_MAP_SIDE; ++i)
        for (j = 0; j < MAX_MAP_SIDE; ++j)
        {
            mvGridSum[i][j] = 0;
            mvGridArg[i][j] = 0;
            mvGridCoords[i][j] = {};
        }

    for (sector_y = 0; sector_y < nSectorsY; sector_y++)
    {
        for (sector_x = 0; sector_x < nSectorsX; sector_x++)
        {
            sumx = 0;
            sumy = 0;
            nVectors = 0;
            for (mb_sect_y = 0; mb_sect_y < mbPerSectorY; mb_sect_y++)
            {
                for (mb_sect_x = 0; mb_sect_x < mbPerSectorX; mb_sect_x++)
                {

                    mb_x = sector_x * mbPerSectorX + mb_sect_x;
                    mb_y = sector_y * mbPerSectorY + mb_sect_y;
                    mb_index = mb_x + mb_y * mb_stride;

                    if (pict->motion_val)
                    {

                        for (type = 0; type < 3; type++)
                        {
                            int direction = 0;
                            switch (type)
                            {
                            case 0:
                                if (pict->pict_type != FF_P_TYPE)
                                    continue;
                                direction = 0;
                                break;
                            case 1:
                                if (pict->pict_type != FF_B_TYPE)
                                    continue;
                                direction = 0;
                                break;
                            case 2:
                                if (pict->pict_type != FF_B_TYPE)
                                    continue;
                                direction = 1;
                                break;
                            }

                            if (IS_8X8(pict->mb_type[mb_index]))
                            {
                                for (i = 0; i < 4; i++)
                                {
                                    xy = (mb_x * 2 + (i & 1) + (mb_y * 2 + (i >> 1)) * mv_stride) << (mv_sample_log2 - 1);
                                    dx = (pict->motion_val[direction][xy][0] >> shift);
                                    dy = (pict->motion_val[direction][xy][1] >> shift);
                                    mvGridSum[sector_y][sector_x] += (int)sqrt(dx * dx + dy * dy);
                                    sumx += dx;
                                    sumy += dy;
                                    nVectors++;
                                }
                            }
                            else if (IS_16X8(pict->mb_type[mb_index]))
                            {
                                for (i = 0; i < 2; i++)
                                {
                                    xy = (mb_x * 2 + (mb_y * 2 + i) * mv_stride) << (mv_sample_log2 - 1);
                                    dx = (pict->motion_val[direction][xy][0] >> shift);
                                    dy = (pict->motion_val[direction][xy][1] >> shift);
                                    if (IS_INTERLACED(pict->mb_type[mb_index]))
                                    {
                                        dy *= 2;
                                    }
                                    mvGridSum[sector_y][sector_x] += (int)sqrt(dx * dx + dy * dy);
                                    sumx += dx;
                                    sumy += dy;
                                    nVectors++;
                                }
                            }
                            else if (IS_8X16(pict->mb_type[mb_index]))
                            {
                                for (i = 0; i < 2; i++)
                                {
                                    xy = (mb_x * 2 + i + mb_y * 2 * mv_stride) << (mv_sample_log2 - 1);
                                    dx = (pict->motion_val[direction][xy][0] >> shift);
                                    dy = (pict->motion_val[direction][xy][1] >> shift);
                                    if (IS_INTERLACED(pict->mb_type[mb_index]))
                                    {
                                        dy *= 2;
                                    }
                                    mvGridSum[sector_y][sector_x] += (int)sqrt(dx * dx + dy * dy);
                                    sumx += dx;
                                    sumy += dy;
                                    nVectors++;
                                }
                            }
                            else
                            {
                                xy = (mb_x + mb_y * mv_stride) << mv_sample_log2;
                                dx = (pict->motion_val[direction][xy][0] >> shift);
                                dy = (pict->motion_val[direction][xy][1] >> shift);
                                mvGridSum[sector_y][sector_x] += (int)sqrt(dx * dx + dy * dy);
                                sumx += dx;
                                    sumy += dy;
                                    nVectors++;
                            }
                        } // type (0...2)
                    }     // pict->motion_val

                } // end x macroblock
            }     // end y macroblock
            //mvGridSum[sector_x][sector_y] = (int)sqrt(sumx * sumx + sumy * sumy);
            mvGridArg[sector_y][sector_x] = atan2f((float)sumy / (float)nVectors, (float)sumx / (float)nVectors);
            mvGridCoords[sector_y][sector_x] = {sumx, sumy};
        } // end x sector scan
    }     // end y sector scan

    // set sensible to mvGridSum[]
    if (sensivity)
    {
        for (i = 0; i < nSectorsY; i++)
        {
            for (j = 0; j < nSectorsX; j++)
            {
                mvGridSum[i][j] /= sensivity;
            }
        }
    }

    // morph enabled: 5.8s -> 6.6s
    MorphologyProcess();

    for (i = 0; i < nSectorsY; i++)
        {
            for (j = 0; j < nSectorsX; j++)
            {
                if (mvGridSum[i][j])
                mvGridArg[i][j] = mvGridArg[i][j] * (float)180 / (float) M_PI + (float)180;
                else mvGridArg[i][j] = 0;
            }
        }
    // show data
    if (movemask_std_flag)
    {
        printf("\n\n ==== 2D MAP ====\n");
        for (i = 0; i < nSectorsY; i++)
        {
            for (j = 0; j < nSectorsX; j++)
            {
                // printf("%3d ", mvGridSum[j][i]);
                //printf("%4.0f ", mvGridArg[j][i]);
                areaGridMarked[i][j] == 0 ? printf("  .") : printf("%3d", areaGridMarked[i][j]);
            }
            printf("\n");
        }

        printf("---- Detected areas ---- \n");
        int currId = 1;
        i = 0;
        while (currId)
        {
            if ((i < MAX_CONNAREAS) && (detectedAreas[i].id != 0))
            {
                currId = detectedAreas[i].id;
                printf("ID: %3d  Size: %5d  Center: (%5.2f %5.2f)  Mag/Angle: %6.2f %6.2f \n", 
                detectedAreas[i].id,
                detectedAreas[i].size,
                detectedAreas[i].centroidX,
                detectedAreas[i].centroidY,
                detectedAreas[i].directionMag,
                detectedAreas[i].directionAng);
                i++;
            } else break;
        }
    }

    if(movemask_file_flag) WriteMaskFile(fvideomask_desc);

}

void MoveDetector::MorphologyProcess(){

    int i,j;
    int u,v;    

    //threshold the array
    for (i = 0; i < nSectorsY; i++)
    {
        for (j = 0; j < nSectorsX; j++)
        {
            if (mvGridSum[i][j] < binThreshold)
                mvGridSum[i][j] = 0;
            else
                mvGridSum[i][j] = 1;
        }
    }

    //set edge pixels to black for fill to work properly
    for (i = 0; i < nSectorsX; i++)
        mvGridSum[0][i] = 0;
    for (i = 0; i < nSectorsX; i++)
        mvGridSum[nSectorsY - 1][i] = 0;
    for (i = 0; i < nSectorsY; i++)
        mvGridSum[i][0] = 0;
    for (i = 0; i < nSectorsY; i++)
        mvGridSum[i][nSectorsX - 1] = 0;

    //flood fill (stack-based)
    
    std::stack<coordinate> toFill;
    toFill.push({0, 0});

    coordinate top;
    while (!toFill.empty())
    {
        top = toFill.top();
        toFill.pop();
        if ((top.x >= 0 && top.y >= 0) 
            && (top.x < nSectorsX && top.y < nSectorsY) 
            && (mvGridSum[top.y][top.x]) == 0)
        {
            //paints background with '-1'
            mvGridSum[top.y][top.x] = -1;
            toFill.push({top.x + 1, top.y});
            toFill.push({top.x - 1, top.y});
            toFill.push({top.x, top.y + 1});
            toFill.push({top.x, top.y - 1});
        }
    }
    //all holes are now marked with zeros

    //fill holes, restore background to '0'
    for (i = 0; i < nSectorsY; i++)
    {
        for (j = 0; j < nSectorsX; j++)
        {
            switch (mvGridSum[i][j])
            {
            case 0:
                mvGridSum[i][j] = 1;
                break;
            case -1:
                mvGridSum[i][j] = 0;
                break;
            }
        }
    }

    //create a temp array
    int gtable_temp[MAX_MAP_SIDE][MAX_MAP_SIDE];
    for (i = 0; i < MAX_MAP_SIDE; ++i)
        for (j = 0; j < MAX_MAP_SIDE; ++j)
            gtable_temp[i][j] = mvGridSum[i][j];

    //morph closing
    //erode
    ErodeDilate(useSquareElement, MORPH_OP_ERODE, mvGridSum, gtable_temp);

    //remove pixels not adjacent to any other
    for (i = 0; i < MAX_MAP_SIDE; ++i)
        for (j = 0; j < MAX_MAP_SIDE; ++j)
            gtable_temp[i][j] = mvGridSum[i][j];

    for (i = 0 + 1; i < nSectorsY - 1; i++){
        for (j = 0 + 1; j < nSectorsX - 1; j++){
            if (
                (mvGridSum[i - 1][j] == 0)
                && (mvGridSum[i + 1][j] == 0)
                && (mvGridSum[i][j - 1] == 0)
                && (mvGridSum[i][j + 1] == 0)
            )
            gtable_temp[i][j] = 0;
        }
    }
    
    //dilate
    ErodeDilate(useSquareElement, MORPH_OP_DILATE, gtable_temp, mvGridSum);

    DetectConnectedAreas(mvGridSum, areaGridMarked);
    ProcessConnectedAreas(areaGridMarked, detectedAreas);


}

static const char kernelCross[3][3] = {
    {0,1,0},
    {1,1,1},
    {0,1,0}
    };
static const char kernelSquare[3][3] = {
    {1,1,1},
    {1,1,1},
    {1,1,1}
    };
static const int kernelSize = 3;

void MoveDetector::ErodeDilate(int useSquareKernel, int operation, int (*inputArray)[MAX_MAP_SIDE], int (*outputArray)[MAX_MAP_SIDE])
{
    //probably wont ever use kernel sizes larger than 3, so this is alright
    char convKernel[kernelSize][kernelSize];
    if (useSquareKernel){
        for (int i = 0; i < kernelSize; ++i)
            for (int j = 0; j < kernelSize; ++j)
                convKernel[i][j] = kernelSquare[i][j];
    } else
    {
        for (int i = 0; i < kernelSize; ++i)
            for (int j = 0; j < kernelSize; ++j)
                convKernel[i][j] = kernelCross[i][j];
    }

    //the following is, however, reusable
    int halfOffset = kernelSize / 2;
    int doOperation = 0;
    int u,v;
    for (int i = 0 + halfOffset; i < nSectorsY - halfOffset; i++)
    {
        for (int j = 0 + halfOffset; j < nSectorsX - halfOffset; j++)
        {
            doOperation = 0;
            //possible to optimize
            for (u = 0; u < kernelSize; u++)
            {
                for (v = 0; v < kernelSize; v++)
                {
                    if ((inputArray[i + u - halfOffset][j + v - halfOffset] == operation)
                        &&(convKernel[u][v]))
                    {
                        doOperation = 1;
                    }
                }
            }
            if (doOperation)
            {
                outputArray[i][j] = operation;
            }
            else
            {
                outputArray[i][j] = !operation;
            }
        }
    }
}

void MoveDetector::DetectConnectedAreas(int (*inputArray)[MAX_MAP_SIDE], int (*outputArray)[MAX_MAP_SIDE]){

    int i,j,u,v;

      for (j = 0; j < MAX_MAP_SIDE; ++j)
        for (i = 0; i < MAX_MAP_SIDE; ++i)
        {
            outputArray[i][j] = 0;
        }
    
    int areasCounter = 1;
    // ABC-mask area detection
    // TODO: rework to only do 2 passes
    for (i = 0 + 1; i < nSectorsY - 1; i++)
    {
        for (j = 0 + 1; j < nSectorsX - 1; j++)
        {
            // if A is not '0'
            if (inputArray[i][j])
            {
                // neither B nor C are labled
                if ((!outputArray[i - 1][j]) && (!outputArray[i][j - 1]))
                {
                    outputArray[i][j] = areasCounter;
                    areasCounter++;
                }
                // if B xor C is labled
                else if ((outputArray[i - 1][j] && !outputArray[i][j - 1]))
                {
                    outputArray[i][j] = outputArray[i - 1][j];
                }
                else if ((!outputArray[i - 1][j] && outputArray[i][j - 1]))
                {
                    outputArray[i][j] = outputArray[i][j - 1];
                }
                // if both B and C are labled
                else if (outputArray[i - 1][j] && outputArray[i][j - 1])
                {
                    if (outputArray[i - 1][j] == outputArray[i][j - 1])
                    {
                        outputArray[i][j] = outputArray[i - 1][j];
                    }
                    else
                    {
                        outputArray[i][j] = outputArray[i - 1][j];
                        for (u = 0 + 1; u < nSectorsY - 1; u++)
                        {
                            for (v = 0 + 1; v < nSectorsX - 1; v++)
                            {
                                if (outputArray[u][v] == outputArray[i][j - 1]){
                                    outputArray[u][v] = outputArray[i - 1][j];
                                }
                            }
                        }
                        
                    }
                }
            }
        }
    }
}

void MoveDetector::ProcessConnectedAreas(int (*markedAreas)[MAX_MAP_SIDE], connectedArea (&processedAreas)[MAX_CONNAREAS])
{

    int i, j;

    int areaCounter = 0;
    int currentArea = 0;
    connectedArea newArea;

    for (i = 0; i < MAX_CONNAREAS; i++)
    {
        processedAreas[i] = {};
    }

    for (i = 0; i < nSectorsY; i++)
    {
        for (j = 0; j < nSectorsX; j++)
        {
            if (markedAreas[i][j])
            {
                //search the list of areas to see if this one is added or not
                currentArea = markedAreas[i][j];
                connectedArea *findresult =
                    std::find_if(begin(processedAreas), end(processedAreas),
                                 [&currentArea](const connectedArea &x) { return x.id == currentArea; });
                //if this one is not in the list, add it
                if (findresult == end(processedAreas))
                {
                    newArea = {};
                    newArea.id = currentArea;
                    newArea.size = 1;
                    newArea.directionX = mvGridCoords[i][j].x;
                    newArea.directionY = mvGridCoords[i][j].y;
                    newArea.centroidX = j;
                    newArea.centroidY = i;
                    newArea.boundBoxU = {j, i};
                    newArea.boundBoxB = {j, i};
                    processedAreas[areaCounter] = newArea;
                    areaCounter++;
                }
                //otherwise, update existing entry with new values
                else
                {
                    if (findresult->boundBoxU.x > j)
                        findresult->boundBoxU.x = j;
                    if (findresult->boundBoxU.y > i)
                        findresult->boundBoxU.y = i;

                    if (findresult->boundBoxB.x < j)
                        findresult->boundBoxB.x = j;
                    if (findresult->boundBoxB.y < i)
                        findresult->boundBoxB.y = i;

                    //cumulative average
                    findresult->centroidX = (j + findresult->size * findresult->centroidX) / (findresult->size + 1);
                    findresult->centroidY = (i + findresult->size * findresult->centroidY) / (findresult->size + 1);

                    findresult->directionX += mvGridCoords[i][j].x;
                    findresult->directionY += mvGridCoords[i][j].y;

                    findresult->size++;
                }
            }
        }
    }

    for (i = 0; i < areaCounter; i++)
    {
        processedAreas[i].directionAng =
            atan2f((float)processedAreas[i].directionY / (float)processedAreas[i].size,
                   (float)processedAreas[i].directionX / (float)processedAreas[i].size) * (float)180 / (float) M_PI + (float)180;
        processedAreas[i].directionMag =
            sqrt(processedAreas[i].directionX * processedAreas[i].directionX +
                 processedAreas[i].directionY * processedAreas[i].directionY) / (float)processedAreas[i].size;
    }
}

int MoveDetector::OpenVideoFile(const char *video_name)
{
    int ret;
    AVCodec *dec;

    if ((ret = avformat_open_input(&fmt_ctx, video_name, NULL, NULL)) < 0) {
        av_log(NULL, AV_LOG_INFO, "FFMpeg: cannot open input file\n");
        return ret;
    }
    if ((ret = avformat_find_stream_info(fmt_ctx, NULL)) < 0) {
        av_log(NULL, AV_LOG_INFO, "FFMpeg: cannot find stream information\n");
        return ret;
    }
    // select the video stream
    ret = av_find_best_stream(fmt_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, &dec, 0);
    if (ret < 0) {
        av_log(NULL, AV_LOG_INFO, "FFMpeg: cannot find a video stream in the input file\n");
        return ret;
    }
    video_stream_index = ret;
    dec_ctx = fmt_ctx->streams[video_stream_index]->codec;
    if (!dec_ctx) {
        av_log(NULL, AV_LOG_INFO, "FFMpeg: context is NULL, exiting..\n");
        return ret;
    }
    // init the video decoder
    if ((ret = avcodec_open2(dec_ctx, dec, NULL)) < 0) {
        av_log(NULL, AV_LOG_INFO, "FFMpeg: cannot open video decoder\n");
        return ret;
    }

    return 0;
}

void MoveDetector::WriteMaskFile(FILE *filemask) {

	uint8_t i, j;
    uint8_t sector_x, sector_y;
    //uint8_t tmp_table2d_sum[MAX_MAP_SIDE][MAX_MAP_SIDE];
    uint8_t tmp_table2d_arg[MAX_MAP_SIDE][MAX_MAP_SIDE];
    uint8_t boundBoxType[MAX_MAP_SIDE][MAX_MAP_SIDE];

    //uint8_t zerodata;

    //zerodata = 0;
    //int sum1 =0, sm = 0;
    

    for (sector_y = 0; sector_y < nSectorsY; sector_y++)
    {
        for (sector_x = 0; sector_x < nSectorsX; sector_x++)
        {
            boundBoxType[sector_y][sector_x] = 0;
            for (int u = 0; u < MAX_CONNAREAS; u++)
            {
                if (detectedAreas[u].id)
                {
                    // top border
                    if ((detectedAreas[u].boundBoxU.y == sector_y) &&
                        (detectedAreas[u].boundBoxU.x < sector_x) &&
                        (detectedAreas[u].boundBoxB.x > sector_x))
                    {
                        boundBoxType[sector_y][sector_x] = 1;
                        break;
                    }
                    // bottom border
                    else if ((detectedAreas[u].boundBoxB.y == sector_y) &&
                             (detectedAreas[u].boundBoxU.x < sector_x) &&
                             (detectedAreas[u].boundBoxB.x > sector_x))
                    {
                        boundBoxType[sector_y][sector_x] = 2;
                        break;
                    }
                    // left border
                    else if ((detectedAreas[u].boundBoxU.x == sector_x) &&
                             (detectedAreas[u].boundBoxU.y < sector_y) &&
                             (detectedAreas[u].boundBoxB.y > sector_y))
                    {
                        boundBoxType[sector_y][sector_x] = 3;
                        break;
                    }
                    // right border
                    else if ((detectedAreas[u].boundBoxB.x == sector_x) &&
                             (detectedAreas[u].boundBoxU.y < sector_y) &&
                             (detectedAreas[u].boundBoxB.y > sector_y))
                    {
                        boundBoxType[sector_y][sector_x] = 4;
                        break;
                    }
                    // top left corner
                    else if ((detectedAreas[u].boundBoxU.x == sector_x) &&
                             (detectedAreas[u].boundBoxU.y == sector_y))
                    {
                        boundBoxType[sector_y][sector_x] = 5;
                        break;
                    }
                    // top right cornder 
                    else if ((detectedAreas[u].boundBoxB.x == sector_x) &&
                             (detectedAreas[u].boundBoxU.y == sector_y))
                    {
                        boundBoxType[sector_y][sector_x] = 6;
                        break;
                    }
                    // bottom left
                    else if ((detectedAreas[u].boundBoxU.x == sector_x) &&
                             (detectedAreas[u].boundBoxB.y == sector_y))
                    {
                        boundBoxType[sector_y][sector_x] = 7;
                        break;
                    }
                    // bottom right
                    else if ((detectedAreas[u].boundBoxB.x == sector_x) &&
                             (detectedAreas[u].boundBoxB.y == sector_y))
                    {
                        boundBoxType[sector_y][sector_x] = 8;
                        break;
                    }
                }
                else
                    break;
            }
        }
    }

    if (!amplify_yuv)
        amplify_yuv = 1;

    const uint8_t boxY = 255;
    for (sector_y = 0; sector_y < nSectorsY; sector_y++)
    {
        for (j = 0; j < mbPerSectorY * 16; j++)
        {
            for (sector_x = 0; sector_x < nSectorsX; sector_x++)
            {
                for (i = 0; i < 16 * mbPerSectorX; i++)
                {
                    // tmp_table2d_sum[sector_x][sector_y] = (uint8_t) mvGridSum[sector_x][sector_y]*amplify_yuv;		// change amplify
                    // fwrite((const void *)&(tmp_table2d_sum[sector_x][sector_y]), sizeof(uint8_t), sizeof(tmp_table2d_sum[sector_x][sector_y]), filemask);

                    if (mvGridSum[sector_y][sector_x])
                        tmp_table2d_arg[sector_y][sector_x] = (uint8_t)((mvGridArg[sector_y][sector_x] / 540.0f + 0.25f) * amplify_yuv);
                    else
                        tmp_table2d_arg[sector_y][sector_x] = (uint8_t)0;

                    if (
                        (j == 0 && boundBoxType[sector_y][sector_x] == 1) ||
                        (j == 15 && boundBoxType[sector_y][sector_x] == 2) ||
                        (i == 0 && boundBoxType[sector_y][sector_x] == 3) ||
                        (i == 15 && boundBoxType[sector_y][sector_x] == 4))
                    {
                        fwrite((const void *)&(boxY), sizeof(uint8_t), sizeof(tmp_table2d_arg[sector_y][sector_x]), filemask);
                    }
                    else if (
                        (j == 0 && (boundBoxType[sector_y][sector_x] == 5 || boundBoxType[sector_y][sector_x] == 6)) ||
                        (j == 15 && (boundBoxType[sector_y][sector_x] == 7 || boundBoxType[sector_y][sector_x] == 8)) ||
                        (i == 0 && (boundBoxType[sector_y][sector_x] == 5 || boundBoxType[sector_y][sector_x] == 7)) ||
                        (i == 15 && (boundBoxType[sector_y][sector_x] == 6 || boundBoxType[sector_y][sector_x] == 8)))
                    {
                        fwrite((const void *)&(boxY), sizeof(uint8_t), sizeof(tmp_table2d_arg[sector_y][sector_x]), filemask);
                    }
                    else
                    {
                        fwrite((const void *)&(tmp_table2d_arg[sector_y][sector_x]), sizeof(uint8_t), sizeof(tmp_table2d_arg[sector_y][sector_x]), filemask);
                    }
                }
            }
        }
    }    
}

void MoveDetector::MainDec()
{
    int ret;
    count = 0;
    sum = 0;
    int got_frame, processed_frame;

    if (!frame)
    {
        perror("Could not allocate frame");
    }

    dec_ctx->skip_loop_filter = AVDISCARD_ALL; // 1m12s -> 54s
    dec_ctx->flags2 |= CODEC_FLAG2_FAST;       // did nothing
    dec_ctx->flags |= CODEC_FLAG_GRAY;         // 55s->46s

    // read all packets
    int packet_n = 1;
    int frame_n = 0;
    processed_frame = 0;
    //time_t start_t, end_t;
    clock_t start_t, end_t;
    double diff_t;
    //time(&start_t);
    start_t = clock();

    //printf("video: size: %d x %d macroblocks\n", nBlocksY, nBlocksX);

    while (1)
    {
        //AVFilterBufferRef *picref;
        if ((ret = av_read_frame(fmt_ctx, &packet)) < 0)
            break;

        // if (packet.stream_index == video_stream_index) {
        if (packet.stream_index == video_stream_index && ((packet_n % packet_skip == 0) || (packet_n < 10)))
        {
            avcodec_get_frame_defaults(frame);
            got_frame = 0;

            ret = avcodec_decode_video2(dec_ctx, frame, &got_frame, &packet);
            if (ret < 0)
            {
                av_log(NULL, AV_LOG_ERROR, "Error decoding video\n");
                break;
            }

            AllocAnalyzeBuffers();

            if (got_frame)
            {
                frame_n = frame -> best_effort_timestamp / frame -> pkt_duration;
                if (frame->pict_type != FF_I_TYPE)
                {
                    printf("processing frame %d (actual frame %d, packet no. %d), \n", processed_frame, frame_n, packet_n);

                    // multithread ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    // ToDO: .............

                    // one thread ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    MvScanFrame(packet_n, frame, dec_ctx);
                    processed_frame++;
                    // if (movemask_file_flag)
                    // 	printf("Play mask file: mplayer -demuxer rawvideo -rawvideo w=%d:h=%d:format=y8 %s -loop 0 \n", output_width, output_height, mask_filename);
                }
                //frame_n++;
            }
        }
        ++packet_n;
        av_free_packet(&packet);
    }

end:
    //time(&end_t);
    end_t = clock();
    diff_t = ((double) (end_t - start_t)) / CLOCKS_PER_SEC;
    printf("TOTAL FRAMES PROCESSED = %d\n", processed_frame);
    //diff_t = difftime(end_t, start_t);
    printf("Execution time = %f\n", diff_t);
    //printf("[%d] sum mv: %f, total # mv: %d\n", (int)start_t, sum, count);
    printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

    if (movemask_file_flag)
    	printf("Play mask file: mplayer -demuxer rawvideo -rawvideo w=%d:h=%d:format=y8 %s -loop 0 \n\n", output_width, output_height, mask_filename);

    if (dec_ctx)  avcodec_close(dec_ctx);
    if (fmt_ctx)  avformat_close_input(&fmt_ctx);
    if (frame)    av_freep(&frame);
}

void MoveDetector::Close(void)
{
    if (dec_ctx)  avcodec_close(dec_ctx);
    if (fmt_ctx)  avformat_close_input(&fmt_ctx);
    if (frame)    av_freep(&frame);
    if (movemask_file_flag) fclose(fvideomask_desc);
}

/* void MoveDetector::SetFileParams(char *gfilename, int gsector_size, char *gout_filename = NULL, int gsensivity = 0, int gamplify = 0)
{
	int ret, error = 0;

	fvideo_desc;
	fvideomask_desc;
 
    // test gsector_size
    if(gsector_size > MAX_MAP_SIDE) {
    	printf("sector size can`t be more than %d\n", MAX_MAP_SIDE);
        goto end;
    }

    // input videofile
    if ((ret = OpenVideoFile(gfilename)) < 0) {
    	printf("Error while opening orig videostream %s\n", gfilename);
        goto end;
    }

    // output mask fileMoveDetector::MoveDetector()
{
	int i,j;

	nSectors = 0;
    nSectorsX = 0;
    nSectorsY = 0;
	nBlocksX = 0;    
	nBlocksY = 0;
	mb_stride = 0;
	mbPerSectorX = 0;
	mbPerSectorY = 0;
	sensivity = 0;
	quarter_sample = 0;
	mv_sample_log2 = 0;
	mv_stride = 0;
	shift = 0;

	count = 0;
	sum = 0;

    
	fvideo_desc = NULL;
	fvideomask_desc = NULL;
	movemask_file_flag = 0;
	amplify_yuv = 1;
    movemask_std_flag = 0;

	output_width = 0;
	output_height = 0;

	// ffmpeg
    fmt_ctx = NULL;
    dec_ctx = NULL;
	frame = avcodec_alloc_frame();
    if (!frame) {
        fprintf(stdout, "Could not allocate frame\n");
        exit(0);
    }
	video_stream_index = -1;
	last_pts = AV_NOPTS_VALUE;

    // init logic arrays
    fvideomask_desc = NULL;
}
    if(gout_filename) {
		strncpy(mask_filename, gout_filename, MAX_FILENAME);
		if ((fvideomask_desc = fopen(gout_filename, "wb")) == NULL) {
			printf("Error while opening mask videostream  %s\n", gfilename);
			movemask_file_flag = 0;
		}
		else movemask_file_flag = 1;
    }
    else movemask_file_flag = 0;

    nSectors = gsector_size;
    sensivity = gsensivity;
    amplify_yuv = gamplify;

    return;

end:
	Help();
	exit(0);

} */

void MoveDetector::Help(void)
{
 printf(
    "Usage: motion_detect [options] input_stream\n"
    "Options:\n\n"
    "  -g <n>                  Grid size.\n"
    "                          ex: <16> divides the input frame into a 16x16 grid. Max 120.\n"
    "                          Use <0> to divide into 16x16px blocks instead (default)\n\n"
    "  -c                      Write output map to console.\n\n"
    "  -o <filename.yuv>       Write output map to filename.yuv.\n\n"
    "  -s <n>                  Sensitivity.\n"
    "                          Divides resulting vector magnitude by <n>.\n\n"
    "  -a <n>                  Display amplification.\n"
    "                          Multiplies resulting vector magnitude by <n> (only for .yuv output).\n\n"
    "  -p <n>                  Process only every n-th packet (default: 1).\n\n"
    "  -t <n>                  Threshold to use for binarization (absolute value). Default: 15\n\n"
    "  -e <element>            Element to use for morphological closing.\n"
    "                          Can be a 3x3 <cross> (default) or a <square>.\n\n"
);
}

static const char *mvOptions = {"g:o:s:a:p:e:t:c"};

int main(int argc, char **argv)
{
    MoveDetector movedec;

    movedec.AllocBuffers();

    if (argc == 1)
    {
        movedec.Help();
        exit(0);
    }

    int opt;

    while ((opt = getopt(argc, argv, mvOptions)) != -1)
    {
        switch (opt)
        {
        case 'g':
        {
            int gsector_size = atoi(optarg);
            if (gsector_size > MAX_MAP_SIDE)
            {
                printf("sector size cannot be greater than %d\n", MAX_MAP_SIDE);
                movedec.Help();
                exit(0);
            }
            movedec.nSectors = gsector_size;
            break;
        }
        case 'o':
        {
            char *gout_filename = optarg;
            strncpy(movedec.mask_filename, gout_filename, MAX_FILENAME);
            if ((movedec.fvideomask_desc = fopen(gout_filename, "wb")) == NULL)
            {
                printf("Error while opening mask videostream  %s\n", gout_filename);
                movedec.movemask_file_flag = 0;
            }
            else
            {
                movedec.movemask_file_flag = 1;
            }
            break;
        }
        case 's':
        {
            movedec.sensivity = atoi(optarg);
            break;
        }
        case 'a':
        {
            movedec.amplify_yuv = atoi(optarg);
            break;
        }
        case 'c':
        {
            movedec.movemask_std_flag = 1;
            break;
        }
        case 'p':
        {
            movedec.packet_skip = atoi(optarg);
            break;
        }
        case 't':
        {
            movedec.binThreshold = atoi(optarg);
            break;
        }
        case 'e':
        {
            string argElement = optarg;
            if (argElement == "square")
                movedec.useSquareElement = 1;
            else
                movedec.useSquareElement = 0;
            break;
        }
        }
    }
    char *gfilename = argv[optind];
    if (!gfilename){
        printf("No input stream provided\n");
        exit(0);
    }
    if (movedec.OpenVideoFile(gfilename) < 0)
    {
        printf("Error while opening orig videostream %s\n", gfilename);
        movedec.Help();
        exit(0);
    }

    //   if(argc==6) 		movedec.SetFileParams( argv[1], atoi(argv[2]), argv[3], atoi(argv[4]), atoi(argv[5]) );
    //   else if (argc==3) movedec.SetFileParams( argv[1], atoi(argv[2]) );
    //   else {
    // 	  movedec.Help();
    // 	  exit(0);
    //   }

    movedec.MainDec();
    movedec.Close();

    return 0;
}
