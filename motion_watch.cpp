#include <unistd.h>
#include <chrono>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include <errno.h>
#include <sys/types.h>
#include <sys/inotify.h>
#include <limits.h>

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
    output_block_size = 16;

    alpha = 0.7f;
    beta = 4.0f;
    sizeThreshold = 0;

    // ffmpeg
    fmt_ctx = NULL;
    dec_ctx = NULL;
    frame = av_frame_alloc();
    if (!frame)
    {
        fprintf(stderr, "Could not allocate frame\n");
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

    if (nSectors > 0)
    {
        mbPerSectorX = nBlocksX / nSectors;
        mbPerSectorY = nBlocksY / nSectors;
        nSectorsX = nSectors;
        nSectorsY = nSectors;
    } 
    else if (nSectors == 0)
    {
        mbPerSectorX = 1;
        mbPerSectorY = 1;
        nSectorsX = nBlocksX;
        nSectorsY = nBlocksY;
    }
    else
    {
        mbPerSectorX = 1;
        mbPerSectorY = 1;
        nSectorsX = nBlocksX * 4;
        nSectorsY = nBlocksY * 4;
        output_block_size = 4;
    }
    output_width = nSectorsX * mbPerSectorX * output_block_size;
    output_height = nSectorsY * mbPerSectorY * output_block_size;
    input_width = dec_ctx->width;
    input_height = dec_ctx->height;
}

void MoveDetector::PrepareFrameBuffers()
{
    int i, j;
    for (i = 0; i < MAX_MAP_SIDE; ++i)
        for (j = 0; j < MAX_MAP_SIDE; ++j)
        {
            // mvGridSum[i][j] = 0;
            // mvGridMag[i][j] = 0;
            // mvGridArg[i][j] = 0;
            mvGridCoords[currFrameBuffer][i][j] = {};
            subMbTypes[currFrameBuffer][i][j] = 0;
        }
    for (i = 0; i < MAX_CONNAREAS; i++)
    {
        areaBuffer[currFrameBuffer][i] = {};
    }
}

/* void MoveDetector::MvScanFrame(int index, AVFrame *pict, AVCodecContext *ctx)
{
    int i, j, type, processed_frame;
    int mb_index, xy, dx, dy, mb_x, mb_y;
    uint8_t sector_x, sector_y, mb_sect_y, mb_sect_x;
    int sumx, sumy, nVectors;

    // prepare new array before
    PrepareFrameBuffers();   

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
                                    //mvGridSum[sector_y][sector_x] += (int)sqrt(dx * dx + dy * dy);
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
                                    //mvGridSum[sector_y][sector_x] += (int)sqrt(dx * dx + dy * dy);
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
                                    //mvGridSum[sector_y][sector_x] += (int)sqrt(dx * dx + dy * dy);
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
                                //mvGridSum[sector_y][sector_x] += (int)sqrt(dx * dx + dy * dy);
                                sumx += dx;
                                sumy += dy;
                                nVectors++;
                            }
                        } // type (0...2)
                    }     // pict->motion_val
                } // end x macroblock
            }     // end y macroblock
            //mvGridSum[sector_x][sector_y] = (int)sqrt(sumx * sumx + sumy * sumy);
            //mvGridArg[sector_y][sector_x] = atan2f((float)sumy / (float)nVectors, (float)sumx / (float)nVectors);
            //mvGridMag[sector_y][sector_x] = sqrt(sumx * sumx + sumy * sum) / (float)nVectors;

            mvGridCoords[currFrameBuffer][sector_y][sector_x] = {sumx / nVectors, sumy / nVectors};
        } // end x sector scan
    }     // end y sector scan

    // set sensible to mvGridSum[]
    // if (sensivity)
    // {
    //     for (i = 0; i < nSectorsY; i++)
    //     {
    //         for (j = 0; j < nSectorsX; j++)
    //         {
    //             mvGridSum[i][j] /= sensivity;
    //         }
    //     }
    // }

    // for (i = 0; i < nSectorsY; i++)
    // {
    //     for (j = 0; j < nSectorsX; j++)
    //     {
    //         if (mvGridSum[i][j])
    //             mvGridArg[i][j] = mvGridArg[i][j] * (float)180 / (float)M_PI + (float)180;
    //         else
    //             mvGridArg[i][j] = 0;
    //     }
    // }

    MotionFieldProcessing();
    currFrameBuffer = (currFrameBuffer + 1) % AREABUFFER_SIZE;
} */

void MoveDetector::MvScanFrameH(int index, AVFrame *pict, AVCodecContext *ctx)
{
    int i;
    int mb_x, mb_y;
    int subBlockY, subBlockX;
    const int subBlocksY = nSectorsY, subBlocksX = nSectorsX;
    int mv_x, mv_y;

    AVFrameSideData *sd = av_frame_get_side_data(frame, AV_FRAME_DATA_MOTION_VECTORS);
    if (!sd)
        return;
    const AVMotionVector *mvs = (const AVMotionVector *)sd->data;
    int mvsCount = sd->size / sizeof(*mvs);

    const int is_iframe = frame->pict_type == AV_PICTURE_TYPE_I;
    const int is_pframe = frame->pict_type == AV_PICTURE_TYPE_P;
    const int is_bframe = frame->pict_type == AV_PICTURE_TYPE_B;

    PrepareFrameBuffers();

    for (int mvIndex = 0; mvIndex < mvsCount; mvIndex++)
    {
        const AVMotionVector *mv = &mvs[mvIndex];
        const int direction = mv->source > 0;

        if ((direction == 0 && is_pframe) ||
            (direction == 0 && is_bframe) ||
            (direction == 1 && is_bframe))
        {
            mv_x = mv->src_x - mv->dst_x;
            mv_y = mv->src_x - mv->dst_y;
            //16x16
            if (mv->w == 16 && mv->h == 16)
            {
                subBlockX = mv->dst_x / 16 * 4;
                subBlockY = mv->dst_y / 16 * 4;
                for (i = 0; i < 16; i++)
                {
                    mvGridCoords[currFrameBuffer][subBlockY + (i >> 2)][subBlockX + (i & 3)] = {mv_x, mv_y};
                }
            }
            //16x8
            else if (mv->w == 16 && mv->h == 8)
            {
                subBlockX = mv->dst_x / 16 * 4;
                subBlockY = mv->dst_y / 8 * 2;
                for (i = 0; i < 8; i++)
                {
                    mvGridCoords[currFrameBuffer][subBlockY + (i >> 2)][subBlockX + (i & 3)] = {mv_x, mv_y};
                }
            }
            //8x16
            else if (mv->w == 8 && mv->h == 16)
            {
                subBlockX = mv->dst_x / 8 * 2;
                subBlockY = mv->dst_y / 16 * 4;
                for (i = 0; i < 8; i++)
                {
                    mvGridCoords[currFrameBuffer][subBlockY + (i >> 1)][subBlockX + (i & 1)] = {mv_x, mv_y};
                }
            }
            //8x8
            else if (mv->w == 8 && mv->h == 8)
            {
                subBlockX = mv->dst_x / 8 * 2;
                subBlockY = mv->dst_y / 8 * 2;
                for (i = 0; i < 4; i++)
                {
                    mvGridCoords[currFrameBuffer][subBlockY + (i >> 1)][subBlockX + (i & 1)] = {mv_x, mv_y};
                }
            }
        }
    }
    MotionFieldProcessing();
    currFrameBuffer = (currFrameBuffer + 1) % AREABUFFER_SIZE;
}

void MoveDetector::MotionFieldProcessing()
{
    if (delayedFrameNumber >= 0) {

        CalculateMagAng();

        // MorphologyProcess();
        // SpatialConsistProcess();

        TemporalConsistProcess();
        MorphologyProcess();
        
        fprintf(stderr, "motion data for frame %d (output frame %d)\n", currFrameNumber - 1, delayedFrameNumber - AREABUFFER_SIZE + 1);

        if (delayedFrameNumber >= AREABUFFER_SIZE - 3)
        {
            TrackedAreasFiltering();
            if (movemask_std_flag)
                WriteMapConsole();

            if (movemask_file_flag)
                WriteMaskFile(fvideomask_desc);
        }
    }    
}

void MoveDetector::SkipDummyFrame()
{
    //output last frame again
    if (movemask_file_flag)
        WriteMaskFile(fvideomask_desc);
}

void MoveDetector::MainDec()
{
    srand(time(NULL));
    int ret;
    count = 0;
    sum = 0;
    int got_frame, processed_frame;

    if (!frame)
    {
        perror("Could not allocate frame");
    }

    // read all packets
    int packet_n = 1;
    currFrameNumber = 0;
    processed_frame = 0;

    currFrameBuffer = 0;
    delayedFrameNumber = 1 - 3;

    chrono::high_resolution_clock::time_point start_t = chrono::high_resolution_clock::now();

    if (movemask_file_flag && USE_YUV2MPEG2)
        WriteMPEG2Header(fvideomask_desc);

    while (1)
    {
        if ((ret = av_read_frame(fmt_ctx, &packet)) < 0)
            break;

        if (packet.stream_index == video_stream_index && ((packet_n % packet_skip == 0) || (packet_n < 10)))
        {
            // avcodec_get_frame_defaults(frame);
            got_frame = 0;

            // ret = avcodec_decode_video2(dec_ctx, frame, &got_frame, &packet);
            ret = decode(dec_ctx, frame, &got_frame, &packet);
            if (ret < 0)
            {
                av_log(NULL, AV_LOG_ERROR, "Error decoding video\n");
                break;
            }

            AllocAnalyzeBuffers();

            if (got_frame)
            {
                currFrameNumber = frame->best_effort_timestamp / frame->pkt_duration;
                if (frame->pict_type != FF_I_TYPE)
                {
                    fprintf(stderr, "processing frame %d (packet no. %d, %d frames with MVs processed), \n", currFrameNumber, packet_n, processed_frame);

                    // multithread ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    // ToDO: .............

                    // one thread ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    if (nSectors >= 0)
                        // MvScanFrame(packet_n, frame, dec_ctx);
                        throw std::runtime_error("Can only wheelchair with -g -1");
                    else
                        MvScanFrameH(packet_n, frame, dec_ctx);
                    delayedFrameNumber++;
                    processed_frame++;
                    // if (movemask_file_flag)
                    // 	printf("Play mask file: mplayer -demuxer rawvideo -rawvideo w=%d:h=%d:format=y8 %s -loop 0 \n", output_width, output_height, mask_filename);
                }
                else
                {
                    fprintf(stderr, "skipping frame %d (packet no. %d, %d frames with MVs processed), \n", currFrameNumber, packet_n, processed_frame);
                    if (currFrameNumber)
                    {
                        SkipDummyFrame();
                    }
                }
            }
        }
        ++packet_n;
        av_free_packet(&packet);
    }

    chrono::high_resolution_clock::time_point end_t = chrono::high_resolution_clock::now();
    int64_t duration = chrono::duration_cast<chrono::microseconds>( end_t - start_t ).count();
    fprintf(stderr, "Total frames processed: %d\n", processed_frame);
    fprintf(stderr, "Execution time = %f\n", double(duration) / 1000000.0f);
    fprintf(stderr, "Average FPS: %4.3f\n", (double)processed_frame * 1000000.0f / double(duration));
    fprintf(stderr, "Video resolution: %dx%d; Framerate: %2.2f\n", dec_ctx->width, dec_ctx->height,
            (float)fmt_ctx->streams[video_stream_index]->r_frame_rate.num / fmt_ctx->streams[video_stream_index]->r_frame_rate.den);
    fprintf(stderr, "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

    if (movemask_file_flag)
        if (USE_YUV2MPEG2)
            fprintf(stderr, "Play mask file: mplayer %s -loop 0 \n\n",  mask_filename);
        else
            fprintf(stderr, "Play mask file: mplayer -demuxer rawvideo -rawvideo w=%d:h=%d:format=i420 %s -loop 0 \n\n", output_width, output_height, mask_filename);

    if (dec_ctx)
        avcodec_close(dec_ctx);
    if (fmt_ctx)
        avformat_close_input(&fmt_ctx);
    if (frame)
        av_freep(&frame);
}

void MoveDetector::Help(void)
{
    fprintf(stderr,
            "Usage: motion_detect [options] input_stream\n"
            "Options:\n\n"
            "  -g <n>                  Grid size.\n"
            "                          ex: <16> divides the input frame into a 16x16 grid. Max 120.\n"
            "                          Use <0> to divide into 16x16px blocks instead (default)\n"
            "                          (DISABLED: use 0 for 16x16 and -1 for 4x4 resolution\n\n"
            "  -c                      Write output map to console.\n\n"
            "  -o <filename.y4m>       Write output map to filename.y4m.\n\n"
            "  -p <n>                  Process only every n-th packet (default: 1).\n\n"
            "  -e <element>            Element to use for morphological closing.\n"
            "                          Can be a 3x3 <cross> (default) or a <square>.\n\n"
            "  -a <n>                  Alpha for MV preprocessing: interframe similarity threshold.\n"
            "                          Greater values will reject more MVs if they are not consistent between frames.\n"
            "                          Ranges from 0 to 100 (default: 70).\n\n"
            "  -b <n>                  Beta for MV preprocessing: vector magnitude threshold.\n"
            "                          MVs with magnitude lower than Beta (in px) will be rejected (default: 4).\n\n"
            "  -s <n>                  Threshold for detected area sizes. Default: 0 blocks (no thresholding).\n"
            "                          (Temporary solution against smaller local MV noise)\n\n");
    fprintf(stderr, "Using libavcodec version %d.%d.%d \n", LIBAVCODEC_VERSION_MAJOR, LIBAVCODEC_VERSION_MINOR, LIBAVCODEC_VERSION_MICRO);
}

static const char *mvOptions = {"g:o:p:e:a:b:s:c"};

void Initialize(int argc, char **argv)
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
                fprintf(stderr, "sector size cannot be greater than %d\n", MAX_MAP_SIDE);
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
                fprintf(stderr, "Error while opening mask videostream  %s\n", gout_filename);
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
            movedec.sizeThreshold = atoi(optarg);
            break;
        }
        // case 'a':
        // {
        //     movedec.amplify_yuv = atoi(optarg);
        //     break;
        // }
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
        // case 't':
        // {
        //     movedec.binThreshold = atoi(optarg);
        //     break;
        // }
        case 'e':
        {
            string argElement = optarg;
            if (argElement == "square")
                movedec.useSquareElement = 1;
            else
                movedec.useSquareElement = 0;
            break;
        }
        case 'a':
        {
            int alpha = atoi(optarg);
            if ((alpha > 100) || (alpha < 0))
            {
                fprintf(stderr, "alpha must be in (0..100)\n");
                movedec.Help();
                exit(0);
            }
            movedec.alpha = (float)alpha / 100.0f;
            break;
        }
        case 'b':
        {
            int beta = atoi(optarg);
            if (beta < 0)
            {
                fprintf(stderr, "beta must be greater than 0\n");
                movedec.Help();
                exit(0);
            }
            movedec.beta = (float)beta;
            break;
        }
        }
    }
    char *gfilename = argv[optind];
    if (!gfilename)
    {
        fprintf(stderr, "No input stream provided\n");
        exit(0);
    }
    if (movedec.OpenVideoFile(gfilename) < 0)
    {
        fprintf(stderr, "Error while opening orig videostream %s\n", gfilename);
        movedec.Help();
        exit(0);
    }

    movedec.MainDec();
    movedec.Close();
}

int main(int argc, char **argv)
{
    //FIXME: stop storing massive arrays on stack
    //seriously. disgusting.

    fprintf(stderr, "Increasing stack size...\n");

    const rlim_t kStackSize = 64 * 1024 * 1024; // 64 MB
    struct rlimit rl;
    int result;

    result = getrlimit(RLIMIT_STACK, &rl);
    if (result == 0)
    {
        if (rl.rlim_cur < kStackSize)
        {
            rl.rlim_cur = kStackSize;
            result = setrlimit(RLIMIT_STACK, &rl);
            if (result != 0)
            {
                fprintf(stderr, "setrlimit returned result = %d\n", result);
                return result;
            }
        }
    }

    Initialize(argc, argv);

    return 0;
}
