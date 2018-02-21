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

    // ffmpeg
    fmt_ctx = NULL;
    dec_ctx = NULL;
    frame = avcodec_alloc_frame();
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
                mvGridArg[i][j] = mvGridArg[i][j] * (float)180 / (float)M_PI + (float)180;
            else
                mvGridArg[i][j] = 0;
        }
    }
    // show data
    if (movemask_std_flag)
        WriteMapConsole();

    if (movemask_file_flag)
        WriteMaskFile(fvideomask_desc);
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
    chrono::high_resolution_clock::time_point start_t = chrono::high_resolution_clock::now();

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
                    fprintf(stderr, "processing frame %d (actual frame %d, packet no. %d), \n", processed_frame, frame_n, packet_n);

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

    chrono::high_resolution_clock::time_point end_t = chrono::high_resolution_clock::now();
    int64_t duration = chrono::duration_cast<chrono::microseconds>( end_t - start_t ).count();
    fprintf(stderr, "Total frames processed: %d\n", processed_frame);
    fprintf(stderr, "Execution time = %f\n", double(duration) / 1000000.0f);
    fprintf(stderr, "Average FPS: %4.3f\n", (double)processed_frame * 1000000.0f / double(duration));
    fprintf(stderr, "Video resolution: %dx%d; Framerate: %2.2f\n", dec_ctx->width, dec_ctx->height,
            (float)fmt_ctx->streams[video_stream_index]->r_frame_rate.num / fmt_ctx->streams[video_stream_index]->r_frame_rate.den);
    fprintf(stderr, "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

    if (movemask_file_flag)
        fprintf(stderr, "Play mask file: mplayer -demuxer rawvideo -rawvideo w=%d:h=%d:format=y8 %s -loop 0 \n\n", output_width, output_height, mask_filename);

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
            "                          Can be a 3x3 <cross> (default) or a <square>.\n\n");
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

    return 0;
}
