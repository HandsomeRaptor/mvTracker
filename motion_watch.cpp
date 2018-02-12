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

#include "motion_watch.h"

// #################################################################
MoveDetector::MoveDetector()
{
    int i, j;

    sector_size = 0;
    sector_size_x = 0;
    sector_size_y = 0;
    mb_width = 0;
    mb_height = 0;
    mb_stride = 0;
    sector_max_mb_x = 0;
    sector_max_mb_y = 0;
    sensivity = 0;
    quarter_sample = 0;
    mv_sample_log2 = 0;
    mv_stride = 0;
    shift = 0;

    count = 0;
    sum = 0;

    packet_skip = 3;

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

// #################################################################
MoveDetector::~MoveDetector()
{  }

// #################################################################
void MoveDetector::AllocBuffers(void)
{
    avcodec_register_all();
    av_register_all();
}


// #################################################################
void MoveDetector::AllocAnalyzeBuffers() 
{
    mb_width  = (dec_ctx->width + 15) / 16;
    mb_height = (dec_ctx->height + 15) / 16;
    mb_stride = mb_width + 1;
    mv_sample_log2 = 4 - frame->motion_subsample_log2;
    mv_stride = (mb_width << mv_sample_log2) + (dec_ctx->codec_id == CODEC_ID_H264 ? 0 : 1);
    quarter_sample = (dec_ctx->flags & CODEC_FLAG_QPEL) != 0;
    shift = 1 + quarter_sample;

    if (sector_size != 0)
    {
        sector_max_mb_x = mb_width / sector_size;
        sector_max_mb_y = mb_height / sector_size;
        sector_size_x = sector_size;
        sector_size_y = sector_size;
    } else {
        sector_max_mb_x = 1;
        sector_max_mb_y = 1;
        sector_size_x = mb_width;
        sector_size_y = mb_height;
    }
    output_width = sector_size_x*sector_max_mb_x*16;
    output_height = sector_size_y*sector_max_mb_y*16;
}


// #################################################################
// int motion vector for each macroblock in this frame.  
void MoveDetector::MvScanFrame(int index, AVFrame *pict, AVCodecContext *ctx)
{
    int i, j, type, processed_frame;
    int mb_index, xy, dx, dy, mb_x, mb_y;
    uint8_t sector_x, sector_y, mb_sect_y, mb_sect_x;

    // prepare new array before
	for (i=0; i<MAX_MAP_SIDE; ++i)
		for (j=0; j<MAX_MAP_SIDE; ++j)
			gtable2d_sum[i][j] = 0;

    for (sector_x = 0; sector_x < sector_size_x; sector_x++)
    {
        for (sector_y = 0; sector_y < sector_size_y; sector_y++)
        {
            for (mb_sect_y = 0; mb_sect_y < sector_max_mb_y; mb_sect_y++)
            {
                for (mb_sect_x = 0; mb_sect_x < sector_max_mb_x; mb_sect_x++)
                {

                    mb_x = sector_x * sector_max_mb_x + mb_sect_x;
                    mb_y = sector_y * sector_max_mb_y + mb_sect_y;
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
                                    gtable2d_sum[sector_x][sector_y] += (int)sqrt(dx * dx + dy * dy);
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
                                    gtable2d_sum[sector_x][sector_y] += (int)sqrt(dx * dx + dy * dy);
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
                                    gtable2d_sum[sector_x][sector_y] += (int)sqrt(dx * dx + dy * dy);
                                }
                            }
                            else
                            {
                                xy = (mb_x + mb_y * mv_stride) << mv_sample_log2;
                                dx = (pict->motion_val[direction][xy][0] >> shift);
                                dy = (pict->motion_val[direction][xy][1] >> shift);
                                gtable2d_sum[sector_x][sector_y] += (int)sqrt(dx * dx + dy * dy);
                            }
                        }// type (0...2)
                    }// pict->motion_val

                }// end x macroblock
            }// end y macroblock
        }// end x sector scan
    }// end y sector scan

    // set sensible to gtable2d_sum[]
	if(sensivity) {
		for (i=0; i < sector_size_x; i++) {
			for (j=0; j < sector_size_y; j++) {
				gtable2d_sum[i][j] /= sensivity;
			}
		}
	}


	// show data
    if (movemask_std_flag)
    {
        printf("\n\n ==== 2D MAP ====\n");
        for (i = 0; i < sector_size_y; i++)
        {
            for (j = 0; j < sector_size_x; j++)
            {
                printf("%3d ", gtable2d_sum[j][i]);
            }
            printf("\n");
        }
    }

    if(movemask_file_flag) WriteMaskFile(fvideomask_desc);

}




// #################################################################

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

// #################################################################

// write sector
void MoveDetector::WriteMaskFile(FILE *filemask) {

	uint8_t i, j;
    uint8_t sector_x, sector_y, mb_sect_y, mb_sect_x;
    uint8_t tmp_table2d_sum[MAX_MAP_SIDE][MAX_MAP_SIDE];
    uint8_t zerodata;

    zerodata = 0;
    int sum1 =0, sm = 0;

    if(!amplify_yuv) amplify_yuv = 1;

	// visualize sector level (1 value --> sector zone)
    for (sector_y = 0; sector_y < sector_size_y; sector_y++) {
		for (mb_sect_y = 0; mb_sect_y < sector_max_mb_y*16; mb_sect_y++) {
			for (sector_x = 0; sector_x < sector_size_x; sector_x++) {
				for (i = 0; i < 16*sector_max_mb_x; i++) {
					tmp_table2d_sum[sector_x][sector_y] = (uint8_t) gtable2d_sum[sector_x][sector_y]*amplify_yuv;		// change amplify
					fwrite((const void *)&(tmp_table2d_sum[sector_x][sector_y]), sizeof(uint8_t), sizeof(tmp_table2d_sum[sector_x][sector_y]), filemask);
				}
			}
		}
    }
}

// #################################################################

// main loop
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

    //printf("video: size: %d x %d macroblocks\n", mb_height, mb_width);

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

// #################################################################

void MoveDetector::Close(void)
{
    if (dec_ctx)  avcodec_close(dec_ctx);
    if (fmt_ctx)  avformat_close_input(&fmt_ctx);
    if (frame)    av_freep(&frame);
    if (movemask_file_flag) fclose(fvideomask_desc);
}

// #################################################################

void MoveDetector::SetFileParams(char *gfilename, int gsector_size, char *gout_filename = NULL, int gsensivity = 0, int gamplify = 0)
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

	sector_size = 0;
    sector_size_x = 0;
    sector_size_y = 0;
	mb_width = 0;    
	mb_height = 0;
	mb_stride = 0;
	sector_max_mb_x = 0;
	sector_max_mb_y = 0;
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

    sector_size = gsector_size;
    sensivity = gsensivity;
    amplify_yuv = gamplify;

    return;

end:
	Help();
	exit(0);

}

// #################################################################

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
    "  -p <n>                  Process only every n-th packet (default: 3).\n\n"
);
}

static const char *mvOptions = {"g:o:s:a:p:c"};

// #################################################################

// main()
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
        }
    }
    char *gfilename = argv[optind];
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
