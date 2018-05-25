#include "motion_watch.h"
#include <sstream>
#include <string>

typedef struct RgbColor
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RgbColor;

typedef struct HsvColor
{
    unsigned char h;
    unsigned char s;
    unsigned char v;
} HsvColor;

#define CLIP(X) ((X) > 255 ? 255 : (X) < 0 ? 0 : X)
#define CRGB2Y(R, G, B) CLIP((19595 * R + 38470 * G + 7471 * B) >> 16)
#define CRGB2Cb(R, G, B) CLIP((36962 * (B - CLIP((19595 * R + 38470 * G + 7471 * B) >> 16)) >> 16) + 128)
#define CRGB2Cr(R, G, B) CLIP((46727 * (R - CLIP((19595 * R + 38470 * G + 7471 * B) >> 16)) >> 16) + 128)

RgbColor HsvToRgb(HsvColor hsv)
{
    RgbColor rgb;
    unsigned char region, remainder, p, q, t;

    if (hsv.s == 0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    region = hsv.h / 43;
    remainder = (hsv.h - (region * 43)) * 6;

    p = (hsv.v * (255 - hsv.s)) >> 8;
    q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
    case 0:
        rgb.r = hsv.v;
        rgb.g = t;
        rgb.b = p;
        break;
    case 1:
        rgb.r = q;
        rgb.g = hsv.v;
        rgb.b = p;
        break;
    case 2:
        rgb.r = p;
        rgb.g = hsv.v;
        rgb.b = t;
        break;
    case 3:
        rgb.r = p;
        rgb.g = q;
        rgb.b = hsv.v;
        break;
    case 4:
        rgb.r = t;
        rgb.g = p;
        rgb.b = hsv.v;
        break;
    default:
        rgb.r = hsv.v;
        rgb.g = p;
        rgb.b = q;
        break;
    }

    return rgb;
}

HsvColor RgbToHsv(RgbColor rgb)
{
    HsvColor hsv;
    unsigned char rgbMin, rgbMax;

    rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : (rgb.g < rgb.b ? rgb.g : rgb.b);
    rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : (rgb.g > rgb.b ? rgb.g : rgb.b);

    hsv.v = rgbMax;
    if (hsv.v == 0)
    {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * long(rgbMax - rgbMin) / hsv.v;
    if (hsv.s == 0)
    {
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == rgb.r)
        hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
    else if (rgbMax == rgb.g)
        hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
    else
        hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

    return hsv;
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
    dec_ctx = avcodec_alloc_context3(NULL);
    if (!dec_ctx)
    {
        av_log(NULL, AV_LOG_INFO, "FFMpeg: cannot allocate a AVCodecContext\n");
        return ret;
    }
    ret = avcodec_parameters_to_context(dec_ctx, fmt_ctx->streams[video_stream_index]->codecpar);
    if (ret) {
        av_log(NULL, AV_LOG_INFO, "FFMpeg: context is NULL, exiting..\n");
        return ret;
    }
    AVDictionary *opts = 0;
    av_dict_set(&opts, "flags2", "+export_mvs", 0);
    if (avcodec_open2(dec_ctx, dec, &opts) < 0)
    {
        throw std::runtime_error("Failed to open codec");
    }

    if (AV_CODEC_ID_H264 == dec_ctx->codec_id)
    {    
        dec_ctx->flags2 |= AV_CODEC_FLAG2_CHUNKS;
        dec_ctx->flags2 |= AV_CODEC_FLAG2_FAST;
        dec_ctx->flags |= AV_CODEC_FLAG_GRAY;
        dec_ctx->skip_loop_filter = AVDISCARD_ALL;
        dec_ctx->skip_idct = AVDISCARD_ALL;
    }

    return 0;
}

int MoveDetector::decode(AVCodecContext *avctx, AVFrame *frame, int *got_frame, AVPacket *pkt)
{
    int ret;

    *got_frame = 0;

    if (pkt)
    {
        ret = avcodec_send_packet(avctx, pkt);
        // In particular, we don't expect AVERROR(EAGAIN), because we read all
        // decoded frames with avcodec_receive_frame() until done.
        if (ret < 0)
            return ret == AVERROR_EOF ? 0 : ret;
    }

    ret = avcodec_receive_frame(avctx, frame);
    if (ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF)
        return ret;
    if (ret >= 0)
        *got_frame = 1;

    return 0;
}

void MoveDetector::Close(void)
{
    if (dec_ctx)  avcodec_close(dec_ctx);
    if (fmt_ctx)  avformat_close_input(&fmt_ctx);
    if (frame)    av_freep(&frame);
    if (movemask_file_flag) fclose(fvideomask_desc);
}

void MoveDetector::WriteMaskFile(FILE *filemask) {

	int i, j;
    int u = 0;
    int sector_x, sector_y;
    //uint8_t tmp_table2d_sum[MAX_MAP_SIDE][MAX_MAP_SIDE];
    uint8_t outFrameY[MAX_MAP_SIDE][MAX_MAP_SIDE];
    uint8_t outFrameU[MAX_MAP_SIDE][MAX_MAP_SIDE];
    uint8_t outFrameV[MAX_MAP_SIDE][MAX_MAP_SIDE];
    uint8_t boundBoxType[MAX_MAP_SIDE][MAX_MAP_SIDE];
    connectedArea *detectedAreas = areaBuffer[BUFFER_OLDEST(currFrameBuffer)];

    // for (sector_y = 0; sector_y < nSectorsY; sector_y++)
    // {
    //     for (sector_x = 0; sector_x < nSectorsX; sector_x++)
    //     {
    //         boundBoxType[sector_y][sector_x] = 0;
    //         for (int u = 0; u < MAX_CONNAREAS; u++)
    //         {
    //             if (detectedAreas[u].id)
    //             {
    //                 // top border
    //                 if ((detectedAreas[u].boundBoxU.y == sector_y) &&
    //                     (detectedAreas[u].boundBoxU.x < sector_x) &&
    //                     (detectedAreas[u].boundBoxB.x > sector_x))
    //                 {
    //                     boundBoxType[sector_y][sector_x] = 1;
    //                     break;
    //                 }
    //                 // bottom border
    //                 else if ((detectedAreas[u].boundBoxB.y == sector_y) &&
    //                          (detectedAreas[u].boundBoxU.x < sector_x) &&
    //                          (detectedAreas[u].boundBoxB.x > sector_x))
    //                 {
    //                     boundBoxType[sector_y][sector_x] = 2;
    //                     break;
    //                 }
    //                 // left border
    //                 else if ((detectedAreas[u].boundBoxU.x == sector_x) &&
    //                          (detectedAreas[u].boundBoxU.y < sector_y) &&
    //                          (detectedAreas[u].boundBoxB.y > sector_y))
    //                 {
    //                     boundBoxType[sector_y][sector_x] = 3;
    //                     break;
    //                 }
    //                 // right border
    //                 else if ((detectedAreas[u].boundBoxB.x == sector_x) &&
    //                          (detectedAreas[u].boundBoxU.y < sector_y) &&
    //                          (detectedAreas[u].boundBoxB.y > sector_y))
    //                 {
    //                     boundBoxType[sector_y][sector_x] = 4;
    //                     break;
    //                 }
    //                 // top left corner
    //                 else if ((detectedAreas[u].boundBoxU.x == sector_x) &&
    //                          (detectedAreas[u].boundBoxU.y == sector_y))
    //                 {
    //                     boundBoxType[sector_y][sector_x] = 5;
    //                     break;
    //                 }
    //                 // top right cornder 
    //                 else if ((detectedAreas[u].boundBoxB.x == sector_x) &&
    //                          (detectedAreas[u].boundBoxU.y == sector_y))
    //                 {
    //                     boundBoxType[sector_y][sector_x] = 6;
    //                     break;
    //                 }
    //                 // bottom left
    //                 else if ((detectedAreas[u].boundBoxU.x == sector_x) &&
    //                          (detectedAreas[u].boundBoxB.y == sector_y))
    //                 {
    //                     boundBoxType[sector_y][sector_x] = 7;
    //                     break;
    //                 }
    //                 // bottom right
    //                 else if ((detectedAreas[u].boundBoxB.x == sector_x) &&
    //                          (detectedAreas[u].boundBoxB.y == sector_y))
    //                 {
    //                     boundBoxType[sector_y][sector_x] = 8;
    //                     break;
    //                 }
    //             }
    //             else
    //                 break;
    //         }
    //     }
    // }

    if (!amplify_yuv)
        amplify_yuv = 1;

    const uint8_t boxY = 255;
    HsvColor currColorHSV;
    RgbColor currColorRGB;
    currColorHSV.s = 255;
    currColorHSV.v = 255;

    while (detectedAreas[u].id > 0)
    {
        for (i = 0; i < nSectorsY; i++)
        {
            for (j = 0; j < nSectorsX; j++)
            {
                if (areaGridMarked[i][j])
                {
                    if (areaGridMarked[BUFFER_OLDEST(currFrameBuffer)][i][j] == detectedAreas[u].areaID)
                    {
                        if (detectedAreas[u].isTracked)
                        {
                            areaGridMarked[BUFFER_OLDEST(currFrameBuffer)][i][j] = detectedAreas[u].id;
                        }
                        else
                        {
                            areaGridMarked[BUFFER_OLDEST(currFrameBuffer)][i][j] = 0;
                        }
                    }
                }
            }
        }
        u++;
    }

    for (sector_y = 0; sector_y < nSectorsY; sector_y++)
    {
        for (j = 0; j < mbPerSectorY * output_block_size; j++)
        {
            for (sector_x = 0; sector_x < nSectorsX; sector_x++)
            {
                for (i = 0; i < output_block_size * mbPerSectorX; i++)
                {
                    outFrameY[sector_y][sector_x] = (uint8_t)32;
                    outFrameU[sector_y][sector_x] = (uint8_t)128;
                    outFrameV[sector_y][sector_x] = (uint8_t)128;

                    if (areaGridMarked[BUFFER_OLDEST(currFrameBuffer)][sector_y][sector_x] > 0)
                    {
                        currColorHSV.h = areaGridMarked[BUFFER_OLDEST(currFrameBuffer)][sector_y][sector_x] % 255;
                        currColorRGB = HsvToRgb(currColorHSV);
                        outFrameY[sector_y][sector_x] = (uint8_t)(CRGB2Y(currColorRGB.r,currColorRGB.g,currColorRGB.b));
                        outFrameU[sector_y][sector_x] = (uint8_t)(CRGB2Cb(currColorRGB.r, currColorRGB.g, currColorRGB.b));
                        outFrameV[sector_y][sector_x] = (uint8_t)(CRGB2Cr(currColorRGB.r, currColorRGB.g, currColorRGB.b));
                    }


                    // outFrameU[sector_y][sector_x] = (uint8_t)128;
                    // outFrameV[sector_y][sector_x] = (uint8_t)128;
                    // coordinate *v = &(mvGridCoords[BUFFER_CURR(currFrameBuffer)][sector_y][sector_x]);
                    // outFrameY[sector_y][sector_x] = (uint8_t)(sqrt(v->x * v->x + v->y * v->y) * 50);
                    // outFrameY[sector_y][sector_x] = (uint8_t)(areaGridMarked[BUFFER_CURR(currFrameBuffer)][sector_y][sector_x] * 20);

                }
            }
        }
    }
    for (auto const &i : trackedObjects)
    {
        outFrameY[int(i.centerY / output_block_size)][int(i.centerX / output_block_size)] = (uint8_t)255;
    }

    WriteFrameToFile(filemask, outFrameY, outFrameU, outFrameV);
}

void MoveDetector::WriteFrameToFile(FILE *filemask, uint8_t Y[][MAX_MAP_SIDE], uint8_t U[][MAX_MAP_SIDE], uint8_t V[][MAX_MAP_SIDE])
{
    const unsigned char frameheader[] = {0x46, 0x52, 0x41, 0x4D, 0x45, 0x0A};
    if (USE_YUV2MPEG2)
        fwrite((const void *)&(frameheader), sizeof(char), sizeof(frameheader), filemask);

    uint8_t i, j;
    int sector_x, sector_y;
    for (sector_y = 0; sector_y < nSectorsY; sector_y++)
    {
        for (j = 0; j < mbPerSectorY * output_block_size; j++)
        {
            for (sector_x = 0; sector_x < nSectorsX; sector_x++)
            {
                for (i = 0; i < output_block_size * mbPerSectorX; i++)
                {
                    fwrite((const void *)&(Y[sector_y][sector_x]), sizeof(uint8_t), sizeof(Y[sector_y][sector_x]), filemask);
                }
            }
        }
    }
    for (sector_y = 0; sector_y < nSectorsY; sector_y++)
    {
        for (j = 0; j < mbPerSectorY * output_block_size / 2; j++)
        {
            for (sector_x = 0; sector_x < nSectorsX; sector_x++)
            {
                for (i = 0; i < output_block_size * mbPerSectorX / 2; i++)
                {
                    fwrite((const void *)&(U[sector_y][sector_x]), sizeof(uint8_t), sizeof(U[sector_y][sector_x]), filemask);
                }
            }
        }
    }
    for (sector_y = 0; sector_y < nSectorsY; sector_y++)
    {
        for (j = 0; j < mbPerSectorY * output_block_size / 2; j++)
        {
            for (sector_x = 0; sector_x < nSectorsX; sector_x++)
            {
                for (i = 0; i < output_block_size * mbPerSectorX / 2; i++)
                {
                    fwrite((const void *)&(V[sector_y][sector_x]), sizeof(uint8_t), sizeof(V[sector_y][sector_x]), filemask);
                }
            }
        }
    }
}

void MoveDetector::WriteMPEG2Header(FILE *file)
{
    ostringstream header;
    const unsigned char spacer = {0x20};
    const unsigned char framespacer = {0x0A};
    header << "YUV4MPEG2" << spacer << "W" << dec_ctx->coded_width << spacer << "H" << dec_ctx->coded_height << spacer;
    header << "F" << fmt_ctx->streams[video_stream_index]->r_frame_rate.num << ":" << fmt_ctx->streams[video_stream_index]->r_frame_rate.den << spacer;
    header << "Ip" << spacer << "A1:1" << spacer << "C420" << framespacer;
    fwrite((const void *)(header.str().c_str()), sizeof(char), header.str().size(), file);
}

void MoveDetector::WriteMapConsole()
{
    int i, j;
    // fprintf(stderr, "\n\n ==== 2D MAP ====\n");

    // for (i = 0; i < nSectorsY; i++)
    // {
    //     for (j = 0; j < nSectorsX; j++)
    //     {
    //         //printf("%2.0f ", mvGridMag[i][j]);
    //         //printf("%4.0f ", mvGridArg[i][j]);
    //         //areaGridMarked[i][j] == 0 ? printf("  .") : fprintf(stdout, "%3d", areaGridMarked[i][j]);
    //         // fprintf(stdout, "%3d", bwProjected[i][j].x);
    //         fprintf(stderr, "%5d ", areaGridMarked[i][j]);
    //         // switch (areaFgMarked[i][j])
    //         // {
    //         // case 0:
    //         //     fprintf(stderr, "? ");
    //         //     break;
    //         // case 1:
    //         //     fprintf(stderr, "0 ");
    //         //     break;
    //         // case -1:
    //         //     fprintf(stderr, ". ");
    //         //     break;
    //         // case 2:
    //         //     fprintf(stderr, "o ");
    //         //     break;
    //         // case -2:
    //         //     fprintf(stderr, ", ");
    //         //     break;
    //         // case 3:
    //         //     fprintf(stderr, "8 ");
    //         //     break;
    //         // case -3:
    //         //     fprintf(stderr, "_ ");
    //         //     break;
    //         // case 4:
    //         //     fprintf(stderr, "# ");
    //         //     break;
    //         // case -4:
    //         //     fprintf(stderr, "\" ");
    //         //     break;
    //         // }
    //     }
    //     fprintf(stdout, "\n");
    // }

    // fprintf(stdout, "\n");

    // fprintf(stderr, "SIMILARITYBW \n");
    // for (i = 0; i < nSectorsY; i++)
    // {
    //     for (j = 0; j < nSectorsX; j++)
    //     {
    //         fprintf(stdout, "%4.2f ", similarityBW[i][j]);
    //     }
    //     fprintf(stdout, "\n");
    // }

    // fprintf(stdout, "\n");

    // fprintf(stderr, "MAGNITUDE FW\n");
    // for (i = 0; i < nSectorsY; i++)
    // {
    //     for (j = 0; j < nSectorsX; j++)
    //     {
    //         fprintf(stdout, "%3.0f", sqrt(fwProjected[i][j].x * fwProjected[i][j].x + fwProjected[i][j].y * fwProjected[i][j].y));
    //     }
    //     fprintf(stdout, "\n");
    // }

    // fprintf(stdout, "\n");

    // fprintf(stderr, "MAGNITUDE PREV\n");
    // for (i = 0; i < nSectorsY; i++)
    // {
    //     for (j = 0; j < nSectorsX; j++)
    //     {
    //         fprintf(stdout, "%3.0f", sqrt(mvGridCoords[BUFFER_PREV(currFrameBuffer)][i][j].x * mvGridCoords[BUFFER_PREV(currFrameBuffer)][i][j].x + mvGridCoords[BUFFER_PREV(currFrameBuffer)][i][j].y * mvGridCoords[BUFFER_PREV(currFrameBuffer)][i][j].y));
    //     }
    //     fprintf(stdout, "\n");
    // }

    // fprintf(stdout, "\n");

    // fprintf(stderr, "MAGNITUDE \n");
    // for (i = 0; i < nSectorsY; i++)
    // {
    //     for (j = 0; j < nSectorsX; j++)
    //     {
    //         fprintf(stdout, "%3.0f", mvGridMag[i][j]);
    //     }
    //     fprintf(stdout, "\n");
    // }

    // fprintf(stdout, "\n");

    // fprintf(stderr, "MAGNITUDE BW\n");
    // for (i = 0; i < nSectorsY; i++)
    // {
    //     for (j = 0; j < nSectorsX; j++)
    //     {
    //         fprintf(stdout, "%3.0f", sqrt(bwProjected[i][j].x * bwProjected[i][j].x + bwProjected[i][j].y * bwProjected[i][j].y));
    //     }
    //     fprintf(stdout, "\n");
    // }

    // fprintf(stdout, "\n");

    // fprintf(stderr, "MAGNITUDE NEXT\n");
    // for (i = 0; i < nSectorsY; i++)
    // {
    //     for (j = 0; j < nSectorsX; j++)
    //     {
    //         fprintf(stdout, "%3.0f", sqrt(mvGridCoords[BUFFER_NEXT(currFrameBuffer)][i][j].x * mvGridCoords[BUFFER_NEXT(currFrameBuffer)][i][j].x + mvGridCoords[BUFFER_NEXT(currFrameBuffer)][i][j].y * mvGridCoords[BUFFER_NEXT(currFrameBuffer)][i][j].y));
    //     }
    //     fprintf(stdout, "\n");
    // }

    // fprintf(stdout, "\n");

    fprintf(stderr, "---- Detected areas ---- \n");
    int currId = 1;
    i = 0;

    connectedArea *detectedAreas = areaBuffer[BUFFER_OLDEST(currFrameBuffer)];
    while (currId)
    {
        if ((i < MAX_CONNAREAS) && (detectedAreas[i].id != 0))
        {
            currId = detectedAreas[i].id;
            fprintf(stderr, "ID: %5d  Size: %5d  Center: (%6.2f %6.2f) dirX/dirY: %6.2f %6.2f  Mag/Angle: %6.2f %6.2f IsTracked: %d Status: %d Appearances: %d \n",
                    detectedAreas[i].id,
                    detectedAreas[i].size,
                    detectedAreas[i].centroidX,
                    detectedAreas[i].centroidY,
                    detectedAreas[i].directionX,
                    detectedAreas[i].directionY,
                    detectedAreas[i].directionMag,
                    detectedAreas[i].directionAng,
                    detectedAreas[i].isTracked,
                    detectedAreas[i].areaStatus,
                    detectedAreas[i].appearances);
            i++;
        }
        else
            break;
    }

    fprintf(stderr, "---- Tracked objects ---- \n");
    for (auto &i : trackedObjects)
    {
        fprintf(stderr, "ID: %5d  Size: %5d  Center: (%6.2f %6.2f) dirX/dirY: %6.2f %6.2f  Status: %d  FTL: %d\n",
                i.id,
                i.size,
                i.centerX,
                i.centerY,
                i.directionX,
                i.directionY,
                i.objStatus,
                i.framesToLive);
    }

    fprintf(stderr, "\n");
}

