#include "motion_watch.h"
#include <sstream>
#include <string>

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

void MoveDetector::Close(void)
{
    if (dec_ctx)  avcodec_close(dec_ctx);
    if (fmt_ctx)  avformat_close_input(&fmt_ctx);
    if (frame)    av_freep(&frame);
    if (movemask_file_flag) fclose(fvideomask_desc);
}

void MoveDetector::WriteMaskFile(FILE *filemask) {

	uint8_t i, j;
    int sector_x, sector_y;
    //uint8_t tmp_table2d_sum[MAX_MAP_SIDE][MAX_MAP_SIDE];
    uint8_t outFrameY[MAX_MAP_SIDE][MAX_MAP_SIDE];
    uint8_t outFrameU[MAX_MAP_SIDE / 2][MAX_MAP_SIDE / 2];
    uint8_t outFrameV[MAX_MAP_SIDE / 2][MAX_MAP_SIDE / 2];
    uint8_t boundBoxType[MAX_MAP_SIDE][MAX_MAP_SIDE];
    connectedArea *detectedAreas = areaBuffer[BUFFER_CURR(currFrameBuffer)];

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
    for (sector_y = 0; sector_y < nSectorsY; sector_y++)
    {
        for (j = 0; j < mbPerSectorY * output_block_size; j++)
        {
            for (sector_x = 0; sector_x < nSectorsX; sector_x++)
            {
                for (i = 0; i < output_block_size * mbPerSectorX; i++)
                {
                    // tmp_table2d_sum[sector_x][sector_y] = (uint8_t) mvGridSum[sector_x][sector_y]*amplify_yuv;		// change amplify
                    // fwrite((const void *)&(tmp_table2d_sum[sector_x][sector_y]), sizeof(uint8_t), sizeof(tmp_table2d_sum[sector_x][sector_y]), filemask);

                    //vector arg output
                    coordinate *v = &(mvGridCoords[BUFFER_CURR(currFrameBuffer)][sector_y][sector_x]);
                    //coordinateF *v = &(bwProjected[sector_y][sector_x]);

                    // if (v->x || v->y)
                    //     outFrameY[sector_y][sector_x] = (uint8_t)(((atan2f(v->y, v->x) * (float)180 / (float)M_PI + (float)180) / 540.0f + 0.25f) * amplify_yuv);
                    // else
                    //     outFrameY[sector_y][sector_x] = (uint8_t)0;

                    //vector mag output
                    outFrameY[sector_y][sector_x] = (uint8_t)(sqrt(v->x * v->x + v->y * v->y) * 100);

                    //similarity output
                    //outFrameY[sector_y][sector_x] = (uint8_t)(similarityBWFW[sector_y][sector_x] * 255.0f);

                    //markedFg output
                    switch (areaFgMarked[sector_y][sector_x])
                    {
                    case 0:
                        outFrameY[sector_y][sector_x] = (uint8_t)0;
                        break;
                    case 1:
                        outFrameY[sector_y][sector_x] = (uint8_t)255;
                        break;
                    case -1:
                        outFrameY[sector_y][sector_x] = (uint8_t)0;
                        break;
                    case 2:
                        outFrameY[sector_y][sector_x] = (uint8_t)255;
                        break;
                    case -2:
                        outFrameY[sector_y][sector_x] = (uint8_t)0;
                        break;
                    case 3:
                        outFrameY[sector_y][sector_x] = (uint8_t)255;
                        break;
                    case -3:
                        outFrameY[sector_y][sector_x] = (uint8_t)0;
                        break;
                    case 4:
                        outFrameY[sector_y][sector_x] = (uint8_t)128;
                        break;
                    case -4:
                        outFrameY[sector_y][sector_x] = (uint8_t)32;
                        break;
                    }
                    outFrameU[sector_y >> 1][sector_x >> 1] = (uint8_t)128;
                    outFrameV[sector_y >> 1][sector_x >> 1] = (uint8_t)128;
                    //sumbmtype output
                    // switch (subMbTypes[BUFFER_CURR(currFrameBuffer)][sector_y][sector_x])
                    // {
                    // case SUBMB_TYPE_INTRA:
                    //     outFrameY[sector_y][sector_x] = (uint8_t)255;
                    //     break;
                    // case SUBMB_TYPE_16x16:
                    //     outFrameY[sector_y][sector_x] = (uint8_t)0;
                    //     break;
                    // case SUBMB_TYPE_16x8:
                    //     outFrameY[sector_y][sector_x] = (uint8_t)0;
                    //     break;
                    // case SUBMB_TYPE_8x16:
                    //     outFrameY[sector_y][sector_x] = (uint8_t)0;
                    //     break;
                    // case SUBMB_TYPE_8x8:
                    //     outFrameY[sector_y][sector_x] = (uint8_t)128;
                    //     break;
                    // case SUBMB_TYPE_8x4:
                    //     outFrameY[sector_y][sector_x] = (uint8_t)0;
                    //     break;
                    // case SUBMB_TYPE_4x8:
                    //     outFrameY[sector_y][sector_x] = (uint8_t)0;
                    //     break;
                    // case SUBMB_TYPE_4x4:
                    //     outFrameY[sector_y][sector_x] = (uint8_t)0;
                    //     break;
                    // }

                    //after morph
                    // if (areaGridMarked[sector_y][sector_x] > 0)
                    //     outFrameY[sector_y][sector_x] = (uint8_t)255;
                    // else
                    //     outFrameY[sector_y][sector_x] = (uint8_t)0;

                    // if (subMbTypes[BUFFER_CURR(currFrameBuffer)][sector_y][sector_x] == SUBMB_TYPE_INTRA)
                    //     outFrameY[sector_y][sector_x] = (uint8_t)0;
                    // else
                    //     outFrameY[sector_y][sector_x] = (uint8_t)255;

                    // if (
                    //     (j == 0 && boundBoxType[sector_y][sector_x] == 1) ||
                    //     (j == 15 && boundBoxType[sector_y][sector_x] == 2) ||
                    //     (i == 0 && boundBoxType[sector_y][sector_x] == 3) ||
                    //     (i == 15 && boundBoxType[sector_y][sector_x] == 4))
                    // {
                    //     fwrite((const void *)&(boxY), sizeof(uint8_t), sizeof(outFrameY[sector_y][sector_x]), filemask);
                    // }
                    // else if (
                    //     (j == 0 && (boundBoxType[sector_y][sector_x] == 5 || boundBoxType[sector_y][sector_x] == 6)) ||
                    //     (j == 15 && (boundBoxType[sector_y][sector_x] == 7 || boundBoxType[sector_y][sector_x] == 8)) ||
                    //     (i == 0 && (boundBoxType[sector_y][sector_x] == 5 || boundBoxType[sector_y][sector_x] == 7)) ||
                    //     (i == 15 && (boundBoxType[sector_y][sector_x] == 6 || boundBoxType[sector_y][sector_x] == 8)))
                    // {
                    //     fwrite((const void *)&(boxY), sizeof(uint8_t), sizeof(outFrameY[sector_y][sector_x]), filemask);
                    // }
                    // else
                    // {
                        // fwrite((const void *)&(outFrameY[sector_y][sector_x]), sizeof(uint8_t), sizeof(outFrameY[sector_y][sector_x]), filemask);
                    // }
                }
            }
        }
    }
    WriteFrameToFile(filemask, outFrameY, outFrameU, outFrameV);
}

void MoveDetector::WriteFrameToFile(FILE *filemask, uint8_t Y[][MAX_MAP_SIDE], uint8_t U[][MAX_MAP_SIDE / 2], uint8_t V[][MAX_MAP_SIDE / 2])
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
                    fwrite((const void *)&(U[sector_y / 2][sector_x / 2]), sizeof(uint8_t), sizeof(U[sector_y / 2][sector_x / 2]), filemask);
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
                    fwrite((const void *)&(V[sector_y / 2][sector_x / 2]), sizeof(uint8_t), sizeof(V[sector_y / 2][sector_x / 2]), filemask);
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
    header << "YUV4MPEG2" << spacer << "W" << dec_ctx->width << spacer << "H" << dec_ctx->height << spacer;
    header << "F" << fmt_ctx->streams[video_stream_index]->r_frame_rate.num << ":" << fmt_ctx->streams[video_stream_index]->r_frame_rate.den << spacer;
    header << "Ip" << spacer << "A1:1" << spacer << "C420" << framespacer;
    fwrite((const void *)(header.str().c_str()), sizeof(char), header.str().size(), file);
}

void MoveDetector::WriteMapConsole()
{
    int i, j;
    fprintf(stderr, "\n\n ==== 2D MAP ====\n");

    for (i = 0; i < nSectorsY; i++)
    {
        for (j = 0; j < nSectorsX; j++)
        {
            //printf("%2.0f ", mvGridMag[i][j]);
            //printf("%4.0f ", mvGridArg[i][j]);
            //areaGridMarked[i][j] == 0 ? printf("  .") : fprintf(stdout, "%3d", areaGridMarked[i][j]);
            // fprintf(stdout, "%3d", bwProjected[i][j].x);
            //fprintf(stderr, "%2d", areaFgMarked[i][j]);
            switch (areaFgMarked[i][j])
            {
            case 0:
                fprintf(stderr, "? ");
                break;
            case 1:
                fprintf(stderr, "0 ");
                break;
            case -1:
                fprintf(stderr, ". ");
                break;
            case 2:
                fprintf(stderr, "o ");
                break;
            case -2:
                fprintf(stderr, ", ");
                break;
            case 3:
                fprintf(stderr, "8 ");
                break;
            case -3:
                fprintf(stderr, "_ ");
                break;
            case 4:
                fprintf(stderr, "# ");
                break;
            case -4:
                fprintf(stderr, "\" ");
                break;
            }
        }
        fprintf(stdout, "\n");
    }

    fprintf(stdout, "\n");

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

    connectedArea *detectedAreas = areaBuffer[BUFFER_CURR(currFrameBuffer)];
    while (currId)
    {
        if ((i < MAX_CONNAREAS) && (detectedAreas[i].id != 0))
        {
            currId = detectedAreas[i].id;
            fprintf(stderr, "ID: %3d  Size: %5d  Center: (%5.2f %5.2f)  Mag/Angle: %6.2f %6.2f Nonuniformity: %3.2f \n",
                    detectedAreas[i].id,
                    detectedAreas[i].size,
                    detectedAreas[i].centroidX,
                    detectedAreas[i].centroidY,
                    detectedAreas[i].directionMag,
                    detectedAreas[i].directionAng,
                    detectedAreas[i].uniformity);
            i++;
        }
        else
            break;
    }

    fprintf(stderr, "\n");
                }