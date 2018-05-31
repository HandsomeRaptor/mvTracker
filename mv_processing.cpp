#include <algorithm>
#include <stack>

#include "motion_watch.h"

void MoveDetector::CalculateMagAng()
{
    int i, j;
    for (i = 0; i < MAX_MAP_SIDE; ++i)
        for (j = 0; j < MAX_MAP_SIDE; ++j)
        {
            mvGridMag[i][j] = 0;
            mvGridArg[i][j] = 0;
        }

    auto *mvGrid = mvGridCoords[BUFFER_CURR(currFrameBuffer)];

    for (i = 0; i < nSectorsY; i++)
    {
        for (j = 0; j < nSectorsX; j++)
        {
            mvGridArg[i][j] = atan2f(mvGrid[i][j].y, mvGrid[i][j].x);
            mvGridArg[i][j] = mvGridArg[i][j] * (float)180 / (float)M_PI + (float)180;
            
            mvGridMag[i][j] = sqrt(mvGrid[i][j].x *
                                       mvGrid[i][j].x +
                                   mvGrid[i][j].y *
                                       mvGrid[i][j].y);
        }
    }
}

void MoveDetector::MorphologyProcess()
{
    int i, j;
    int u, v;

    int mvMask[MAX_MAP_SIDE][MAX_MAP_SIDE];

    // //threshold MVs by vector magnitude
    // for (i = 0; i < nSectorsY; i++)
    // {
    //     for (j = 0; j < nSectorsX; j++)
    //     {
    //         if (mvGridMag[i][j] < binThreshold)
    //             mvMask[i][j] = 0;
    //         else
    //             mvMask[i][j] = 1;
    //     }
    // }

    //plug in fg-bg mask from temporal filtering instead
    for (i = 0; i < nSectorsY; i++)
    {
        for (j = 0; j < nSectorsX; j++)
        {
            mvMask[i][j] = areaFgMarked[i][j] > 0 ? 1 : 0;
        }
    }

    //set edge pixels to black for fill to work properly
    for (i = 0; i < nSectorsX; i++)
        mvMask[0][i] = 0;
    for (i = 0; i < nSectorsX; i++)
        mvMask[nSectorsY - 1][i] = 0;
    for (i = 0; i < nSectorsY; i++)
        mvMask[i][0] = 0;
    for (i = 0; i < nSectorsY; i++)
        mvMask[i][nSectorsX - 1] = 0;

    //flood fill (stack-based)

    std::stack<coordinate> toFill;
    toFill.push({0, 0});

    coordinate top;
    while (!toFill.empty())
    {
        top = toFill.top();
        toFill.pop();
        if ((top.x >= 0 && top.y >= 0) && (top.x < nSectorsX && top.y < nSectorsY) && (mvMask[top.y][top.x]) == 0)
        {
            //paints background with '-1'
            mvMask[top.y][top.x] = -1;
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
            switch (mvMask[i][j])
            {
            case 0:
                mvMask[i][j] = 1;
                break;
            case -1:
                mvMask[i][j] = 0;
                break;
            }
        }
    }

    //create a temp array
    int mvMask_temp[MAX_MAP_SIDE][MAX_MAP_SIDE];
    for (i = 0; i < MAX_MAP_SIDE; ++i)
        for (j = 0; j < MAX_MAP_SIDE; ++j)
            mvMask_temp[i][j] = mvMask[i][j];

    //morph closing
    //erode
    ErodeDilate(useSquareElement, MORPH_OP_ERODE, mvMask, mvMask_temp);

    //remove pixels not adjacent to any other
    for (i = 0; i < MAX_MAP_SIDE; ++i)
        for (j = 0; j < MAX_MAP_SIDE; ++j)
            mvMask_temp[i][j] = mvMask[i][j];

    for (i = 0 + 1; i < nSectorsY - 1; i++)
    {
        for (j = 0 + 1; j < nSectorsX - 1; j++)
        {
            if (
                (mvMask[i - 1][j] == 0) && (mvMask[i + 1][j] == 0) && (mvMask[i][j - 1] == 0) && (mvMask[i][j + 1] == 0))
                mvMask_temp[i][j] = 0;
        }
    }

    //dilate
    ErodeDilate(useSquareElement, MORPH_OP_DILATE, mvMask_temp, mvMask);

    DetectConnectedAreas2(mvMask, areaGridMarked[BUFFER_CURR(currFrameBuffer)]);
    ProcessConnectedAreas(areaGridMarked[BUFFER_CURR(currFrameBuffer)], areaBuffer[BUFFER_CURR(currFrameBuffer)]);
    //TrackAreas();
}

static const char kernelCross[3][3] = {
    {0, 1, 0},
    {1, 1, 1},
    {0, 1, 0}};
static const char kernelSquare[3][3] = {
    {1, 1, 1},
    {1, 1, 1},
    {1, 1, 1}};
static const int kernelSize = 3;

void MoveDetector::ErodeDilate(int useSquareKernel, int operation, int (*inputArray)[MAX_MAP_SIDE], int (*outputArray)[MAX_MAP_SIDE])
{
    //probably wont ever use kernel sizes larger than 3, so this is alright
    char convKernel[kernelSize][kernelSize];
    if (useSquareKernel)
    {
        for (int i = 0; i < kernelSize; ++i)
            for (int j = 0; j < kernelSize; ++j)
                convKernel[i][j] = kernelSquare[i][j];
    }
    else
    {
        for (int i = 0; i < kernelSize; ++i)
            for (int j = 0; j < kernelSize; ++j)
                convKernel[i][j] = kernelCross[i][j];
    }

    //the following is, however, reusable
    int halfOffset = kernelSize / 2;
    int doOperation = 0;
    int u, v;
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
                    if ((inputArray[i + u - halfOffset][j + v - halfOffset] == operation) && (convKernel[u][v]))
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

void MoveDetector::DetectConnectedAreas(int (*inputArray)[MAX_MAP_SIDE], int (*outputArray)[MAX_MAP_SIDE])
{

    int i, j, u, v;

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
                                if (outputArray[u][v] == outputArray[i][j - 1])
                                {
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

void MoveDetector::DetectConnectedAreas2(int (*inputArray)[MAX_MAP_SIDE], int (*outputArray)[MAX_MAP_SIDE])
{
    int i, j, u, v;

    for (j = 0; j < MAX_MAP_SIDE; ++j)
        for (i = 0; i < MAX_MAP_SIDE; ++i)
        {
            outputArray[i][j] = 0;
        }

    std::stack<coordinate> blocks;
    coordinate top;
    int currentLabel = 1;

    for (i = 0; i < nSectorsY; i++)
    {
        for (j = 0; j < nSectorsX; j++)
        {
            if (!outputArray[i][j] && inputArray[i][j])
            {
                blocks.push({j, i});
                while (!blocks.empty())
                {
                    top = blocks.top();
                    blocks.pop();

                    if ((top.x >= 0 && top.y >= 0) &&
                        (top.x < nSectorsX && top.y < nSectorsY) &&
                        (!outputArray[top.y][top.x]) &&
                        (inputArray[top.y][top.x]))
                    {
                        outputArray[top.y][top.x] = currentLabel;
                        blocks.push({top.x + 1, top.y});
                        blocks.push({top.x - 1, top.y});
                        blocks.push({top.x, top.y + 1});
                        blocks.push({top.x, top.y - 1});
                    }
                }
                currentLabel++;
            }
        }
    }
}

void MoveDetector::ProcessConnectedAreas(int (*markedAreas)[MAX_MAP_SIDE], connectedArea (&processedAreas)[MAX_CONNAREAS])
{
    int i, j ,u;

    int areaCounter = 0;
    int currentArea = 0;
    connectedArea newArea;

    // coordinateF normVector;
    float length;

    for (i = 0; i < MAX_CONNAREAS; i++)
    {
        processedAreas[i] = {};
    }

    //enumerate all connected areas in this frame
    for (i = 0; i < nSectorsY; i++)
    {
        for (j = 0; j < nSectorsX; j++)
        {
            if (markedAreas[i][j])
            {
                //search the list of areas to see if this one is added or not                
                currentArea = markedAreas[i][j];
                //potential bottleneck:
                connectedArea *findresult =
                    std::find_if(begin(processedAreas), end(processedAreas),
                                 [&currentArea](const connectedArea &x) { return x.areaID == currentArea; });
                //if this one is not in the list, add it
                if (findresult == end(processedAreas))
                {
                    newArea = {};
                    newArea.areaID = currentArea;
                    newArea.size = 1;
                    newArea.directionX = mvGridCoords[BUFFER_CURR(currFrameBuffer)][i][j].x;
                    newArea.directionY = mvGridCoords[BUFFER_CURR(currFrameBuffer)][i][j].y;
                    // newArea.directionXVar = -1;
                    // newArea.directionYVar = -1;
                    newArea.centroidX = j;
                    newArea.centroidY = i;
                    newArea.boundBoxU = {j, i};
                    newArea.boundBoxB = {j, i};
                    newArea.isTracked = false;
                    newArea.appearances = 1;
                    // newArea.uniformity = 0;
                    // newArea.normV = {};
                    // newArea.delta = {};
                    // newArea.delta2 = {};
                    // newArea.M2 = {};
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

                    findresult->directionX = (mvGridCoords[BUFFER_CURR(currFrameBuffer)][i][j].x + findresult->size * findresult->directionX) / (findresult->size + 1);
                    findresult->directionY = (mvGridCoords[BUFFER_CURR(currFrameBuffer)][i][j].y + findresult->size * findresult->directionY) / (findresult->size + 1);

                    findresult->size++;

                    //normalize vectors to be able to estimate angular variance from x,y variances
                    //TODO: alot happening here, probably could be optimized
                    length = sqrt(mvGridCoords[BUFFER_CURR(currFrameBuffer)][i][j].x * mvGridCoords[BUFFER_CURR(currFrameBuffer)][i][j].x + mvGridCoords[BUFFER_CURR(currFrameBuffer)][i][j].y * mvGridCoords[BUFFER_CURR(currFrameBuffer)][i][j].y);
                    // normVector.x = (length) ? mvGridCoords[BUFFER_CURR(currFrameBuffer)][i][j].x / length : 0;
                    // normVector.y = (length) ? mvGridCoords[BUFFER_CURR(currFrameBuffer)][i][j].y / length : 0;

                    //accumulative mean and variance (Welford's algorithm on wiki)
                    // findresult->delta.x = normVector.x - findresult->normV.x;
                    // findresult->normV.x += findresult->delta.x / findresult->size;
                    // findresult->delta2.x = normVector.x - findresult->normV.x;
                    // findresult->M2.x = findresult->M2.x + findresult->delta2.x * findresult->delta.x;
                    // findresult->directionXVar = findresult->M2.x / (findresult->size - 1);

                    // findresult->delta.y = normVector.y - findresult->normV.y;
                    // findresult->normV.y += findresult->delta.y / findresult->size;
                    // findresult->delta2.y = normVector.y - findresult->normV.y;
                    // findresult->M2.y = findresult->M2.y + findresult->delta2.y * findresult->delta.y;
                    // findresult->directionYVar = findresult->M2.y / (findresult->size - 1);
                }
            }
        }
    }
    for (i = 0; i < areaCounter; i++)
    {
        processedAreas[i].directionAng =
            atan2f((float)processedAreas[i].directionY,
                   (float)processedAreas[i].directionX) *
                (float)180 / (float)M_PI +
            (float)180;
        processedAreas[i].directionMag =
            sqrt(processedAreas[i].directionX * processedAreas[i].directionX +
                 processedAreas[i].directionY * processedAreas[i].directionY);
        processedAreas[i].id = rand() % 30000 + 1;
        processedAreas[i].centroidX *= output_block_size;
        processedAreas[i].centroidY *= output_block_size;
        processedAreas[i].boundBoxB.x *= output_block_size;
        processedAreas[i].boundBoxB.y *= output_block_size;
        processedAreas[i].boundBoxU.x *= output_block_size;
        processedAreas[i].boundBoxU.y *= output_block_size;
        // processedAreas[i].uniformity = (processedAreas[i].directionXVar + processedAreas[i].directionYVar) * 100;
    }
}

void MoveDetector::TrackAreas()
{
    int i = 0;
    //step 0: predict motion for existing trackers
    for (auto &tracker : trackedObjects)
    {
        if (tracker.objStatus == TRACKERSTATUS_LOST)
        {
            tracker.center.x += tracker.direction.x;
            tracker.center.y += tracker.direction.y;
        }
        else
        {
            tracker.direction.x = tracker.candidatePos.x - tracker.center.x;
            tracker.direction.y = tracker.candidatePos.y - tracker.center.y;
            tracker.center = tracker.candidatePos;
            tracker.boundBoxU = tracker.candidateArea->boundBoxU;
            tracker.boundBoxB = tracker.candidateArea->boundBoxB;
            tracker.areaID = tracker.candidateAreaID;
            tracker.id = tracker.candidateId;
        }
        tracker.candidatePos = {};
        tracker.candidateAreaID = 0;
        tracker.candidateId = 0;
        tracker.iou = 0;
        tracker.lifeTime--;
    }
    //also create new trackers for detected areas
    connectedArea *currentAreas = areaBuffer[BUFFER_PREV(currFrameBuffer)];
    connectedArea *nextAreas = areaBuffer[BUFFER_CURR(currFrameBuffer)];

    while (currentAreas[i].id > 0)
    {
        if (!currentAreas[i].isTracked)
            trackedObjects.push_back(trackedObject(currentAreas[i]));
        i++;
    }

    auto detectedGridCurr = areaGridMarked[BUFFER_PREV(currFrameBuffer)];
    auto detectedGridNext = areaGridMarked[BUFFER_CURR(currFrameBuffer)];
    
    //step 1: find good matches for every tracker-area pair based on IoU
    for (auto tracker = trackedObjects.begin(); tracker != trackedObjects.end(); )
    {
        i = 0;
        while (nextAreas[i].id > 0)
        {           
            // if ((abs(tracker->predictedPos.x - nextAreas[i].centroidX) > 150) || (abs(tracker->predictedPos.y - nextAreas[i].centroidY) > 150))
            // {
            //     i++;
            //     continue;
            // }

            //mark working area
            coordinate boundU = {min(tracker->boundBoxU.x + tracker->direction.x, nextAreas[i].boundBoxU.x), min(tracker->boundBoxU.y + tracker->direction.y, nextAreas[i].boundBoxU.y)};
            coordinate boundB = {max(tracker->boundBoxB.x + tracker->direction.x, nextAreas[i].boundBoxB.x), max(tracker->boundBoxB.y + tracker->direction.x, nextAreas[i].boundBoxB.y)};
            ValidateCoordinate(boundU);
            ValidateCoordinate(boundB);

            //calculate IoU
            int _intersection = 0;
            int _union = 0;
            for (int v = boundU.y / output_block_size; v < boundB.y / output_block_size; v++)
            {
                for (int u = boundU.x / output_block_size; u < boundB.x / output_block_size; u++)
                {
                    if (detectedGridCurr[v + tracker->direction.y / output_block_size][u + tracker->direction.x / output_block_size] == tracker->areaID || detectedGridNext[v][u] == nextAreas[i].areaID)
                    {
                        _union++;
                        if (detectedGridCurr[v + tracker->direction.y / output_block_size][u + tracker->direction.x / output_block_size] == tracker->areaID && detectedGridNext[v][u] == nextAreas[i].areaID)
                            _intersection++;
                    }
                }
            }
            float iou = (float)_intersection / (float)_union;
            //find area with best iou
            if (iou > tracker->iou)
            {
                tracker->iou = iou;
                tracker->candidatePos.x = nextAreas[i].centroidX;
                tracker->candidatePos.y = nextAreas[i].centroidY;
                tracker->candidateAreaID = nextAreas[i].areaID;
                tracker->candidateId = nextAreas[i].id;
                tracker->candidateArea = &nextAreas[i];
            }
            i++;
        }

        //if tracker has found a good area in next frame and is alive, update it
        if (tracker->iou > 0.5)
        {
            tracker->lifeTime = 3;
            tracker->candidateArea->isTracked = true;
            if (tracker->objStatus == TRACKERSTATUS_INTOFRAME)
                tracker->objStatus = TRACKERSTATUS_TRACKING;
            if (tracker->objStatus == TRACKERSTATUS_NONE)
                tracker->objStatus = TRACKERSTATUS_INTOFRAME;
            ++tracker;
        }
        else
        {
            // if (tracker->lifeTime > 0)
            // {
            //     tracker->objStatus = TRACKERSTATUS_LOST;
            //     ++tracker;
            // }
            // else
                tracker = trackedObjects.erase(tracker);
        }
    }
    }

void inline MoveDetector::ValidateCoordinate(coordinate c)
{
    c.x = c.x > 0 ? c.x : 0;
    c.y = c.y > 0 ? c.y : 0;
    c.x = c.x < input_width ? c.x : input_width;
    c.y = c.y < input_height ? c.y : input_height;
}

/*void MoveDetector::TrackAreas()
{
    int i = 0, j = 0, u = 0, v = 0, w = 0;
    //int sizeTolerance = 1024 / output_block_size; //blocks
    //const float directionAngTolerance = 45;
    int searchWindow = 32; //px
    int areasCounter = 0;
    const int frameBorderThreshold = 4 * output_block_size; //px

    connectedArea *currentAreas = areaBuffer[BUFFER_CURR(currFrameBuffer)];
    connectedArea *prevAreas = areaBuffer[BUFFER_PREV(currFrameBuffer)];
    connectedArea *oldAreas;

    connectedArea *currArea, *prevArea;
    //for each area in current frame
    while (currentAreas[i].id > 0)
    {
        j = 0;
        u = 0;
        currArea = &currentAreas[i];

        if (
            (currArea->boundBoxU.x < frameBorderThreshold) ||
            (currArea->boundBoxU.x > input_width - frameBorderThreshold) ||
            (currArea->boundBoxU.y < frameBorderThreshold) ||
            (currArea->boundBoxU.y > input_height - frameBorderThreshold) ||
            (currArea->boundBoxB.x < frameBorderThreshold) ||
            (currArea->boundBoxB.x > input_width - frameBorderThreshold) ||
            (currArea->boundBoxB.y < frameBorderThreshold) ||
            (currArea->boundBoxB.y > input_height - frameBorderThreshold))
        {
            currArea->areaStatus = AREASTATUS_OUTOFFRAME;
        }

        //iterate over areas in previous frame
        while (prevAreas[j].id > 0)
        {
            prevArea = &prevAreas[j];

            if (currArea->areaStatus == AREASTATUS_OUTOFFRAME)
            {
                coordinate newBoxU = {
                    prevArea->boundBoxU.x - (int)currArea->directionX - searchWindow,
                    prevArea->boundBoxU.y - (int)currArea->directionY - searchWindow};
                coordinate newBoxB = {
                    prevArea->boundBoxB.x - (int)currArea->directionX + searchWindow,
                    prevArea->boundBoxB.y - (int)currArea->directionY + searchWindow};

                if (
                    (currArea->centroidX > newBoxU.x) && (currArea->centroidX < newBoxB.x) &&
                    (currArea->centroidY > newBoxU.y) && (currArea->centroidY < newBoxB.y) &&
                    (currArea->size > sizeThreshold) &&
                    (prevArea->isUsed == false))
                {
                    currArea->id = prevArea->id;
                    // currArea->isTracked = true;                    
                    prevArea->isUsed = true;
                    break;
                }
            }
            else if (
                (abs(prevArea->centroidX - currArea->directionX - currArea->centroidX) < searchWindow) &&
                (abs(prevArea->centroidY - currArea->directionY - currArea->centroidY) < searchWindow) &&
                (abs(prevArea->size - currArea->size) < sizeTolerance) &&
                (currArea->size > sizeThreshold) &&
                (prevArea->isUsed == false))
            {
                currArea->id = prevArea->id;
                // currArea->isTracked = true;
                prevArea->isUsed = true;
                break;
            }
            j++;
        }
        i++;
        areasCounter++;
    }
}*/

/* void MoveDetector::TrackedAreasFiltering()
{
    int i = 0, j = 0;
    connectedArea *detectedAreas = areaBuffer[BUFFER_OLDEST(currFrameBuffer)];
    connectedArea *currentBuffer;
    vector<connectedArea *> futureAreas;

    //continuity area filtering
    while (detectedAreas[i].id > 0)
    {
        futureAreas.clear();
        for (int frameBufferIndex = 0; frameBufferIndex < AREABUFFER_SIZE - 3; frameBufferIndex++)
        {
            int bufferid = (currFrameBuffer + 2 + frameBufferIndex) % AREABUFFER_SIZE;
            currentBuffer = areaBuffer[bufferid];
            j = 0;
            while (currentBuffer[j].id > 0)
            {
                if (currentBuffer[j].id == detectedAreas[i].id)
                {
                    futureAreas.push_back(&currentBuffer[j]);
                    break;
                }
                j++;
            }
        }
        detectedAreas[i].appearances = futureAreas.size();
        if (futureAreas.size() > 2)
        {
            detectedAreas[i].isTracked = true;
            for (int u = 0; u < futureAreas.size(); u++)
            {
                futureAreas[u]->isTracked = true;
            }
            int thisID = detectedAreas[i].id;
            auto existingObject = std::find_if(trackedObjects.begin(), trackedObjects.end(), [&thisID](const trackedObject &x) { return x.id == thisID; });
            if (existingObject == trackedObjects.end())
            {
                trackedObject newObj(detectedAreas[i]);
                newObj.framesToLive = futureAreas.size() + 3;
                trackedObjects.push_back(newObj);
            }
            else
            {
                existingObject->UpdateFromArea(detectedAreas[i]);
                existingObject->framesToLive = futureAreas.size() + 3;
            }
        }
        i++;
    }

    //processing tracked objects

    //decrementing TTLs
    auto it = trackedObjects.begin();
    while (it != trackedObjects.end())
    {
        it->framesToLive--;
        if (it->framesToLive == 0)
            it = trackedObjects.erase(it);
        else
            ++it;
    }
} */

/*
void MoveDetector::SpatialConsistProcess()
{
    int i, j, u, v, regionsN = 0;

    for (j = 0; j < MAX_MAP_SIDE; ++j)
        for (i = 0; i < MAX_MAP_SIDE; ++i)
        {
            areaGridMarked[i][j] = 0;
        }

    struct spRegion
    {
        int id;
        int size;
        coordinate blocks[120 * 120];
    };

    spRegion *regions = new spRegion[MAX_CONNAREAS];

    // spRegion regions[MAX_CONNAREAS];
    spRegion currRegion;

    float blockSimilarity[4];
    for (i = 0; i < 3; i++)
        blockSimilarity[i] = 0;
    float currMetric = 0.0;

    for (i = 1; i < nSectorsY - 1; i++)
    {
        for (j = 1; j < nSectorsX - 1; j++)
        {
            //if current block doesnt belong to an object
            if (areaGridMarked[i][j] == 0)
            {
                //test this seed
                blockSimilarity[0] = abs(mvGridCoords[BUFFER_CURR(currFrameBuffer)][i][j - 1].y - mvGridCoords[BUFFER_CURR(currFrameBuffer)][i][j].y);
                blockSimilarity[1] = abs(mvGridCoords[BUFFER_CURR(currFrameBuffer)][i + 1][j].x - mvGridCoords[BUFFER_CURR(currFrameBuffer)][i][j].x);
                blockSimilarity[2] = abs(mvGridCoords[BUFFER_CURR(currFrameBuffer)][i][j + 1].y - mvGridCoords[BUFFER_CURR(currFrameBuffer)][i][j].y);
                blockSimilarity[3] = abs(mvGridCoords[BUFFER_CURR(currFrameBuffer)][i - 1][j].x - mvGridCoords[BUFFER_CURR(currFrameBuffer)][i][j].x);
                if (blockSimilarity[0] < CONSIST_THRESHOLD &&
                    blockSimilarity[1] < CONSIST_THRESHOLD &&
                    blockSimilarity[2] < CONSIST_THRESHOLD &&
                    blockSimilarity[3] < CONSIST_THRESHOLD)
                {
                    //currRegion = {regionsN + 1, 5};
                    currRegion = {};
                    currRegion.id = regionsN + 1;
                    currRegion.size = 5;
                    currRegion.blocks[0] = {i, j};
                    currRegion.blocks[1] = {i - 1, j};
                    currRegion.blocks[2] = {i + 1, j};
                    currRegion.blocks[3] = {i, j - 1};
                    currRegion.blocks[4] = {i, j + 1};
                    areaGridMarked[i][j] = regionsN;
                }
                //if this isnt a good seed, go to next block
                else
                {
                    continue;
                }

                //start growing region around this object
                stack<coordinate> toFill;
                toFill.push({i - 2, j});
                toFill.push({i + 2, j});
                toFill.push({i, j - 2});
                toFill.push({i, j + 2});
                toFill.push({i - 1, j - 1});
                toFill.push({i + 1, j - 1});
                toFill.push({i - 1, j + 1});
                toFill.push({i + 1, j + 1});

                coordinate top;
                while (!toFill.empty())
                {
                    top = toFill.top();
                    toFill.pop();
                    if ((top.x >= 0 && top.y >= 0) &&
                        (top.x < nSectorsX && top.y < nSectorsY))
                    {
                        if (areaGridMarked[top.x][top.y] == 0)
                        {
                            currMetric = 0.0;
                            for (u = 0; u < currRegion.size; u++)
                            {
                                coordinate thisBlock = currRegion.blocks[u];
                                float t = abs(
                                    (top.x - thisBlock.x) * (mvGridCoords[BUFFER_CURR(currFrameBuffer)][top.x][top.y].x - mvGridCoords[BUFFER_CURR(currFrameBuffer)][thisBlock.x][thisBlock.y].x) +
                                    (top.y - thisBlock.y) * (mvGridCoords[BUFFER_CURR(currFrameBuffer)][top.x][top.y].y - mvGridCoords[BUFFER_CURR(currFrameBuffer)][thisBlock.x][thisBlock.y].y));
                                currMetric += t;
                            }
                            currMetric /= currRegion.size;
                            if (currMetric < CONSIST_THRESHOLD)
                            {
                                currRegion.blocks[u] = {top.x, top.y};
                                currRegion.size++;
                                areaGridMarked[top.x][top.y] = currRegion.id;
                                toFill.push({top.x + 1, top.y});
                                toFill.push({top.x - 1, top.y});
                                toFill.push({top.x, top.y + 1});
                                toFill.push({top.x, top.y - 1});
                            }
                        }
                    }
                }
                regions[regionsN] = currRegion;
                regionsN++;
            }
        }
    }
}
*/

void MoveDetector::TemporalConsistProcess()
{
    int i, j;
    for (i = 0; i < MAX_MAP_SIDE; i++)
        for (j = 0; j < MAX_MAP_SIDE; j++)
        {
            bwProjected[i][j] = {};
            fwProjected[i][j] = {};
        }
    ProjectMVectors(mvGridCoords[BUFFER_NEXT(currFrameBuffer)], bwProjected, MV_PROJECT_BACKWARDS);
    ProjectMVectors(mvGridCoords[BUFFER_PREV(currFrameBuffer)], fwProjected, MV_PROJECT_BACKWARDS);
    CalculateSimilarity(mvGridCoords[BUFFER_CURR(currFrameBuffer)], bwProjected, similarityBW);
    CalculateSimilarity(mvGridCoords[BUFFER_CURR(currFrameBuffer)], fwProjected, similarityFW);
    CalculateSimilarity(bwProjected, fwProjected, similarityBWFW);
    DetectForeground();
    SpatialFilter(areaFgMarked);
}

void MoveDetector::ProjectMVectors(coordinate mVectors[][MAX_MAP_SIDE], coordinateF projectedOut[][MAX_MAP_SIDE], int projectionDir)
{
    int i, j, xOffset, yOffset;

    //multiplier to help with comparing fields
    const float weightFactor = 4.0f;
    coordinate currMV;
    coordinateF *A, *B, *C, *D;
    int *cA, *cB, *cC, *cD;
    float aA, aB, aC, aD;

    int mvCount[MAX_MAP_SIDE][MAX_MAP_SIDE];
    coordinateF projected[MAX_MAP_SIDE][MAX_MAP_SIDE];

    for (i = 0; i < nSectorsY; i++)
        for (j = 0; j < nSectorsX; j++)
        {
            mvCount[i][j] = 0;
            projected[i][j] = {};
        }

    for (i = 0; i < nSectorsY; i++)
    {
        for (j = 0; j < nSectorsX; j++)
        {
            currMV.x = mVectors[i][j].x * projectionDir;
            currMV.y = mVectors[i][j].y * projectionDir;

            //no MV here
            if (!currMV.x && !currMV.y)
                continue;

            //MV points outside this frame (should consider this case maybe?)
            if (i * 16 + currMV.y < 0 || i * 16 + currMV.y > 16 * nSectorsY ||
                j * 16 + currMV.x < 0 || j * 16 + currMV.x > 16 * nSectorsX)
                continue;

            xOffset = (currMV.x % 16 + 16) % 16;
            yOffset = (currMV.y % 16 + 16) % 16;

            //if 4 cells are affected
            if (xOffset && yOffset)
            {
                A = &(projected[(i * 16 + currMV.y) / 16][(j * 16 + currMV.x) / 16]);
                cA = &(mvCount[(i * 16 + currMV.y) / 16][(j * 16 + currMV.x) / 16]);

                B = &(projected[(i * 16 + currMV.y) / 16][(j * 16 + currMV.x) / 16 + 1]);
                cB = &(mvCount[(i * 16 + currMV.y) / 16][(j * 16 + currMV.x) / 16 + 1]);

                C = &(projected[(i * 16 + currMV.y) / 16 + 1][(j * 16 + currMV.x) / 16]);
                cC = &(mvCount[(i * 16 + currMV.y) / 16 + 1][(j * 16 + currMV.x) / 16]);

                D = &(projected[(i * 16 + currMV.y) / 16 + 1][(j * 16 + currMV.x) / 16 + 1]);
                cD = &(mvCount[(i * 16 + currMV.y) / 16 + 1][(j * 16 + currMV.x) / 16 + 1]);

                aA = (16 - xOffset) * (16 - yOffset) / (float)256;
                aB = (xOffset) * (16 - yOffset) / (float)256;
                aC = (16 - xOffset) * (yOffset) / (float)256;
                aD = (xOffset) * (yOffset) / (float)256;

                A->x += currMV.x * aA * weightFactor;
                A->y += currMV.y * aA * weightFactor;

                B->x += currMV.x * aB * weightFactor;
                B->y += currMV.y * aB * weightFactor;

                C->x += currMV.x * aC * weightFactor;
                C->y += currMV.y * aC * weightFactor;

                D->x += currMV.x * aD * weightFactor;
                D->y += currMV.y * aD * weightFactor;

                (*cA)++;
                (*cB)++;
                (*cC)++;
                (*cD)++;
            }
            //2 cells affected (L/R)
            else if (!yOffset && xOffset)
            {
                A = &(projected[(i * 16 + currMV.y) / 16][(j * 16 + currMV.x) / 16]);
                cA = &(mvCount[(i * 16 + currMV.y) / 16][(j * 16 + currMV.x) / 16]);

                B = &(projected[(i * 16 + currMV.y) / 16][(j * 16 + currMV.x) / 16 + 1]);
                cB = &(mvCount[(i * 16 + currMV.y) / 16][(j * 16 + currMV.x) / 16 + 1]);

                aA = (16 - xOffset) * (16) / (float)256;
                aB = (xOffset) * (16) / (float)256;

                A->x += currMV.x * aA * weightFactor;
                A->y += currMV.y * aA * weightFactor;

                B->x += currMV.x * aB * weightFactor;
                B->y += currMV.y * aB * weightFactor;

                (*cA)++;
                (*cB)++;
            }
            //2 cells affected (T/B)
            else if (!xOffset && yOffset)
            {
                A = &(projected[(i * 16 + currMV.y) / 16][(j * 16 + currMV.x) / 16]);
                cA = &(mvCount[(i * 16 + currMV.y) / 16][(j * 16 + currMV.x) / 16]);

                C = &(projected[(i * 16 + currMV.y) / 16 + 1][(j * 16 + currMV.x) / 16]);
                cC = &(mvCount[(i * 16 + currMV.y) / 16 + 1][(j * 16 + currMV.x) / 16]);

                aA = (16) * (16 - yOffset) / (float)256;
                aC = (16) * (yOffset) / (float)256;

                A->x += currMV.x * aA * weightFactor;
                A->y += currMV.y * aA * weightFactor;

                C->x += currMV.x * aC * weightFactor;
                C->y += currMV.y * aC * weightFactor;

                (*cA)++;
                (*cC)++;
            }
            //MV points to another block exactly
            else
            {
                A = &(projected[(i * 16 + currMV.y) / 16][(j * 16 + currMV.x) / 16]);
                cA = &(mvCount[(i * 16 + currMV.y) / 16][(j * 16 + currMV.x) / 16]);

                aA = 1.0;

                A->x += currMV.x * aA * weightFactor;
                A->y += currMV.y * aA * weightFactor;

                (*cA)++;
            }
        }
    }
    for (i = 0; i < nSectorsY; i++)
        for (j = 0; j < nSectorsX; j++)
        {
            projectedOut[i][j].x = mvCount[i][j] > 0 ? projected[i][j].x / (float)mvCount[i][j] : 0;
            projectedOut[i][j].y = mvCount[i][j] > 0 ? projected[i][j].y / (float)mvCount[i][j] : 0;
        }
}

void MoveDetector::CalculateSimilarity(coordinate currentMV[][MAX_MAP_SIDE], coordinateF projectedMV[][MAX_MAP_SIDE], float metricOut[][MAX_MAP_SIDE])
{
    int i, j;
    float absdiff = 0.0;
    float abscurr = 0.0;
    float absproj = 0.0;
    coordinate *currMV;
    coordinateF *projMV;

    for (i = 0; i < nSectorsY; i++)
        for (j = 0; j < nSectorsX; j++)
        {
            metricOut[i][j] = 0.0;
        }

    for (i = 0; i < nSectorsY; i++)
    {
        for (j = 0; j < nSectorsX; j++)
        {
            currMV = &currentMV[i][j];
            projMV = &projectedMV[i][j];

            abscurr = sqrt(currMV->x * currMV->x + currMV->y * currMV->y);
            absproj = sqrt(projMV->x * projMV->x + projMV->y * projMV->y);
            absdiff = (currMV->x - projMV->x) * (currMV->x - projMV->x) + (currMV->y - projMV->y) * (currMV->y - projMV->y);

            metricOut[i][j] = (abscurr + absproj) ? exp(-1 * (absdiff) / ((abscurr + absproj) * (abscurr + absproj))) : 1.0;
        }
    }
}

void MoveDetector::CalculateSimilarity(coordinateF currentMV[][MAX_MAP_SIDE], coordinateF projectedMV[][MAX_MAP_SIDE], float metricOut[][MAX_MAP_SIDE])
{
    int i, j;
    float absdiff = 0.0;
    float abscurr = 0.0;
    float absproj = 0.0;
    coordinateF *currMV;
    coordinateF *projMV;

    for (i = 0; i < nSectorsY; i++)
        for (j = 0; j < nSectorsX; j++)
        {
            metricOut[i][j] = 0.0;
        }

    for (i = 0; i < nSectorsY; i++)
    {
        for (j = 0; j < nSectorsX; j++)
        {
            currMV = &currentMV[i][j];
            projMV = &projectedMV[i][j];

            abscurr = sqrt(currMV->x * currMV->x + currMV->y * currMV->y);
            absproj = sqrt(projMV->x * projMV->x + projMV->y * projMV->y);
            absdiff = (currMV->x - projMV->x) * (currMV->x - projMV->x) + (currMV->y - projMV->y) * (currMV->y - projMV->y);

            metricOut[i][j] = (abscurr + absproj) ? exp(-1 * (absdiff) / ((abscurr + absproj) * (abscurr + absproj))) : 1.0;
        }
    }
}

void MoveDetector::DetectForeground()
{
    //const float alpha = 0.7, beta = 4;
    int i, j;
    for (i = 0; i < MAX_MAP_SIDE; i++)
        for (j = 0; j < MAX_MAP_SIDE; j++)
        {
            areaFgMarked[i][j] = 0;
        }

    for (i = 0; i < nSectorsY; i++)
    {
        for (j = 0; j < nSectorsX; j++)
        {
            if ((similarityFW[i][j] > alpha) && (similarityBW[i][j] > alpha))
            {
                areaFgMarked[i][j] = mvGridMag[i][j] > beta ? 1 : -1;
            }
            else if ((similarityFW[i][j] > alpha) || (similarityBW[i][j] > alpha))
            {
                float max = std::max(similarityBW[i][j], similarityFW[i][j]);
                areaFgMarked[i][j] = mvGridMag[i][j] * max * max > beta ? 2 : -2;
            }
            else if (similarityBWFW[i][j] > alpha)
            {
                float absFW = sqrt(fwProjected[i][j].x * fwProjected[i][j].x + fwProjected[i][j].y * fwProjected[i][j].y);
                areaFgMarked[i][j] = similarityBWFW[i][j] * similarityBWFW[i][j] * absFW > beta ? 3 : -3;
            }
        }
    }
}

void MoveDetector::SpatialFilter(int marked[][MAX_MAP_SIDE])
{
    int i, j, u;

    int marked_tmp[MAX_MAP_SIDE][MAX_MAP_SIDE];
    for (i = 0; i < nSectorsY; i++)
    {
        for (j = 0; j < nSectorsX; j++)
        {
            marked_tmp[i][j] = marked[i][j];
        }
    }

    //0 - close to BG, 1 - closer to FG
    float score = 0.0f;
    for (i = 0; i < nSectorsY; i++)
    {
        for (j = 0; j < nSectorsX; j++)
        {
            //if this sector is unmarked
            if (!marked_tmp[i][j])
            {
                score = 0.0f;
                //search downwards
                for (u = i; (u < nSectorsY && u < i + 3); u++)
                {
                    //found a marked sector
                    if (marked_tmp[u][j])
                    {
                        score += marked_tmp[u][j] > 0 ? 1.0f / u : -1.0f / u;
                        break;
                    }
                }

                //search upwards
                for (u = i; (u >= 0 && u > i - 3); u--)
                {
                    //found a marked sector
                    if (marked_tmp[u][j])
                    {
                        score += marked_tmp[u][j] > 0 ? 1.0f / u : -1.0f / u;
                        break;
                    }
                }

                //search to the right
                for (u = j; (u < nSectorsX && u < j + 3); u++)
                {
                    //found a marked sector
                    if (marked_tmp[i][u])
                    {
                        score += marked_tmp[i][u] > 0 ? 1.0f / u : -1.0f / u;
                        break;
                    }
                }

                //search to the left
                for (u = j; (u >= 0 && u > j - 3); u--)
                {
                    //found a marked sector
                    if (marked_tmp[i][u])
                    {
                        score += marked_tmp[i][u] > 0 ? 1.0f / u : -1.0f / u;
                        break;
                    }
                }

                score /= 4.0f;

                marked[i][j] = score > 0.0f ? 4 : -4;
            }
        }
    }
}