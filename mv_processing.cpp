#include <stack>
#include <algorithm>

#include "motion_watch.h"

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
    ProcessConnectedAreas(areaGridMarked, areaBuffer[currFrameBuffer]);


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

    coordinateF normVector;
    float length;

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
                    newArea.directionXVar = -1;
                    newArea.directionYVar = -1;
                    newArea.centroidX = j;
                    newArea.centroidY = i;
                    newArea.boundBoxU = {j, i};
                    newArea.boundBoxB = {j, i};
                    newArea.uniformity = 0;
                    newArea.normV = {};
                    newArea.delta = {};
                    newArea.delta2 = {};
                    newArea.M2 = {};
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

                    findresult->directionX = (mvGridCoords[i][j].x + findresult->size * findresult->centroidX) / (findresult->size + 1);
                    findresult->directionY = (mvGridCoords[i][j].y + findresult->size * findresult->centroidY) / (findresult->size + 1);                    

                    findresult->size++;

                    //normalize vectors to be able to estimate angular variance from x,y variances
                    //TODO: alot happening here, probably could be optimized
                    length = sqrt(mvGridCoords[i][j].x * mvGridCoords[i][j].x + mvGridCoords[i][j].y * mvGridCoords[i][j].y);
                    normVector.x = (length) ? mvGridCoords[i][j].x / length : 0;
                    normVector.y = (length) ? mvGridCoords[i][j].y / length : 0;

                    //accumulative mean and variance (Welford's algorithm on wiki)
                    findresult->delta.x = normVector.x - findresult->normV.x;
                    findresult->normV.x += findresult->delta.x / findresult->size;
                    findresult->delta2.x = normVector.x - findresult->normV.x;
                    findresult->M2.x = findresult->M2.x + findresult->delta2.x * findresult->delta.x;
                    findresult->directionXVar = findresult->M2.x / (findresult->size - 1);

                    findresult->delta.y = normVector.y - findresult->normV.y;
                    findresult->normV.y += findresult->delta.y / findresult->size;
                    findresult->delta2.y = normVector.y - findresult->normV.y;
                    findresult->M2.y = findresult->M2.y + findresult->delta2.y * findresult->delta.y;
                    findresult->directionYVar = findresult->M2.y / (findresult->size - 1);
                }
            }
        }
    }

    for (i = 0; i < areaCounter; i++)
    {
        processedAreas[i].directionAng =
            atan2f((float)processedAreas[i].directionY,
                   (float)processedAreas[i].directionX) * (float)180 / (float) M_PI + (float)180;
        processedAreas[i].directionMag =
            sqrt(processedAreas[i].directionX * processedAreas[i].directionX +
                 processedAreas[i].directionY * processedAreas[i].directionY);
        processedAreas[i].uniformity = (processedAreas[i].directionXVar + processedAreas[i].directionYVar) * 100;
    }
}