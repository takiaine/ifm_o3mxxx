/* This file provides the definition of the struct AlgoIFOutput in the interface version DIA1_1.0.
   
   It also provides endianess-aware functions to safely convert a binary buffer to an 
   instance of this struct (if possible).
*/

#ifndef IFM_O3M_ALGOIFOUTPUT_DIA1_1_0_CONVERTER_H_INCLUDED
#define IFM_O3M_ALGOIFOUTPUT_DIA1_1_0_CONVERTER_H_INCLUDED

#include "ifm_types.h"

/* For a documentation of the struct members refer to your reference manual.
   The struct has explicit padding, so that it should be usable on any target 
   without special compiler flags or pragmas related to padding.
*/
typedef struct
{
    struct
    {
        ifm_o3m_uint16_t sensorWidth;
        ifm_o3m_uint16_t sensorHeight;
        ifm_o3m_uint16_t distanceData[1024];
        ifm_o3m_float32_t X[1024];
        ifm_o3m_float32_t Y[1024];
        ifm_o3m_float32_t Z[1024];
        ifm_o3m_uint16_t confidence[1024];
        ifm_o3m_uint16_t amplitude[1024];
        ifm_o3m_float32_t amplitude_normalization[4];
        ifm_o3m_uint32_t masterclockTimestamp;
        ifm_o3m_uint32_t frameCounter;
        ifm_o3m_uint32_t available;
        struct
        {
            ifm_o3m_float32_t transX;
            ifm_o3m_float32_t transY;
            ifm_o3m_float32_t transZ;
            ifm_o3m_float32_t rotX;
            ifm_o3m_float32_t rotY;
            ifm_o3m_float32_t rotZ;
        } cameraCalibration;
        struct
        {
            ifm_o3m_float32_t upperLeft[3];
            ifm_o3m_float32_t upperRight[3];
            ifm_o3m_float32_t lowerLeft[3];
            ifm_o3m_float32_t lowerRight[3];
        } fieldOfView;
    } distanceImageResult;
    struct
    {
        ifm_o3m_sint32_t pacValid;
        struct
        {
            ifm_o3m_float32_t transX;
            ifm_o3m_float32_t transY;
            ifm_o3m_float32_t transZ;
            ifm_o3m_float32_t rotX;
            ifm_o3m_float32_t rotY;
            ifm_o3m_float32_t rotZ;
        } pacResult;
        ifm_o3m_uint8_t numTrianglesDetected;
        ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_002; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_003; /* explicit padding, do not access this member */
        struct
        {
            ifm_o3m_float32_t score;
            ifm_o3m_float32_t pos3D[3];
            ifm_o3m_float32_t corners[3][2];
        } triangleDetections[8];
        ifm_o3m_sint32_t frameValid;
        ifm_o3m_float32_t frameReprojectError;
        ifm_o3m_sint32_t calibrationStableCounter;
    } calibrationResult;
} ifm_o3m_AlgoIFOutput_DIA1_1_0;

        
/* Casts the buffer to ifm_o3m_AlgoIFOutput_DIA1_1_0 (if possible) and returns a pointer to it.
   Use this function on little Endian systems.

   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutput_DIA1_1_0* ifm_o3m_ConvertBufferToLittleEndian_DIA1_1_0(void *buffer, ifm_o3m_uint32_t bufferSize);

/* Converts the endianess of the buffer to native form and returns a pointer to ifm_o3m_AlgoIFOutput_DIA1_1_0. 
   Note: The original buffer is modified in place. 
   Use this function on big Endian systems.
   
   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutput_DIA1_1_0* ifm_o3m_ConvertBufferToBigEndian_DIA1_1_0(void *buffer, ifm_o3m_uint32_t bufferSize);

#endif
