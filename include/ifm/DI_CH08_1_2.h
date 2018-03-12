/* This file provides the definition of the struct AlgoIFOutput in the interface version DIA1_1.2.
   
   It also provides endianess-aware functions to safely convert a binary buffer to an 
   instance of this struct (if possible).
*/

#ifndef IFM_O3M_ALGOIFOUTPUT_DIA1_1_2_CONVERTER_H_INCLUDED
#define IFM_O3M_ALGOIFOUTPUT_DIA1_1_2_CONVERTER_H_INCLUDED

#include "ifm_types.h"

/* For a documentation of the struct members refer to your reference manual.
   The struct has explicit padding, so that it should be usable on any target 
   without special compiler flags or pragmas related to padding.
*/
typedef struct
{
    ifm_o3m_sint8_t magic_no[4];
    ifm_o3m_sint8_t struct_id[4];
    ifm_o3m_uint8_t version[2];
    ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
    ifm_o3m_uint8_t pad_002; /* explicit padding, do not access this member */
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
        struct
        {
            ifm_o3m_float32_t intrCalib_2D_fx;
            ifm_o3m_float32_t intrCalib_2D_fy;
            ifm_o3m_float32_t intrCalib_2D_mx;
            ifm_o3m_float32_t intrCalib_2D_my;
            ifm_o3m_float32_t intrCalib_alpha;
            ifm_o3m_float32_t intrCalib_k1;
            ifm_o3m_float32_t intrCalib_k2;
            ifm_o3m_float32_t intrCalib_k5;
            ifm_o3m_float32_t intrCalib_k3;
            ifm_o3m_float32_t intrCalib_k4;
            ifm_o3m_float32_t extrCalib_center_tx;
            ifm_o3m_float32_t extrCalib_center_ty;
            ifm_o3m_float32_t extrCalib_center_tz;
            ifm_o3m_float32_t extrCalib_delta_tx;
            ifm_o3m_float32_t extrCalib_delta_ty;
            ifm_o3m_float32_t extrCalib_delta_tz;
            ifm_o3m_float32_t extrCalib_rot_x;
            ifm_o3m_float32_t extrCalib_rot_y;
            ifm_o3m_float32_t extrCalib_rot_z;
        } intrExtrCalib_2d;
        ifm_o3m_float32_t illuPosition[3];
        ifm_o3m_float32_t blockageRatio;
        ifm_o3m_uint8_t blockageAvailable;
        ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_002; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_003; /* explicit padding, do not access this member */
    } distanceImageResult;
    struct
    {
        struct
        {
            ifm_o3m_sint32_t calibValid;
            ifm_o3m_sint32_t calibrationStableCounter;
            struct
            {
                ifm_o3m_float32_t transX;
                ifm_o3m_float32_t transY;
                ifm_o3m_float32_t transZ;
                ifm_o3m_float32_t rotX;
                ifm_o3m_float32_t rotY;
                ifm_o3m_float32_t rotZ;
            } calibResult;
        } commonCalibrationResult;
        struct
        {
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
        } pacCalibrationResult;
        struct
        {
            ifm_o3m_sint32_t planeValid;
            struct
            {
                ifm_o3m_float32_t pitchAngle;
                ifm_o3m_float32_t rollAngle;
                ifm_o3m_float32_t camHeight;
                ifm_o3m_float32_t normalx;
                ifm_o3m_float32_t normaly;
                ifm_o3m_float32_t normalz;
            } planeEstimation;
            ifm_o3m_float32_t plausibility;
            ifm_o3m_float32_t distanceDeviation;
        } streetCalibrationResult;
    } calibrationResult;
} ifm_o3m_AlgoIFOutput_DIA1_1_2;

        
/* Casts the buffer to ifm_o3m_AlgoIFOutput_DIA1_1_2 (if possible) and returns a pointer to it.
   Use this function on little Endian systems.

   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutput_DIA1_1_2* ifm_o3m_ConvertBufferToLittleEndian_DIA1_1_2(void *buffer, ifm_o3m_uint32_t bufferSize);

/* Converts the endianess of the buffer to native form and returns a pointer to ifm_o3m_AlgoIFOutput_DIA1_1_2. 
   Note: The original buffer is modified in place. 
   Use this function on big Endian systems.
   
   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutput_DIA1_1_2* ifm_o3m_ConvertBufferToBigEndian_DIA1_1_2(void *buffer, ifm_o3m_uint32_t bufferSize);

#endif
