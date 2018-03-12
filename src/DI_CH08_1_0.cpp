#include "DI_CH08_1_0.h"

/* Macros are used for performing endianess corrections; these might be replaced with compiler- or machine-dependent equivalents */
#define IFM_SWAP16(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[1]; (b)[1] = tmp; }
#define IFM_SWAP32(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[3]; (b)[3] = tmp; tmp = (b)[1]; (b)[1] = (b)[2]; (b)[2] = tmp; }
#define IFM_SWAP64(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[7]; (b)[7] = tmp; tmp = (b)[1]; (b)[1] = (b)[6]; (b)[6] = tmp; tmp = (b)[2]; (b)[2] = (b)[5]; (b)[5] = tmp; tmp = (b)[3]; (b)[3] = (b)[4]; (b)[4] = tmp; }

/* Casts the buffer to ifm_o3m_AlgoIFOutput_DIA1_1_0 (if possible) and returns a pointer to it.
   Use this function on little Endian systems.

   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutput_DIA1_1_0* ifm_o3m_ConvertBufferToLittleEndian_DIA1_1_0(void *buffer, ifm_o3m_uint32_t bufferSize)
{
    ifm_o3m_AlgoIFOutput_DIA1_1_0* res = (ifm_o3m_AlgoIFOutput_DIA1_1_0*)buffer;
    if( (!buffer) || (bufferSize != 18900) || (sizeof(ifm_o3m_AlgoIFOutput_DIA1_1_0) != 18900) )
    {
        return 0;
    }

    return res;
}

/* Converts the endianess of the buffer to native form and returns a pointer to ifm_o3m_AlgoIFOutput_DIA1_1_0. 
   Note: The original buffer is modified in place. 
   Use this function on big Endian systems.
   
   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutput_DIA1_1_0* ifm_o3m_ConvertBufferToBigEndian_DIA1_1_0(void *buffer, ifm_o3m_uint32_t bufferSize)
{
    ifm_o3m_uint32_t i;
    ifm_o3m_uint8_t *buf = (ifm_o3m_uint8_t *)buffer;
    ifm_o3m_AlgoIFOutput_DIA1_1_0* res = (ifm_o3m_AlgoIFOutput_DIA1_1_0*)buffer;
    if( (!buffer) || (bufferSize != 18900) || (sizeof(ifm_o3m_AlgoIFOutput_DIA1_1_0) != 18900) )
    {
        return 0;
    }

    /* distanceImageResult.sensorWidth */
    IFM_SWAP16(&buf[0]);
    /* distanceImageResult.sensorHeight */
    IFM_SWAP16(&buf[2]);
    /* distanceImageResult.distanceData */
    for(i = 0; i < 1024; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4]);
    }
    /* distanceImageResult.X */
    for(i = 0; i < 1024; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2052]);
    }
    /* distanceImageResult.Y */
    for(i = 0; i < 1024; i++)
    {
        IFM_SWAP32(&buf[(i*4)+6148]);
    }
    /* distanceImageResult.Z */
    for(i = 0; i < 1024; i++)
    {
        IFM_SWAP32(&buf[(i*4)+10244]);
    }
    /* distanceImageResult.confidence */
    for(i = 0; i < 1024; i++)
    {
        IFM_SWAP16(&buf[(i*2)+14340]);
    }
    /* distanceImageResult.amplitude */
    for(i = 0; i < 1024; i++)
    {
        IFM_SWAP16(&buf[(i*2)+16388]);
    }
    /* distanceImageResult.amplitude_normalization */
    for(i = 0; i < 4; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18436]);
    }
    /* distanceImageResult.masterclockTimestamp */
    IFM_SWAP32(&buf[18452]);
    /* distanceImageResult.frameCounter */
    IFM_SWAP32(&buf[18456]);
    /* distanceImageResult.available */
    IFM_SWAP32(&buf[18460]);
    /* distanceImageResult.cameraCalibration.transX */
    IFM_SWAP32(&buf[18464]);
    /* distanceImageResult.cameraCalibration.transY */
    IFM_SWAP32(&buf[18468]);
    /* distanceImageResult.cameraCalibration.transZ */
    IFM_SWAP32(&buf[18472]);
    /* distanceImageResult.cameraCalibration.rotX */
    IFM_SWAP32(&buf[18476]);
    /* distanceImageResult.cameraCalibration.rotY */
    IFM_SWAP32(&buf[18480]);
    /* distanceImageResult.cameraCalibration.rotZ */
    IFM_SWAP32(&buf[18484]);
    /* distanceImageResult.fieldOfView.upperLeft */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18488]);
    }
    /* distanceImageResult.fieldOfView.upperRight */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18500]);
    }
    /* distanceImageResult.fieldOfView.lowerLeft */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18512]);
    }
    /* distanceImageResult.fieldOfView.lowerRight */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18524]);
    }
    /* calibrationResult.pacValid */
    IFM_SWAP32(&buf[18536]);
    /* calibrationResult.pacResult.transX */
    IFM_SWAP32(&buf[18540]);
    /* calibrationResult.pacResult.transY */
    IFM_SWAP32(&buf[18544]);
    /* calibrationResult.pacResult.transZ */
    IFM_SWAP32(&buf[18548]);
    /* calibrationResult.pacResult.rotX */
    IFM_SWAP32(&buf[18552]);
    /* calibrationResult.pacResult.rotY */
    IFM_SWAP32(&buf[18556]);
    /* calibrationResult.pacResult.rotZ */
    IFM_SWAP32(&buf[18560]);
    /* calibrationResult.triangleDetections.score */
    for(i = 0; i < 8; i++)
    {
        IFM_SWAP32(&buf[(i*40)+18568]);
    }
    /* calibrationResult.triangleDetections.pos3D */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18572]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18612]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18652]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18692]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18732]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18772]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18812]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18852]);
    }
    /* calibrationResult.triangleDetections.corners */
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18584]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18624]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18664]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18704]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18744]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18784]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18824]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18864]);
    }
    /* calibrationResult.frameValid */
    IFM_SWAP32(&buf[18888]);
    /* calibrationResult.frameReprojectError */
    IFM_SWAP32(&buf[18892]);
    /* calibrationResult.calibrationStableCounter */
    IFM_SWAP32(&buf[18896]);

    return res;
}
