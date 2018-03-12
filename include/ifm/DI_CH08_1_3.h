/* This file provides the definition of the struct AlgoIFOutput in the interface version DIA1_1.3.
   
   It also provides endianess-aware functions to safely convert a binary buffer to an 
   instance of this struct (if possible).
*/

#ifndef IFM_O3M_ALGOIFOUTPUT_DIA1_1_3_CONVERTER_H_INCLUDED
#define IFM_O3M_ALGOIFOUTPUT_DIA1_1_3_CONVERTER_H_INCLUDED

#include "ifm_types.h"

/* The struct has explicit padding, so that it should be usable on any target 
   without special compiler flags or pragmas related to padding.
*/
typedef struct
{
    /*
      Magic number of project, can be used to identify the output type.
    */
    ifm_o3m_sint8_t magic_no[4];
    /*
      Magic number of struct, can be used to identify the output type.
    */
    ifm_o3m_sint8_t struct_id[4];
    /*
      Version number of struct.
    */
    ifm_o3m_uint8_t version[2];
    ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
    ifm_o3m_uint8_t pad_002; /* explicit padding, do not access this member */
    struct
    {
        /*
          Sensor width included for completeness.
          
          Unit: [px]
          Range: [64,64]
        */
        ifm_o3m_uint16_t sensorWidth;
        /*
          Sensor height included for completeness.
          
          Unit: [px]
          Range: [16,16]
        */
        ifm_o3m_uint16_t sensorHeight;
        /*
          Radial distance measurements relative to the camera's optical center.         
                  
          Note: Pixels with invalid distance measurements are assigned a value of 1cm. This allows to derive
          the pixel's 3D direction vector from the Cartesian coordinates (X,Y,Z) even for invalid pixels.
          
          Unit: [cm]
          Range: [0,15000]
        */
        ifm_o3m_uint16_t distanceData[1024];
        /*
          Matrix of cartesian X coordinates, one for each pixel.         
                  
          Note: The 3D direction vector (dvx_i, dvy_i, dvz_i) associated with a given pixel index i can be
          obtained by
          dvx_i = DistanceImageResult::X[i] - DistanceImageResult::cameraCalibration.transX
          dvy_i = DistanceImageResult::Y[i] - DistanceImageResult::cameraCalibration.transY
          dvz_i = DistanceImageResult::Z[i] - DistanceImageResult::cameraCalibration.transZ
          
          Unit: [m]
          Range: [-150.0,150.0]
        */
        ifm_o3m_float32_t X[1024];
        /*
          Matrix of cartesian Y coordinates, one for each pixel.         
                  
          Note: The 3D direction vector (dvx_i, dvy_i, dvz_i) associated with a given pixel index i can be
          obtained by
          dvx_i = DistanceImageResult::X[i] - DistanceImageResult::cameraCalibration.transX
          dvy_i = DistanceImageResult::Y[i] - DistanceImageResult::cameraCalibration.transY
          dvz_i = DistanceImageResult::Z[i] - DistanceImageResult::cameraCalibration.transZ
          
          Unit: [m]
          Range: [-150.0,150.0]
        */
        ifm_o3m_float32_t Y[1024];
        /*
          Matrix of cartesian Z coordinates, one for each pixel.         
                  
          Note: The 3D direction vector (dvx_i, dvy_i, dvz_i) associated with a given pixel index i can be
          obtained by
          dvx_i = DistanceImageResult::X[i] - DistanceImageResult::cameraCalibration.transX
          dvy_i = DistanceImageResult::Y[i] - DistanceImageResult::cameraCalibration.transY
          dvz_i = DistanceImageResult::Z[i] - DistanceImageResult::cameraCalibration.transZ
          
          Unit: [m]
          Range: [-150.0,150.0]
        */
        ifm_o3m_float32_t Z[1024];
        /*
          Matrix of confidence values, one for each pixel.         
                  
          Each confidence value c is a bitmask composed of the following bits:
          (c & 1 ) != 0: pixel is regarded invalid
          (c & 2 ) != 0: long exposure used for distance/amplitude calculataion
          (c & 4 ) != 0: high gain channel used for distance/amplitude calculation
          (c & 8 ) != 0: internal
          (c & 16 ) != 0: itnernal
          (c & 32 ) != 0: pixel has been clipped due to spatial filtering
          (c & 64 ) != 0: pixel is classified as belonging to the ground (and there is a ground estimation)
          (c & 128 ) != 0: pixel is classified as belonging to the ground (with default ground Z=0)
          (c & 256 ) != 0: pixel is classified as spray/dust pixel
          (c & 512 ) != 0: internal
          (c & 1024) != 0: internal
          (c & 2048) != 0: internal
          (c & 4096) != 0: pixel has been invalidated due to low signal to noise ratio
          
          Unit: [N/A]
          Range: [0,65535]
        */
        ifm_o3m_uint16_t confidence[1024];
        /*
          Matrix of raw amplitude values, one for each pixel.         
                  
          To calculate a normalized amplitude value for pixel with index i, use the following
          formula:ampl_norm = amplitude[i] * amplitude_normalization[(confidence[i] >> 1) & 3]
          
          Unit: [N/A]
          Range: [0,65535]
        */
        ifm_o3m_uint16_t amplitude[1024];
        /*
          Amplitude normalization factors.         
                  
          
          [0]: factor for short exposure, low gain
          [1]: factor for long exposure, low gain
          [2]: factor for short exposure, high gain
          [3]: factor for long exposure, high gain
          
          Unit: [N/A]
          Range: [0,1000.f]
        */
        ifm_o3m_float32_t amplitude_normalization[4];
        /*
          Timestamp of the resulting data in masterclock domain.
          
          Unit: [1e-6 s]
          Range: [0,MAX_UINT32]
        */
        ifm_o3m_uint32_t masterclockTimestamp;
        /*
          Rolling frame counter.
          
          Unit: [N/A]
          Range: [0,MAX_UINT32]
        */
        ifm_o3m_uint32_t frameCounter;
        /*
          availability bitmask         
                  
          
          available == 0 : available
          (available & 1 ) != 0: interference detected
          (available & 2 ) != 0: spray detected
          (available & 4 ) != 0: tracking error (e.g. necessary vehicle data is not available)
          (available & 8 ) != 0: invalid camera orientation
          (available & 16 ) != 0: lvds cable length not detected
          (available & 32 ) != 0: internal algo error
          (available & 64 ) != 0: blockage detected
          (available & 128) != 0: calibration in progress
          
          Unit: [enum]
          Range: [0,255]
        */
        ifm_o3m_uint32_t available;
        struct
        {
            /*
              X component of the camera's translation vector expressed in world coordinates.         
                      
              If the standard vehicle coordinate system is used, this value is the distance from rear axle to the
              camera's reference point parallel to the vehicle's driving direction. Positive values indicate that
              the camera is in front of the rear axle.
              
              Unit: [m]
              Range: [-30,30]
            */
            ifm_o3m_float32_t transX;
            /*
              Y component of the camera's translation vector expressed in world coordinates.         
                      
              If the standard vehicle coordinate system is used, this value is the distance from the vehicle's
              longitudinal center plane to the camera's reference point. Positive values indicate that the camera
              is located at the left side of the vehicle.
              
              Unit: [m]
              Range: [-30,30]
            */
            ifm_o3m_float32_t transY;
            /*
              Z component of the camera's translation vector expressed in world coordinates.         
                      
              If the standard vehicle coordinate system is used, this value is the distance from the ground plane
              to the camera's reference point. Positive values indicate that the camera is above the ground.
              
              Unit: [m]
              Range: [-30,30]
            */
            ifm_o3m_float32_t transZ;
            /*
              Camera rotation around the X axis.         
                      
              The camera orientation is expressed in so-called Euler angles.
              
              Unit: [rad]
              Range: [-pi,pi]
            */
            ifm_o3m_float32_t rotX;
            /*
              Camera rotation around the Y axis.         
                      
              The camera orientation is expressed in so-called Euler angles.
              
              Unit: [rad]
              Range: [-pi,pi]
            */
            ifm_o3m_float32_t rotY;
            /*
              Camera rotation around the Z axis.         
                      
              The camera orientation is expressed in so-called Euler angles.
              
              Unit: [rad]
              Range: [-pi,pi]
            */
            ifm_o3m_float32_t rotZ;
        } cameraCalibration;
        struct
        {
            /*
              Field of view, given as unit vectors in the world coordinate system.         
                      
              This is the vector (x,y,z) corresponding to the upper left corner.
            */
            ifm_o3m_float32_t upperLeft[3];
            /*
              Field of view, given as unit vectors in the world coordinate system.         
                      
              This is the vector (x,y,z) corresponding to the upper right corner.
            */
            ifm_o3m_float32_t upperRight[3];
            /*
              Field of view, given as unit vectors in the world coordinate system.         
                      
              This is the vector (x,y,z) corresponding to the lower left corner.
            */
            ifm_o3m_float32_t lowerLeft[3];
            /*
              Field of view, given as unit vectors in the world coordinate system.         
                      
              This is the vector (x,y,z) corresponding to the lower right corner.
            */
            ifm_o3m_float32_t lowerRight[3];
        } fieldOfView;
        struct
        {
            /*
              Focal length x direction.         
                      
              Negative in case of mirroring enabled for x direction. Zero in case of no 2D sensor present.
              
              Unit: [px]
              Range: [-MAX_FLOAT32,+MAX_FLOAT32]
            */
            ifm_o3m_float32_t intrCalib_2D_fx;
            /*
              Focal length y direction.         
                      
              Negative in case of mirroring enabled for y direction. Zero in case of no 2D sensor present.
              
              Unit: [px]
              Range: [-MAX_FLOAT32,+MAX_FLOAT32]
            */
            ifm_o3m_float32_t intrCalib_2D_fy;
            /*
              Main point in x.
              
              Unit: [px]
              Range: [-320,+320]
            */
            ifm_o3m_float32_t intrCalib_2D_mx;
            /*
              Main point in y.
              
              Unit: [px]
              Range: [-240,+240]
            */
            ifm_o3m_float32_t intrCalib_2D_my;
            /*
              skew parameter
              
              Unit: [N/A]
              Range: [-1,+1]
            */
            ifm_o3m_float32_t intrCalib_alpha;
            /*
              First radial distortion parameter.
              
              Unit: [N/A]
              Range: [-10,10]
            */
            ifm_o3m_float32_t intrCalib_k1;
            /*
              Second radial distortion parameter.
              
              Unit: [N/A]
              Range: [-10,10]
            */
            ifm_o3m_float32_t intrCalib_k2;
            /*
              Third radial distortion parameter.
              
              Unit: [N/A]
              Range: [-10,10]
            */
            ifm_o3m_float32_t intrCalib_k5;
            /*
              First tangential distortion parameter.
              
              Unit: [N/A]
              Range: [-10,10]
            */
            ifm_o3m_float32_t intrCalib_k3;
            /*
              Second tangential distortion parameter.
              
              Unit: [N/A]
              Range: [-10,10]
            */
            ifm_o3m_float32_t intrCalib_k4;
            /*
              x coordinate of center point for extrinsic calibration
              
              Unit: [N/A]
              Range: [-10,10]
            */
            ifm_o3m_float32_t extrCalib_center_tx;
            /*
              y coordinate of center point for extrinsic calibration
              
              Unit: [N/A]
              Range: [-10,10]
            */
            ifm_o3m_float32_t extrCalib_center_ty;
            /*
              z coordinate of center point for extrinsic calibration
              
              Unit: [N/A]
              Range: [-10,10]
            */
            ifm_o3m_float32_t extrCalib_center_tz;
            /*
              x coordinate of offset to ceneter point of extrinsic calibration
              
              Unit: [N/A]
              Range: [-10,10]
            */
            ifm_o3m_float32_t extrCalib_delta_tx;
            /*
              y coordinate of offset to ceneter point of extrinsic calibration
              
              Unit: [N/A]
              Range: [-10,10]
            */
            ifm_o3m_float32_t extrCalib_delta_ty;
            /*
              z coordinate of offset to ceneter point of extrinsic calibration
              
              Unit: [N/A]
              Range: [-10,10]
            */
            ifm_o3m_float32_t extrCalib_delta_tz;
            /*
              x rotation
              
              Unit: [rad]
              Range: [-pi,pi]
            */
            ifm_o3m_float32_t extrCalib_rot_x;
            /*
              y rotation
              
              Unit: [rad]
              Range: [-pi,pi]
            */
            ifm_o3m_float32_t extrCalib_rot_y;
            /*
              z rotation
              
              Unit: [rad]
              Range: [-pi,pi]
            */
            ifm_o3m_float32_t extrCalib_rot_z;
        } intrExtrCalib_2d;
        /*
          Absolute position of illumination in world coordinates [X,Y,Z].
          
          Unit: [m]
          Range: [-100,100]
        */
        ifm_o3m_float32_t illuPosition[3];
        /*
          Blockage ratio (for transmission over CAN)
          
          Unit: [N/A]
          Range: [0,1]
        */
        ifm_o3m_float32_t blockageRatio;
        /*
          Flag indicating if the blockage is available for this sample.
          
          Unit: [bool]
          Range: [0,1]
        */
        ifm_o3m_uint8_t blockageAvailable;
        ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_002; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_003; /* explicit padding, do not access this member */
    } distanceImageResult;
    struct
    {
        struct
        {
            /*
              valid flag for calibration
            */
            ifm_o3m_sint32_t calibValid;
            /*
              Stable calibration counter, can be used as an acceptance criterium.
            */
            ifm_o3m_sint32_t calibrationStableCounter;
            struct
            {
                /*
                  X component of the camera's translation vector expressed in world coordinates.         
                          
                  If the standard vehicle coordinate system is used, this value is the distance from rear axle to the
                  camera's reference point parallel to the vehicle's driving direction. Positive values indicate that
                  the camera is in front of the rear axle.
                  
                  Unit: [m]
                  Range: [-30,30]
                */
                ifm_o3m_float32_t transX;
                /*
                  Y component of the camera's translation vector expressed in world coordinates.         
                          
                  If the standard vehicle coordinate system is used, this value is the distance from the vehicle's
                  longitudinal center plane to the camera's reference point. Positive values indicate that the camera
                  is located at the left side of the vehicle.
                  
                  Unit: [m]
                  Range: [-30,30]
                */
                ifm_o3m_float32_t transY;
                /*
                  Z component of the camera's translation vector expressed in world coordinates.         
                          
                  If the standard vehicle coordinate system is used, this value is the distance from the ground plane
                  to the camera's reference point. Positive values indicate that the camera is above the ground.
                  
                  Unit: [m]
                  Range: [-30,30]
                */
                ifm_o3m_float32_t transZ;
                /*
                  Camera rotation around the X axis.         
                          
                  The camera orientation is expressed in so-called Euler angles.
                  
                  Unit: [rad]
                  Range: [-pi,pi]
                */
                ifm_o3m_float32_t rotX;
                /*
                  Camera rotation around the Y axis.         
                          
                  The camera orientation is expressed in so-called Euler angles.
                  
                  Unit: [rad]
                  Range: [-pi,pi]
                */
                ifm_o3m_float32_t rotY;
                /*
                  Camera rotation around the Z axis.         
                          
                  The camera orientation is expressed in so-called Euler angles.
                  
                  Unit: [rad]
                  Range: [-pi,pi]
                */
                ifm_o3m_float32_t rotZ;
            } calibResult;
        } commonCalibrationResult;
        struct
        {
            /*
              Number of triangles detected.
              
              Unit: [N/A]
              Range: [0,8]
            */
            ifm_o3m_uint8_t numTrianglesDetected;
            ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
            ifm_o3m_uint8_t pad_002; /* explicit padding, do not access this member */
            ifm_o3m_uint8_t pad_003; /* explicit padding, do not access this member */
            struct
            {
                /*
                  score of triangle detection (-1: inverse match, 1: perfect match)
                  
                  Unit: [N/A]
                  Range: [-1,1]
                */
                ifm_o3m_float32_t score;
                /*
                  Cartesian 3D position of triangle center (x, y, z)
                */
                ifm_o3m_float32_t pos3D[3];
                /*
                  Corner 2D positions of triangle corners (for image overlay).         
                          
                  The first index represents the corner point index in the triangle. The second index represents the
                  x (=0) or y (=1) coordinate on the 2D image plane.
                */
                ifm_o3m_float32_t corners[3][2];
            } triangleDetections[8];
            /*
              Boolean indicating a valid Pattern Autocalibration frame.
            */
            ifm_o3m_sint32_t frameValid;
            /*
              Mean calibration reprojection error per frame.
              
              Unit: [m]
              Range: [0, 0.25]
            */
            ifm_o3m_float32_t frameReprojectError;
        } pacCalibrationResult;
        struct
        {
            /*
              Set to 1 if street estimation is considered valid and may be used for further processing.
            */
            ifm_o3m_sint32_t planeValid;
            struct
            {
                /*
                  Pitch angle of the camera in [rad].
                */
                ifm_o3m_float32_t pitchAngle;
                /*
                  Roll angle of the camera in [rad].
                */
                ifm_o3m_float32_t rollAngle;
                /*
                  height of the camera in [m].
                */
                ifm_o3m_float32_t camHeight;
                /*
                  normal vector of the currently estimated road plane (x component)         
                          
                  This is derived directly from pitchAngle and rollAngle and is provided for convenience.
                */
                ifm_o3m_float32_t normalx;
                /*
                  normal vector of the currently estimated road plane (y component)         
                          
                  This is derived directly from pitchAngle and rollAngle and is provided for convenience.
                */
                ifm_o3m_float32_t normaly;
                /*
                  normal vector of the currently estimated road plane (z component)         
                          
                  This is derived directly from pitchAngle and rollAngle and is provided for convenience.
                */
                ifm_o3m_float32_t normalz;
            } planeEstimation;
            /*
              Plausibility [0,1] for the estimated plane parameters.
            */
            ifm_o3m_float32_t plausibility;
            /*
              Standard deviation of distance to plane of all pixels, which are assigned as street pixels.
            */
            ifm_o3m_float32_t distanceDeviation;
        } streetCalibrationResult;
    } calibrationResult;
    struct
    {
        /*
          the digitial outputs of the logic graph         
                  
          This output should be assigned with PVO nodes addressing ChannelID 2, expression
          "AlgoState.algoLogicState.ifOutput.digitalOutput[]". Using this pattern, it is possible to treat
          the digital outputs as a kind of external memory for the logic graph.
        */
        ifm_o3m_uint8_t digitalOutput[100];
        /*
          the analog outputs of the logic graph         
                  
          This output should be assigned with PVO nodes addressing ChannelID 2, expression
          "AlgoState.algoLogicState.ifOutput.analogOuptut[]". Using this pattern, it is possible to treat the
          analog outputs as a kind of external memory for the logic graph.
        */
        ifm_o3m_float32_t analogOutput[20];
    } logicOutput;
} ifm_o3m_AlgoIFOutput_DIA1_1_3;

        
/* Casts the buffer to ifm_o3m_AlgoIFOutput_DIA1_1_3 (if possible) and returns a pointer to it.
   Use this function on little Endian systems.

   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutput_DIA1_1_3* ifm_o3m_ConvertBufferToLittleEndian_DIA1_1_3(void *buffer, ifm_o3m_uint32_t bufferSize);

/* Converts the endianess of the buffer to native form and returns a pointer to ifm_o3m_AlgoIFOutput_DIA1_1_3. 
   Note: The original buffer is modified in place. 
   Use this function on big Endian systems.
   
   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutput_DIA1_1_3* ifm_o3m_ConvertBufferToBigEndian_DIA1_1_3(void *buffer, ifm_o3m_uint32_t bufferSize);

#endif
