/* This IFM data receiver is done based on the IFM example codes.
 * Done by Antti Kolu, antti.kolu@tut.fi, anttikolu@gmail.com
*/

#include "ros/ros.h"
#include "ifm_o3mxxx/udpreceiver.h"
#include "ifm/example_results.h"
#include "ifm/example_types.h"
#include "ifm_o3mxxx/IFMO3mxxx.h"

#include "DI_CH08_1_0.h"
#include "DI_CH08_1_1.h"
#include "DI_CH08_1_2.h"
#include "DI_CH08_1_3.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>


using std::string;

// include all versions of distance image channel 14
//#include "DI_CH14_1_3.h"

// include all versions of distance image channel 256
//#include "DI_CH256_1_4.h"

#define NUM_SENSOR_PIXELS (1024)


// We copy the received data into this structures.
//
//struct distanceData_t
//{
//    uint16_t distanceData[NUM_SENSOR_PIXELS];
//    uint16_t confidence[NUM_SENSOR_PIXELS];
//    float x[NUM_SENSOR_PIXELS];
//    float y[NUM_SENSOR_PIXELS];
//    float z[NUM_SENSOR_PIXELS];
//    uint16_t width;
//    uint16_t height;
//}__attribute__((packed));

ifm_o3mxxx::IFMO3mxxx ifm_ros_msg;

//static distanceData_t distanceData;

#define UDP_PACKET_BUF_LEN (2000)

const uint16_t confidence1 = 1;
const uint16_t confidence2 = 2;
const uint16_t confidence4 = 4;
const uint16_t confidence8 = 8;
const uint16_t confidence16 = 16;
const uint16_t confidence32 = 32;
const uint16_t confidence64 = 64;
const uint16_t confidence128 = 128;
const uint16_t confidence256 = 256;
const uint16_t confidence512 = 512;
const uint16_t confidence1024 = 1024;
const uint16_t confidence2048 = 2048;
const uint16_t confidence4096 = 4096;


struct PacketHeader
{
    uint16_t Version;
    uint16_t Device;
    uint32_t PacketCounter;
    uint32_t CycleCounter;
    uint16_t NumberOfPacketsInCycle;
    uint16_t IndexOfPacketInCycle;
    uint16_t NumberOfPacketsInChannel;
    uint16_t IndexOfPacketInChannel;
    uint32_t ChannelID;
    uint32_t TotalLengthOfChannel;
    uint32_t LengthPayload;

}__attribute__((packed));


struct ChannelHeader
{
    uint32_t StartDelimiter;
    uint8_t reserved[24];

}__attribute__((packed));

struct ChannelEnd
{
    uint32_t EndDelimiter;

}__attribute__((packed));

// Extracts the data in the payload of the udp packet und puts it into the channel buffer

int processPacket( int8_t* currentPacketData,  // payload of the udp packet (without ethernet/IP/UDP header)
                    int currentPacketSize, // size of the udp packet payload
                    int8_t* channelBuffer,      // buffer for a complete channel
                    uint32_t channelBufferSize, // size of the buffer for the complete channel
                    uint32_t* pos)              // the current pos in the channel buffer
{

    // There is always a PacketHeader structure at the beginning
    PacketHeader* ph = (PacketHeader*)currentPacketData;
    int Start = sizeof(PacketHeader);
    int Length = currentPacketSize - sizeof(PacketHeader);

    // Only the first packet of a channel contains a ChannelHeader
    if (ph->IndexOfPacketInChannel == 0)
    {
        Start += sizeof(ChannelHeader);
        Length -= sizeof(ChannelHeader);
    }

    // Only the last packet of a channel contains an EndDelimiter (at the end, after the data)
    if (ph->IndexOfPacketInChannel == ph->NumberOfPacketsInChannel - 1)
    {
        Length -= sizeof(ChannelEnd);
    }

    // Is the buffer big enough?
    if ((*pos) + Length > channelBufferSize)
    {
        // Too small means either an error in the program logic or a corrupt packet
        printf("Channel buffer is too small.\n");
        return RESULT_ERROR;
    }
    else
    {
        memcpy(channelBuffer+(*pos), currentPacketData+Start, Length);
    }

    (*pos) += Length;

    return RESULT_OK;

}


// For every version of the data structure there needs to be a copy function.
// The content of the function is always the same, only the type of p will be different each time.
// In C++ this could be solved with a template.

// copy the data for DI structure version 1.0 into our internal structure
int copyChannel8_DIA1_1_0(ifm_o3m_AlgoIFOutput_DIA1_1_0* p)
{

    if (p == NULL)
        return RESULT_ERROR;

    memcpy(&ifm_ros_msg.distance_data[0], p->distanceImageResult.distanceData, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.distanceData)));
    memcpy(&ifm_ros_msg.confidence[0], p->distanceImageResult.confidence, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.confidence)));
    memcpy(&ifm_ros_msg.amplitude[0], p->distanceImageResult.amplitude, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.amplitude)));
    memcpy(&ifm_ros_msg.x[0], p->distanceImageResult.X, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.X)));
    memcpy(&ifm_ros_msg.y[0], p->distanceImageResult.Y, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.Y)));
    memcpy(&ifm_ros_msg.z[0], p->distanceImageResult.Z, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.Z)));

    ifm_ros_msg.trans_x = p->distanceImageResult.cameraCalibration.transX;
    ifm_ros_msg.trans_y = p->distanceImageResult.cameraCalibration.transY;
    ifm_ros_msg.trans_z = p->distanceImageResult.cameraCalibration.transZ;
    ifm_ros_msg.rot_x = p->distanceImageResult.cameraCalibration.rotX;
    ifm_ros_msg.rot_y = p->distanceImageResult.cameraCalibration.rotY;
    ifm_ros_msg.rot_z = p->distanceImageResult.cameraCalibration.rotZ;

    ifm_ros_msg.height = p->distanceImageResult.sensorHeight;
    ifm_ros_msg.width = p->distanceImageResult.sensorWidth;
    ifm_ros_msg.available = p->distanceImageResult.available;

    memcpy(&ifm_ros_msg.amplitude_normalization[0], p->distanceImageResult.amplitude_normalization, 4*sizeof(*(p->distanceImageResult.amplitude_normalization)));


    return RESULT_OK;
}


// copy the data for DI structure version 1.1 into our internal structure
int copyChannel8_DIA1_1_1(ifm_o3m_AlgoIFOutput_DIA1_1_1* p)
{
    if (p == NULL)
        return RESULT_ERROR;

    memcpy(&ifm_ros_msg.distance_data[0], p->distanceImageResult.distanceData, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.distanceData)));
    memcpy(&ifm_ros_msg.confidence[0], p->distanceImageResult.confidence, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.confidence)));
    memcpy(&ifm_ros_msg.amplitude[0], p->distanceImageResult.amplitude, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.amplitude)));
    memcpy(&ifm_ros_msg.x[0], p->distanceImageResult.X, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.X)));
    memcpy(&ifm_ros_msg.y[0], p->distanceImageResult.Y, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.Y)));
    memcpy(&ifm_ros_msg.z[0], p->distanceImageResult.Z, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.Z)));

    ifm_ros_msg.trans_x = p->distanceImageResult.cameraCalibration.transX;
    ifm_ros_msg.trans_y = p->distanceImageResult.cameraCalibration.transY;
    ifm_ros_msg.trans_z = p->distanceImageResult.cameraCalibration.transZ;
    ifm_ros_msg.rot_x = p->distanceImageResult.cameraCalibration.rotX;
    ifm_ros_msg.rot_y = p->distanceImageResult.cameraCalibration.rotY;
    ifm_ros_msg.rot_z = p->distanceImageResult.cameraCalibration.rotZ;

    ifm_ros_msg.height = p->distanceImageResult.sensorHeight;
    ifm_ros_msg.width = p->distanceImageResult.sensorWidth;
    ifm_ros_msg.available = p->distanceImageResult.available;

    memcpy(&ifm_ros_msg.amplitude_normalization[0], p->distanceImageResult.amplitude_normalization, 4*sizeof(*(p->distanceImageResult.amplitude_normalization)));


    return RESULT_OK;
}

// copy the data for DI structure version 1.2 into our internal structure
int copyChannel8_DIA1_1_2(ifm_o3m_AlgoIFOutput_DIA1_1_2* p)
{
    if (p == NULL)
        return RESULT_ERROR;

    memcpy(&ifm_ros_msg.distance_data[0], p->distanceImageResult.distanceData, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.distanceData)));
    memcpy(&ifm_ros_msg.confidence[0], p->distanceImageResult.confidence, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.confidence)));
    memcpy(&ifm_ros_msg.amplitude[0], p->distanceImageResult.amplitude, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.amplitude)));
    memcpy(&ifm_ros_msg.x[0], p->distanceImageResult.X, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.X)));
    memcpy(&ifm_ros_msg.y[0], p->distanceImageResult.Y, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.Y)));
    memcpy(&ifm_ros_msg.z[0], p->distanceImageResult.Z, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.Z)));

    ifm_ros_msg.trans_x = p->distanceImageResult.cameraCalibration.transX;
    ifm_ros_msg.trans_y = p->distanceImageResult.cameraCalibration.transY;
    ifm_ros_msg.trans_z = p->distanceImageResult.cameraCalibration.transZ;
    ifm_ros_msg.rot_x = p->distanceImageResult.cameraCalibration.rotX;
    ifm_ros_msg.rot_y = p->distanceImageResult.cameraCalibration.rotY;
    ifm_ros_msg.rot_z = p->distanceImageResult.cameraCalibration.rotZ;

    ifm_ros_msg.height = p->distanceImageResult.sensorHeight;
    ifm_ros_msg.width = p->distanceImageResult.sensorWidth;
    ifm_ros_msg.available = p->distanceImageResult.available;

    memcpy(&ifm_ros_msg.amplitude_normalization[0], p->distanceImageResult.amplitude_normalization, 4*sizeof(*(p->distanceImageResult.amplitude_normalization)));


    return RESULT_OK;
}

// copy the data for DI structure version 1.3 into our internal structure
int copyChannel8_DIA1_1_3(ifm_o3m_AlgoIFOutput_DIA1_1_3* p)
{
    if (p == NULL)
        return RESULT_ERROR;

    memcpy(&ifm_ros_msg.distance_data[0], p->distanceImageResult.distanceData, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.distanceData)));
    memcpy(&ifm_ros_msg.confidence[0], p->distanceImageResult.confidence, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.confidence)));
    memcpy(&ifm_ros_msg.amplitude[0], p->distanceImageResult.amplitude, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.amplitude)));
    memcpy(&ifm_ros_msg.x[0], p->distanceImageResult.X, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.X)));
    memcpy(&ifm_ros_msg.y[0], p->distanceImageResult.Y, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.Y)));
    memcpy(&ifm_ros_msg.z[0], p->distanceImageResult.Z, NUM_SENSOR_PIXELS*sizeof(*(p->distanceImageResult.Z)));

    ifm_ros_msg.trans_x = p->distanceImageResult.cameraCalibration.transX;
    ifm_ros_msg.trans_y = p->distanceImageResult.cameraCalibration.transY;
    ifm_ros_msg.trans_z = p->distanceImageResult.cameraCalibration.transZ;
    ifm_ros_msg.rot_x = p->distanceImageResult.cameraCalibration.rotX;
    ifm_ros_msg.rot_y = p->distanceImageResult.cameraCalibration.rotY;
    ifm_ros_msg.rot_z = p->distanceImageResult.cameraCalibration.rotZ;

    ifm_ros_msg.height = p->distanceImageResult.sensorHeight;
    ifm_ros_msg.width = p->distanceImageResult.sensorWidth;
    ifm_ros_msg.available = p->distanceImageResult.available;

    memcpy(&ifm_ros_msg.amplitude_normalization[0], p->distanceImageResult.amplitude_normalization, 4*sizeof(*(p->distanceImageResult.amplitude_normalization)));


    return RESULT_OK;
}


// checks which version of the data it is and copies the data into our own structure.
// You need this for every channel you want to process.
int processChannel8(void* buf, uint32_t size)
{
    ifm_o3m_AlgoIFOutput_DIA1_1_0* pDIA1_1_0;
    ifm_o3m_AlgoIFOutput_DIA1_1_1* pDIA1_1_1;
    ifm_o3m_AlgoIFOutput_DIA1_1_2* pDIA1_1_2;
    ifm_o3m_AlgoIFOutput_DIA1_1_3* pDIA1_1_3;

    // Is this DI structure version 1.0?
    pDIA1_1_0 = ifm_o3m_ConvertBufferToLittleEndian_DIA1_1_0(buf, size);
    if (pDIA1_1_0)
    {
        // yes it is, so copy the data from this structure
        copyChannel8_DIA1_1_0(pDIA1_1_0);
        return RESULT_OK;
    }

    // It wasn't a version 1.0 structure, so let's check if it's a version 1.1 structure
    pDIA1_1_1 = ifm_o3m_ConvertBufferToLittleEndian_DIA1_1_1(buf, size);
    if (pDIA1_1_1)
    {
        // It's a version 1.1 structure. Copy data from this structure
        copyChannel8_DIA1_1_1(pDIA1_1_1);
        return RESULT_OK;
    }

    // It wasn't a version 1.0 structure, so let's check if it's a version 1.1 structure
    pDIA1_1_2 = ifm_o3m_ConvertBufferToLittleEndian_DIA1_1_2(buf, size);
    if (pDIA1_1_2)
    {
        // It's a version 1.1 structure. Copy data from this structure
        copyChannel8_DIA1_1_2(pDIA1_1_2);
        return RESULT_OK;
    }

    // It wasn't a version 1.0 structure, so let's check if it's a version 1.1 structure
    pDIA1_1_3 = ifm_o3m_ConvertBufferToLittleEndian_DIA1_1_3(buf, size);
    if (pDIA1_1_3)
    {
        // It's a version 1.1 structure. Copy data from this structure
        copyChannel8_DIA1_1_3(pDIA1_1_3);
        return RESULT_OK;
    }

    // For new versions you have to add the additional converts/copy hier.


    // This is no known version of the data
    printf("*** Unknown version *** \n\n");
    return RESULT_ERROR;


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "imf_o3mxxx_node");
    ros::NodeHandle node;
    ros::NodeHandle node_priv("~");


    string out_imf_depth_image = "";
    string out_imf_point_cloud = "";
    string out_imf_raw = "";
    string frame_id = "";

    int port = 4599;
    string ip = "255.255.255.255";

    node_priv.param("OutDepthImageTopic", out_imf_depth_image, std::string("ifm_o3mxxx_depth_image"));
    node_priv.param("PointCloudTopic", out_imf_point_cloud, std::string("ifm_o3mxxx_pc"));
    node_priv.param("RawTopic", out_imf_raw, std::string("ifm_o3mxxx_raw"));
    node_priv.param("Port", port, 4599);
    node_priv.param("IP", ip, std::string("255.255.255.255"));
    node_priv.param("FrameID", frame_id, std::string("ifm"));

    ros::Publisher pub_img = node.advertise<sensor_msgs::Image> (out_imf_depth_image, 1);
    ros::Publisher pub_pc = node.advertise<sensor_msgs::PointCloud2> (out_imf_point_cloud, 1);
    ros::Publisher pub_imf_raw = node.advertise<ifm_o3mxxx::IFMO3mxxx> (out_imf_raw, 1);

    UDPReceiver receiver;
    receiver.init(port, ip);

    // The data is in the channel 8
    const uint32_t customerDataChannel = 8;

    // buffer for a single UDP packet
    int8_t udpPacketBuf[UDP_PACKET_BUF_LEN];

    // As the alignment was forced to 1 we can work with the struct on the buffer.
    // This assumes the byte order is little endian which it is on a PC.
    PacketHeader* ph = (PacketHeader*)udpPacketBuf;

    // the size of the channel may change so the size will be taken from the packet
    uint32_t channel_buf_size = 0;
    int8_t* channelBuf = NULL;

    // As there is no offset in the packet header we have to remember where the next part should go
    uint32_t pos_in_channel = 0;

    // remember the counter of the previous packet so we know when we are losing packets
    uint32_t previous_packet_counter = 0;
    bool previous_packet_counter_valid = false;

    // the receiption of the data may start at any time. So we wait til we find the beginning of our channel
    bool startOfChannelFound = false;

    while (node.ok())
    {
        int ret_val = receiver.receive(udpPacketBuf, UDP_PACKET_BUF_LEN, 100000);

        if(ret_val >= 0){
            //ROS_INFO("packet received");
            // Check the packet counter for missing packets
            if (previous_packet_counter_valid)
            {
                // if the type of the variables is ui32, it will also work when the wrap around happens.
                if ((ph->PacketCounter - previous_packet_counter) != 1)
                {
                    printf("Packet Counter jumped from %i to %i (missing packets; try to redirect output)\n", previous_packet_counter, ph->PacketCounter);

                    // With this it will ignore the already received parts and resynchronize at
                    // the beginning of the next cycle.
                    startOfChannelFound = false;
                }
            }

            previous_packet_counter = ph->PacketCounter;
            previous_packet_counter_valid = true;

            // is this the channel with our data?
            //ROS_INFO("channel %i, prev_counter %i, packet_counter %i", ph->ChannelID, previous_packet_counter, ph->PacketCounter );
            if (ph->ChannelID == customerDataChannel)
            {

                // are we at the beginning of the channel?
                if (ph->IndexOfPacketInChannel == 0)
                {
                    startOfChannelFound = true;

                    // If we haven't allocated memory for the channel do it now.
                    if (channel_buf_size == 0)
                    {
                        channel_buf_size = ph->TotalLengthOfChannel;
                        channelBuf = (int8_t*) malloc(channel_buf_size);
                    }

                    // as we reuse the buffer we clear it at the beginning of a transmission
                    memset(channelBuf, 0, channel_buf_size);
                    pos_in_channel = 0;

                }

                // if we have found the start of the channel at least once, we are ready to process the packet
                if (startOfChannelFound)
                {

                    processPacket(udpPacketBuf, ret_val, channelBuf, channel_buf_size, &pos_in_channel);

                    // Have we found the last packet in this channel? Then we are able to process it
                    // The index is zero based so a channel with n parts will have indices from 0 to n-1
                    if (ph->IndexOfPacketInChannel == ph->NumberOfPacketsInChannel -1)
                    {
                        // pos_in_channel is the position where the (not existing) next packet would be
                        // placed. This is also the size of the data.
                        int ret = processChannel8((void*)channelBuf, pos_in_channel);
                        //displayData();
                        sensor_msgs::Image img;
                        img.header.stamp = ros::Time::now();
                        img.header.frame_id = frame_id;
                        img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
                        img.data.resize(NUM_SENSOR_PIXELS*4);
                        for(unsigned int i = 0 ; i < NUM_SENSOR_PIXELS; ++i){
                            //img.data.at(i) = (static_cast<float>(ifm_ros_msg.distance_data[i])/100.0);
                            //img.data.at(i*2) = static_cast<uint16_t>(ifm_ros_msg.distance_data[i]);
                            float32_t pixel = static_cast<float32_t>(ifm_ros_msg.distance_data[i])/100;
                            memcpy(&(img.data.at(i*4)), &pixel, sizeof(float32_t) );
                        }

                        img.height = ifm_ros_msg.height;
                        img.width = ifm_ros_msg.width;
                        img.step = img.width*4;

                        pub_img.publish(img);

                        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
                        sensor_msgs::PointCloud2 cloud;

                        pcl::PointXYZI point;
                        for(unsigned int i = 0 ; i < NUM_SENSOR_PIXELS; ++i){
                            if(ifm_ros_msg.confidence[i] & confidence1 || ifm_ros_msg.confidence[i] & confidence4096) // invalid pixel
                                continue;
                            point.x = ifm_ros_msg.x[i];
                            point.y = ifm_ros_msg.y[i];
                            point.z = ifm_ros_msg.z[i];
                            point.intensity = ifm_ros_msg.amplitude[i];
                            pcl_cloud.push_back(point);
                        }

                        pcl::toROSMsg(pcl_cloud, cloud);
                        cloud.header.frame_id = frame_id;
                        cloud.header.stamp = img.header.stamp;
                        pub_pc.publish(cloud);

                        ifm_ros_msg.header.frame_id = frame_id;
                        ifm_ros_msg.header.stamp = img.header.stamp;
                        pub_imf_raw.publish(ifm_ros_msg);

                    }
                }
            }
        }else{
            ROS_ERROR("timeout");
        }

        ros::spinOnce();// Handle ROS events
    }
}
