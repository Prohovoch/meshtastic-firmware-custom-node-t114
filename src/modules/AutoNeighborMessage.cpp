#include "AutoNeighborMessage.h"
#include "MeshService.h"
#include "configuration.h"
#include "mesh/generated/meshtastic/portnums.pb.h"
#include <Arduino.h>

// Global pointer to a autoneighbour
AutoNeighborMessage *autoNeighborMessage;
// Making a position logs.
AutoNeighborMessage::AutoNeighborMessage()
    : SinglePortModule("AutoNeighborMessage", meshtastic_PortNum_POSITION_APP), concurrency::OSThread("AutoNeighborMessage")
{
    // Logs for module creation
    LOG_INFO("AutoNeighborMessage module constructed");
}
// Module begin its job here.
int32_t AutoNeighborMessage::runOnce()
{
    LOG_INFO("AutoNeighborMessage runOnce started");

    //FIXME: potential problem with node. WDYM auto, node btw gets a LITE object, not an object we need
    // so we need to refactor it.
    auto node = nodeDB->getMeshNode(nodeDB->getNodeNum());
    if (!node || !node->has_position || node->position.latitude_i == 0) {
        LOG_WARN("No valid position in nodeDB yet. Skipping send.");
        return 60000; // Попробуем снова через 30 секунд
    }

   
    // Creating a position structure for data. Initializing and making a proto info
    // Potential warning: node is a node LITE object, which is not containing any LDOP or HDOP fields.
    
    // refactor, maybe... or not
    meshtastic_Position currPos = meshtastic_Position_init_default;
    currPos.latitude_i = node->position.latitude_i;
    currPos.longitude_i = node->position.longitude_i;
    currPos.altitude = node->position.altitude;
    currPos.time = node->position.time;
    
    // just for logs tests.
    LOG_INFO("Encoding Position: Lat=%d, Lon=%d, Alt=%d, Time=%u,",
          currPos.latitude_i, 
          currPos.longitude_i, 
          currPos.altitude,
          currPos.time);

    // Making a meshpacket here. its ok.
    meshtastic_MeshPacket *p = allocDataPacket();
    if (!p) {
        LOG_ERROR("allocDataPacket failed");
        return 60000; // повторим через 30 секунд
    }
    // Nanopb proto serialization. For future cycle mb refactor or making a normal method/procedure
    pb_ostream_t stream = pb_ostream_from_buffer(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes));
    if(!pb_encode(&stream, meshtastic_Position_fields, &currPos)){
        LOG_ERROR("Encoding failed!");
        service->releaseToPool(p);
        return 60000;
    }
    // logging for proto, hex format
    LOG_DEBUG("Protobuf bytes (%d):", stream.bytes_written);
    for (size_t i = 0; i < stream.bytes_written; i++) {
        LOG_DEBUG("%02x ", p->decoded.payload.bytes[i]);
    }
    LOG_DEBUG("\n");


    // sending to mesh using protobuf
    p->decoded.payload.size = stream.bytes_written;
    p->decoded.portnum = meshtastic_PortNum_POSITION_APP;
    
 
    service->sendToMesh(p, RX_SRC_LOCAL, false);

    

    
    return 60000;
}